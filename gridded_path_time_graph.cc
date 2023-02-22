/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file gridded_path_time_graph.cc
 **/

#include "modules/planning/tasks/optimizers/path_time_heuristic/gridded_path_time_graph.h"

#include <algorithm>
#include <limits>
#include <string>

#include "modules/common/proto/pnc_point.pb.h"

#include "cyber/common/log.h"
#include "cyber/task/task.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/point_factory.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::SpeedPoint;
using apollo::common::Status;
using apollo::common::util::PointFactory;

namespace {

static constexpr double kDoubleEpsilon = 1.0e-6;

// Continuous-time collision check using linear interpolation as closed-loop
// dynamics
bool CheckOverlapOnDpStGraph(const std::vector<const STBoundary*>& boundaries,
                             const StGraphPoint& p1, const StGraphPoint& p2) {
  if (FLAGS_use_st_drivable_boundary) {
    return false;
  }

  for (const auto* boundary : boundaries) {

    if (boundary->boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    // Check collision between a polygon and a line segment
    if (boundary->HasOverlap({p1.point(), p2.point()})) {
      return true;
    }
    
  }

  return false;
}
}  // namespace

GriddedPathTimeGraph::GriddedPathTimeGraph(
    const StGraphData& st_graph_data, const DpStSpeedOptimizerConfig& dp_config,
    const std::vector<const Obstacle*>& obstacles,
    const common::TrajectoryPoint& init_point)
    : st_graph_data_(st_graph_data),
      gridded_path_time_graph_config_(dp_config),
      obstacles_(obstacles),
      init_point_(init_point),
      dp_st_cost_(dp_config, st_graph_data_.total_time_by_conf(),
                  st_graph_data_.path_length(), obstacles,
                  st_graph_data_.st_drivable_boundary(), init_point_) {
  total_length_t_ = st_graph_data_.total_time_by_conf();
  unit_t_ = gridded_path_time_graph_config_.unit_t();
  total_length_s_ = st_graph_data_.path_length();
  dense_unit_s_ = gridded_path_time_graph_config_.dense_unit_s();
  sparse_unit_s_ = gridded_path_time_graph_config_.sparse_unit_s();
  dense_dimension_s_ = gridded_path_time_graph_config_.dense_dimension_s();
  // Safety approach preventing unreachable acceleration/deceleration
  max_acceleration_ =
      std::min(std::abs(vehicle_param_.max_acceleration()),
               std::abs(gridded_path_time_graph_config_.max_acceleration()));
  max_deceleration_ =
      -1.0 *
      std::min(std::abs(vehicle_param_.max_deceleration()),
               std::abs(gridded_path_time_graph_config_.max_deceleration()));
}

/****************************************************************************
 * 
 * 速度规划的主流程：
 * 1、初始化CostTable，   InitCostTable()
 * 2、初始化限速查询表，   InitSpeedLimitLookUp()
 * 3、计算更新CostTable， CalculateTotalCost()
 * 4、回溯，得到SpeedProfile
 * **************************************************************************/
// 从st图中寻找代价值最小的速度曲线
// s：行驶距离，纵坐标；
// t：行驶时间，横坐标
// SpeedData* const speed_data表示speed_data本身（即指针自身）不能被修改，
// 但speed_data指向的内容可被修改，该函数就是通过它返回最优速度数据的。
Status GriddedPathTimeGraph::Search(SpeedData* const speed_data) {

  static constexpr double kBounadryEpsilon = 1e-2;
  for (const auto& boundary : st_graph_data_.st_boundaries()) {
    // KeepClear obstacles not considered in Dp St decision
    // 若边界类型为禁停区，直接跳过
    if (boundary->boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }
    // If init point in collision with obstacle, return speed fallback
    if (boundary->IsPointInBoundary({0.0, 0.0}) ||
        (std::fabs(boundary->min_t()) < kBounadryEpsilon &&
         std::fabs(boundary->min_s()) < kBounadryEpsilon)) {
      dimension_t_ = static_cast<uint32_t>(std::ceil(
                         total_length_t_ / static_cast<double>(unit_t_))) +
                     1;
      std::vector<SpeedPoint> speed_profile;
      double t = 0.0;
      for (uint32_t i = 0; i < dimension_t_; ++i, t += unit_t_) {
        speed_profile.push_back(PointFactory::ToSpeedPoint(0, t));
      }
      *speed_data = SpeedData(speed_profile);
      return Status::OK();
    }
  }
  
  // 1 初始化CostTable
  if (!InitCostTable().ok()) {
    const std::string msg = "Initialize cost table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 2 初始化限速查询表
  if (!InitSpeedLimitLookUp().ok()) {
    const std::string msg = "Initialize speed limit lookup table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 3 计算所有的cost，并更新CostTable，DP主体
  if (!CalculateTotalCost().ok()) {
    const std::string msg = "Calculate total cost failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 4 回溯得到SpeedProfile
  if (!RetrieveSpeedProfile(speed_data).ok()) {
    const std::string msg = "Retrieve best speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  return Status::OK();
}

/*************************************************************************
*
* cost_table_ 初始化
* 1、只是在t方向和 s方向的离散化
* 2、两层for循环完成初始化
**************************************************************************/

Status GriddedPathTimeGraph::InitCostTable() {
  // Time dimension is homogeneous while Spatial dimension has two resolutions,
  // dense and sparse with dense resolution coming first in the spatial horizon

  // Sanity check for numerical stability
  if (unit_t_ < kDoubleEpsilon) {
    const std::string msg = "unit_t is smaller than the kDoubleEpsilon.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // Sanity check on s dimension setting
  if (dense_dimension_s_ < 1) {
    const std::string msg = "dense_dimension_s is at least 1.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // t方向的离散化，dimension_t_ = total_length_t_（7） 除以 unit_t_（时间分辨率 1s）。
  dimension_t_ = static_cast<uint32_t>(std::ceil(
                     total_length_t_ / static_cast<double>(unit_t_))) + //ceil函数：返回大于或者等于指定表达式的最小整数
                 1;//8
  // static_cast< >数据类型转换

  // s方向的离散化//前20m密后20米后的疏
  double sparse_length_s =
      total_length_s_ -
      static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_;//dense_dimension_s_=41 
      //dense_unit_s_=0.5

  // 稀疏采样点个数
  sparse_dimension_s_ =
      sparse_length_s > std::numeric_limits<double>::epsilon()
          ? static_cast<uint32_t>(std::ceil(sparse_length_s / sparse_unit_s_))
          : 0;//sparse_unit_s_=1

  // 密集采样点个数
  dense_dimension_s_ =
      sparse_length_s > std::numeric_limits<double>::epsilon()
          ? dense_dimension_s_
          : static_cast<uint32_t>(std::ceil(total_length_s_ / dense_unit_s_)) +
                1;

  //总共采样点数
  dimension_s_ = dense_dimension_s_ + sparse_dimension_s_;//稀疏+稠密

  // Sanity Check
  if (dimension_t_ < 1 || dimension_s_ < 1) {
    const std::string msg = "Dp st cost table size incorrect.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  //cost_table_为双层vector，外层是t，内层是s，每个cost_table_[][]都是StGraphPoint 
  cost_table_ = std::vector<std::vector<StGraphPoint>>(
      dimension_t_, std::vector<StGraphPoint>(dimension_s_, StGraphPoint()));

  double curr_t = 0.0;

  //外层是时间t，内层是s
  for (uint32_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_) {
    auto& cost_table_i = cost_table_[i];
    double curr_s = 0.0;

    //先对 dense 初始化
    for (uint32_t j = 0; j < dense_dimension_s_; ++j, curr_s += dense_unit_s_) {
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
    }

    //dense区从20m后
    curr_s = static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_ +
             sparse_unit_s_;

    //在对sparse初始化
    for (uint32_t j = dense_dimension_s_; j < cost_table_i.size();
         ++j, curr_s += sparse_unit_s_) {
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
    }
  }
  // 这是干什么用的？
  const auto& cost_table_0 = cost_table_[0];
  spatial_distance_by_index_ = std::vector<double>(cost_table_0.size(), 0.0);
  for (uint32_t i = 0; i < cost_table_0.size(); ++i) {
    spatial_distance_by_index_[i] = cost_table_0[i].point().s();
  }
  return Status::OK();
}

/*************************************************************************
*
* 初始化限速信息
* 这个限速来自于参考线上的限速，可能是地图上车道线限速或者其他的什么在固定位置的限速
**************************************************************************/
Status GriddedPathTimeGraph::InitSpeedLimitLookUp() {

  speed_limit_by_index_.clear();

  speed_limit_by_index_.resize(dimension_s_);//dimension_s_ 总采样点数
  const auto& speed_limit = st_graph_data_.speed_limit();//

  //得到每个采样S处限速
  for (uint32_t i = 0; i < dimension_s_; ++i) {
    speed_limit_by_index_[i] =
        speed_limit.GetSpeedLimitByS(cost_table_[0][i].point().s());
  }
  return Status::OK();
}

/*******************************************************************************
 * 
 * 动态规划的主体，两层for循环遍历cost_table_，t外循环，s内循环
 * 向前递推子节点
 * 调用CalculateCostAt()计算total_cost
 * ****************************************************************************/

Status GriddedPathTimeGraph::CalculateTotalCost() {
  // col and row are for STGraph col是列 row是行
  // t corresponding to col 列
  // s corresponding to row 行

  // 需要通过函数GetRowRange()计算出s的范围[next_lowest_row,next_highest_row]
  // 剪枝操作，避免不必要的计算
  size_t next_highest_row = 0;
  size_t next_lowest_row = 0;

  // 外循环，每一列的index，即每一个t
  for (size_t c = 0; c < cost_table_.size(); ++c) {
    size_t highest_row = 0;
    size_t lowest_row = cost_table_.back().size() - 1;
    //count 为每一列的点数
    int count = static_cast<int>(next_highest_row) -
                static_cast<int>(next_lowest_row) + 1;
    if (count > 0) {
      std::vector<std::future<void>> results;//future 提供访问异步操作结果的机制，很轻松解决从异步任务中返回结果。
      // 内循环，每行的index，即每一个s,每一行
      for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
        auto msg = std::make_shared<StGraphMessage>(c, r);
        if (FLAGS_enable_multi_thread_in_dp_st_graph) {
          results.push_back(
              cyber::Async(&GriddedPathTimeGraph::CalculateCostAt, this, msg));
        } else {

          //每个点计算代价值
          CalculateCostAt(msg);
        }
      }

      if (FLAGS_enable_multi_thread_in_dp_st_graph) {
        for (auto& result : results) {
          result.get();
        }
      }
    }

    // 下一轮循环的准备工作，更新范围 [next_lowest_row, next_highest_row]
    for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
      const auto& cost_cr = cost_table_[c][r];
      if (cost_cr.total_cost() < std::numeric_limits<double>::infinity()) {
        size_t h_r = 0;
        size_t l_r = 0;
        GetRowRange(cost_cr, &h_r, &l_r);//剪枝操作
        highest_row = std::max(highest_row, h_r);
        lowest_row = std::min(lowest_row, l_r);
      }
    }

    next_highest_row = highest_row;
    next_lowest_row = lowest_row;
  }

  return Status::OK();
}

/*******************************************************************************
 * 
 * 剪枝操作，实现细节见 GetRowRange( )，依据max_acceleration_ 和 max_deceleration_ 
 * 算出下一时刻可以到达的S，进而得到范围[next_lowest_row, next_lowest_row]
 * 
 * ****************************************************************************/

void GriddedPathTimeGraph::GetRowRange(const StGraphPoint& point,
                                       size_t* next_highest_row,
                                       size_t* next_lowest_row) {
  double v0 = 0.0;
  // TODO(all): Record speed information in StGraphPoint and deprecate this.
  // A scaling parameter for DP range search due to the lack of accurate
  // information of the current velocity (set to 1 by default since we use
  // past 1 second's average v as approximation)
  double acc_coeff = 0.5;
  if (!point.pre_point()) {
    v0 = init_point_.v();
  } else {
    v0 = point.GetOptimalSpeed();
  }

  const auto max_s_size = dimension_s_ - 1;
  const double t_squared = unit_t_ * unit_t_;
  //确定上限
  const double s_upper_bound = v0 * unit_t_ +
                               acc_coeff * max_acceleration_ * t_squared +
                               point.point().s();//运动学方程s=v*t+1/2*a*t^2
  const auto next_highest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), s_upper_bound);
  if (next_highest_itr == spatial_distance_by_index_.end()) {
    *next_highest_row = max_s_size;
  } else {
    *next_highest_row =
        std::distance(spatial_distance_by_index_.begin(), next_highest_itr);
  }
  //确定下限
  const double s_lower_bound =
      std::fmax(0.0, v0 * unit_t_ + acc_coeff * max_deceleration_ * t_squared) +
      point.point().s();
  const auto next_lowest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), s_lower_bound);
  if (next_lowest_itr == spatial_distance_by_index_.end()) {
    *next_lowest_row = max_s_size;
  } else {
    *next_lowest_row =
        std::distance(spatial_distance_by_index_.begin(), next_lowest_itr);
  }
}

/*****************************************************************************
 * 
 * 计算每一个点的totalCost，并更新父节点
 * 输入 msg，即(c,r)，cost_table_中t、s方向离散后的index
 * 
 * *************************************************************************/

void GriddedPathTimeGraph::CalculateCostAt(
    const std::shared_ptr<StGraphMessage>& msg) {
  const uint32_t c = msg->c;
  const uint32_t r = msg->r;
  auto& cost_cr = cost_table_[c][r];

  // 1、计算 obstacle_cost，如果为无穷大，则停止，
  cost_cr.SetObstacleCost(dp_st_cost_.GetObstacleCost(cost_cr));
  //太大就不计算了舍弃
  if (cost_cr.obstacle_cost() > std::numeric_limits<double>::max()) {
    return;
  }
  
  // 2、计算SpatialPotentialCost
  cost_cr.SetSpatialPotentialCost(dp_st_cost_.GetSpatialPotentialCost(cost_cr));

  const auto& cost_init = cost_table_[0][0];//初始点

  // 0列的特殊处理，设置起始点的TotalCost 为0。
  if (c == 0) {
    DCHECK_EQ(r, 0U) << "Incorrect. Row should be 0 with col = 0. row: " << r;
    cost_cr.SetTotalCost(0.0); // 起点的cost设置为0
    cost_cr.SetOptimalSpeed(init_point_.v());
    return;
  }

  const double speed_limit = speed_limit_by_index_[r];
  const double cruise_speed = st_graph_data_.cruise_speed();
  // The mininal s to model as constant acceleration formula
  // default: 0.25 * 7 = 1.75 m
  const double min_s_consider_speed = dense_unit_s_ * dimension_t_;

  //第一列处理
  if (c == 1) {

    //当前点的加速度 根据s=v*t+1/2*a*t^2;
    const double acc =
        2 * (cost_cr.point().s() / unit_t_ - init_point_.v()) / unit_t_;

    // 加速度、减速度超出范围，返回
    if (acc < max_deceleration_ || acc > max_acceleration_) {
      return;
    }

    // 剔除倒车并确保第一个点不小于1.75
    if (init_point_.v() + acc * unit_t_ < -kDoubleEpsilon &&
        cost_cr.point().s() > min_s_consider_speed) {
      return;
    }

    // 当前点与起始点的连线与stboundary有重叠，返回
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                cost_init)) {
      return;
    }

    // 计算当前点的total_cost
    cost_cr.SetTotalCost(
        cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
        cost_init.total_cost() +
        CalculateEdgeCostForSecondCol(r, speed_limit, cruise_speed));

    cost_cr.SetPrePoint(cost_init);//前一个点为初始点

    //设置最优速度
    cost_cr.SetOptimalSpeed(init_point_.v() + acc * unit_t_);

    return;
  }
  
  
  static constexpr double kSpeedRangeBuffer = 0.20;
  // 根据速度剪枝操作
  // 由当前点推出能到达该点的前一列最小的s
  // 将当前点的pre_col缩小至 [r_low, r]
  const double pre_lowest_s =
      cost_cr.point().s() -
      FLAGS_planning_upper_speed_limit * (1 + kSpeedRangeBuffer) * unit_t_;
  
  ////函数用于在指定区域内查找不小于目标值的第一个元素的标号
  const auto pre_lowest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), pre_lowest_s);
  uint32_t r_low = 0;

  // 计算 pre_lowest_s 的 index
  if (pre_lowest_itr == spatial_distance_by_index_.end()) {
    r_low = dimension_s_ - 1;
  } else {
    r_low = static_cast<uint32_t>(
        std::distance(spatial_distance_by_index_.begin(), pre_lowest_itr));
  }
  
  // 父节点有几个
  const uint32_t r_pre_size = r - r_low + 1;

  //上一列节点，父节点
  const auto& pre_col = cost_table_[c - 1];

  double curr_speed_limit = speed_limit;


  // 第二列的特殊处理
  if (c == 2) {

    // 对于前一列，遍历从r->r_low的点， 
    // 依据重新算得的cost，当前点的pre_point，也就是DP过程的状态转移方程
    for (uint32_t i = 0; i < r_pre_size; ++i) {
      
      // 父节点index
      uint32_t r_pre = r - i;

      //剔除代价函数无穷大和剪枝操作的点
      if (std::isinf(pre_col[r_pre].total_cost()) ||
          pre_col[r_pre].pre_point() == nullptr) {
        continue;
      }

      // TODO(Jiaxuan): Calculate accurate acceleration by recording speed
      // data in ST point.
      // Use curr_v = (point.s - pre_point.s) / unit_t as current v
      // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
      // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
      // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)
      const double curr_a =
          2 *
          ((cost_cr.point().s() - pre_col[r_pre].point().s()) / unit_t_ -
           pre_col[r_pre].GetOptimalSpeed()) /
          unit_t_;//当前点的加速度
      
      //剔除加速度过大的点
      if (curr_a < max_deceleration_ || curr_a > max_acceleration_) {
        continue;
      }

      // 剔除倒车点
      if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ <
              -kDoubleEpsilon &&
          cost_cr.point().s() > min_s_consider_speed) {
        continue;
      }

      // 当前点与前一个点的连线是不是在boundary里
      // Filter out continuous-time node connection which is in collision with
      // obstacle
      if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                  pre_col[r_pre])) {
        continue;
      }
      
      curr_speed_limit =
          std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);

      //这个cost有点疑问
      const double cost = cost_cr.obstacle_cost() +
                          cost_cr.spatial_potential_cost() +
                          pre_col[r_pre].total_cost() +
                          CalculateEdgeCostForThirdCol(
                              r, r_pre, curr_speed_limit, cruise_speed);

      if (cost < cost_cr.total_cost()) {
        cost_cr.SetTotalCost(cost);
        cost_cr.SetPrePoint(pre_col[r_pre]);
        cost_cr.SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() +
                                curr_a * unit_t_);
      }
    }
    return;
  }

  // 第3列，及以后列的处理
  // 依据重新算得的cost，当前点的pre_point，也就是DP过程的状态转移方程
  for (uint32_t i = 0; i < r_pre_size; ++i) {

    uint32_t r_pre = r - i;

    //剔除代价函数无穷大和剪枝操作的点
    if (std::isinf(pre_col[r_pre].total_cost()) ||
        pre_col[r_pre].pre_point() == nullptr) {
      continue;
    }
    // Use curr_v = (point.s - pre_point.s) / unit_t as current v
    // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
    // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
    // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)
    const double curr_a =
        2 *
        ((cost_cr.point().s() - pre_col[r_pre].point().s()) / unit_t_ -
         pre_col[r_pre].GetOptimalSpeed()) /
        unit_t_;//当前点加速度

    //剔除加速度过大的点
    if (curr_a > max_acceleration_ || curr_a < max_deceleration_) {
      continue;
    }

    // 剔除倒车点
    if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ < -kDoubleEpsilon &&
        cost_cr.point().s() > min_s_consider_speed) {
      continue;
    }

    // 与上一列点连接是否发生碰撞
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                pre_col[r_pre])) {
      continue;
    }

    uint32_t r_prepre = pre_col[r_pre].pre_point()->index_s();

    // 前两列的节点，父节点的父节点
    const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];

    // 提出父节点无穷大的节点
    if (std::isinf(prepre_graph_point.total_cost())) {
      continue;
    }

    // 剔除无父节点的子节点
    if (!prepre_graph_point.pre_point()) {
      continue;
    }

    const STPoint& triple_pre_point = prepre_graph_point.pre_point()->point();//上上上个点
    const STPoint& prepre_point = prepre_graph_point.point();//上上个点
    const STPoint& pre_point = pre_col[r_pre].point();//上个点
    const STPoint& curr_point = cost_cr.point();//当前点

    // 获得当前节点速度最大值
    curr_speed_limit =
        std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);

    //之所以跟前边不一样是要算ACC JERK
    double cost = cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
                  pre_col[r_pre].total_cost() +
                  CalculateEdgeCost(triple_pre_point, prepre_point, pre_point,
                                    curr_point, curr_speed_limit, cruise_speed);

    // 寻找最小代价函数父节点并更新
    if (cost < cost_cr.total_cost()) {
      cost_cr.SetTotalCost(cost);
      cost_cr.SetPrePoint(pre_col[r_pre]);
      cost_cr.SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() +
                              curr_a * unit_t_);
    }
  }
}

/************************************************************************************
 * 
 * 回溯，得到最优 speed_data
 * 
 * **********************************************************************************/

Status GriddedPathTimeGraph::RetrieveSpeedProfile(SpeedData* const speed_data) {

  double min_cost = std::numeric_limits<double>::infinity();
  const StGraphPoint* best_end_point = nullptr;

  //从 cost_table_ 最后一列找 min_cost
  for (const StGraphPoint& cur_point : cost_table_.back()) {
    // 遍历最后一列找最值
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }
  
  // 遍历每一列的最后一个点，找正在的best_end_point，并更新min_cost
  // 这里不直接使用最后一列的min_cost点作为终点
  // 因为采样时间是一个预估时间窗口，在这之前可能就到达终点了
  for (const auto& row : cost_table_) {

    const StGraphPoint& cur_point = row.back();

    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }

  }

  // 确保最优节点非空
  if (best_end_point == nullptr) {
    const std::string msg = "Fail to find the best feasible trajectory.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  
  std::vector<SpeedPoint> speed_profile;

  // 最优节点
  const StGraphPoint* cur_point = best_end_point;

  // 回溯，得到最优的 speed_profile
  while (cur_point != nullptr) {
    ADEBUG << "Time: " << cur_point->point().t();
    ADEBUG << "S: " << cur_point->point().s();
    ADEBUG << "V: " << cur_point->GetOptimalSpeed();
    SpeedPoint speed_point;
    speed_point.set_s(cur_point->point().s());
    speed_point.set_t(cur_point->point().t());
    speed_profile.push_back(speed_point);
    cur_point = cur_point->pre_point();
  }

  //由于是逆向寻优因此要翻转回来
  std::reverse(speed_profile.begin(), speed_profile.end());

  static constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

  if (speed_profile.front().t() > kEpsilon ||
      speed_profile.front().s() > kEpsilon) {
    const std::string msg = "Fail to retrieve speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 计算每个点的速度 v
  for (size_t i = 0; i + 1 < speed_profile.size(); ++i) {
    const double v = (speed_profile[i + 1].s() - speed_profile[i].s()) /
                     (speed_profile[i + 1].t() - speed_profile[i].t() + 1e-3);
    speed_profile[i].set_v(v);
  }

  *speed_data = SpeedData(speed_profile);

  return Status::OK();
}

// 这个通过s计算加速度所以要4个点
double GriddedPathTimeGraph::CalculateEdgeCost(
    const STPoint& first, const STPoint& second, const STPoint& third,
    const STPoint& forth, const double speed_limit, const double cruise_speed) {
  return dp_st_cost_.GetSpeedCost(third, forth, speed_limit, cruise_speed) +
         dp_st_cost_.GetAccelCostByThreePoints(second, third, forth) +
         dp_st_cost_.GetJerkCostByFourPoints(first, second, third, forth);
}

// 第二列 Edge代价计算
double GriddedPathTimeGraph::CalculateEdgeCostForSecondCol(
    const uint32_t row, const double speed_limit, const double cruise_speed) {
  double init_speed = init_point_.v();
  double init_acc = init_point_.a();
  const STPoint& pre_point = cost_table_[0][0].point();
  const STPoint& curr_point = cost_table_[1][row].point();

  return dp_st_cost_.GetSpeedCost(pre_point, curr_point, speed_limit,
                                  cruise_speed) +
         dp_st_cost_.GetAccelCostByTwoPoints(init_speed, pre_point,
                                             curr_point) +
         dp_st_cost_.GetJerkCostByTwoPoints(init_speed, init_acc, pre_point,
                                            curr_point);
}

// 第三列 Edge 及以后代价计算
double GriddedPathTimeGraph::CalculateEdgeCostForThirdCol(
    const uint32_t curr_row, const uint32_t pre_row, const double speed_limit,
    const double cruise_speed) {
  double init_speed = init_point_.v();
  const STPoint& first = cost_table_[0][0].point();
  const STPoint& second = cost_table_[1][pre_row].point();
  const STPoint& third = cost_table_[2][curr_row].point();
  return dp_st_cost_.GetSpeedCost(second, third, speed_limit, cruise_speed) +
         dp_st_cost_.GetAccelCostByThreePoints(first, second, third) +
         dp_st_cost_.GetJerkCostByThreePoints(init_speed, first, second, third);
}

}  // namespace planning
}  // namespace apollo
