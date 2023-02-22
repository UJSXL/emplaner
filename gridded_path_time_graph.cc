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
 * �ٶȹ滮�������̣�
 * 1����ʼ��CostTable��   InitCostTable()
 * 2����ʼ�����ٲ�ѯ��   InitSpeedLimitLookUp()
 * 3���������CostTable�� CalculateTotalCost()
 * 4�����ݣ��õ�SpeedProfile
 * **************************************************************************/
// ��stͼ��Ѱ�Ҵ���ֵ��С���ٶ�����
// s����ʻ���룬�����ꣻ
// t����ʻʱ�䣬������
// SpeedData* const speed_data��ʾspeed_data������ָ���������ܱ��޸ģ�
// ��speed_dataָ������ݿɱ��޸ģ��ú�������ͨ�������������ٶ����ݵġ�
Status GriddedPathTimeGraph::Search(SpeedData* const speed_data) {

  static constexpr double kBounadryEpsilon = 1e-2;
  for (const auto& boundary : st_graph_data_.st_boundaries()) {
    // KeepClear obstacles not considered in Dp St decision
    // ���߽�����Ϊ��ͣ����ֱ������
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
  
  // 1 ��ʼ��CostTable
  if (!InitCostTable().ok()) {
    const std::string msg = "Initialize cost table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 2 ��ʼ�����ٲ�ѯ��
  if (!InitSpeedLimitLookUp().ok()) {
    const std::string msg = "Initialize speed limit lookup table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 3 �������е�cost��������CostTable��DP����
  if (!CalculateTotalCost().ok()) {
    const std::string msg = "Calculate total cost failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 4 ���ݵõ�SpeedProfile
  if (!RetrieveSpeedProfile(speed_data).ok()) {
    const std::string msg = "Retrieve best speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  return Status::OK();
}

/*************************************************************************
*
* cost_table_ ��ʼ��
* 1��ֻ����t����� s�������ɢ��
* 2������forѭ����ɳ�ʼ��
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

  // t�������ɢ����dimension_t_ = total_length_t_��7�� ���� unit_t_��ʱ��ֱ��� 1s����
  dimension_t_ = static_cast<uint32_t>(std::ceil(
                     total_length_t_ / static_cast<double>(unit_t_))) + //ceil���������ش��ڻ��ߵ���ָ�����ʽ����С����
                 1;//8
  // static_cast< >��������ת��

  // s�������ɢ��//ǰ20m�ܺ�20�׺����
  double sparse_length_s =
      total_length_s_ -
      static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_;//dense_dimension_s_=41 
      //dense_unit_s_=0.5

  // ϡ����������
  sparse_dimension_s_ =
      sparse_length_s > std::numeric_limits<double>::epsilon()
          ? static_cast<uint32_t>(std::ceil(sparse_length_s / sparse_unit_s_))
          : 0;//sparse_unit_s_=1

  // �ܼ����������
  dense_dimension_s_ =
      sparse_length_s > std::numeric_limits<double>::epsilon()
          ? dense_dimension_s_
          : static_cast<uint32_t>(std::ceil(total_length_s_ / dense_unit_s_)) +
                1;

  //�ܹ���������
  dimension_s_ = dense_dimension_s_ + sparse_dimension_s_;//ϡ��+����

  // Sanity Check
  if (dimension_t_ < 1 || dimension_s_ < 1) {
    const std::string msg = "Dp st cost table size incorrect.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  //cost_table_Ϊ˫��vector�������t���ڲ���s��ÿ��cost_table_[][]����StGraphPoint 
  cost_table_ = std::vector<std::vector<StGraphPoint>>(
      dimension_t_, std::vector<StGraphPoint>(dimension_s_, StGraphPoint()));

  double curr_t = 0.0;

  //�����ʱ��t���ڲ���s
  for (uint32_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_) {
    auto& cost_table_i = cost_table_[i];
    double curr_s = 0.0;

    //�ȶ� dense ��ʼ��
    for (uint32_t j = 0; j < dense_dimension_s_; ++j, curr_s += dense_unit_s_) {
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
    }

    //dense����20m��
    curr_s = static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_ +
             sparse_unit_s_;

    //�ڶ�sparse��ʼ��
    for (uint32_t j = dense_dimension_s_; j < cost_table_i.size();
         ++j, curr_s += sparse_unit_s_) {
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
    }
  }
  // ���Ǹ�ʲô�õģ�
  const auto& cost_table_0 = cost_table_[0];
  spatial_distance_by_index_ = std::vector<double>(cost_table_0.size(), 0.0);
  for (uint32_t i = 0; i < cost_table_0.size(); ++i) {
    spatial_distance_by_index_[i] = cost_table_0[i].point().s();
  }
  return Status::OK();
}

/*************************************************************************
*
* ��ʼ��������Ϣ
* ������������ڲο����ϵ����٣������ǵ�ͼ�ϳ��������ٻ���������ʲô�ڹ̶�λ�õ�����
**************************************************************************/
Status GriddedPathTimeGraph::InitSpeedLimitLookUp() {

  speed_limit_by_index_.clear();

  speed_limit_by_index_.resize(dimension_s_);//dimension_s_ �ܲ�������
  const auto& speed_limit = st_graph_data_.speed_limit();//

  //�õ�ÿ������S������
  for (uint32_t i = 0; i < dimension_s_; ++i) {
    speed_limit_by_index_[i] =
        speed_limit.GetSpeedLimitByS(cost_table_[0][i].point().s());
  }
  return Status::OK();
}

/*******************************************************************************
 * 
 * ��̬�滮�����壬����forѭ������cost_table_��t��ѭ����s��ѭ��
 * ��ǰ�����ӽڵ�
 * ����CalculateCostAt()����total_cost
 * ****************************************************************************/

Status GriddedPathTimeGraph::CalculateTotalCost() {
  // col and row are for STGraph col���� row����
  // t corresponding to col ��
  // s corresponding to row ��

  // ��Ҫͨ������GetRowRange()�����s�ķ�Χ[next_lowest_row,next_highest_row]
  // ��֦���������ⲻ��Ҫ�ļ���
  size_t next_highest_row = 0;
  size_t next_lowest_row = 0;

  // ��ѭ����ÿһ�е�index����ÿһ��t
  for (size_t c = 0; c < cost_table_.size(); ++c) {
    size_t highest_row = 0;
    size_t lowest_row = cost_table_.back().size() - 1;
    //count Ϊÿһ�еĵ���
    int count = static_cast<int>(next_highest_row) -
                static_cast<int>(next_lowest_row) + 1;
    if (count > 0) {
      std::vector<std::future<void>> results;//future �ṩ�����첽��������Ļ��ƣ������ɽ�����첽�����з��ؽ����
      // ��ѭ����ÿ�е�index����ÿһ��s,ÿһ��
      for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
        auto msg = std::make_shared<StGraphMessage>(c, r);
        if (FLAGS_enable_multi_thread_in_dp_st_graph) {
          results.push_back(
              cyber::Async(&GriddedPathTimeGraph::CalculateCostAt, this, msg));
        } else {

          //ÿ����������ֵ
          CalculateCostAt(msg);
        }
      }

      if (FLAGS_enable_multi_thread_in_dp_st_graph) {
        for (auto& result : results) {
          result.get();
        }
      }
    }

    // ��һ��ѭ����׼�����������·�Χ [next_lowest_row, next_highest_row]
    for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
      const auto& cost_cr = cost_table_[c][r];
      if (cost_cr.total_cost() < std::numeric_limits<double>::infinity()) {
        size_t h_r = 0;
        size_t l_r = 0;
        GetRowRange(cost_cr, &h_r, &l_r);//��֦����
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
 * ��֦������ʵ��ϸ�ڼ� GetRowRange( )������max_acceleration_ �� max_deceleration_ 
 * �����һʱ�̿��Ե����S�������õ���Χ[next_lowest_row, next_lowest_row]
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
  //ȷ������
  const double s_upper_bound = v0 * unit_t_ +
                               acc_coeff * max_acceleration_ * t_squared +
                               point.point().s();//�˶�ѧ����s=v*t+1/2*a*t^2
  const auto next_highest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), s_upper_bound);
  if (next_highest_itr == spatial_distance_by_index_.end()) {
    *next_highest_row = max_s_size;
  } else {
    *next_highest_row =
        std::distance(spatial_distance_by_index_.begin(), next_highest_itr);
  }
  //ȷ������
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
 * ����ÿһ�����totalCost�������¸��ڵ�
 * ���� msg����(c,r)��cost_table_��t��s������ɢ���index
 * 
 * *************************************************************************/

void GriddedPathTimeGraph::CalculateCostAt(
    const std::shared_ptr<StGraphMessage>& msg) {
  const uint32_t c = msg->c;
  const uint32_t r = msg->r;
  auto& cost_cr = cost_table_[c][r];

  // 1������ obstacle_cost�����Ϊ�������ֹͣ��
  cost_cr.SetObstacleCost(dp_st_cost_.GetObstacleCost(cost_cr));
  //̫��Ͳ�����������
  if (cost_cr.obstacle_cost() > std::numeric_limits<double>::max()) {
    return;
  }
  
  // 2������SpatialPotentialCost
  cost_cr.SetSpatialPotentialCost(dp_st_cost_.GetSpatialPotentialCost(cost_cr));

  const auto& cost_init = cost_table_[0][0];//��ʼ��

  // 0�е����⴦��������ʼ���TotalCost Ϊ0��
  if (c == 0) {
    DCHECK_EQ(r, 0U) << "Incorrect. Row should be 0 with col = 0. row: " << r;
    cost_cr.SetTotalCost(0.0); // ����cost����Ϊ0
    cost_cr.SetOptimalSpeed(init_point_.v());
    return;
  }

  const double speed_limit = speed_limit_by_index_[r];
  const double cruise_speed = st_graph_data_.cruise_speed();
  // The mininal s to model as constant acceleration formula
  // default: 0.25 * 7 = 1.75 m
  const double min_s_consider_speed = dense_unit_s_ * dimension_t_;

  //��һ�д���
  if (c == 1) {

    //��ǰ��ļ��ٶ� ����s=v*t+1/2*a*t^2;
    const double acc =
        2 * (cost_cr.point().s() / unit_t_ - init_point_.v()) / unit_t_;

    // ���ٶȡ����ٶȳ�����Χ������
    if (acc < max_deceleration_ || acc > max_acceleration_) {
      return;
    }

    // �޳�������ȷ����һ���㲻С��1.75
    if (init_point_.v() + acc * unit_t_ < -kDoubleEpsilon &&
        cost_cr.point().s() > min_s_consider_speed) {
      return;
    }

    // ��ǰ������ʼ���������stboundary���ص�������
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                cost_init)) {
      return;
    }

    // ���㵱ǰ���total_cost
    cost_cr.SetTotalCost(
        cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
        cost_init.total_cost() +
        CalculateEdgeCostForSecondCol(r, speed_limit, cruise_speed));

    cost_cr.SetPrePoint(cost_init);//ǰһ����Ϊ��ʼ��

    //���������ٶ�
    cost_cr.SetOptimalSpeed(init_point_.v() + acc * unit_t_);

    return;
  }
  
  
  static constexpr double kSpeedRangeBuffer = 0.20;
  // �����ٶȼ�֦����
  // �ɵ�ǰ���Ƴ��ܵ���õ��ǰһ����С��s
  // ����ǰ���pre_col��С�� [r_low, r]
  const double pre_lowest_s =
      cost_cr.point().s() -
      FLAGS_planning_upper_speed_limit * (1 + kSpeedRangeBuffer) * unit_t_;
  
  ////����������ָ�������ڲ��Ҳ�С��Ŀ��ֵ�ĵ�һ��Ԫ�صı��
  const auto pre_lowest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), pre_lowest_s);
  uint32_t r_low = 0;

  // ���� pre_lowest_s �� index
  if (pre_lowest_itr == spatial_distance_by_index_.end()) {
    r_low = dimension_s_ - 1;
  } else {
    r_low = static_cast<uint32_t>(
        std::distance(spatial_distance_by_index_.begin(), pre_lowest_itr));
  }
  
  // ���ڵ��м���
  const uint32_t r_pre_size = r - r_low + 1;

  //��һ�нڵ㣬���ڵ�
  const auto& pre_col = cost_table_[c - 1];

  double curr_speed_limit = speed_limit;


  // �ڶ��е����⴦��
  if (c == 2) {

    // ����ǰһ�У�������r->r_low�ĵ㣬 
    // ����������õ�cost����ǰ���pre_point��Ҳ����DP���̵�״̬ת�Ʒ���
    for (uint32_t i = 0; i < r_pre_size; ++i) {
      
      // ���ڵ�index
      uint32_t r_pre = r - i;

      //�޳����ۺ��������ͼ�֦�����ĵ�
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
          unit_t_;//��ǰ��ļ��ٶ�
      
      //�޳����ٶȹ���ĵ�
      if (curr_a < max_deceleration_ || curr_a > max_acceleration_) {
        continue;
      }

      // �޳�������
      if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ <
              -kDoubleEpsilon &&
          cost_cr.point().s() > min_s_consider_speed) {
        continue;
      }

      // ��ǰ����ǰһ����������ǲ�����boundary��
      // Filter out continuous-time node connection which is in collision with
      // obstacle
      if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                  pre_col[r_pre])) {
        continue;
      }
      
      curr_speed_limit =
          std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);

      //���cost�е�����
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

  // ��3�У����Ժ��еĴ���
  // ����������õ�cost����ǰ���pre_point��Ҳ����DP���̵�״̬ת�Ʒ���
  for (uint32_t i = 0; i < r_pre_size; ++i) {

    uint32_t r_pre = r - i;

    //�޳����ۺ��������ͼ�֦�����ĵ�
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
        unit_t_;//��ǰ����ٶ�

    //�޳����ٶȹ���ĵ�
    if (curr_a > max_acceleration_ || curr_a < max_deceleration_) {
      continue;
    }

    // �޳�������
    if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ < -kDoubleEpsilon &&
        cost_cr.point().s() > min_s_consider_speed) {
      continue;
    }

    // ����һ�е������Ƿ�����ײ
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                pre_col[r_pre])) {
      continue;
    }

    uint32_t r_prepre = pre_col[r_pre].pre_point()->index_s();

    // ǰ���еĽڵ㣬���ڵ�ĸ��ڵ�
    const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];

    // ������ڵ������Ľڵ�
    if (std::isinf(prepre_graph_point.total_cost())) {
      continue;
    }

    // �޳��޸��ڵ���ӽڵ�
    if (!prepre_graph_point.pre_point()) {
      continue;
    }

    const STPoint& triple_pre_point = prepre_graph_point.pre_point()->point();//�����ϸ���
    const STPoint& prepre_point = prepre_graph_point.point();//���ϸ���
    const STPoint& pre_point = pre_col[r_pre].point();//�ϸ���
    const STPoint& curr_point = cost_cr.point();//��ǰ��

    // ��õ�ǰ�ڵ��ٶ����ֵ
    curr_speed_limit =
        std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);

    //֮���Ը�ǰ�߲�һ����Ҫ��ACC JERK
    double cost = cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
                  pre_col[r_pre].total_cost() +
                  CalculateEdgeCost(triple_pre_point, prepre_point, pre_point,
                                    curr_point, curr_speed_limit, cruise_speed);

    // Ѱ����С���ۺ������ڵ㲢����
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
 * ���ݣ��õ����� speed_data
 * 
 * **********************************************************************************/

Status GriddedPathTimeGraph::RetrieveSpeedProfile(SpeedData* const speed_data) {

  double min_cost = std::numeric_limits<double>::infinity();
  const StGraphPoint* best_end_point = nullptr;

  //�� cost_table_ ���һ���� min_cost
  for (const StGraphPoint& cur_point : cost_table_.back()) {
    // �������һ������ֵ
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }
  
  // ����ÿһ�е����һ���㣬�����ڵ�best_end_point��������min_cost
  // ���ﲻֱ��ʹ�����һ�е�min_cost����Ϊ�յ�
  // ��Ϊ����ʱ����һ��Ԥ��ʱ�䴰�ڣ�����֮ǰ���ܾ͵����յ���
  for (const auto& row : cost_table_) {

    const StGraphPoint& cur_point = row.back();

    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }

  }

  // ȷ�����Žڵ�ǿ�
  if (best_end_point == nullptr) {
    const std::string msg = "Fail to find the best feasible trajectory.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  
  std::vector<SpeedPoint> speed_profile;

  // ���Žڵ�
  const StGraphPoint* cur_point = best_end_point;

  // ���ݣ��õ����ŵ� speed_profile
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

  //����������Ѱ�����Ҫ��ת����
  std::reverse(speed_profile.begin(), speed_profile.end());

  static constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

  if (speed_profile.front().t() > kEpsilon ||
      speed_profile.front().s() > kEpsilon) {
    const std::string msg = "Fail to retrieve speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // ����ÿ������ٶ� v
  for (size_t i = 0; i + 1 < speed_profile.size(); ++i) {
    const double v = (speed_profile[i + 1].s() - speed_profile[i].s()) /
                     (speed_profile[i + 1].t() - speed_profile[i].t() + 1e-3);
    speed_profile[i].set_v(v);
  }

  *speed_data = SpeedData(speed_profile);

  return Status::OK();
}

// ���ͨ��s������ٶ�����Ҫ4����
double GriddedPathTimeGraph::CalculateEdgeCost(
    const STPoint& first, const STPoint& second, const STPoint& third,
    const STPoint& forth, const double speed_limit, const double cruise_speed) {
  return dp_st_cost_.GetSpeedCost(third, forth, speed_limit, cruise_speed) +
         dp_st_cost_.GetAccelCostByThreePoints(second, third, forth) +
         dp_st_cost_.GetJerkCostByFourPoints(first, second, third, forth);
}

// �ڶ��� Edge���ۼ���
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

// ������ Edge ���Ժ���ۼ���
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
