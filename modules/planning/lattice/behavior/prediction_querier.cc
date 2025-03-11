/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @file
 **/

#include "modules/planning/lattice/behavior/prediction_querier.h"

#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/path_matcher.h"

namespace apollo
{
namespace planning
{
PredictionQuerier::PredictionQuerier(
        const std::vector<const Obstacle*>& obstacles,
        const std::shared_ptr<std::vector<common::PathPoint>>&
                ptr_reference_line) :
    ptr_reference_line_(ptr_reference_line)
{
    for (const auto ptr_obstacle : obstacles)
    {
        if (common::util::InsertIfNotPresent(&id_obstacle_map_,
                                             ptr_obstacle->Id(), ptr_obstacle))
        {
            obstacles_.push_back(ptr_obstacle);
        }
        else
        {
            AWARN << "Duplicated obstacle found [" << ptr_obstacle->Id() << "]";
        }
    }
}

std::vector<const Obstacle*> PredictionQuerier::GetObstacles() const
{
    return obstacles_;
}

//! 用于计算沿参考线（reference line）的速度投影
double PredictionQuerier::ProjectVelocityAlongReferenceLine(
        const std::string& obstacle_id, const double s, const double t) const
{
    //使用ACHECK宏来确保id_obstacle_map_中存在给定的obstacle_id
    ACHECK(id_obstacle_map_.find(obstacle_id) != id_obstacle_map_.end());

    const auto& trajectory = id_obstacle_map_.at(obstacle_id)->Trajectory();
    int num_traj_point = trajectory.trajectory_point_size();
    if (num_traj_point < 2) //如果轨迹点少于2个，无法计算速度，直接返回0.0
    {
        return 0.0;
    }

    //检查时间点是否在轨迹范围内
    if (t < trajectory.trajectory_point(0).relative_time() ||
        t > trajectory.trajectory_point(num_traj_point - 1).relative_time())
    {
        return 0.0;
    }

    //找到匹配的时间点
    //* 使用std::lower_bound找到第一个不小于时间点 t的轨迹点
    auto matched_it = std::lower_bound(
            trajectory.trajectory_point().begin(),
            trajectory.trajectory_point().end(), t,
            [](const common::TrajectoryPoint& p, const double t) {
                return p.relative_time() < t;
            });

    //* 从匹配的轨迹点获取速度 v和路径点的角度 theta，并计算速度的x和y分量
    double v = matched_it->v();
    double theta = matched_it->path_point().theta();
    double v_x = v * std::cos(theta);
    double v_y = v * std::sin(theta);

    //* 使用路径匹配器在参考线上找到与位置 s匹配的点，并获取该点的角度 ref_theta
    common::PathPoint obstacle_point_on_ref_line =
            common::math::PathMatcher::MatchToPath(*ptr_reference_line_, s);
    auto ref_theta = obstacle_point_on_ref_line.theta();

    //* 计算并返回速度在参考线上的投影值
    return std::cos(ref_theta) * v_x + std::sin(ref_theta) * v_y;
}

}  // namespace planning
}  // namespace apollo
