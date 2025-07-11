// Copyright (c) 2025, b»robotized
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Mathias Fuhrer

#ifndef APPROX_PRIMITIVES_WITH_RDP_HPP
#define APPROX_PRIMITIVES_WITH_RDP_HPP

#include <vector>
#include <string>
#include "geometry_msgs/msg/pose.hpp"
#include "industrial_robot_motion_interfaces/msg/motion_primitive.hpp"
#include "industrial_robot_motion_interfaces/msg/motion_sequence.hpp"
#include "industrial_robot_motion_interfaces/msg/motion_argument.hpp"
#include "motion_primitives_from_planned_trajectory/rdp.hpp"

using MotionSequence = industrial_robot_motion_interfaces::msg::MotionSequence;

namespace approx_primitives_with_rdp {

struct PlannedTrajectoryPoint
{
    double time_from_start;
    std::vector<double> joint_positions;
    geometry_msgs::msg::Pose pose;
};

// Approximate with LIN Primitives in Cartesian Space
MotionSequence approxLinPrimitivesWithRDP(
    const std::vector<PlannedTrajectoryPoint>& trajectory,
    double epsilon,
    bool use_time_not_vel_and_acc = false
);

// Approximate with PTP Primitives in Joint Space
MotionSequence approxPtpPrimitivesWithRDP(
    const std::vector<PlannedTrajectoryPoint>& trajectory,
    double epsilon,
    bool use_time_not_vel_and_acc = false
);

double calculateBlendRadius(
    const rdp::Point& previous_point, 
    const rdp::Point& current_point,
    const rdp::Point& next_point
);

} // namespace approx_primitives_with_rdp

#endif // APPROX_PRIMITIVES_WITH_RDP_HPP