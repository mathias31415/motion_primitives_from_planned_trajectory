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

#ifndef TRAJECTORY_UTILS_HPP
#define TRAJECTORY_UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <vector>

namespace trajectory_utils
{

/**
 * @brief Compares the current joint positions with the start point of the planned trajectory.
 *
 * @param node Shared pointer to the current node.
 * @param joint_names Order of joint names in the trajectory.
 * @param trajectory_points List of planned trajectory points.
 * @param tolerance Allowed numerical difference between target and actual values.
 * @return true If all joint positions match within the tolerance.
 * @return false If at least one joint does not match or data is missing.
 */
bool isStartStateMatching(
  const std::shared_ptr<rclcpp::Node> & node, const std::vector<std::string> & joint_names,
  const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> & trajectory_points,
  double tolerance = 1e-3);

}  // namespace trajectory_utils

#endif  // TRAJECTORY_UTILS_HPP
