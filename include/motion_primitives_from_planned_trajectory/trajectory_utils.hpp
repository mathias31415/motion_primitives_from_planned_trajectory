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

#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace trajectory_utils {

/**
 * @brief Vergleicht die aktuelle Joint-Position mit dem Startpunkt der geplanten Trajektorie.
 * 
 * @param node Shared Pointer auf den aktuellen Node.
 * @param joint_names Reihenfolge der Gelenknamen in der Trajektorie.
 * @param trajectory_points Liste der geplanten Trajektorienpunkte.
 * @param tolerance Erlaubter numerischer Unterschied zwischen Soll- und Istwert.
 * @return true Wenn alle Gelenkpositionen innerhalb der Toleranz übereinstimmen.
 * @return false Wenn mindestens ein Gelenk nicht übereinstimmt oder Daten fehlen.
 */
bool isStartStateMatching(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::vector<std::string>& joint_names,
    const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& trajectory_points,
    double tolerance = 1e-3
);

} // namespace trajectory_utils

#endif // TRAJECTORY_UTILS_HPP