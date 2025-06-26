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

#include <algorithm>
#include <chrono>
#include "motion_primitives_from_planned_trajectory/trajectory_utils.hpp"

namespace trajectory_utils {

bool isStartStateMatching(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::vector<std::string>& joint_names,
    const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& trajectory_points,
    double tolerance)
{
    if (trajectory_points.empty()) {
        RCLCPP_WARN(node->get_logger(), "No trajectory points available to compare.");
        return false;
    }

    sensor_msgs::msg::JointState::SharedPtr current_joint_state = nullptr;

    auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [&current_joint_state](const sensor_msgs::msg::JointState::SharedPtr msg) {
            current_joint_state = msg;
        }
    );

    // Wait to receive data
    auto start = std::chrono::steady_clock::now();
    rclcpp::Rate rate(20);
    while (rclcpp::ok() && current_joint_state == nullptr) {
        rclcpp::spin_some(node);
        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(2)) {
            RCLCPP_WARN(node->get_logger(), "Timeout while waiting for /joint_states");
            return false;
        }
        rate.sleep();
    }

    const auto& planned_point = trajectory_points.front();  // Get the first trajectory point

    if (planned_point.positions.size() != joint_names.size()) {
        RCLCPP_WARN(node->get_logger(), "Mismatch in size between joint_names and planned point.");
        return false;
    }

    bool all_match = true;
    for (size_t i = 0; i < joint_names.size(); ++i) {
        const std::string& joint = joint_names[i];

        auto it = std::find(current_joint_state->name.begin(), current_joint_state->name.end(), joint);
        if (it == current_joint_state->name.end()) {
            RCLCPP_WARN(node->get_logger(), "Joint name %s not found in current joint state.", joint.c_str());
            all_match = false;
            continue;
        }

        size_t idx = std::distance(current_joint_state->name.begin(), it);
        double actual = current_joint_state->position[idx];
        double expected = planned_point.positions[i];

        if (std::abs(actual - expected) > tolerance) {
            RCLCPP_WARN(node->get_logger(),
                "Joint %s differs: expected=%.4f, actual=%.4f", joint.c_str(), expected, actual);
            all_match = false;
        }
    }

    if (all_match) {
        RCLCPP_INFO(node->get_logger(), "Current joint state matches the first trajectory point.");
    } else {
        RCLCPP_WARN(node->get_logger(), "Current joint state does NOT match the first trajectory point.");
    }

    return all_match;
}

} // namespace trajectory_utils
