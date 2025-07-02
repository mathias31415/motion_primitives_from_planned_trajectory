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

#include <fstream>
#include <iomanip>
#include "motion_primitives_from_planned_trajectory/planned_trajectory_reader.hpp"

PlannedTrajectoryReader::PlannedTrajectoryReader(const rclcpp::NodeOptions& options)
    : Node("planned_trajectory_reader", options), trajectory_received_(false)
{
    subscription_ = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
        "/display_planned_path", 10,
        std::bind(&PlannedTrajectoryReader::trajectoryCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Waiting for trajectory...");
}

void PlannedTrajectoryReader::trajectoryCallback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg)
{
    if (trajectory_received_ || msg->trajectory.empty()) {
        return;
    }

    const auto& traj = msg->trajectory[0].joint_trajectory;
    joint_names_ = traj.joint_names;
    trajectory_points_ = traj.points;
    trajectory_received_ = true;

    RCLCPP_INFO(this->get_logger(), "Trajectory received with %zu points.", trajectory_points_.size());
}

const std::vector<std::string>& PlannedTrajectoryReader::getJointNames() const {
    return joint_names_;
}

const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& PlannedTrajectoryReader::getTrajectoryPoints() const {
    return trajectory_points_;
}