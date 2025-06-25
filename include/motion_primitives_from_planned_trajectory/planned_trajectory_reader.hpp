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

#ifndef PLANNED_TRAJECTORY_READER_HPP
#define PLANNED_TRAJECTORY_READER_HPP

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class PlannedTrajectoryReader : public rclcpp::Node {
public:
    explicit PlannedTrajectoryReader(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    const std::vector<std::string>& getJointNames() const;
    const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& getTrajectoryPoints() const;

    void writeToCSV(const std::vector<geometry_msgs::msg::Pose>& fk_poses, const std::string& filepath) const;

private:
    void trajectoryCallback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg);

    std::vector<std::string> joint_names_;
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> trajectory_points_;
    bool trajectory_received_;
    rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr subscription_;
};

#endif // PLANNED_TRAJECTORY_READER_HPP
