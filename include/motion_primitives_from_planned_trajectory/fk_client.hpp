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

#ifndef FK_CLIENT_HPP
#define FK_CLIENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <geometry_msgs/msg/pose.hpp>

class FKClient
{
public:
  explicit FKClient(const rclcpp::Node::SharedPtr& node);

  std::optional<geometry_msgs::msg::Pose> computeFK(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& joint_positions,
    const std::string& from_frame = "base",
    const std::string& to_link = "tool0");

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr client_;
};

#endif // FK_CLIENT_HPP
