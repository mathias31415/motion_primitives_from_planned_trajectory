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

#ifndef JOINT_STATE_LOGGER_HPP
#define JOINT_STATE_LOGGER_HPP

#include <fstream>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JointStateLogger
{
public:
  JointStateLogger(std::shared_ptr<rclcpp::Node> node,
                   const std::string & filepath,
                   const std::string & topic_name = "/joint_states");
  ~JointStateLogger();

  void start();
  void stop();

private:
  void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  std::shared_ptr<rclcpp::Node> node_;
  std::string filepath_;
  std::string topic_name_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

  std::ofstream file_;
  bool header_written_;
  std::vector<std::string> joint_names_;
};


#endif // JOINT_STATE_LOGGER_HPP