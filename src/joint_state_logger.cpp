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
// Authors: Mathias Fuhre

#include <iomanip>  // for std::setprecision
#include "motion_primitives_from_planned_trajectory/joint_state_logger.hpp"

JointStateLogger::JointStateLogger(std::shared_ptr<rclcpp::Node> node,
                                   const std::string & filepath,
                                   const std::string & topic_name)
: node_(node), filepath_(filepath), topic_name_(topic_name), header_written_(false)
{
  file_.open(filepath_, std::ios::out);
  if (!file_.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open file: %s", filepath_.c_str());
    throw std::runtime_error("Could not open file for joint state logging");
  }
}

JointStateLogger::~JointStateLogger()
{
  stop();
}

void JointStateLogger::start()
{
  subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    topic_name_,
    10,
    std::bind(&JointStateLogger::topic_callback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(node_->get_logger(), "Started recording joint states to %s", filepath_.c_str());
}

void JointStateLogger::topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!header_written_) {
    joint_names_ = msg->name;
    file_ << "timestamp";
    for (const auto & name : joint_names_) {
      file_ << "," << name << "_pos";
    }
    for (const auto & name : joint_names_) {
      file_ << "," << name << "_vel";
    }
    for (const auto & name : joint_names_) {
      file_ << "," << name << "_eff";
    }
    file_ << "\n";
    header_written_ = true;
  }

  double timestamp = node_->get_clock()->now().seconds();

  file_ << std::fixed << std::setprecision(9) << timestamp;
  for (const auto & val : msg->position) file_ << "," << val;
  for (const auto & val : msg->velocity) file_ << "," << val;
  for (const auto & val : msg->effort)  file_ << "," << val;
  file_ << "\n";
}

void JointStateLogger::stop()
{
  if (subscription_) {
    subscription_.reset();
  }
  if (file_.is_open()) {
    file_.close();
  }
  RCLCPP_INFO(node_->get_logger(), "Stopped recording joint states. File saved to %s", filepath_.c_str());
}
