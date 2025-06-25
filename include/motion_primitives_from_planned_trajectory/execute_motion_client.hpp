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

#ifndef EXECUTE_MOTION_CLIENT_HPP_
#define EXECUTE_MOTION_CLIENT_HPP_

#include <thread>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <industrial_robot_motion_interfaces/action/execute_motion.hpp>
#include <industrial_robot_motion_interfaces/msg/motion_sequence.hpp>

class ExecuteMotionClient : public rclcpp::Node
{
public:
    using ExecuteMotion = industrial_robot_motion_interfaces::action::ExecuteMotion;
    using GoalHandleExecuteMotion = rclcpp_action::ClientGoalHandle<ExecuteMotion>;

    explicit ExecuteMotionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~ExecuteMotionClient();

    void send_motion_sequence(const industrial_robot_motion_interfaces::msg::MotionSequence & motion_sequence, bool enable_cancel = true);

private:
    // Action client callbacks
    void goal_response_callback(GoalHandleExecuteMotion::SharedPtr goal_handle);
    void feedback_callback(GoalHandleExecuteMotion::SharedPtr, const std::shared_ptr<const ExecuteMotion::Feedback> feedback);
    void result_callback(const GoalHandleExecuteMotion::WrappedResult & result);

    // Cancel goal logic
    void wait_for_keypress();
    bool is_stdin_ready();  // Check if stdin is ready for reading without blocking
    void cancel_goal();
    void cancel_response_callback(rclcpp::Client<action_msgs::srv::CancelGoal>::SharedFuture future);

    rclcpp_action::Client<ExecuteMotion>::SharedPtr action_client_;
    rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr cancel_client_;

    std::shared_ptr<GoalHandleExecuteMotion> goal_handle_;
    std::thread cancel_thread_;
    std::atomic<bool> cancel_thread_running_;
};

#endif  // EXECUTE_MOTION_CLIENT_HPP_
