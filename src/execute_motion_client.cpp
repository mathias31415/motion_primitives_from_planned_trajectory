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

#include <iostream>
#include <sys/select.h>
#include "motion_primitives_from_planned_trajectory/execute_motion_client.hpp"

using ExecuteMotion = industrial_robot_motion_interfaces::action::ExecuteMotion;
using GoalHandleExecuteMotion = rclcpp_action::ClientGoalHandle<ExecuteMotion>;

ExecuteMotionClient::ExecuteMotionClient(const rclcpp::NodeOptions & options)
: Node("motion_sequence_client", options), cancel_thread_running_(false)
{
    action_client_ = rclcpp_action::create_client<ExecuteMotion>(this, "/motion_primitive_controller/motion_sequence");
    cancel_client_ = this->create_client<action_msgs::srv::CancelGoal>("/motion_primitive_controller/motion_sequence/_action/cancel_goal");
}

ExecuteMotionClient::~ExecuteMotionClient()
{
    cancel_thread_running_ = false;
    if (cancel_thread_.joinable()) {
        cancel_thread_.join();
    }
}

void ExecuteMotionClient::send_motion_sequence(const industrial_robot_motion_interfaces::msg::MotionSequence & motion_sequence, bool enable_cancel)
{
    RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return;
    }

    auto goal_msg = ExecuteMotion::Goal();
    goal_msg.trajectory = motion_sequence;

    RCLCPP_INFO(this->get_logger(), "Sending %zu motion primitives as a sequence...", goal_msg.trajectory.motions.size());

    auto send_goal_options = rclcpp_action::Client<ExecuteMotion>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&ExecuteMotionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&ExecuteMotionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&ExecuteMotionClient::result_callback, this, std::placeholders::_1);

    auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);

    if (enable_cancel) {
        cancel_thread_running_ = true;
        cancel_thread_ = std::thread(&ExecuteMotionClient::wait_for_keypress, this);
    }
}

void ExecuteMotionClient::goal_response_callback(GoalHandleExecuteMotion::SharedPtr goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
    goal_handle_ = goal_handle;
}


void ExecuteMotionClient::feedback_callback(GoalHandleExecuteMotion::SharedPtr, const std::shared_ptr<const ExecuteMotion::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Executing primitive index: %d", feedback->current_primitive_index);
}

void ExecuteMotionClient::result_callback(const GoalHandleExecuteMotion::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Motion sequence executed successfully!");
        break;
        case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(this->get_logger(), "Motion sequence was canceled.");
        break;
        case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Motion sequence execution failed.");
        break;
        default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }

    cancel_thread_running_ = false;
    if (cancel_thread_.joinable()) {
        cancel_thread_.join();
    }

    rclcpp::shutdown();
}

bool ExecuteMotionClient::is_stdin_ready()
{
    // Check if stdin is ready for reading without blocking
    fd_set set;
    struct timeval timeout;

    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int ret = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);
    return ret > 0;
}

void ExecuteMotionClient::wait_for_keypress()
{
    std::cout << "Press ENTER to cancel the motion sequence..." << std::endl;
    while (cancel_thread_running_) {
        if (is_stdin_ready()) { // Check if stdin is ready for reading without blocking
            std::string input;
            if (!std::getline(std::cin, input)) {
                // EOF or error
                break;
            }
            if (input.empty()) {
                RCLCPP_INFO(this->get_logger(), "ENTER key pressed: sending cancel request.");
                cancel_goal();
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void ExecuteMotionClient::cancel_goal()
{
    if (!goal_handle_) {
        RCLCPP_WARN(this->get_logger(), "No goal to cancel (goal_handle not set).");
        return;
    }

    if (!cancel_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Cancel service is not available.");
        return;
    }

    auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
    // request->goal_info.goal_id = goal_handle_->get_goal_id();
    unique_identifier_msgs::msg::UUID uuid_msg;
    std::array<uint8_t, 16> uuid = goal_handle_->get_goal_id();
    std::copy(uuid.begin(), uuid.end(), uuid_msg.uuid.begin());
    request->goal_info.goal_id = uuid_msg;

    auto cancel_future = cancel_client_->async_send_request(request,
        std::bind(&ExecuteMotionClient::cancel_response_callback, this, std::placeholders::_1));
}

void ExecuteMotionClient::cancel_response_callback(rclcpp::Client<action_msgs::srv::CancelGoal>::SharedFuture future)
{
    try {
        auto response = future.get();
        if (response->return_code == 0) {
        RCLCPP_INFO(this->get_logger(), "Cancel request accepted.");
        } else if (response->return_code == 1) {
        RCLCPP_WARN(this->get_logger(), "Cancel request rejected.");
        } else {
        RCLCPP_WARN(this->get_logger(), "Cancel returned code: %d", response->return_code);
        }
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call cancel service: %s", e.what());
    }
}
