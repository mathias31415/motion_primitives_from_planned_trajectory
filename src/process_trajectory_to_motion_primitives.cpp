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
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include "motion_primitives_from_planned_trajectory/planned_trajectory_reader.hpp"
#include "motion_primitives_from_planned_trajectory/fk_client.hpp"
#include "industrial_robot_motion_interfaces/msg/motion_sequence.hpp"
#include "motion_primitives_from_planned_trajectory/approx_primitives_with_rdp.hpp"
#include "motion_primitives_from_planned_trajectory/pose_marker_visualizer.hpp"
#include "motion_primitives_from_planned_trajectory/execute_motion_client.hpp"

#define DATA_DIR "src/motion_primitives_from_planned_trajectory/data"

using geometry_msgs::msg::Pose;
using industrial_robot_motion_interfaces::msg::MotionSequence;
using approx_primitives_with_rdp::approxPtpPrimitivesWithRDP;
using approx_primitives_with_rdp::approxLinPrimitivesWithRDP;

double epsilon = 0.01;
double blend_radius = 0.1;
double velocity = 0.5;
double acceleration = 0.5;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlannedTrajectoryReader>();

    // Generate target path for CSV file (with timestamp)
    std::ostringstream filename;
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&t);

    filename << DATA_DIR << "/trajectory_"
             << std::put_time(&tm, "%Y%m%d_%H%M%S")
             << "_planned.csv";

    std::string output_file = filename.str();

    // Create data directory if necessary
    if (!std::filesystem::exists(DATA_DIR)) {
        try {
            std::filesystem::create_directories(DATA_DIR);
        } catch (const std::filesystem::filesystem_error& e) {
            RCLCPP_WARN_STREAM(node->get_logger(), "Could not create data directory: " << e.what());
        }
    }

    // Spin until a trajectory is received
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (!node->getTrajectoryPoints().empty()) {
            break;
        }
        rate.sleep();
    }

    // Start FK client
    FKClient fk_client(node);
    const auto& joint_names = node->getJointNames();
    const auto& points = node->getTrajectoryPoints();

    std::vector<Pose> fk_poses;
    std::vector<std::vector<double>> joint_positions;
    fk_poses.reserve(points.size());
    joint_positions.reserve(points.size());

    for (const auto& point : points) {
        joint_positions.push_back(point.positions);

        auto pose_opt = fk_client.computeFK(joint_names, point.positions);
        fk_poses.push_back(pose_opt.value_or(Pose()));
    }

    // Save FK poses to CSV
    node->writeToCSV(fk_poses, output_file);

    // Prompt user for motion type
    std::string user_input_moprim;
    std::string method = "1";  // Default to PTP

    while (true) {
        std::cout << "Which motion type should be used?\n"
                  << "[1] PTP (default)\n"
                  << "[2] LIN\n"
                  << "(Cancel with Ctrl+C)\n> ";
        std::getline(std::cin, user_input_moprim);

        if (user_input_moprim.empty()) {
            break;  // default
        }

        if (user_input_moprim == "1" || user_input_moprim == "2") {
            method = user_input_moprim;
            break;
        } else {
            std::cout << "Invalid input. Please enter '1' for PTP or '2' for LIN." << std::endl;
        }
    }

    // Approximation step
    MotionSequence motion_sequence;
    std::vector<Pose> reduced_poses;

    if (method == "1") {
        motion_sequence = approxPtpPrimitivesWithRDP(joint_positions, epsilon, blend_radius, velocity, acceleration);
        RCLCPP_INFO(node->get_logger(), "Approximated PTP motion sequence with %zu primitives", motion_sequence.motions.size());

        // Match joint positions and retrieve corresponding FK pose
        for (const auto& primitive : motion_sequence.motions) {
            if (primitive.type != industrial_robot_motion_interfaces::msg::MotionPrimitive::LINEAR_JOINT) {
                continue;
            }

            const auto& joint_vec = primitive.joint_positions;
            auto match_it = std::find_if(joint_positions.begin(), joint_positions.end(),
                [&joint_vec](const std::vector<double>& jp) {
                    if (jp.size() != joint_vec.size()) return false;
                    for (size_t i = 0; i < jp.size(); ++i) {
                        if (std::abs(jp[i] - joint_vec[i]) > 1e-5) return false;
                    }
                    return true;
                });

            if (match_it != joint_positions.end()) {
                size_t match_idx = std::distance(joint_positions.begin(), match_it);
                reduced_poses.push_back(fk_poses[match_idx]);
            } else {
                RCLCPP_WARN(node->get_logger(), "No FK match found for joint values in primitive.");
            }
        }
    } else if (method == "2") {
        motion_sequence = approxLinPrimitivesWithRDP(fk_poses, epsilon, blend_radius, velocity, acceleration);
        RCLCPP_INFO(node->get_logger(), "Approximated LIN motion sequence with %zu primitives", motion_sequence.motions.size());

        for (const auto& primitive : motion_sequence.motions) {
            for (const auto& pose_stamped : primitive.poses) {
                reduced_poses.push_back(pose_stamped.pose);
            }
    }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Invalid method selected: %s", method.c_str());
    }

    RCLCPP_INFO(node->get_logger(), "Reduced poses:");
    for (size_t i = 0; i < reduced_poses.size(); ++i) {
        const auto& pose = reduced_poses[i];
        RCLCPP_INFO_STREAM(node->get_logger(),
            "Pose [" << i << "]: "
                    << "Position(x=" << pose.position.x
                    << ", y=" << pose.position.y
                    << ", z=" << pose.position.z << "), "
                    << "Orientation(x=" << pose.orientation.x
                    << ", y=" << pose.orientation.y
                    << ", z=" << pose.orientation.z
                    << ", w=" << pose.orientation.w << ")"
        );
    }

    PoseMarkerVisualizer visualizer(node);
    std::string frame_id = "base";
    std::string marker_ns = "motion_primitive_goal_poses";
    visualizer.publishPoseMarkersToRViz(reduced_poses, frame_id, marker_ns);

    // TODO: publish markers, execute primitives etc.
    std::string user_input_execute;
    while (true) {
        std::cout << "Do you want to continue with motion primitive execution? (y/N): ";
        std::getline(std::cin, user_input_execute);

        // Convert to lowercase
        std::transform(user_input_execute.begin(), user_input_execute.end(), user_input_execute.begin(), ::tolower);

        if (user_input_execute == "y" || user_input_execute == "yes") {
            visualizer.deletePoseMarkers(static_cast<int>(reduced_poses.size()), frame_id, marker_ns);
            RCLCPP_INFO(node->get_logger(), "Starting execution of motion primitives...");

            // Setup motion execution client
            auto motion_node = std::make_shared<ExecuteMotionClient>();

            // Start logging
            // std::string executed_csv_path = save_dir + "/trajectory_" + timestamp + "_executed.csv";
            // JointStateLogger joint_state_logger(motion_node, executed_csv_path);
            // joint_state_logger.start();

            // Send motion sequence
            motion_node->send_motion_sequence(motion_sequence);

            // Spin node while executing
            rclcpp::spin(motion_node);

            // // Stop logging
            // joint_state_logger.stop();

            break;
        } else if (user_input_execute.empty() || user_input_execute == "n" || user_input_execute == "no") {
            RCLCPP_INFO(node->get_logger(), "Exiting without primitive execution.");
            visualizer.deletePoseMarkers(static_cast<int>(reduced_poses.size()), frame_id, marker_ns);
            break;
        } else {
            std::cout << "Invalid input." << std::endl;
        }
    }

    rclcpp::shutdown();
    return 0;
}
