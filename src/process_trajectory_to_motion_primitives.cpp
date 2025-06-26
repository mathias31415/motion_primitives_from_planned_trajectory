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
#include "motion_primitives_from_planned_trajectory/joint_state_logger.hpp"
#include "motion_primitives_from_planned_trajectory/trajectory_utils.hpp"

#define DATA_DIR "src/motion_primitives_from_planned_trajectory/data"

using geometry_msgs::msg::Pose;
using industrial_robot_motion_interfaces::msg::MotionSequence;
using approx_primitives_with_rdp::approxPtpPrimitivesWithRDP;
using approx_primitives_with_rdp::approxLinPrimitivesWithRDP;

double epsilon = 0.01;
double blend_radius = 0.1;
double velocity = 0.5;
double acceleration = 0.5;

void writeJointPositionsToCSV(const std::vector<std::vector<double>>& joint_positions,
                               const std::vector<std::string>& joint_names,
                               const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }

    // Header
    for (size_t i = 0; i < joint_names.size(); ++i) {
        file << joint_names[i];
        if (i < joint_names.size() - 1) file << ",";
    }
    file << "\n";

    // Values
    for (const auto& vec : joint_positions) {
        for (size_t i = 0; i < vec.size(); ++i) {
            file << vec[i];
            if (i < vec.size() - 1) file << ",";
        }
        file << "\n";
    }

    file.close();
}
void writePosesToCSV(const std::vector<Pose>& poses,
                     const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }
    // Header
    file << "pose_x,pose_y,pose_z,pose_qx,pose_qy,pose_qz,pose_qw\n";
    // Values
    for (const auto& pose : poses) {
        file << pose.position.x << "," << pose.position.y << "," << pose.position.z << ","
             << pose.orientation.x << "," << pose.orientation.y << ","
             << pose.orientation.z << "," << pose.orientation.w << "\n";
    }

    file.close();
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlannedTrajectoryReader>();

    // Generate target path for CSV file (with timestamp)
    std::ostringstream filename_planned_traj;
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&t);

    filename_planned_traj << DATA_DIR << "/trajectory_"
             << std::put_time(&tm, "%Y%m%d_%H%M%S")
             << "_planned.csv";

    std::string output_file_planned_traj = filename_planned_traj.str();

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

    const auto& joint_names = node->getJointNames();
    const auto& traj_points = node->getTrajectoryPoints();

    // Check if first trajectory point matches the robot's current joint state
    if (!trajectory_utils::isStartStateMatching(node, joint_names, traj_points)) {
        RCLCPP_ERROR(node->get_logger(), "First trajectory point doesn't match the robot's current joint state. Execution not safe.");
        rclcpp::shutdown();
        return 1;
    }

    // Start FK client
    FKClient fk_client(node);
    std::vector<Pose> fk_poses;
    std::vector<std::vector<double>> joint_positions;
    fk_poses.reserve(traj_points.size());
    joint_positions.reserve(traj_points.size());

    for (const auto& point : traj_points) {
        joint_positions.push_back(point.positions);

        auto pose_opt = fk_client.computeFK(joint_names, point.positions);
        fk_poses.push_back(pose_opt.value_or(Pose()));
    }

    // Save FK poses to CSV
    node->writeToCSV(fk_poses, output_file_planned_traj);

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
        // RCLCPP_INFO(node->get_logger(), "Approximated PTP motion sequence with %zu primitives", motion_sequence.motions.size());

        std::vector<std::vector<double>> reduced_joint_positions;
        // Match joint positions and retrieve corresponding FK pose
        for (const auto& primitive : motion_sequence.motions) {
            if (primitive.type != industrial_robot_motion_interfaces::msg::MotionPrimitive::LINEAR_JOINT) {
                continue;
            }
            reduced_joint_positions.push_back(primitive.joint_positions);
            // Find the FK pose that matches the joint positions in this primitive
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

        // Save reduced joint positions to CSV
        std::ostringstream filename_reduced;
        filename_reduced << DATA_DIR << "/trajectory_"
             << std::put_time(&tm, "%Y%m%d_%H%M%S")
             << "_reduced_PTP_joint.csv";
        std::string output_file_reduced_traj = filename_reduced.str();
        writeJointPositionsToCSV(reduced_joint_positions, joint_names, output_file_reduced_traj);


    } else if (method == "2") {
        motion_sequence = approxLinPrimitivesWithRDP(fk_poses, epsilon, blend_radius, velocity, acceleration);
        // RCLCPP_INFO(node->get_logger(), "Approximated LIN motion sequence with %zu primitives", motion_sequence.motions.size());

        for (const auto& primitive : motion_sequence.motions) {
            for (const auto& pose_stamped : primitive.poses) {
                reduced_poses.push_back(pose_stamped.pose);
            }
        }
        // Save reduced joint positions to CSV
        std::ostringstream filename_reduced;
        filename_reduced << DATA_DIR << "/trajectory_"
             << std::put_time(&tm, "%Y%m%d_%H%M%S")
             << "_reduced_LIN_cartesian.csv";
        std::string output_file_reduced_traj = filename_reduced.str();
        writePosesToCSV(reduced_poses, output_file_reduced_traj);

    } else {
        RCLCPP_ERROR(node->get_logger(), "Invalid method selected: %s", method.c_str());
    }

    PoseMarkerVisualizer visualizer(node);
    std::string frame_id = "base";
    std::string marker_ns = "motion_primitive_goal_poses";
    visualizer.publishPoseMarkersToRViz(reduced_poses, frame_id, marker_ns);

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

            // Start logging trajectory to compare planned and executed trajectory
            std::ostringstream filename_executed_traj;
            filename_executed_traj << DATA_DIR << "/trajectory_"
                    << std::put_time(&tm, "%Y%m%d_%H%M%S")
                    << "_executed.csv";
            std::string output_file_executed_traj = filename_executed_traj.str();

            JointStateLogger joint_state_logger(motion_node, output_file_executed_traj);
            joint_state_logger.start();

            // Send motion sequence
            motion_node->send_motion_sequence(motion_sequence);

            // Spin node while executing
            rclcpp::spin(motion_node);

            // joint_state_logger.stop(); // not needed since destructor will call stop() when motion_node gets destroyed

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
