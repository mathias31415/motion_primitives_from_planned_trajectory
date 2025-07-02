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
#include <algorithm>
#include <cmath>
#include "motion_primitives_from_planned_trajectory/approx_primitives_with_rdp.hpp"

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using industrial_robot_motion_interfaces::msg::MotionSequence;
using industrial_robot_motion_interfaces::msg::MotionPrimitive;
using industrial_robot_motion_interfaces::msg::MotionArgument;

namespace approx_primitives_with_rdp {

MotionSequence approxLinPrimitivesWithRDP(
    const std::vector<approx_primitives_with_rdp::PlannedTrajectoryPoint>& trajectory,
    double epsilon,
    bool use_time_not_vel_and_acc)
{
    MotionSequence motion_sequence;
    std::vector<MotionPrimitive> motion_primitives;

    if (trajectory.empty()) {
        std::cerr << "[approxLinPrimitivesWithRDP] Warning: trajectory is empty." << std::endl;
        return motion_sequence;
    }

    rdp::PointList points;
    for (const auto& point : trajectory) {
        points.push_back({point.pose.position.x, point.pose.position.y, point.pose.position.z});
    }

    rdp::PointList reduced_points = rdp::rdpRecursive(points, epsilon);

    for (size_t i = 1; i < reduced_points.size(); ++i) {
        MotionPrimitive primitive;
        primitive.type = MotionPrimitive::LINEAR_CARTESIAN;

        if (i == reduced_points.size() - 1) {
            primitive.blend_radius = 0.0;
        } else {
            primitive.blend_radius = calculateBlendRadius(
                reduced_points[i - 1], reduced_points[i], reduced_points[i + 1]);
        }
        double velocity = -1.0;
        double acceleration = -1.0;
        double move_time = -1.0;
        if (use_time_not_vel_and_acc) {
            move_time = 2.0;

            MotionArgument arg_time;
            arg_time.argument_name = "move_time";
            arg_time.argument_value = move_time;
            primitive.additional_arguments.push_back(arg_time);
        } else {
            velocity = 1.0;
            acceleration = 1.0;

            MotionArgument arg_vel;
            arg_vel.argument_name = "velocity";
            arg_vel.argument_value = velocity;
            primitive.additional_arguments.push_back(arg_vel);

            MotionArgument arg_acc;
            arg_acc.argument_name = "acceleration";
            arg_acc.argument_value = acceleration;
            primitive.additional_arguments.push_back(arg_acc);
        }

        PoseStamped pose_stamped;
        pose_stamped.pose.position.x = reduced_points[i][0];
        pose_stamped.pose.position.y = reduced_points[i][1];
        pose_stamped.pose.position.z = reduced_points[i][2];

        int matched_index = -1;
        double min_dist_sq = 1e12;
        for (size_t j = 0; j < trajectory.size(); ++j) {
            double dx = trajectory[j].pose.position.x - reduced_points[i][0];
            double dy = trajectory[j].pose.position.y - reduced_points[i][1];
            double dz = trajectory[j].pose.position.z - reduced_points[i][2];
            double dist_sq = dx*dx + dy*dy + dz*dz;
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                matched_index = static_cast<int>(j);
            }
        }

        if (matched_index >= 0) {
            pose_stamped.pose.orientation = trajectory[matched_index].pose.orientation;
        } else {
            // Print error and throw exception
            std::cerr << "Error: Reduced point at index " << i << " could not be matched to any original point!" << std::endl;
            throw std::runtime_error("No matching original point found for reduced point in approxLinPrimitivesWithRDP.");
        }

        primitive.poses.push_back(pose_stamped);
        motion_primitives.push_back(primitive);

        std::cout << "Added LIN Primitive [" << i << "]: (x,y,z,qx,qy,qz,qw) = ("
          << pose_stamped.pose.position.x << ", "
          << pose_stamped.pose.position.y << ", "
          << pose_stamped.pose.position.z << ", "
          << pose_stamped.pose.orientation.x << ", "
          << pose_stamped.pose.orientation.y << ", "
          << pose_stamped.pose.orientation.z << ", "
          << pose_stamped.pose.orientation.w << "), "
          << "blend_radius = " << primitive.blend_radius << ", "
          << "move_time = " << move_time << ", "
          << "velocity = " << velocity << ", "
          << "acceleration = " << acceleration
          << std::endl;
    }

    motion_sequence.motions = motion_primitives;
    std::cout << "Reduced " << points.size() << " points to " << (reduced_points.size()-1) << " LIN primitives with epsilon=" << epsilon << std::endl;

    return motion_sequence;
}


MotionSequence approxPtpPrimitivesWithRDP(
    const std::vector<approx_primitives_with_rdp::PlannedTrajectoryPoint>& trajectory,
    double epsilon,
    bool use_time_not_vel_and_acc)
{
    MotionSequence motion_sequence;
    std::vector<MotionPrimitive> motion_primitives;

    if (trajectory.empty()) {
        std::cerr << "[approxPtpPrimitivesWithRDP] Warning: trajectory is empty." << std::endl;
        return motion_sequence;
    }

    rdp::PointList points;
    for (const auto& pt : trajectory) {
        points.push_back(pt.joint_positions);
    }

    rdp::PointList reduced_points = rdp::rdpRecursive(points, epsilon);

    // --- Find indices of reduced points in original joint_positions ---
    std::vector<int> reduced_to_original_index(reduced_points.size(), -1);
    for (size_t i = 0; i < reduced_points.size(); ++i) {
        for (size_t j = 0; j < points.size(); ++j) {
            if (points[j] == reduced_points[i]) {
                reduced_to_original_index[i] = static_cast<int>(j);
                break;
            }
        }
    }

    for (size_t i = 1; i < reduced_points.size(); ++i) {
        MotionPrimitive primitive;
        primitive.type = MotionPrimitive::LINEAR_JOINT;

        if (i == reduced_points.size() - 1) {
            primitive.blend_radius = 0.0;
        } else {
            int prev_index = reduced_to_original_index[i - 1];
            int curr_index = reduced_to_original_index[i];
            int next_index = reduced_to_original_index[i + 1];

            if (prev_index == -1 || curr_index == -1 || next_index == -1) {
                std::cerr << "[approxPtpPrimitivesWithRDP] Warning: Could not find all original indices at i=" << i << std::endl;
                primitive.blend_radius = 0.0;
            } else {
                rdp::Point prev_xyz = {trajectory[prev_index].pose.position.x, trajectory[prev_index].pose.position.y, trajectory[prev_index].pose.position.z};
                rdp::Point curr_xyz = {trajectory[curr_index].pose.position.x, trajectory[curr_index].pose.position.y, trajectory[curr_index].pose.position.z};
                rdp::Point next_xyz = {trajectory[next_index].pose.position.x, trajectory[next_index].pose.position.y, trajectory[next_index].pose.position.z};
                primitive.blend_radius = calculateBlendRadius(prev_xyz, curr_xyz, next_xyz);
            }
        }

        double velocity = -1.0;
        double acceleration = -1.0;
        double move_time = -1.0;
        if (use_time_not_vel_and_acc) {
            move_time = 2.0;

            MotionArgument arg_time;
            arg_time.argument_name = "move_time";
            arg_time.argument_value = move_time;
            primitive.additional_arguments.push_back(arg_time);
        } else {
            velocity = 1.0;
            acceleration = 1.0;

            MotionArgument arg_vel;
            arg_vel.argument_name = "velocity";
            arg_vel.argument_value = velocity;
            primitive.additional_arguments.push_back(arg_vel);

            MotionArgument arg_acc;
            arg_acc.argument_name = "acceleration";
            arg_acc.argument_value = acceleration;
            primitive.additional_arguments.push_back(arg_acc);
        }

        primitive.joint_positions = reduced_points[i];

        std::cout << "Added PTP Primitive [" << i << "]: joints = (";
        for (size_t j = 0; j < reduced_points[i].size(); ++j) {
            std::cout << reduced_points[i][j];
            if (j + 1 < reduced_points[i].size()) std::cout << ", ";
        }
        std::cout << "), blend_radius = " << primitive.blend_radius << ", "
          << "velocity = " << velocity << ", "
          << "acceleration = " << acceleration
          << std::endl;

        motion_primitives.push_back(primitive);
    }

    motion_sequence.motions = motion_primitives;
    std::cout << "Reduced " << points.size() << " joint points to " << (reduced_points.size() - 1) << " PTP primitives with epsilon=" << epsilon << std::endl;

    return motion_sequence;
}


double calculateBlendRadius(const rdp::Point& previous_point,
                            const rdp::Point& current_point,
                            const rdp::Point& next_point)
{
    double dist_prev = std::sqrt(
        std::pow(current_point[0] - previous_point[0], 2) +
        std::pow(current_point[1] - previous_point[1], 2) +
        std::pow(current_point[2] - previous_point[2], 2));

    double dist_next = std::sqrt(
        std::pow(next_point[0] - current_point[0], 2) +
        std::pow(next_point[1] - current_point[1], 2) +
        std::pow(next_point[2] - current_point[2], 2));

    double min_dist = std::min(dist_prev, dist_next);
    double blend = 0.1 * min_dist;

    // Clamp blend radius to [0, 0.1] with minimum threshold 0.001
    if (blend < 0.001) {
        blend = 0.0;
    } else if (blend > 0.1) {
        blend = 0.1;
    }

    return blend;
}

} // namespace approx_primitives_with_rdp
