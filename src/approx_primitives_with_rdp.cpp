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
#include "motion_primitives_from_planned_trajectory/rdp.hpp"

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using industrial_robot_motion_interfaces::msg::MotionSequence;
using industrial_robot_motion_interfaces::msg::MotionPrimitive;
using industrial_robot_motion_interfaces::msg::MotionArgument;

namespace approx_primitives_with_rdp {

MotionSequence approxLinPrimitivesWithRDP(
    const std::vector<Pose>& poses_list,
    double epsilon,
    double velocity,
    double acceleration)
{
    MotionSequence motion_sequence;
    std::vector<MotionPrimitive> motion_primitives;

    if (poses_list.empty()) {
        std::cerr << "[approxLinPrimitivesWithRDP] Warning: poses_list is empty." << std::endl;
        return motion_sequence;
    }

    // Convert Pose list to PointList for RDP processing
    rdp::PointList points;
    for (const auto& pose : poses_list) {
        points.push_back({pose.position.x, pose.position.y, pose.position.z});
    }

    // Reduce points using RDP algorithm
    rdp::PointList reduced_points = rdp::rdpRecursive(points, epsilon);

    // Create motion primitives from reduced points (skipping the first point --> current position)
    for (size_t i = 1; i < reduced_points.size(); ++i) {
        MotionPrimitive primitive;
        primitive.type = MotionPrimitive::LINEAR_CARTESIAN;
        
        if (i == reduced_points.size() - 1) {
            // Last point: blend radius = 0
            primitive.blend_radius = 0.0;
        } else {
            // Calculate distance to previous point
            double dx = reduced_points[i][0] - reduced_points[i - 1][0];
            double dy = reduced_points[i][1] - reduced_points[i - 1][1];
            double dz = reduced_points[i][2] - reduced_points[i - 1][2];
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

            double blend = 0.1 * dist;

            // Clamp blend radius to [0, 0.1] with minimum threshold 0.001
            if (blend < 0.001) {
                blend = 0.0;
            } else if (blend > 0.1) {
                blend = 0.1;
            }

            primitive.blend_radius = blend;
        }

        // TODO(mathias31415): Calculate velocity and acceleration based on the time from start
        MotionArgument arg_vel;
        arg_vel.argument_name = "velocity";
        arg_vel.argument_value = velocity;
        primitive.additional_arguments.push_back(arg_vel);

        MotionArgument arg_acc;
        arg_acc.argument_name = "acceleration";
        arg_acc.argument_value = acceleration;
        primitive.additional_arguments.push_back(arg_acc);

        PoseStamped pose_stamped;
        pose_stamped.pose.position.x = reduced_points[i][0];
        pose_stamped.pose.position.y = reduced_points[i][1];
        pose_stamped.pose.position.z = reduced_points[i][2];

        // Take orientation from original point (find matching index)
        // If not exactly equal, find closest point in original points:
        int matched_index = -1;
        double min_dist_sq = 1e12;
        for (size_t j = 0; j < points.size(); ++j) {
            double dx = points[j][0] - reduced_points[i][0];
            double dy = points[j][1] - reduced_points[i][1];
            double dz = points[j][2] - reduced_points[i][2];
            double dist_sq = dx*dx + dy*dy + dz*dz;
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                matched_index = static_cast<int>(j);
            }
        }
        if (matched_index >= 0 && matched_index < static_cast<int>(poses_list.size())) {
            pose_stamped.pose.orientation = poses_list[matched_index].orientation;
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
          << "velocity = " << velocity << ", "
          << "acceleration = " << acceleration
          << std::endl;
    }

    motion_sequence.motions = motion_primitives;
    std::cout << "Reduced " << points.size() << " points to " << (reduced_points.size()-1) << " LIN primitives with epsilon=" << epsilon << std::endl;

    return motion_sequence;
}


MotionSequence approxPtpPrimitivesWithRDP(
    const std::vector<std::vector<double>>& joint_positions,
    double epsilon,
    double velocity,
    double acceleration)
{
    MotionSequence motion_sequence;
    std::vector<MotionPrimitive> motion_primitives;

    if (joint_positions.empty()) {
        std::cerr << "[approxPtpPrimitivesWithRDP] Warning: joint_positions is empty." << std::endl;
        return motion_sequence;
    }

    rdp::PointList points = joint_positions;
    rdp::PointList reduced_points = rdp::rdpRecursive(points, epsilon);

    for (size_t i = 1; i < reduced_points.size(); ++i) {
        MotionPrimitive primitive;
        primitive.type = MotionPrimitive::LINEAR_JOINT;

        // TODO(mathias31415): Calculate blend radius based on distance to previous point
        primitive.blend_radius = 0.1;

        // TODO(mathias31415): Calculate velocity and acceleration based on the time from start
        MotionArgument arg_vel;
        arg_vel.argument_name = "velocity";
        arg_vel.argument_value = velocity;
        primitive.additional_arguments.push_back(arg_vel);

        MotionArgument arg_acc;
        arg_acc.argument_name = "acceleration";
        arg_acc.argument_value = acceleration;
        primitive.additional_arguments.push_back(arg_acc);

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

} // namespace approx_primitives_with_rdp
