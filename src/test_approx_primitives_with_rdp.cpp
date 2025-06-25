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
#include <vector>
#include <memory>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <industrial_robot_motion_interfaces/msg/motion_sequence.hpp>
#include <industrial_robot_motion_interfaces/msg/motion_primitive.hpp>
#include <industrial_robot_motion_interfaces/msg/motion_argument.hpp>
#include "motion_primitives_from_planned_trajectory/approx_primitives_with_rdp.hpp"

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using industrial_robot_motion_interfaces::msg::MotionSequence;
using industrial_robot_motion_interfaces::msg::MotionPrimitive;
using industrial_robot_motion_interfaces::msg::MotionArgument;
using approx_primitives_with_rdp::approxPtpPrimitivesWithRDP;
using approx_primitives_with_rdp::approxLinPrimitivesWithRDP;

Pose createPose(double x, double y, double z, double qx, double qy, double qz, double qw) {
    Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;
    return pose;
}

void testCartesianRDP() {
    std::vector<Pose> poses;

    // Simple arc or curve-like shape
    for (double t = 0.0; t < 1.0; t += 0.05) {
        double x = t;
        double y = std::sin(t * 3.14);
        double z = 0.1 * t;
        poses.push_back(createPose(x, y, z, 0.0, 0.0, 0.0, 1.0));
    }

    std::cout << "Testing LIN primitive RDP simplification with " << poses.size() << " poses...\n";

    MotionSequence result = approxLinPrimitivesWithRDP(poses, 0.02, 0.01, 0.2, 0.3);

    std::cout << "Result contains " << result.motions.size() << " simplified LIN primitives.\n\n";
}

void testJointRDP() {
    std::vector<std::vector<double>> joint_positions;

    // Simulierter Bewegungsverlauf über 100 Zeitpunkte für 6 Gelenke
    for (int i = 0; i < 100; ++i) {
        double t = static_cast<double>(i) / 100.0;
        joint_positions.push_back({
            std::sin(t * M_PI),         // joint1
            std::cos(t * M_PI),         // joint2
            std::sin(2 * t * M_PI),     // joint3
            std::cos(2 * t * M_PI),     // joint4
            std::sin(4 * t * M_PI),     // joint5
            t                           // joint6 (linear Verlauf)
        });
    }

    std::cout << "Testing PTP primitive RDP simplification with "
              << joint_positions.size() << " joint points."
              << std::endl;

    MotionSequence result = approxPtpPrimitivesWithRDP(
        joint_positions,
        0.05,  // epsilon (Toleranz)
        0.02,  // blend radius
        0.1,   // velocity
        0.1    // acceleration
    );

    std::cout << "Result contains " << result.motions.size()
              << " simplified PTP primitives.\n\n";
}

int main() {
    std::cout << "==== RDP Approximation Test ====" << std::endl;

    try {
        testCartesianRDP();
        testJointRDP();
    } catch (const std::exception& ex) {
        std::cerr << "Test failed: " << ex.what() << std::endl;
        return 1;
    }

    std::cout << "==== Test completed ====" << std::endl;
    return 0;
}
