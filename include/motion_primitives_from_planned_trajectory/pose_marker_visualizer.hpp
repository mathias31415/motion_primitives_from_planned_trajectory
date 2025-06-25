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

#ifndef POSE_MARKER_VISUALIZER_HPP
#define POSE_MARKER_VISUALIZER_HPP

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class PoseMarkerVisualizer {
public:
    PoseMarkerVisualizer(rclcpp::Node::SharedPtr node);

    void publishPoseMarkersToRViz(
        const std::vector<geometry_msgs::msg::Pose>& poses,
        const std::string& frame_id = "world",
        const std::string& marker_ns = "pose_axes",
        float axis_length = 0.1f,
        float axis_width = 0.002f
    );

    void deletePoseMarkers(
        int num_markers,
        const std::string& frame_id = "world",
        const std::string& marker_ns = "pose_axes"
    );

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

#endif // POSE_MARKER_VISUALIZER_HPP
