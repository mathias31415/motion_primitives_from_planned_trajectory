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

#include "motion_primitives_from_planned_trajectory/pose_marker_visualizer.hpp"

using namespace visualization_msgs::msg;
using namespace geometry_msgs::msg;

PoseMarkerVisualizer::PoseMarkerVisualizer(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    marker_pub_ = node_->create_publisher<MarkerArray>("/visualization_marker_array", 10);
}

void PoseMarkerVisualizer::publishPoseMarkersToRViz(
    const std::vector<Pose>& poses,
    const std::string& frame_id,
    const std::string& marker_ns,
    float axis_length,
    float axis_width)
{
    MarkerArray marker_array;
    int marker_id = 0;

    for (const auto& pose : poses) {
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        tf2::Matrix3x3 rot(q);

        Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = node_->get_clock()->now();
        marker.ns = marker_ns;
        marker.id = marker_id++;
        marker.type = Marker::LINE_LIST;
        marker.action = Marker::ADD;
        marker.scale.x = axis_width;

        std_msgs::msg::ColorRGBA red, green, blue;
        red.r = 1.0; red.a = 1.0;
        green.g = 1.0; green.a = 1.0;
        blue.b = 1.0; blue.a = 1.0;

        std::array<tf2::Vector3, 3> directions = {
            rot.getColumn(0),  // X-axis
            rot.getColumn(1),  // Y-axis
            rot.getColumn(2)   // Z-axis
        };

        std::array<std_msgs::msg::ColorRGBA, 3> colors = {red, green, blue};

        for (size_t i = 0; i < 3; ++i) {
            Point start, end;
            start.x = pose.position.x;
            start.y = pose.position.y;
            start.z = pose.position.z;

            end.x = pose.position.x + axis_length * directions[i].x();
            end.y = pose.position.y + axis_length * directions[i].y();
            end.z = pose.position.z + axis_length * directions[i].z();

            marker.points.push_back(start);
            marker.points.push_back(end);
            marker.colors.push_back(colors[i]);
            marker.colors.push_back(colors[i]);
        }

        marker_array.markers.push_back(marker);
    }

    marker_pub_->publish(marker_array);
    RCLCPP_INFO(node_->get_logger(), "Published %zu coordinate frames to /visualization_marker_array.", poses.size());
}

void PoseMarkerVisualizer::deletePoseMarkers(int num_markers, const std::string& frame_id, const std::string& marker_ns)
{
    MarkerArray marker_array;

    for (int i = 0; i < num_markers; ++i) {
        Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = node_->get_clock()->now();
        marker.ns = marker_ns;
        marker.id = i;
        marker.action = Marker::DELETE;
        marker_array.markers.push_back(marker);
    }

    marker_pub_->publish(marker_array);
    RCLCPP_INFO(node_->get_logger(), "Deleted %d markers from /visualization_marker_array.", num_markers);
}
