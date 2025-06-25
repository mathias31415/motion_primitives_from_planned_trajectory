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

#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include "motion_primitives_from_planned_trajectory/planned_trajectory_reader.hpp"
#include "motion_primitives_from_planned_trajectory/fk_client.hpp"

#define DATA_DIR "src/motion_primitives_from_planned_trajectory/data"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlannedTrajectoryReader>();

    // Erzeuge Zielpfad für CSV-Datei mit Zeitstempel
    std::ostringstream filename;
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&t);

    filename << DATA_DIR << "/trajectory_"
             << std::put_time(&tm, "%Y%m%d_%H%M%S")
             << "_planned.csv";

    std::string output_file = filename.str();

    // Erzeuge data-Verzeichnis falls nötig
    if (!std::filesystem::exists(DATA_DIR)) {
        try {
            std::filesystem::create_directories(DATA_DIR);
        } catch (const std::filesystem::filesystem_error& e) {
            RCLCPP_WARN_STREAM(node->get_logger(), "Could not create data directory: " << e.what());
        }
    }

    RCLCPP_INFO(node->get_logger(), "Waiting for planned trajectory...");

    // Spin bis Trajektorie empfangen
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (!node->getTrajectoryPoints().empty()) {
            break;
        }
        rate.sleep();
    }

    // Starte FK-Client
    FKClient fk_client(node);

    const auto& joint_names = node->getJointNames();
    const auto& points = node->getTrajectoryPoints();

    std::vector<geometry_msgs::msg::Pose> fk_poses;
    fk_poses.reserve(points.size());

    for (size_t i = 0; i < points.size(); ++i) {
        const auto& positions = points[i].positions;
        auto pose_opt = fk_client.computeFK(joint_names, positions);

        if (pose_opt.has_value()) {
            fk_poses.push_back(pose_opt.value());
        } else {
            fk_poses.emplace_back();  // leere Pose bei Fehler
        }
    }

    // Exportiere als CSV
    node->writeToCSV(fk_poses, output_file);
    RCLCPP_INFO(node->get_logger(), "Saved CSV to: %s", output_file.c_str());

    rclcpp::shutdown();
    return 0;
}
