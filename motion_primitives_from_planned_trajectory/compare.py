#!/usr/bin/env python3

# Copyright (c) 2025, b»robotized
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Mathias Fuhrer

import os

# to run with ros2 run ...
from motion_primitives_from_planned_trajectory.compare_planned_and_executed_trajectory import compare_and_plot_trajectories
from motion_primitives_from_planned_trajectory.compare_planned_and_reduced_points import plot_cartesian_trajectory, plot_joint_trajectory

# to run with play button in VSCode ...
# from compare_planned_and_executed_trajectory import compare_and_plot_trajectories
# from compare_planned_and_reduced_points import plot_cartesian_trajectory, plot_joint_trajectory

def main():
    data_dir = "src/motion_primitives_from_planned_trajectory/data"

    # filename_planned = "trajectory_20250626_095540_planned.csv" 
    # filename_executed = "trajectory_20250626_095540_executed.csv" 
    # filename_reduced = "trajectory_20250626_095540_reduced_LIN_cartesian.csv"
    # mode = "cartesian" 

    filename_planned = "trajectory_20250626_095502_planned.csv" 
    filename_executed = "trajectory_20250626_095502_executed.csv" 
    filename_reduced = "trajectory_20250626_095502_reduced_PTP_joint.csv"
    mode = "joint" 

    filepath_planned = os.path.join(data_dir, filename_planned)
    filepath_executed = os.path.join(data_dir, filename_executed)
    filepath_reduced = os.path.join(data_dir, filename_reduced)

    joint_pos_names = [
        'shoulder_pan_joint_pos', 
        'shoulder_lift_joint_pos', 
        'elbow_joint_pos',
        'wrist_1_joint_pos', 
        'wrist_2_joint_pos', 
        'wrist_3_joint_pos'
    ]
    pose_names=["pose_x", "pose_y", "pose_z", "pose_qx", "pose_qy", "pose_qz", "pose_qw"]

    n_points = 100
    compare_and_plot_trajectories(filepath_planned, filepath_executed, joint_pos_names, n_points)

    if mode == "cartesian":
        plot_cartesian_trajectory(filepath_planned, filepath_reduced, pose_names)
    elif mode == "joint":
        plot_joint_trajectory(filepath_planned, filepath_reduced, joint_pos_names)

if __name__ == "__main__":
    main()