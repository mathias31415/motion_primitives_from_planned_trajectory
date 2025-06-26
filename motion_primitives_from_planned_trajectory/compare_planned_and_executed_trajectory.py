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

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import os

def compare_and_plot_trajectories(filepath_planned, filepath_executed, joint_pos_names, n_points):
    # Load CSV files
    df_planned = pd.read_csv(filepath_planned)
    df_executed = pd.read_csv(filepath_executed)

    # Remove leading/trailing rows of executed trajectory where all velocities are zero
    vel_cols = [col for col in df_executed.columns if 'vel' in col]
    moving_mask = ~(df_executed[vel_cols] == 0).all(axis=1)
    start_index = moving_mask.idxmax()
    end_index = moving_mask[::-1].idxmax()
    df_executed_clean = df_executed.loc[start_index:end_index].reset_index(drop=True)

    # Resample planned trajectory
    planned_positions = df_planned[joint_pos_names].values
    interp_planned = interp1d(np.linspace(0, 1, len(planned_positions)), planned_positions, axis=0)
    planned_resampled = interp_planned(np.linspace(0, 1, n_points))

    # Resample executed trajectory
    executed_positions = df_executed_clean[joint_pos_names].values
    interp_executed = interp1d(np.linspace(0, 1, len(executed_positions)), executed_positions, axis=0)
    executed_resampled = interp_executed(np.linspace(0, 1, n_points))

    # Compute RMSE per joint and total
    rmse = np.sqrt(np.mean((planned_resampled - executed_resampled) ** 2, axis=0))
    total_rmse = np.sqrt(np.mean((planned_resampled - executed_resampled) ** 2))
    print(f'Total RMSE of planned and executed trajectory: {total_rmse:.6f}')

    # Plot
    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    axes = axes.ravel()

    for i, joint in enumerate(joint_pos_names):
        axes[i].plot(planned_resampled[:, i], 'o--', color='blue', label='Planned', markersize=2)
        axes[i].plot(executed_resampled[:, i], 'o-', color='red', label='Executed', markersize=2)
        axes[i].set_title(f'{joint} (RMSE: {rmse[i]:.4f})')
        axes[i].set_xlabel('Normalized index')
        axes[i].set_ylabel('Joint-Position in radians')
        axes[i].set_ylim(-3.5, 3.5)
        axes[i].grid(True)
        axes[i].legend()

    # Save figure
    base_name = os.path.basename(filepath_planned).replace('_planned.csv', '_compare_planned_vs_executed.png')
    plot_path = os.path.join(os.path.dirname(filepath_planned), base_name)
    plt.tight_layout()
    plt.savefig(plot_path)
    plt.show()
    print(f"Figure with comparison saved to: {plot_path}")

def main():
    data_dir = 'src/motion_primitives_from_planned_trajectory/data'
    filename_planned = 'trajectory_20250626_095540_planned.csv'
    filename_executed = 'trajectory_20250626_095540_executed.csv'
    filepath_planned = os.path.join(data_dir, filename_planned)
    filepath_executed = os.path.join(data_dir, filename_executed)
    joint_pos_names = [
        'shoulder_pan_joint_pos', 'shoulder_lift_joint_pos', 'elbow_joint_pos',
        'wrist_1_joint_pos', 'wrist_2_joint_pos', 'wrist_3_joint_pos'
    ]
    n_points = 100

    compare_and_plot_trajectories(filepath_planned, filepath_executed, joint_pos_names, n_points)

if __name__ == "__main__":
    main()
