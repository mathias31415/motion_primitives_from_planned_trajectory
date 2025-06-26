motion_primitives_from_planned_trajectory
==========================================

Package to approximate a planned trajectory using motion primitives such as PTP, LIN, and CIRC

![Licence](https://img.shields.io/badge/License-Apache-2.0-blue.svg)

# Usage notes
## Launch UR Driver
**With Simulation**
Start Simulation
```
ros2 run ur_client_library start_ursim.sh -m ur10e
```
Start Driver
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=192.168.56.101 initial_joint_controller:=motion_primitive_controller launch_rviz:=false
```
(optional) switch control mode
```
ros2 control switch_controllers --activate motion_primitive_controller --deactivate scaled_joint_trajectory_controller
```
Start MoveIt
```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e launch_rviz:=true
```

## Process and execute motion primitives from planned trajectory
Start the python script with the following command:
```
ros2 run motion_primitives_from_planned_trajectory process_trajectory_to_motion_primitives
```
Then plan a trajectory in RViz with MoveIt by pressing `plan`. The python script will:
1. Read the planned trajectory from `/display_planned_path`.
2. Calculate the endefector pose for every point in the trajectory using the `/compute_fk` service.
3. Save the trajectory and endefector poses to a `trajectory_<date>_<time>_planned.csv` file.
4. Checks if first point in planned trajectory matches the current robot state (make sure to click Start State: `current` in RViz).
5. Ask user if path should get approximated with PTP or LIN Motion Primitives.
6. Approximate the path with motion primitives using Ramer-Douglas-Peucker Algorithm (RDP). For PTP in joint-space, for LIN in cartesian-space.
7. Save the reduced trajectory points to a `trajectory_<date>_<time>_reduced_<LIN_cartesian or PTP_joint>.csv` file.
8. Publish Goal-Poses of the Motion Primitives to `/visualization_marker_array` topic to visualize in RViz using MarkerArray
9. Ask user if planned primitives should get executed.
10. Execution using the [`motion_primitives_forward_controller`](https://github.com/b-robotized-forks/ros2_controllers/tree/motion_primitive_forward_controller/motion_primitives_forward_controller).
11. Save the executed trajectory to a `trajectory_<date>_<time>_executed.csv` file.

## Analyze saved data
Enter the filenames into the `compare.py` script and run it with:
```
ros2 run motion_primitives_from_planned_trajectory compare.py
```
This script generates two plots for trajectory analysis:
1. **Approximation of the Planned Trajectory Using Motion Primitives**  
   - **PTP Primitives**: These are visualized in joint space. The trajectory of each individual joint is plotted to show how the motion is approximated.
   - **LIN Primitives**: These are visualized in Cartesian space. The end-effector trajectory is shown in 3D, along with the target poses of each linear segment (LIN primitive).

2. **Comparison Between Planned and Executed Trajectories**  
   - The joint trajectories of both the planned and executed paths are plotted for each joint.
   - The **Root Mean Square Error (RMSE)** is computed for each joint to quantify the deviation between planned and executed movements.
