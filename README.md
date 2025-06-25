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

## Process planned trajectory
Start the python script with the following command:
```
ros2 run motion_primitives_from_planned_trajectory process_trajectory_to_motion_primitives
```
Then plan a trajectory in RViz with MoveIt by pressing `plan`. The python script will:
    1. Read the planned trajectory from `/display_planned_path`.
    2. Calculate the endefector pose for every point in the trajectory using the `/compute_fk` service.
    3. Save the trajectory and endefector poses to a `trajectory_<date>_<time>_planned.csv` file.
    4. Ask user if path should get approximated with PTP or LIN Motion Primitives.
    5. Approximate the path with motion primitives using using Ramer-Douglas-Peucker Algorithm (RDP). For PTP in joint-space, for LIN in cartesian-space.
    6. ~~Plot the trajectory vs the aproximated motion primitives path.~~
    7. Publish Goal-Poses of the Motion Primitives to `/visualization_marker_array` topic to visualize in RViz using MarkerArray
    8. Ask user if planned primitives should get executed.
    9. Execution using the [`motion_primitives_forward_controller`](https://github.com/b-robotized-forks/ros2_controllers/tree/motion_primitive_forward_controller/motion_primitives_forward_controller).
    10. Save the executed trajectory to a `trajectory_<date>_<time>_executed.csv` file.
    11. ~~Compare planned and executed joint-trajectory~~


## Plot saved planned trajectory
```
ros2 run motion_primitives_from_planned_trajectory plot_saved_planned_trajectory "<filename>"
```
(The filename can either be passed as a command line argument or specified directly in the Python script.)

## Plot saved executed trajectory
```
ros2 run motion_primitives_from_planned_trajectory plot_saved_executed_trajectory "<filename>"
```
(The filename can either be passed as a command line argument or specified directly in the Python script.)

## Compare planned and executed trajectory
```
ros2 run motion_primitives_from_planned_trajectory compare_planned_and_executed_traj
```
