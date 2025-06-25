cmake_minimum_required(VERSION 3.8)
project(motion_primitives_from_planned_trajectory)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(industrial_robot_motion_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directory
include_directories(include)

# Executable for process_trajectory_to_motion_primitives
add_executable(process_trajectory_to_motion_primitives 
  src/process_trajectory_to_motion_primitives.cpp
  src/planned_trajectory_reader.cpp
  src/approx_primitives_with_rdp.cpp
  src/rdp.cpp
  src/fk_client.cpp
  src/pose_marker_visualizer.cpp
)
target_include_directories(process_trajectory_to_motion_primitives PRIVATE include)
ament_target_dependencies(process_trajectory_to_motion_primitives
  rclcpp 
  moveit_msgs
  trajectory_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  industrial_robot_motion_interfaces
)

# Installation
install(TARGETS
  process_trajectory_to_motion_primitives
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
endif()

ament_package()
