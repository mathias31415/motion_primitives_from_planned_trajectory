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

# Include directory
include_directories(include)

# Executable for test_rdp
add_executable(test_rdp 
  src/test_rdp.cpp 
  src/rdp.cpp)
target_include_directories(test_rdp PRIVATE include)

# Executable for test_approx_primitives_with_rdp
add_executable(test_approx_primitives_with_rdp
  src/test_approx_primitives_with_rdp.cpp
  src/approx_primitives_with_rdp.cpp
  src/rdp.cpp
)
target_include_directories(test_approx_primitives_with_rdp PRIVATE include)
ament_target_dependencies(test_approx_primitives_with_rdp
  rclcpp
  geometry_msgs
  industrial_robot_motion_interfaces
)

# Executable for test_planned_trajectory_reader
add_executable(test_planned_trajectory_reader 
  src/test_planned_trajectory_reader.cpp
  src/planned_trajectory_reader.cpp
)
target_include_directories(test_planned_trajectory_reader PRIVATE include)
ament_target_dependencies(test_planned_trajectory_reader
  rclcpp 
  moveit_msgs
  trajectory_msgs
  geometry_msgs
)


# Executable for process_trajectory_to_motion_primitives
add_executable(process_trajectory_to_motion_primitives 
  src/process_trajectory_to_motion_primitives.cpp
  src/planned_trajectory_reader.cpp
  src/approx_primitives_with_rdp.cpp
  src/rdp.cpp
  src/fk_client.cpp
)
target_include_directories(process_trajectory_to_motion_primitives PRIVATE include)
ament_target_dependencies(process_trajectory_to_motion_primitives
  rclcpp 
  moveit_msgs
  trajectory_msgs
  geometry_msgs
  industrial_robot_motion_interfaces
)

# Installation
install(TARGETS
  test_rdp
  test_approx_primitives_with_rdp
  test_planned_trajectory_reader
  process_trajectory_to_motion_primitives
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
endif()

ament_package()
