cmake_minimum_required(VERSION 3.8)
project(motion_primitives_from_planned_trajectory)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Include directory
include_directories(include)

# Executable for test_rdp
add_executable(test_rdp src/test_rdp.cpp src/rdp.cpp)
target_include_directories(test_rdp PRIVATE include)

# Installation
install(TARGETS test_rdp
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
endif()

ament_package()
