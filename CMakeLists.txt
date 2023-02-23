cmake_minimum_required(VERSION 3.5)
project(pal_gazebo_plugins)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## Plugins
ament_auto_add_library(gazebo_underactuated_finger SHARED src/gazebo_underactuated_finger.cpp)
ament_auto_add_library(pal_gazebo_collisions SHARED src/pal_gazebo_collisions.cpp)
ament_auto_add_library(gazebo_world_odometry SHARED src/gazebo_world_odometry.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Add your tests here
endif()

ament_auto_package()
