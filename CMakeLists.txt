cmake_minimum_required(VERSION 3.5)
project(pal_gazebo_plugins)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Depend on system install of Gazebo and SDFormat
find_package(gazebo REQUIRED)

include_directories(include)
include_directories(
  SYSTEM
    ${control_toolbox_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${SDFormat_INCLUDE_DIRS})


# Remove wrongly placed link flag in GAZEBO_LIBRARIES
string(REPLACE "-lpthread;" "" GAZEBO_LIBRARIES "${GAZEBO_LIBRARIES}")

link_directories(${GAZEBO_LIBRARY_DIRS})

## Plugins

 add_library(gazebo_underactuated_finger SHARED src/gazebo_underactuated_finger.cpp)
 target_link_libraries(gazebo_underactuated_finger ${GAZEBO_LIBRARIES})
 ament_target_dependencies(gazebo_underactuated_finger
   rclcpp
   control_toolbox
 )

 add_library(pal_gazebo_collisions SHARED src/pal_gazebo_collisions.cpp)
 target_link_libraries(pal_gazebo_collisions ${GAZEBO_LIBRARIES})
 ament_target_dependencies(pal_gazebo_collisions
   rclcpp
   gazebo_msgs
 )

 add_library(gazebo_world_odometry SHARED src/gazebo_world_odometry.cpp)
 target_link_libraries(gazebo_world_odometry ${GAZEBO_LIBRARIES})
 ament_target_dependencies(gazebo_world_odometry
   rclcpp
   nav_msgs
   tf2_geometry_msgs
 )

install(TARGETS
   gazebo_underactuated_finger
   pal_gazebo_collisions
   gazebo_world_odometry
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Add your tests here
endif()

ament_package()
