cmake_minimum_required(VERSION 3.5)
project(pal_gazebo_plugins)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(gazebo_ros REQUIRED)

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

install(TARGETS
  gazebo_underactuated_finger
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  )

install(DIRECTORY include/
   DESTINATION include
   FILES_MATCHING PATTERN "*.h"
)
