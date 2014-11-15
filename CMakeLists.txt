cmake_minimum_required(VERSION 2.8.3)
project(pal_gazebo_plugins)

find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    std_srvs
    tf
    pal_multirobot_msgs
    roscpp
    control_toolbox)

# Depend on system install of Gazebo and SDFormat
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${catkin_INCLUDE_DIRS}
                            ${GAZEBO_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIRS}
                            ${SDFormat_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_ros_range
            gazebo_ros_forcetorque
            gazebo_pal_hand
            gazebo_wifi_ap
  CATKIN_DEPENDS
    std_msgs
    std_srvs
    tf
    pal_multirobot_msgs
    roscpp
    control_toolbox
  DEPENDS gazebo)

link_directories(${GAZEBO_LIBRARY_DIRS})

## Plugins
add_library(gazebo_ros_range src/gazebo_ros_range.cpp)
target_link_libraries(gazebo_ros_range ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDFormat_LIBRARIES} RayPlugin)

add_library(gazebo_ros_forcetorque src/gazebo_ros_forcetorque.cpp)
target_link_libraries(gazebo_ros_forcetorque ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_pal_hand src/gazebo_pal_hand.cpp)
target_link_libraries(gazebo_pal_hand ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_wifi_ap src/gazebo_wifi_ap.cpp)
target_link_libraries(gazebo_wifi_ap ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(gazebo_wifi_ap pal_multirobot_msgs_generate_messages_cpp)

add_library(gazebo_harness src/gazebo_harness.cpp)
target_link_libraries(gazebo_harness ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})


install(TARGETS
  gazebo_ros_range
  gazebo_ros_forcetorque
  gazebo_pal_hand
  gazebo_wifi_ap
  gazebo_harness
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
