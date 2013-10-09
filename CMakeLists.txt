cmake_minimum_required(VERSION 2.8.3)
project(pal_gazebo_plugins)

find_package(catkin REQUIRED)

# Depend on system install of Gazebo and SDFormat
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${catkin_INCLUDE_DIRS}
                            ${GAZEBO_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIRS}
                            ${SDFormat_INCLUDE_DIRS})

catkin_package(
  LIBRARIES gazebo_ros_range
            gazebo_ros_forcetorque
            pal_model_plugin
  DEPENDS
    gazebo
  )

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(pal_model_plugin src/PalModelPlugin.cpp)
target_link_libraries(pal_model_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

## Plugins
add_library(gazebo_ros_range src/gazebo_ros_range.cpp)
target_link_libraries(gazebo_ros_range ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDFormat_LIBRARIES} RayPlugin)

add_library(gazebo_ros_forcetorque src/gazebo_ros_forcetorque.cpp)
target_link_libraries(gazebo_ros_forcetorque ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  gazebo_ros_range
  gazebo_ros_forcetorque
  pal_model_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(PROGRAMS scripts/run_gazebo scripts/run_gzfactory
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
