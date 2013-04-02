cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# add dynamic reconfigure api for hokuyo
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
 pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


rosbuild_add_library(PalModelPlugin  src/PalModelPlugin.cpp)
target_link_libraries(PalModelPlugin ${GAZEBO_LIBRARIES})

rosbuild_add_library(FTPlugin  src/gazebo_ros_forcetorque.cpp)
target_link_libraries(FTPlugin ${GAZEBO_LIBRARIES})

rosbuild_add_library(gazebo_ros_range  src/gazebo_ros_range.cpp)
target_link_libraries(gazebo_ros_range RayPlugin)

rosbuild_add_library(gazebo_pal_ros_diffdrive src/gazebo_ros_diffdrive.cpp)
target_link_libraries(gazebo_pal_ros_diffdrive ${GAZEBO_LIBRARIES})

rosbuild_add_library(gazebo_ros_openni_kinect src/gazebo_ros_openni_kinect.cpp)
target_link_libraries(gazebo_ros_openni_kinect gazebo_ros_camera_utils DepthCameraPlugin)
rosbuild_link_boost(gazebo_ros_openni_kinect thread)

