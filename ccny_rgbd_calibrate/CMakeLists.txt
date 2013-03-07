cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

# opencv
pkg_check_modules(OPENCV REQUIRED opencv)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

# gsl
pkg_check_modules(GSL REQUIRED gsl)
include_directories(${GSL_INCLUDE_DIRS})
link_directories(${GSL_LIBRARY_DIRS})

# boost
rosbuild_add_boost_directories()

rosbuild_add_library(ccny_rgbd_calibrate
  src/calib_util.cpp)

rosbuild_add_executable(extrinsics_calibrator_node 
  src/extrinsics_calibrator_node.cpp
  src/extrinsics_calibrator.cpp)
  
rosbuild_add_executable(depth_calibrator_node 
  src/depth_calibrator_node.cpp
  src/depth_calibrator.cpp)
  
target_link_libraries (depth_calibrator_node 
  ${OPENCV_LIBRARIES}
  ${GSL_LIBRARIES}
  ccny_rgbd_util
  ccny_rgbd_calibrate
  ccny_rgbd_proc_util
  boost_signals 
  boost_system
  boost_filesystem)

target_link_libraries (extrinsics_calibrator_node 
  ${OPENCV_LIBRARIES}
  ccny_rgbd_util
  ccny_rgbd_calibrate
  boost_signals 
  boost_system
  boost_filesystem)

################################################################
# Build logger applications
################################################################

rosbuild_add_executable(logger_node 
  src/logger_node.cpp
  src/logger.cpp)

target_link_libraries (logger_node
  ccny_rgbd_structures 
  ccny_rgbd_util
  boost_signals 
  boost_system
  boost_filesystem
  ${OPENCV_LIBRARIES}
)

rosbuild_add_executable(avg_logger_node 
  src/avg_logger_node.cpp
  src/avg_logger.cpp)

target_link_libraries (avg_logger_node
  ccny_rgbd_structures 
  ccny_rgbd_util
  boost_signals 
  boost_system
  boost_filesystem
  ${OPENCV_LIBRARIES}
)

################################################################
# Build comparator app
################################################################

rosbuild_add_executable(uncertainty_comparator_node 
  src/uncertainty_comparator_node.cpp
  src/uncertainty_comparator.cpp)
  
target_link_libraries (uncertainty_comparator_node
  ccny_rgbd_util
  boost_signals 
  boost_system
  boost_filesystem
  ${OPENCV_LIBRARIES})
  
  
  