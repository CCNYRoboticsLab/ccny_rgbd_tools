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

# services
rosbuild_gensrv()

# opencv
pkg_check_modules(OPENCV REQUIRED opencv)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

# boost
rosbuild_add_boost_directories()

# TORO
add_library(toro3d STATIC IMPORTED)
rosbuild_find_ros_package(ccny_toro)
set_property(TARGET toro3d PROPERTY IMPORTED_LOCATION ${ccny_toro_PACKAGE_PATH}/lib/libtoro3d.a)

################################################################
# Build libraries
################################################################

rosbuild_add_library (ccny_rgbd_structures
  src/structures/rgbd_frame.cpp
  src/structures/rgbd_keyframe.cpp
  src/structures/feature_history.cpp
)

rosbuild_add_library (ccny_rgbd_features
  src/features/feature_detector.cpp
  src/features/orb_detector.cpp
  src/features/gft_detector.cpp
  src/features/surf_detector.cpp
  src/features/star_detector.cpp
)

rosbuild_add_library (ccny_rgbd_registration
  src/registration/motion_estimation.cpp
  src/registration/motion_estimation_icp.cpp
  src/registration/motion_estimation_icp_prob_model.cpp
)

rosbuild_add_library (ccny_rgbd_mapping
  src/mapping/keyframe_loop_detector.cpp
  src/mapping/keyframe_loop_solver.cpp
  src/mapping/keyframe_loop_solver_toro.cpp
  src/mapping/keyframe_loop_solver_g2o.cpp
  src/mapping/octree_pointcloud_storage.cpp
)

rosbuild_add_library (ccny_rgbd_util
  src/rgbd_util.cpp
)

# will link using full path
target_link_libraries(ccny_rgbd_mapping toro3d) 
target_link_libraries(ccny_rgbd_mapping
  cholmod
  g2o_core
  g2o_solver_cholmod
  g2o_solver_csparse
  g2o_types_slam3d)

################################################################
# Build visual odometry application
################################################################

rosbuild_add_executable(visual_odometry_node 
  src/node/visual_odometry_node.cpp
  src/apps/visual_odometry.cpp)

target_link_libraries (visual_odometry_node 
  ccny_rgbd_structures
  ccny_rgbd_features
  ccny_rgbd_registration
  ccny_rgbd_mapping
  ccny_rgbd_util
  ${OPENCV_LIBRARIES}
  boost_signals 
  boost_system
)

################################################################
# Build keyframe mapper application
################################################################

rosbuild_add_executable(keyframe_mapper_node 
  src/node/keyframe_mapper_node.cpp
  src/apps/keyframe_mapper.cpp)

target_link_libraries (keyframe_mapper_node
  ccny_rgbd_structures 
  ccny_rgbd_mapping
  ccny_rgbd_util
  boost_signals 
  boost_system
)

################################################################
# Build octree mapper application
################################################################

rosbuild_add_executable(octree_mapper_node 
  src/node/octree_mapper_node.cpp
  src/apps/octree_mapper.cpp)

target_link_libraries (octree_mapper_node
  ccny_rgbd_structures 
  ccny_rgbd_mapping
  ccny_rgbd_util
  boost_signals 
  boost_system
)

################################################################
# Build logger applications
################################################################

rosbuild_add_executable(logger_node 
  src/node/logger_node.cpp
  src/apps/logger.cpp)

target_link_libraries (logger_node
  ccny_rgbd_structures 
  ccny_rgbd_util
  boost_signals 
  boost_system
  boost_filesystem
  ${OPENCV_LIBRARIES}
)

rosbuild_add_executable(avg_logger_node 
  src/node/avg_logger_node.cpp
  src/apps/avg_logger.cpp)

target_link_libraries (avg_logger_node
  ccny_rgbd_structures 
  ccny_rgbd_util
  boost_signals 
  boost_system
  boost_filesystem
  ${OPENCV_LIBRARIES}
)


