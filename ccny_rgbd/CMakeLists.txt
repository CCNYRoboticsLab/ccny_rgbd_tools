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

# opencv

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

pkg_check_modules(OPENCV REQUIRED opencv)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

# services
rosbuild_gensrv()

# boost
rosbuild_add_boost_directories()

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
)

rosbuild_add_library (ccny_rgbd_registration
  src/registration/motion_estimation.cpp
  src/registration/motion_estimation_icp.cpp
  src/registration/motion_estimation_icp_prob_model.cpp
  src/registration/motion_prediction_imu.cpp
)

rosbuild_add_library (ccny_rgbd_mapping
  src/mapping/keyframe_mapper.cpp
)

rosbuild_add_library (ccny_rgbd_util
  src/rgbd_util.cpp
)

################################################################
# Build visual odometry application
################################################################

rosbuild_add_executable(visual_odometry_node 
  src/node/visual_odometry_node.cpp
  src/apps/visual_odometry.cpp)

target_link_libraries (visual_odometry_node ccny_rgbd_structures) 
target_link_libraries (visual_odometry_node ccny_rgbd_features) 
target_link_libraries (visual_odometry_node ccny_rgbd_registration) 
target_link_libraries (visual_odometry_node ccny_rgbd_mapping) 
target_link_libraries (visual_odometry_node ccny_rgbd_util) 

target_link_libraries (visual_odometry_node ccny_gicp) 
target_link_libraries (visual_odometry_node ${OPENCV_LIBRARIES}) 
target_link_libraries (visual_odometry_node boost_signals boost_system)

