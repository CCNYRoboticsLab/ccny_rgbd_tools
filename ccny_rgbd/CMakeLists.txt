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

# Generate services
rosbuild_gensrv()

# Generate dynamic parameters
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

# opencv
pkg_check_modules(OPENCV REQUIRED opencv)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

# Octomap
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

# boost
rosbuild_add_boost_directories()

################################################################
# Build libraries
################################################################

rosbuild_add_library (ccny_rgbd_util
  src/rgbd_util.cpp
)

rosbuild_add_library (ccny_rgbd_proc_util
  src/proc_util.cpp
)

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

target_link_libraries(ccny_rgbd_features
  ${OPENCV_LIBRARIES})

rosbuild_add_library (ccny_rgbd_registration
  src/registration/motion_estimation.cpp
  src/registration/motion_estimation_icp.cpp
  src/registration/motion_estimation_icp_prob_model.cpp
  src/registration/motion_estimation_ransac.cpp
)

target_link_libraries(ccny_rgbd_registration
  ccny_rgbd_util)

rosbuild_add_library (ccny_rgbd_mapping
  src/mapping/keyframe_graph_detector.cpp
  src/mapping/keyframe_graph_solver.cpp
  src/mapping/keyframe_graph_solver_g2o.cpp
)

target_link_libraries(ccny_rgbd_mapping
  ccny_rgbd_util
  boost_regex
  cholmod
  g2o_core
  g2o_stuff
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
  ccny_rgbd_util
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
  ${OCTOMAP_LIBRARIES}
  ccny_rgbd_structures 
  ccny_rgbd_mapping
  ccny_rgbd_util
  boost_signals 
  boost_system
)

################################################################
# Build feature viewer application
################################################################

rosbuild_add_executable(feature_viewer_node 
  src/node/feature_viewer_node.cpp
  src/apps/feature_viewer.cpp)

target_link_libraries (feature_viewer_node
  ccny_rgbd_structures 
  ccny_rgbd_features
  ccny_rgbd_util
  boost_signals 
  boost_system
)

################################################################
# Build image proc application
################################################################

rosbuild_add_library(rgbd_image_proc_app src/apps/rgbd_image_proc.cpp)
  
target_link_libraries (rgbd_image_proc_app
  ccny_rgbd_util
  ccny_rgbd_proc_util
  boost_signals 
  boost_system
  boost_filesystem
  ${OPENCV_LIBRARIES}
)
  
rosbuild_add_executable(rgbd_image_proc_node src/node/rgbd_image_proc_node.cpp)

rosbuild_add_library(rgbd_image_proc_nodelet src/nodelet/rgbd_image_proc_nodelet.cpp)
  
target_link_libraries(rgbd_image_proc_node    rgbd_image_proc_app)
target_link_libraries(rgbd_image_proc_nodelet rgbd_image_proc_app)

#supress pcl-1.6 pragma warnings
rosbuild_add_compile_flags(ccny_rgbd_structures '-Wno-unknown-pragmas')
rosbuild_add_compile_flags(ccny_rgbd_util '-Wno-unknown-pragmas')
rosbuild_add_compile_flags(ccny_rgbd_registration '-Wno-unknown-pragmas')
rosbuild_add_compile_flags(ccny_rgbd_mapping '-Wno-unknown-pragmas')
rosbuild_add_compile_flags(ccny_rgbd_features '-Wno-unknown-pragmas')
rosbuild_add_compile_flags(keyframe_mapper_node '-Wno-unknown-pragmas')
rosbuild_add_compile_flags(rgbd_image_proc_node '-Wno-unknown-pragmas')
rosbuild_add_compile_flags(rgbd_image_proc_nodelet '-Wno-unknown-pragmas')
rosbuild_add_compile_flags(visual_odometry_node '-Wno-unknown-pragmas')
rosbuild_add_compile_flags(feature_viewer_node  '-Wno-unknown-pragmas')
