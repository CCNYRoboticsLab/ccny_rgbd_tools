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

## opencv

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

pkg_check_modules(OPENCV REQUIRED opencv)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

# services
rosbuild_gensrv()

## boost

rosbuild_add_boost_directories()

################################################################################

rosbuild_add_executable(rgbd_vo_node 
                          src/node/rgbd_vo_node.cpp
                          src/structures/rgbd_frame.cpp
                          src/structures/rgbd_keyframe.cpp
                          src/structures/feature_history.cpp
                          src/features/feature_detector.cpp
                          src/features/orb_detector.cpp
                          src/features/gft_detector.cpp
                          src/features/klt_detector.cpp
                          src/features/surf_detector.cpp
                          src/features/fast_detector.cpp
                          src/features/sift_gpu_detector.cpp
                          src/registration/rgbd_vo.cpp
                          src/registration/motion_estimation.cpp
                          src/registration/motion_estimation_icp.cpp
                          src/registration/motion_estimation_icp_model.cpp
                          src/registration/motion_estimation_icp_prob_model.cpp
                          #src/registration/motion_estimation_icp_loop.cpp
                          #src/registration/motion_estimation_icp_particles.cpp
                          src/registration/motion_estimation_ransac.cpp
                          #src/registration/motion_estimation_ransac_model.cpp
                          src/registration/motion_prediction_imu.cpp
                          src/registration/icp_kd.cpp
                          src/loop/keyframe_generator.cpp
                          src/loop/loop_detection.cpp
                          src/loop/loop_solver.cpp
                          src/loop/loop_solver_sba.cpp
                          src/loop/loop_solver_gicp.cpp
                          src/rgbd_util.cpp)

target_link_libraries (rgbd_vo_node ccny_gicp) 
target_link_libraries (rgbd_vo_node ${OPENCV_LIBRARIES}) 
target_link_libraries (rgbd_vo_node boost_signals boost_system)

