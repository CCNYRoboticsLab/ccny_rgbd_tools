cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE MinSizeRel)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package (PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rosbuild_add_library (ccny_gicp src/bfgs_funcs.cpp
                                src/gicp.cpp
                                src/gicp_align.cpp
                                src/optimize.cpp
                                src/scan.cpp 
                                src/transform.cpp 
                                src/scan2ascii.cpp
                                src/octree_pointcloud_storage)

target_link_libraries (ccny_gicp gsl gslcblas ${PCL_LIBRARIES})
target_link_libraries (ccny_gicp boost_program_options boost_system stdc++)


rosbuild_add_compile_flags(ccny_gicp -fpermissive)

rosbuild_add_compile_flags(ccny_gicp -fpermissive)

rosbuild_add_executable(test_gicp src/test_gicp.cpp)
target_link_libraries (test_gicp ccny_gicp)
target_link_libraries (test_gicp boost_program_options boost_system stdc++)

rosbuild_link_boost (ccny_gicp program_options) 
