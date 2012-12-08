CCNY RGB-D tools 
===================================

Ivan Dryanovski  
CCNY Robotics Lab 2011  
ivan.dryanovski@gmail.com  
http://robotics.ccny.cuny.edu/  

Overview
-----------------------------------

The stack contains a tools for visual odometry and mapping using RGB-D cameras. 

This code is at an experimental stage. 

Installing
-----------------------------------

### From source ###

Create a directory where you want the package downloaded (ex. `~/ros`), 
and add it to `$ROS_PACKAGE_PATH`.

Make sure you have git installed:

    sudo apt-get install git-core

Download the stack from our repository:

    git clone https://github.com/ccny-ros-pkg/ccny_rgbd_tools.git

Install any dependencies using rosdep.

    rosdep install ccny_rgbd_tools

Compile the stack:

    rosmake ccny_rgbd_tools

Usage
-----------------------------------

Connect your RGB-D camera and launch the Openni device. 

    roslaunch ccny_openni_launch openni.launch 

For best performace, consider using `dynamic reconfigure` to set the 
resolution to QVGA, especially if using a slower machine.

Next, launch the visual odometry:

    roslaunch ccny_rgbd vo+mapping.launch reg_type:=ICPProbModel

Finally, launch rviz. 

    rosrun rviz rviz

For convenience, you can load the `ccny_rgbd/launch/rviz.cfg` file.

Configuration
----------------------------------

There are many paramters - the first ones you can try changing are:
 - resolution of the OpenNI camera, through `dynamic reconfigure`. 
   QVGA is recommended, VGA is the default
 - in `ccny_rgbd/launch/visual_odometry.launch`: `feature/GFT/n_features`: 
   the number of features to detect in each image. Default is 150, hhigher numbers
   (try up to 500) might give more robust tracking)

More info
-----------------------------------

Some videos:



http://ros.org/wiki/ccny_rgbd_tools
