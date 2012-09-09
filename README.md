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

Install any dependencies using [[rosdep]].

    rosdep install ccny_rgbd_tools

Compile the stack:

    rosmake ccny_rgbd_tools

Usage
-----------------------------------

Connect your RGB-D camera and launch the Openni device. 

    roslaunch ccny_openni_launch openni.launch 

Next, launch the visual odometry:

    roslaunch ccny_rgbd visual_odometry.launch

Finally, launch rviz. 

    rosrun rviz rviz

For convenience, you can load the ccny_rgbd/launch/rviz.cfg file.

More info
-----------------------------------

http://ros.org/wiki/ccny_rgbd_tools
