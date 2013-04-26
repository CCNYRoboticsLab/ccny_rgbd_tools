CCNY RGB-D tools 
===================================

Ivan Dryanovski  
<ivan.dryanovski@gmail.com>

Copyright (C) 2013, City University of New York  
CCNY Robotics Lab  
<http://robotics.ccny.cuny.edu/>
 
Overview
-----------------------------------

The stack contains ROS applications for visual odometry and mapping using RGB-D cameras. 
The applications are built on top of the [rgbdtools](https://github.com/ccny-ros-pkg/rgbdtools.git) library.

This code is at an experimental stage, and licensed under the GPLv3 license.

Installing
-----------------------------------

### From source ###

Create a directory where you want the package downloaded (ex. `~/ros`), 
and make sure it's added to your`$ROS_PACKAGE_PATH`.

If you don't have git installed, do so:

    sudo apt-get install git-core

Download the stack from our repository:

    git clone https://github.com/ccny-ros-pkg/ccny_rgbd_tools.git

Install any dependencies using rosdep.

    rosdep install ccny_rgbd_tools

Alternatively, you can manually install the dependencies by

    sudo apt-get install libsuitesparse-dev

Compile the stack:

    rosmake ccny_rgbd_tools

If you get an error compiling `ccny_g2o`, it might be because of an incompatible `g2o` installation. Try removing `libg2o`:
    
    sudo apt-get remove ros-fuerte-libg2o
    sudo apt-get remove ros-groovy-libg2o

Quick usage
-----------------------------------

Connect your RGB-D camera and launch the Openni device. The openni_launch file will 
start the driver and the processing nodelets.

    roslaunch ccny_openni_launch openni.launch 

For faster performace, consider using `dynamic reconfigure` to change the sampling rate of 
the `rgbd_image_proc` nodelet. For example, setting it to to 0.5 will downsample the images by a factor of 2.

Next, launch the visual odometry:

    roslaunch ccny_rgbd vo+mapping.launch

Finally, launch rviz. 

    rosrun rviz rviz

For convenience, you can load the `ccny_rgbd/launch/rviz.cfg` file.

References
-----------------------------------

If you use this system in your reasearch, please cite the following paper:

Ivan Dryanovski, Roberto G. Valenti, Jizhong Xiao. 
*Fast Visual Odometry and Mapping from RGB-D Data*. 
2013 International Conference on Robotics and Automation (ICRA2013).

More info
-----------------------------------

Documentation:

 * ROS wiki: http://ros.org/wiki/ccny_rgbd_tools
 * API: http://ros.org/doc/fuerte/api/ccny_rgbd/html/

Videos:
 * Visual odometry & 3D mapping: http://youtu.be/YE9eKgek5pI
 * Feature viewer: http://youtu.be/kNkrPuBu8JA
