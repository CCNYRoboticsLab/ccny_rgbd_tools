ccny_rgbd changelist
========================

current
------------------------
 * added new g2o interface (can use libg2o)
 * added camera path saving and loading in keyframe_mapper

0.2.0        (4/15/2013)
------------------------
 * seperated into ROS apps (ccny_rgbd) and stand-alone library (rgbdtools)
 * added seperate package for calibration and misc files: ccny_rgbd_data
 * added rviz config file for groovy
 * added range threshold parameters to keyframe_mapper
 * unadvertised cloud publishing topic from rgbd_image_proc if param is set to false. Otherwise, advertised.
 * removed ICP registration class (ICPProbModel the default and only option right now)
 * added flags for verbose output

0.1.1         (3/1/2013)
------------------------
 * some topic renames for consistency with other packges (odom -> vo)
 * added CHANGELIST
 * keyframe_mapper: lazy pointcloud building, reduces memory requirements considerably
 * rgbd_image_proc now supports non-VGA input
 * added Octomap exporting
 * added path publishing
 * added default calibration files for OpenNI devices
 * added supprt for 32FC1 images
 * bug fixes in manifests
 * bug fixes in linking

0.1.0        (2/12/2013)
------------------------
 * Initial release
