/*
 * resize_img_to_bag_node.cpp
 *
 *  Created on: Apr 17, 2012
 *      Author: carlos
 */

#include <rgbd_bagger/rebagger.h>

// %Tag(MAIN)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rebagger_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  rgbd_bagger::Rebagger resize_rebag(nh, nh_private);

//  ros::spin();  // ros::spin  not needed if you want to exit the program automatically when finished
  return 0;
}
// %EndTag(MAIN)%
