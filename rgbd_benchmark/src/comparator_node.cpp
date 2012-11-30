/*
 * comparator_node.cpp
 *
 *  Created on: Sep 13, 2011
 *      Author: carlos
 */

#include <rgbd_benchmark/comparator.h>

// %Tag(MAIN)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "comparator_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  rgbd_benchmark::Comparator benchmark(nh, nh_private);

  ros::spin();
}
// %EndTag(MAIN)%
