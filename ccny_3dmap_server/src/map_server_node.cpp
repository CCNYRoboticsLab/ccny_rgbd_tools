#include "ccny_3dmap_server/map_server.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MapServer");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::MapServer ms(nh, nh_private);
  ros::spin();
  return 0;
}
