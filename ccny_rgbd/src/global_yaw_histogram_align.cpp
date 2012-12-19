#include "ccny_rgbd/rgbd_map_util.h"
#include "ccny_rgbd/types.h"

using namespace ccny_rgbd;

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    printf("error: usage is global_yaw_histogram_align [filename]\n");
    return -1;
  }
  
  // read in
  printf("Reading cloud\n");
  PointCloudT::Ptr cloud;
  cloud.reset(new PointCloudT());
  pcl::PCDReader reader;
  reader.read(argv[1], *cloud);
  
  ccny_rgbd::alignGlobalMap(cloud);
  return 0;
}
