#include "rgbd_image_proc/rgbd_image_proc_nodelet.h"

namespace ccny_rgbd {

PLUGINLIB_DECLARE_CLASS(rgbd_image_proc, RGBDImageProcNodelet, RGBDImageProcNodelet, nodelet::Nodelet);

void RGBDImageProcNodelet::onInit()
{
  NODELET_INFO("Initializing RGBD Image Proc Nodelet");
  
  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh         = getMTNodeHandle();
  ros::NodeHandle nh_private = getMTPrivateNodeHandle();

  rgbd_image_proc_ = new RGBDImageProc(nh, nh_private);
}

} // namespace ccny_rgbd