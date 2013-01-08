#ifndef RGBD_IMAGE_PROC_RGBD_IMAGE_PROC_NODELET_H
#define RGBD_IMAGE_PROC_RGBD_IMAGE_PROC_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "rgbd_image_proc/rgbd_image_proc.h"

namespace ccny_rgbd {

class RGBDImageProcNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    RGBDImageProc * rgbd_image_proc_;  // FIXME: change to smart pointer
};

} // namespace ccny_rgbd

#endif // RGBD_IMAGE_PROC_RGBD_IMAGE_PROC_NODELET_H
