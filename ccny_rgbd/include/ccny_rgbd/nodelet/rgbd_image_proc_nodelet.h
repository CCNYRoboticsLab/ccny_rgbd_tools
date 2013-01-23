#ifndef CCNY_RGBD_RGBD_IMAGE_PROC_NODELET_H
#define CCNY_RGBD_RGBD_IMAGE_PROC_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "ccny_rgbd/apps/rgbd_image_proc.h"

namespace ccny_rgbd {

class RGBDImageProcNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    RGBDImageProc * rgbd_image_proc_;  // FIXME: change to smart pointer
};

} // namespace ccny_rgbd

#endif // CCNY_RGBD_RGBD_IMAGE_PROC_NODELET_H
