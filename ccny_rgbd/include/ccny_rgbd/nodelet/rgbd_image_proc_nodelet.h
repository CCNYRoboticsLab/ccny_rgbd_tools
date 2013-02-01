/**
 *  @file rgbd_image_proc_nodelet.h
 *  @author Ivan Dryanovski <ivan.dryanovski@gmail.com>
 * 
 *  @section LICENSE
 * 
 *  Copyright (C) 2013, City University of New York
 *  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CCNY_RGBD_RGBD_IMAGE_PROC_NODELET_H
#define CCNY_RGBD_RGBD_IMAGE_PROC_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "ccny_rgbd/apps/rgbd_image_proc.h"

namespace ccny_rgbd {

/** @brief Nodelet driver for the RGBDImageProc class.
 */  
class RGBDImageProcNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    RGBDImageProc * rgbd_image_proc_;  // FIXME: change to smart pointer
};

} // namespace ccny_rgbd

#endif // CCNY_RGBD_RGBD_IMAGE_PROC_NODELET_H
