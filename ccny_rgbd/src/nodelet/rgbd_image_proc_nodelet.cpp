/*
 *  Copyright (C) 2013, City University of New York
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  CCNY Robotics Lab
 *  http://robotics.ccny.cuny.edu
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

#include "ccny_rgbd/nodelet/rgbd_image_proc_nodelet.h"

namespace ccny_rgbd {

PLUGINLIB_DECLARE_CLASS(ccny_rgbd, RGBDImageProcNodelet, RGBDImageProcNodelet, nodelet::Nodelet);

void RGBDImageProcNodelet::onInit()
{
  NODELET_INFO("Initializing RGBD Image Proc Nodelet");
  
  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh         = getMTNodeHandle();
  ros::NodeHandle nh_private = getMTPrivateNodeHandle();

  rgbd_image_proc_ = new RGBDImageProc(nh, nh_private);
}

} // namespace ccny_rgbd