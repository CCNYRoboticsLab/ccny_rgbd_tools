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

#ifndef CCNY_RGBD_KEYFRAME_GRAPH_SOLVER_H
#define CCNY_RGBD_KEYFRAME_GRAPH_SOLVER_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include "ccny_rgbd/structures/rgbd_keyframe.h"
#include "ccny_rgbd/structures/keyframe_association.h"

namespace ccny_rgbd
{

class KeyframeGraphSolver
{
  public:

    KeyframeGraphSolver(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~KeyframeGraphSolver();
 
    virtual void solve(
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations) = 0;

  protected:
 
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
   
  private:

};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_GRAPH_SOLVER_H
