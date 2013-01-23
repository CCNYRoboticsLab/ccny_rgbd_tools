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

#ifndef CCNY_RGBD_MOTION_ESTIMATION_H
#define CCNY_RGBD_MOTION_ESTIMATION_H

#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>

#include "ccny_rgbd/structures/rgbd_frame.h"

namespace ccny_rgbd
{

class MotionEstimation
{
  typedef nav_msgs::Odometry OdomMsg;

  enum MotionConstraint {NONE = 0, ROLL_PITCH = 1, ROLL_PITCH_Z = 2};

  public:

    MotionEstimation(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~MotionEstimation();

    tf::Transform getMotionEstimation(RGBDFrame& frame);

    void setBaseToCameraTf(const tf::Transform& b2c);

    virtual int getModelSize() const { return 0; }

  protected:
  
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    tf::Transform b2c_; // Base (moving) frame to Camera-optical frame

    int min_feature_count_;
    int motion_constraint_;

    virtual bool getMotionEstimationImpl(
      RGBDFrame& frame,
      const tf::Transform& prediction,
      tf::Transform& motion) = 0;

    void constrainMotion(tf::Transform& motion);

  private:

};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_MOTION_ESTIMATION_H
