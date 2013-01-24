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

#include "ccny_rgbd/registration/motion_estimation.h"

namespace ccny_rgbd
{

MotionEstimation::MotionEstimation(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{
  // params

  if (!nh_private_.getParam ("reg/min_feature_count", min_feature_count_))
    min_feature_count_ = 15;
  if (!nh_private_.getParam ("reg/motion_constraint", motion_constraint_ ))
    motion_constraint_  = 0;
}

MotionEstimation::~MotionEstimation()
{

}

tf::Transform MotionEstimation::getMotionEstimation(RGBDFrame& frame)
{
  // motion prediction 
  /// @todo: motion prediction disabled for now
  tf::Transform prediction;
  prediction.setIdentity();

  tf::Transform motion;
  bool result;

  if (frame.n_valid_keypoints < min_feature_count_)
  {
    ROS_WARN("Not enough features (%d detected, min is %d)", 
      frame.n_valid_keypoints, min_feature_count_);
    result = false;
  }
  else
  {
    result = getMotionEstimationImpl(frame, prediction, motion);
  }

  if (!result)
  {
    ROS_WARN("Could not estimate motion from RGBD data, using Identity transform");
    motion.setIdentity();
  }

  return motion;
}

void MotionEstimation::constrainMotion(tf::Transform& motion)
{
  // **** degree-of-freedom constraints

  if (motion_constraint_ == ROLL_PITCH)
  {
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, tf::getYaw(motion.getRotation()));

    motion.setRotation(q); 
  }
  else if (motion_constraint_ == ROLL_PITCH_Z)
  {
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, tf::getYaw(motion.getRotation()));
    
    tf::Vector3 p(motion.getOrigin().getX(), motion.getOrigin().getY(), 0.0);
    
    motion.setOrigin(p);
    motion.setRotation(q); 
  }

}

void MotionEstimation::setBaseToCameraTf(const tf::Transform& b2c)
{
  b2c_ = b2c;
}

} // namespace ccny_rgbd
