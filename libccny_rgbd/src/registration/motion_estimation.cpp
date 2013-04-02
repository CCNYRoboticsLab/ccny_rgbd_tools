/**
 *  @file motion_estimation.cpp
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

#include "ccny_rgbd/registration/motion_estimation.h"

namespace ccny_rgbd {

MotionEstimation::MotionEstimation()
{

}

MotionEstimation::~MotionEstimation()
{

}

AffineTransform MotionEstimation::getMotionEstimation(RGBDFrame& frame)
{
  ///@todo this should return a covariance
  
  // motion prediction 
  /// @todo motion prediction disabled for now
  AffineTransform prediction;
  prediction.setIdentity();

  AffineTransform motion;
  bool result;

  if (frame.n_valid_keypoints == 0)
  {
    ROS_WARN("No features detected.");
    result = false;
  }
  else
  {
    result = getMotionEstimationImpl(frame, prediction, motion);
  }

  if (!result)
  {
    ROS_WARN("Could not estimate motion from RGBD data, using Identity transform.");
    motion.setIdentity();
  }

  return motion;
}

void MotionEstimation::constrainMotion(AffineTransform& motion)
{
  // **** degree-of-freedom constraints

  if (motion_constraint_ == ROLL_PITCH)
  {
    float x, y, z, roll, pitch, yaw;
    eigenAffineToXYZRPY(motion, x, y, z, roll, pitch, yaw);
    XYZRPYToEigenAffine(x, y, z, 0, 0, yaw, motion);
  }
  else if (motion_constraint_ == ROLL_PITCH_Z)
  {
    float x, y, z, roll, pitch, yaw;
    eigenAffineToXYZRPY(motion, x, y, z, roll, pitch, yaw);
    XYZRPYToEigenAffine(x, y, 0, 0, 0, yaw, motion);
  }
}

void MotionEstimation::setBaseToCameraTf(const AffineTransform& b2c)
{
  b2c_ = b2c;
}

void MotionEstimation::setMotionContstraint(int motion_constraint)
{
  motion_constraint_ = motion_constraint;
}

} // namespace ccny_rgbd
