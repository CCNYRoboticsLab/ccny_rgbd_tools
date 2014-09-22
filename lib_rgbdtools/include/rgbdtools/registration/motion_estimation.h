/**
 *  @file motion_estimation.h
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

#ifndef RGBDTOOLS_MOTION_ESTIMATION_H
#define RGBDTOOLS_MOTION_ESTIMATION_H

#include <boost/thread/mutex.hpp>

#include "rgbdtools/rgbd_frame.h"
#include "rgbdtools/types.h"

namespace rgbdtools {

/** @brief Base class for visual odometry motion estimation methods
 * 
 * The motion is estimated in increments of the change of pose
 * of the base frame. The increments are expressed wrt fixed frame.
 */  
class MotionEstimation
{
  public:

    enum MotionConstraint {NONE = 0, ROLL_PITCH = 1, ROLL_PITCH_Z = 2};
    
    /** @brief Constructor from ROS nodehandles
     * @param nh the public nodehandle
     * @param nh_private the private nodehandle
     */    
    MotionEstimation();
    
    /** @brief Default destructor
     */    
    virtual ~MotionEstimation();

    /** @brief Main function for estimating motion
     * 
     * The motion is equal to the change of pose of the base frame, and is
     * expressed wrt the fixed frame
     *   
     * Pose_new = motion * Pose_old;
     * 
     * @param frame The RGBD Frame for which the motion is estimated
     * @return incremental motion transform
     */    
    AffineTransform getMotionEstimation(RGBDFrame& frame);

    /** @brief Set the transformation between the base and camera frames.
     * @param b2c The transform from the base frame to the camera frame,
     *        expressed wrt the base frame.
     */    
    void setBaseToCameraTf(const AffineTransform& b2c);

    void setMotionConstraint(int motion_constraint); 

    /** @brief Return the size of the internal model. Overriden for classes
     * that use a model.
     * 
     * @return the size of the model
     */    
    virtual int getModelSize() const { return 0; }

  protected:
  
    AffineTransform b2c_; ///< Base (moving) frame to Camera-optical frame

    int motion_constraint_;   ///< The motion constraint type

    /** @brief Implementation of the motion estimation algorithm.
     * @param frame the current RGBD frame
     * @param prediction the motion prediction (currently ignored)
     * @param motion the output motion
     * @retval true the motion estimation was successful
     * @retval false the motion estimation failed
     */
    virtual bool getMotionEstimationImpl(
      RGBDFrame& frame,
      const AffineTransform& prediction,
      AffineTransform& motion) = 0;

    /** @brief Constrains the motion in accordance to the class motion constraints
     * 
     * This method can be called by the inheriting classes if desired. 
     * 
     * @param motion the incremental motion which is constrained.
     */
    void constrainMotion(AffineTransform& motion);
};

} // namespace rgbdtools

#endif // RGBDTOOLS_MOTION_ESTIMATION_H
