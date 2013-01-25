/**
 *  @file motion_estimation_icp.h
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

#ifndef CCNY_RGBD_MOTION_ESTIMATION_ICP_H
#define CCNY_RGBD_MOTION_ESTIMATION_ICP_H

#include <tf/transform_datatypes.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "ccny_rgbd/types.h"
#include "ccny_rgbd/structures/feature_history.h"
#include "ccny_rgbd/registration/motion_estimation.h"

namespace ccny_rgbd {

  
/** @brief Frame-to-frame ICP motion estimation
 * 
 * The motion is estimated by aligning the current features
 * to the features from a number of previous frames.
 */  
class MotionEstimationICP: public MotionEstimation
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** @brief Constructor from ROS noehandles
     * @param nh the public nodehandle
     * @param nh_private the private notehandle
     */  
    MotionEstimationICP(const ros::NodeHandle& nh, 
                        const ros::NodeHandle& nh_private);
        
    /** @brief Default destructor
     */
    virtual ~MotionEstimationICP();

    /** @brief Main method for estimating the motion given an RGBD frame
     * @param frame the current RGBD frame
     * @param prediction the predicted motion (currently ignored)
     * @param motion the (output) incremental motion, wrt the fixed frame
     * @retval true the motion estimation was successful
     * @retval false the motion estimation failed
     */
    bool getMotionEstimationImpl(
      RGBDFrame& frame,
      const tf::Transform& prediction,
      tf::Transform& motion);
  
    /** @brief Returns the number of points in the model built from the feature buffer
     * @returns number of points in model
     */
    int getModelSize() const { return model_ptr_->points.size(); }
    
  private:

    // **** ros-related
    
    ros::Publisher model_publisher_;  ///< the ROS published for the model pointcloud
    
    // **** parameters
  
    std::string fixed_frame_;      ///< the fixed frame (typically odom)
    std::string base_frame_;       ///< the moving frame (typically base_link or camera_link)

    int max_iterations_;           ///< max ICP iterations
    int min_correspondences_;      ///< minimum correspondeces for ICP to continue
    double tf_epsilon_linear_;     ///< linear convergence criteria for ICP
    double tf_epsilon_angular_;    ///< angular convergence criteria for ICP
    double max_corresp_dist_eucl_; ///< maximum Euclidean correspondce distance for ICP
    
    bool publish_model_;           ///< if true, the model will be published 
    
    double max_corresp_dist_eucl_sq_; ///< max squared correspondce distance, derived
    
    // **** variables

    PointCloudFeature::Ptr model_ptr_; ///< the PointCloud which holds the aggregated history
    KdTree model_tree_;                ///< kdtree of model_ptr_

    FeatureHistory<PointFeature> feature_history_; ///< ring buffer of all the frames

    tf::Transform f2b_; ///< fixed frame to base (moving) frame
    
    /** @brief Performs ICP alignment using the Euclidean distance for corresopndences
     * @param data_means a vector of 3x1 matrices, repesenting the 3D positions of the features
     * @param correction reference to the resulting transformation
     * @retval true the motion estimation was successful
     * @retval false the motion estimation failed
     */
    bool alignICPEuclidean(
      const Vector3fVector& data_means,
      tf::Transform& correction);
    
    /** @brief Performs ICP alignment using the Euclidean distance for corresopndences
     * @param data_cloud a pointcloud of the 3D positions of the features
     * @param data_indices reference to a vector containting the resulting data indices
     * @param model_indices reference to a vector containting the resulting model indices
     */
    void getCorrespEuclidean(
      const PointCloudFeature& data_cloud,
      IntVector& data_indices,
      IntVector& model_indices);
    
    /** @brief Finds the nearest Euclidean neighbor
     * @param data_point the query 3D data point
     * @param eucl_nn_idx reference to the resulting nearest neigbor index in the model
     * @param eucl_dist_sq reference to the resulting squared distance
     * @retval true a neighbor was found
     * @retval false a neighbor was not found
     */
    bool getNNEuclidean(
      const PointFeature& data_point,
      int& eucl_nn_idx, double& eucl_dist_sq);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_MOTION_ESTIMATION_ICP_H
