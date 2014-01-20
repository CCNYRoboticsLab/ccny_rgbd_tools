/**
 *  @file motion_estimation_icp_prob_model.h
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

#ifndef RGBDTOOLS_MOTION_ESTIMATION_ICP_PROB_MODEL_H
#define RGBDTOOLS_MOTION_ESTIMATION_ICP_PROB_MODEL_H

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include "rgbdtools/types.h"
#include "rgbdtools/registration/motion_estimation.h"

namespace rgbdtools {

/** @brief Motion estimation based on aligning sparse features
 * against a persistent, dynamic model.
 * 
 * The model is build from incoming features through a Kalman Filter
 * update step.
 */  
class MotionEstimationICPProbModel: public MotionEstimation
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** @brief Constructor from ROS noehandles
     * @param nh the public nodehandle
     * @param nh_private the private notehandle
     */
    MotionEstimationICPProbModel();
    
    /** @brief Default destructor
     */    
    virtual ~MotionEstimationICPProbModel();

    /** @brief Main method for estimating the motion given an RGBD frame
     * @param frame the current RGBD frame
     * @param prediction the predicted motion (currently ignored)
     * @param motion the (output) incremental motion, wrt the fixed frame
     * @retval true the motion estimation was successful
     * @retval false the motion estimation failed
     */
    bool getMotionEstimationImpl(
      RGBDFrame& frame,
      const AffineTransform& prediction,
      AffineTransform& motion);
  
    /** @brief Returns the number of points in the model built from the feature buffer
     * @returns number of points in model
     */
    int getModelSize() const { return model_size_; }

    PointCloudFeature::Ptr getModel()  { return model_ptr_; }
    Vector3fVector* getMeans()  { return &means_; }
    Matrix3fVector* getCovariances()  { return &covariances_; }
        
    void setMaxIterations(int max_iterations);
    void setMinCorrespondences(int min_correspondences);
    void setNNearestNeighbors(int n_nearest_neighbors);
    void setMaxModelSize(int max_model_size);
    void setTfEpsilonLinear(double tf_epsilon_linear);
    void setTfEpsilonAngular(double tf_epsilon_angular);     
    void setMaxAssociationDistMahalanobis(double max_assoc_dist_mah);
    void setMaxCorrespondenceDistEuclidean(double max_corresp_dist_eucl);
    
  private:

    // **** params

    int max_iterations_;        ///< Maximum number of ICP iterations
    int min_correspondences_;   ///< Minimum number of correspondences for ICP to contuinue
    
    /** @brief How many euclidean neighbors to go through, in a brute force
     * search of the closest Mahalanobis neighbor.
     */
    int n_nearest_neighbors_;  
    
    /** @brief Upper bound for how many features to store in the model.
     * 
     * New features added beyond thi spoint will overwrite old features
     */
    int max_model_size_;

    double tf_epsilon_linear_;     ///< Linear convergence criteria for ICP
    double tf_epsilon_angular_;    ///< Angular convergence criteria for ICP

    /** @brief Maximum Mahalanobis distance for associating points
     * between the data and the model
     */
    double max_assoc_dist_mah_;    

    /** @brief Maximum Euclidean correspondce distance for ICP
     */
    double max_corresp_dist_eucl_; 
    

    /** @brief Maximum squared Mahalanobis distance for associating points
     * between the data and the model, derived.
     */
    double max_assoc_dist_mah_sq_;    
    
    /** @brief Maximum Euclidean correspondce distance for ICP, derived
     */
    double max_corresp_dist_eucl_sq_;

    // **** variables

    PointCloudFeature::Ptr model_ptr_; ///< The model point cloud
    int model_idx_;         ///< Current intex in the ring buffer
    int model_size_;        ///< Current model size
    Vector3fVector means_;  ///< Vector of model feature mean
    Matrix3fVector covariances_; ///< Vector of model feature covariances

    KdTree model_tree_;     ///< Kdtree of model_ptr_

    Matrix3f I_;            ///< 3x3 Identity matrix
    
    AffineTransform f2b_; ///< Transform from fixed to moving frame
    
    // ***** funtions
  
    /** @brief Performs ICP alignment using the Euclidean distance for corresopndences
     * @param data_means a vector of 3x1 matrices, repesenting the 3D positions of the features
     * @param correction reference to the resulting transformation
     * @retval true the motion estimation was successful
     * @retval false the motion estimation failed
     */
    bool alignICPEuclidean(
      const Vector3fVector& data_means,
      AffineTransform& correction);

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

    /** @brief Finds the nearest Mahalanobis neighbor
     * 
     * Requests the K nearest Euclidean neighbors (K = n_nearest_neighbors_)
     * using a kdtree, and performs a brute force search for the closest
     * Mahalanobis nighbor. Reasonable values for K are 4 or 8.
     * 
     * @param data_mean 3x1 matrix of the query 3D data point mean
     * @param data_cov 3x3 matrix of the query 3D data point covariance
     * @param mah_nn_idx reference to the resulting nearest neigbor index in the model
     * @param mah_dist_sq reference to the resulting squared Mahalanobis distance
     * @param indices cache vector, pre-allocated with n_nearest_neighbors_ size
     * @param dists_sq cache vector, pre-allocated with n_nearest_neighbors_ size
     * @retval true a neighbor was found
     * @retval false a neighbor was not found
     */
    bool getNNMahalanobis(
      const Vector3f& data_mean, const Matrix3f& data_cov,
      int& mah_nn_idx, double& mah_dist_sq,
      IntVector& indices, FloatVector& dists_sq);

    /** @brief Initializes the (empty) model from a set of data means and
     * covariances
     * @param data_means vector of 3x1 data point means
     * @param data_covariances vector of 3x3 data point covariances
     */
    void initializeModelFromData(
      const Vector3fVector& data_means,
      const Matrix3fVector& data_covariances);
    
    /** @brief Updates the (non-empty) model from a set of data means and
     * covariances
     * 
     * Any data points which have a Mahalanobis neighbor in the model
     * under a certain threshold distance get updated using a Kalman filter. 
     * 
     * Any data points without a Mahalanobis neighbor get insterted as new 
     * model points.
     * 
     * @param data_means vector of 3x1 data point means
     * @param data_covariances vector of 3x3 data point covariances
     */
    void updateModelFromData(const Vector3fVector& data_means,
                             const Matrix3fVector& data_covariances);
  
    /** @brief Adds a new point to the model
     * 
     * The model is implemented using a rign buffer. If the number of 
     * points in the model has reached the maximum, the new point will 
     * overwrite the oldest model point
     * 
     * @param data_mean 3x1 data point means
     * @param data_cov 3x3 data point covariances
     */
    void addToModel(
      const Vector3f& data_mean,
      const Matrix3f& data_cov);

    bool saveModel(const std::string& filename);
};

} // namespace rgbdtools

#endif // RGBDTOOLS_MOTION_ESTIMATION_ICP_PROB_MODEL_H

