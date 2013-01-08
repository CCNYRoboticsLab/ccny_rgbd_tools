#ifndef CCNY_RGBD_MOTION_ESTIMATION_ICP_H
#define CCNY_RGBD_MOTION_ESTIMATION_ICP_H

#include <tf/transform_datatypes.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "ccny_rgbd/structures/feature_history.h"
#include "ccny_rgbd/registration/motion_estimation.h"


namespace ccny_rgbd
{

class MotionEstimationICP: public MotionEstimation
{
  typedef pcl::KdTreeFLANN<PointFeature> KdTree;

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MotionEstimationICP(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~MotionEstimationICP();

    bool getMotionEstimationImpl(
      RGBDFrame& frame,
      const tf::Transform& prediction,
      tf::Transform& motion);
  
  private:

    // **** ros-related
    
    ros::Publisher model_publisher_;
    
    // **** params
  
    std::string fixed_frame_; 
    std::string base_frame_;

    int max_iterations_;
    int min_correspondences_;
    double tf_epsilon_linear_;   
    double tf_epsilon_angular_;
    double max_corresp_dist_eucl_;
    
    // for visualization
    bool publish_model_;      
    
    // derived   
    double max_corresp_dist_eucl_sq_;  
    
    // **** variables

    PointCloudFeature::Ptr model_ptr_;
    KdTree model_tree_;

    FeatureHistory<PointFeature> feature_history_;

    tf::Transform f2b_; // Fixed frame to Base (moving) frame
    
    // **** functions
    
    bool alignICPEuclidean(
      const Vector3fVector& data_means,
      tf::Transform& correction);
    
    void getCorrespEuclidean(
      const PointCloudFeature& data_cloud,
      IntVector& data_indices,
      IntVector& model_indices);
    
    bool getNNEuclidean(
      const PointFeature& data_point,
      int& eucl_nn_idx, double& eucl_dist_sq);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_MOTION_ESTIMATION_ICP_H
