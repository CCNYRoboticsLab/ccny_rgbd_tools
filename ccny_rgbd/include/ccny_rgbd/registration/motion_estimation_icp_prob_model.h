#ifndef CCNY_RGBD_MOTION_ESTIMATION_ICP_PROB_MODEL_H
#define CCNY_RGBD_MOTION_ESTIMATION_ICP_PROB_MODEL_H

#include <tf/transform_datatypes.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <visualization_msgs/Marker.h>
#include <pcl/io/pcd_io.h>

#include "ccny_rgbd/registration/motion_estimation.h"
#include "ccny_rgbd/structures/feature_history.h"
#include "ccny_rgbd/Save.h"

namespace ccny_rgbd
{

typedef std::vector<cv::Mat> MatVector;

class MotionEstimationICPProbModel: public MotionEstimation
{
  typedef ccny_rgbd::ICPKd<PointFeature, PointFeature> ICPReg;

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MotionEstimationICPProbModel(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~MotionEstimationICPProbModel();

    bool getMotionEstimationImpl(
      RGBDFrame& frame,
      const tf::Transform& prediction,
      tf::Transform& motion);
  
    int getModelSize() const { return model_size_; }

    bool saveSrvCallback(ccny_rgbd::Save::Request& request,
                         ccny_rgbd::Save::Response& response);

  private:

    // **** ros

    ros::Publisher model_publisher_;
    ros::Publisher covariances_publisher_;

    // **** params

    std::string fixed_frame_; 
    std::string base_frame_;

    double max_association_dist_;
    double alpha_;

    int max_model_size_;    // bound for how many features to store in the model
    
    // **** variables

    int model_idx_;         // current intex in the ring buffer
    int model_size_;        // current model size
    int n_nearest_neighbors_; // for searching for mah NN

    double max_association_dist_sq_;
    double max_association_dist_mah_;
    
    ICPReg reg_;
    PointCloudFeature::Ptr model_ptr_;   
    MatVector covariances_;
    MatVector means_;
    tf::Transform f2b_; // Fixed frame to Base (moving) frame

    ros::ServiceServer save_service_;

    // ***** funtions
    void publishCovariances();

    void getNNMahalanobis(
      const pcl::KdTreeFLANN<PointFeature>& model_tree,
      const cv::Mat& f_mean,
      const cv::Mat& f_cov,
      double& mah_dist,
      int& mah_nn_idx);

    void transformToRotationCV(const tf::Transform& transform,
                             cv::Mat& translation,
                             cv::Mat& rotation);

    void addToModel(const cv::Mat& feature_mean,
                    const cv::Mat& feature_cov);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_MOTION_ESTIMATION_ICP_PROB_MODEL_H

