#include "ccny_rgbd/registration/motion_estimation_ransac_model.h"

namespace ccny_rgbd
{

MotionEstimationRANSACModel::MotionEstimationRANSACModel(ros::NodeHandle nh, ros::NodeHandle nh_private):
  MotionEstimation(nh, nh_private),
  bf_matcher_(cv::NORM_L2)
{
  // **** init variables

  f2b_.setIdentity();

  // *** init params

  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/camera_link";

  /*
  max_association_dist_ = 0.15;
  max_association_dist_sq_ = max_association_dist_ * max_association_dist_;
  alpha_ = 0.01;

  int icp_max_iterations;
  double icp_tf_epsilon;
  double icp_max_corresp_dist;
  bool use_ransac_rejection;
  double ransac_inlier_threshold;

  if (!nh_private_.getParam ("reg/max_iterations", icp_max_iterations))
    icp_max_iterations = 30;
  if (!nh_private_.getParam ("reg/tf_epsilon", icp_tf_epsilon))
    icp_tf_epsilon = 0.000001;
  if (!nh_private_.getParam ("reg/use_ransac_rejection", use_ransac_rejection))
    use_ransac_rejection = false;
  if (!nh_private_.getParam ("reg/max_corresp_dist", icp_max_corresp_dist))
    icp_max_corresp_dist = 0.15;
  if (!nh_private_.getParam ("reg/ransac_inlier_threshold", ransac_inlier_threshold))
    ransac_inlier_threshold = 0.10;

  reg_.setMaxIterations(icp_max_iterations);
  reg_.setTransformationEpsilon(icp_tf_epsilon);
  reg_.setMaxCorrDist(icp_max_corresp_dist);
  reg_.setUseRANSACRejection(use_ransac_rejection);
  reg_.setRANSACThreshold(ransac_inlier_threshold);
*/

  model_ptr_.reset(new PointCloudFeature());

  model_ptr_->header.frame_id = fixed_frame_;

  model_publisher_ = nh_.advertise<PointCloudFeature>(
    "model", 1);
}

MotionEstimationRANSACModel::~MotionEstimationRANSACModel()
{

}

tf::Transform MotionEstimationRANSACModel::getMotionEstimation(
  RGBDFrame& frame,
  const tf::Transform& prediction)
{
  tf::Transform motion;

  PointCloudFeature::Ptr features_ptr =
    boost::shared_ptr<PointCloudFeature> (new PointCloudFeature());

  // account for prediction
  // prediction is in the world frame
  tf::Transform predicted_f2b = prediction * f2b_;

  // rotate into the fixed frame and account for prediction
  pcl::transformPointCloud(frame.features , *features_ptr, eigenFromTf(predicted_f2b * b2c_));
  features_ptr->header.frame_id = fixed_frame_;


  // **** build model ***************************************************

  if (model_ptr_->points.empty())
  {
    ROS_WARN("No points in model");
    motion.setIdentity();

    // transform features and add to model
    pcl::transformPointCloud (frame.features , *features_ptr, eigenFromTf(f2b_* b2c_));
    features_ptr->header.frame_id = fixed_frame_;
    *model_ptr_ += *features_ptr;

    // add descriptors
  
    std::vector<cv::Mat> new_descriptors;

    for (unsigned int kp_idx = 0; kp_idx < frame.keypoints.size(); ++kp_idx)
    {
      if (frame.kp_valid[kp_idx])
      {
        model_keypoints_.push_back(frame.keypoints[kp_idx]);
        model_descriptors_.push_back(frame.descriptors.row(kp_idx));
      
        new_descriptors.push_back(frame.descriptors.row(kp_idx));
      }
    }

    matcher_.add(new_descriptors);
  }
  else
  {  
    // **** calculate feature-space kdtree of model
 
    ros::WallTime s_train = ros::WallTime::now();
    matcher_.train();
    double d_train = (ros::WallTime::now() - s_train).toSec();
    printf("Train dur: %4.1f\n", d_train * 1000.0);
    
    // match new features to model (possibly k nn?)
   
    ros::WallTime s_match = ros::WallTime::now();
    std::vector<cv::DMatch> matches;

    //matcher_.match(frame.descriptors, matches);
    bf_matcher_.match(frame.descriptors, model_descriptors_, matches);

    double d_match = (ros::WallTime::now() - s_match).toSec();
    printf("Match dur: %4.1f\n", d_match * 1000.0);

    // remove outliers using SVD-based RANSAC 
    // apply transform
    // check for new features, add them to models

    // add descriptors FIXME: replace with real
   
    printf("adding\n");

    pcl::transformPointCloud (frame.features , *features_ptr, eigenFromTf(f2b_* b2c_));
    features_ptr->header.frame_id = fixed_frame_;
    *model_ptr_ += *features_ptr;

    std::vector<cv::Mat> new_descriptors;

    for (unsigned int kp_idx = 0; kp_idx < frame.keypoints.size(); ++kp_idx)
    {
      if (frame.kp_valid[kp_idx])
      {
        model_keypoints_.push_back(frame.keypoints[kp_idx]);
        model_descriptors_.push_back(frame.descriptors.row(kp_idx));
      
        new_descriptors.push_back(frame.descriptors.row(kp_idx));
      }
    }

    matcher_.add(new_descriptors);
  }

  model_ptr_->header.stamp = ros::Time::now(); // TODO - change to actual timestamp
  model_ptr_->width = model_ptr_->points.size();
  model_publisher_.publish(model_ptr_);
  printf("Model size: %d\n", (int)model_ptr_->points.size()); 

  return motion;
}

} // namespace ccny_rgbd
