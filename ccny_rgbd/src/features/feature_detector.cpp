#include "ccny_rgbd/features/feature_detector.h"

namespace ccny_rgbd
{

FeatureDetector::FeatureDetector(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), nh_private_(nh_private)
  //mask_created_(false),
  //mask_w_(30)
{
  if (!nh_private_.getParam ("feature/compute_descriptors", compute_descriptors_))
    compute_descriptors_ = false;
  if (!nh_private_.getParam ("feature/smooth", smooth_))
    smooth_ = 0;
  if (!nh_private_.getParam ("feature/max_range", max_range_))
    max_range_ = 5.5;
  if (!nh_private_.getParam ("feature/show_keypoints", show_keypoints_))
    show_keypoints_ = false;
  if (!nh_private_.getParam ("feature/publish_features", publish_features_))
    publish_features_ = true;
  if (!nh_private_.getParam ("feature/publish_covariances", publish_covariances_))
    publish_covariances_ = true;

  if (publish_features_)
  {
    features_publisher_ = nh_private.advertise<PointCloudFeature>(
      "features", 1);
  }

  if (publish_covariances_)
  {
    covariances_publisher_ = nh_private.advertise<visualization_msgs::Marker>(
      "/covariances", 1);
  }
}

FeatureDetector::~FeatureDetector()
{

}

double FeatureDetector::findFeaturesTimed(RGBDFrame& frame)
{
  ros::WallTime start = ros::WallTime::now();
  findFeatures(frame);
  return (start - ros::WallTime::now()).toSec();
}

void FeatureDetector::findFeatures(RGBDFrame& frame)
{
  cv::Mat * input_img = frame.getRGBImage();

  // convert from RGB to grayscale
  cv::Mat gray_img(input_img->rows, input_img->cols, CV_8UC1);
  cvtColor(*input_img, gray_img, CV_BGR2GRAY);

  // blur if needed
  if(smooth_ > 0)
  {
    int blur_size = smooth_*2 + 1;
    cv::GaussianBlur(gray_img, gray_img, cv::Size(blur_size, blur_size), 0);
  }

  // find the 2D coordinates of keypoints
  findFeatures(frame, &gray_img);
  frame.keypoints_computed = true;

  // calculates the 3D position and covariance of features
  frame.computeDistributions();

  // filters out features with bad data and constructs a point cloud
  frame.constructFeatureCloud(max_range_);

  if (show_keypoints_)
  {
    cv::namedWindow("Keypoints", CV_WINDOW_NORMAL);
    cv::Mat kp_img(input_img->rows, input_img->cols, CV_8UC1);
    cv::drawKeypoints(*input_img, frame.keypoints, kp_img);
    cv::imshow("Keypoints", kp_img);
    cv::waitKey(1);
  }

  if (publish_features_)
    features_publisher_.publish(frame.features);

  if (publish_covariances_)
    publishCovariances(frame);
}

void FeatureDetector::publishCovariances(RGBDFrame& frame)
{
  // create markers
  visualization_msgs::Marker marker;
  marker.header = frame.header_;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.scale.x = 0.0025;
  marker.action = visualization_msgs::Marker::ADD;
  marker.ns = "covariances";
  marker.id = 0;
  marker.lifetime = ros::Duration();

  for (unsigned int kp_idx = 0; kp_idx < frame.keypoints.size(); ++kp_idx)
  {
    if (!frame.kp_valid[kp_idx]) continue;
    
    // compute eigenvectors
    cv::Mat evl(1, 3, CV_64F);
    cv::Mat evt(3, 3, CV_64F);

    cv::Mat& m   = frame.kp_mean[kp_idx];
    cv::Mat& cov = frame.kp_covariance[kp_idx];
    cv::eigen(cov, evl, evt);

    double mx = m.at<double>(0,0);
    double my = m.at<double>(1,0);
    double mz = m.at<double>(2,0);

    for (int e = 0; e < 3; ++e)
    {
      geometry_msgs::Point a;
      geometry_msgs::Point b;

      double sigma = sqrt(evl.at<double>(0,e));
      double scale = sigma * 3.0;
      tf::Vector3 evt_tf(evt.at<double>(e,0), 
                         evt.at<double>(e,1), 
                         evt.at<double>(e,2));
    
      a.x = mx + evt_tf.getX() * scale;
      a.y = my + evt_tf.getY() * scale;
      a.z = mz + evt_tf.getZ() * scale;
   
      b.x = mx - evt_tf.getX() * scale;
      b.y = my - evt_tf.getY() * scale;
      b.z = mz - evt_tf.getZ() * scale;

      marker.points.push_back(a);
      marker.points.push_back(b);
    }
  }

  covariances_publisher_.publish(marker);
}

void FeatureDetector::setSmooth(int smooth)
{
  smooth_ = smooth;
}

int FeatureDetector::getSmooth() const
{
  return smooth_;
}

void FeatureDetector::setMaxRange(double max_range)
{
  max_range_ = max_range;
}

double FeatureDetector::getMaxRange() const
{
  return max_range_;
} 

} //namespace
