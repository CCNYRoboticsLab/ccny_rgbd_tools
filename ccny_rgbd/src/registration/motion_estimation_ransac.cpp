#include "ccny_rgbd/registration/motion_estimation_ransac.h"

namespace ccny_rgbd
{

MotionEstimationRANSAC::MotionEstimationRANSAC(ros::NodeHandle nh, ros::NodeHandle nh_private):
  MotionEstimation(nh, nh_private),
  have_prev_frame_(false)
{
  // **** init variables

  f2b_.setIdentity();
  last_keyframe_f2b_ = f2b_;

  // *** init params

  // TODO - change to dynamic
  // TODO - these aren't used, do we need to skip frames?
  kf_dist_eps_  = 0.15; 
  kf_angle_eps_ = 15.0 * M_PI / 180.0; 

  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/camera_link";
  if (!nh_private_.getParam ("reg/RANSAC/matching_distance", matching_distance_))
    matching_distance_ = 10.0;
  if (!nh_private_.getParam ("reg/RANSAC/eps_reproj", eps_reproj_))
    eps_reproj_ = 2.0;
  if (!nh_private_.getParam ("reg/RANSAC/inlier_threshold", inlier_threshold_))
    inlier_threshold_ = 0.33;
  if (!nh_private_.getParam ("reg/RANSAC/icp_refine", icp_refine_))
    icp_refine_ = true;
}

MotionEstimationRANSAC::~MotionEstimationRANSAC()
{

}

bool MotionEstimationRANSAC::getMotionEstimationImpl(
  RGBDFrame& frame,
  const tf::Transform& prediction,
  tf::Transform& motion)
{
  if (have_prev_frame_)
  {
    bool ransac_result = ransacMatchingOverlap(
      frame, prev_frame_, motion);

    if (!ransac_result)
    {
      motion.setIdentity();
      ROS_WARN("No convergence");
    }

    //if (tfGreaterThan(motion, kf_dist_eps_, kf_angle_eps_))
      prev_frame_ = frame;
  }
  else
  { 
    motion.setIdentity();
    prev_frame_ = frame;
    have_prev_frame_ = true;
  } 

  //calculate new f2b
  tf::Transform f2b_new = f2b_ * b2c_ * motion * b2c_.inverse();

  // trnasform to fixed frame
  motion = f2b_new * f2b_.inverse();

  // update
  f2b_ = f2b_new;

  return true;
}

bool MotionEstimationRANSAC::ransacMatchingOverlap(
  RGBDFrame& frame_src, 
  RGBDFrame& frame_dst, 
  tf::Transform& transform,
  PointCloudT::Ptr cloud_src, 
  PointCloudT::Ptr cloud_dst)
{
  bool show = true;
  bool save = false;

  // **** params

  // **** if needed, detect keypoints
    
  if (!frame_src.keypoints_computed)
  {
    ROS_WARN("no keypoints present, computing SURF keypoints");
    cv::SurfFeatureDetector detector;
    detector.detect(*(frame_src.getRGBImage()), frame_src.keypoints);
    frame_src.keypoints_computed = true;
  }
  if (!frame_dst.keypoints_computed)
  {
    ROS_WARN("no keypoints present, computing SURF keypoints");
    cv::SurfFeatureDetector detector;
    detector.detect(*(frame_dst.getRGBImage()), frame_dst.keypoints);
    frame_dst.keypoints_computed = true;
  }

  // **** if needed, extract descriptors

  if (!frame_src.descriptors_computed)
  {
    ROS_WARN("no descriptors present, computing SURF descriptors");
    cv::BriefDescriptorExtractor extractor;
    extractor.compute(*(frame_src.getRGBImage()), frame_src.keypoints, frame_src.descriptors);
    frame_src.descriptors_computed = true;
  }
  if (!frame_dst.descriptors_computed)
  {
    ROS_WARN("no descriptors present, computing SURF descriptors");
    cv::BriefDescriptorExtractor extractor;
    extractor.compute(*(frame_dst.getRGBImage()), frame_dst.keypoints, frame_dst.descriptors);
    frame_dst.descriptors_computed = true;
  }

  // **** match the descriptors

  ros::WallTime start = ros::WallTime::now();

  //cv::FlannBasedMatcher matcher; // for SURF
  cv::BFMatcher matcher(cv::NORM_HAMMING); // for ORB
  //cv::BFMatcher matcher(cv::NORM_L2); // for Brief
  std::vector<cv::DMatch> matches;
  matcher.match(frame_src.descriptors, frame_dst.descriptors, matches);

  std::vector<cv::DMatch> good_matches;

  for (unsigned int i = 0; i < matches.size(); ++i)
  {
    if (matches[i].distance < matching_distance_)
      good_matches.push_back(matches[i]);
  }

  double duration_matching = (ros::WallTime::now() - start).toSec();
  printf("Matching dur: %.1f\n", duration_matching * 1000.0);

  // **** create vectors of feature points from correspondences

  std::vector<cv::Point2f> h_src_pts;
  std::vector<cv::Point2f> h_dst_pts;

  for(unsigned int i = 0; i < good_matches.size(); ++i)
  {
    int q_idx = good_matches[i].queryIdx;
    int t_idx = good_matches[i].trainIdx; 

    h_src_pts.push_back(frame_src.keypoints[q_idx].pt);
    h_dst_pts.push_back(frame_dst.keypoints[t_idx].pt);
  }

  // **** find inliers using ransac

  ros::WallTime start_r = ros::WallTime::now();

  bool ransac_overlap;
  double inlier_ratio;
  std::vector<cv::DMatch> inlier_matches;

  cv::Mat status;
  cv::Mat h = cv::findHomography(h_src_pts, h_dst_pts, CV_RANSAC, eps_reproj_, status);

  for (unsigned int m = 0; m < good_matches.size(); m++) 
  { 
    if (status.at<char>(m, 0) == 1)
      inlier_matches.push_back(good_matches[m]);
  }

  double duration_ransac = (ros::WallTime::now() - start_r).toSec();
  printf("RANSAC dur: %.1f\n", duration_ransac * 1000.0);

  // **** draw & save the raw matches

  if (show)
  {
    cv::Mat img_matches;
    cv::drawMatches(*(frame_src.getRGBImage()), frame_src.keypoints, 
                    *(frame_dst.getRGBImage()), frame_dst.keypoints, good_matches, img_matches);

    cv::namedWindow("raw matches", CV_WINDOW_NORMAL);
    cv::imshow("raw matches", img_matches);
    cv::waitKey(1);

    if (save)
    {
      std::stringstream ss1;
      ss1 << frame_src.header.seq << "_to_" << frame_dst.header.seq << "_base";
      cv::imwrite("/home/idryanov/ros/images/" + ss1.str() + ".png", img_matches);
    }
  }

  // **** check if ratio of inliers is high enough

  inlier_ratio = (double)inlier_matches.size() / (double)good_matches.size();
  ransac_overlap = (inlier_ratio > inlier_threshold_);

  // ***** compute rigid transformation from inliers

  if (ransac_overlap)
  {
    // **** draw & save the raw matches

    if (show)
    {
      cv::Mat img_inlier_matches;
      cv::drawMatches(*(frame_src.getRGBImage()), frame_src.keypoints, 
                      *(frame_dst.getRGBImage()), frame_dst.keypoints, 
                      inlier_matches, img_inlier_matches);

      cv::namedWindow("inlier matches", CV_WINDOW_NORMAL);
      cv::imshow("inlier matches", img_inlier_matches);
      cv::waitKey(1);

      if (save)
      {
        std::stringstream ss;
        ss << frame_src.header.seq << "_to_" << frame_dst.header.seq << "_inliers";
        cv::imwrite("/home/idryanov/ros/images/" + ss.str() + ".png", img_inlier_matches);
      }
    }

    // create 3D point clouds   
    RGBDFrame::constructCloudsFromInliers(inlier_matches, frame_src, frame_dst, cloud_src, cloud_dst);

    // estimate using simple svd
    Eigen::Matrix4f transform_eigen;
    pcl::registration::TransformationEstimationSVD<PointT, PointT> svd;
    svd.estimateRigidTransformation(*cloud_src, *cloud_dst, transform_eigen);
    transform = tfFromEigen(transform_eigen);

    // refine estimate using icp
    if (icp_refine_)
    {
      pcl::transformPointCloud(*cloud_src, *cloud_src, eigenFromTf(transform));

      ccny_rgbd::ICPKd<PointT, PointT> reg;
    
      reg.setMaxIterations(20);
      reg.setTransformationEpsilon(0.001);
      reg.setMaxCorrDist(0.15);
      reg.setUseValueRejection(false);
      reg.setUseRANSACRejection(false);
      reg.setRANSACThreshold(0.15);

      pcl::KdTreeFLANN<PointT> tree_data;
      pcl::KdTreeFLANN<PointT> tree_model;

      tree_data.setInputCloud(cloud_src);
      tree_model.setInputCloud(cloud_dst);

      reg.setDataCloud  (&*cloud_src);
      reg.setModelCloud (&*cloud_dst);

      reg.setDataTree  (&tree_data);
      reg.setModelTree (&tree_model);

      reg.align();
      Eigen::Matrix4f icp_corr_eigen = reg.getFinalTransformation();
      tf::Transform icp_corr = tfFromEigen(icp_corr_eigen);

      transform = icp_corr * transform;
    }
  }

  return ransac_overlap;
}

} // namespace ccny_rgbd
