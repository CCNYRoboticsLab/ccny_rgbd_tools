#include "ccny_rgbd/apps/mono_vo.h"

namespace ccny_rgbd {

void MonocularVisualOdometry::setFeatureDetector()
{
  // feature params
  if (detector_type_ == "ORB")
    feature_detector_ = new OrbDetector(nh_, nh_private_);
  else if (detector_type_ == "SURF")
    feature_detector_ = new SurfDetector(nh_, nh_private_);
  else if (detector_type_ == "GFT")
    feature_detector_ = new GftDetector(nh_, nh_private_);
  else
    ROS_FATAL("%s is not a valid detector type!", detector_type_.c_str());
}

bool MonocularVisualOdometry::readPointCloudFromPCDFile()
{
  cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_filename_, *cloud_) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return false;
  }
  std::cout << "Loaded "
      << cloud_->width * cloud_->height
      << " data points from test_pcd.pcd with the following fields: "
      << std::endl;
//  for (size_t i = 0; i < cloud_->points.size (); ++i)
//    std::cout << "    " << cloud->points[i].x
//    << " "    << cloud->points[i].y
//    << " "    << cloud->points[i].z << std::endl;

  return true;
}

MonocularVisualOdometry::MonocularVisualOdometry(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  initialized_(false),
  frame_count_(0)
{
  ROS_INFO("Starting RGBD Visual Odometry");
  // **** init parameters

  initParams();

  // **** init variables

  f2b_.setIdentity();

  // feature params
  setFeatureDetector();
  if(readPointCloudFromPCDFile()==false)
    ROS_FATAL("The sky needs its point cloud to operate!");

  // **** publishers
  odom_publisher_ = nh_.advertise<OdomMsg>(
    "odom", 5);

  // **** subscribers
  image_transport::ImageTransport rgb_it(nh_);
  sub_rgb_.subscribe(
    rgb_it, "/camera/rgb/image_rect_color", 1);
  sub_info_.subscribe(
    nh_, "/camera/rgb/camera_info", 1);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  int queue_size = 5;
  sync_.reset(new SynchronizerMonoVO(SyncPolicyMonoVO(queue_size), sub_rgb_, sub_info_));
  sync_->registerCallback(boost::bind(&MonocularVisualOdometry::imageCallback, this, _1, _2));
}

MonocularVisualOdometry::~MonocularVisualOdometry()
{
  ROS_INFO("Destroying Monocular Visual Odometry");

  delete feature_detector_;
}

void MonocularVisualOdometry::initParams()
{
  // PCD File
  if(!nh_private_.getParam("PCD_filename", pcd_filename_))
    pcd_filename_ = "cloud.pcd";

  // **** frames
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/camera_link";

  if (!nh_private_.getParam ("feature/detector_type", detector_type_))
    detector_type_ = "GFT";

  // FIXME: find the right values:
  nh_private_.param("sensor_aperture_width", sensor_aperture_width_, 4.8); // Default for a 1/3" = 4.8 mm
  nh_private_.param("sensor_aperture_height", sensor_aperture_height_, 3.6); // Default for a 1/3" = 3.6 mm

}

void MonocularVisualOdometry::imageCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  mutex_lock_.lock();

  ros::WallTime start = ros::WallTime::now();

  // **** initialize ***************************************************

  if (!initialized_)
  {
    initialized_ = getBaseToCameraTf(rgb_msg->header);
    init_time_ = rgb_msg->header.stamp;
    if (!initialized_) return;

    motion_estimation_->setBaseToCameraTf(b2c_);
  }

  processCameraInfo(info_msg);

  // **** create frame *************************************************

  ros::WallTime start_frame = ros::WallTime::now();
  RGBDFrame frame(rgb_msg, info_msg);
  ros::WallTime end_frame = ros::WallTime::now();

  // **** find features ************************************************

  ros::WallTime start_features = ros::WallTime::now();
  feature_detector_->findFeatures(frame);
  ros::WallTime end_features = ros::WallTime::now();

  // ------- Perspective projection of cloud 3D points onto image plane
  ros::WallTime start_3D_cloud_projection = ros::WallTime::now();
  // TODO
  // projectPoints(InputArray objectPoints, InputArray rvec, InputArray tvec, InputArray cameraMatrix, InputArray distCoeffs, OutputArray imagePoints, OutputArray jacobian=noArray(), double aspectRatio=0 )
  // Assume known initial position of camera position at the origin of the world center of coordinates
  // TODO: needs a base2cam static transformation to correct for the camera coordinates in the world where +Z is point upwards
  // ---------------------------------------------------------------
  std::vector<cv::Point3f> objectPoints3D;
  float cloud_point_x, cloud_point_y, cloud_point_z; // TODO: Get each 3D point from the cloud
  cv::Point3f cloud_point(cloud_point_x, cloud_point_y, cloud_point_z);
  objectPoints3D.push_back(cloud_point);

  // NOTE: the OpenNI driver publishes the static transformation (doing the rotation of the axis) between the rgb optical frame (/camera_rgb_optical_frame) and the /camera_link
  std::vector<cv::Point2f> projectedPoints;
  // With rectified image:
//  cv::projectPoints(objectPoints3D, rvec, tvec, instrinsicRectifiedCamMatrix , distCoeffs, projectedPoints);
  cv::projectPoints(objectPoints3D, rvec_, tvec_, K_, dist_coeffs_, projectedPoints);

  for(unsigned int i = 0; i < projectedPoints.size(); ++i)
    {
    std::cout << "3D Object point: " << objectPoints3D[i] << " Projected to " << projectedPoints[i] << std::endl;
    }


  ros::WallTime end_3D_cloud_projection = ros::WallTime::now();

  // **** registration *************************************************
  // TODO: with the 3Dto2D method of registration and using PnP
  ros::WallTime start_reg = ros::WallTime::now();
  tf::Transform motion = motion_estimation_->getMotionEstimation(frame);
  f2b_ = motion * f2b_;
  ros::WallTime end_reg = ros::WallTime::now();

  // **** publish motion **********************************************

  publishTf(rgb_msg->header);

  // **** print diagnostics *******************************************

  ros::WallTime end = ros::WallTime::now();

  int n_features = frame.features.points.size();
  int n_keypoints = frame.keypoints.size();

  double d_frame    = 1000.0 * (end_frame    - start_frame   ).toSec();
  double d_features = 1000.0 * (end_features - start_features).toSec();
  double d_cloud_projection = 1000.0 * (end_3D_cloud_projection - start_3D_cloud_projection).toSec();
  double d_reg      = 1000.0 * (end_reg      - start_reg     ).toSec();
  double d_total    = 1000.0 * (end          - start         ).toSec();

  float time = (rgb_msg->header.stamp - init_time_).toSec();
  int model_size = motion_estimation_->getModelSize();

  double pos_x = f2b_.getOrigin().getX();
  double pos_y = f2b_.getOrigin().getY();
  double pos_z = f2b_.getOrigin().getZ();

  ROS_INFO("[%d] Fr: %2.1f s %s[%d][%d]: %3.1f s  \t Proj: %2.1f s \t Reg: %4.1f s \t TOTAL %4.1f s\n",
    frame_count_,
    d_frame, 
    detector_type_.c_str(), n_features, n_keypoints, d_features, 
    d_cloud_projection, d_reg,
    d_total);

  frame_count_++;

  mutex_lock_.unlock();
}

// %Tag(CALLBACK)%
void MonocularVisualOdometry::processCameraInfo(const sensor_msgs::CameraInfoConstPtr &cam_info_ptr)
{
  static bool first_time = true;
  //  cv::Mat cameraMatrix(cv::Size(4,3), CV_64FC1);
  // Projection/camera matrix
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]
  //     [ 0   0   1   0]
  // Note that calibrationMatrixValues doesn't require the Tx,Ty column (it only takes the 3x3 intrinsic parameteres)
  if(first_time)
  {
  // Create the known projection matrix (obtained from Calibration)
  cv::Mat P(3,4,cv::DataType<float>::type);
  P.at<float>(0,0) = cam_info_ptr->P[0];
  P.at<float>(1,0) = cam_info_ptr->P[4];
  P.at<float>(2,0) = cam_info_ptr->P[8];

  P.at<float>(0,1) = cam_info_ptr->P[1];
  P.at<float>(1,1) = cam_info_ptr->P[5];
  P.at<float>(2,1) = cam_info_ptr->P[9];

  P.at<float>(0,2) = cam_info_ptr->P[2];
  P.at<float>(1,2) = cam_info_ptr->P[6];
  P.at<float>(2,2) = cam_info_ptr->P[10];

  P.at<float>(0,3) = cam_info_ptr->P[3];
  P.at<float>(1,3) = cam_info_ptr->P[7];
  P.at<float>(2,3) = cam_info_ptr->P[11];

  // Decompose the projection matrix into:
  K_.create(3,3,cv::DataType<float>::type); // intrinsic parameter matrix
  R_.create(3,3,cv::DataType<float>::type); // rotation matrix
  T_.create(4,1,cv::DataType<float>::type); // translation vector
  cv::decomposeProjectionMatrix(P, K_, R_, T_);

  cv::Rodrigues(R_, rvec_);
  tvec_.create(3,1,cv::DataType<float>::type);
  for(unsigned int t=0; t<3; t++)
     tvec_.at<float>(t) = T_.at<float> (t);

  // Create zero distortion
   dist_coeffs_.create(cam_info_ptr->D.size(),1,cv::DataType<float>::type);
//  cv::Mat distCoeffs(4,1,cv::DataType<float>::type, &cam_info_ptr->D.front());
   for(unsigned int d=0; d<cam_info_ptr->D.size(); d++)
   {
     dist_coeffs_.at<float>(d) = cam_info_ptr->D[d];
   }
   //   distCoeffs.at<float>(0) = cam_info_ptr->D[0];
   //   distCoeffs.at<float>(1) = cam_info_ptr->D[1];
   //   distCoeffs.at<float>(2) = cam_info_ptr->D[2];
   //   distCoeffs.at<float>(3) = cam_info_ptr->D[3];

   std::cout << "Decomposed Matrix" << std::endl;
   std::cout << "P: " << P << std::endl;
   std::cout << "R: " << R_ << std::endl;
   std::cout << "rvec_: " << rvec_ << std::endl;
   std::cout << "t: " << T_ << std::endl;
   std::cout << "tvec: " << tvec_ << std::endl;
   std::cout << "K: " << K_ << std::endl;
   std::cout << "Distortion: " << dist_coeffs_ << std::endl;


   // More:
   cv::Size imageSize(cam_info_ptr->width, cam_info_ptr->height);
   double fovx ,fovy, focalLength;
   double aspectRatio;

   std::cout << "Sensor apertures: width = " << sensor_aperture_width_ << " height = " << sensor_aperture_height_ << std::endl;  // passed as parameters
   std::cout << "Image Size: " << imageSize.width << ", " << imageSize.height << std::endl;

   cv::calibrationMatrixValues(K_, imageSize, sensor_aperture_width_, sensor_aperture_height_, fovx, fovy, focalLength, principal_point_, aspectRatio);

   std::cout << "Camera intrinsic parameters: " << std::endl
       << "fovx=" << fovx << ", fovy=" << fovy << ", Focal Length= " << focalLength << " mm"
       << ", Principal Point=" <<  principal_point_ << ", Aspect Ratio=" << aspectRatio
       << std::endl;

   // FIXME: why was I doing this?
//     principal_point_.x += imageSize.width /2.0;
//     principal_point_.y += imageSize.height /2.0;

   first_time = false;
  }
}
// %EndTag(CALLBACK)%

void MonocularVisualOdometry::publishTf(const std_msgs::Header& header)
{
  tf::StampedTransform transform_msg(
   f2b_, header.stamp, fixed_frame_, base_frame_);
  tf_broadcaster_.sendTransform (transform_msg);

  OdomMsg odom;
  odom.header.stamp = header.stamp;
  odom.header.frame_id = fixed_frame_;
  tf::poseTFToMsg(f2b_, odom.pose.pose);
  odom_publisher_.publish(odom);
}

bool MonocularVisualOdometry::getBaseToCameraTf(const std_msgs::Header& header)
{
  tf::StampedTransform tf_m;

  try
  {
    tf_listener_.waitForTransform(
      base_frame_, header.frame_id, header.stamp, ros::Duration(1.0));
    tf_listener_.lookupTransform (
      base_frame_, header.frame_id, header.stamp, tf_m);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("Base to camera transform unavailable %s", ex.what());
    return false;
  }

  b2c_ = tf_m;

  return true;
}

} //namespace ccny_rgbd
