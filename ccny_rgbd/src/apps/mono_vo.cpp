#include "ccny_rgbd/apps/mono_vo.h"

namespace ccny_rgbd {

MonocularVisualOdometry::MonocularVisualOdometry(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  initialized_(false),
  frame_count_(0)
{
  ROS_INFO("Starting Monocular Visual Odometry from a 3D Sparse Model");
  // **** init parameters

  initParams();

  // **** init variables
  f2b_.setIdentity();

  // **** publishers
  pub_model_ = nh_private.advertise<PointCloudFeature>(
   "sparse_model", 1);
  odom_publisher_ = nh_.advertise<OdomMsg>(
    "odom", 5);

  // **** subscribers
  image_transport::ImageTransport rgb_it(nh_);
  sub_rgb_.subscribe(
    rgb_it, topic_image_, 1);
  sub_info_.subscribe(
    nh_, topic_cam_info_, 1);

  // feature params
  setFeatureDetector();

  if(readPointCloudFromPCDFile()==false)
    ROS_FATAL("The sky needs its point cloud to operate!");


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
  if(!nh_private_.getParam("apps/mono_vo/PCD_filename", pcd_filename_))
    pcd_filename_ = "cloud.pcd";

  // **** frames
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/camera_link";
  if (!nh_private_.getParam ("feature/detector_type", detector_type_))
    detector_type_ = "GFT";
  if (!nh_private_.getParam ("apps/mono_vo/publish_cloud_model", publish_cloud_model_))
    publish_cloud_model_ = false;
  if (!nh_private_.getParam ("apps/mono_vo/topic_cam_info", topic_cam_info_))
    topic_cam_info_ = "/camera/rgb/camera_info";
  if (!nh_private_.getParam ("apps/mono_vo/topic_image", topic_image_))
    topic_image_ = "/camera/rgb/image_rect_color";

  // TODO: find the right values:
  nh_private_.param("app/mono_vo/sensor_aperture_width", sensor_aperture_width_, 4.8); // Default for a 1/3" = 4.8 mm
  nh_private_.param("app/mono_vo/sensor_aperture_height", sensor_aperture_height_, 3.6); // Default for a 1/3" = 3.6 mm

  ROS_INFO("Parameters initialized.");

}

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
//  cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  model_ptr_ = PointCloudFeature::Ptr(new PointCloudFeature);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_filename_, *model_ptr_) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return false;
  }
  model_ptr_->header.frame_id = fixed_frame_;
  model_ptr_->header.stamp = ros::Time::now();

  std::cout << "Loaded "
      << model_ptr_->width * model_ptr_->height
      << " data points from " << pcd_filename_ << " with header = " <<   model_ptr_->header.frame_id
      << std::endl;
//  for (size_t i = 0; i < cloud_->points.size (); ++i)
//    std::cout << "    " << cloud->points[i].x
//    << " "    << cloud->points[i].y
//    << " "    << cloud->points[i].z << std::endl;

  return true;
}

void MonocularVisualOdometry::imageCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
//  mutex_lock_.lock();
  ros::WallTime start = ros::WallTime::now();

  // **** initialize ***************************************************

  if (!initialized_)
  {
    frame_ = boost::shared_ptr<MonocularFrame> (new MonocularFrame(rgb_msg, info_msg));
    ROS_INFO("RGB header = %s", rgb_msg->header.frame_id.c_str());
    initialized_ = getBaseToCameraTf(rgb_msg->header);
    init_time_ = rgb_msg->header.stamp;
    if (!initialized_) return;

    frame_->setCameraAperture(sensor_aperture_width_, sensor_aperture_height_);
    motion_estimation_->setBaseToCameraTf(b2c_);
  }

  // **** create frame *************************************************

  ros::WallTime start_frame = ros::WallTime::now();
  frame_->setFrame(rgb_msg);

  ros::WallTime end_frame = ros::WallTime::now();

  // **** find features ************************************************

  ros::WallTime start_features = ros::WallTime::now();
  feature_detector_->onlyFind2DFeatures(*frame_);
  ros::WallTime end_features = ros::WallTime::now();

  // ------- Perspective projection of cloud 3D points onto image plane
  ros::WallTime start_3D_cloud_projection = ros::WallTime::now();
  // Assume known initial position of camera position at the origin of the world center of coordinates
  // TODO: needs a base2cam static transformation to correct for the camera coordinates in the world where +Z is point upwards
  // NOTE: the OpenNI driver publishes the static transformation (doing the rotation of the axis) between the rgb optical frame (/camera_rgb_optical_frame) and the /camera_link
  // ---------------------------------------------------------------
  is_first_time_projecting_ = frame_->project3DModelToCamera(model_ptr_, is_first_time_projecting_);
  ros::WallTime end_3D_cloud_projection = ros::WallTime::now();

  // **** registration *************************************************
  // TODO: with the 3Dto2D method of registration and using PnP
  ros::WallTime start_PnP_reg = ros::WallTime::now();
//  tf::Transform motion = motion_estimation_->getMotionEstimation(frame);
//  f2b_ = motion * f2b_; // TODO: the transformation based on motion estimation after PnP
  ros::WallTime end_PnP_reg = ros::WallTime::now();
  // **** publish motion **********************************************

  publishTf(rgb_msg->header);

  ros::WallTime end = ros::WallTime::now();
  // **** print diagnostics *******************************************

//  int n_features = frame.features.points.size();
  int n_keypoints = frame_->keypoints.size();

  double d_frame    = 1000.0 * (end_frame    - start_frame   ).toSec();
  double d_features = 1000.0 * (end_features - start_features).toSec();
  double d_cloud_projection = 1000.0 * (end_3D_cloud_projection - start_3D_cloud_projection).toSec();
  double d_PnP_reg      = 1000.0 * (end_PnP_reg      - start_PnP_reg).toSec();
  double d_total    = 1000.0 * (end          - start).toSec();

  /*
//  float time = (rgb_msg->header.stamp - init_time_).toSec();
  int model_size = motion_estimation_->getModelSize();

  double pos_x = f2b_.getOrigin().getX();
  double pos_y = f2b_.getOrigin().getY();
  double pos_z = f2b_.getOrigin().getZ();
*/

  ROS_INFO("[%d] Fr: %2.1f s %s[%d keyspoints]: %3.1f s  \t Proj: %2.1f s \t Reg: %4.1f s \t TOTAL %4.1f s\n",
    frame_count_,
    d_frame, 
    detector_type_.c_str(),
    n_keypoints, d_features,
    d_cloud_projection, d_PnP_reg,
    d_total);

  frame_count_++;

  if(publish_cloud_model_)
  {
    printf("Publishing model cloud read from PCD\n");
    pub_model_.publish(*model_ptr_);
  }

//  mutex_lock_.unlock();
}

void MonocularVisualOdometry::publishTf(const std_msgs::Header& header)
{
  ROS_INFO("Transforming Fixed Frame to Base (Camera link)");

  tf::StampedTransform transform_msg(
   f2b_, header.stamp, fixed_frame_, base_frame_);
  tf_broadcaster_.sendTransform (transform_msg);

  ROS_WARN("Successfully sent transform Fixed (%s) to Base (%s)", fixed_frame_.c_str(), base_frame_.c_str());

  OdomMsg odom;
  odom.header.stamp = header.stamp;
  odom.header.frame_id = fixed_frame_;
  tf::poseTFToMsg(f2b_, odom.pose.pose);
  odom_publisher_.publish(odom);
}

bool MonocularVisualOdometry::getBaseToCameraTf(const std_msgs::Header& header)
{
  tf::StampedTransform tf_m;

  ROS_INFO("Transforming Base to Camera");
  try
  {
    tf_listener_.waitForTransform(
      base_frame_, header.frame_id, header.stamp, ros::Duration(1.0));
    tf_listener_.lookupTransform (
      base_frame_, header.frame_id, header.stamp, tf_m);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("Base to camera transform unavailable: %s", ex.what());
    return false;
  }

  b2c_ = tf_m;

  ROS_INFO("Successfully transformed Base to Camera");
  return true;
}

} //namespace ccny_rgbd
