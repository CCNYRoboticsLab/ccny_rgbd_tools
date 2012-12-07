#include "ccny_rgbd/apps/mono_vo.h"
#include <opencv2/opencv.hpp>

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

  if (!nh_private_.getParam ("apps/mono_vo/min_inliers", min_inliers_))
    min_inliers_ = 50;
  if (!nh_private_.getParam ("apps/mono_vo/max_iterations", max_iterations_))
    max_iterations_ = 1000;
  if (!nh_private_.getParam ("apps/mono_vo/distance_threshold", distance_threshold_))
    distance_threshold_ = 2;

  if (!nh_private_.getParam ("apps/mono_vo/max_PnP_iterations", max_PnP_iterations_))
    max_PnP_iterations_ = 10;

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

  convertPointCloudModelPointsToVector(model_ptr_);

  return true;
}


void MonocularVisualOdometry::convertPointCloudModelPointsToVector(const PointCloudFeature::Ptr model)
{
  model_cloud_vector_.clear();

  PointCloudFeature::iterator cloud_it = model->begin();
  for(; cloud_it!=model->end(); ++cloud_it)
  {
    PointFeature point_from_model = *cloud_it;
    cv::Point3d cv_cloud_point((double) point_from_model.x,  (double) point_from_model.y, (double) point_from_model.z);
    model_cloud_vector_.push_back(cv_cloud_point);
  }
}


// TODO: Roberto implements this:
void MonocularVisualOdometry::estimateMotion(const cv::Mat &E_prev, cv::Mat &E_new, const std::vector<cv::Point3d> &model, const std::vector<cv::Point2d> &features, int max_PnP_iterations)
{

}

void MonocularVisualOdometry::imageCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
//  mutex_lock_.lock();
  ros::WallTime start = ros::WallTime::now();

  // **** initialize ***************************************************
  std::vector<cv::Point2d> features_vector;
  cv::Mat E;

  if (!initialized_)
  {
    frame_ = boost::shared_ptr<MonocularFrame> (new MonocularFrame(rgb_msg, info_msg));
    ROS_INFO("RGB header = %s", rgb_msg->header.frame_id.c_str());
    initialized_ = getBaseToCameraTf(rgb_msg->header);
    init_time_ = rgb_msg->header.stamp;


    // TODO:
    // Estimate initial camera pose relative to the model
    feature_detector_->onlyFind2DFeatures(*frame_);
    frame_->getFeaturesVector(features_vector);

    E = estimateFirstPose(frame_->getIntrinsicCameraMatrix(), model_cloud_vector_, features_vector, min_inliers_, max_iterations_, distance_threshold_);

    // TODO: Roberto: create tvec and rvec with 0;0;0
    // Then, make it into E
    cv::Mat tvec;
    cv::Mat rvec;

    E = matrixFromRvecTvec(tvec, rvec);

    frame_->setExtrinsicMatrix(E);

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

  frame_->getFeaturesVector(features_vector);


  ros::WallTime start_3D_cloud_projection = ros::WallTime::now();
  cv::Mat E_new;
  estimateMotion(E, E_new, model_cloud_vector_, features_vector, max_PnP_iterations_);
  /* FIXME: take outside here:
  // ------- Perspective projection of cloud 3D points onto image plane
  // Assume known initial position of camera position at the origin of the world center of coordinates
  // TODO: needs a base2cam static transformation to correct for the camera coordinates in the world where +Z is point upwards
  // NOTE: the OpenNI driver publishes the static transformation (doing the rotation of the axis) between the rgb optical frame (/camera_rgb_optical_frame) and the /camera_link
  // ---------------------------------------------------------------
  is_first_time_projecting_ = frame_->project3DModelToCamera(model_ptr_, is_first_time_projecting_);
  */
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


bool MonocularVisualOdometry::fitness(const cv::Mat M, const cv::Mat E, const int distance_threshold, const int min_inliers, const std::vector<cv::Point3d> &sample_3D_points, const std::vector<cv::Point2d> & feature_2D_points, std::vector<cv::Point3d> &inliers_3D_points, std::vector<cv::Point2d> & inliers_2D_points)
{
  // Clean old results (if any)
  inliers_2D_points.clear();
  inliers_3D_points.clear();



  if(inliers_3D_points.size() >= min_inliers)
    return true;
  else
    return false;
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

/** min_inliers - sufficient number of inliers to terminate
  */
cv::Mat MonocularVisualOdometry::estimateFirstPose(
	const cv::Mat& intrinsic_matrix,
	const std::vector<cv::Point3d>& model,
        const std::vector<cv::Point2d>& image_2d_points,
        int min_inliers, 
        int max_iterations,
        int distance_threshold)

{
  ROS_INFO("Estimating the First Camera Pose");

  srand(time(NULL));

  std::vector<cv::Point3d> vector_3d;
  std::vector<cv::Point2d> vector_2d;
  std::vector<cv::Point3d> cloud_vector_3d;
  std::vector<cv::Point2d> cloud_vector_2d;
  std::vector<cv::Point3d> best_3d_vector;
  std::vector<cv::Point2d> best_2d_vector;
  
  ROS_WARN("resize???");

  vector_3d.resize(6);
  vector_2d.resize(6);
  bool valid_inliers = false;

  //gets 2 vectors of 6 random points from the model(3d map) and from the camera image(2d points)

  for (int i = 0; i <= max_iterations ; ++i)
  {
    for (uint j = 0; j < vector_2d.size(); ++j)
    {  
      int index1 = rand() % model.size();
      vector_3d[j] = model[index1];
      int index2 = rand() % image_2d_points.size();
//      printf("Rand %d: [%d][%d] \n", j, index1, index2);
      vector_2d[j] = image_2d_points[index2];
    }

    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat extrinsic_matrix;

    cv::solvePnP(vector_3d, vector_2d, intrinsic_matrix, cv::Mat(), rvec, tvec);
    extrinsic_matrix = matrixFromRvecTvec(rvec, tvec);


    std::vector<cv::Point3d> inliers_3D_points;
    std::vector<cv::Point2d> inliers_2D_points;


    valid_inliers = fitness(intrinsic_matrix, extrinsic_matrix, distance_threshold, min_inliers, vector_3d, vector_2d, inliers_3D_points, inliers_2D_points);


    if (valid_inliers)
    {
      best_3d_vector = inliers_3D_points;
      best_2d_vector = inliers_2D_points;
      break;
    }

  }

  //refine the transformation after getting the best fitting vectors
  cv::Mat rvec_ref;
  cv::Mat tvec_ref;
  cv::solvePnP(best_3d_vector, best_2d_vector, intrinsic_matrix, cv::Mat(), rvec_ref, tvec_ref);
  return matrixFromRvecTvec(rvec_ref, tvec_ref);
}

} //namespace ccny_rgbd
