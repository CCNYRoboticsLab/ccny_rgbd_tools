#include "ccny_rgbd/apps/mono_vo.h"

namespace ccny_rgbd {

MonocularVisualOdometry::MonocularVisualOdometry(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  initialized_(false),
  frame_count_(0)
{
  ROS_INFO("Starting Monocular Visual Odometry from a 3D Dense Model");
  // **** init parameters
  initParams();
  f2b_.setIdentity();

  // **** publishers
  if(publish_cloud_model_)
  {
    pub_model_ = nh_private.advertise<PointCloudT>(
        "model_3D", 1);
    pub_cloud_est_ = nh_private.advertise<PointCloudT>(
        "cloud_est", 1);
  }

  odom_publisher_ = nh_.advertise<OdomMsg>(
    "odom", 5);

  if(readPointCloudFromPCDFile()==false)
    ROS_FATAL("The sky needs its point cloud to operate!");

  /*
  image_transport::ImageTransport it(nh_private_);
  if(publish_virtual_img_)
  {
    // %Tag(PUB)%
    publish_virtual_img_ = it.advertise(topic_virtual_image_, 1); // Publish raw image
    //        omni_img_pub_ = nh_private_.advertise<sensor_msgs::Image> (opencv_omni_topic_name_, 1);
    ROS_INFO("Publishing %s images via \"image_transport\"", topic_virtual_image_.c_str());
    // %EndTag(PUB)%
  }

  */

  // **** subscribers
  image_transport::ImageTransport rgb_it(nh_);
  sub_rgb_.subscribe(
    rgb_it, topic_image_, 1);
  sub_info_.subscribe(
    nh_, topic_cam_info_, 1);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  /*
  int queue_size = 5;
  sync_.reset(new SynchronizerMonoVO(SyncPolicyMonoVO(queue_size), sub_rgb_, sub_info_));
  sync_->registerCallback(boost::bind(&MonocularVisualOdometry::imageCallback, this, _1, _2));
  */
  //  testEstimationFromKeyFrames(path_to_keyframes_, initial_keyframe_number_);
    testEstimationFromVirtualKeyFrames(path_to_keyframes_, initial_keyframe_number_);


}

MonocularVisualOdometry::~MonocularVisualOdometry()
{
  ROS_INFO("Destroying Monocular Visual Odometry");
}

std::string MonocularVisualOdometry::formKeyframeName(int keyframe_number, int num_of_chars)
{
  std::stringstream ss;
  ss << keyframe_number;
  std::string keyframe_number_as_str = ss.str();
  while(keyframe_number_as_str.size() < (uint) num_of_chars)
    keyframe_number_as_str = "0" + keyframe_number_as_str;

  return keyframe_number_as_str;
}

void MonocularVisualOdometry::generateKeyframePaths(const std::string& keyframe_path, int keyframe_number, std::string& current_keyframe_path, std::string& next_keyframe_path)
{
  current_keyframe_path = keyframe_path + formKeyframeName(keyframe_number, 4);
  next_keyframe_path = keyframe_path + formKeyframeName(keyframe_number+1, 4);
  std::cout << "Current Frame: " << current_keyframe_path << std::endl;
  std::cout << "Next Frame: " << next_keyframe_path << std::endl;
}

void MonocularVisualOdometry::getVirtualImageFromKeyframe(const PointCloudT& cloud, const Matrix3f& intrinsic, const tf::Transform& extrinsic_tf, cv::Mat& rgb_img, cv::Mat& depth_img)
{
  Matrix3f rmat;
  Vector3f tvec;
  tfToEigenRt(extrinsic_tf, rmat, tvec);

  cv::Mat rgb_img_projected;
  cv::Mat depth_img_projected;

  projectCloudToImage(cloud, rmat, tvec, intrinsic, image_width_, image_height_, rgb_img_projected, depth_img_projected);

  holeFilling2(rgb_img_projected, depth_img_projected, 3, rgb_img, depth_img);

//  cv::medianBlur(rgb_img,rgb_img, 3);
  double sigmaX = 1.5;
  cv::GaussianBlur(rgb_img,rgb_img, cv::Size(3, 3), sigmaX);

}

void MonocularVisualOdometry::testEstimationFromKeyFrames(std::string keyframe_path, int keyframe_number)
{
  tf::Transform transform_est; // Frame to frame

  std::string current_keyframe_path;
  std::string next_keyframe_path;
  generateKeyframePaths(keyframe_path, keyframe_number, current_keyframe_path, next_keyframe_path);

  bool current_success = false;
  bool next_success = false;

  RGBDKeyframe current_keyframe, next_keyframe;
  current_success = loadKeyframe(current_keyframe, current_keyframe_path);
  if(current_success)
  {
    if(loadKeyframe(next_keyframe, next_keyframe_path))
    {
      next_success = true;
    }
    else
    {
      int next_keyframe_number = 0; // Wraps around to the first frame
      while(next_success == false)
      {
        next_success = loadKeyframe(next_keyframe, keyframe_path + formKeyframeName(next_keyframe_number, 4));
        next_keyframe_number++;
      }
    }
  }

  cv::Mat cv_intrinsic = current_keyframe.model.intrinsicMatrix();
  //std::cout << "CV Intrinsic matrix: " << cv_intrinsic <<std::endl;

  Matrix3f intrinsic;
  float scale_factor_intrinsic = 1.0;
  if(next_success)
  {
    cv::Mat cv_intrinsic = next_keyframe.model.intrinsicMatrix();
    //std::cout << "CV Intrinsic matrix: " << cv_intrinsic <<std::endl;
    openCVRToEigenR(cv_intrinsic,intrinsic);
    // Rescale intrinsice
    scale_factor_intrinsic =  (float) image_width_ / (float) next_keyframe.rgb_img.cols;
    printf("Scale factor = %f \n", scale_factor_intrinsic);
    intrinsic(0,0) = scale_factor_intrinsic*intrinsic(0,0); // fx
    intrinsic(0,2) = scale_factor_intrinsic*intrinsic(0,2); // cx
    intrinsic(1,1) = scale_factor_intrinsic*intrinsic(1,1); // fy
    intrinsic(1,2) = scale_factor_intrinsic*intrinsic(1,2); // cy
  }

  f2b_ = current_keyframe.pose; // Initialized to first keyframe's position // TODO: temporarily

  while(current_success && next_success)
  {
    cv::Mat new_img;
    cv::resize(next_keyframe.rgb_img, new_img,cv::Size(image_width_, image_height_) );

    tfFromImagePair(
      current_keyframe.rgb_img,
      new_img,
      current_keyframe.depth_img,
      intrinsic,
      transform_est,
      max_descriptor_space_distance_,
      detector_type_,
      descriptor_type_,
      number_of_iterations_,
      (float) reprojection_error_,
      min_inliers_count_,
      true,
      true // profiling delays
      );
    // Compare to ground truth
    tf::Transform transform_gt = (current_keyframe.pose).inverse() * next_keyframe.pose;

    // output
    cv::Mat tvec_est, rmat_est;
    tfToOpenCVRt(transform_est, rmat_est, tvec_est);
    std::cout << "ESTIMATION:" << std::endl;
    std::cout << "tvec:" << tvec_est << std::endl << "rmat:" << rmat_est << std::endl;

    cv::Mat tvec_gt, rmat_gt;
    tfToOpenCVRt(transform_gt, rmat_gt, tvec_gt);
    std::cout << "GROUND TRUTH:" << std::endl;
    std::cout << "tvec:" << tvec_gt << std::endl << "rmat:" << rmat_gt << std::endl;

    // Error metric:
    double dist, angle;
    getTfDifference(transform_gt, transform_est, dist, angle);
    std::cout << "ERROR:" << std::endl;
    std::cout << "\t Distance difference: " << dist << " m" <<
    std::endl << "\t Angle difference: " << RAD2DEG(angle) << " degrees" << std::endl;


    cv::waitKey(0);

    f2b_ = f2b_ * transform_est;
    publishTransform(f2b_, fixed_frame_, base_frame_);

    if(publish_cloud_model_)
    {
      PointCloudT cloud_next = next_keyframe.cloud;
//      cloud_next.header.frame_id = base_frame_;
      cloud_next.header.frame_id = fixed_frame_; // FIXME: Using fixed_problem to publish and visualize in the "odom" frame without need for transformation
      std::string cloud_filename_next_est = current_keyframe_path + "_next_est.pcd";
      // derotate to fixed frame if needed
      PointCloudT cloud_next_transformed_est;
      pcl::transformPointCloud(cloud_next, cloud_next_transformed_est, eigenFromTf(f2b_));
      pub_cloud_est_.publish(cloud_next_transformed_est);
    }
    // +++++++++++++++++++++++++ Advance frames  ++++++++++++++++++++++++++++++++++++++++

    ++keyframe_number;
    generateKeyframePaths(keyframe_path, keyframe_number, current_keyframe_path, next_keyframe_path);

    current_success = loadKeyframe(current_keyframe, current_keyframe_path);
    if(current_success)
    {
      if(loadKeyframe(next_keyframe, next_keyframe_path))
      {
        next_success = true;
      }
      else
      {
        int next_keyframe_number = 0; // Wraps around to the first frame
        while(next_success == false)
        {
          next_success = loadKeyframe(next_keyframe, keyframe_path + formKeyframeName(next_keyframe_number, 4));
          next_keyframe_number++;
        }
      }
    }
  }

}

void MonocularVisualOdometry::testEstimationFromVirtualKeyFrames(std::string keyframe_path, int keyframe_number)
{
  tf::Transform transform_est; // Frame to frame

  std::string current_keyframe_path;
  std::string next_keyframe_path;
  generateKeyframePaths(keyframe_path, keyframe_number, current_keyframe_path, next_keyframe_path);

  bool current_success = false;
  bool next_success = false;

  RGBDKeyframe current_keyframe, next_keyframe;
  current_success = loadKeyframe(current_keyframe, current_keyframe_path);
  if(current_success)
  {
    if(loadKeyframe(next_keyframe, next_keyframe_path))
    {
      next_success = true;
    }
    else
    {
      int next_keyframe_number = 0; // Wraps around to the first frame
      while(next_success == false)
      {
        next_success = loadKeyframe(next_keyframe, keyframe_path + formKeyframeName(next_keyframe_number, 4));
        next_keyframe_number++;
      }
    }
  }

  Matrix3f intrinsic;
  float scale_factor_intrinsic = 1.0;
  if(next_success)
  {
    cv::Mat cv_intrinsic = next_keyframe.model.intrinsicMatrix();
    //std::cout << "CV Intrinsic matrix: " << cv_intrinsic <<std::endl;
    openCVRToEigenR(cv_intrinsic,intrinsic);
    // Rescale intrinsice
    scale_factor_intrinsic =  (float) image_width_ / (float) next_keyframe.rgb_img.cols;
    printf("Scale factor = %f \n", scale_factor_intrinsic);
    intrinsic(0,0) = scale_factor_intrinsic*intrinsic(0,0); // fx
    intrinsic(0,2) = scale_factor_intrinsic*intrinsic(0,2); // cx
    intrinsic(1,1) = scale_factor_intrinsic*intrinsic(1,1); // fy
    intrinsic(1,2) = scale_factor_intrinsic*intrinsic(1,2); // cy
  }

  f2b_ = current_keyframe.pose; // Initialized to first keyframe's position

  while(current_success && next_success)
  {
    cv::Mat virtual_rgb_img, virtual_depth_img;
    ros::WallTime start_projection = ros::WallTime::now();
    // Function being profiled
    getVirtualImageFromKeyframe(*model_ptr_, intrinsic, f2b_.inverse(), virtual_rgb_img, virtual_depth_img);
    printf("Projection delay =  %f ms\n", getMsDuration(start_projection));

    cv::Mat new_img;
    cv::resize(next_keyframe.rgb_img, new_img,cv::Size(image_width_, image_height_) );
    tfFromImagePair(
        virtual_rgb_img,
        new_img,
        virtual_depth_img,
        intrinsic,
        transform_est,
        max_descriptor_space_distance_,
        detector_type_,
        descriptor_type_,
        number_of_iterations_,
        (float) reprojection_error_,
        min_inliers_count_,
        true,
        true // profiling delays
    );

    /*
    // Compare to ground truth
    tf::Transform transform_gt = (current_keyframe.pose).inverse() * next_keyframe.pose;

    // output
    cv::Mat tvec_est, rmat_est;
    tfToOpenCVRt(transform_est, rmat_est, tvec_est);
    std::cout << "ESTIMATION:" << std::endl;
    std::cout << "tvec:" << tvec_est << std::endl << "rmat:" << rmat_est << std::endl;

    cv::Mat tvec_gt, rmat_gt;
    tfToOpenCVRt(transform_gt, rmat_gt, tvec_gt);
    std::cout << "GROUND TRUTH:" << std::endl;
    std::cout << "tvec:" << tvec_gt << std::endl << "rmat:" << rmat_gt << std::endl;

    // Error metric:
    double dist, angle;
    getTfDifference(transform_gt, transform_est, dist, angle);
    std::cout << "ERROR:" << std::endl;
    std::cout << "\t Distance difference: " << dist << " m" <<
    std::endl << "\t Angle difference: " << RAD2DEG(angle) << " degrees" << std::endl;
    */
    f2b_ = f2b_ * transform_est;
    publishTransform(f2b_, fixed_frame_, base_frame_);

    if(publish_cloud_model_)
    {
      PointCloudT cloud_next = next_keyframe.cloud;
      cloud_next.header.frame_id = base_frame_;

      bool write_PCDs = false;
      if(write_PCDs)
      {
        std::string cloud_filename_next_est = current_keyframe_path + "_next_est.pcd";
        // derotate to fixed frame if needed
        PointCloudT cloud_next_transformed_est;
        cloud_next.header.frame_id = fixed_frame_;

        pcl::transformPointCloud(cloud_next, cloud_next_transformed_est, eigenFromTf(f2b_));

        pcl::PCDWriter writer;
        int result_pcd_next;
        result_pcd_next = writer.writeBinary<PointT>(cloud_filename_next_est, cloud_next_transformed_est);
        pub_cloud_est_.publish(cloud_next_transformed_est);

      }
//      pub_cloud_est_.publish(cloud_next);
      pub_model_.publish(*model_ptr_);
    }

    // +++++++++++++++++++++++++ Advance frames  ++++++++++++++++++++++++++++++++++++++++
    ++keyframe_number;
    generateKeyframePaths(keyframe_path, keyframe_number, current_keyframe_path, next_keyframe_path);

    current_success = loadKeyframe(current_keyframe, current_keyframe_path);
    if(current_success)
    {
      if(loadKeyframe(next_keyframe, next_keyframe_path))
      {
        next_success = true;
      }
      else
      {
        int next_keyframe_number = 0; // Wraps around to the first frame
        while(next_success == false)
        {
          next_success = loadKeyframe(next_keyframe, keyframe_path + formKeyframeName(next_keyframe_number, 4));
          next_keyframe_number++;
        }
      }
    }
  }


}

void MonocularVisualOdometry::estimatePose(const PointCloudT::Ptr& model_cloud, const cv::Mat& mono_img, cv::Mat& virtual_img)
{
  tf::Transform transform_est; // Frame to frame

  cv::Mat virtual_depth_img;
  ros::WallTime start_projection = ros::WallTime::now();
  // Function being profiled
  getVirtualImageFromKeyframe(*model_cloud, intrinsic_matrix_, f2b_.inverse(), virtual_img, virtual_depth_img);
  printf("Projection delay =  %f ms\n", getMsDuration(start_projection));

  cv::Mat new_img;
  cv::resize(mono_img, new_img,cv::Size(image_width_, image_height_) );
  tfFromImagePair(
      virtual_img,
      new_img,
      virtual_depth_img,
      intrinsic_matrix_,
      transform_est,
      max_descriptor_space_distance_,
      detector_type_,
      descriptor_type_,
      number_of_iterations_,
      (float) reprojection_error_,
      min_inliers_count_,
      true,
      true // profiling delays
  );

  f2b_ = f2b_ * transform_est;
  publishTransform(f2b_, fixed_frame_, base_frame_);
}

void MonocularVisualOdometry::initParams()
{
//  testKDTree();

  // PCD File
  if(!nh_private_.getParam("apps/mono_vo/PCD_filename", pcd_filename_))
    pcd_filename_ = "cloud.pcd";

  if (!nh_private_.getParam ("apps/mono_vo/path_to_keyframes", path_to_keyframes_))
    path_to_keyframes_ = "~/ros/Keyframes";
  if (!nh_private_.getParam ("apps/mono_vo/initial_keyframe_number", initial_keyframe_number_))
    initial_keyframe_number_ = 0;

  // **** frames
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/camera_link";
  if (!nh_private_.getParam ("apps/mono_vo/detector_type", detector_type_))
    detector_type_ = "ORB";
  if (!nh_private_.getParam ("apps/mono_vo/descriptor_type", descriptor_type_))
    descriptor_type_ = "ORB";
  if (!nh_private_.getParam ("apps/mono_vo/max_descriptor_space_distance", max_descriptor_space_distance_))
    max_descriptor_space_distance_ = 0.5;
  if (!nh_private_.getParam ("apps/mono_vo/image_width", image_width_))
    image_width_ = 320;
  if (!nh_private_.getParam ("apps/mono_vo/image_height", image_height_))
    image_height_ = 240;
  if (!nh_private_.getParam ("apps/mono_vo/min_inliers_count", min_inliers_count_))
    min_inliers_count_ = 70;
  if (!nh_private_.getParam ("apps/mono_vo/number_of_iterations", number_of_iterations_))
    number_of_iterations_ = 1000;
  if (!nh_private_.getParam ("apps/mono_vo/reprojection_error", reprojection_error_))
    reprojection_error_ = 16;


  if (!nh_private_.getParam ("apps/mono_vo/publish_cloud_model", publish_cloud_model_))
    publish_cloud_model_ = false;
  if (!nh_private_.getParam ("apps/mono_vo/publish_virtual_img", publish_virtual_img_))
    publish_virtual_img_ = false;


  if (!nh_private_.getParam ("apps/mono_vo/topic_cam_info", topic_cam_info_))
    topic_cam_info_ = "/camera/rgb/camera_info";
  if (!nh_private_.getParam ("apps/mono_vo/topic_image", topic_image_))
    topic_image_ = "/camera/rgb/image_rect_color";
  if (!nh_private_.getParam ("apps/mono_vo/topic_virtual_image", topic_virtual_image_))
    topic_virtual_image_ = "/camera/rgb/virtual";

/*
  if (!nh_private_.getParam ("apps/mono_vo/max_PnP_iterations", max_PnP_iterations_))
    max_PnP_iterations_ = 10;
  if (!nh_private_.getParam ("apps/mono_vo/number_of_random_trees", number_of_random_trees_))
    number_of_random_trees_ = 1;
  if (!nh_private_.getParam ("apps/mono_vo/assume_initial_position", assume_initial_position_))
    assume_initial_position_ = true;
  if (!nh_private_.getParam ("apps/mono_vo/visualize_correspondences", visualize_correspondences_))
    visualize_correspondences_ = false;
   */

  ROS_INFO("Parameters initialized.");

}

bool MonocularVisualOdometry::readPointCloudFromPCDFile()
{
//  cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  model_ptr_ = PointCloudT::Ptr(new PointCloudT);

  if (pcl::io::loadPCDFile<PointT> (pcd_filename_, *model_ptr_) == -1) //* load the file
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

  return true;
}


void MonocularVisualOdometry::imageCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{

  ROS_INFO("Processing frame %d", frame_count_);
  // **** initialize ***************************************************
  if (!initialized_)
  {
    ROS_INFO("RGB header = %s", rgb_msg->header.frame_id.c_str());
    initialized_ = getBaseToCameraTf(rgb_msg->header);
    init_time_ = rgb_msg->header.stamp;

    if (!initialized_) return;

    cam_model_.fromCameraInfo(info_msg);
    openCVRToEigenR(cam_model_.intrinsicMatrix(), intrinsic_matrix_);
    float scale_factor_intrinsic = 1.0;
    // Rescale intrinsice
    scale_factor_intrinsic =  (float) image_width_ / (float) cam_model_.width();
    printf("Scale factor = %f \n", scale_factor_intrinsic);
    intrinsic_matrix_(0,0) = scale_factor_intrinsic*intrinsic_matrix_(0,0); // fx
    intrinsic_matrix_(0,2) = scale_factor_intrinsic*intrinsic_matrix_(0,2); // cx
    intrinsic_matrix_(1,1) = scale_factor_intrinsic*intrinsic_matrix_(1,1); // fy
    intrinsic_matrix_(1,2) = scale_factor_intrinsic*intrinsic_matrix_(1,2); // cy


    // TODO:
    // Estimate initial camera pose relative to the model
    f2b_ = b2c_.inverse(); // Initialized to first keyframe's position

    printf("Initialization successful at Frame %d\n", frame_count_);
  }

  cv::Mat rgb_img = cv_bridge::toCvShare(rgb_msg)->image;

  cv::Mat virtual_img;

  // Process frame for position estimation
  estimatePose(model_ptr_, rgb_img, virtual_img);

  if(publish_cloud_model_)
  {
    pub_model_.publish(*model_ptr_);
  }

  if(publish_virtual_img_)
  {
    sensor_msgs::ImagePtr ros_virtual_img_ptr;
    cv_bridge::CvImage cv_virtual_img;
    cv_virtual_img.header.frame_id = "virtual_img_frame"; // Same timestamp and tf frame as input image
    cv_virtual_img.header.stamp = ros::Time::now(); // Same timestamp and tf frame as input image
    //                  cv_img_frame.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
    if (rgb_img.type() == CV_8UC3)
      cv_virtual_img.encoding = sensor_msgs::image_encodings::TYPE_8UC3; // Or whatever
    else if (rgb_img.type() == CV_8UC1)
      cv_virtual_img.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    cv_virtual_img.image = virtual_img; // Your cv::Mat
    ros_virtual_img_ptr = cv_virtual_img.toImageMsg();
    virtual_img_pub_.publish(ros_virtual_img_ptr);
  }

  frame_count_++;
}


void MonocularVisualOdometry::publishTransform(const tf::Transform &source2target_transform, const std::string& source_frame_id, const std::string& target_frame_id)
{
  ROS_INFO("Transforming Fixed Frame (%s) to Base (%s)", source_frame_id.c_str(), target_frame_id.c_str());

  ros::Time current_time = ros::Time::now();
  tf::StampedTransform transform_msg(
      source2target_transform, current_time, source_frame_id, target_frame_id);
  tf_broadcaster_.sendTransform (transform_msg);

  ROS_WARN("Successfully sent transform Fixed (%s) to Base (%s)", source_frame_id.c_str(), target_frame_id.c_str());

  OdomMsg odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = source_frame_id;
  tf::poseTFToMsg(source2target_transform, odom.pose.pose);
  odom_publisher_.publish(odom);
}

bool MonocularVisualOdometry::getBaseToCameraTf(const std_msgs::Header& header)
{
  tf::StampedTransform tf_m;

  ROS_INFO("Transforming Base (%s) to Camera (%s)", base_frame_.c_str(), header.frame_id.c_str());
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
