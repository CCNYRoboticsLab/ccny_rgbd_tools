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

  testEstimationFromKeyFrames(path_to_keyframes_, initial_keyframe_number_);

  /*
  // **** init variables


  f2b_.setIdentity();

  // **** publishers
  if(publish_cloud_model_)
  {
    pub_model_ = nh_private.advertise<PointCloudFeature>(
        "model_3D", 1);
  }

  image_transport::ImageTransport it(nh_private_);
  if(publish_virtual_img_)
  {
    // %Tag(PUB)%
    publish_virtual_img_ = it.advertise(topic_virtual_image_, 1); // Publish raw image
    //        omni_img_pub_ = nh_private_.advertise<sensor_msgs::Image> (opencv_omni_topic_name_, 1);
    ROS_INFO("Publishing %s images via \"image_transport\"", topic_virtual_image_.c_str());
    // %EndTag(PUB)%
  }
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

  */
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

void MonocularVisualOdometry::testEstimationFromKeyFrames(std::string keyframe_path, int keyframe_number)
{
  std::string current_keyframe_path = keyframe_path + formKeyframeName(keyframe_number, 4);
  std::string next_keyframe_path = keyframe_path + formKeyframeName(keyframe_number+1, 4);
  std::cout << "Current Frame: " << current_keyframe_path << std::endl;
  std::cout << "Next Frame: " << next_keyframe_path << std::endl;

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

  if(current_success && next_success)
  {
    cv::namedWindow("Current", CV_WINDOW_KEEPRATIO);
    cv::imshow("Current", current_keyframe.rgb_img);
    cv::namedWindow("Next", CV_WINDOW_KEEPRATIO);
    cv::imshow("Next", next_keyframe.rgb_img);
    cv::waitKey(1);
  }

  Matrix3f intrinsic;
  openCVRToEigenR(current_keyframe.model.intrinsicMatrix(),intrinsic);
  tf::Transform transform;

  tfFromImagePair(
    current_keyframe.rgb_img,
    next_keyframe.rgb_img,
    next_keyframe.depth_img,
    intrinsic,
    transform,
    "ORB",
    "BRIEF",
    10,
    true
    );
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

  /*
  // **** frames
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/camera_link";
  if (!nh_private_.getParam ("feature/detector_type", detector_type_))
    detector_type_ = "GFT";
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
 */

  /*
  if (!nh_private_.getParam ("apps/mono_vo/min_inliers", min_inliers_))
    min_inliers_ = 50;
  if (!nh_private_.getParam ("apps/mono_vo/max_iterations", max_iterations_))
    max_iterations_ = 1000;
  if (!nh_private_.getParam ("apps/mono_vo/distance_threshold", distance_threshold_))
    distance_threshold_ = 2;

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

void MonocularVisualOdometry::setFeatureDetector()
{
  // feature params
  /*
  if (detector_type_ == "ORB")
    feature_detector_ = new OrbDetector(nh_, nh_private_);
  else if (detector_type_ == "SURF")
    feature_detector_ = new SurfDetector(nh_, nh_private_);
  else if (detector_type_ == "GFT")
    feature_detector_ = new GftDetector(nh_, nh_private_);
  else
    ROS_FATAL("%s is not a valid detector type!", detector_type_.c_str());
    */
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


void MonocularVisualOdometry::estimateMotion(
    Matrix3f& rmat, // Input/Output 3x3 rotation matrix
    Vector3f& tvec, // Input/Output 3x1 translation vector
    const PointCloudT::Ptr& model,        // full 3D model
    int max_PnP_iterations)
{
  // **************************************************

  /*//FIXME: implement !!!!
  cv::Mat intrinsic_matrix = cam_model_.intrinsicMatrix();

  cv::Mat rvec = rvecFromMatrix(E_prev);
  cv::Mat tvec = tvecFromMatrix(E_prev);

  std::vector<cv::Point3d> visible_3d_points;
  std::vector<cv::Point2d> visible_2d_points;

  E_new = E_prev;

  printf("estimateMotion...\n");

  // output 
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  std::cout << "tvec: " << std::endl << tvec << std::endl;
  std::cout << "rmat: " << std::endl << rmat << std::endl;
  bool last_iteration = false;

  for (int i=0; i < max_PnP_iterations; ++i)
  {
    printf("[iter %d] estimateMotion:\n", i);

    std::vector<cv::Point2d> vector_2d_corr;
    std::vector<cv::Point3d> vector_3d_corr;    
    
    if (i==max_PnP_iterations-1)
    {
      last_iteration = true;
    }
    bool corresp_result = getCorrespondences(visible_3d_points, features, E_new, vector_3d_corr, vector_2d_corr, last_iteration);
  
    if(corresp_result)
    {
//      cv::solvePnP(vector_3d_corr, vector_2d_corr, intrinsic_matrix, cv::Mat(), rvec, tvec, true);
      std::vector<cv::Point3f> vector_3d_corr_as_float;
      convert3DPointDoubleVectorToFloatVector(vector_3d_corr, vector_3d_corr_as_float);
      std::vector<cv::Point2f> vector_2d_corr_as_float;
      convert2DPointDoubleVectorToFloatVector(vector_2d_corr, vector_2d_corr_as_float);
      cv::solvePnPRansac(vector_3d_corr_as_float, vector_2d_corr_as_float, intrinsic_matrix, cv::Mat(), rvec, tvec, true);
      E_new = matrixFromRvecTvec(rvec, tvec);

      // output 
      std::cout << "tvec: " << std::endl << tvec << std::endl;
      std::cout << "rmat: " << std::endl << rmatFromMatrix(E_new) << std::endl;
    }
    else
    {
      // TODO: handle this better
      ROS_WARN("Could not estimate correspondences. Leaving estimateMotion loop.");
      break;
    }
  }
  */
}

bool MonocularVisualOdometry::getCorrespondences(
  const std::vector<cv::Point3d>& visible_3d_points, 
  const std::vector<cv::Point2d>& features_2D, 
  const cv::Mat &E, 
  std::vector<cv::Point3d> &corr_3D_points, 
  std::vector<cv::Point2d> &corr_2D_points, 
  bool last_iteration)
{
  /*
  // Clean old results (if any)
  corr_2D_points.clear();
  corr_3D_points.clear();

  // project visible 3d to 2d
  std::vector<cv::Point2d> projected_model_2D_points;
  project3DTo2D(visible_3d_points, E, cam_model_.intrinsicMatrix(), projected_model_2D_points);

  std::vector<cv::Point2d> train_points_vector; // Points in the KD-tree
  std::vector<cv::Point2d> query_points_vector;
  cv::Mat query_points_matrix;
  std::vector<int> match_indices;
  std::vector<float> match_distances; // ATTENTION: don't use double, otherwise the KDTree will crash
  double total_distance_error = 0.0; // Accumulator error of distances to nearest neighbor

  train_points_vector = projected_model_2D_points;
  query_points_vector = features_2D;

  cv::Mat train_points_matrix;
  convert2DPointVectorToMatrix(train_points_vector, train_points_matrix, CV_32FC1);

  // Create KD-Tree:
  // ---------------------------------------------------------------------------------
  cv::flann::Index kd_tree;
  if(number_of_random_trees_ > 1)
  {
    // KdTree with 5 random trees
    cv::flann::KDTreeIndexParams indexParams(number_of_random_trees_);
    // Create the Index
    kd_tree.build(train_points_matrix, indexParams);
  }
  else// You can also use LinearIndex
  {
    cv::flann::LinearIndexParams indexParams;
    // Create the Index
    kd_tree.build(train_points_matrix, indexParams);
  }
  // ---------------------------------------------------------------------------------

  convert2DPointVectorToMatrix(query_points_vector, query_points_matrix, CV_32FC1);
  if(getMatches(kd_tree, query_points_matrix, match_indices, match_distances, false))
  {
    cv::vector<cv::DMatch> good_matches; // vector of matches where distance is below threshold

    // Filter points according to threshold distance
    for(uint i = 0 ; i < match_indices.size(); i++)
    {
      if(match_distances[i] < (float) distance_threshold_)
      {
        int query_idx = i;
        int train_idx = match_indices[query_idx];

//        corr_2D_points.push_back(features_2D[query_idx]);
        corr_2D_points.push_back(query_points_vector[query_idx]);
        corr_3D_points.push_back(visible_3d_points[train_idx]);

        total_distance_error += (double) match_distances[query_idx];
        good_matches.push_back(cv::DMatch(query_idx, train_idx, match_distances[query_idx]));
      }
    }
    printf("Found %d good matches out of %d train-points (projected points) and %d query-points (detected features)\n", corr_2D_points.size(), train_points_vector.size(), query_points_vector.size());

   if ((visualize_correspondences_) && (last_iteration==true))
    {
      std::vector<cv::KeyPoint> train_keypoints;
      std::vector<cv::KeyPoint> query_keypoints;
      std::vector<cv::Point2f> train_points_as_floats;
      std::vector<cv::Point2f> query_points_as_floats;

      convert2DPointDoubleVectorToFloatVector(train_points_vector, train_points_as_floats);
      cv::KeyPoint::convert(train_points_as_floats, train_keypoints);
      convert2DPointDoubleVectorToFloatVector(query_points_vector, query_points_as_floats);
      cv::KeyPoint::convert(query_points_as_floats, query_keypoints);

      cv::Mat train_points_img, query_points_img;
      train_points_img = frame_->getRGBImage()->clone();
      query_points_img = frame_->getRGBImage()->clone();

      // Draw matches
      cv::Mat good_matches_img;
      cv::drawMatches(query_points_img, query_keypoints, // (QUERY) 1nd image and its keypoints
                      train_points_img, train_keypoints,  // (TRAIN) 2st image and its keypoints
                      good_matches, // the matches
                      good_matches_img); // the image produced

      cv::namedWindow("Good matches", CV_WINDOW_KEEPRATIO);
      cv::imshow("Good matches", good_matches_img);
      cv::waitKey(1);
    }

    if(corr_2D_points.size() > 0)
      return true;
    else
    {
      ROS_WARN("No correspondences found!");
      return false;
    }
  }

*/
  return true;
}


void MonocularVisualOdometry::imageCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  ros::WallTime start = ros::WallTime::now();

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
    // TODO:
    // Estimate initial camera pose relative to the model
    estimateFirstPose(intrinsic_matrix_, rmat_, tvec_, model_ptr_, min_inliers_, max_iterations_, distance_threshold_);

    printf("Initialization successful at Frame %d\n", frame_count_);
  }

  cv::Mat rgb_img;
  cv::Mat depth_img;
  projectCloudToImage(model_ptr_, rmat_, tvec_, intrinsic_matrix_, (int) cam_model_.width(), (int) cam_model_.height(), rgb_img, depth_img);
  // **** estimate motion ************************************************

    ros::WallTime start_PnP_reg = ros::WallTime::now();
    cv::Mat E_new;
    estimateMotion(rmat_, tvec_, model_ptr_, max_PnP_iterations_);
    ros::WallTime end_PnP_reg = ros::WallTime::now();

    // Update extrinsic matrix
    std::cout << "Computed Translation vector at frame " << frame_count_ << ": " << tvec_ << std::endl;

    // **** publish motion **********************************************
    // Publish f2b = [b2c * f2c^-1]^-1   where f2c^-1=E
    tf::Transform f2c_inv = tfFromEigenRt(rmat_, tvec_);
    tf::Transform f2b_inv = (b2c_ * f2c_inv);
    f2b_ = f2b_inv.inverse();
    publishTransformF2B(rgb_msg->header);
/*
    ros::WallTime end = ros::WallTime::now();
    // **** print diagnostics *******************************************

    //  int n_features = frame.features.points.size();
    int n_keypoints = frame_->keypoints.size();

    double d_features = 1000.0 * (end_features - start_features).toSec();
    double d_PnP_pose_est      = 1000.0 * (end_PnP_reg      - start_PnP_reg).toSec();
    double d_total    = 1000.0 * (end          - start).toSec();

    ROS_INFO("[%d] Fr: %2.1f s %s[%d keyspoints]: %3.1f s  \t Pose Est: %4.1f s \t TOTAL %4.1f s\n",
             frame_count_,
             detector_type_.c_str(),
             n_keypoints, d_features,
             d_PnP_pose_est,
             d_total);

    double pos_x = f2b_.getOrigin().getX();
    double pos_y = f2b_.getOrigin().getY();
    double pos_z = f2b_.getOrigin().getZ();
    // for position profiling
    printf("%d \t %.2f \t %.3f \t %.3f \t %.3f \n",
      frame_count_, time,
      pos_x, pos_y, pos_z);
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
     */
    frame_count_++;
  //}

  if(publish_cloud_model_)
  {
    printf("Publishing model cloud read from PCD\n");
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
    cv_virtual_img.image = rgb_img; // Your cv::Mat
    ros_virtual_img_ptr = cv_virtual_img.toImageMsg();
    virtual_img_pub_.publish(ros_virtual_img_ptr);
  }
//  mutex_lock_.unlock();
}


bool MonocularVisualOdometry::fitness(const cv::Mat M, const cv::Mat E, const int distance_threshold, const int min_inliers, const std::vector<cv::Point3d> &sample_3D_points, const std::vector<cv::Point2d> & feature_2D_points, std::vector<cv::Point3d> &inliers_3D_points, std::vector<cv::Point2d> & inliers_2D_points)
{
/*
  // Clean old results (if any)
  inliers_2D_points.clear();
  inliers_3D_points.clear();

  // FIXME: reanalized if filtering of points within frame is needed?
  std::vector<cv::Point2d> projected_model_2D_points;
  std::vector<cv::Point3d> valid_3D_points;
  std::vector<cv::Point2d> valid_2D_points;

   project3DTo2D(sample_3D_points, E, frame_->getIntrinsicCameraMatrix(), projected_model_2D_points);

  frame_->filterPointsWithinFrame(sample_3D_points, projected_model_2D_points, valid_3D_points, valid_2D_points);

  std::vector<int> match_indices;
  std::vector<float> match_distances;
  cv::Mat detected_points_matrix;
  cv::Mat projected_points_matrix;
  convert2DPointVectorToMatrix(feature_2D_points, detected_points_matrix, CV_32FC1); // Type supported by FLANN is float only
  convert2DPointVectorToMatrix(valid_2D_points, projected_points_matrix, CV_32FC1); // Type supported by FLANN is float only
  getMatches(detected_points_matrix, projected_points_matrix, match_indices, match_distances);

  // Filter points according to threshold distance
  for(uint i = 0 ; i < match_indices.size(); i++)
  {
    if(match_distances[i] < distance_threshold)
    {
      inliers_2D_points.push_back(valid_2D_points[match_indices[i]]);
      inliers_3D_points.push_back(valid_3D_points[match_indices[i]]);
    }
  }

  if(inliers_3D_points.size() >= (uint) min_inliers)
    return true;
  else
    return false;
*/
  return false;
}

void MonocularVisualOdometry::publishTransformF2B(const std_msgs::Header& header)
{
  ROS_INFO("Transforming Fixed Frame (%s) to Base (%s)", fixed_frame_.c_str(), base_frame_.c_str());

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

bool MonocularVisualOdometry::getMatches(
  cv::flann::Index& kd_tree,
  const cv::Mat& query_points,          // 2d visible query points
  std::vector<int>& match_indices,
  std::vector<float>& match_distances,
  bool prune_repeated_matches)
{
  // Perform knn search
  cv::Mat indices, dists;
  kd_tree.knnSearch(query_points, indices, dists, 1, cv::flann::SearchParams(64));
  printf("[matching] query_pts: %d\n", query_points.rows);

  // transfirm from Mat to std::vector
  match_indices.clear();
  match_distances.clear();
  match_indices.resize(indices.rows);
  match_distances.resize(dists.rows);
  for(int row = 0 ; row < indices.rows ; row++)
  {
    match_indices[row]   = indices.at<int>(row, 0);
    match_distances[row] = dists.at<float>(row, 0);
  }

/*
  if(prune_repeated_matches)
  {
    // TODO: prune matches
    std::vector<int> match_indices_pruned;
    std::vector<float> match_distances_pruned;
    pruneMatches(match_indices, match_distances, match_indices_pruned, match_distances_pruned);
  }
*/
  if(match_indices.size() > 0)
    return true;
  else
    return false;
}



/** min_inliers - sufficient number of inliers to terminate
  */
void MonocularVisualOdometry::estimateFirstPose(
	const Matrix3f& intrinsic_matrix,
	Matrix3f& rmat, // Output 3x3 rotation matrix
	Vector3f& tvec, // Output 3x1 translation vector
	const PointCloudT::Ptr& cloud,
        int min_inliers, 
        int max_iterations,
        int distance_threshold)
{
  ROS_INFO("Estimating Initial Camera Pose");

  if(assume_initial_position_ == false)
  {
    /*// TODO: re-implement
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
        vector_2d[j] = image_2d_points[index2];
      }

      cv::Mat rvec;
      cv::Mat tvec;
      cv::Mat intrinsic_matrix = frame_->getIntrinsicCameraMatrix();
      cv::solvePnP(vector_3d, vector_2d, intrinsic_matrix, cv::Mat(), rvec, tvec);
      cv::Mat extrinsic_matrix = matrixFromRvecTvec(rvec, tvec);

      std::vector<cv::Point3d> inliers_3D_points;
      std::vector<cv::Point2d> inliers_2D_points;

      // FIXME: CHECKME
      valid_inliers = fitness(intrinsic_matrix, extrinsic_matrix, distance_threshold, min_inliers, vector_3d, vector_2d, inliers_3D_points, inliers_2D_points);
      std::cout << i << ": rvec_: " << rvec << std::endl;
      std::cout << "tvec: " << tvec << std::endl;


      if (valid_inliers)
      {
        printf("Valid inliers at %d\n", i);
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
    */
  }
  else
  {
//    cv::Mat tvec = (cv::Mat_<double> (3,1) << 0.0, 0.0, 0.0);
//    cv::Mat rvec = (cv::Mat_<double> (3,1) << 0.0, 0.0, 0.0);
    tfToEigenRt(b2c_.inverse(), rmat, tvec);
  }
}

} //namespace ccny_rgbd
