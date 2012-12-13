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

void testKDTree()
{
  /*
  cv::KDTree my_tree;
//  cv::Mat input = (cv::Mat_<float> (4,2) << 10.0, 0,
//                                              3.2, 0,
//                                              0, 1.1,
//                                              0, 2);
//  cv::Mat input = (cv::Mat_<float> (4,1) << 10.0,
//  cv::Mat input = (cv::Mat_<double> (4,1) << 10.0,
//                                              3.2,
//                                              0,
//                                              2);
  std::vector<double> input(4);
  input[0] = 10.0;
  input[1] = 0.0;
  input[2] = 4.0;
  input[3] = 1.0;

  for(int i=0; i<input.size(); i++)
    std::cout << "Input to KDTree" << input[i] << std::endl;
  my_tree.build(input, false);

  std::vector<int> indices;
  indices.push_back(0);
  indices.push_back(3);
  cv::Mat pts;
  ROS_WARN("FUCK");
  my_tree.getPoints(indices, pts);
  ROS_WARN("WTF?");
  std::cout << "Output pts from KDTree" << pts << std::endl;


  const int K = 2, Emax = INT_MAX;
  int idx[K];
  float dist[K];
//  my_tree.findNearest(query_vec, K, Emax, idx, 0, dist);
*/
  // Parse input
//    parseInput(argc, argv);

  using namespace std;
  using namespace cv;

    int numData = 5;
    int numQueries = 2;
    int numDimensions = 2;
    int k = 1;

    // Create the data
//    cv::Mat features(numData,numDimensions,CV_32F), query(numQueries,numDimensions,CV_32F);
//    cv::randu(features, Scalar::all(Mean), cv::Scalar::all(Variance));
//    cv::randu(query, Scalar::all(Mean), cv::Scalar::all(Variance));

      cv::Mat features = (cv::Mat_<float> (numData,numDimensions) << 10.0, 0,
                                                        3.2, 0,
                                                        0, 1.1,
                                                        0, 2,
                                                        6.2, 30);
      cv::Mat query = (cv::Mat_<float> (numQueries,numDimensions) << 9.0, 20,
                                                  6,5);

    // Print generated data
    std::cout << "Input::" << std::endl;
    for(int row = 0 ; row < features.rows ; row++){
            for(int col = 0 ; col < features.cols ; col++){
                    std::cout << features.at<float>(row,col) <<"\t";
            }
            std::cout << endl;
    }
    std::cout << "Query::" << std::endl;
    for(int row = 0 ; row < query.rows ; row++){
            for(int col = 0 ; col < query.cols ; col++){
                    cout << query.at<float>(row,col) <<"\t";
            }
            cout << endl;
    }

    // KdTree with 5 random trees
    cv::flann::KDTreeIndexParams indexParams(5);

    // You can also use LinearIndex
    //cv::flann::LinearIndexParams indexParams;

    // Create the Index
    cv::flann::Index kdtree(features, indexParams);
/*
    // Perform single search for mean
    cout << "Performing single search to find closest data point to mean:" << endl;
    vector<double> singleQuery;
    vector<int> index(1);
    vector<double> dist(1);

    // Searching for the Mean
    for(int i = 0 ; i < numDimensions ;i++)
            singleQuery.push_back(Mean);

    // Invoke the function
    kdtree.knnSearch(singleQuery, index, dist, 1, cv::flann::SearchParams(64));

    // Print single search results
    cout << "(index,dist):" << index[0] << "," << dist[0]<< endl;
*/
    // Batch: Call knnSearch
    cout << "Batch search:"<< endl;
    Mat indices;//(numQueries, k, CV_32S);
    Mat dists;//(numQueries, k, CV_32F);

    // Invoke the function
    kdtree.knnSearch(query, indices, dists, k, cv::flann::SearchParams(64));

    cout << indices.rows << "\t" << indices.cols << endl;
    cout << dists.rows << "\t" << dists.cols << endl;

    // Print batch results
    cout << "Output::"<< endl;
    for(int row = 0 ; row < indices.rows ; row++){
            cout << "(index,dist):";
            for(int col = 0 ; col < indices.cols ; col++){
                    cout << "(" << indices.at<int>(row,col) << "," << dists.at<float>(row,col) << ")" << "\t";
            }
            cout << endl;
    }

}

void MonocularVisualOdometry::initParams()
{
//  testKDTree();
//  testGetMatches();

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
  if (!nh_private_.getParam ("apps/mono_vo/number_of_random_trees", number_of_random_trees_))
    number_of_random_trees_ = 1;
  if (!nh_private_.getParam ("apps/mono_vo/use_opencv_projection", use_opencv_projection_))
    use_opencv_projection_ = true;
  if (!nh_private_.getParam ("apps/mono_vo/assume_initial_position", assume_initial_position_))
    assume_initial_position_ = true;
  if (!nh_private_.getParam ("apps/mono_vo/visualize_correspondences", visualize_correspondences_))
    visualize_correspondences_ = false;

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

void MonocularVisualOdometry::project3DTo2D(const std::vector<cv::Point3d> &input_3D_points, 
					                                  const cv::Mat &extrinsic, 
  					                                const cv::Mat &intrinsic, 
					                                  std::vector<cv::Point2d> &vector_2D_points)
{
  //this function assumes that all the points are visible from the camera
  vector_2D_points.clear();
  cv::Mat m_hom(4,1,CV_64FC1);

  for (uint i=0; i<input_3D_points.size(); ++i)
  {
    cv::Mat m(input_3D_points[i]);
    m_hom.at<double>(0,0) = m.at<double>(0,0);
    m_hom.at<double>(1,0) = m.at<double>(1,0);
    m_hom.at<double>(2,0) = m.at<double>(2,0);
    m_hom.at<double>(3,0) = 1.0;

    cv::Mat m_proj = intrinsic * extrinsic * m_hom;
    
    double z_proj = m_proj.at<double>(2,0);
    cv::Point2d temp_2D; 
    temp_2D.x = (m_proj.at<double>(0,0))/z_proj;          
    temp_2D.y = (m_proj.at<double>(1,0))/z_proj;
    vector_2D_points.push_back(temp_2D);
    
  }
}   


void MonocularVisualOdometry::getVisible3DPoints(const std::vector<cv::Point3d> &input_3D_points,
                                                 const cv::Mat &extrinsic,
                                                 const cv::Mat &intrinsic,
                                                 std::vector<cv::Point3d> &visible_3D_points)
{
  cv::Mat rmat = rmatFromMatrix(extrinsic);
  cv::Mat tvec = tvecFromMatrix(extrinsic);
  std::vector<cv::Point3d> positive_z_3D_points;
  std::vector<cv::Point3d> positive_z_3D_points_tf;
  std::vector<cv::Point2d> projected_positive_z_points;
  std::vector<cv::Point2d> valid_2D_points;

  for (uint i=0; i<input_3D_points.size(); ++i)
  {
    cv::Mat m(input_3D_points[i]);
    cv::Mat m_tf = rmat*m + tvec;                           //transform the model to the camera frame
    double z = m_tf.at<double>(2,0);
    if (z>0) 
    {
      positive_z_3D_points.push_back(cv::Point3d(m));       // 3d model in the world frame with positive z wrt the camera frame
      positive_z_3D_points_tf.push_back(cv::Point3d(m_tf)); // 3d model with positive z (camera frame)      

      cv::Mat m_proj = intrinsic * m_tf;                    // projected model with positive z into the image plane
      double z_proj = m_proj.at<double>(2,0);
      
      cv::Point2d temp_2D; 
      temp_2D.x = (m_proj.at<double>(0,0))/z_proj;          
      temp_2D.y = (m_proj.at<double>(1,0))/z_proj;          

      projected_positive_z_points.push_back(temp_2D);       // projected points 
    }  
  }
  // this function take only the 3D points (with positive z) wich projection is inside the image plane
  frame_->filterPointsWithinFrame(positive_z_3D_points, projected_positive_z_points, visible_3D_points, valid_2D_points);
}

void MonocularVisualOdometry::estimateMotion(const cv::Mat &E_prev, 
					     cv::Mat &E_new, 
					     const std::vector<cv::Point3d> &model, 
					     const std::vector<cv::Point2d> &features, 
					     int max_PnP_iterations)
{
 cv::Mat intrinsic_matrix = frame_->getIntrinsicCameraMatrix();

 cv::Mat rvec = rvecFromMatrix(E_prev);
 cv::Mat tvec = tvecFromMatrix(E_prev);

 std::vector<cv::Point3d> visible_3d_points;
 getVisible3DPoints(model, E_prev, intrinsic_matrix, visible_3d_points);

 E_new = E_prev;

 for (int i=0; i<=max_PnP_iterations; ++i) 
 {
   std::vector<cv::Point2d> vector_2d_corr;
   std::vector<cv::Point3d> vector_3d_corr;    

   if(getCorrespondences(visible_3d_points, features, E_new, vector_3d_corr, vector_2d_corr))
   {
     if (visualize_correspondences_)
     {
       std::vector<cv::KeyPoint> features_as_keypoints = frame_->keypoints;
       std::vector<cv::KeyPoint> projections_as_keypoints;
       std::vector<cv::Point2f> vector_2d_corr_as_floats;
       convert2DPointDoubleVectorToFloatVector(vector_2d_corr, vector_2d_corr_as_floats);
       cv::KeyPoint::convert(vector_2d_corr_as_floats, projections_as_keypoints);

       cv::namedWindow("Features", CV_WINDOW_NORMAL);
       cv::namedWindow("Correspondences", CV_WINDOW_NORMAL);
       cv::Mat feat_img, corrs_img;
       cv::drawKeypoints(*frame_->getRGBImage(), features_as_keypoints, feat_img);
       cv::drawKeypoints(*frame_->getRGBImage(), projections_as_keypoints, corrs_img);
       cv::imshow("Features", feat_img);
       cv::imshow("Correspondences", corrs_img);
       cv::waitKey(0);

       // TODO: draw matches
       cv::Mat matches_img;
       cv::vector<cv::DMatch> matches; // TODO: find matches as vector
       /*
       cv::drawMatches(feat_img, features_as_keypoints, // 1st image and its keypoints
                       corrs_img, projections_as_keypoints, // 2nd image and its keypoints
                       matches, // the matches
                       matches_img, // the image produced
                       cv::Scalar(255, 255, 255)); // color of the lines
       cv::namedWindow("Matches", CV_WINDOW_KEEPRATIO);
       cv::imshow("Matches", matches_img);
        */
     }


     cv::solvePnP(vector_3d_corr, vector_2d_corr, intrinsic_matrix, cv::Mat(), rvec, tvec, true);
     E_new = matrixFromRvecTvec(tvec, rvec);
   }
   ROS_WARN("estimate motion: Iteration %d", i);
 }
  
}

bool MonocularVisualOdometry::getCorrespondences(const std::vector<cv::Point3d> &model_3D, const std::vector<cv::Point2d> &features_2D, const cv::Mat &E, std::vector<cv::Point3d> &corr_3D_points, std::vector<cv::Point2d> &corr_2D_points, bool use_opencv_projection )
{
  // Clean old results (if any)
  corr_2D_points.clear();
  corr_3D_points.clear();

  std::vector<cv::Point2d> projected_model_2D_points;
  std::vector<cv::Point3d> valid_3D_points;
  std::vector<cv::Point2d> valid_2D_points;

  if(use_opencv_projection)
  {
    // WARNING: OpenCV projection function projects any 3D point (even those behind the image plane)
    cv::Mat rvec = rvecFromMatrix(E);
    cv::Mat tvec = tvecFromMatrix(E);
    cv::projectPoints(model_3D, rvec, tvec, frame_->getIntrinsicCameraMatrix(), frame_->pihole_model_.distortionCoeffs(), projected_model_2D_points);
  }
  else
    project3DTo2D(model_3D, E, frame_->getIntrinsicCameraMatrix(), projected_model_2D_points);

  frame_->filterPointsWithinFrame(model_3D, projected_model_2D_points, valid_3D_points, valid_2D_points);

  std::vector<int> match_indices;
  std::vector<float> match_distances; // ATTENTION: don't use double, otherwise the KDTree will crash
  cv::Mat detected_points_matrix;
  cv::Mat projected_points_matrix;
  convert2DPointVectorToMatrix(features_2D, detected_points_matrix);
  convert2DPointVectorToMatrix(valid_2D_points, projected_points_matrix);

  double total_distance_error = 0.0; // Accumulator error of distances to nearest neighbor

  if(getMatches(detected_points_matrix, projected_points_matrix, match_indices, match_distances))
  {
    // Filter points according to threshold distance
    for(uint i = 0 ; i < match_indices.size(); i++)
    {
      if(match_distances[i] < distance_threshold_)
      {
        corr_2D_points.push_back(valid_2D_points[match_indices[i]]);
        corr_3D_points.push_back(valid_3D_points[match_indices[i]]);
        total_distance_error += (double) match_distances[i];
      }
    }
    printf("Found %d correspondences\n", corr_2D_points.size());
    //  return (double) total_distance_error/corr_2D_points.size();
    if(corr_2D_points.size() > 0)
      return true;
    else
    {
      ROS_WARN("No correspondences found!");
      //  return (double) total_distance_error/corr_2D_points.size();
      return false;
    }
  }

}

void MonocularVisualOdometry::imageCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
//  mutex_lock_.lock();
  ros::WallTime start = ros::WallTime::now();

  // **** initialize ***************************************************
  std::vector<cv::Point2d> features_vector;

  if (!initialized_)
  {
    frame_ = boost::shared_ptr<MonocularFrame> (new MonocularFrame(rgb_msg, info_msg));
    ROS_INFO("RGB header = %s", rgb_msg->header.frame_id.c_str());
    initialized_ = getBaseToCameraTf(rgb_msg->header);
    init_time_ = rgb_msg->header.stamp;

    // TODO:
    // Estimate initial camera pose relative to the model
    feature_detector_->onlyFind2DFeatures(*frame_);
    if(frame_->buildKDTreeFromKeypoints(number_of_random_trees_))
    {
      frame_->getFeaturesVector(features_vector);
      E_ = estimateFirstPose(frame_->getIntrinsicCameraMatrix(), model_cloud_vector_, features_vector, min_inliers_, max_iterations_, distance_threshold_);

      frame_->setExtrinsicMatrix(E_);
    }
    else
      initialized_ = false;

    if (!initialized_) return;

    frame_->setCameraAperture(sensor_aperture_width_, sensor_aperture_height_);

    printf("Initialization successful at Frame %d\n", frame_count_);
  }

  // **** create frame *************************************************
  ros::WallTime start_frame = ros::WallTime::now();
  frame_->setFrame(rgb_msg);
  ros::WallTime end_frame = ros::WallTime::now();

  // **** find features ************************************************

  ros::WallTime start_features = ros::WallTime::now();
  feature_detector_->onlyFind2DFeatures(*frame_);
  ros::WallTime end_features = ros::WallTime::now();

  /*// TESTING Projections
  if(frame_->buildKDTreeFromKeypoints(number_of_random_trees_))
  {

    // Testing CV projection versus our own projection implementation

    // Does it filter Z??? (points behind the camera on the projection

    std::vector<cv::Point3d> arbitrary_3D_points_wrt_world;
    std::vector<cv::Point2d> projected_2D_points;
    cv::Point3d point_A(1,0,0.25);

    arbitrary_3D_points_wrt_world.push_back(point_A);
    arbitrary_3D_points_wrt_world.push_back(cv::Point3d(1,0,-0.25));
    arbitrary_3D_points_wrt_world.push_back(cv::Point3d(1,0.25, 0));
    arbitrary_3D_points_wrt_world.push_back(cv::Point3d(1,-0.25, 0));

    // Test conversion of a vector of points to homogeneous coordinates (as a matrix)
    cv::Mat matrix_of_3D_points;
    convert3DPointVectorToMatrix(arbitrary_3D_points_wrt_world, matrix_of_3D_points, CV_32FC1);
    cv::Mat homo_points;
    cv::convertPointsToHomogeneous(matrix_of_3D_points, homo_points);
    std::cout <<"HOMO points" << homo_points << std::endl;

    printf("Projecting with OpenCV...\n");
      cv::Mat rvec = rvecFromMatrix(E);
      cv::Mat tvec = tvecFromMatrix(E);
      cv::projectPoints(arbitrary_3D_points_wrt_world, rvec, tvec, frame_->getIntrinsicCameraMatrix(), frame_->pihole_model_.distortionCoeffs(), projected_2D_points);

    printf("Projection results:\n");

    for(uint i=0; i<projected_2D_points.size(); ++i)
    {
      std::cout << "3D Point: " << arbitrary_3D_points_wrt_world[i] <<  "---> projected to 2D Point: " << projected_2D_points[i] << std::endl;
    }



    printf("Projecting with our own function...\n");
    projected_2D_points.clear();
    project3DTo2D(arbitrary_3D_points_wrt_world, E, frame_->getIntrinsicCameraMatrix(), projected_2D_points);
    printf("Projection results:\n");
    for(uint i=0; i<projected_2D_points.size(); ++i)
    {
      std::cout << "3D Point: " << arbitrary_3D_points_wrt_world[i] <<  "---> projected to 2D Point: " << projected_2D_points[i] << std::endl;
    }
  }
   */

  if(frame_->buildKDTreeFromKeypoints(number_of_random_trees_))
  {
    printf("Processing frame %d\n", frame_count_);
    frame_->getFeaturesVector(features_vector);

    ros::WallTime start_3D_cloud_projection = ros::WallTime::now();
    cv::Mat E_new;
    estimateMotion(E_, E_new, model_cloud_vector_, features_vector, max_PnP_iterations_);
    // Update extrinsic matrix
    E_ = E_new;
    std::cout << "Translation vector at frame " << frame_count_ << ": " << tvecFromMatrix(E_) << std::endl;
    // TODO: use E_new to compute odom!!!!
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


  // ------- Perspective projection of cloud 3D points onto image plane
  // Assume known initial position of camera position at the origin of the world center of coordinates
  // TODO: needs a base2cam static transformation to correct for the camera coordinates in the world where +Z is point upwards
  // NOTE: the OpenNI driver publishes the static transformation (doing the rotation of the axis) between the rgb optical frame (/camera_rgb_optical_frame) and the /camera_link
  // ---------------------------------------------------------------
    ros::WallTime end_3D_cloud_projection = ros::WallTime::now();

    // TODO: compute odom
    // **** registration *************************************************
    // TODO: with the 3Dto2D method of registration and using PnP
    //ros::WallTime start_PnP_reg = ros::WallTime::now();
    //  tf::Transform motion = motion_estimation_->getMotionEstimation(frame);
    //  f2b_ = motion * f2b_; // TODO: the transformation based on motion estimation after PnP
    //ros::WallTime end_PnP_reg = ros::WallTime::now();
    // **** publish motion **********************************************

    publishTf(rgb_msg->header);

    ros::WallTime end = ros::WallTime::now();
    // **** print diagnostics *******************************************

    //  int n_features = frame.features.points.size();
    int n_keypoints = frame_->keypoints.size();

    double d_frame    = 1000.0 * (end_frame    - start_frame   ).toSec();
    double d_features = 1000.0 * (end_features - start_features).toSec();
    double d_cloud_projection = 1000.0 * (end_3D_cloud_projection - start_3D_cloud_projection).toSec();
    // double d_PnP_reg      = 1000.0 * (end_PnP_reg      - start_PnP_reg).toSec();
    double d_PnP_reg      = 0; // TODO: ^^^^^^ compute in correct place
    double d_total    = 1000.0 * (end          - start).toSec();

//  float time = (rgb_msg->header.stamp - init_time_).toSec();
  //  int model_size = motion_estimation_->getModelSize();
  //
  //  double pos_x = f2b_.getOrigin().getX();
  //  double pos_y = f2b_.getOrigin().getY();
  //  double pos_z = f2b_.getOrigin().getZ();

  ROS_INFO("[%d] Fr: %2.1f s %s[%d keyspoints]: %3.1f s  \t Proj: %2.1f s \t Reg: %4.1f s \t TOTAL %4.1f s\n",
    frame_count_,
    d_frame, 
    detector_type_.c_str(),
    n_keypoints, d_features,
    d_cloud_projection, d_PnP_reg,
    d_total);

  frame_count_++;
  }

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

  std::vector<cv::Point2d> projected_model_2D_points;
  std::vector<cv::Point3d> valid_3D_points;
  std::vector<cv::Point2d> valid_2D_points;

  if(use_opencv_projection_)
   {
     cv::Mat rvec = rvecFromMatrix(E);
     cv::Mat tvec = tvecFromMatrix(E);
     cv::projectPoints(sample_3D_points, rvec, tvec, frame_->getIntrinsicCameraMatrix(), frame_->pihole_model_.distortionCoeffs(), projected_model_2D_points);
   }
   else
     // TODO: Test custom function in comparisson to OpenCV's implementation
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
}

void MonocularVisualOdometry::publishTf(const std_msgs::Header& header)
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

void MonocularVisualOdometry::testGetMatches()
{
  std::vector<cv::Point2d> detected_points;
  std::vector<cv::Point2d> projected_points;

  detected_points.push_back(cv::Point2d(10.0, 10.0));
  detected_points.push_back(cv::Point2d(20.0, 20.0));
  detected_points.push_back(cv::Point2d(30.0, 30.0));
  detected_points.push_back(cv::Point2d(40.0, 40.0));
  
  projected_points.push_back(cv::Point2d(30.1, 31.2));
  projected_points.push_back(cv::Point2d(18.1, 16.0));
  projected_points.push_back(cv::Point2d(9.1,  11.5));
  projected_points.push_back(cv::Point2d(41.2,   42.01));
  
  cv::Mat detected_points_matrix;
  cv::Mat projected_points_matrix;
  convert2DPointVectorToMatrix(detected_points, detected_points_matrix);
  convert2DPointVectorToMatrix(projected_points, projected_points_matrix);
  
  std::vector<int> match_indices;
  std::vector<float> match_distances;
  // results should be (0, 2), (1, 1), (2, 0), (3, 3)
  getMatches(projected_points_matrix, detected_points_matrix, match_indices, match_distances);
}

bool MonocularVisualOdometry::getMatches (
  const cv::Mat& reference_points,
  const cv::Mat& query_points,
  std::vector<int>& match_indices,
  std::vector<float>& match_distances
  )
{
//  printf("matching...\n");

  // Batch: Call knnSearch
//  std::cout << "Batch search:"<< std::endl;
  // Invoke the function
  int k = 1;
  cv::Mat indices;//(numQueries, k, CV_32S);
  cv::Mat dists;//(numQueries, k, CV_32F);
  frame_->kdtree_->knnSearch(query_points, indices, dists, k, cv::flann::SearchParams(64));
  match_indices.resize(indices.rows);
  match_distances.resize(dists.rows);

  std::cout << "Number of features: " << reference_points.rows << "\t Matches: " << match_indices.size()  << std::endl;
//  std::cout << match_distances.size() << std::endl;

  // Print batch results
//  std::cout << "Output::"<< std::endl;
  for(int row = 0 ; row < indices.rows ; row++)
  {
    match_indices[row]=indices.at<int>(row,0);
    match_distances[row]=dists.at<float>(row,0);
//    printf("Row %d: col[0]=%d \t Distances: col[0]=%f \n", row, match_indices[row], match_distances[row]);
//    printf("Point = (%f, %f) \n\n", reference_points.at<float>(match_indices[row],0), reference_points.at<float>(match_indices[row],1));
  }

  if(match_indices.size() > 0)
    return true;
  else
    return false;

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
  ROS_INFO("Estimating Initial Camera Pose");

  if(assume_initial_position_ == false)
  {
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
  }
  else
  {
//    cv::Mat tvec = (cv::Mat_<double> (3,1) << 0.0, 0.0, 0.0);
//    cv::Mat rvec = (cv::Mat_<double> (3,1) << 0.0, 0.0, 0.0);
    cv::Mat trans, rot;
    transformToRotationCV(
        b2c_.inverse(), trans, rot);
    return matrixFromRT(rot, trans);
  }
}

} //namespace ccny_rgbd
