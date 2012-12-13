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

  using namespace std;

    int numData = 1;
    int numQueries = 2;
    int numDimensions = 2;
    int k = 1; // Number of neighbors

    // Create the data
//    cv::Mat features(numData,numDimensions,CV_32F), query(numQueries,numDimensions,CV_32F);
//    cv::randu(features, Scalar::all(Mean), cv::Scalar::all(Variance));
//    cv::randu(query, Scalar::all(Mean), cv::Scalar::all(Variance));

//    cv::Mat reference_points = (cv::Mat_<float> (numData,numDimensions) << 3.0, 3.0
//                                                            );
//    cv::Mat query_points = (cv::Mat_<float> (numQueries,numDimensions) << 20.0, 3.0,
//                                                                            18, 4.0);

    std::vector<cv::Point2d> reference_points;
    reference_points.push_back(cv::Point2d(3.0, 3.0));
    reference_points.push_back(cv::Point2d(18.0, 15.0));
//    reference_points.push_back(cv::Point2d(30.0, 30.0));
//    reference_points.push_back(cv::Point2d(40.0, 40.0));

    std::vector<cv::Point2d> query_points;
    query_points.push_back(cv::Point2d(10., 3.));
    query_points.push_back(cv::Point2d(18, 4.0));
    query_points.push_back(cv::Point2d(6, 4.0));
//    query_points.push_back(cv::Point2d(9.1,  11.5));
//    query_points.push_back(cv::Point2d(41.2,   42.01));

    cv::Mat reference_points_matrix;
    cv::Mat query_points_matrix;
    convert2DPointVectorToMatrix(reference_points, reference_points_matrix, CV_32FC1);
    convert2DPointVectorToMatrix(query_points, query_points_matrix, CV_32FC1);

    std::vector<int> match_indices;
    std::vector<float> match_distances;

    // Print generated data
    std::cout << "Reference (Tree) points:" << reference_points_matrix << std::endl;
    std::cout << "Query points:" << query_points_matrix << std::endl;
//    for(int row = 0 ; row < reference_points.rows ; row++){
//            for(int col = 0 ; col < reference_points.cols ; col++){
//                    std::cout << reference_points.at<float>(row,col) <<"\t";
//            }
//            std::cout << endl;
//    }
//    std::cout << "Query::" << std::endl;
//    for(int row = 0 ; row < query_points.rows ; row++){
//            for(int col = 0 ; col < query_points.cols ; col++){
//                    cout << query_points.at<float>(row,col) <<"\t";
//            }
//            cout << endl;
//    }


    // KdTree with 5 random trees
//    cv::flann::KDTreeIndexParams indexParams(5);
    // You can also use LinearIndex
    cv::flann::LinearIndexParams indexParams;

    // Create the Index
    cv::flann::Index kdtree(reference_points_matrix, indexParams);

    // Batch: Call knnSearch
    cout << "Batch search:"<< endl;
    cv::Mat indices;//(numQueries, k, CV_32S);
    cv::Mat dists_sq;//(numQueries, k, CV_32F);

    // Invoke the function
    kdtree.knnSearch(query_points_matrix, indices, dists_sq, k, cv::flann::SearchParams(64, 0, true));

    cout << "Indices:  " << indices.rows << " rows, " << indices.cols << " cols" << endl;
    cout << "Distances (Squared values):  " << dists_sq.rows << " rows, " << dists_sq.cols << " cols" << endl;

    // Print batch results
    cout << "Output::"<< endl;
    for(int row = 0 ; row < indices.rows ; row++){
            cout << "(index,dist):";
            for(int col = 0 ; col < indices.cols ; col++){
                    cout << "(KNN index: " << indices.at<int>(row,col) << ", Sq. dist: " << dists_sq.at<float>(row,col) << ")" << "\t";
            }
            cout << endl;
    }

}

void MonocularVisualOdometry::initParams()
{
//  testKDTree();

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

void MonocularVisualOdometry::estimateMotion(
  const cv::Mat& E_prev, 
  cv::Mat& E_new, 
  const std::vector<cv::Point3d>& model,        // full 3D model
  const std::vector<cv::Point2d>& features,     // observed 2D features
  int max_PnP_iterations)
{
  // **** build kd tree

  int number_of_random_trees = 4;
  cv::Mat train_points; // observed 2d features
  convert2DPointVectorToMatrix(features, train_points, CV_32FC1);

  //cv::Mat train_points(features);
  //train_points.convertTo(train_points, CV_32FC1);
  
  cv::flann::LinearIndexParams indexParams;
  cv::flann::Index kd_tree(train_points, indexParams);

  // **************************************************

  cv::Mat intrinsic_matrix = frame_->getIntrinsicCameraMatrix();

  cv::Mat rvec = rvecFromMatrix(E_prev);
  cv::Mat tvec = tvecFromMatrix(E_prev);

  std::vector<cv::Point3d> visible_3d_points;
  getVisible3DPoints(model, E_prev, intrinsic_matrix, visible_3d_points);

  E_new = E_prev;

  printf("estimateMotion...\n");

  // output 
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  std::cout << "tvec: " << std::endl << tvec << std::endl;
  std::cout << "rmat: " << std::endl << rmat << std::endl;

  for (int i=0; i <= max_PnP_iterations; ++i) 
  {
    printf("[iter %d] estimateMotion:\n", i);

    std::vector<cv::Point2d> vector_2d_corr;
    std::vector<cv::Point3d> vector_3d_corr;    

    bool corresp_result = getCorrespondences(kd_tree, visible_3d_points, features, E_new, vector_3d_corr, vector_2d_corr);
  
    if(corresp_result)
    {
      cv::solvePnP(vector_3d_corr, vector_2d_corr, intrinsic_matrix, cv::Mat(), rvec, tvec, true);
      E_new = matrixFromRvecTvec(rvec, tvec);

      // output 
      cv::Mat rmat;
      cv::Rodrigues(rvec, rmat);
      std::cout << "tvec: " << std::endl << tvec << std::endl;
      std::cout << "rmat: " << std::endl << rmat << std::endl;
    }
    else
    {
      // TODO: handle this better
      ROS_WARN("Could not estimate correspondences. Leaving estimateMotion loop.");
      break;
    }
  } 
}

bool MonocularVisualOdometry::getCorrespondences(
  cv::flann::Index& kd_tree,
  const std::vector<cv::Point3d>& visible_3d_points, 
  const std::vector<cv::Point2d>& features_2D, 
  const cv::Mat &E, 
  std::vector<cv::Point3d> &corr_3D_points, 
  std::vector<cv::Point2d> &corr_2D_points, 
  bool use_opencv_projection )
{
  // Clean old results (if any)
  corr_2D_points.clear();
  corr_3D_points.clear();

  // project visible 3d to 2d
  std::vector<cv::Point2d> projected_model_2D_points;
  project3DTo2D(visible_3d_points, E, frame_->getIntrinsicCameraMatrix(), projected_model_2D_points);

  std::vector<int> match_indices;
  std::vector<float> match_distances; // ATTENTION: don't use double, otherwise the KDTree will crash
  cv::Mat detected_points_matrix;
  cv::Mat projected_points_matrix;
  convert2DPointVectorToMatrix(features_2D, detected_points_matrix, CV_32FC1);
  convert2DPointVectorToMatrix(projected_model_2D_points, projected_points_matrix, CV_32FC1);

  double total_distance_error = 0.0; // Accumulator error of distances to nearest neighbor

  if(getMatches(kd_tree, detected_points_matrix, projected_points_matrix, match_indices, match_distances, false))
  {
    cv::vector<cv::DMatch> good_matches; // vector of matches where distance is below threshold

    // Filter points according to threshold distance
    for(uint i = 0 ; i < match_indices.size(); i++)
    {
      if(match_distances[i] < distance_threshold_)
      {
        int query_idx = i;
        int train_idx = match_indices[i];

        corr_2D_points.push_back(features_2D[train_idx]);
        corr_3D_points.push_back(visible_3d_points[query_idx]);

        total_distance_error += (double) match_distances[i];
        good_matches.push_back(cv::DMatch(query_idx, train_idx, match_distances[i]));
      }
    }
    printf("Found %d good matches\n", corr_2D_points.size());

    if (visualize_correspondences_)
    {
      std::vector<cv::KeyPoint> features_as_keypoints = frame_->keypoints;
      std::vector<cv::KeyPoint> projections_as_keypoints;
      std::vector<cv::Point2f> vector_2d_corr_as_floats;
      convert2DPointDoubleVectorToFloatVector(projected_model_2D_points, vector_2d_corr_as_floats);
      cv::KeyPoint::convert(vector_2d_corr_as_floats, projections_as_keypoints);

      cv::Mat feat_img, proj_points_img;
      feat_img = frame_->getRGBImage()->clone();
      proj_points_img = frame_->getRGBImage()->clone();

      // Draw matches
      cv::Mat good_matches_img;
      cv::drawMatches(proj_points_img, projections_as_keypoints, // (QUERY) 1nd image and its keypoints
                      feat_img, features_as_keypoints,           // (TRAIN) 2st image and its keypoints
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

  return true;
}

void MonocularVisualOdometry::imageCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  ros::WallTime start = ros::WallTime::now();

  ROS_INFO("Processing frame %d", frame_count_);
  frame_.reset();
  frame_ = boost::shared_ptr<MonocularFrame> (new MonocularFrame(rgb_msg, info_msg));

  // **** initialize ***************************************************
  std::vector<cv::Point2d> features_vector;

  if (!initialized_)
  {
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

  // **** find features ************************************************

  ros::WallTime start_features = ros::WallTime::now();
  feature_detector_->onlyFind2DFeatures(*frame_);
  ros::WallTime end_features = ros::WallTime::now();

  // **** estimate motion ************************************************

  //if(frame_->buildKDTreeFromKeypoints(number_of_random_trees_))
 // {
    frame_->getFeaturesVector(features_vector);

    ros::WallTime start_3D_cloud_projection = ros::WallTime::now();
    cv::Mat E_new;
    estimateMotion(E_, E_new, model_cloud_vector_, features_vector, max_PnP_iterations_);
    // Update extrinsic matrix
    E_ = E_new;
    std::cout << "Computed Translation vector at frame " << frame_count_ << ": " << tvecFromMatrix(E_) << std::endl;
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

    double d_features = 1000.0 * (end_features - start_features).toSec();
    double d_cloud_projection = 1000.0 * (end_3D_cloud_projection - start_3D_cloud_projection).toSec();
    // double d_PnP_reg      = 1000.0 * (end_PnP_reg      - start_PnP_reg).toSec();
    double d_PnP_reg      = 0; // TODO: ^^^^^^ compute in correct place
    double d_total    = 1000.0 * (end          - start).toSec();


/*
  ROS_INFO("[%d] Fr: %2.1f s %s[%d keyspoints]: %3.1f s  \t Proj: %2.1f s \t Reg: %4.1f s \t TOTAL %4.1f s\n",
    frame_count_,
    detector_type_.c_str(),
    n_keypoints, d_features,
    d_cloud_projection, d_PnP_reg,
    d_total)
*/

    frame_count_++;
  //}

  if(publish_cloud_model_)
  {
    printf("Publishing model cloud read from PCD\n");
    pub_model_.publish(*model_ptr_);
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
*/
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
/*
  std::vector<cv::Point2d> reference_points;
  std::vector<cv::Point2d> query_points;

  reference_points.push_back(cv::Point2d(10.0, 10.0));
  reference_points.push_back(cv::Point2d(20.0, 20.0));
  reference_points.push_back(cv::Point2d(30.0, 30.0));
  reference_points.push_back(cv::Point2d(40.0, 40.0));
  
  query_points.push_back(cv::Point2d(30.1, 31.2));
  query_points.push_back(cv::Point2d(18.1, 16.0));
  query_points.push_back(cv::Point2d(9.1,  11.5));
  query_points.push_back(cv::Point2d(41.2,   42.01));
  
  cv::Mat detected_points_matrix;
  cv::Mat projected_points_matrix;
  convert2DPointVectorToMatrix(reference_points, detected_points_matrix);
  convert2DPointVectorToMatrix(query_points, projected_points_matrix);
  
  std::vector<int> match_indices;
  std::vector<float> match_distances;
  // results should be (0, 2), (1, 1), (2, 0), (3, 3)
  getMatches(projected_points_matrix, detected_points_matrix, match_indices, match_distances);
*/
}

// TODO: no need to pass train_points
bool MonocularVisualOdometry::getMatches(
  cv::flann::Index& kd_tree,
  const cv::Mat& train_points,          // 2d detected features
  const cv::Mat& query_points,          // 2d visible projected 
  std::vector<int>& match_indices,
  std::vector<float>& match_distances,
  bool prune_repeated_matches)
{
  // Perform knn search
  cv::Mat indices, dists;
  kd_tree.knnSearch(query_points, indices, dists, 1, cv::flann::SearchParams(64));
  printf("[matching] query_pts: %d, train_pts: %d\n", query_points.rows, train_points.rows);

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


void MonocularVisualOdometry::pruneMatches (
    const std::vector<int>& match_indices_in,
    const std::vector<float>& match_distances_in,
    std::vector<int>& match_indices_out,
    std::vector<float>& match_distances_out
) const
{
  KMatch::kMatchSet kmatch_prunning_set;
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
