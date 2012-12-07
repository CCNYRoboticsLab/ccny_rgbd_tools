#include "ccny_rgbd/structures/monocular_frame.h"

namespace ccny_rgbd
{

MonocularFrame::MonocularFrame(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg):
                               RGBDFrame(rgb_msg, info_msg)
{
  processCameraInfo(info_msg);
}

MonocularFrame::~MonocularFrame()
{
  delete image_size_;
}

void MonocularFrame::setFrame(const sensor_msgs::ImageConstPtr& rgb_msg)
{
  this->cv_ptr_rgb_   = cv_bridge::toCvCopy(rgb_msg);
  // record header - frame is from RGB camera
  this->header_ = rgb_msg->header;
}

void MonocularFrame::setCameraModel(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  // create camera model
  this->pihole_model_.fromCameraInfo(info_msg);
  processCameraInfo(info_msg);
}

void MonocularFrame::setExtrinsicMatrix(const cv::Mat &E)
{
  rvec_ = rvecFromMatrix(E);
  R_ = rmatFromMatrix(E);
  tvec_ = tvecFromMatrix(E);
  T_ = tvec_;

  std::cout << "Results from initial Pose: " << std::endl;
  std::cout << "R: " << R_ << std::endl;
  std::cout << "rvec_: " << rvec_ << std::endl;
  std::cout << "t: " << T_ << std::endl;
  std::cout << "tvec: " << tvec_ << std::endl;

}

void MonocularFrame::processCameraInfo(const sensor_msgs::CameraInfoConstPtr &cam_info_ptr)
{
  //  cv::Mat cameraMatrix(cv::Size(4,3), CV_64FC1);
  // Projection/camera matrix
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]
  //     [ 0   0   1   0]
  // Note that calibrationMatrixValues doesn't require the Tx,Ty column (it only takes the 3x3 intrinsic parameteres)

  this->pihole_model_.fromCameraInfo(cam_info_ptr);
  image_size_ = new cv::Size(cam_info_ptr->width, cam_info_ptr->height);

  /*
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
  */
  cv::Mat P =  this->pihole_model_.projectionMatrix();
  cv::decomposeProjectionMatrix(P, K_, R_, T_);
  cv::Rodrigues(R_, rvec_);

  tvec_ = cv::Mat::zeros(3, 1, CV_64FC1);
//  cv::Mat tvec = cv::Mat::zeros(3, 1, E.type()); // FIXME: it would be better to template the types or not to change at all

  tvec_.at<double>(0,0) = T_.at<double>(0,3);
  tvec_.at<double>(1,0) = T_.at<double>(1,3);
  tvec_.at<double>(2,0) = T_.at<double>(2,3);
  // TODO: later
  /*
  cv::Mat E = matrixFromRvecTvec(rvec_, tvec_);
  tvec_ = tvecFromMatrix()

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
   */
  dist_coeffs_ = this->pihole_model_.distortionCoeffs();

  std::cout << "Decomposed Matrix" << std::endl;
  std::cout << "P: " << P << std::endl;
  std::cout << "R: " << R_ << std::endl;
  std::cout << "rvec_: " << rvec_ << std::endl;
  std::cout << "t: " << T_ << std::endl;
  std::cout << "tvec: " << tvec_ << std::endl;
  std::cout << "K: " << K_ << std::endl;
  std::cout << "Distortion: " << dist_coeffs_ << std::endl;

  cv::Mat E = matrixFromRvecTvec(rvec_, tvec_);

//  std::cout << "COMPARISSON to Pinhole_Camera_Model's results" << std::endl;
//  std::cout << "K: " << this->pihole_model_.intrinsicMatrix() << std::endl;
//  std::cout << "K_full: " << this->pihole_model_.fullIntrinsicMatrix() << std::endl;
//  std::cout << "Projection Matrix: " << this->pihole_model_.projectionMatrix() << std::endl;
  std::cout << "CHECK with utils" << std::endl;
  std::cout << "rvec: " << rvecFromMatrix(E) << std::endl;
  std::cout << "R: " << rmatFromMatrix(E) << std::endl;
  std::cout << "tvec: " << tvecFromMatrix(E) << std::endl;
}

cv::Mat MonocularFrame::getIntrinsicCameraMatrix() const
{
  return K_;
}

void MonocularFrame::setCameraAperture(double width, double height)
{
  sensor_aperture_width_ = width;
  sensor_aperture_height_ = height;

  double fovx ,fovy, focalLength;
  double aspectRatio;

  std::cout << "Sensor apertures: width = " << sensor_aperture_width_ << " height = " << sensor_aperture_height_ << std::endl;  // passed as parameters
  std::cout << "Image Size: " << image_size_->width << ", " << image_size_->height << std::endl;

  cv::calibrationMatrixValues(K_, *image_size_, sensor_aperture_width_, sensor_aperture_height_, fovx, fovy, focalLength, principal_point_, aspectRatio);

  std::cout << "Camera intrinsic parameters: " << std::endl
      << "fovx=" << fovx << ", fovy=" << fovy << ", Focal Length= " << focalLength << " mm"
      << ", Principal Point=" <<  principal_point_ << ", Aspect Ratio=" << aspectRatio
      << std::endl;

  //NOTE:
  // theta_x = 2*tan_inv( (width/2) / fx )
  // theta_y = 2*tan_inv( (height/2) / fy )

  std::cout << "COMPARISSON to Pinhole_Camera_Model's results" << std::endl;
  std::cout << "fx: " << this->pihole_model_.fx() << std::endl;
  std::cout << "fy: " << this->pihole_model_.fy() << std::endl;
  std::cout << "cx: " << this->pihole_model_.cx() << std::endl;
  std::cout << "cy: " << this->pihole_model_.cy() << std::endl;

  // FIXME: why was I doing this?
//     principal_point_.x += imageSize.width /2.0;
//     principal_point_.y += imageSize.height /2.0;

}

bool MonocularFrame::isPointWithinFrame(const cv::Point2f &point) const
{
  return (
      point.x > 0 && point.x < image_size_->width
      &&
      point.y > 0 && point.y < image_size_->height
      );
}

void MonocularFrame::filterPointsWithinFrame(const std::vector<cv::Point3d> &all_3D_points, const std::vector<cv::Point2d> &all_2D_points)
{
  valid_2D_points_.clear();
  valid_3D_points_.clear();

#ifdef DEVELOP
  ROS_DEBUG("%d points in model", all_3D_points.size());
#endif

  for(unsigned int i = 0; i < all_2D_points.size(); ++i)
  {
    if(isPointWithinFrame(all_2D_points[i]))
    {
//      model_feature_pair_ valid_point_pair(all_3D_points[i], all_2D_points[i]);
//      valid_points.push_back(valid_point_pair);
//      std::cout << "3D Object point: " << valid_points.end()->model_point << " Projected to " << valid_points.end()->keyframe << std::endl;
      valid_2D_points_.push_back(all_2D_points[i]);
      valid_3D_points_.push_back(all_3D_points[i]);
    }

    // FIXME: breaking with 10 points temporarily
    if(valid_2D_points_.size()>10)
      break;
  }

  for(unsigned int i = 0; i < valid_3D_points_.size(); ++i)
    std::cout << "3D Object point: " << valid_3D_points_[i] << " Projected to " << valid_2D_points_[i] << std::endl;
  ROS_DEBUG("%d valid points projected to frame", all_2D_points.size());
}


void MonocularFrame::getFeaturesVector(std::vector<cv::Point2d> &features_vector) const
{
  features_vector.clear();

  std::vector<cv::KeyPoint>::const_iterator feat_it = keypoints.begin();
  for(; feat_it!=keypoints.end(); ++feat_it)
  {
    cv::KeyPoint feature_from_frame = *feat_it;
    cv::Point2d keypoint_point = feat_it->pt;
    features_vector.push_back(keypoint_point);
  }
//  }

}


bool MonocularFrame::project3DModelToCamera(const PointCloudFeature::Ptr model_3Dcloud, bool is_first_time)
{
  std::vector<cv::Point3d> cv_model_3D_points;
//  std::vector<cv::Point3f> model_in_frustum; // TODO: define a valid frustum volume for 2nd time frame (instead of projecting the whole cloud)

  std::vector<cv::Point2d> projected_model_2D_points;
//  std::vector<model_feature_pair_> valid_projected_points;

//  if(is_first_time)
//  {  // Project entire cloud:
    PointCloudFeature::iterator cloud_it = model_3Dcloud->begin();
    for(; cloud_it!=model_3Dcloud->end(); ++cloud_it)
    {
      PointFeature point_from_model = *cloud_it;
      cv::Point3d cv_cloud_point((double) point_from_model.x,  (double) point_from_model.y, (double) point_from_model.z);
      cv_model_3D_points.push_back(cv_cloud_point);
    }
//  }
  cv::projectPoints(cv_model_3D_points, rvec_, tvec_, K_, dist_coeffs_, projected_model_2D_points);

  filterPointsWithinFrame(cv_model_3D_points, projected_model_2D_points);

  // TODO: if they are within the camera's frustum
  // Save into a KD-tree for fast search
//  tree_2D_points_from_cloud_ = boost::shared_ptr<flann::KDTreeSingleIndex> (new flann::KDTreeSingleIndex());


  // ---------------------------------- TEST KD Trees -------------------------------------
  // KdTree with 5 random trees
//  cv::flann::KDTreeIndexParams indexParams(5);

  // You can also use LinearIndex
//  cv::flann::LinearIndexParams indexParams;

  // Create the Index
  cv::Mat valid_2D_points(valid_2D_points_);
    std::cout << "Valid 2D Points as Matrix:" << valid_2D_points << std::endl
        << " with: " << valid_2D_points.rows << " rows, " << valid_2D_points.cols << " cols" << std::endl;

  cv::KDTree my_tree;

  /*
//  my_tree.build(valid_2D_points_, false);
//  cv::Mat test(3,5, CV_32F);
  std::vector<cv::Point2f> test(5);
//  test.at<float>(0,0) = 6.6;
//  test.at<float>(0,1) = 7.6;
//  test.at<float>(0,4) = 8.6;
  test.push_back(cv::Point2f(3.2,4.4));
  test.push_back(cv::Point2f(3.2,4.4));
  test.push_back(cv::Point2f(3.2,4.4));


  my_tree.build(fuck, false);
  */
//  my_tree.build(valid_2D_points, false);
//  std::cout << "The search space dimensionality: " << my_tree.dims() << std::endl;
//  std::cout << "Tree points:" << my_tree.points << std::endl;
//  std::cout << "Valid 2D Points as Matrix:" << valid_2D_points << std::endl
//      << " with: " << valid_2D_points.rows << " rows, " << valid_2D_points.cols << " cols" << std::endl;

//  cv::Mat query(numQueries,numDimensions,CV_32F);

//  cv::flann::Index kdtree(valid_2D_points, indexParams);

  // Perform single search for mean
  std::cout << "Performing single search to find closest data point to mean:" << std::endl;
  /*
  vector<float> singleQuery;
  vector<int> index(1);
  vector<float> dist(1);

  // Searching for the Mean
  for(int i = 0 ; i < numDimensions ;i++)
          singleQuery.push_back(Mean);

  // Invoke the function
  kdtree.knnSearch(singleQuery, index, dist, 1, cv::flann::SearchParams(64));

  // Print single search results
  cout << "(index,dist):" << index[0] << "," << dist[0]<< endl;

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

*/

  /*
        pcl::KdTreeFLANN<PointTV>::Ptr tree =
        boost::make_shared<pcl::KdTreeFLANN<PointTV> > ();

      tree->setInputCloud(model_);
      int rc = 0;

      // Allocate enough space to hold the results
      std::vector<int> nn_indices (1);
      std::vector<float> nn_dists (1);

      PointCloudTV::Ptr new_features = boost::make_shared<PointCloudTV>();
      new_features->header.frame_id = odom_frame_;

      for (int ii = 0; ii < features_tf->points.size(); ++ii)
      {
        int result = tree->nearestKSearch(features_tf->points[ii], 1, nn_indices, nn_dists);

        if (result == 0 || nn_dists[0] > 0.001)
        {
          new_features->points.push_back(features_tf->points[ii]);
        }
      }

      new_features->width = new_features->points.size();

      *model_ += *new_features;
      */

  // TODO:
  // model_in_frustum.push_back(cloud_point);


  return false; // Indicates that the first time projection is history!
}

} // namespace ccny_rgbd
