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

/*
  std::cout << "Decomposed Matrix" << std::endl;
  std::cout << "P: " << P << std::endl;
  std::cout << "R: " << R_ << std::endl;
  std::cout << "rvec_: " << rvec_ << std::endl;
  std::cout << "t: " << T_ << std::endl;
  std::cout << "tvec: " << tvec_ << std::endl;
  std::cout << "K: " << K_ << std::endl;
  std::cout << "Distortion: " << dist_coeffs_ << std::endl;
*/

  cv::Mat E = matrixFromRvecTvec(rvec_, tvec_);

//  std::cout << "COMPARISSON to Pinhole_Camera_Model's results" << std::endl;
//  std::cout << "K: " << this->pihole_model_.intrinsicMatrix() << std::endl;
//  std::cout << "K_full: " << this->pihole_model_.fullIntrinsicMatrix() << std::endl;
//  std::cout << "Projection Matrix: " << this->pihole_model_.projectionMatrix() << std::endl;
/*
  std::cout << "CHECK with utils" << std::endl;
  std::cout << "rvec: " << rvecFromMatrix(E) << std::endl;
  std::cout << "R: " << rmatFromMatrix(E) << std::endl;
  std::cout << "tvec: " << tvecFromMatrix(E) << std::endl;
*/
}

cv::Mat MonocularFrame::getIntrinsicCameraMatrix() const
{
  return K_;
}

bool MonocularFrame::buildKDTreeFromKeypoints(int number_of_random_trees)
{
  if(this->keypoints.empty())
    return false;


  std::vector<cv::Point2d> features_vector;
  this->getFeaturesVector(features_vector);
  cv::Mat reference_points;
  convert2DPointVectorToMatrix(features_vector, reference_points, CV_32FC1);

  if(number_of_random_trees > 1)
  {
  // KdTree with 5 random trees
    cv::flann::KDTreeIndexParams indexParams(number_of_random_trees);
    // Create the Index
    kdtree_ = boost::shared_ptr<cv::flann::Index> (new cv::flann::Index(reference_points, indexParams));
  }
  else// You can also use LinearIndex
  {
    cv::flann::LinearIndexParams indexParams;
    // Create the Index
    kdtree_ = boost::shared_ptr<cv::flann::Index> (new cv::flann::Index(reference_points, indexParams));
  }

  printf("Built KD-Tree successfully!\n\n");

  return true;
}

void MonocularFrame::getFeaturesVector(std::vector<cv::Point2d> &features_vector) const
{
  features_vector.clear();

  std::vector<cv::KeyPoint>::const_iterator feat_it = keypoints.begin();
  for(; feat_it!=keypoints.end(); ++feat_it)
  {
    cv::KeyPoint feature_from_frame = *feat_it;
    cv::Point2d keypoint_point = feat_it->pt; // Assuming automatic cast from Point2f to Point2d?
    features_vector.push_back(keypoint_point);
  }

  printf("Obtaining features vector successfully. \n");
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

void MonocularFrame::filterPointsWithinFrame(const std::vector<cv::Point3d> &all_3D_points, const std::vector<cv::Point2d> &all_2D_points,
                                             std::vector<cv::Point3d> &valid_3D_points,
                                             std::vector<cv::Point2d> &valid_2D_points)
{
  valid_2D_points.clear();
  valid_3D_points.clear();

#ifdef DEVELOP
  ROS_DEBUG("%d points in model", all_3D_points.size());
#endif

  for(unsigned int i = 0; i < all_2D_points.size(); ++i)
  {
    if(isPointWithinFrame(all_2D_points[i]))
    {
      valid_2D_points.push_back(all_2D_points[i]);
      valid_3D_points.push_back(all_3D_points[i]);
    }
  }

  ROS_DEBUG("%d valid points projected to frame", valid_2D_points.size());
//  for(unsigned int i = 0; i < valid_3D_points.size(); ++i)
//    std::cout << "3D Object point: " << valid_3D_points[i] << " Projected to " << valid_2D_points[i] << std::endl;
}


} // namespace ccny_rgbd
