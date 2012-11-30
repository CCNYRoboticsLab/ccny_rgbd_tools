#include "ccny_rgbd/structures/monocular_frame.h"

namespace ccny_rgbd
{

MonocularFrame::MonocularFrame(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg):
                               RGBDFrame(rgb_msg, info_msg)
{
  processCameraInfo(info_msg);
}

void MonocularFrame::processCameraInfo(const sensor_msgs::CameraInfoConstPtr &cam_info_ptr)
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

void MonocularFrame::setCameraAperture(double width, double height)
{
  sensor_aperture_width_ = width;
  sensor_aperture_height_ = height;
}

bool MonocularFrame::project3DModelToCamera(const PointCloudFeature::Ptr model_3Dcloud, bool is_first_time)
{
  std::vector<cv::Point3f> cv_model_3D_points;
  std::vector<cv::Point3f> model_in_frustum; // TODO: define a valid frustum volume for 2nd time frame (instead of projecting the whole cloud)

  std::vector<cv::Point2d> projected_model_2D_points;

//  if(is_first_time)
//  {  // Project entire cloud:
    PointCloudFeature::iterator cloud_it = model_3Dcloud->begin();
    for(; cloud_it!=model_3Dcloud->end(); ++cloud_it)
    {
      PointFeature point_from_model = *cloud_it;
      cv::Point3f cv_cloud_point(point_from_model.x, point_from_model.y, point_from_model.z);
      cv_model_3D_points.push_back(cv_cloud_point);
    }
//  }
  cv::projectPoints(cv_model_3D_points, rvec_, tvec_, K_, dist_coeffs_, projected_model_2D_points);

  for(unsigned int i = 0; i < projected_model_2D_points.size(); ++i)
    {
    std::cout << "3D Object point: " << cv_model_3D_points[i] << " Projected to " << projected_model_2D_points[i] << std::endl;
    }


  // TODO: if they are within the camera's frustum
  // TODO: save into a KD-tree for fast search
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
