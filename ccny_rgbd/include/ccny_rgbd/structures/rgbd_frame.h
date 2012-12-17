#ifndef CCNY_RGBD_RGBD_FRAME_H
#define CCNY_RGBD_RGBD_FRAME_H

#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>

#include "ccny_rgbd/types.h"
#include "ccny_rgbd/rgbd_util.h"

namespace ccny_rgbd
{

// Khoshelham, K.; Elberink, S.O. Accuracy and Resolution of Kinect 
// Depth Data for Indoor Mapping Applications. Sensors 2012, 12, 1437-1454.
const double Z_STDEV_CONSTANT = 0.001425;

class RGBDFrame
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RGBDFrame();

    RGBDFrame(const sensor_msgs::ImageConstPtr& rgb_msg,
              const sensor_msgs::ImageConstPtr& depth_msg,
              const sensor_msgs::CameraInfoConstPtr& info_msg);
    
    // header taken from rgb_msg
    std_msgs::Header header;

    // RGB image (8UC3)
    cv::Mat rgb_img;
  
    // Depth image in mm (16UC1). 0 = invalid data
    cv::Mat depth_img;

    // the intrinsic matrix which applies to both images. 
    // it's assumed that the images are already
    // rectified and in the same camera frame(RGB)
    image_geometry::PinholeCameraModel model;
    
    // keypoints, descriptos, and kp_* are index the same way
    // and may inculude invalid points
    // invalid point: no z data, or var_z > threshold

    KeypointVector keypoints;         // 2D keypoint locations
    cv::Mat        descriptors;       // feature descriptor vectors

    BoolVector     kp_valid;          // is the z data valid?
    Vector3fVector kp_means;          // 1x3 mat of 3D locations
    Matrix3fVector kp_covariances;    // 3x3 mat of covariances

    int n_valid_keypoints;            // how many keypoints have usable 3D data

    // PCL points, invalid kp's removed
    // TODO: remove this from class
    PointCloudFeature kp_cloud;

    /* computes the 3D means and covariances for all the detected
     * keypoints, and determines if tehy are valid or not
     * TODO: do we want default values? 
     */
    void computeDistributions(
      double max_z = 5.5,
      double max_stdev_z = 0.03);    

   void constructFeaturePointCloud(PointCloudFeature& cloud);

  protected:

    double getVarZ(double z);
    double getStdDevZ(double z);

    void getGaussianDistribution(int u, int v, double& z_mean, double& z_var);
    void getGaussianMixtureDistribution(int u, int v, double& z_mean, double& z_var);
};

bool saveFrame(const RGBDFrame&, const std::string& path);

bool loadFrame(RGBDFrame&, const std::string& path);

} // namespace ccny_rgbd

#endif // CCNY_RGBD_RGBD_FRAME_H
