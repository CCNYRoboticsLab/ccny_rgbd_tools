#ifndef CCNY_RGBD_MONOCULAR_FRAME_H
#define CCNY_RGBD_MONOCULAR_FRAME_H

#include "ccny_rgbd/rgbd_util.h"
#include "ccny_rgbd/structures/rgbd_frame.h"

namespace ccny_rgbd
{
class MonocularFrame : public RGBDFrame
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MonocularFrame(const sensor_msgs::ImageConstPtr& rgb_msg,
              const sensor_msgs::CameraInfoConstPtr& info_msg);

    
    // TODO:use this model
    // image_geometry::PinholeCameraModel model_;

    bool project3DModelToCamera(const PointCloudFeature::Ptr model_3Dcloud, bool is_first_time); ///< Returns false after the first time projection
    void setCameraAperture(double width, double height);

  protected:

  private:
    // Camera parameters
    cv::Mat K_; ///< intrinsic parameter matrix
    cv::Mat R_; ///< rotation matrix
    cv::Mat T_; ///< translation vector
    cv::Mat rvec_; //< rotation vector
    cv::Mat tvec_; ///< translation vector
    cv::Mat dist_coeffs_; ///< distortion coefficients
    cv::Point2d principal_point_; ///< Location of principal/optical center point in perspective camera (obtained from calibration)
    double sensor_aperture_width_, sensor_aperture_height_;

    PointCloudFeature::Ptr model_;   ///< 3D point cloud model

    typedef struct{
      cv::Point2f keyframe;
      cv::Point3f model_point;
    } model_feature_pair_;

//    cv::KDTree<cv::Point2f> tree_of_projections_to_2D_; // TODO: define properly for the structure FIXME: how to index it if it only takes points, but no structure?

    // private functions
    void processCameraInfo(const sensor_msgs::CameraInfoConstPtr &cam_info_ptr);
};

} //namespace ccny_rgbd


#endif // CCNY_RGBD_MONOCULAR_FRAME_H
