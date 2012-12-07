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

    ~MonocularFrame();
    
    // TODO:use this model
    // image_geometry::PinholeCameraModel model_;

    bool isPointWithinFrame(const cv::Point2f &point) const;

    bool project3DModelToCamera(const PointCloudFeature::Ptr model_3Dcloud, bool is_first_time); ///< Returns false after the first time projection
    void setCameraAperture(double width, double height);
    void setFrame(const sensor_msgs::ImageConstPtr& rgb_msg);
    void setCameraModel(const sensor_msgs::CameraInfoConstPtr& info_msg);
    void setExtrinsicMatrix(const cv::Mat &E);

//    boost::shared_ptr<cv::flann::KDTreeIndexParams flann::KDTreeSingleIndex> tree_2D_points_from_cloud_;

    cv::Mat getIntrinsicCameraMatrix() const;
    void getFeaturesVector(std::vector<cv::Point2d> &features_vector) const;

  protected:

  private:
    // Camera parameters
    cv::Size *image_size_; ///< Image size: width x height in pixels
    cv::Mat K_; ///< intrinsic parameter matrix
    cv::Mat R_; ///< rotation matrix
    cv::Mat T_; ///< translation vector
    cv::Mat rvec_; //< rotation vector
    cv::Mat tvec_; ///< translation vector
    cv::Mat dist_coeffs_; ///< distortion coefficients
    cv::Point2d principal_point_; ///< Location of principal/optical center point in perspective camera (obtained from calibration)
    double sensor_aperture_width_, sensor_aperture_height_;

    PointCloudFeature::Ptr pointcloud_model_;   ///< 3D point cloud model

    // Not being used for now
    typedef struct ModelFeaturePair{
      cv::Point2d keyframe;
      cv::Point3d model_point;

      ModelFeaturePair(const cv::Point3d &point3D, const cv::Point2d & point2D)
      {
        model_point = point3D;
        keyframe = point2D;
      }
    } model_feature_pair_;

    std::vector<cv::Point3d> valid_3D_points_; ///< 3D points that project within frame
    std::vector<cv::Point2d> valid_2D_points_; ///< Valid 2D points projected from 3D model
//    boost::shared_ptr<flann::Matrix<cv::Point2f> > valid_2D_points_;

    void filterPointsWithinFrame(const std::vector<cv::Point3d> &all_3D_points, const std::vector<cv::Point2d> &all_2D_points);
//    void filterPointsWithinFrame(const std::vector<cv::Point3f> &all_3D_points, const std::vector<cv::Point2f> &all_2D_points, std::vector<model_feature_pair_> &valid_points);

//    cv::KDTree<cv::Point2f> tree_of_projections_to_2D_; // TODO: define properly for the structure FIXME: how to index it if it only takes points, but no structure?

    // private functions
    void processCameraInfo(const sensor_msgs::CameraInfoConstPtr &cam_info_ptr);
};

} //namespace ccny_rgbd


#endif // CCNY_RGBD_MONOCULAR_FRAME_H
