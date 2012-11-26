#ifndef CCNY_RGBD_CALIBRATE_DEPTH_CALIBRATOR_H
#define CCNY_RGBD_CALIBRATE_DEPTH_CALIBRATOR_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <boost/filesystem.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

namespace ccny_rgbd
{

class DepthCalibrator
{
  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;

  typedef std::vector<cv::Point2f> Point2fVector;
  typedef std::vector<cv::Point3f> Point3fVector;
  
  public:

    DepthCalibrator(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~DepthCalibrator();

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // parameters
    float square_size_;
    int n_cols_;
    int n_rows_;
    
    cv::Size patternsize_;
    
    // input filenames
    std::string path_;
    std::string calib_rgb_filename_;
    std::string calib_ir_filename_;

    std::string rgb_test_filename_;
    std::string depth_test_filename_;
       
    //output filenames
    std::string calib_extrinsic_filename_;
   
    std::string cloud_filename_;
    
    // input to calibration       
    std::vector<cv::Point3f> corners_3d_;
    
    cv::Mat intr_rgb_, intr_ir_;
    cv::Mat dist_rgb_, dist_ir_;
    
    // output of calibration
    cv::Mat extr_rgb_, extr_ir_;
    
    cv::Mat intr_rect_rgb_, intr_rect_ir_;
    
    cv::Mat map_rgb_1_, map_rgb_2_;
    cv::Mat map_ir_1_,  map_ir_2_;
    
    cv::Mat rgb2ir_;
    
    void calibrate();
    void build3dCornerVector();
    void buildRectMaps();
    bool loadCameraParams();
 
    bool getCorners(
      const cv::Mat& img, 
      std::vector<cv::Point2f>& corners);

    void testExtrinsicCalibration();
    
    bool loadCalibrationImagePair(
      int idx,
      cv::Mat& rgb_img,
      cv::Mat& depth_img);

    void create8bImage(
      const cv::Mat depth_img,
      cv::Mat& depth_img_u);
    
    void testPlaneDetection(
      const cv::Mat& rgb_img_rect,
      const cv::Mat& depth_img_rect,
      const cv::Mat& rvec,
      const cv::Mat& tvec);
    
    void buildPouintCloud(
      const cv::Mat& depth_img_rect_reg,
      const cv::Mat& rgb_img_rect,
      const cv::Mat& intr_rect_rgb,
      PointCloudT& cloud);
    
    void blendImages(const cv::Mat& rgb_img,
                     const cv::Mat depth_img,
                     cv::Mat& blend_img);
    
    void matrixFromRvecTvec(const cv::Mat& rvec,
                            const cv::Mat& tvec,
                            cv::Mat& E);

    void matrixFromRT(const cv::Mat& rmat,
                      const cv::Mat& tvec,
                      cv::Mat& E);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_CALIBRATE_DEPTH_CALIBRATOR_H
