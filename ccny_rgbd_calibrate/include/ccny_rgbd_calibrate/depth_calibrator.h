#ifndef CCNY_RGBD_CALIBRATE_DEPTH_CALIBRATOR_H
#define CCNY_RGBD_CALIBRATE_DEPTH_CALIBRATOR_H

#include <gsl/gsl_multifit.h>
#include <gsl/gsl_fit.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <boost/filesystem.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <ccny_rgbd/proc_util.h>
#include "ccny_rgbd_calibrate/calib_util.h"

namespace ccny_rgbd
{

class DepthCalibrator
{
  typedef struct  {
    uint16_t ground_truth;
    uint16_t measured;
  } ReadingPair;
  
  typedef std::vector<double> DoubleVector;
  typedef std::vector<ReadingPair> ReadingVector;
    
  public:

    DepthCalibrator(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~DepthCalibrator();

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // parameters
    std::string path_;
    double square_size_;
    int n_cols_;
    int n_rows_;
    int fit_window_size_;
    int fit_mode_;    

    // paths & filenames
    std::string cloud_path_;
    std::string train_path_;
    std::string test_path_;

    std::string calib_rgb_filename_;
    std::string calib_ir_filename_;
    std::string calib_extr_filename_;
    std::string calib_warp_filename_;
    
    std::string rgb_test_filename_;
    std::string depth_test_filename_;
              
    // calibration variables
    cv::Size patternsize_;
   
    std::vector<cv::Point3f> corners_3d_;
    
    cv::Mat intr_rgb_, intr_ir_;
    cv::Mat dist_rgb_, dist_ir_;
        
    cv::Mat intr_rect_rgb_, intr_rect_ir_;
    
    cv::Mat map_rgb_1_, map_rgb_2_;
    cv::Mat map_ir_1_,  map_ir_2_;
    
    cv::Mat ir2rgb_;
    
    void calibrate();
    void build3dCornerVector();
    void buildRectMaps();
    bool loadCameraParams();

    void testExtrinsicCalibration();
    void testDepthCalibrationRMS();
    
    bool loadCalibrationImagePair(
      int idx,
      cv::Mat& rgb_img,
      cv::Mat& depth_img);
    
    bool loadTestImagePair(
      int idx,
      cv::Mat& rgb_img,
      cv::Mat& depth_img);
        
    void testPlaneDetection(
      const cv::Mat& rgb_img_rect,
      const cv::Mat& depth_img_rect,
      const cv::Mat& rvec,
      const cv::Mat& tvec);
    
    bool processTrainingImagePair(
      int img_idx,
      const cv::Mat& rgb_img,
      const cv::Mat& depth_img,
      cv::Mat& depth_img_g,
      cv::Mat& depth_img_m);

    void fitData(
      const ReadingVector& v, 
      std::vector<double>& coeff);

    void quadraticFit(
      const ReadingVector& v, 
      std::vector<double>& coeff);

    void quadraticFitZero(
      const ReadingVector& v, 
      std::vector<double>& coeff);

    void linearFit(
      const ReadingVector& v, 
      std::vector<double>& coeff);
    
    void linearFitZero(
      const ReadingVector& v, 
      std::vector<double>& coeff);

    void testDepthCalibration();
    double getRMSError(const cv::Mat& g, const cv::Mat& m);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_CALIBRATE_DEPTH_CALIBRATOR_H
