#ifndef CCNY_RGBD_CALIBRATE_UNCERTAINTY_COMPARATOR_H 
#define CCNY_RGBD_CALIBRATE_UNCERTAINTY_COMPARATOR_H 

#include <tf/transform_listener.h>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ccny_rgbd/types.h>
#include <ccny_rgbd/rgbd_util.h>

namespace ccny_rgbd
{

class UncertaintyComparator
{
  public:

    UncertaintyComparator(const ros::NodeHandle& nh, 
              const ros::NodeHandle& nh_private);
    
    virtual ~UncertaintyComparator();

    void RGBDCallback(const ImageMsg::ConstPtr& rgb_msg,
                      const ImageMsg::ConstPtr& depth_msg,
                      const CameraInfoMsg::ConstPtr& info_msg);  

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    boost::shared_ptr<RGBDSynchronizer3> sync_;
       
    ImageSubFilter      sub_depth_;
    ImageSubFilter      sub_rgb_;
    CameraInfoSubFilter sub_info_;

    std::string rgb_test_path_;
    
    std::string depth_test_path_;
    std::string depth_gt_path_;
    
    std::string stdev_gt_path_;
    std::string stdev_q_path_;
    std::string stdev_qgmm_path_;

    boost::thread input_thread_;
    
    // state variables
    int count_;
    bool logging_;
    bool rgb_saved_;
    
    cv::Mat c_img_;   // uint16t
    cv::Mat m_img_;   // double
    cv::Mat s_img_;   // double
       
    cv::Mat rgb_test_img_;
    cv::Mat depth_gt_img_; 
    cv::Mat depth_test_img_;
        
    cv::Mat stdev_gt_img_;
    cv::Mat stdev_q_img_;
    cv::Mat stdev_qgmm_img_;     
        
    double w0_, w1_, w2_;    
        
    // parameters   
    int n_depth_;
    int id_;   
    
    std::string path_;

    void prepareDirectories();
    
    void keyboardThread();
    
    double getStDevGT(int v, int u);

    void buildUncertaintyImages();
    void buildStDevGroundTruthImage();
    void buildStDevQuadraticImage();
    void buildStDevQuadraticGMMImage();
    void getGaussianMixtureDistribution(
      int u, int v, double& z_mean, double& z_var);
      
    void evaluateRMSError(double& rms_q, double& rms_qgmm);
    void evaluateRMSErrorFeatures(double& rms_q, double& rms_qgmm);
};

} // namespace ccny_rgbd

#endif // CCNY_RGBD_CALIBRATE_UNCERTAINTY_COMPARATOR_H 
