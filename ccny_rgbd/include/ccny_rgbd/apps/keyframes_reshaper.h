#ifndef CCNY_RGBD_KEYFRAMES_RESHAPER
#define CCNY_RGBD_KEYFRAMES_RESHAPER

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Time.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/mls.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>

#include "ccny_rgbd/types.h"
#include "ccny_rgbd/rgbd_util.h"
#include "ccny_rgbd/structures/rgbd_keyframe.h"

//#define USE_RGB_COLOR

namespace ccny_rgbd
{
#ifdef USE_RGB_COLOR
    typedef pcl::PointXYZRGB                    PointFilteredT;
    typedef pcl::PointCloud<PointFilteredT>     PointCloudFilteredT;
    typedef pcl::PointXYZRGBNormal              PointNormalT;
#else
    typedef pcl::PointXYZ                       PointFilteredT;
    typedef pcl::PointCloud<PointFilteredT>     PointCloudFilteredT;
    typedef pcl::PointXYZINormal                PointNormalT;
#endif

class KeyFramesReshaper
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KeyFramesReshaper(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~KeyFramesReshaper();


  private:

    FILE * xf_conf_file_; // For saving configuration on names and position

    // **** ROS-related
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // **** parameters 
    std::string path_to_keyframes_;
    int keyframe_dir_num_of_chars_;
    bool generate_config_file_;
    bool load_as_keyframes_; ///< If set true, it will use the CCNY_RGBD Keyframe loading mechanism
    int first_keyframe_number_;
    int last_keyframe_number_;

    double vgf_res_;
    double neighbor_max_proximity_;
    double smoothing_res_;


    std::string input_cloud_filename_;
    std::string output_filename_;
    std::string fixed_frame_;
    std::string base_frame_;


    // **** variables
    int  frame_count_;
    ros::Time init_time_;

    PointCloudT::Ptr model_ptr_;

    // **** private functions
    bool reshapeSingleKeyFrame(int keyframe_number);
    void processKeyframes();
    std::string formKeyframeName(int keyframe_number, int num_of_chars);
    void generateKeyframePath(const std::string& keyframe_path, int keyframe_number, std::string& current_keyframe_path);
    void initParams();
    void deleteNaNs(const PointCloudFilteredT::Ptr& cloud_in, PointCloudFilteredT::Ptr& cloud_out) const;
    void filterCloud(const PointCloudFilteredT::ConstPtr& cloud_in, PointCloudFilteredT::Ptr& cloud_out, double vgf_res, double neighbor_max_proximity, double smoothing_res = 0.0) const;
    void saveCloudAsPLY(const PointCloudFilteredT::Ptr& cloud_in, const std::string& ply_name) const;


    bool readPointCloudFromPCDFile(); ///< Returns true if PCD file was read successfully.

};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAMES_RESHAPER
