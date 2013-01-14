#ifndef CCNY_RGBD_KEYFRAME_MAPPER_H
#define CCNY_RGBD_KEYFRAME_MAPPER_H

#include <ros/ros.h>
#include <ros/publisher.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "ccny_rgbd/types.h"
#include "ccny_rgbd/structures/rgbd_frame.h"
#include "ccny_rgbd/structures/rgbd_keyframe.h"
#include "ccny_rgbd/mapping/keyframe_loop_detector.h"
#include "ccny_rgbd/mapping/keyframe_loop_solver.h"
#include "ccny_rgbd/mapping/keyframe_loop_solver_toro.h"
#include "ccny_rgbd/mapping/keyframe_loop_solver_g2o.h"

#include "ccny_rgbd/GenerateAssociations.h"
#include "ccny_rgbd/SolveLoop.h"
#include "ccny_rgbd/AddManualKeyframe.h"
#include "ccny_rgbd/PublishKeyframe.h"
#include "ccny_rgbd/PublishAllKeyframes.h"
#include "ccny_rgbd/Save.h"
#include "ccny_rgbd/Load.h"

namespace ccny_rgbd
{

class KeyframeMapper
{
  public:

    KeyframeMapper(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~KeyframeMapper();

    bool publishAllKeyframesSrvCallback(
      PublishAllKeyframes::Request& request,
      PublishAllKeyframes::Response& response);

    bool publishKeyframeSrvCallback(
      PublishKeyframe::Request& request,
      PublishKeyframe::Response& response);

    bool saveKeyframesSrvCallback(
      Save::Request& request,
      Save::Response& response);

    bool saveFullSrvCallback(
      Save::Request& request,
      Save::Response& response);

    bool saveKeyframesFFSrvCallback(
      Save::Request& request,
      Save::Response& response);

    bool loadKeyframesSrvCallback(
      Load::Request& request,
      Load::Response& response);

    bool addManualKeyframeSrvCallback(
      AddManualKeyframe::Request& request,
      AddManualKeyframe::Response& response);
    
     bool generateAssociationsSrvCallback(
      GenerateAssociations::Request& request,
      GenerateAssociations::Response& response);

    bool solveLoopSrvCallback(
      SolveLoop::Request& request,
      SolveLoop::Response& response);
    
  protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    std::string fixed_frame_;
    
    KeyframeVector keyframes_;
    
    virtual void RGBDCallback(
      const ImageMsg::ConstPtr& depth_msg,
      const ImageMsg::ConstPtr& rgb_msg,
      const CameraInfoMsg::ConstPtr& info_msg);

  private:

    ros::Publisher keyframes_pub_;
    ros::Publisher poses_pub_;
    ros::Publisher edges_pub_;
    ros::Publisher associations_pub_;
    
    ros::ServiceServer generate_associations_service_;
    ros::ServiceServer solve_loop_service_;   
    ros::ServiceServer pub_frames_service_;
    ros::ServiceServer pub_frame_service_;
    ros::ServiceServer save_kf_service_;
    ros::ServiceServer save_full_service_;
    ros::ServiceServer load_kf_service_;
    ros::ServiceServer save_kf_ff_service_;
    ros::ServiceServer add_manual_keyframe_service_;

    tf::TransformListener tf_listener_;

    boost::shared_ptr<image_transport::ImageTransport> rgb_it_;
    boost::shared_ptr<image_transport::ImageTransport> depth_it_;
    boost::shared_ptr<Synchronizer> sync_;
        
    ImageSubFilter      sub_depth_;
    ImageSubFilter      sub_rgb_;
    CameraInfoSubFilter sub_info_;

    // params
    double full_map_res_;
    double kf_dist_eps_;
    double kf_angle_eps_;
          
    // state vars
    bool manual_add_;

    KeyframeLoopDetector loop_detector_;
    KeyframeLoopSolver   * loop_solver_;

    KeyframeAssociationVector associations_;
    
    bool processFrame(const RGBDFrame& frame, const tf::Transform& pose);
    void addKeyframe(const RGBDFrame& frame, const tf::Transform& pose);

    void publishMatchEdge(int i, int j);
    void publishKeyframeData(int i);
    void publishKeyframePose(int i);
    void publishEdges();  
    void publishKeyframeAssociations();
    
    bool saveFullMap(const std::string& path);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_MAPPER_H
