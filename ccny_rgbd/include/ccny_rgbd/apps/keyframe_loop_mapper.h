#ifndef CCNY_RGBD_KEYFRAME_LOOP_MAPPER_H
#define CCNY_RGBD_KEYFRAME_LOOP_MAPPER_H

#include <tf/transform_listener.h>

#include "ccny_rgbd/apps/keyframe_mapper.h"
#include "ccny_rgbd/mapping/keyframe_loop_detector.h"
#include "ccny_rgbd/mapping/keyframe_loop_solver.h"
#include "ccny_rgbd/mapping/keyframe_loop_solver_toro.h"

#include "ccny_rgbd/GenerateAssociations.h"
#include "ccny_rgbd/AddManualAssociation.h"
#include "ccny_rgbd/SolveLoop.h"

namespace ccny_rgbd
{

class KeyframeLoopMapper: public KeyframeMapper
{
  public:

    KeyframeLoopMapper(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~KeyframeLoopMapper();

    void RGBDCallback(const ImageMsg::ConstPtr& depth_msg,
                      const ImageMsg::ConstPtr& rgb_msg,
                      const CameraInfoMsg::ConstPtr& info_msg);

    bool generateAssociationsSrvCallback(
      GenerateAssociations::Request& request,
      GenerateAssociations::Response& response);

    bool addManualAssociationSrvCallback(
      AddManualAssociation::Request& request,
      AddManualAssociation::Response& response);

    bool solveLoopSrvCallback(
      SolveLoop::Request& request,
      SolveLoop::Response& response);

  private:

    ros::Publisher associations_pub_;
    ros::ServiceServer generate_associations_service_;
    ros::ServiceServer add_manual_association_service_;
    ros::ServiceServer solve_loop_service_;

    KeyframeLoopDetector * loop_detector_;
    KeyframeLoopSolver   * loop_solver_;

    KeyframeAssociationVector associations_;

    void publishKeyframeAssociations();
};

} // namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_LOOP_MAPPER_H
