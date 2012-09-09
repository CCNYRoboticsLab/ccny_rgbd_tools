#include "ccny_rgbd/loop/loop_solver_sba.h"

namespace ccny_rgbd
{

LoopSolverSBA::LoopSolverSBA(ros::NodeHandle nh, ros::NodeHandle nh_private):
  LoopSolver(nh, nh_private)
{
  cam_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
    "/sba/cameras", 1);
  point_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
    "/sba/points", 1);

  cam_marker_s_pub_ = nh_.advertise<visualization_msgs::Marker>(
    "/sba/cameras_s", 1);
  point_marker_s_pub_ = nh_.advertise<visualization_msgs::Marker>(
    "/sba/points_s", 1);
}

LoopSolverSBA::~LoopSolverSBA()
{

}

void LoopSolverSBA::solve()
{
  sys_sba_ = sba::SysSBA();

  // **** SBA system ******************************************
  setUp();

  // visualize, after solving
  drawGraph(sys_sba_, cam_marker_pub_, point_marker_pub_);
  //ros::spinOnce();
  usleep(1000000);

  // solve
  printf("solving...\n");
  sys_sba_.doSBA(10, 1e-3, SBA_SPARSE_CHOLESKY);
  printf("solving done.\n");

  // visualize, after solving
  drawGraph(sys_sba_, cam_marker_pub_, point_marker_pub_);
  //ros::spinOnce();

  for (unsigned int i = 0; i < keyframes_.size(); ++i)
  {
    tf::Vector3 p(
      sys_sba_.nodes[i].trans.x(),
      sys_sba_.nodes[i].trans.y(),
      sys_sba_.nodes[i].trans.z());

    tf::Quaternion q(
      sys_sba_.nodes[i].qrot.x(),
      sys_sba_.nodes[i].qrot.y(),
      sys_sba_.nodes[i].qrot.z(),
      sys_sba_.nodes[i].qrot.w());

    keyframes_[i].pose.setOrigin(p);
    keyframes_[i].pose.setRotation(q);
  }

  publishKeyframeAssociations();
}

void LoopSolverSBA::setUp()
{
  printf("Setting up SBA system\n");

  // **** set up camera parameters ******************************
  // use camera info from first frame

  image_geometry::PinholeCameraModel& model = keyframes_[0].model_;

  frame_common::CamParams cam_params;
  cam_params.fx = model.fx(); // Focal length in x
  cam_params.fy = model.fy(); // Focal length in y
  cam_params.cx = model.cx(); // X position of principal point
  cam_params.cy = model.cy(); // Y position of principal point
  cam_params.tx = 0;          // Baseline (no baseline since this is monocular)

  // **** add nodes *********************************************
  printf("Adding nodes\n");

  for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); kf_idx++)
  { 
    tf::Transform& kf_pose = keyframes_[kf_idx].pose;

    Vector4d trans(kf_pose.getOrigin().getX(), 
                   kf_pose.getOrigin().getY(),
                   kf_pose.getOrigin().getZ(), 
                   1);
      
    Quaterniond rot(kf_pose.getRotation().getW(),
                    kf_pose.getRotation().getX(), 
                    kf_pose.getRotation().getY(),
                    kf_pose.getRotation().getZ());
    
    sys_sba_.addNode(trans, rot, cam_params, false);
  }

  printf("%d nodes added\n", (int)keyframes_.size());

  // **** from features to sba points *************************

  // counter for how many points we've added to the sba system
  int pt_idx = 0;

  printf("Iterating over %d associations\n", (int)associations_.size());

  // iterate over all associations
  for (unsigned int as_idx = 0; as_idx < associations_.size(); ++as_idx)
  { 
    // set up shortcut references
    KeyframeAssociation& association = associations_[as_idx];
    int kf_idx_a = association.kf_idx_a;
    int kf_idx_b = association.kf_idx_b;
    RGBDKeyframe& frame_a = keyframes_[kf_idx_a];
    RGBDKeyframe& frame_b = keyframes_[kf_idx_b];
    std::vector<cv::DMatch>& matches = association.matches;

    //printf("\tAssociation %d: [%d %d] %d matches\n", 
    //  as_idx, kf_idx_a, kf_idx_b, (int)matches.size());
    //printf("\tSizes: %d, %d\n", 
    //  (int)frame_a.kp_mean.size(), (int)frame_b.kp_mean.size());

    for (unsigned int m_idx = 0; m_idx < matches.size(); m_idx++)
    {
      int idx_a = matches[m_idx].queryIdx;
      int idx_b = matches[m_idx].trainIdx;
    
      //printf("\t\t Match[%d]: %d, %d\n", (int)m_idx, idx_a, idx_b);

      // check for invalid measurements
      if (frame_a.kp_valid[idx_a] && frame_b.kp_valid[idx_b])
      {
        cv::Mat mat_a = frame_a.kp_mean[idx_a];
        cv::Mat mat_b = frame_b.kp_mean[idx_b];

        // the feature point from frame_a, in the camera coords
        tf::Point point_a(mat_a.at<float>(0,0),
                          mat_a.at<float>(1,0),
                          mat_a.at<float>(2,0));

        tf::Point point_b(mat_b.at<float>(0,0),
                          mat_b.at<float>(1,0),
                          mat_b.at<float>(2,0));

        // transform into world coords
        point_a = frame_a.pose * point_a;
        point_b = frame_b.pose * point_b;

        // the same feature point, in Eigen format
        Vector4d sba_pt_a(point_a.getX(), 
                          point_a.getY(),
                          point_a.getZ(), 1.0);

        Vector4d sba_pt_b(point_b.getX(), 
                          point_b.getY(),
                          point_b.getZ(), 1.0);
       
        // the image coords (aka 2d projections)
        Vector2d proj_a(frame_a.keypoints[idx_a].pt.x, 
                        frame_a.keypoints[idx_a].pt.y);
        Vector2d proj_b(frame_b.keypoints[idx_b].pt.x, 
                        frame_b.keypoints[idx_b].pt.y);

        // correlate point a to both frames
        sys_sba_.addPoint(sba_pt_a);
        sys_sba_.addMonoProj(kf_idx_a, pt_idx, proj_a); // correct
        sys_sba_.addMonoProj(kf_idx_b, pt_idx, proj_b); // with error
        pt_idx++;

        sys_sba_.addPoint(sba_pt_b);
        sys_sba_.addMonoProj(kf_idx_a, pt_idx, proj_a); // with error
        sys_sba_.addMonoProj(kf_idx_b, pt_idx, proj_b); // correct
        pt_idx++;
      }
    }
  }
}

} // namespace ccny_rgbd
