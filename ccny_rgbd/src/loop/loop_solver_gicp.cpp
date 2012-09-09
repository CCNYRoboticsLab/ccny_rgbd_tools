#include "ccny_rgbd/loop/loop_solver_gicp.h"

namespace ccny_rgbd
{

LoopSolverGICP::LoopSolverGICP(ros::NodeHandle nh, ros::NodeHandle nh_private):
  LoopSolver(nh, nh_private)
{
//  cam_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
//    "/sba/cameras", 1);

  vgf_res_ = 0.05;
  nn_count_ = 20;
  gicp_epsilon_ = 0.004;
  vgf_range_ = 6.0;
  window_size_ = 1; 

  GICPParams params;

  if (!nh_private_.getParam ("max_distance", params.max_distance))
     params.max_distance = 1.30;
  if (!nh_private_.getParam ("solve_rotation", params.solve_rotation))
     params.solve_rotation = true;
  if (!nh_private_.getParam ("max_iterations", params.max_iteration))
     params.max_iteration = 10;
  if (!nh_private_.getParam ("max_iteration_inner", params.max_iteration_inner))
     params.max_iteration_inner = 20;
  if (!nh_private_.getParam ("epsilon", params.epsilon))
     params.epsilon = 5e-4;
  if (!nh_private_.getParam ("epsilon_rot", params.epsilon_rot))
     params.epsilon_rot = 2e-3;

  params.debug = false;

  reg_.setUseColor(false);
  reg_.setParams(params);

  align_pair_service_ = nh_.advertiseService(
    "align_pair", &LoopSolverGICP::alignPairSrvCallback, this);
}

LoopSolverGICP::~LoopSolverGICP()
{

}

bool LoopSolverGICP::alignPairSrvCallback(
  ccny_rgbd_vo::AlignPair::Request&  request,
  ccny_rgbd_vo::AlignPair::Response& response)
{
  alignPair(request.idx_a, request.idx_b);
  return true;
}


void LoopSolverGICP::alignPair(int idx_a, int idx_b)
{
  pcl::VoxelGrid<PointT> vgf; //TODO make member
  vgf.setLeafSize (vgf_res_, vgf_res_, vgf_res_);
  vgf.setFilterFieldName ("z");
  vgf.setFilterLimits (0, vgf_range_);

  RGBDKeyframe& keyframe_a = keyframes_[idx_a];
  RGBDKeyframe& keyframe_b = keyframes_[idx_b];

  keyframe_a.constructDataCloud();
  keyframe_b.constructDataCloud();
  
  PointCloudT::Ptr cloud_in_a = 
    boost::shared_ptr<PointCloudT>(new PointCloudT());
  PointCloudT::Ptr cloud_in_b = 
    boost::shared_ptr<PointCloudT>(new PointCloudT());

  *cloud_in_a = keyframe_a.data;
  *cloud_in_b = keyframe_b.data;

  PointCloudT cloud_a;
  PointCloudT cloud_b;

  vgf.setInputCloud(cloud_in_a);
  vgf.filter(cloud_a);

  vgf.setInputCloud(cloud_in_b);
  vgf.filter(cloud_b);

  pcl_ros::transformPointCloud(cloud_a, cloud_a, keyframe_a.pose);
  pcl_ros::transformPointCloud(cloud_b, cloud_b, keyframe_b.pose);

  // ***** set up GICP data set ********************************
  printf("\t\tSetting up data...\n");

  ccny_gicp::GICPPointSetKd gicp_data;
  gicp_data.setNormalCount(nn_count_);

  for (unsigned int i = 0; i < cloud_a.points.size(); ++i)
  {
    PointGICP p;
    p.x   = cloud_a.points[i].x;
    p.y   = cloud_a.points[i].y;
    p.z   = cloud_a.points[i].z;
    p.rgb = cloud_a.points[i].rgb;
    gicp_data.AppendPoint(p);
  }

  // create kd tree
  KdTree gicp_data_kdtree;
  gicp_data_kdtree.setInputCloud(gicp_data.getCloud());
  gicp_data.setKdTree(&gicp_data_kdtree);
  
  // compute matrices
  gicp_data.SetGICPEpsilon(gicp_epsilon_);
  gicp_data.computeMatrices();

  reg_.setData(&gicp_data);

  // **** set up gicp model set ********************************
  printf("\t\tSetting up model...\n");

  ccny_gicp::GICPPointSetKd gicp_model;
  gicp_model.setNormalCount(nn_count_);

  for (unsigned int i = 0; i < cloud_b.points.size(); ++i)
  {
    PointGICP p;
    p.x   = cloud_b.points[i].x;
    p.y   = cloud_b.points[i].y;
    p.z   = cloud_b.points[i].z;
    p.rgb = cloud_b.points[i].rgb;
    gicp_model.AppendPoint(p);
  }

  // create kd tree
  KdTree gicp_model_kdtree;
  gicp_model_kdtree.setInputCloud(gicp_model.getCloud());
  gicp_model.setKdTree(&gicp_model_kdtree);
  
  // compute matrices
  gicp_model.SetGICPEpsilon(gicp_epsilon_);
  gicp_model.computeMatrices();

  reg_.setModel(&gicp_model);

  // **** perform alignment
  printf("\t\tAligning...\n");

  Eigen::Matrix4f corr_eigen;
  reg_.align(corr_eigen);

  keyframe_a.pose = tfFromEigen(corr_eigen) * keyframe_a.pose;
}

void LoopSolverGICP::solve()
{
  // **** downsample all frames and build a vector of pointclouds

  printf("Filtering clouds...\n");

  pcl::VoxelGrid<PointT> vgf; //TODO make member
  vgf.setLeafSize (vgf_res_, vgf_res_, vgf_res_);
  vgf.setFilterFieldName ("z");
  vgf.setFilterLimits (0, vgf_range_);

  PointCloudTVector clouds;
  clouds.resize(keyframes_.size());

  for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
  {
    RGBDKeyframe& keyframe = keyframes_[kf_idx];

    keyframe.constructDataCloud();

    PointCloudT::Ptr cloud_in = 
      boost::shared_ptr<PointCloudT>(new PointCloudT());

    *cloud_in = keyframe.data;

    PointCloudT cloud_filtered;

    vgf.setInputCloud(cloud_in);
    vgf.filter(cloud_filtered);

    clouds[kf_idx] = cloud_filtered;
  }

  // *******************************************************************

  std::vector<tf::Transform> transforms;
  transforms.resize(keyframes_.size());

  printf("Aligning...\n");

  for (int kf_idx = 0; kf_idx < (int)keyframes_.size(); ++kf_idx)
  {
    printf("\tFrame [%d]...\n", kf_idx);

    RGBDKeyframe& keyframe = keyframes_[kf_idx];
    PointCloudT data;
    pcl_ros::transformPointCloud(clouds[kf_idx], data, keyframe.pose);

    PointCloudT accumulated;

    // accumulate data from all neighbors
    
    printf("\t\tAccumulating model:\n\t\t\t");
    for (int w_idx = -window_size_; w_idx <= window_size_; w_idx++)
    //for (int w_idx = 0; w_idx <= window_size_; w_idx++)
    {
      // calculate index with wraparound, and skip self 
      int kf_idx_n = (kf_idx + w_idx);
      if (kf_idx == kf_idx_n) continue; 
      else if (kf_idx_n < 0) 
        kf_idx_n += (int)keyframes_.size();
      else if (kf_idx_n >= (int)keyframes_.size())
        kf_idx_n -= (int)keyframes_.size();

      printf("%d ", kf_idx_n);

      // set up references
      RGBDKeyframe& keyframe_n = keyframes_[kf_idx_n];
      PointCloudT&  cloud_n    = clouds[kf_idx_n];

      // transform and accumulate into a single cloud
      PointCloudT cloud_tf;
      pcl_ros::transformPointCloud(cloud_n, cloud_tf, keyframe_n.pose);
      accumulated += cloud_tf;    
    }
    printf("\n");

    // ***** set up GICP data set ********************************
    printf("\t\tSetting up data...\n");
 
    ccny_gicp::GICPPointSetKd gicp_data;
    gicp_data.setNormalCount(nn_count_);
  
    for (unsigned int i = 0; i < data.points.size(); ++i)
    {
      PointGICP p;
      p.x   = data.points[i].x;
      p.y   = data.points[i].y;
      p.z   = data.points[i].z;
      p.rgb = data.points[i].rgb;
      gicp_data.AppendPoint(p);
    }

    // create kd tree
    KdTree gicp_data_kdtree;
    gicp_data_kdtree.setInputCloud(gicp_data.getCloud());
    gicp_data.setKdTree(&gicp_data_kdtree);
    
    // compute matrices
    gicp_data.SetGICPEpsilon(gicp_epsilon_);
    gicp_data.computeMatrices();

    reg_.setData(&gicp_data);

    // **** set up gicp model set ********************************
    printf("\t\tSetting up model...\n");

    ccny_gicp::GICPPointSetKd gicp_model;
    gicp_model.setNormalCount(nn_count_);
  
    for (unsigned int i = 0; i < accumulated.points.size(); ++i)
    {
      PointGICP p;
      p.x   = accumulated.points[i].x;
      p.y   = accumulated.points[i].y;
      p.z   = accumulated.points[i].z;
      p.rgb = accumulated.points[i].rgb;
      gicp_model.AppendPoint(p);
    }

    // create kd tree
    KdTree gicp_model_kdtree;
    gicp_model_kdtree.setInputCloud(gicp_model.getCloud());
    gicp_model.setKdTree(&gicp_model_kdtree);
    
    // compute matrices
    gicp_model.SetGICPEpsilon(gicp_epsilon_);
    gicp_model.computeMatrices();

    reg_.setModel(&gicp_model);

    // **** perform alignment
    printf("\t\tAligning...\n");

    Eigen::Matrix4f corr_eigen;
    reg_.align(corr_eigen);
    transforms[kf_idx] = tfFromEigen(corr_eigen);
  }

  // **** apply transformations
    
  printf("Propagating tfs...\n");

  for (int kf_idx = 0; kf_idx < (int)keyframes_.size(); ++kf_idx)
  {
    RGBDKeyframe& keyframe = keyframes_[kf_idx];
    tf::Transform corr = transforms[kf_idx];
   
    keyframe.pose = corr * keyframe.pose;
  }
}

  

} // namespace ccny_rgbd
