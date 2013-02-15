#include "ccny_rgbd/apps/keyframes_reshaper.h"

namespace ccny_rgbd {

KeyFramesReshaper::KeyFramesReshaper(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  frame_count_(0)
{
  ROS_INFO("Starting KeyFrames Reshaper");
  // **** init parameters
  initParams();

  processKeyframes();
}

KeyFramesReshaper::~KeyFramesReshaper()
{
  ROS_INFO("Destroying ReshapeKeyFrames");
}

void KeyFramesReshaper::processKeyframes()
{
  std::string xf_conf_file_name = path_to_keyframes_ + output_filename_ + ".xf";
  xf_conf_file_ = fopen(xf_conf_file_name.c_str(), "w");

  for(frame_count_ = first_keyframe_number_; frame_count_ <= last_keyframe_number_; frame_count_++)
  {
    reshapeSingleKeyFrame(frame_count_);
  }

  fclose(xf_conf_file_);

}

std::string KeyFramesReshaper::formKeyframeName(int keyframe_number, int num_of_chars)
{
  std::stringstream ss;
  ss << keyframe_number;
  std::string keyframe_number_as_str = ss.str();

  while(keyframe_number_as_str.size() < (uint) num_of_chars)
    keyframe_number_as_str = "0" + keyframe_number_as_str;

  return keyframe_number_as_str;
}

void KeyFramesReshaper::generateKeyframePath(const std::string& keyframe_path, int keyframe_number, std::string& current_keyframe_path)
{
  current_keyframe_path = keyframe_path + formKeyframeName(keyframe_number, 4);
  std::cout << "Current Frame: " << current_keyframe_path << std::endl;
}

bool KeyFramesReshaper::reshapeSingleKeyFrame(int keyframe_number)
{
  tf::Transform transform_est; // Frame to frame

  std::string current_keyframe_path;
  std::string next_keyframe_path;

  generateKeyframePath(path_to_keyframes_, keyframe_number, current_keyframe_path);

  bool current_success = false;

  RGBDKeyframe current_keyframe;
  current_success = loadKeyframe(current_keyframe, current_keyframe_path);

  PointCloudT::ConstPtr cloud_in (new PointCloudT(current_keyframe.cloud));

  PointCloudT::Ptr cloud_without_NaNs (new PointCloudT);
  deleteNaNs(cloud_in, cloud_without_NaNs);

  PointCloudT::Ptr cloud_out (new PointCloudT);
  filterCloud(cloud_without_NaNs, cloud_out, vgf_res_, neighbor_max_proximity_, smoothing_res_);

  pcl::PCDWriter writer;
  std::string downsampled_pcd_filename;
  downsampled_pcd_filename = current_keyframe_path + "/" + output_filename_;

  int result_pcd = writer.writeBinary<PointT>(downsampled_pcd_filename+".pcd", *cloud_out);

  //pcl::io::saveVTKFile (downsampled_pcd_filename+".vtk", *cloud_out);
  pcl::io::savePLYFile(downsampled_pcd_filename+".ply", *cloud_out, false); // We don't want the ply in binary mode?

  //cv::Mat cv_intrinsic = current_keyframe.model.intrinsicMatrix();
  //std::cout << "CV Intrinsic matrix: " << cv_intrinsic <<std::endl;

  /*
  Matrix3f intrinsic;
  float scale_factor_intrinsic = 1.0;
  if(next_success)
  {
    cv::Mat cv_intrinsic = next_keyframe.model.intrinsicMatrix();
    //std::cout << "CV Intrinsic matrix: " << cv_intrinsic <<std::endl;
    openCVRToEigenR(cv_intrinsic,intrinsic);
    // Rescale intrinsice
    scale_factor_intrinsic =  (float) image_width_ / (float) next_keyframe.rgb_img.cols;
    printf("Scale factor = %f \n", scale_factor_intrinsic);
    intrinsic(0,0) = scale_factor_intrinsic*intrinsic(0,0); // fx
    intrinsic(0,2) = scale_factor_intrinsic*intrinsic(0,2); // cx
    intrinsic(1,1) = scale_factor_intrinsic*intrinsic(1,1); // fy
    intrinsic(1,2) = scale_factor_intrinsic*intrinsic(1,2); // cy
  }

  f2b_ = current_keyframe.pose; // Initialized to first keyframe's position // TODO: temporarily

  while(current_success && next_success)
  {
    cv::Mat new_img;
    cv::resize(next_keyframe.rgb_img, new_img,cv::Size(image_width_, image_height_) );

    tfFromImagePair(
      current_keyframe.rgb_img,
      new_img,
      current_keyframe.depth_img,
      intrinsic,
      transform_est,
      max_descriptor_space_distance_,
      detector_type_,
      descriptor_type_,
      number_of_iterations_,
      (float) reprojection_error_,
      min_inliers_count_,
      true,
      true // profiling delays
      );
    // Compare to ground truth
    tf::Transform transform_gt = (current_keyframe.pose).inverse() * next_keyframe.pose;

    // output
    cv::Mat tvec_est, rmat_est;
    tfToOpenCVRt(transform_est, rmat_est, tvec_est);
    std::cout << "ESTIMATION:" << std::endl;
    std::cout << "tvec:" << tvec_est << std::endl << "rmat:" << rmat_est << std::endl;

    cv::Mat tvec_gt, rmat_gt;
    tfToOpenCVRt(transform_gt, rmat_gt, tvec_gt);
    std::cout << "GROUND TRUTH:" << std::endl;
    std::cout << "tvec:" << tvec_gt << std::endl << "rmat:" << rmat_gt << std::endl;

    // Error metric:
    double dist, angle;
    getTfDifference(transform_gt, transform_est, dist, angle);
    std::cout << "ERROR:" << std::endl;
    std::cout << "\t Distance difference: " << dist << " m" <<
    std::endl << "\t Angle difference: " << RAD2DEG(angle) << " degrees" << std::endl;


    cv::waitKey(0);

    f2b_ = f2b_ * transform_est;
    publishTransform(f2b_, fixed_frame_, base_frame_);

    if(publish_cloud_model_)
    {
      PointCloudT cloud_next = next_keyframe.cloud;
//      cloud_next.header.frame_id = base_frame_;
      cloud_next.header.frame_id = fixed_frame_; // FIXME: Using fixed_problem to publish and visualize in the "odom" frame without need for transformation
      std::string cloud_filename_next_est = current_keyframe_path + "_next_est.pcd";
      // derotate to fixed frame if needed
      PointCloudT cloud_next_transformed_est;
      pcl::transformPointCloud(cloud_next, cloud_next_transformed_est, eigenFromTf(f2b_));
      pub_cloud_est_.publish(cloud_next_transformed_est);
    }
    // +++++++++++++++++++++++++ Advance frames  ++++++++++++++++++++++++++++++++++++++++

    ++keyframe_number;
    generateKeyframePath(keyframe_path, keyframe_number, current_keyframe_path, next_keyframe_path);

    current_success = loadKeyframe(current_keyframe, current_keyframe_path);
    if(current_success)
    {
      if(loadKeyframe(next_keyframe, next_keyframe_path))
      {
        next_success = true;
      }
      else
      {
        int next_keyframe_number = 0; // Wraps around to the first frame
        while(next_success == false)
        {
          next_success = loadKeyframe(next_keyframe, keyframe_path + formKeyframeName(next_keyframe_number, 4));
          next_keyframe_number++;
        }
      }
    }
  }

*/

  return current_success;
}

void KeyFramesReshaper::deleteNaNs(const PointCloudT::ConstPtr& cloud_in, PointCloudT::Ptr& cloud_out) const
{
  cloud_out->header = cloud_in->header;

  uint points_in_size = cloud_in->points.size();
  for (uint i=0; i<points_in_size; ++i)
  {
    // convert from pcl PointT to Eigen Vector3f
    PointT point = cloud_in->points[i];
    if (isnan(point.x) || isnan(point.y) || isnan(point.z) )
      continue;
    else
      cloud_out->points.push_back(point);
  }
  uint points_out_size = cloud_out->points.size();

  printf("NaN filtering: BEFORE = %d points \t AFTER = %d points \n", points_in_size, points_out_size);

}

void KeyFramesReshaper::filterCloud(const PointCloudT::ConstPtr& cloud_in, PointCloudT::Ptr& cloud_out, double vgf_res, double neighbor_max_proximity, double smoothing_res) const
{

  // double mesh_search_radius = atof(argv[5]); TODO: if meshing here
  uint points_in_size = cloud_in->points.size();
  printf("Before FILTER: %d points \n", points_in_size);

  bool with_smoothing = false;
  if(smoothing_res > 0.0)
    with_smoothing = true;

  // Load input file into a PointCloud<T> with an appropriate type
  PointCloudT::Ptr cloud_filtered (new PointCloudT);

  // Create the filtering object
  pcl::VoxelGrid<PointT> sor1;
  sor1.setInputCloud (cloud_in);
  sor1.setLeafSize (vgf_res, vgf_res, vgf_res);
  sor1.filter (*cloud_filtered);

  uint number_of_points = cloud_filtered->points.size();
  printf("VOXEL GRID FILTER: %d points \n", number_of_points);
  std::vector<bool> valid_indices(number_of_points, true);

  PointT searchPoint;
  // ... populate the cloud and the search point
  // create a kd-tree instance
  pcl::KdTreeFLANN<PointT> kdtree_naive;
  // assign a point cloud_in - this builds the tree
  kdtree_naive.setInputCloud (cloud_filtered);
  std::vector<int> pointIdxRadius;
  std::vector<float> pointsSquaredDistRadius;
  float radius = neighbor_max_proximity;
  // radius search

  for(int i = 0; i<number_of_points; i++)
  {
    if(valid_indices[i])
    {
      pointIdxRadius.clear();
      pointsSquaredDistRadius.clear();
      searchPoint = cloud_filtered->points[i];
      int count = kdtree_naive.radiusSearch (searchPoint, radius,
                                             pointIdxRadius, pointsSquaredDistRadius);

      for(int c = 1; c<pointIdxRadius.size(); c++)
      {
        valid_indices[pointIdxRadius[c]] = false;
      }

    }
  }

  int count_valid_indices = 0;
  for(int i = 0; i<number_of_points; i++)
  {
    if(valid_indices[i])
      count_valid_indices++;
  }

  printf("After max neighbors FILTER: %d points using knn radius search: %f \n", count_valid_indices, radius);

  // Form the new tree
  cloud_out->points.clear();
  cloud_out->points.resize(count_valid_indices);
  int pt = 0;
  for(int i = 0; i<number_of_points; i++)
  {
    if(valid_indices[i])
    {
      PointT valid_point = cloud_filtered->points[i];
      cloud_out->points[pt] = valid_point;
      pt++;
    }
  }

  printf("done!\n");

  if(with_smoothing)
  {
    // smooth using mls
    printf("Creating kd-tree for smoothing\n");
    pcl::search::KdTree<PointT>::Ptr mls_tree;
    mls_tree.reset(new pcl::search::KdTree<PointT>());

    printf("MLS...\n");
    pcl::MovingLeastSquares<PointT, PointNormalT> mls;
    mls.setInputCloud (cloud_out);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (mls_tree);
    mls.setSearchRadius (smoothing_res);
    mls.reconstruct(*cloud_out);
    // the dense data should be available in cloud_for_mesh
    printf("done!\n");

  }

  /*
  pcl::PCDWriter writer;
  std::string downsampled_pcd_filename;
  if(with_smoothing)
    downsampled_pcd_filename = std::string(argv[1]) + "_downsampled_smoothed.pcd";
  else
    downsampled_pcd_filename = std::string(argv[1]) + "_downsampled.pcd";


  int result_pcd = writer.writeBinary<PointT>(downsampled_pcd_filename, *cloud_for_mesh);
  */

  /*
// Normal estimation*
pcl::NormalEstimation<PointT, pcl::Normal> n;
pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
tree->setInputCloud (cloud_for_mesh);
n.setInputCloud (cloud_for_mesh);
n.setSearchMethod (tree);
n.setKSearch (20);
printf("Computing Normals...\n");
n.compute (*normals);
//* normals should not contain the point normals + surface curvatures
printf("done!\n");

// Concatenate the XYZ and normal fields*
pcl::PointCloud<PointNormalT>::Ptr cloud_with_normals (new pcl::PointCloud<PointNormalT>);
pcl::concatenateFields (*cloud_for_mesh, *normals, *cloud_with_normals);
// cloud_with_normals = cloud + normals

// Create search tree*
pcl::search::KdTree<PointNormalT>::Ptr tree2 (new pcl::search::KdTree<PointNormalT>);
tree2->setInputCloud (cloud_with_normals);

// Initialize objects
printf("Greedy Triangulation\n");

pcl::GreedyProjectionTriangulation<PointNormalT> gp3;
pcl::PolygonMesh triangles;

// Set the maximum distance between connected points (maximum edge length)
gp3.setSearchRadius (mesh_search_radius);

// Set typical values for the parameters
//  gp3.setMu (2.5);
gp3.setMu (10);
gp3.setMaximumNearestNeighbors (50);
gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
gp3.setMinimumAngle(M_PI/18); // 10 degrees
gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
gp3.setNormalConsistency(false);

// Get result
gp3.setInputCloud (cloud_with_normals);
gp3.setSearchMethod (tree2);
printf("Reconstructing mesh...\n");
gp3.reconstruct (triangles);

// Additional vertex information
std::vector<int> parts = gp3.getPartIDs();
std::vector<int> states = gp3.getPointStates();

std::string mesh_filename;
if(with_smoothing)
  mesh_filename = std::string(argv[1]) + "_mesh_smoothed";
else
  mesh_filename = std::string(argv[1]) + "_mesh";

pcl::io::saveVTKFile (mesh_filename+".vtk", triangles);
pcl::io::savePLYFile(mesh_filename+".ply", triangles);
   */


}

void KeyFramesReshaper::initParams()
{
  // PCD File
  if(!nh_private_.getParam("apps/keyframes_reshaper/output_filename", output_filename_))
    output_filename_ = "cloud_reshaped";

  if (!nh_private_.getParam ("apps/keyframes_reshaper/path_to_keyframes", path_to_keyframes_))
    path_to_keyframes_ = "~/ros/Keyframes/";
  else
    {
    char last_char_in_path = path_to_keyframes_.at( path_to_keyframes_.length() - 1 );
    if(last_char_in_path != '/')
      path_to_keyframes_ = path_to_keyframes_ + "/";
    }
  ROS_INFO("Path to KeyFrames: %s", path_to_keyframes_.c_str());

  if (!nh_private_.getParam ("apps/keyframes_reshaper/first_keyframe_number", first_keyframe_number_))
    first_keyframe_number_ = 0;
  if (!nh_private_.getParam ("apps/keyframes_reshaper/last_keyframe_number", last_keyframe_number_))
    last_keyframe_number_ = first_keyframe_number_;

  if (!nh_private_.getParam ("apps/keyframes_reshaper/vgf_res", vgf_res_))
    vgf_res_ = 0.001;
  if (!nh_private_.getParam ("apps/keyframes_reshaper/neighbor_max_proximity", neighbor_max_proximity_))
    neighbor_max_proximity_ = 0.002;
  if (!nh_private_.getParam ("apps/keyframes_reshaper/smoothing_res", smoothing_res_))
    smoothing_res_ = 0.0;

  ROS_INFO("Parameters initialized.");

}

bool KeyFramesReshaper::readPointCloudFromPCDFile()
{
//  cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  model_ptr_ = PointCloudT::Ptr(new PointCloudT);

  std::string cloud_in_name = path_to_keyframes_+"/cloud.pcd"; // FIXME: it shouldn't be a static name!
  if (pcl::io::loadPCDFile<PointT> (cloud_in_name, *model_ptr_) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return false;
  }
  model_ptr_->header.frame_id = fixed_frame_;
  model_ptr_->header.stamp = ros::Time::now();

  std::cout << "Loaded "
      << model_ptr_->width * model_ptr_->height
      << " data points from " << cloud_in_name << " with header = " <<   model_ptr_->header.frame_id
      << std::endl;

  return true;
}



} //namespace ccny_rgbd
