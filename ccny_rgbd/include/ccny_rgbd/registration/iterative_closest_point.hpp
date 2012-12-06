template <typename PointData, typename PointModel>
ccny_rgbd::IterativeClosestPoint<PointData, PointModel>::IterativeClosestPoint():
  max_iterations_(25),
  corr_dist_threshold_(0.15),
  transformation_epsilon_(1e-9),
  min_correspondences_(10)
{

}

template <typename PointData, typename PointModel>
ccny_rgbd::IterativeClosestPoint<PointData, PointModel>::~IterativeClosestPoint()
{


}

template <typename PointData, typename PointModel>
inline void ccny_rgbd::IterativeClosestPoint<PointData, PointModel>::setDataCloud(PointCloudDataPtr cloud_data)
{
  cloud_data_ = cloud_data;
}

template <typename PointData, typename PointModel>
inline void ccny_rgbd::IterativeClosestPoint<PointData, PointModel>::setModelCloud(PointCloudModelPtr cloud_model)
{
  cloud_model_ = cloud_model;
}

template <typename PointData, typename PointModel>
inline void ccny_rgbd::IterativeClosestPoint<PointData, PointModel>::setModelTree(KdTreePtr tree_model)
{
  tree_model_ = tree_model;
}

template <typename PointData, typename PointModel>
bool ccny_rgbd::IterativeClosestPoint<PointData, PointModel>::align()
{
  // Allocate enough space to hold the nn results
  IntVector   nn_indices  (1); 
  FloatVector nn_dists_sq (1);

  // reset state
  iterations_ = 0;
  transformation_.setIdentity();
  final_transformation_.setIdentity();

  // set initial variables
  int N = cloud_data_->points.size();    // the size of the data cloud
  bool converged = false;
  float dist_threshold_sq = corr_dist_threshold_ * corr_dist_threshold_;

  while (!converged)
  {
    int corr_cnt = 0;

    IntVector source_indices(N);
    IntVector target_indices(N);

    // Iterating over the entire index vector and  find all correspondences
    for (int idx = 0; idx < N; ++idx)
    {
      // **** search for nearest neighbors

      PointData p = cloud_data_->points[idx];

      int nn_retrieved = tree_model_->nearestKSearch(
        p, 1, nn_indices, nn_dists_sq);

      if (nn_retrieved != 0 && nn_dists_sq[0] < dist_threshold_sq)
      {
        source_indices[corr_cnt] = idx;
        target_indices[corr_cnt] = nn_indices[0];
        corr_cnt++;
      }
    }

    // Resize to the actual number of valid correspondences
    source_indices.resize(corr_cnt);
    target_indices.resize(corr_cnt);

    // Check whether we have enough correspondences
    corr_cnt = (int)source_indices.size();
    if (corr_cnt < min_correspondences_)
    {
      ROS_WARN ("[ICP] Not enough correspondences. Leaving ICP loop");
      converged = false;
      break;
    }

    pcl::registration::TransformationEstimationSVD<PointData, PointModel> svd;

    // Save the previously estimated transformation
    previous_transformation_ = transformation_;

    svd.estimateRigidTransformation (*cloud_data_, source_indices,
                                     *cloud_model_, target_indices,
                                     transformation_);

    // Obtain the final transformation
    final_transformation_ = transformation_ * final_transformation_;

    // Tranform the data
    transformPointCloud (*cloud_data_, *cloud_data_, transformation_);

    iterations_++;

    // Check for convergence
    if (iterations_ >= max_iterations_) converged = true;

    float delta = (transformation_ - previous_transformation_).sum();
    if (fabs(delta) < transformation_epsilon_) converged = true;
  }

  return converged;
}


