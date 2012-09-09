template <typename PointData, typename PointModel>
ccny_rgbd::ICPKd<PointData, PointModel>::ICPKd():
  max_iterations_(25),
  corr_dist_threshold_(0.15),
  transformation_epsilon_(1e-9),
  min_correspondences_(10),
  ransac_inlier_threshold_(0.05),
  use_ransac_rejection_(true),
  use_value_rejection_(false),
  max_value_diff_(1.0)
{

}

template <typename PointData, typename PointModel>
ccny_rgbd::ICPKd<PointData, PointModel>::~ICPKd()
{


}

template <typename PointData, typename PointModel>
inline void ccny_rgbd::ICPKd<PointData, PointModel>::setDataCloud(PointCloudData  * cloud_data)
{
  cloud_data_ = cloud_data;
}

template <typename PointData, typename PointModel>
inline void ccny_rgbd::ICPKd<PointData, PointModel>::setModelCloud(PointCloudModel * cloud_model)
{
  cloud_model_ = cloud_model;
}

template <typename PointData, typename PointModel>
inline void ccny_rgbd::ICPKd<PointData, PointModel>::setDataTree(KdTreeData  * tree_data)
{
  tree_data_ = tree_data;
}

template <typename PointData, typename PointModel>
inline void ccny_rgbd::ICPKd<PointData, PointModel>::setModelTree(KdTreeModel * tree_model)
{
  tree_model_ = tree_model;
}

template <typename PointData, typename PointModel>
bool ccny_rgbd::ICPKd<PointData, PointModel>::align()
{
  // Allocate enough space to hold the results
  // for nearestK, use nr_neighbors_

  IntVector   nn_indices  (1); // for nn search
  FloatVector nn_dists_sq (1); // for nn search

  // reset state
  iterations_ = 0;
  transformation_.setIdentity();
  final_transformation_.setIdentity();

  // set initial variables
  int N = cloud_data_->points.size();    // the size of the data cloud
  bool converged = false;
  float dist_threshold_sq = corr_dist_threshold_ * corr_dist_threshold_;

  while (!converged)           // repeat until convergence
  {
    //printf("start iteration %d\n", iterations_);

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
        //float dv = diff ((input_->points[idx], target_->points[nn_indices[i]]));

        source_indices[corr_cnt] = idx;
        target_indices[corr_cnt] = nn_indices[0];
        corr_cnt++;
      }
    }

    // Resize to the actual number of valid correspondences
    source_indices.resize(corr_cnt);
    target_indices.resize(corr_cnt);

    // rejection

    if (use_value_rejection_)
      rejectionValue(source_indices, target_indices);

    if (use_ransac_rejection_)
      rejectionRansac(source_indices, target_indices);

    // Check whether we have enough correspondences
    corr_cnt = (int)source_indices.size();
    if (corr_cnt < min_correspondences_)
    {
      ROS_WARN ("[ICPKd] Not enough correspondences. Leaving ICP loop");
      converged = false;
      break;
    }

    pcl::registration::TransformationEstimationSVD<PointData, PointModel> svd;

    // **** Estimate the transform

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
    if (iterations_ >= max_iterations_)
    {
      converged = true;
    }

    float delta = (transformation_ - previous_transformation_).sum();
    if (fabs(delta) < transformation_epsilon_)
    {
      converged = true;
    }
  }

  return converged;
  //printf("Finished in %d\n", iterations_);
}

template <typename PointData, typename PointModel>
void ccny_rgbd::ICPKd<PointData, PointModel>::rejectionRansac(
    IntVector& source_indices,
    IntVector& target_indices)
{
  //int N = source_indices.size();

  IntVector source_indices_good;
  IntVector target_indices_good;

  SampleConsensusModelRegistrationPtr sac_model;

  PointCloudData  source = *cloud_data_;
  PointCloudModel target = *cloud_model_;

  sac_model.reset (new pcl::SampleConsensusModelRegistration<PointData> (source.makeShared(), source_indices));

  // Pass the target_indices

  sac_model->setInputTarget (target.makeShared(), target_indices);

  // Create a RANSAC model
  pcl::RandomSampleConsensus<PointData> sac (sac_model, ransac_inlier_threshold_);
  sac.setMaxIterations (1000);

  bool result = sac.computeModel();

  // Compute the set of inliers

  if (result)
  {
    IntVector inliers;
    sac.getInliers (inliers);

    //printf("\tRANSAC: %d of %d rejected\n", N - inliers.size(), N);

    if ((int)inliers.size() < min_correspondences_)
    {
      ROS_WARN ("[ICPKd] Not enough correspondences. Dropping RANSAC rejection");
      return;
    }

    source_indices_good.resize (inliers.size ());
    target_indices_good.resize (inliers.size ());

    boost::unordered_map<int, int> source_to_target;
    for (unsigned int i = 0; i < source_indices.size(); ++i)
      source_to_target[source_indices[i]] = target_indices[i];

    // Copy just the inliers
    std::copy(inliers.begin(), inliers.end(), source_indices_good.begin());
    for (size_t i = 0; i < inliers.size (); ++i)
      target_indices_good[i] = source_to_target[inliers[i]];

    source_indices = source_indices_good;
    target_indices = target_indices_good;
  }
  else
  {
    //printf("NO MODEL\n");
  }
}

template <typename PointData, typename PointModel>
void ccny_rgbd::ICPKd<PointData, PointModel>::rejectionValue(
    IntVector& source_indices,
    IntVector& target_indices)
{
  // FIXME - add distance functions
  /*
  IntVector source_indices_good;
  IntVector target_indices_good;

  int N = source_indices.size();

  for (int i = 0; i < N; ++i)
  {
    PointData  * d = &cloud_data_->points [source_indices[i]];
    PointModel * m = &cloud_model_->points[target_indices[i]];

    float dv = dist (*d, *m);

    if (dv < max_value_diff_)
    {
      source_indices_good.push_back(source_indices[i]);
      target_indices_good.push_back(target_indices[i]);
    }
  }

  //printf("\tValue: %d of %d rejected\n", N - source_indices_good.size(), N);


  if ((int)source_indices_good.size() > min_correspondences_)
  {
    source_indices = source_indices_good;
    target_indices = target_indices_good;
  }
  else
  {
    //FIXME
    ROS_WARN ("[ICPKd] Not enough correspondences. Dropping VALUE rejection");
    //source_indices = source_indices_good;
    //target_indices = target_indices_good;
  }
  */
}
