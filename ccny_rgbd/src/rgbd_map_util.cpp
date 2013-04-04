#include "ccny_rgbd/rgbd_map_util.h"

namespace ccny_rgbd {
 
void buildSURFAssociationMatrixBruteForce(
  const KeyframeVector& keyframes,
  cv::Mat& association_matrix,
  int threshold)
{  
  // create a candidate matrix which considers all posiible combinations
  int size = keyframes.size();
  cv::Mat candidate_matrix = cv::Mat::ones(size, size, CV_8UC1);
  
  // perfrom pairwise matching for all candidates
  cv::Mat correspondence_matrix;
  buildRANSACCorrespondenceMatrix(keyframes, candidate_matrix, correspondence_matrix);
  
  // threshold
  thresholdMatrix(correspondence_matrix, association_matrix, threshold);
}
 
void buildSURFAssociationMatrixTree(
  const KeyframeVector& keyframes,
  cv::Mat& association_matrix,
  int k_nearest_neighbors,
  int n_candidates,
  int threshold)
{
  // build a math knn matrix using a kdtree
  cv::Mat match_matrix;
  buildSURFMatchMatrixTree(keyframes, match_matrix, k_nearest_neighbors);
   
  // keep only the top n candidates
  cv::Mat candidate_matrix;
  buildSURFCandidateMatrixTree(match_matrix, candidate_matrix, k_nearest_neighbors);
  cv::namedWindow("Candidate matrix", 0);
  cv::imshow("Candidate matrix", candidate_matrix*255);
  
  // perfrom pairwise matching for all candidates
  cv::Mat correspondence_matrix;
  buildRANSACCorrespondenceMatrix(keyframes, candidate_matrix, correspondence_matrix); 
  
  // threshold
  thresholdMatrix(correspondence_matrix, association_matrix, threshold);
}
 
void trainSURFMatcher(
  const KeyframeVector& keyframes,
  cv::FlannBasedMatcher& matcher)
{
  printf("Building aggregate feature vector...\n"); 
  std::vector<cv::Mat> descriptors_vector;
  
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
  {
    const RGBDKeyframe& keyframe = keyframes[kf_idx];
    descriptors_vector.push_back(keyframe.descriptors);
  }
  matcher.add(descriptors_vector);

  printf("Training feature matcher...\n");
  matcher.train();
}
 
void buildRANSACCorrespondenceMatrix(
  const KeyframeVector& keyframes, 
  const cv::Mat& candidate_matrix,
  cv::Mat& correspondence_matrix)
{
  // check for square matrix
  assert(candidate_matrix.rows == candidate_matrix.cols);
  int size = candidate_matrix.rows;    
  
  // params
  float max_corresp_dist_eucl = 0.03;
  float max_corresp_dist_desc = 0.5;
  float sufficient_ransac_inlier_ratio = 1.0;
  int max_ransac_iterations = 1000;
  
  bool save_ransac_results = true; 
  std::string ransac_results_path = "/home/idryanov/ros/images/ransac";  
    
  // derived params
  float max_corresp_dist_eucl_sq = max_corresp_dist_eucl * max_corresp_dist_eucl;
    
  correspondence_matrix = cv::Mat::zeros(size, size, CV_32FC1);
  
  for (int kf_idx_a = 0; kf_idx_a < size; ++kf_idx_a)
  {
    const RGBDKeyframe& keyframe_a = keyframes[kf_idx_a];
    
    // self-association
    // @todo actually this should only account for the "valid" keypoints
    correspondence_matrix.at<float>(kf_idx_a, kf_idx_a) = keyframe_a.keypoints.size();
    
    for (int kf_idx_b = kf_idx_a+1; kf_idx_b < size; ++kf_idx_b)
    {
      // skip non-candidates
      if (candidate_matrix.at<uint8_t>(kf_idx_b, kf_idx_a) != 0 ||
          candidate_matrix.at<uint8_t>(kf_idx_a, kf_idx_b) != 0)
      {
      
        printf("[%d %d]\n", kf_idx_a, kf_idx_b);

        const RGBDKeyframe& keyframe_b = keyframes[kf_idx_b];

        std::vector<cv::DMatch> all_matches;
        std::vector<cv::DMatch> inlier_matches;

        // perform ransac matching, b onto a
        Eigen::Matrix4f transformation;

        pairwiseMatchingRANSAC(keyframe_a, keyframe_b, 
          max_corresp_dist_eucl_sq, max_corresp_dist_desc, 
          sufficient_ransac_inlier_ratio, max_ransac_iterations,
          all_matches, inlier_matches, transformation);
        
        if (save_ransac_results && inlier_matches.size() > 30)
        {
          cv::Mat img_matches;
          cv::drawMatches(keyframe_b.rgb_img, keyframe_b.keypoints, 
                          keyframe_a.rgb_img, keyframe_a.keypoints, 
                          inlier_matches, img_matches);

          std::stringstream ss1;
          ss1 << kf_idx_a << "_to_" << kf_idx_b;
          cv::imwrite(ransac_results_path + "/" + ss1.str() + ".png", img_matches);
        }
      
        // both entries in matrix
        correspondence_matrix.at<float>(kf_idx_a, kf_idx_b) = inlier_matches.size();
        correspondence_matrix.at<float>(kf_idx_b, kf_idx_a) = inlier_matches.size();
      }
    }
  }
}  
  
/** @brief Takes in a matrix of matches from a SURF tree, and marks the top
 * n candidates in each row.
 */  
void buildSURFCandidateMatrixTree(
  const cv::Mat& match_matrix,
  cv::Mat& candidate_matrix,
  int n_candidates)
{
  // check for square matrix
  assert(match_matrix.rows == match_matrix.cols);
  
  // check for validity of n_candidates argument
  int size = match_matrix.rows;
  assert(n_candidates <= size);
  
  // initialize candidate matrix as all 0
  candidate_matrix = cv::Mat::zeros(match_matrix.size(), CV_8UC1);
  
  for (int v = 0; v < match_matrix.rows; ++v)
  {
    // create a vector from the current row
    std::vector<std::pair<int, int> > values(match_matrix.cols);
    for (int u = 0; u < match_matrix.cols; ++u)
    {
      int value = match_matrix.at<float>(v,u);
      values[u] =  std::pair<int, int>(value, u);
    }
    
    // sort the vector based on values, highest first
    std::sort(values.begin(), values.end(), std::greater<std::pair<int, int> >());

    // mark 1 for the top n_candidates
    for (int u = 0; u < n_candidates; ++u)
    {
      unsigned int uc = values[u].second;     
      candidate_matrix.at<uint8_t>(v,uc) = 1;
    }
  }
}
  
 /** @brief Builds a matrix of nearest neighbor matches between keyframes 
  * using a kdtree
  * 
  * match_matrix[query,train] = X correspondences
  */  
void buildSURFMatchMatrixTree(
  const KeyframeVector& keyframes,
  cv::Mat& match_matrix,
  int k_nearest_neighbors)
{
  // extra params
  //float nn_radius_surf = 0.5;
    
  unsigned int kf_size = keyframes.size(); 
  
  // train matcher from all the features
  cv::FlannBasedMatcher matcher;
  trainSURFMatcher(keyframes, matcher);

  // lookup per frame
  printf("Keyframe lookups...\n");

  match_matrix = cv::Mat::zeros(kf_size, kf_size, CV_32FC1);
  for (unsigned int kf_idx = 0; kf_idx < kf_size; ++kf_idx)
  {
    printf("[KF %d of %d]:\n", (int)kf_idx, (int)kf_size);
    const RGBDFrame& keyframe = keyframes[kf_idx];

    // find k nearest matches for each feature in the keyframe
    std::vector<std::vector<cv::DMatch> > matches_vector;
    matcher.knnMatch(keyframe.descriptors, matches_vector, k_nearest_neighbors);
    //matcher.radiusMatch(keyframe.descriptors, matches_vector, nn_radius_surf);

    // create empty bins vector of Pairs <count, image_index>
    std::vector<std::pair<int, int> > bins;
    bins.resize(keyframes.size());
    for (unsigned int b = 0; b < bins.size(); ++b) 
      bins[b] = std::pair<int, int>(0, b);

    // fill out bins with match indices
    for (unsigned int j = 0; j < matches_vector.size(); ++j)
    {
      std::vector<cv::DMatch>& matches = matches_vector[j];
      for (unsigned int k = 0; k < matches.size(); ++k)
      {
        bins[matches[k].imgIdx].first++;
      }
    }

    /*
    // sort - highest counts first
    std::sort(bins.begin(), bins.end(), std::greater<std::pair<int, int> >());

    // output results
    printf(" - best matches: ");
    for (int b = 0; b < bins.size(); ++b)
      printf("[%d(%d)] ", bins[b].second, bins[b].first);
    printf("\n");  
    */
    
    for (unsigned int b = 0; b < kf_size; ++b)
    {
      unsigned int index_a = kf_idx;
      unsigned int index_b = bins[b].second;
      int corresp_count = bins[b].first;
      
      if (index_a != index_b)
        match_matrix.at<float>(index_a, index_b) = corresp_count;
    }
  }
}
  
void buildDenseAssociationMatrix(
  const KeyframeVector& keyframes,
  cv::Mat& association_matrix)
{
  // params
  float nn_radius = 0.15;
  float vgf_res = 0.05;
  
  unsigned int kf_size = keyframes.size(); 
   
  // aggregate global cloud, and a vecto of kf indices
  printf("Aggregating keyframes\n");
  
  PointCloudT::Ptr aggregate_cloud;
  aggregate_cloud.reset(new PointCloudT());
  IntVector aggregate_indices;
  
  pcl::VoxelGrid<PointT> vgf;
  vgf.setLeafSize(vgf_res, vgf_res, vgf_res);
      
  for (unsigned int kf_idx = 0; kf_idx < kf_size; ++kf_idx)
  {
    const RGBDKeyframe& keyframe = keyframes[kf_idx];
    
    // prepare keyframe cloud
    PointCloudT::Ptr kf_cloud;
    kf_cloud.reset(new PointCloudT());
    keyframe.constructDensePointCloud(*kf_cloud);
      
    // filter the keyframe cloud
    PointCloudT kf_cloud_f;
    vgf.setInputCloud(kf_cloud);
    vgf.filter(kf_cloud_f);
    
    // rotate cloud
    PointCloudT kf_cloud_tf;
    pcl::transformPointCloud(kf_cloud_f, kf_cloud_tf, eigenFromTf(keyframe.pose));
        
    for (unsigned int pt_idx = 0; pt_idx < kf_cloud_tf.points.size(); ++pt_idx)
    {
      const PointT& p = kf_cloud_tf.points[pt_idx];     
      aggregate_cloud->push_back(p);
      aggregate_indices.push_back((int)kf_idx);
    }
  }
  
  // create kd tree
  printf("Creating kd-tree\n");
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(aggregate_cloud);

  // perform the search
  std::vector<int> nn_indices;
  std::vector<float> nn_distances_sq;

  int total_size = aggregate_cloud->points.size();
  association_matrix = cv::Mat::zeros(kf_size, kf_size, CV_32FC1);
  for (int pt_idx = 0; pt_idx < total_size; ++pt_idx)
  {
    if (pt_idx%1000 == 0) 
      printf("pt_idx: %dK of %dK\n", pt_idx/1000, total_size/1000);
    
    int kf_idx = aggregate_indices[pt_idx];
    
    const PointT& p = aggregate_cloud->points[pt_idx];
    
    int nn_results = kdtree.radiusSearch(
      p, nn_radius, nn_indices, nn_distances_sq);   
    
    for (int nn_idx = 0; nn_idx < nn_results; ++nn_idx)
    {
      // index of the point which is the nearest neighbor
      int nn_pt_idx = nn_indices[nn_idx];
      
      // index of the keyframe to which the NN point belongs to
      int nn_kf_idx = aggregate_indices[nn_pt_idx];
           
      association_matrix.at<float>(kf_idx, nn_kf_idx) += 1.0;
    }
  } 
  printf("Done\n");
}

void floatMatrixToUintMatrix(
  const cv::Mat& mat_in, 
  cv::Mat& mat_out, 
  float scale)
{
  if (scale == 0)
  {
    // normalize the matrix
    float max_val = 0;
    
    for (int u = 0; u < mat_in.cols; ++u)
    for (int v = 0; v < mat_in.rows; ++v)   
    {
      float val_in = mat_in.at<float>(v, u);
      if (val_in > max_val) max_val = val_in;
    }
  
    scale = 255.0 / max_val;
  }
   
  mat_out = cv::Mat::zeros(mat_in.size(), CV_8UC1);
  for (int u = 0; u < mat_in.cols; ++u)
  for (int v = 0; v < mat_in.rows; ++v)   
  {
    float val_in = mat_in.at<float>(v, u) ;
    uint8_t& val_out = mat_out.at<uint8_t>(v, u); 

    val_out = val_in * scale;
  }
}

void thresholdMatrix(
  const cv::Mat& mat_in, 
  cv::Mat& mat_out,
  float threshold)
{
  mat_out = cv::Mat(mat_in.size(), CV_8UC1);
  cv::threshold(mat_in, mat_out, threshold, 1, cv::THRESH_BINARY);   
}

void alignGlobalMap(
  const PointCloudT::Ptr& cloud)
{
   double vgf_res = 0.01;

  // filter cloud
  printf("Filtering cloud\n");
  PointCloudT::Ptr cloud_f;
  cloud_f.reset(new PointCloudT());
  pcl::VoxelGrid<PointT> vgf;
  vgf.setInputCloud(cloud);
  vgf.setLeafSize(vgf_res, vgf_res, vgf_res);
  vgf.filter(*cloud_f);

  // rotate 45 deg
  tf::Transform t;
  t.setOrigin(tf::Vector3(0,0,0));
  tf::Quaternion q;
  q.setRPY(0, 0, M_PI/6.0);
  t.setRotation(q);
  pcl::transformPointCloud(*cloud_f, *cloud_f, eigenFromTf(t)); 

  // show map
  printf("Showing map\n");
  cv::Mat map_r;
  create2DProjectionImage(*cloud_f, map_r);
  cv::imshow("map_r", map_r);
  cv::waitKey(0);

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<PointT, pcl::PointXYZRGBNormal> ne;
  ne.setInputCloud(cloud_f);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  printf("Creating kd-tree\n");
  pcl::search::KdTree<PointT>::Ptr tree;
  tree.reset(new pcl::search::KdTree<PointT>());
  ne.setSearchMethod(tree);

  // Output datasets
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals;
  cloud_normals.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  // Use all neighbors in a sphere of radius x cm
  ne.setRadiusSearch(0.05);
  
  // Compute the features
  printf("Estimating normals\n");
  ne.compute (*cloud_normals);

  /*
  for (unsigned int i = 0; i < cloud_f->points.size(); ++i)
  {
    const PointT& p_in = cloud_f->points[i]; 
    pcl::PointXYZRGBNormal& p_out = cloud_normals->points[i]; 

    p_out.x   = p_in.x;
    p_out.y   = p_in.y;
    p_out.z   = p_in.z;
    p_out.rgb = p_in.rgb;
  }

  // write out  
  printf("Writing out\n");
  pcl::PCDWriter writer;
  writer.write ("/home/idryanov/cloud_00_n.pcd", *cloud_normals);
*/
  
  // create expected histogram
  double hist_resolution = 0.25;
  cv::Mat hist_exp;
  buildExpectedPhiHistorgtam(hist_exp, hist_resolution, 5.0); 
  cv::Mat hist_exp_img;
  createImageFromHistogram(hist_exp, hist_exp_img);
  cv::imshow("hist_exp_img", hist_exp_img);

  // create histogram
  printf("creating histogram\n");
  cv::Mat hist;
  buildPhiHistogram(*cloud_normals, hist, hist_resolution);

  // show histogram
  printf("showing histogram\n");
  cv::Mat hist_img;
  createImageFromHistogram(hist, hist_img);
  cv::imshow("Histogram Phi", hist_img);
  cv::waitKey(0);

  // find alignement
  double best_angle;
  alignHistogram(hist, hist_exp, hist_resolution, best_angle);
  printf("best_angle: %f\n", best_angle);

  // show best aligned histogram
  cv::Mat hist_best;
  shiftHistogram(hist, hist_best, best_angle/hist_resolution);
  cv::Mat hist_best_img;
  createImageFromHistogram(hist_best, hist_best_img);
  cv::imshow("hist_best_img", hist_best_img);
  cv::waitKey(0);

  // derotate
  tf::Transform t1;
  t1.setOrigin(tf::Vector3(0,0,0));
  tf::Quaternion q1;
  q1.setRPY(0, 0, best_angle * M_PI/180.0);
  t1.setRotation(q1);
  pcl::transformPointCloud(*cloud_f, *cloud_f, eigenFromTf(t1)); 

  // show map
  cv::Mat map_f;
  create2DProjectionImage(*cloud_f, map_f);
  cv::imshow("map_f", map_f);
  cv::waitKey(0);

  printf("Done\n");
}

void buildExpectedPhiHistorgtam(
  cv::Mat& histogram,
  double degrees_per_bin,
  double stdev)
{
  int n_bins = (int)(360.0 / degrees_per_bin);
  histogram = cv::Mat::zeros(1, n_bins, CV_32FC1);

  double s = stdev  / degrees_per_bin;

  double mean[4];
  mean[0] =   0.0 / degrees_per_bin;
  mean[1] =  90.0 / degrees_per_bin;
  mean[2] = 180.0 / degrees_per_bin;
  mean[3] = 270.0 / degrees_per_bin;

  double a = 1.0 / (s * sqrt(2.0 * M_PI));
  double b = 2.0 * s * s; 

  for (int u = 0; u < n_bins; ++u)
  {
    float& bin = histogram.at<float>(0, u);

    // accumulate 4 gaussians
    for (int g = 0; g < 4; g++)
    {
      int x = u - mean[g];
  
      // wrap around to closer distance
      if (x < -n_bins/2) x += n_bins;
      if (x >= n_bins/2) x -= n_bins;

      float r = a * exp(-x*x / b);

      bin += r;
    }
  }

  normalizeHistogram(histogram);
}  

void buildPhiHistogram(
  const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud,
  cv::Mat& histogram,
  double degrees_per_bin)
{
  int phi_bins = (int)(360.0 / degrees_per_bin);
  histogram = cv::Mat::zeros(1, phi_bins, CV_32FC1);
  
  for (unsigned int i = 0; i < cloud.points.size(); ++i)
  {
    const pcl::PointXYZRGBNormal& p = cloud.points[i]; 

    double nx = p.normal_x;
    double ny = p.normal_y;
    double nz = p.normal_z;

    if (isnan(nx) || isnan(ny) || isnan(nz)) continue;

    double r = sqrt(nx*nx + ny*ny + nz*nz);
    double theta = acos(nz/r);
    double phi   = atan2(ny, nx);

    double phi_deg   = phi   * 180.0 / M_PI;
    double theta_deg = theta * 180.0 / M_PI; 

    // normalize phi to [0, 360)
    if (phi_deg < 0.0) phi_deg += 360.0;

    // only consider points which are close to vertical
    if (std::abs(90-theta_deg) > 3.0) continue; 

    int idx_phi = (int)(phi_deg / degrees_per_bin);

    float& bin = histogram.at<float>(0, idx_phi);
    bin = bin + 1.0; 
  }

  normalizeHistogram(histogram);
}

void shiftHistogram(
  const cv::Mat& hist_in,
  cv::Mat& hist_out,
  int bins)
{
  hist_out = cv::Mat(hist_in.size(), CV_32FC1);
  int w = hist_in.cols;
  for (int u = 0; u < w; ++u)
  {
    int u_shifted = (u + bins) % w;

    hist_out.at<float>(0, u_shifted) = hist_in.at<float>(0, u);
  } 
}

void normalizeHistogram(cv::Mat& histogram)
{
  float sum = 0;

  for (int u = 0; u < histogram.cols; ++u)
    sum += histogram.at<float>(0,u);

  histogram = histogram / sum;
}

bool alignHistogram(
  const cv::Mat& hist,
  const cv::Mat& hist_exp,
  double hist_resolution,
  double& best_angle)
{
  // check diff
  int best_i = 0;
  double best_diff = 9999999999;

  for (int i = 0; i < 90.0 / hist_resolution; ++i)
  {
    cv::Mat hist_shifted;
    shiftHistogram(hist, hist_shifted, i);

    double diff = cv::compareHist(hist_shifted, hist_exp, CV_COMP_BHATTACHARYYA);
    if (std::abs(diff) < best_diff)
    {
      best_diff = std::abs(diff);
      best_i = i;
    }
  }

  best_angle = best_i * hist_resolution;

  cv::Mat hist_best;
  shiftHistogram(hist, hist_best, best_i);
  cv::Mat hist_best_img;
  createImageFromHistogram(hist_best, hist_best_img);
  cv::imshow("hist_best_img", hist_best_img);

  return true;
}

void create8bitHistogram(
  const cv::Mat& histogram,
  cv::Mat& histogram_norm)
{
  // find max value
  double min_val, max_val;
  cv::Point minLoc, maxLoc;
  cv::minMaxLoc(histogram, &min_val, &max_val, &minLoc, &maxLoc);

  // rescale so that max = 255, for visualization purposes
  cv::Mat temp = histogram.clone();
  temp = temp * 255.0 / max_val;

  // convert to uint
  histogram_norm = cv::Mat::zeros(histogram.size(), CV_8UC1); 
  temp.convertTo(histogram_norm, CV_8UC1); 
}

void createImageFromHistogram(
  const cv::Mat& histogram,
  cv::Mat& image)
{
  // normalize the histogram in range 0 - 255
  cv::Mat hist_norm;
  create8bitHistogram(histogram, hist_norm);

  image = cv::Mat::zeros(256, histogram.cols, CV_8UC1);
  for (int u = 0; u < histogram.cols; ++u)
  {
    uint8_t val = hist_norm.at<uint8_t>(0, u);
    for (int v = 0; v < val; ++v)
      image.at<uint8_t>(255-v, u) = 255;
  }
}

void create2DProjectionImage(
  const PointCloudT& cloud, 
  cv::Mat& img,
  double min_z,
  double max_z)
{
  // TODO: thses shouldnt be hard-coded
  double resolution = 0.02; // 2cm
  double w = 20.0;
  double h = 20.0;
  double cx = w/2;
  double cy = h/2;
  
  img = cv::Mat::zeros(h/resolution, w/resolution, CV_8UC1);

  for (unsigned int i = 0; i < cloud.points.size(); ++i)
  {
    const PointT& p = cloud.points[i];

    // filter z
    if (isnan(p.z) || p.z >= max_z || p.z < min_z) continue;

    int u = (h - p.x + cx)/resolution;
    int v = (p.y + cy)/resolution;

    uint8_t& bin = img.at<uint8_t>(v, u);
    if(bin < 255) bin++;
  }
}

void filterCloudByHeight(
  const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_in,
  pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_out,
  double min_z,
  double max_z)
{
  for (unsigned int i = 0; i < cloud_in.points.size(); ++i)
  {
    const pcl::PointXYZRGBNormal& p = cloud_in.points[i]; 
    
    if (p.z >= min_z && p.z < max_z)
      cloud_out.push_back(p); 
  }
}

void prepareFeaturesForRANSAC(KeyframeVector& keyframes)
{
  double init_surf_threshold = 400.0;
  double min_surf_threshold = 25;
  int n_keypoints = 400;

  printf("preparing SURF features for RANSAC associations...\n");  

  cv::SurfDescriptorExtractor extractor;
 
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); kf_idx++)
  { 
    RGBDKeyframe& keyframe = keyframes[kf_idx];

    double surf_threshold = init_surf_threshold;

    while (surf_threshold >= min_surf_threshold)
    {
      cv::SurfFeatureDetector detector(surf_threshold);
      keyframe.keypoints.clear();
      detector.detect(keyframe.rgb_img, keyframe.keypoints);

      printf("[KF %d of %d] %d SURF keypoints detected (threshold: %.1f)\n", 
        (int)kf_idx, (int)keyframes.size(), 
        (int)keyframe.keypoints.size(), surf_threshold); 

      if ((int)keyframe.keypoints.size() < n_keypoints)
        surf_threshold /= 2.0;
      else break;
    }

    extractor.compute(keyframe.rgb_img, keyframe.keypoints, keyframe.descriptors);
    keyframe.computeDistributions();
  }
}

void pairwiseMatchingRANSAC(
  const RGBDFrame& frame_a, const RGBDFrame& frame_b,
  double max_eucl_dist_sq, 
  double max_desc_dist,
  double sufficient_inlier_ratio,
  int max_ransac_iterations,
  std::vector<cv::DMatch>& all_matches,
  std::vector<cv::DMatch>& best_inlier_matches,
  Eigen::Matrix4f& best_transformation)
{
  // params
  bool use_ratio_test = true;
  float max_desc_ratio = 0.75;

  // constants
  int min_sample_size = 3;

  cv::FlannBasedMatcher matcher;          // for SURF
  TransformationEstimationSVD svd;

  std::vector<cv::DMatch> candidate_matches;

  // **** build candidate matches ***********************************
  // assumes detectors and distributions are computed
  // establish all matches from b to a

  if (use_ratio_test)
  {
    std::vector<std::vector<cv::DMatch> > all_matches2;
    
    matcher.knnMatch(
      frame_b.descriptors, frame_a.descriptors, all_matches2, 2);

    for (unsigned int m_idx = 0; m_idx < all_matches2.size(); ++m_idx)
    {
      const cv::DMatch& match1 = all_matches2[m_idx][0];
      const cv::DMatch& match2 = all_matches2[m_idx][1];
      
      double ratio =  match1.distance / match2.distance;
      
      // remove bad matches - ratio test, valid keypoints
      if (ratio < max_desc_ratio)
      {
        int idx_b = match1.queryIdx;
        int idx_a = match1.trainIdx; 

        if (frame_a.kp_valid[idx_a] && frame_b.kp_valid[idx_b])
          candidate_matches.push_back(match1);
      }
    }
  }
  else
  {
    matcher.match(
      frame_b.descriptors, frame_a.descriptors, all_matches);

    for (unsigned int m_idx = 0; m_idx < all_matches.size(); ++m_idx)
    {
      const cv::DMatch& match = all_matches[m_idx];

      // remove bad matches - descriptor distance, valid keypoints
      if (match.distance < max_desc_dist)
      {      
        int idx_b = match.queryIdx;
        int idx_a = match.trainIdx; 
        
        if (frame_a.kp_valid[idx_a] && frame_b.kp_valid[idx_b])
          candidate_matches.push_back(match);
      }
    }
  }

  int size = candidate_matches.size();
  //printf("size: %d\n", size);
  
  if (size < min_sample_size) return;
  
  // **** build 3D features for SVD ********************************

  PointCloudFeature features_a, features_b;

  features_a.resize(size);
  features_b.resize(size);

  for (int m_idx = 0; m_idx < size; ++m_idx)
  {
    const cv::DMatch& match = candidate_matches[m_idx];
    int idx_b = match.queryIdx;
    int idx_a = match.trainIdx; 

    PointFeature& p_a = features_a[m_idx];
    p_a.x = frame_a.kp_means[idx_a](0,0);
    p_a.y = frame_a.kp_means[idx_a](1,0);
    p_a.z = frame_a.kp_means[idx_a](2,0);

    PointFeature& p_b = features_b[m_idx];
    p_b.x = frame_b.kp_means[idx_b](0,0);
    p_b.y = frame_b.kp_means[idx_b](1,0);
    p_b.z = frame_b.kp_means[idx_b](2,0);
  }

  // **** main RANSAC loop ****************************************
  
  int best_n_inliers = 0;
  Eigen::Matrix4f transformation; // transformation used inside loop
  
  for (int iteration = 0; iteration < max_ransac_iterations; ++iteration)
  {   
    // generate random indices
    IntVector sample_idx;
    getRandomIndices(min_sample_size, size, sample_idx);
    
    // build initial inliers from random indices
    IntVector inlier_idx;
    std::vector<cv::DMatch> inlier_matches;

    for (unsigned int s_idx = 0; s_idx < sample_idx.size(); ++s_idx)
    {
      int m_idx = sample_idx[s_idx];
      inlier_idx.push_back(m_idx);
      inlier_matches.push_back(candidate_matches[m_idx]);
    } 
    
    // estimate transformation from minimum set of random samples
    svd.estimateRigidTransformation(
      features_b, inlier_idx,
      features_a, inlier_idx,
      transformation);

    // evaluate transformation fitness by checking distance to all points
    PointCloudFeature features_b_tf;
    pcl::transformPointCloud(features_b, features_b_tf, transformation);

    for (int m_idx = 0; m_idx < size; ++m_idx)
    {
      const PointFeature& p_a = features_a[m_idx];
      const PointFeature& p_b = features_b_tf[m_idx];

      float dist_sq = distEuclideanSq(p_a, p_b);
      
      if (dist_sq < max_eucl_dist_sq)
      {
        inlier_idx.push_back(m_idx);
        inlier_matches.push_back(candidate_matches[m_idx]);

        // reestimate transformation from all inliers
        svd.estimateRigidTransformation(
          features_b, inlier_idx,
          features_a, inlier_idx,
          transformation);
        pcl::transformPointCloud(features_b, features_b_tf, transformation);
      }
    }
    
    // check if inliers are better than the best model so far
    int n_inliers = inlier_idx.size();

    if (n_inliers > best_n_inliers)
    {
      svd.estimateRigidTransformation(
        features_b, inlier_idx,
        features_a, inlier_idx,
        transformation);

      best_n_inliers = n_inliers;
      best_transformation = transformation;
      best_inlier_matches = inlier_matches;
    }

    // check if we reached ratio termination criteria
    double inlier_ratio = (double) n_inliers / (double) size;

    if (inlier_ratio > sufficient_inlier_ratio)
      break;
  }
}

void getRandomIndices(
  int k, int n, IntVector& output)
{
  while ((int)output.size() < k)
  {
    int random_number = rand() % n;
    bool duplicate = false;    

    for (unsigned int i = 0; i < output.size(); ++i)
    {
      if (output[i] == random_number)
      {
        duplicate = true;
        break;
      }
    }

    if (!duplicate)
      output.push_back(random_number);
  }
}

double distEuclideanSq(const PointFeature& a, const PointFeature& b)
{
  float dx = a.x - b.x;
  float dy = a.y - b.y;
  float dz = a.z - b.z;
  return dx*dx + dy*dy + dz*dz;
}

void compareAssociationMatrix(
  const cv::Mat& a,
  const cv::Mat& b,
  int& false_pos,
  int& false_neg,
  int& total)
{
  false_pos = 0;
  false_neg = 0;
  total = 0;
  
  // assert both matrices are square, and same size
  assert(a.rows == a.cols);
  assert(b.rows == b.cols);
  assert(a.rows == b.rows);
  
  int size = a.rows;
  
  for (int u = 0; u < size; ++u)
  for (int v = 0; v < size; ++v)
  {
    int val_a = a.at<uint8_t>(v,u);
    int val_b = b.at<uint8_t>(v,u);
    
    if (val_a != 0 && val_b == 0) false_neg++;
    if (val_a == 0 && val_b != 0) false_pos++;
    
    if (u !=v && val_a != 0) total++;
  }
}

} // namespace ccny_rgbd
