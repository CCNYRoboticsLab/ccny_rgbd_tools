#ifndef CCNY_RGBD_ICP_KD
#define CCNY_RGBD_ICP_KD

#include "ccny_rgbd/rgbd_util.h"

#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>

namespace ccny_rgbd
{

template <typename PointData, typename PointModel>
class ICPKd
{
  public:

    typedef pcl::PointCloud<PointData>  PointCloudData;
    typedef pcl::PointCloud<PointModel> PointCloudModel;

    typedef pcl::KdTree<PointData>   KdTreeData;
    typedef pcl::KdTree<PointModel>  KdTreeModel;

    typedef std::vector<int>   IntVector;
    typedef std::vector<float> FloatVector;

    typedef typename pcl::SampleConsensusModelRegistration<PointData>::Ptr SampleConsensusModelRegistrationPtr;

    ICPKd();
    virtual ~ICPKd();

    inline void setDataCloud  (PointCloudData  * cloud_data);
    inline void setModelCloud (PointCloudModel * cloud_model);

    inline void setDataTree  (KdTreeData  * tree_data);
    inline void setModelTree (KdTreeModel * tree_model);

    inline void setMaxIterations(int max_iterations) {max_iterations_ = max_iterations; }
    inline void setMaxValueDiff(float max_value_diff) {max_value_diff_ = max_value_diff;}
    inline void setMaxCorrDist(float corr_dist_threshold) {corr_dist_threshold_ = corr_dist_threshold;}
    inline void setRANSACThreshold(float ransac_inlier_threshold) { ransac_inlier_threshold_ = ransac_inlier_threshold; }
    inline void setTransformationEpsilon(float transformation_epsilon) {transformation_epsilon_ = transformation_epsilon; }

    inline void setUseRANSACRejection(bool use_ransac_rejection) { use_ransac_rejection_ = use_ransac_rejection;}
    inline void setUseValueRejection (bool use_value_rejection) { use_value_rejection_ = use_value_rejection;}

    inline int getFinalIterations() const { return iterations_; }

    bool align();

    inline Eigen::Matrix4f getFinalTransformation() const { return final_transformation_; }

  private:

    // **** parameters

    int max_iterations_;            // for value-based correspondence search
    float corr_dist_threshold_;     // max euclidean distance between corresponding points
    float transformation_epsilon_;  // for convergence detection
    int min_correspondences_;       // minimum required correspondences
    float ransac_inlier_threshold_; // threshold for RANSAC rejection of outliers
    bool use_ransac_rejection_;     // whether to do RANSAC rejection
    bool use_value_rejection_;      // whether to do value rejection
    float max_value_diff_;          // maximum difference when doing value rejection

    PointCloudData  * cloud_data_;
    PointCloudModel * cloud_model_;

    KdTreeData  * tree_data_;
    KdTreeModel * tree_model_;

    // **** state

    int iterations_;                        // current number of iterations
    Eigen::Matrix4f transformation_;
    Eigen::Matrix4f previous_transformation_;
    Eigen::Matrix4f final_transformation_;

    void rejectionRansac(IntVector& source_indices, IntVector& target_indices);
    void rejectionValue (IntVector& source_indices, IntVector& target_indices);

};

#include "ccny_rgbd/registration/icp_kd.hpp"

} //namespace ccny_rgbd

#endif // CCNY_RGBD_ICP_KD
