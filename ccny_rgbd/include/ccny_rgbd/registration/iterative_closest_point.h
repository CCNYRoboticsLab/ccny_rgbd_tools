#ifndef CCNY_RGBD_ITERATIVE_CLOSEST_POINT
#define CCNY_RGBD_ITERATIVE_CLOSEST_POINT

#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include "ccny_rgbd/rgbd_util.h"

namespace ccny_rgbd
{

template <typename PointData, typename PointModel>
class IterativeClosestPoint
{
  typedef pcl::PointCloud<PointData>  PointCloudData;
  typedef pcl::PointCloud<PointModel> PointCloudModel;
  typedef pcl::KdTree<PointModel>  KdTree;
  typedef std::vector<int>   IntVector;
  typedef std::vector<float> FloatVector;

  typedef typename KdTree::Ptr          KdTreePtr;
  typedef typename PointCloudData::Ptr  PointCloudDataPtr;
  typedef typename PointCloudModel::Ptr PointCloudModelPtr;

  public:

    IterativeClosestPoint();
    virtual ~IterativeClosestPoint();

    void setDataCloud(PointCloudDataPtr cloud_data);
    void setModelCloud(PointCloudModelPtr cloud_model);
    void setModelTree(KdTreePtr tree_model);

    inline void setMaxIterations(int max_iterations) {max_iterations_ = max_iterations; }
    inline void setMaxCorrDist(float corr_dist_threshold) {corr_dist_threshold_ = corr_dist_threshold;}
    inline void setTransformationEpsilon(float transformation_epsilon) {transformation_epsilon_ = transformation_epsilon; }

    inline int getFinalIterations() const { return iterations_; }

    bool align();

    inline Eigen::Matrix4f getFinalTransformation() const { return final_transformation_; }

  private:

    // **** parameters

    int max_iterations_;            // for value-based correspondence search
    float corr_dist_threshold_;     // max euclidean distance between corresponding points
    float transformation_epsilon_;  // for convergence detection
    int min_correspondences_;       // minimum required correspondences

    // **** input

    PointCloudDataPtr cloud_data_;
    PointCloudModelPtr cloud_model_;
    KdTreePtr tree_model_;

    // **** state

    int iterations_;                        // current number of iterations
    Eigen::Matrix4f transformation_;
    Eigen::Matrix4f previous_transformation_;
    Eigen::Matrix4f final_transformation_;
};

#include "ccny_rgbd/registration/iterative_closest_point.hpp"

} //namespace ccny_rgbd

#endif // CCNY_RGBD_CCNY_RGBD_ITERATIVE_CLOSEST_POINT
