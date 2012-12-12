#ifndef CCNY_RGBD_MOTION_ESTIMATION_ICP_PROB_MODEL_H
#define CCNY_RGBD_MOTION_ESTIMATION_ICP_PROB_MODEL_H

#include <tf/transform_datatypes.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <visualization_msgs/Marker.h>

#include "ccny_rgbd/registration/motion_estimation.h"
#include "ccny_rgbd/Save.h"
#include "ccny_rgbd/Load.h"

namespace ccny_rgbd
{

typedef std::vector<cv::Mat> MatVector;

class MotionEstimationICPProbModel: public MotionEstimation
{
  typedef pcl::KdTreeFLANN<PointFeature> KdTree;

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MotionEstimationICPProbModel(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~MotionEstimationICPProbModel();

    bool getMotionEstimationImpl(
      RGBDFrame& frame,
      const tf::Transform& prediction,
      tf::Transform& motion);
  
    int getModelSize() const { return model_size_; }

    bool saveSrvCallback(ccny_rgbd::Save::Request& request,
                         ccny_rgbd::Save::Response& response);
    bool loadSrvCallback(ccny_rgbd::Save::Request& request,
                         ccny_rgbd::Save::Response& response);

  private:

    // **** ros

    ros::Publisher model_publisher_;
    ros::Publisher covariances_publisher_;
    ros::ServiceServer save_service_;
    ros::ServiceServer load_service_;

    // **** params

    std::string fixed_frame_; 
    std::string base_frame_;

    //double max_association_dist_;
    int max_iterations_;
    int min_correspondences_;
    int n_nearest_neighbors_; // for searching for mah NN
    int max_model_size_;      // bound for how many features to store in the model

    double tf_epsilon_linear_;   
    double tf_epsilon_angular_;

    double max_assoc_dist_mah_;
    double max_corresp_dist_mah_;
    double max_corresp_dist_eucl_;
    
    // derived
    double max_assoc_dist_mah_sq_;
    double max_corresp_dist_mah_sq_;
    double max_corresp_dist_eucl_sq_;

    // **** variables

    PointCloudFeature::Ptr model_ptr_;   
    int model_idx_;         // current intex in the ring buffer
    int model_size_;        // current model size
    MatVector covariances_;
    MatVector means_;

    KdTree::Ptr model_tree_ptr_;

    cv::Mat I_; // identity matrix
    
    tf::Transform f2b_; // Fixed frame to Base (moving) frame
    
    // ***** funtions

    bool alignICPEuclidean(
      const MatVector& data_means,
      tf::Transform& correction);
    
    bool alignICPMahalanobis(
      const MatVector& data_means_in,
      const MatVector& data_covariances_in,
      tf::Transform& correction);

    void getCorrespEuclidean(
      const PointCloudFeature& data_cloud,
      IntVector& data_indices,
      IntVector& model_indices);
    
    void getCorrespMahalanobis(
      const MatVector& data_means_in,
      const MatVector& data_covariances_in,
      IntVector& data_indices,
      IntVector& model_indices);
  
    bool getNNEuclidean(
      const PointFeature& data_point,
      int& eucl_nn_idx, double& eucl_dist_sq);

    bool getNNMahalanobis(
      const cv::Mat& data_mean, const cv::Mat& data_cov,
      int& mah_nn_idx, double& mah_dist_sq,
      IntVector& indices, FloatVector& dists_sq);
    
   // void updateModelFromFrame(const RGBDFrame& frame);

    void updateModelFromData(const MatVector& data_means,
                             const MatVector& data_covariances);

    void initializeModelFromFrame(const RGBDFrame& frame);
    
    void addToModel(const cv::Mat& feature_mean,
                    const cv::Mat& feature_cov);

    void publishCovariances();
    
    bool saveModel(const std::string& filename);
    bool loadModel(const std::string& filename);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_MOTION_ESTIMATION_ICP_PROB_MODEL_H

