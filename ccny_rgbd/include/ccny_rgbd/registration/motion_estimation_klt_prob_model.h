#ifndef CCNY_RGBD_MOTION_ESTIMATION_KLT_PROB_MODEL_H
#define CCNY_RGBD_MOTION_ESTIMATION_KLT_PROB_MODEL_H

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

class MotionEstimationKLTProbModel: public MotionEstimation
{
  typedef pcl::KdTreeFLANN<PointFeature> KdTree;

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MotionEstimationKLTProbModel(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~MotionEstimationKLTProbModel();

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

    int max_iterations_;
    int min_correspondences_;
    int n_nearest_neighbors_; // for searching for mah NN
    int max_model_size_;      // bound for how many features to store in the model

    double tf_epsilon_linear_;   
    double tf_epsilon_angular_;

    double max_assoc_dist_mah_;
    double max_corresp_dist_mah_;
    double max_corresp_dist_eucl_;
    
    bool publish_model_;      // for visualization
    bool publish_model_cov_;  // for visualization

    // derived
    double max_assoc_dist_mah_sq_;
    double max_corresp_dist_mah_sq_;
    double max_corresp_dist_eucl_sq_;

    // **** variables

    bool first_time_;
    cv::Mat prev_img_;
    std::vector<cv::Point2f> prev_points_;
    
    PointCloudFeature::Ptr model_ptr_;   
    int model_idx_;         // current intex in the ring buffer
    int model_size_;        // current model size
    Vector3fVector means_;
    Matrix3fVector covariances_;

    KdTree model_tree_;

    Matrix3f I_;
    
    tf::Transform f2b_; // Fixed frame to Base (moving) frame
    
    // ***** funtions
  
    void getCorrespEuclidean(
      const PointCloudFeature& data_cloud,
      IntVector& data_indices,
      IntVector& model_indices);
    
    void getCorrespMahalanobis(
      const Vector3fVector& data_means,
      const Matrix3fVector& data_covariances,
      IntVector& data_indices,
      IntVector& model_indices);
 
    bool getNNEuclidean(
      const PointFeature& data_point,
      int& eucl_nn_idx, double& eucl_dist_sq);

    bool getNNMahalanobis(
      const Vector3f& data_mean, const Matrix3f& data_cov,
      int& mah_nn_idx, double& mah_dist_sq,
      IntVector& indices, FloatVector& dists_sq);

    void updateModelFromData(
      const Vector3fVector& data_means,
      const Matrix3fVector& data_covariances,
      const IntVector& model_indices);
  
    void initializeModelFromData(
      const Vector3fVector& data_means,
      const Matrix3fVector& data_covariances);

    void addToModel(
      const Vector3f& data_mean,
      const Matrix3f& data_cov);

    void publishCovariances();
    
    bool saveModel(const std::string& filename);
    bool loadModel(const std::string& filename);
    
    void projectModelToFeaures(
      const RGBDFrame& frame,
      std::vector<cv::Point2f>& virtual_points, 
      std::vector<int>& virtual_point_indices);
      
    void showTrackingImage(
      const RGBDFrame& frame,
      const std::vector<cv::Point2f>& image_points, 
      const std::vector<cv::Point2f>& virtual_points, 
      const std::vector<uchar>& status);
      
    void detectFeatures(const RGBDFrame& frame);
    
    void ransacSVD(
      const PointCloudFeature& data_cloud, 
      const PointCloudFeature& model_cloud,
      IntVector& best_inlier_indices,
      tf::Transform& output_tf);
    
    void alignRansacSVD(
      const Vector3fVector& data_means,  
      const Matrix3fVector& data_covariances, 
      const Vector3fVector& model_means, 
      const Matrix3fVector& model_covariances, 
      IntVector& best_inlier_indices, 
      tf::Transform motion);

    // produces k random numbers in the range [0, n).
    // Monte-Carlo based random sampling
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
 
};

} // namespace ccny_rgbd

#endif // CCNY_RGBD_MOTION_ESTIMATION_KLT_PROB_MODEL_H

