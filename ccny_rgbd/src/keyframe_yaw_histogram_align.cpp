#include "ccny_rgbd/rgbd_normals.h"
#include "ccny_rgbd/types.h"

using namespace ccny_rgbd;

int main(int argc, char** argv)
{
  // params
  
  int kf_window = 5;
  
  if (argc != 2)
  {
    printf("error: usage is keyframe_yaw_histogram_align [path]\n");
    return -1;
  }
  
  std::string path = argv[1];
  KeyframeVector keyframes; 
  loadKeyframes(keyframes, path);
  
  for (unsigned int i = 0; i < keyframes.size(); i = i+kf_window)
  {
    PointCloudT::Ptr agr(new PointCloudT);
    
    for (unsigned int j = i; j < i+kf_window; ++j)
    {
      if (j >= keyframes.size()) continue;
      
      PointCloudT temp;
      pcl::transformPointCloud(keyframes[j].cloud, temp, eigenFromTf(keyframes[j].pose));
      *agr += temp;
    }
    
    double best_angle;
    
    printf("analyzing keyframe %d...\n", i);
    analyzeCloud(agr, best_angle);
    printf("analyzing keyframe %d done.\n", i);
    cv::waitKey(10);
    
    tf::Quaternion q;
    q.setRPY(0, 0, best_angle * M_PI / 180);
    tf::Transform tf;
    tf.setIdentity();
    tf.setRotation(q);
    
    for (unsigned int j = i; j < i+kf_window; ++j)
    {
      if (j >= keyframes.size()) continue;

      PointCloudT temp;
      pcl::transformPointCloud(keyframes[j].cloud, temp, eigenFromTf(keyframes[j].pose));    
      pcl::transformPointCloud(temp, keyframes[j].cloud, eigenFromTf(tf));
    }
  }
  
  saveKeyframes(keyframes, "/home/idyanov/ros/images/ccny_3rooms_seq_loop_hist");
  
  return 0;
}
