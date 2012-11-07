#include "ccny_rgbd/mapping/keyframe_aggregator.h"

namespace ccny_rgbd
{

KeyframeAggregator::KeyframeAggregator(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  id_(0)
{
  map_pub_ = nh_.advertise<PointCloudT>(
    "agg_map", 1);

  model_.header.frame_id = "odom";

  cv::namedWindow("Data", 0);
  cv::namedWindow("Proj", 0);
  cv::namedWindow("Mask", 0);
  cv::namedWindow("Scale", 0);
  cv::namedWindow("Blend", 0);
}

KeyframeAggregator::~KeyframeAggregator()
{

}

void KeyframeAggregator::processKeyframe(
  const RGBDKeyframe& keyframe)
{
  printf("projecting...\n");

  // parameters
  bool save = false;
  float alpha = 0.5;
  float max_dist = 0.10;
  float max_dist_sq = max_dist * max_dist;

  // preliminaries
  const cv::Mat& data_img = *keyframe.getRGBImage();
  int w = data_img.cols;
  int h = data_img.rows;

  // create intrinsic matrix from camera model
  image_geometry::PinholeCameraModel cm = *keyframe.getModel();
 
  cv::Mat Mi = cv::Mat::zeros(3, 3, CV_32F);

  Mi.at<float>(0,0) = cm.fx();
  Mi.at<float>(1,1) = cm.fy();
  Mi.at<float>(0,2) = cm.cx();
  Mi.at<float>(1,2) = cm.cy();
  Mi.at<float>(2,2) = 1.0;

  // create extrinsic matrix from pose
  cv::Mat Me = cv::Mat::zeros(3, 4, CV_32F);
  Eigen::Matrix4f pose_eigen = eigenFromTf(keyframe.pose.inverse());

  for (int i = 0; i < 4; ++i)
  for (int j = 0; j < 3; ++j)
    Me.at<float>(j,i) = pose_eigen(j,i);

  // create combined projection matrix
  cv::Mat M = Mi * Me;

  // project model to image
  cv::Mat proj_img = cv::Mat::zeros(h, w, CV_8UC3);
  cv::Mat mask_img (h, w, CV_8UC1);
  cv::Mat idx_img  (h, w, CV_32S);   // points to a point in cloud
  projectCloudToImage(model_, M, proj_img, idx_img, mask_img);

  // create a scale image for blending
  cv::Mat scale_img(h, w, CV_32FC1);
  createScaleImage(mask_img, scale_img);

  // create a blended image
  cv::Mat blend_img (h, w, CV_8UC3);
  blendImages(*keyframe.getRGBImage(), proj_img, scale_img, blend_img);

  cv::imshow("Data",  data_img);
  cv::waitKey(1);
  cv::imshow("Proj",  proj_img);
  cv::waitKey(1);
  cv::imshow("Mask",  mask_img);
  cv::waitKey(1);
  cv::imshow("Scale", scale_img);
  cv::waitKey(1);
  cv::imshow("Blend", blend_img);
  cv::waitKey(1);

  if (save)
  {
    // save incoming image and prohjected image
    std::stringstream ss_filename_d, ss_filename_p, ss_filename_m;
    ss_filename_d << std::setw(4) << std::setfill('0') << id_ << "_d.png";
    ss_filename_p << std::setw(4) << std::setfill('0') << id_ << "_p.png";
    ss_filename_m << std::setw(4) << std::setfill('0') << id_ << "_m.png";

    cv::imwrite("/home/idryanov/ros/images/blend/" + ss_filename_d.str(), data_img);
    cv::imwrite("/home/idryanov/ros/images/blend/" + ss_filename_p.str(), proj_img);
    cv::imwrite("/home/idryanov/ros/images/blend/" + ss_filename_m.str(), mask_img);
  }

  // update model
  PointCloudT data_tf;
  pcl::transformPointCloud(keyframe.data, data_tf, eigenFromTf(keyframe.pose));

  for (int u = 0; u < w; ++u) 
  for (int v = 0; v < h; ++v) 
  {
    //int8_t mask = mask_img.at<int8_t>(v,u);
    //int model_idx = idx_img.at<int32_t>(v,u);
    int data_idx = v * w + u;

    const PointT& pd = data_tf.points[data_idx];
    cv::Vec3b color = blend_img.at<cv::Vec3b>(v,u);

    if (isnan(pd.z)) continue;

    PointT p = pd;
    p.r = color[2];
    p.g = color[1];
    p.b = color[0];

    model_.points.push_back(p);
  }

  // publish the model
  map_pub_.publish(model_);

  // increment keyframe counter
  id_++;
}

void KeyframeAggregator::projectCloudToImage(
  const PointCloudT& scene,  
  const cv::Mat& M,
  cv::Mat& projected_img,
  cv::Mat& idx_img,
  cv::Mat& mask_img)
{
  // params
  int W = 0;

  int w = projected_img.cols;
  int h = projected_img.rows;

  // build projected lookup image, each pixel is an index to cloud
  
  cv::Mat buffer_img(h, w, CV_32F);   // z-buffer

  // initialize idx_img to -1
  for (int i = 0; i < w; ++i) 
  for (int j = 0; j < h; ++j) 
  {
    idx_img.at<int32_t>(j, i) = -1; 
  }

  for (int p_idx = 0; p_idx < (int)scene.points.size(); ++p_idx)
  {
    // create cv matrix from point location
    const PointT& pt = scene.points[p_idx];
    cv::Mat p(4, 1, CV_32F);
    p.at<float>(0,0) = pt.x;
    p.at<float>(1,0) = pt.y;
    p.at<float>(2,0) = pt.z;
    p.at<float>(3,0) = 1.0;

    // project point
    cv::Mat q = M * p;

    float x = q.at<float>(0,0);
    float y = q.at<float>(1,0);
    float z = q.at<float>(2,0);

    // get pixel coordinates
    int u = (int)(x/z);
    int v = (int)(y/z);

    // test if inside image, and in front of camera
    if (u >= 0 && u < w &&
        v >= 0 && v < h &&
        z > 0.0)
    {
      for (int uu = u-W; uu <= u+W; ++uu)
      for (int vv = v-W; vv <= v+W; ++vv)
      {
        if (uu < 0 || uu >=w) continue;
        if (vv < 0 || vv >=h) continue;
      
        int32_t& idx  = idx_img.at<int32_t>(vv,uu);
        float& buffer_z = buffer_img.at<float>(vv,uu);
        if (idx == -1 || z < buffer_z)
        {
          buffer_z = z;
          idx = p_idx;
        }
      }
    }
  }

  // build projected color image and corresponding mask
  for (int u = 0; u < w; ++u) 
  for (int v = 0; v < h; ++v) 
  {
    int& idx  = idx_img.at<int32_t>(v,u);
    if (idx != -1)
    {
      PointT p = scene.points[idx];
      cv::Vec3b color;
      color[0] = p.b;
      color[1] = p.g;
      color[2] = p.r;
      projected_img.at<cv::Vec3b>(v,u) = color;
      mask_img.at<uint8_t>(v,u) = 255;
    }
    else
      mask_img.at<uint8_t>(v,u) = 0;
  }
}

void KeyframeAggregator::blendImages(
  const cv::Mat& data_img, 
  const cv::Mat& proj_img,
  const cv::Mat& scale_img,
  cv::Mat& blend_img)
{
  int w = data_img.cols;
  int h = data_img.rows;

  for (int u = 0; u < w; ++u) 
  for (int v = 0; v < h; ++v) 
  {
    cv::Vec3b d = data_img.at<cv::Vec3b>(v, u);
    cv::Vec3b p = proj_img.at<cv::Vec3b>(v, u);
    float     s = scale_img.at<float>(v, u);

    blend_img.at<cv::Vec3b>(v,u) = s * d + (1.0 - s) * p;
  }
}

void KeyframeAggregator::createScaleImage(
  const cv::Mat& mask_img, 
  cv::Mat& scale_img)
{
  // params
  float max_d = 100;

  int w = mask_img.cols;
  int h = mask_img.rows;

  // calculate dist image
  cv::Mat dist_img(h, w, CV_32FC1);
  cv::distanceTransform(mask_img, dist_img, CV_DIST_L2, 3);

  // calculate scale image
  for (int u = 0; u < w; ++u) 
  for (int v = 0; v < h; ++v) 
  {
    float  d = dist_img.at<float>(v, u);
    float& s = scale_img.at<float>(v, u);
    
    float d_clipped = std::min(d, max_d);

    s = (max_d - d_clipped) / max_d;
  }
}

} // namespace ccny_rgbd
