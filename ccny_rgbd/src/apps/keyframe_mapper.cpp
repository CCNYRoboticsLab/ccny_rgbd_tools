#include "ccny_rgbd/apps/keyframe_mapper.h"

namespace ccny_rgbd
{

KeyframeMapper::KeyframeMapper(ros::NodeHandle nh, ros::NodeHandle nh_private):
  KeyframeGenerator(nh, nh_private)
{
  ROS_INFO("Starting RGBD Keyframe Mapper");

  // **** init variables

  keyframes_pub_ = nh_.advertise<PointCloudT>(
    "keyframes", 1);
  poses_pub_ = nh_.advertise<visualization_msgs::Marker>( 
    "keyframe_poses", 1);
  edges_pub_ = nh_.advertise<visualization_msgs::Marker>( 
    "keyframe_edges", 1);

  // **** services

  pub_frame_service_ = nh_.advertiseService(
    "publish_keyframe", &KeyframeMapper::publishKeyframeSrvCallback, this);
  pub_frames_service_ = nh_.advertiseService(
    "publish_keyframes", &KeyframeMapper::publishAllKeyframesSrvCallback, this);

 // **** subscribers

  image_transport::ImageTransport rgb_it(nh_);
  image_transport::ImageTransport depth_it(nh_);

  sub_depth_.subscribe(
    depth_it, "/camera/depth_registered/image_rect_raw", 1);
  sub_rgb_.subscribe(
    rgb_it, "/camera/rgb/image_rect_color", 1);
  sub_info_.subscribe(
    nh_, "/camera/rgb/camera_info", 1);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  int queue_size = 5;
  sync_.reset(new Synchronizer(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_));
  sync_->registerCallback(boost::bind(&KeyframeMapper::RGBDCallback, this, _1, _2, _3));  
}

KeyframeMapper::~KeyframeMapper()
{

}

void KeyframeMapper::RGBDCallback(
  const ImageMsg::ConstPtr& depth_msg,
  const ImageMsg::ConstPtr& rgb_msg,
  const CameraInfoMsg::ConstPtr& info_msg)
{
  tf::StampedTransform transform;

  const ros::Time& time = rgb_msg->header.stamp;

  try{
    tf_listener_.waitForTransform(
     fixed_frame_, rgb_msg->header.frame_id, time, ros::Duration(0.1));
    tf_listener_.lookupTransform(
      fixed_frame_, rgb_msg->header.frame_id, time, transform);  
  }
  catch(...)
  {
    return;
  }
  RGBDFrame frame(rgb_msg, depth_msg, info_msg);
  bool result = KeyframeGenerator::processFrame(frame, transform);
  if (result) publishKeyframeData(keyframes_.size() - 1);
}

bool KeyframeMapper::publishKeyframeSrvCallback(
  PublishKeyframe::Request& request,
  PublishKeyframe::Response& response)
{
  if (request.id < 0 || request.id >= (int)keyframes_.size())
  {
    ROS_ERROR("request.id %d out of bounds (%d keyframes)", (int)request.id, (int)keyframes_.size());
    return false;
  }

  publishKeyframeData(request.id);
  publishKeyframePose(request.id);
  usleep(25000);

  return true;
}

bool KeyframeMapper::publishAllKeyframesSrvCallback(
  PublishAllKeyframes::Request&  request,
  PublishAllKeyframes::Response& response)
{
  if (request.step <= 0)
  {
    ROS_ERROR("request.step has to be >= 1");
    return false;
  }

  for (int i = 0; i < (int)keyframes_.size(); i += request.step)
  {
    publishKeyframeData(i);
    publishKeyframePose(i);
    usleep(25000);
  }

  publishEdges();

  return true;
}

void KeyframeMapper::publishKeyframeData(int i)
{
  RGBDKeyframe& keyframe = keyframes_[i];

  // **** publish PointCloud data

  keyframe.constructDataCloud();

  PointCloudT keyframe_data_ff; // data transformed to the fixed frame
  
  pcl::transformPointCloud(
    keyframe.data, keyframe_data_ff, eigenFromTf(keyframe.pose));

  keyframe_data_ff.header.frame_id = fixed_frame_;

  keyframes_pub_.publish(keyframe_data_ff);
}

void KeyframeMapper::publishEdges()
{
  visualization_msgs::Marker marker_edge;
  marker_edge.header.stamp = ros::Time::now();
  marker_edge.header.frame_id = fixed_frame_;
  marker_edge.ns = "consecutive";
  marker_edge.id = 0;
  marker_edge.type = visualization_msgs::Marker::LINE_STRIP;
  marker_edge.action = visualization_msgs::Marker::ADD;

  marker_edge.points.resize(keyframes_.size());
  marker_edge.scale.x = 0.001;

  marker_edge.color.a = 1.0;
  marker_edge.color.r = 1.0;
  marker_edge.color.g = 1.0;
  marker_edge.color.b = 0.0;

  for (unsigned int i = 0; i < keyframes_.size(); ++i)
  {
    RGBDKeyframe& keyframe = keyframes_[i];

    // start point for the edge
    marker_edge.points[i].x = keyframe.pose.getOrigin().getX();  
    marker_edge.points[i].y = keyframe.pose.getOrigin().getY();
    marker_edge.points[i].z = keyframe.pose.getOrigin().getZ();
  }

  edges_pub_.publish(marker_edge);
}

void KeyframeMapper::publishKeyframePose(int i)
{
  RGBDKeyframe& keyframe = keyframes_[i];

  // **** publish camera pose

  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = fixed_frame_;
  marker.ns = "keyframe_poses";
  marker.id = i;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.points.resize(2);

  // start point for the arrow
  marker.points[0].x = keyframe.pose.getOrigin().getX();
  marker.points[0].y = keyframe.pose.getOrigin().getY();
  marker.points[0].z = keyframe.pose.getOrigin().getZ();

  // end point for the arrow
  tf::Transform ep; 
  ep.setIdentity();
  ep.setOrigin(tf::Vector3(0.00, 0.00, 0.12)); // z = arrow length
  ep = keyframe.pose * ep;

  marker.points[1].x = ep.getOrigin().getX();
  marker.points[1].y = ep.getOrigin().getY();
  marker.points[1].z = ep.getOrigin().getZ(); 
  
  marker.scale.x = 0.02; // shaft radius
  marker.scale.y = 0.05; // head radius

  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  poses_pub_.publish(marker);

  // **** publish frame index text

  visualization_msgs::Marker marker_text;
  marker_text.header.stamp = ros::Time::now();
  marker_text.header.frame_id = fixed_frame_;
  marker_text.ns = "keyframe_indexes";
  marker_text.id = i;
  marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_text.action = visualization_msgs::Marker::ADD;

  tf::poseTFToMsg(keyframe.pose, marker_text.pose);

  marker_text.pose.position.z -= 0.05;

  char label[6];
  sprintf(label, "%d", i);
  marker_text.text = label;

  marker_text.color.a = 1.0;
  marker_text.color.r = 1.0;
  marker_text.color.g = 1.0;
  marker_text.color.b = 0.0;

  marker_text.scale.z = 0.05; // shaft radius

  poses_pub_.publish(marker_text);
}

} // namespace ccny_rgbd
