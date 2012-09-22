#ifndef CCNY_RGBD_TYPES_H
#define CCNY_RGBD_TYPES_H

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace ccny_rgbd {

typedef sensor_msgs::Image ImageMsg;
typedef sensor_msgs::CameraInfo CameraInfoMsg;

typedef image_transport::SubscriberFilter ImageSubFilter;
typedef message_filters::Subscriber<CameraInfoMsg> CameraInfoSubFilter;
typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, CameraInfoMsg> SyncPolicy;
typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

} // namespace ccny_rgbd

#endif // CCNY_RGBD_TYPES_H
