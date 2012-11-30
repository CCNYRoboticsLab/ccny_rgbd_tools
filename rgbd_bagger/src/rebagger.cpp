#include <rgbd_bagger/rebagger.h>
#include <boost/foreach.hpp>

namespace rgbd_bagger
{

void help()
{
  printf("\n\n"
    "CCNY STEREO\n"
    "   \n"
    "\n");
}

void thresholdTrackbarHandler(int nBarValue, void *)
{
  ROS_INFO("Threshold: %d\n", nBarValue);
}

void zoomTrackbarHandler(int nBarValue, void* user_data)
{
  bool* that = static_cast<bool*> (user_data);
  *that = true;
  ROS_INFO("Zoom: %d\n", nBarValue);
}

Rebagger::Rebagger(ros::NodeHandle &nh, ros::NodeHandle &nh_private) :
  nh_(nh), nh_private_(nh_private)
{
#ifdef STEREO_DEVELOP
  cv::namedWindow(LEFT_WINDOW, CV_WINDOW_FREERATIO);
  cv::namedWindow(RIGHT_WINDOW, CV_WINDOW_FREERATIO);
#endif

  getParams();

  controlLoop();
}

// %Tag(PARAMS)%
void Rebagger::getParams()
{
  loaded_params_ = false;

  // TODO: parametrize topic names:
  topic_in_depth_cam_info_ = "/camera/depth/camera_info";
  topic_in_rgb_cam_info_ = "/camera/rgb/camera_info";
  topic_in_depth_image_ = "/camera/depth/image";
  topic_in_rgb_image_ = "/camera/rgb/image_color";
  topic_in_marker_ = "/cortex_marker_array";
  topic_in_imu_ = "/imu";
  topic_in_tf_ = "/tf";

  topic_out_depth_cam_info_ = "/camera/depth_registered/camera_info";
  topic_out_rgb_cam_info_ = "/camera/rgb/camera_info";
  topic_out_depth_image_ = "/camera/depth_registered/image_raw";
  topic_out_rgb_image_ = "/camera/rgb/image_raw";
  topic_out_marker_ = "/cortex_marker_array";
  topic_out_imu_ = "/imu";
  topic_out_tf_ = "/tf";

  topics_in_.push_back(std::string(topic_in_depth_cam_info_));
  topics_in_.push_back(std::string(topic_in_rgb_cam_info_));
  topics_in_.push_back(std::string(topic_in_depth_image_));
  topics_in_.push_back(std::string(topic_in_rgb_image_));
  topics_in_.push_back(std::string(topic_in_marker_));
  topics_in_.push_back(std::string(topic_in_imu_));
  topics_in_.push_back(std::string(topic_in_tf_));

  nh_private_.param("number_of_bags", number_of_bags_, 1);

  nh_private_.param("depth_resolution_factor", depth_resolution_factor_, 1000.0);

  nh_private_.param("image_resolution_mode", image_resolution_mode_, std::string("QVGA"));

  if(image_resolution_mode_ == "QVGA")
  {
    camera_width_ = 320;
    camera_height_ = 240;
    resize_factor_ = 2.0;
    do_resizing_ = true;
  }
  else if(image_resolution_mode_ == "QQVGA")
  {
    camera_width_ = 160;
    camera_height_ = 120;
    resize_factor_ = 4.0;
    do_resizing_ = true;
  }
  else if(image_resolution_mode_ == "VGA")
  {
    camera_width_ = 640;
    camera_height_ = 480;
    resize_factor_ = 1.0;
    do_resizing_ = false;
   }

  output_img_size_ = cv::Size((int)camera_width_, (int)camera_height_);

  // Choose the depth image's cv::Mat type, such as 32FC1, 16UC1, etc.
  nh_private_.param("depth_image_encoding", depth_image_encoding_, std::string("CV_16UC1"));

  std::string interpolation; // for resizing method
  nh_private_.param("interpolation_mode", interpolation, std::string("CV_INTER_LINEAR"));

  if(interpolation == "CV_INTER_NN")
  {
    interpolation_ = CV_INTER_NN;
    ROS_INFO("Interpolation type: %s", interpolation.c_str());
  }
  if(interpolation == "CV_INTER_LINEAR")
  {
    interpolation_ = CV_INTER_LINEAR;
    ROS_INFO("Interpolation type: %s", interpolation.c_str());
  }
  if(interpolation == "CV_INTER_CUBIC")
  {
    interpolation_ = CV_INTER_CUBIC;
    ROS_INFO("Interpolation type: %s", interpolation.c_str());
  }
  if(interpolation == "CV_INTER_AREA")
  {
    interpolation_ = CV_INTER_AREA;
    ROS_INFO("Interpolation type: %s", interpolation.c_str());
  }
  if(interpolation == "CV_INTER_LANCZOS4")
  {
    interpolation_ = CV_INTER_LANCZOS4;
    ROS_INFO("Interpolation type: %s", interpolation.c_str());
  }

  if (nh_private_.getParam("bag_name_suffix", bag_suffix_))
  {
    nh_private_.param("input_bag_name_prefix", input_bag_file_name_, std::string("input"));
    nh_private_.param("output_bag_name_prefix", output_bag_file_name_, std::string("output"));

      std::string input_bag_name, output_bag_name;
      if(number_of_bags_ == 1)
      {
        input_bag_name = input_bag_file_name_ + "." + bag_suffix_;
        output_bag_name = output_bag_file_name_ + "_" + image_resolution_mode_ + "." + bag_suffix_;
      }
      else
      {
        input_bag_name = input_bag_file_name_ + "-0." + bag_suffix_;
        output_bag_name = output_bag_file_name_  + "_" + image_resolution_mode_ + "-0." + bag_suffix_;
      }
      ROS_INFO("Using FIRST input bag with file name: \"%s\"", input_bag_name.c_str());
  }

  nh_private_.param("ground_truth_frame_name", ground_truth_frame_name_, std::string("/openni_camera"));

  loaded_params_ = true;
}
// %EndTag(PARAMS)%


void Rebagger::controlLoop()
{
  bool processing = true;
  while (processing && ros::ok())
  {

    static int bag_count = 0;
    char count_as_char[5];
    sprintf(count_as_char,"%d",bag_count); // converts to decimal base.= itoa(frame_count);

    std::string input_bag_name;
    std::string output_bag_name;

    if(number_of_bags_ == 1)
    {
      input_bag_name = input_bag_file_name_ + "." + bag_suffix_;
      output_bag_name = output_bag_file_name_ + "_" + image_resolution_mode_ + "." + bag_suffix_;

      processing = false; // Finish
    }
    else
    {
      input_bag_name = input_bag_file_name_+ count_as_char + "." + bag_suffix_;
      output_bag_name = output_bag_file_name_ + "_" + image_resolution_mode_ + "-" + count_as_char + "." + bag_suffix_;

      bag_count++;
      if(bag_count == number_of_bags_)
      {
        processing = false; // Finish
      }
    }
    ROS_INFO("FILENAMES: IN: %s --> OUT: %s", input_bag_name.c_str(), output_bag_name.c_str());

    bag_in_ = new rosbag::Bag(input_bag_name, rosbag::bagmode::Read);
    bag_out_ = new rosbag::Bag(output_bag_name, rosbag::bagmode::Write);

#ifdef DEVELOP
    duration = static_cast<double> (cv::getTickCount()) - duration;
    duration /= cv::getTickFrequency(); // the elapsed time in ms
    ROS_INFO("Capture time = %f ms", duration); // In my laptop it's taking about 40ms to capture each frame
#endif
    processBag(bag_in_, bag_out_);

    bag_in_->close();
    bag_out_->close();

    // Release bag pointers
    delete bag_in_;
    delete bag_out_;
//    ros::spinOnce(); // ros::spin  not needed if you want to exit the program automatically when finished
  }
  ROS_INFO("No longer processing bags");
}


void Rebagger::processBag(rosbag::Bag *bag_in, rosbag::Bag *bag_out)
{
    std::string hello( "Hello, world!" );

  rosbag::View view(*bag_in, rosbag::TopicQuery(topics_in_));

  ROS_DEBUG("View's size = %d", view.size());

  ROS_INFO("Processing bag at %s", bag_in->getFileName().c_str());

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    ROS_DEBUG("Query Message Instance size: %d", m.size());

//    ROS_INFO("First topic name: %s", m.getTopic().c_str());

    //--------------------------------------------------------------------------------------------------------------------
    //------------ topic_in_depth_cam_info_ ---------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    if(m.getTopic() == topic_in_depth_cam_info_)
    {
      sensor_msgs::CameraInfo::ConstPtr depth_cam_info_in = m.instantiate<sensor_msgs::CameraInfo> ();
      if (depth_cam_info_in != NULL)
      {
        sensor_msgs::CameraInfoPtr depth_cam_info_out =
            sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo(*depth_cam_info_in));
        depth_cam_info_out->width = camera_width_;
        depth_cam_info_out->height = camera_height_;
        // Rescale optical center values:
        depth_cam_info_out->K[2] = depth_cam_info_out->K[2] / resize_factor_;
        depth_cam_info_out->K[5] = depth_cam_info_out->K[5] / resize_factor_;
        depth_cam_info_out->P[2] = depth_cam_info_out->P[2] / resize_factor_;
        depth_cam_info_out->P[6] = depth_cam_info_out->P[6] / resize_factor_;
        // Rescale focal length values:
        depth_cam_info_out->K[0] = depth_cam_info_out->K[0] / resize_factor_;
        depth_cam_info_out->K[4] = depth_cam_info_out->K[4] / resize_factor_;
        depth_cam_info_out->P[0] = depth_cam_info_out->P[0] / resize_factor_;
        depth_cam_info_out->P[5] = depth_cam_info_out->P[5] / resize_factor_;

        ROS_DEBUG("(cx,cy): (K[2]:%.1f, K[5]:%.1f), (P[2]:%.1f, P[6]:%.1f)", depth_cam_info_out->K[2], depth_cam_info_out->K[5], depth_cam_info_out->P[2], depth_cam_info_out->P[6]);
        ROS_DEBUG("(fx,fy): (K[0]:%.1f, K[4]:%.1f), (P[0]:%.1f, P[5]:%.1f)", depth_cam_info_out->K[0], depth_cam_info_out->K[4], depth_cam_info_out->P[0], depth_cam_info_out->P[5]);

        depth_cam_info_out->header.frame_id = "/camera_depth_optical_frame";
        // Write to output bag:
        bag_out->write(topic_out_depth_cam_info_, depth_cam_info_out->header.stamp, depth_cam_info_out);
      }
    }
    //--------------------------------------------------------------------------------------------------------------------

    //--------------------------------------------------------------------------------------------------------------------
    //------------ topic_in_rgb_cam_info_ ---------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    if(m.getTopic() == topic_in_rgb_cam_info_)
    {
      sensor_msgs::CameraInfo::ConstPtr rgb_cam_info_in = m.instantiate<sensor_msgs::CameraInfo>();
      if (rgb_cam_info_in != NULL)
      {
        sensor_msgs::CameraInfoPtr rgb_cam_info_out = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo(*rgb_cam_info_in));
        rgb_cam_info_out->width = camera_width_;
        rgb_cam_info_out->height = camera_height_;
        // Rescale optical center values:
        rgb_cam_info_out->K[2] = rgb_cam_info_out->K[2]/resize_factor_;
        rgb_cam_info_out->K[5] = rgb_cam_info_out->K[5]/resize_factor_;
        rgb_cam_info_out->P[2] = rgb_cam_info_out->P[2]/resize_factor_;
        rgb_cam_info_out->P[6] = rgb_cam_info_out->P[6]/resize_factor_;
        // Rescale focal length values:
        rgb_cam_info_out->K[0] = rgb_cam_info_out->K[0]/resize_factor_;
        rgb_cam_info_out->K[4] = rgb_cam_info_out->K[4]/resize_factor_;
        rgb_cam_info_out->P[0] = rgb_cam_info_out->P[0]/resize_factor_;
        rgb_cam_info_out->P[5] = rgb_cam_info_out->P[5]/resize_factor_;

        ROS_DEBUG("(cx,cy): (K[2]:%.1f, K[5]:%.1f), (P[2]:%.1f, P[6]:%.1f)", rgb_cam_info_out->K[2], rgb_cam_info_out->K[5], rgb_cam_info_out->P[2], rgb_cam_info_out->P[6]);
        ROS_DEBUG("(fx,fy): (K[0]:%.1f, K[4]:%.1f), (P[0]:%.1f, P[5]:%.1f)", rgb_cam_info_out->K[0], rgb_cam_info_out->K[4], rgb_cam_info_out->P[0], rgb_cam_info_out->P[5]);

        rgb_cam_info_out->header.frame_id = "/camera_rgb_optical_frame";
        // Write to output bag:
        bag_out->write(topic_out_rgb_cam_info_, rgb_cam_info_out->header.stamp, rgb_cam_info_out);
      }
    }
    //--------------------------------------------------------------------------------------------------------------------

    //--------------------------------------------------------------------------------------------------------------------
    //------------ topic_in_depth_image_ ---------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    if(m.getTopic() == topic_in_depth_image_)
    {
      sensor_msgs::ImageConstPtr depth_img_in_ptr = m.instantiate<sensor_msgs::Image>();

      if (depth_img_in_ptr != NULL)
      {
        cv::Mat depth_frame_out(output_img_size_, CV_16UC1);
        cv::Mat depth_frame_resized;

        cv_bridge::CvImageConstPtr depth_img_msg_in = cv_bridge::toCvShare(depth_img_in_ptr);

        if(do_resizing_)
          cv::resize(depth_img_msg_in->image, depth_frame_resized, output_img_size_, 0, 0, interpolation_);
        else
          depth_frame_resized = depth_img_msg_in->image;

        std::string encoding = depth_img_in_ptr->encoding;

        if (encoding != depth_image_encoding_) // Possible modes are: "mono8", "bgr8", "bgra8", "rgb8", "rgba8", "mono16", and 32FC1?
        {
          if(encoding == "32FC1") // units in meters
          {
        	//depth_frame_out = depth_frame_resized * (float) depth_resolution_factor_; // units in milimeters for 16UC1

        	for (int row = 0; row < camera_height_; row++)
            {
              for (int col = 0; col < camera_width_; col++)
              {
                float mm = depth_frame_resized.at<float> (row, col) * (float) depth_resolution_factor_; // at(row,col)
                depth_frame_out.at<uint16_t> (row, col) = (uint16_t) mm; // at(row,col)
              }
            }
          }
#ifdef REBAGGER_DEVELOP
        static bool is_first_time_callback = true;
        if(is_first_time_callback)
        {
          ROS_INFO("Depth Image Original encoding: %s", encoding.c_str());
          is_first_time_callback = false;
          std::cout << "ORIGINAL as FLOAT" << std::endl << depth_frame_resized << std::endl << std::endl;
          std::cout << "CONVERTED to UINT_16" << std::endl << depth_frame_out << std::endl << std::endl;
        }
        cv::namedWindow("CONVERTED to UINT_16");
        cv::imshow("CONVERTED to UINT_16", depth_frame_out);

        cv::waitKey(10);
#endif

          //depth_frame_out.convertTo(depth_frame_out, CV_16UC1);
          //          encoding = sensor_msgs::image_encodings::MONO16;
          encoding = "16UC1";
        }

        cv_bridge::CvImage cv_depth_img;
        cv_depth_img.header = depth_img_in_ptr->header;
        cv_depth_img.encoding = encoding;
        cv_depth_img.image = depth_frame_out; // the resized cv::Mat
        sensor_msgs::ImagePtr depth_img_out = cv_depth_img.toImageMsg();

        depth_img_out->encoding = encoding;

        // TODO: parametrize name:
        depth_img_out->header.frame_id = "/camera_rgb_optical_frame";

        bag_out->write(topic_out_depth_image_, depth_img_out->header.stamp, depth_img_out);
      }
    }
    //--------------------------------------------------------------------------------------------------------------------

    //--------------------------------------------------------------------------------------------------------------------
    //------------ topic_in_rgb_image_ ---------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    if(m.getTopic() == topic_in_rgb_image_)
    {
      sensor_msgs::ImageConstPtr rgb_img_in = m.instantiate<sensor_msgs::Image>();

      if (rgb_img_in != NULL)
      {
        cv::Mat rgb_frame_out;

        cv_bridge::CvImageConstPtr rgb_img_out_ptr = cv_bridge::toCvShare(rgb_img_in);

        if(do_resizing_)
          cv::resize(rgb_img_out_ptr->image, rgb_frame_out, output_img_size_, 0, 0, interpolation_);
        else
          rgb_frame_out = rgb_img_out_ptr->image.clone();

        cv_bridge::CvImage cv_rgb_img;
        cv_rgb_img.header = rgb_img_in->header;
        cv_rgb_img.encoding = rgb_img_in->encoding;
        cv_rgb_img.image = rgb_frame_out; // the resized cv::Mat
        sensor_msgs::ImagePtr rgb_img_out = cv_rgb_img.toImageMsg();

        // TODO: parametrize name:
        rgb_img_out->header.frame_id = "/camera_rgb_optical_frame";

        bag_out->write(topic_out_rgb_image_, rgb_img_out->header.stamp, rgb_img_out);
      }
    }

    /* FIXME: crashes in 777777777777777777777777
    //--------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    //------------ topic_in_rgb_image_ ---------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    if(m.getTopic() == topic_in_marker_)
    {
      ROS_INFO("Found topic in markers");

      visualization_msgs::MarkerArrayConstPtr markers_in = m.instantiate<visualization_msgs::MarkerArray>();

      ROS_INFO("Instantiated in markers passed");

      if(markers_in != NULL)
//      if (markers_in->markers.empty() == false)
      {

        777777777777777    visualization_msgs::MarkerArrayPtr markers_out = visualization_msgs::MarkerArrayPtr(new visualization_msgs::MarkerArray(*markers_in));

        ROS_INFO("Created out markers passed");

        visualization_msgs::Marker first_marker = markers_out->markers.front();


        ROS_INFO("GOOD in markers passed");

        // Write to output bag:
        bag_out->write(topic_out_marker_, first_marker.header.stamp, markers_out);
      }
    }
    //--------------------------------------------------------------------------------------------------------------------
     */

    //--------------------------------------------------------------------------------------------------------------------
    //------------ topic_in_imu_ ---------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    if(m.getTopic() == topic_in_imu_)
    {
      sensor_msgs::ImuConstPtr imu_in = m.instantiate<sensor_msgs::Imu>();

      if (imu_in != NULL)
      {
        sensor_msgs::ImuPtr imu_out = sensor_msgs::ImuPtr(new sensor_msgs::Imu(*imu_in));
        // Write to output bag:
        bag_out->write(topic_out_imu_, imu_out->header.stamp, imu_out);
      }
    }
    //--------------------------------------------------------------------------------------------------------------------

    //--------------------------------------------------------------------------------------------------------------------
    //------------ topic_in_tf_ ---------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------
    if(m.getTopic() == topic_in_tf_)
    {
      tf::tfMessageConstPtr tf_in = m.instantiate<tf::tfMessage>();

      if (tf_in->transforms.empty() == false)
      {
        tf::tfMessagePtr tf_out = tf::tfMessagePtr(new tf::tfMessage(*tf_in));

        // Split transform frames to separate ground truth frames from optical frames of sensor
        int number_of_transforms = tf_out->transforms.size();
        if(number_of_transforms > 0)
        {
            for(int i = 0; i<number_of_transforms; i++)
            {
              if(tf_out->transforms[i].child_frame_id == ground_truth_frame_name_)
              {
                tf_out->transforms[i].child_frame_id = ground_truth_frame_name_ + "_gt";
              }

              // TODO: parametrize names:
              /*
              if(tf_out->transforms[i].child_frame_id == "/openni_rgb_optical_frame")
              {
                tf_out->transforms[i].child_frame_id = "/camera_rgb_optical_frame";
              }

              if(tf_out->transforms[i].child_frame_id == "/openni_rgb_frame")
              {
                tf_out->transforms[i].child_frame_id = "/camera_rgb_frame";
              }
              */
            }
        }

        // Write to output bag:
        bag_out->write(topic_out_tf_, tf_out->transforms[0].header.stamp, tf_out);
      }
    }
    //--------------------------------------------------------------------------------------------------------------------

  } // BOOST_FOREACH
}

} // end namespace

