#include "ccny_rgbd/features/canny_detector.h"

namespace ccny_rgbd
{

CannyDetector::CannyDetector():
  threshold1_(60),
  threshold2_(60)
{

}

CannyDetector::~CannyDetector()
{

}

void CannyDetector::findFeatures(RGBDFrame& frame, const cv::Mat * input_img)
{
  // **** canny edge

  cv::Mat can_img(input_img->rows, input_img->cols, CV_8UC1);
  cv::Canny(*input_img, can_img, threshold1_, threshold2_, 3);

  //cv::imshow ("can image", can_img);
  //cv::waitKey (10);

  // ****** extract gradient angle

  cv::Mat sob_x_img(input_img->rows, input_img->cols, CV_8U);
  cv::Mat sob_y_img(input_img->rows, input_img->cols, CV_8U);

  if (compute_descriptors_)
  {
    cv::Sobel(*input_img, sob_x_img, CV_8U, 1, 0, 3, 1, 127);
    cv::Sobel(*input_img, sob_y_img, CV_8U, 0, 1, 3, 1, 127);
  }

  img_angles_ = cv::Mat::zeros(input_img->rows, input_img->cols, CV_8U);

  // **** filter

  for(int j = 0; j < can_img.rows; ++j)
  for(int i = 0; i < can_img.cols; ++i)
  {
    img_angles_.at<uint8_t>(j, i) = 255;

    if(can_img.at<uint8_t>(j,i) != 0)
    {
      unsigned int index = j * input_img->cols + i;

      float angle = 0.0;
      if (compute_descriptors_)
      {
        // **** calculate angle
        uint8_t u_sx = sob_x_img.at<uint8_t>(j, i);
        uint8_t u_sy = sob_y_img.at<uint8_t>(j, i);

        float sx = u_sx - 127.0;
        float sy = u_sy - 127.0;

        angle = atan2(sy, sx);             // in radians, [-pi, pi]

        if (angle < 0) angle += (2.0 * 3.14159);    // in radians [0.0, 2pi]

        uint8_t uangle = (uint8_t) (angle / (2.0 * 3.14159) * 180); // [0, 180]

        img_angles_.at<uint8_t>(j, i) = uangle; // for visualization
      }
      else
      {
        img_angles_.at<uint8_t>(j, i) = 0; // for visualization
      }

      // **** filter using z data ****************************************
      // Neighbors := cross (window_*2+1, window*2 +1)
      // If any of the neighbors are closer and the z jump is big enough,
      // use them instead.

      double z = frame.data.points[index].z;

      if (!isnan(z) && z < max_range_)
      {
        int corr_index = index;

        for (int c = 1; c <= window_; ++c)
        {
          if(i-c > 0)
            if (testPixel(*input_img, frame.data, z, i-c, j, corr_index)) break;
          if(i+c < input_img->cols)
            if (testPixel(*input_img, frame.data, z, i+c, j, corr_index)) break;
          if(j-c > 0)
            if (testPixel(*input_img, frame.data, z, i, j-c, corr_index)) break;
          if(j+c < input_img->rows)
            if (testPixel(*input_img, frame.data, z, i, j+c, corr_index)) break;
        }

        PointFeature p;
        p.x = frame.data.points[corr_index].x;
        p.y = frame.data.points[corr_index].y;
        p.z = frame.data.points[corr_index].z;

        frame.rgb_features.points.push_back(p);
      }
    }
  }

  frame.rgb_features.header = frame.data.header;
  frame.rgb_features.height = 1;
  frame.rgb_features.width = frame.rgb_features.points.size();
  frame.rgb_features.is_dense = true;
}

/*
void CannyDetector::showGradientImage()
{
  cv::Mat img_hsv = cv::Mat::zeros(img_angles_.rows, img_angles_.cols, CV_8UC3);
  cv::Mat img_rgb = cv::Mat::zeros(img_angles_.rows, img_angles_.cols, CV_8UC3);

  for(int j = 0; j < img_angles_.rows; ++j)
  for(int i = 0; i < img_angles_.cols; ++i)
  {
    if (img_angles_.at<uint8_t>(j, i) != 255)
    {
      int hue = img_angles_.at<uint8_t>(j, i);

      //if (hue < 0 || hue >= 180) printf("[[[[[[ HUE: %d ]]]]]]]\n", hue);

      img_hsv.at<cv::Vec3b>(j, i)[0] = hue;
      img_hsv.at<cv::Vec3b>(j, i)[1] = 255;
      img_hsv.at<cv::Vec3b>(j, i)[2] = 255;
    }
    else
    {
      img_hsv.at<cv::Vec3b>(j, i)[0] = 0;
      img_hsv.at<cv::Vec3b>(j, i)[1] = 0;
      img_hsv.at<cv::Vec3b>(j, i)[2] = 0;
    }
  }

  cv::cvtColor (img_hsv, img_rgb, CV_HSV2BGR);

  std::string window_name = "Canny Gradient Orientations";

  cv::namedWindow(window_name, CV_WINDOW_NORMAL);
  cv::imshow (window_name, img_rgb);
  cv::waitKey (10);
}
*/

void CannyDetector::setThreshold1(int threshold1)
{
  threshold1_ = threshold1;
}

int CannyDetector::getThreshold1() const
{
  return threshold1_;
}

void CannyDetector::setThreshold2(int threshold2)
{
  threshold2_ = threshold2;
}

int CannyDetector::getThreshold2() const
{
  return threshold2_;
}

} //namespace
