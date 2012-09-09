#include "ccny_rgbd/features/klt_detector.h"

namespace ccny_rgbd
{

KltDetector::KltDetector(ros::NodeHandle nh, ros::NodeHandle nh_private):
  FeatureDetector(nh, nh_private),
  initialized_(false),
  have_points_(false),
  n_features_(100),
  reseed_threshold_(0.85),
  win_size_(15)
{
  //gft_params_.maxCorners = n_features_;

  gft_detector_ = new cv::GoodFeaturesToTrackDetector(n_features_);

  pointsA_     = new CvPoint2D32f[n_features_];
  points_good_ = new CvPoint2D32f[n_features_];
}

KltDetector::~KltDetector()
{
  delete gft_detector_;
}

void KltDetector::findFeatures(RGBDFrame& frame, const cv::Mat * input_img)
{
  CvPoint2D32f * pointsB = new CvPoint2D32f[n_features_]; //TODO
	char  features_found [n_features_];
	float feature_errors [n_features_];

  if (!initialized_)
  {
    CvSize pyr_sz = cvSize(input_img->cols+8, input_img->rows/3);
	  pyrA = cvCreateImage(pyr_sz, IPL_DEPTH_32F, 1);
	  pyrB = cvCreateImage(pyr_sz, IPL_DEPTH_32F, 1);
    initialized_ = true;
  }    

  if (!have_points_)
  {
    // no (or not enough) previous points, seed using GFT
    //printf("Initializing using GFT\n");

    gft_detector_->detect(*input_img, frame.keypoints);

    // create C-stlye points from keypoints
    for(int i = 0; i < n_features_; ++i)
    {
      pointsA_[i].x = frame.keypoints[i].pt.x;
      pointsA_[i].y = frame.keypoints[i].pt.y;
    }

    have_points_ = true; 
  }
  else
  {   
    IplImage imgA = IplImage(prev_input_img_);
    IplImage imgB = IplImage(*input_img);

	  cvCalcOpticalFlowPyrLK( 
      &imgA, &imgB, pyrA, pyrB,pointsA_, pointsB, n_features_, 
		  cvSize(win_size_, win_size_), 5, features_found, feature_errors,
		  cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3), 0);

    // identify how many good points we have, and copy them
    int n_good_points = 0;
    for(int i = 0; i < n_features_; ++i)
    {
      if (pointsB[i].x < 0 || pointsB[i].x >= input_img->cols ||
          pointsB[i].y < 0 || pointsB[i].y >= input_img->rows)
        continue;

      points_good_[i] = pointsB[i];
      n_good_points++;
    }
    //printf("%d good points found\n", n_good_points);

    // check if we have enough points - if not, reseed needed
    if (n_good_points < reseed_threshold_ * n_features_)
      have_points_ = false;

    // copy over good points
    frame.keypoints.clear();
    frame.keypoints.resize(n_good_points);
    for(int i = 0; i < n_good_points; ++i)
    {
      cv::KeyPoint kp;
      kp.pt.x = points_good_[i].x;
      kp.pt.y = points_good_[i].y;
      frame.keypoints.push_back(kp);
    }

    // rotate data
    memcpy(pointsA_, pointsB, n_features_ * sizeof(CvPoint2D32f));
    delete [] pointsB;
  }

  //printf("rotate image\n");
  prev_input_img_ = *input_img;

  /*
  cv::namedWindow("GFT features", CV_WINDOW_NORMAL);
  cv::Mat kp_img(input_img->rows, input_img->cols, CV_8UC1);
  cv::drawKeypoints(*input_img, frame.keypoints, kp_img);
  cv::imshow ("GFT features", kp_img);
  cv::waitKey (10);
  */
}

} // namespace ccny_rgbd
