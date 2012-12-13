#include "ccny_rgbd/rgbd_util.h"

namespace ccny_rgbd
{

void getTfDifference(const tf::Transform& a, const tf::Transform b, double& dist, double& angle)
{
  tf::Transform motion = a.inverse() * b;
  dist = motion.getOrigin().length();
  float trace = motion.getBasis()[0][0] + motion.getBasis()[1][1] + motion.getBasis()[2][2];
  angle = acos(std::min(1.0, std::max(-1.0, (trace - 1.0)/2.0)));
}

tf::Transform tfFromEigen(Eigen::Matrix4f trans)
{
  MyMatrix btm;
  btm.setValue(trans(0,0),trans(0,1),trans(0,2),
            trans(1,0),trans(1,1),trans(1,2),
            trans(2,0),trans(2,1),trans(2,2));
  tf::Transform ret;
  ret.setOrigin(btVector3(trans(0,3),trans(1,3),trans(2,3)));
  ret.setBasis(btm);
  return ret;
}

Eigen::Matrix4f eigenFromTf(const tf::Transform& tf)
{
   Eigen::Matrix4f out_mat;

   double mv[12];
   tf.getBasis().getOpenGLSubMatrix(mv);

   tf::Vector3 origin = tf.getOrigin();

   out_mat (0, 0) = mv[0]; out_mat (0, 1) = mv[4]; out_mat (0, 2) = mv[8];
   out_mat (1, 0) = mv[1]; out_mat (1, 1) = mv[5]; out_mat (1, 2) = mv[9];
   out_mat (2, 0) = mv[2]; out_mat (2, 1) = mv[6]; out_mat (2, 2) = mv[10];

   out_mat (3, 0) = out_mat (3, 1) = out_mat (3, 2) = 0; out_mat (3, 3) = 1;
   out_mat (0, 3) = origin.x ();
   out_mat (1, 3) = origin.y ();
   out_mat (2, 3) = origin.z ();

   return out_mat;
}

void getXYZRPY(const tf::Transform& t,
                     double& x,    double& y,     double& z,
                     double& roll, double& pitch, double& yaw)
{
  x = t.getOrigin().getX();
  y = t.getOrigin().getY();
  z = t.getOrigin().getZ();

  MyMatrix m(t.getRotation());
  m.getRPY(roll, pitch, yaw);
}

bool tfGreaterThan(const tf::Transform& tf, double dist, double angle)
{
  //TODO: can be optimized for squared length
  double d = tf.getOrigin().length();
  
  if (d >= dist) return true;

  double trace = tf.getBasis()[0][0] + tf.getBasis()[1][1] + tf.getBasis()[2][2];
  double a = acos(std::min(1.0, std::max(-1.0, (trace - 1.0)/2.0)));

  if (a > angle) return true;
  
  return false;
}

void transformToRotationCV( // FIXME: rename it
  const tf::Transform& transform,
  cv::Mat& translation,
  cv::Mat& rotation)
{
  // extract translation
  tf::Vector3 translation_tf = transform.getOrigin();
  translation = cv::Mat(3, 1, CV_64F);
  translation.at<double>(0,0) = translation_tf.getX();
  translation.at<double>(1,0) = translation_tf.getY();
  translation.at<double>(2,0) = translation_tf.getZ();

  // extract rotation
  tf::Matrix3x3 rotation_tf(transform.getRotation());
  rotation = cv::Mat(3, 3, CV_64F);
  for(int i = 0; i < 3; ++i)
  for(int j = 0; j < 3; ++j)     
    rotation.at<double>(j,i) = rotation_tf[j][i];
}

cv::Mat matrixFromRvecTvec(const cv::Mat& rvec, const cv::Mat& tvec)
{
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  return matrixFromRT(rmat, tvec);
}


cv::Mat rmatFromMatrix(const cv::Mat& E)
{
  cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1); // FIXME: it would be better to template the types or not to change at all

  R.at<double>(0,0) = E.at<double>(0,0);
  R.at<double>(0,1) = E.at<double>(0,1);
  R.at<double>(0,2) = E.at<double>(0,2);
  R.at<double>(1,0) = E.at<double>(1,0);
  R.at<double>(1,1) = E.at<double>(1,1);
  R.at<double>(1,2) = E.at<double>(1,2);
  R.at<double>(2,0) = E.at<double>(2,0);
  R.at<double>(2,1) = E.at<double>(2,1);
  R.at<double>(2,2) = E.at<double>(2,2);

  return R;
}

cv::Mat tvecFromMatrix(const cv::Mat& E)
{
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
//  cv::Mat tvec = cv::Mat::zeros(3, 1, E.type()); // FIXME: it would be better to template the types or not to change at all

  tvec.at<double>(0,0) = E.at<double>(0,3);
  tvec.at<double>(1,0) = E.at<double>(1,3);
  tvec.at<double>(2,0) = E.at<double>(2,3);

  return tvec;
}

cv::Mat rvecFromMatrix(const cv::Mat& E)
{
  cv::Mat R = rmatFromMatrix(E);
  cv::Mat rvec;
  cv::Rodrigues(R, rvec);
  return rvec;
}

cv::Mat matrixFromRT(const cv::Mat& rmat, const cv::Mat& tvec)
{   
  cv::Mat E = cv::Mat::zeros(3, 4, CV_64FC1);
  
  E.at<double>(0,0) = rmat.at<double>(0,0);
  E.at<double>(0,1) = rmat.at<double>(0,1);
  E.at<double>(0,2) = rmat.at<double>(0,2);
  E.at<double>(1,0) = rmat.at<double>(1,0);
  E.at<double>(1,1) = rmat.at<double>(1,1);
  E.at<double>(1,2) = rmat.at<double>(1,2);
  E.at<double>(2,0) = rmat.at<double>(2,0);
  E.at<double>(2,1) = rmat.at<double>(2,1);
  E.at<double>(2,2) = rmat.at<double>(2,2);

  E.at<double>(0,3) = tvec.at<double>(0,0);
  E.at<double>(1,3) = tvec.at<double>(1,0);
  E.at<double>(2,3) = tvec.at<double>(2,0);

  return E;
}

cv::Mat m4(const cv::Mat& m3)
{
  cv::Mat m4 = cv::Mat::zeros(4, 4, CV_64FC1);

  for (int i = 0; i < 4; ++i)
  for (int j = 0; j < 3; ++j) 
    m4.at<double>(j,i) = m3.at<double>(j,i);

  m4.at<double>(3,3) = 1.0;
  return m4;
}

double getMsDuration(const ros::WallTime& start)
{
  return (ros::WallTime::now() - start).toSec() * 1000.0;
}

void convert2DPointVectorToMatrix(const std::vector<cv::Point2d> &vector_points, cv::Mat &matrix_points, int type)
{
  matrix_points.create(vector_points.size(), 2, type);
  for(uint row = 0 ; row < vector_points.size() ; row++)
  {
    if(type == CV_64FC1)
    {
      matrix_points.at<double> (row, 0) = (double) vector_points[row].x;
      matrix_points.at<double> (row, 1) = (double) vector_points[row].y;
    }
    if(type == CV_32FC1)
    {
      matrix_points.at<float> (row, 0) = (float) vector_points[row].x;
      matrix_points.at<float> (row, 1) = (float) vector_points[row].y;
    }
    if(type == CV_32SC1)
    {
      matrix_points.at<int> (row, 0) = (int) vector_points[row].x;
      matrix_points.at<int> (row, 0) = (int) vector_points[row].x;
      matrix_points.at<int> (row, 1) = (int) vector_points[row].y;
    }
  }
}

void convert3DPointVectorToMatrix(const std::vector<cv::Point3d> &vector_points, cv::Mat &matrix_points, int type)
{
  matrix_points.create(vector_points.size(), 3, type);
  for(uint row = 0 ; row < vector_points.size() ; row++)
  {
    if(type == CV_64FC1)
    {
      matrix_points.at<double> (row, 0) = (double) vector_points[row].x;
      matrix_points.at<double> (row, 1) = (double) vector_points[row].y;
      matrix_points.at<double> (row, 2) = (double) vector_points[row].z;
    }
    if(type == CV_32FC1)
    {
      matrix_points.at<float> (row, 0) = (float) vector_points[row].x;
      matrix_points.at<float> (row, 1) = (float) vector_points[row].y;
      matrix_points.at<float> (row, 2) = (float) vector_points[row].z;
    }
    if(type == CV_32SC1)
    {
      matrix_points.at<int> (row, 0) = (int) vector_points[row].x;
      matrix_points.at<int> (row, 1) = (int) vector_points[row].y;
      matrix_points.at<int> (row, 2) = (int) vector_points[row].z;
    }

  }
}

void convert2DPointDoubleVectorToFloatVector(const std::vector<cv::Point2d> &double_points, std::vector<cv::Point2f> &float_points)
{
  float_points.clear();
  float_points.resize(double_points.size());
  for(uint row = 0 ; row < double_points.size() ; row++)
  {
    float_points[row].x = (float) double_points[row].x;
    float_points[row].y = (float) double_points[row].y;
  }
}

} //namespace ccny_rgbd
