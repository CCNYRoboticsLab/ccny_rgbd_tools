#include "ccny_rgbd/rgbd_util.h"

namespace ccny_rgbd
{

void getTfDifference(const tf::Transform& motion, double& dist, double& angle)
{
  dist = motion.getOrigin().length();
  double trace = motion.getBasis()[0][0] + motion.getBasis()[1][1] + motion.getBasis()[2][2];
  angle = acos(std::min(1.0, std::max(-1.0, (trace - 1.0)/2.0)));
}

void getTfDifference(const tf::Transform& a, const tf::Transform b, double& dist, double& angle)
{
  tf::Transform motion = a.inverse() * b;
  getTfDifference(motion, dist, angle);
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

void tfToCV(
  const tf::Transform& transform,
  cv::Mat& rotation,
  cv::Mat& translation)
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

void removeInvalidFeatures(
  const Vector3fVector& means,
  const Matrix3fVector& covariances,
  const BoolVector& valid,
  Vector3fVector& means_f,
  Matrix3fVector& covariances_f)
{
  unsigned int size = valid.size(); 
  for(unsigned int i = 0; i < size; ++i)
  {
    if (valid[i])
    {
      const Vector3f& mean = means[i];
      const Matrix3f& cov  = covariances[i];

      means_f.push_back(mean);
      covariances_f.push_back(cov);
    }
  }
}

void cvMatToEigenMatrix3f(
  const cv::Mat mat_cv, 
  Matrix3f& mat_eigen)
{
  for (int j = 0; j < 3; ++j)
  for (int i = 0; i < 3; ++i)
    mat_eigen(j, i) = mat_cv.at<double>(j, i);
}

void cvMatToEigenVector3f(
  const cv::Mat mat_cv, 
  Vector3f& mat_eigen)
{
  for (int j = 0; j < 3; ++j)
    mat_eigen(j, 0) = mat_cv.at<double>(j, 0);
}

void transformDistributions(
  MatVector& means,
  MatVector& covariances,
  const tf::Transform& transform)
{
  cv::Mat R, t;
  tfToCV(transform, R, t);
  cv::Mat Rt = R.t();
  
  unsigned int size = means.size(); 
  for(unsigned int i = 0; i < size; ++i)
  {
    cv::Mat& m = means[i];
    cv::Mat& c = covariances[i];
    m = R * m + t;
    c = R * c * Rt;
  }
}

void tfToEigenRt(
  const tf::Transform& tf, 
  Matrix3f& R, 
  Vector3f& t)
{
   double mv[12];
   tf.getBasis().getOpenGLSubMatrix(mv);

   tf::Vector3 origin = tf.getOrigin();

   R(0, 0) = mv[0]; R(0, 1) = mv[4]; R(0, 2) = mv[8];
   R(1, 0) = mv[1]; R(1, 1) = mv[5]; R(1, 2) = mv[9];
   R(2, 0) = mv[2]; R(2, 1) = mv[6]; R(2, 2) = mv[10];

   t(0, 0) = origin.x();
   t(1, 0) = origin.y();
   t(2, 0) = origin.z();
}

void transformDistributions(
  Vector3fVector& means,
  Matrix3fVector& covariances,
  const tf::Transform& transform)
{
  Matrix3f R;
  Vector3f t;
  tfToEigenRt(transform, R, t);
  Matrix3f R_T = R.transpose();
  
  unsigned int size = means.size(); 
  for(unsigned int i = 0; i < size; ++i)
  {
    Vector3f& m = means[i];
    Matrix3f& c = covariances[i];
    m = R * m + t;
    c = R * c * R_T;
  }
}

void getPointCloudFromDistributions(
  const MatVector& means,
  PointCloudFeature& cloud)
{
  unsigned int size = means.size(); 
  cloud.points.resize(size);
  for(unsigned int i = 0; i < size; ++i)
  {
    const cv::Mat& m = means[i];
    PointFeature& p = cloud.points[i];

    p.x = m.at<double>(0,0);
    p.y = m.at<double>(1,0);
    p.z = m.at<double>(2,0);
  }
}

void getPointCloudFromDistributions(
  const Vector3fVector& means,
  PointCloudFeature& cloud)
{
  unsigned int size = means.size(); 
  cloud.points.resize(size);
  for(unsigned int i = 0; i < size; ++i)
  {
    const Vector3f& m = means[i];
    PointFeature& p = cloud.points[i];

    p.x = m(0,0);
    p.y = m(1,0);
    p.z = m(2,0);
  }
}

} //namespace ccny_rgbd
