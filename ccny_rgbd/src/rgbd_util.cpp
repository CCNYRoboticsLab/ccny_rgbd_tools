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
  tf::Matrix3x3 btm;
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

void getXYZRPY(
  const tf::Transform& t,
  double& x,    double& y,     double& z,
  double& roll, double& pitch, double& yaw)
{
  x = t.getOrigin().getX();
  y = t.getOrigin().getY();
  z = t.getOrigin().getZ();

  tf::Matrix3x3  m(t.getRotation());
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

double getMsDuration(const ros::WallTime& start)
{
  return (ros::WallTime::now() - start).toSec() * 1000.0;
}

void removeInvalidMeans(
  const Vector3fVector& means,
  const BoolVector& valid,
  Vector3fVector& means_f)
{
  unsigned int size = valid.size(); 
  for(unsigned int i = 0; i < size; ++i)
  {
    if (valid[i])
    {
      const Vector3f& mean = means[i];
      means_f.push_back(mean);
    }
  }
}

void removeInvalidDistributions(
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

void transformMeans(
  Vector3fVector& means,
  const tf::Transform& transform)
{
  Matrix3f R;
  Vector3f t;
  tfToEigenRt(transform, R, t);
  
  unsigned int size = means.size(); 
  for(unsigned int i = 0; i < size; ++i)
  {
    Vector3f& m = means[i];
    m = R * m + t;
  }  
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

void pointCloudFromMeans(
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
  
  cloud.height = 1;
  cloud.width = size;
  cloud.is_dense = true;
}

} //namespace ccny_rgbd
