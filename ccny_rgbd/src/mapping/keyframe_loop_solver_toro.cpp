#include "ccny_rgbd/mapping/keyframe_loop_solver_toro.h"

namespace ccny_rgbd
{

KeyframeLoopSolverTORO::KeyframeLoopSolverTORO(ros::NodeHandle nh, ros::NodeHandle nh_private):
  KeyframeLoopSolver(nh, nh_private)
{

}

KeyframeLoopSolverTORO::~KeyframeLoopSolverTORO()
{

}

void KeyframeLoopSolverTORO::solve(
  KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{
  int treeType = 1 ;
  bool ignorePreconditioner = false;
  bool adaptiveRestart = false;
  int loop_iterations = 50;

  AISNavigation::TreeOptimizer3::EdgeCompareMode compareMode =
    AISNavigation::EVComparator<AISNavigation::TreeOptimizer3::Edge*>::CompareLevel;

  AISNavigation::TreeOptimizer3 pg;

  pg.verboseLevel = 0;
  pg.restartOnDivergence = adaptiveRestart;

  // construct vertices

  for (unsigned int i = 0; i < keyframes.size(); ++i)
  {
    double x, y, z, roll, pitch, yaw;
    getXYZRPY(keyframes[i].pose, x, y, z, roll, pitch, yaw);

    AISNavigation::Pose3<double> v_pose(x, y, z, roll, pitch, yaw);

    AISNavigation::TreePoseGraph3::Vertex* v = pg.addVertex(i, v_pose);
    if (v)
      v->transformation = AISNavigation::TreePoseGraph3::Transformation(v_pose);
    else
      printf("error in vertex insertion for keyframe %d\n", i);  
  }

  // construct edges

  for (unsigned int i = 0; i < associations.size(); ++i)
  {
    const KeyframeAssociation& association = associations[i];

    double x, y, z, roll, pitch, yaw;

    // TODO: which one a2b or b2a?
    getXYZRPY(association.a2b, x, y, z, roll, pitch, yaw);

    AISNavigation::Transformation3<double> t(x, y, z, roll, pitch, yaw);

    DMatrix<double> inf(6,6);
    inf = DMatrix<double>::I(6);

    bool result = pg.addEdge(pg.vertex(association.kf_idx_a),
                             pg.vertex(association.kf_idx_b),
                             t, inf);

    if(result)
      printf("Successful vertex insertion from %d to %d\n", 
        association.kf_idx_a, association.kf_idx_b);
    else
      printf("Fatal, attempting to insert an edge between non existing nodes %d and %d\n",
        association.kf_idx_a, association.kf_idx_b);
  }

  ROS_INFO("#nodes: %d #edges %d", (int)pg.vertices.size(), (int)pg.edges.size());

  if (treeType == 0)
    pg.buildSimpleTree();
  else
    pg.buildMST(pg.vertices.begin()->first);

  pg.initializeOnTree();
  pg.initializeTreeParameters();
  pg.initializeOptimization(compareMode);

  /*
  double l = pg.totalPathLength();
  int nEdges = pg.edges.size();
  double apl=l/(double)(nEdges);
  printf("Average path length = %f\n", apl);
  printf("Complexity of an iteration = %f\n", l);
  */

  // optimize

  for (int i=0; i < loop_iterations; i++)
  {
     pg.iterate(0, ignorePreconditioner);

     // compute the error and dump it
     double mte, mre, are, ate;
     double error = pg.error(&mre, &mte, &are, &ate);
     //printf("iteration %d RotGain = %f\n", i, pg.getRotGain());
     printf("\tLoop iteration[%d] global error = %f\n", i, error);
  }

  // update transformations from poses?
  for (AISNavigation::TreePoseGraph3::VertexMap::iterator it = pg.vertices.begin();
      it != pg.vertices.end();
      it++)
  {
    AISNavigation::TreePoseGraph3::Vertex * v = it->second;
    v->pose = v->transformation.toPoseType();
  }

  for (unsigned int i = 0; i < keyframes.size(); ++i)
  {
    AISNavigation::Pose3<double> pose = pg.vertex(i)->pose;

    tf::Vector3 v(pose.x(), pose.y(), pose.z());
    tf::Quaternion q = tf::createQuaternionFromRPY(pose.roll(), pose.pitch(), pose.yaw());

    tf::Transform t;
    t.setOrigin(v);
    t.setRotation(q);

    keyframes[i].pose = t;
  }

  for (unsigned int i = 0; i < associations.size(); ++i)
  {
    //edges_[i].tf_computed = false;
    //RGBDFrame& a = (*keyframes_)[edges_[i].index_a];
    //RGBDFrame& b = (*keyframes_)[edges_[i].index_b];

    //edges_[i].a2b = a.pose.inverse() * b.pose;
  }
}

} // namespace ccny_rgbd
