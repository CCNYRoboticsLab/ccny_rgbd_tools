/*
 *  Copyright (C) 2013, City University of New York
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  CCNY Robotics Lab
 *  http://robotics.ccny.cuny.edu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CCNY_RGBD_KEYFRAME_GRAPH_SOLVER_G2O_H
#define CCNY_RGBD_KEYFRAME_GRAPH_SOLVER_G2O_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <g2o/core/graph_optimizer_sparse.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/types_six_dof_quat.h>

#include "ccny_rgbd/mapping/keyframe_graph_solver.h"

namespace ccny_rgbd
{

class KeyframeGraphSolverG2O: public KeyframeGraphSolver
{
  public:

    KeyframeGraphSolverG2O(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~KeyframeGraphSolverG2O();
 
    void solve(KeyframeVector& keyframes,
               KeyframeAssociationVector& associations);

  private:

    int vertexIdx;
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    g2o::BlockSolverX * solver_ptr;
        
    void addVertex(
      const Eigen::Matrix4f& vertex_pose,
      int vertex_idx);
    
    void addEdge(
      int from_idx,
      int to_idx,
      const Eigen::Matrix4f& relative_pose,
      const Eigen::Matrix<double,6,6>& information_matrix);
    
    void optimizeGraph();
    
    void updatePoses(KeyframeVector& keyframes);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_GRAPH_SOLVER_G2O_H
