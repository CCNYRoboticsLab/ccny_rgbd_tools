/**
 *  @file keyframe_graph_solver.h
 *  @author Ivan Dryanovski <ivan.dryanovski@gmail.com>
 * 
 *  @section LICENSE
 * 
 *  Copyright (C) 2013, City University of New York
 *  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
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

#ifndef RGBDTOOLS_KEYFRAME_GRAPH_SOLVER_H
#define RGBDTOOLS_KEYFRAME_GRAPH_SOLVER_H

#include "rgbdtools/rgbd_keyframe.h"
#include "rgbdtools/graph/keyframe_association.h"

namespace rgbdtools {

/** @brief Base class for graph-based global alignment classes
 * 
 * The class takes as input a vector of RGBD keyframes and a 
 * vector of associations between them, and modifies the keyframe
 * poses based on the minimization of some error metric.
 */
class KeyframeGraphSolver
{
  public:

    /** @brief Constructor
     */  
    KeyframeGraphSolver();
    
    /** @brief Default destructor
     */
    virtual ~KeyframeGraphSolver();
 
    /** @brief Main method to call to perform graph solving.
     * 
     * The keyframe poses will be modified according to some error 
     * propagation model informed by the graph defined by the keyframe
     * associations.
     * 
     * @param keyframes vector of keyframes
     * @param associations vector of input keyframe associations
     */
    virtual void solve(
  KeyframeVector& keyframes,
  const KeyframeAssociationVector& associations,
  const KeyframeAssociationVector& odometryEdges)=0;
      
    virtual void solve(
      KeyframeVector& keyframes,
      const KeyframeAssociationVector& associations,
      AffineTransformVector& path) = 0;
};

} // namespace rgbdtools

#endif // RGBDTOOLS_KEYFRAME_GRAPH_SOLVER_H
