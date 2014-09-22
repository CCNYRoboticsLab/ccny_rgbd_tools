/**
 *  @file rgbdtools.h
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

#ifndef RGBDTOOLS_RGBDTOOLS_H
#define RGBDTOOLS_RGBDTOOLS_H

#include "rgbdtools/types.h"
#include "rgbdtools/rgbd_util.h"
#include "rgbdtools/map_util.h"

#include "rgbdtools/header.h"
#include "rgbdtools/rgbd_frame.h"
#include "rgbdtools/rgbd_keyframe.h"

#include "rgbdtools/features/feature_detector.h"
#include "rgbdtools/features/gft_detector.h"
#include "rgbdtools/features/orb_detector.h"
#include "rgbdtools/features/star_detector.h"
#include "rgbdtools/features/surf_detector.h"

#include "rgbdtools/registration/motion_estimation.h"
#include "rgbdtools/registration/motion_estimation_icp_prob_model.h"

#include "rgbdtools/graph/keyframe_association.h"
#include "rgbdtools/graph/keyframe_graph_detector.h"
#include "rgbdtools/graph/keyframe_graph_solver.h"
#include "rgbdtools/graph/keyframe_graph_solver_g2o.h"

#endif // RGBDTOOLS_RGBDTOOLS_H
