/*************************************************************
  Generalized-ICP Copyright (c) 2009 Aleksandr Segal.
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

* Redistributions of source code must retain the above 
  copyright notice, this list of conditions and the 
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
*************************************************************/

#ifndef CCNY_GICP_GICP_H
#define CCNY_GICP_GICP_H

#include <vector>
#include <fstream>
#include <stdint.h>
#include <sstream>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>

#include <boost/make_shared.hpp>

#include "ccny_gicp/types.h"
#include "ccny_gicp/transform.h"
#include "ccny_gicp/optimize.h"

namespace ccny_gicp {

class GICPPointSet 
{
  public:

    GICPPointSet();
    ~GICPPointSet();
   
    void SavePoints(const char *filename);
    void SaveMatrices(const char *filename);
    
    int NumPoints() { return (int)cloud_->points.size(); }
    void Clear(void);
    int Size() { return cloud_->points.size(); }
    inline void AppendPoint(PointGICP const & pt) { cloud_->points.push_back(pt); }

    void SetGICPEpsilon(double eps) { gicp_epsilon_ = eps; }
    void SetDebug(bool d) { debug_ = d; }

    PointGICP & operator[](int i) { return cloud_->points[i]; }
    PointGICP const& operator[](int i) const { return cloud_->points[i]; }
    
    inline void setCloud(PointCloudGICP::Ptr cloud) { cloud_ = cloud; }
    inline PointCloudGICP::Ptr getCloud() const { return cloud_; }

    void computeMatrices();
    void computeMatrix(int index);

    inline void setNormalCount(int nn_normal_count) { nn_normal_count_ = nn_normal_count; }

    virtual int getNN(const PointGICP p, int k, IntVector& nn_indices, FloatVector& nn_distances_sq) = 0;

  protected:

    PointCloudGICP::Ptr cloud_;

    bool debug_;
    double gicp_epsilon_;
    int nn_normal_count_;

    pthread_mutex_t mutex_;

    void computeMatrix(int index, IntVector& nn_indices, FloatVector& nn_distances_sq);
};  

class GICPPointSetKd: public GICPPointSet
{
  public:
 
    inline void setKdTree(KdTree * kdtree) { kdtree_ = kdtree; }
  
    inline int getNN(const PointGICP p, int k, IntVector& nn_indices, FloatVector& nn_distances_sq)
    {
      return kdtree_->nearestKSearch(p, k, nn_indices, nn_distances_sq);
    }

  private:

    KdTree * kdtree_;
};


class GICPPointSetOctree: public GICPPointSet
{
  public:
  
    inline void setOctree(Octree * octree) { octree_ = octree; }
  
    inline int getNN(const PointGICP p, int k, IntVector& nn_indices, FloatVector& nn_distances_sq)
    {
      return octree_->nearestKSearch(p, k, nn_indices, nn_distances_sq);
    }

  private:

    Octree * octree_;
};


} //namespace ccny_gicp

#endif // CCNY_GICP_GICP_H
