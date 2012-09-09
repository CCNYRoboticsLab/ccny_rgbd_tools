#ifndef CCNY_GICP_GICP_ALIGN_H
#define CCNY_GICP_GICP_ALIGN_H

#include <vector>
#include <stdint.h>

#include <boost/make_shared.hpp>

#include <tf/transform_datatypes.h>

//#include <pcl/common/transform.h>

#include "ccny_gicp/types.h"
#include "ccny_gicp/transform.h"
#include "ccny_gicp/optimize.h"

namespace ccny_gicp {

class GICPAlign
{
  public:

    //typedef pcl::octree::OctreePointCloudStorage<GICPPoint> Octree;

    GICPAlign();

    void align();
    void align(Eigen::Matrix4f& tf);
    inline void setData (GICPPointSet * data)  { data_  = data; }
    inline void setModel(GICPPointSet * model) { model_ = model; }

    void setParams(const GICPParams& params);

    inline void setUseColor(bool use_color) { use_color_ = use_color;}
    inline void setMaxColorDiff(double max_color_diff)
    {
      max_color_diff_ = max_color_diff;
      max_color_diff_sq_ = max_color_diff_ * max_color_diff_;
    }

    int getFinalIterations() const { return iterations_; }

  private:

    GICPPointSet * data_;  // p1
    GICPPointSet * model_; // p2

    GICPParams params_;
  
    bool use_color_;
    double max_color_diff_;

    double max_color_diff_sq_; // derived from max_color_diff_
    double max_distance_sq_;   // derived from params

    int iterations_; // how many iterations it took to converge

    void alignScan(dgc_transform_t base_t, dgc_transform_t t);

    double getColorDiff(float c1, float c2);

    void GICP2Eigen(const dgc_transform_t g_m, Eigen::Matrix4f& m)
    {
      for(int i=0;i<4; i++)
      for(int j=0;j<4; j++)
        m(i,j) = g_m[i][j];
    }
};

} //namespace ccny_gicp

#endif // CCNY_GICP_GICP_ALIGN_H
