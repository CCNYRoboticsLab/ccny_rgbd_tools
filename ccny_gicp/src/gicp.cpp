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

#include "ccny_gicp/gicp.h"

using namespace std;//TODO: remove

namespace ccny_gicp {

GICPPointSet::GICPPointSet():
  debug_(false),
  gicp_epsilon_(1e-3),
  nn_normal_count_(20)
{
  cloud_ = boost::shared_ptr<PointCloudGICP> (new PointCloudGICP());
}

GICPPointSet::~GICPPointSet()
{

}
    
void GICPPointSet::Clear(void) 
{
  cloud_->points.clear();
}
   
void GICPPointSet::computeMatrices()
{
  int n = NumPoints();

  IntVector nn_indices;
  FloatVector nn_distances_sq;

  nn_indices.resize(nn_normal_count_);
  nn_distances_sq.resize(nn_normal_count_);
 
  for(int index = 0; index < n; ++index) 
    computeMatrix(index, nn_indices, nn_distances_sq);
}

void GICPPointSet::computeMatrix(int index)
{
  IntVector nn_indices;
  FloatVector nn_distances_sq;

  nn_indices.resize(nn_normal_count_);
  nn_distances_sq.resize(nn_normal_count_);

  computeMatrix(index, nn_indices, nn_distances_sq);
}

void GICPPointSet::computeMatrix(int index, IntVector& nn_indices, FloatVector& nn_distances_sq)
{
  double mean[3];

  gsl_vector *work          = gsl_vector_alloc(3);
  gsl_vector *gsl_singulars = gsl_vector_alloc(3);
  gsl_matrix *gsl_v_mat     = gsl_matrix_alloc(3, 3);

  gicp_mat_t &cov = cloud_->points[index].C;
  
  // zero out the cov and mean
  for(int k = 0; k < 3; k++) 
  {
    mean[k] = 0.;
    for(int l = 0; l < 3; l++) 
    {
      cov[k][l] = 0.;
    }
  }

  int n_found = getNN(cloud_->points[index], nn_normal_count_, nn_indices, nn_distances_sq);

  assert(n_found != 0);

  // find the covariance matrix
  for(int j = 0; j < n_found; j++) 
  {
    PointGICP &pt = cloud_->points[nn_indices[j]];
    
    mean[0] += pt.x;
    mean[1] += pt.y;
    mean[2] += pt.z;

    cov[0][0] += pt.x*pt.x;
    
    cov[1][0] += pt.y*pt.x;
    cov[1][1] += pt.y*pt.y;
    
    cov[2][0] += pt.z*pt.x;
    cov[2][1] += pt.z*pt.y;
    cov[2][2] += pt.z*pt.z;	  
  }

  mean[0] /= (double)n_found;
  mean[1] /= (double)n_found;
  mean[2] /= (double)n_found;

  // get the actual covariance
  for(int k = 0; k < 3; k++) 
  {
    for(int l = 0; l <= k; l++) 
    {
      cov[k][l] /= (double)n_found;
      cov[k][l] -= mean[k]*mean[l];
      cov[l][k] = cov[k][l];
    }
  }

  // compute the SVD
  gsl_matrix_view gsl_cov = gsl_matrix_view_array(&cov[0][0], 3, 3);
  gsl_linalg_SV_decomp(&gsl_cov.matrix, gsl_v_mat, gsl_singulars, work);

  // zero out the cov matrix, since we know U = V since C is symmetric
  for(int k = 0; k < 3; k++) 
  for(int l = 0; l < 3; l++) 
    cov[k][l] = 0;

  // reconstitute the covariance matrix with modified singular values using the column vectors in V.
  for(int k = 0; k < 3; k++) 
  {
    gsl_vector_view col = gsl_matrix_column(gsl_v_mat, k);

    double v = 1.; // biggest 2 singular values replaced by 1
    if(k == 2)    // smallest singular value replaced by gicp_epsilon
      v = gicp_epsilon_;
         
    gsl_blas_dger(v, &col.vector, &col.vector, &gsl_cov.matrix); 
  }

  gsl_matrix_free(gsl_v_mat);
  gsl_vector_free(gsl_singulars);
  gsl_vector_free(work);
}

void GICPPointSet::SavePoints(const char *filename) 
{
  ofstream out(filename);
  
  if(out) 
  {
    int n = NumPoints();

    for(int i = 0; i < n; i++) 
      out << cloud_->points[i].x << "\t" << cloud_->points[i].y << "\t" << cloud_->points[i].z << endl;
  }
  out.close();
}
    
void GICPPointSet::SaveMatrices(const char *filename) 
{
  ofstream out(filename);
  
  if(out) 
  {
    int n = NumPoints();

    for(int i = 0; i < n; i++) 
    {
      for(int k = 0; k < 3; k++) 
      {
        for(int l = 0; l < 3; l++) 
        {
          out << cloud_->points[i].C[k][l] << "\t";
        }
      }
      out << endl;
    }
  }

  out.close();      
}    

} //namespace ccny_gicp

