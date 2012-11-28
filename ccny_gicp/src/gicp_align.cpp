#include "ccny_gicp/gicp_align.h"

using namespace std;//TODO: remove

namespace ccny_gicp {

GICPAlign::GICPAlign()
{
  params_.max_distance = 5.0;
  params_.solve_rotation = true;
  params_.debug = true;
  params_.max_iteration = 100;
  params_.max_iteration_inner = 8;
  params_.epsilon = 5e-4,
  params_.epsilon_rot = 2e-3;
  use_color_ = true;
  max_color_diff_ = 50;

  max_color_diff_sq_ = max_color_diff_ * max_color_diff_;

  max_distance_sq_ = params_.max_distance * params_.max_distance;
}

void GICPAlign::setParams(const GICPParams& params)
{
  params_ = params;
  max_distance_sq_ = params_.max_distance * params_.max_distance;
}

// assumes trees and matrices are ready
void GICPAlign::align()
{
  Eigen::Matrix4f tf;
  return align(tf);
}

// assumes trees and matrices are ready
void GICPAlign::align(Eigen::Matrix4f& tf)
{
  // set up the transformations

  dgc_transform_t t_base, t1;
  dgc_transform_identity(t_base);
  dgc_transform_identity(t1);

  // align using gicp

  alignScan(t_base, t1);

  if (params_.debug)
  {
    // print the result
    dgc_transform_print(t_base, "t_base");
    dgc_transform_print(t1, "t1");
  }

  // compute final transform

  double tx, ty, tz, rx, ry, rz;
  dgc_transform_get_translation(t1, &tx, &ty, &tz);
  dgc_transform_get_rotation(t1, &rx, &ry, &rz);

  GICP2Eigen(t1, tf); 
}

void GICPAlign::alignScan(dgc_transform_t base_t, dgc_transform_t t)
{
  int num_matches = 0;
  int n = data_->NumPoints();
  double delta = 0.;
  dgc_transform_t t_last;

  std::vector<int> corr_indices;
  std::vector<float> corr_distances_sq;

  corr_indices.resize(n);
  corr_distances_sq.resize(n);

  double query_point[3];
     
  gicp_mat_t * mahalanobis = new gicp_mat_t[n];
  gsl_matrix * gsl_R = gsl_matrix_alloc(3, 3);
  gsl_matrix * gsl_temp = gsl_matrix_alloc(3, 3);

  bool converged = false;
  bool opt_status = false;
  iterations_ = 0;

  // set up the optimization parameters
  GICPOptData opt_data;
  opt_data.corr_indices = &corr_indices;
  opt_data.p1 = data_;
  opt_data.p2 = model_;
  opt_data.M = mahalanobis;
  opt_data.solve_rotation = params_.solve_rotation;
  dgc_transform_copy(opt_data.base_t, base_t);
  
  GICPOptimizer opt;
  opt.SetDebug(params_.debug);
  opt.SetMaxIterations(params_.max_iteration_inner);

  // set up the mahalanobis matricies
  // these are identity for now to ease debugging
  for(int i = 0; i < n; i++) 
  for(int k = 0; k < 3; k++) 
  for(int l = 0; l < 3; l++) 
    mahalanobis[i][k][l] = (k == l)?1:0.;
  
  if(params_.debug)
  {
    dgc_transform_write(base_t, "t_base.tfm");
    dgc_transform_write(t, "t_0.tfm");
  }

  std::vector<int> nn_indices;
  std::vector<float> nn_distances_sq;
  nn_indices.resize(1);
  nn_distances_sq.resize(1);

  while(!converged) 
  {
    dgc_transform_t transform_R;
    dgc_transform_copy(transform_R, base_t);
    dgc_transform_left_multiply(transform_R, t);

    // copy the rotation component of the current total transformation (including base), into a gsl matrix
    for(int i = 0; i < 3; i++) 
    for(int j = 0; j < 3; j++) 
      gsl_matrix_set(gsl_R, i, j, transform_R[i][j]);

    // find correpondences
    num_matches = 0;

    int color_discarded = 0;

    for (int i = 0; i < n; i++) 
    {   
      query_point[0] = (*data_)[i].x;
      query_point[1] = (*data_)[i].y;
      query_point[2] = (*data_)[i].z;
      
      dgc_transform_point(&query_point[0], &query_point[1], &query_point[2], base_t);
      dgc_transform_point(&query_point[0], &query_point[1], &query_point[2], t);
      
      PointGICP p;
      p.x = query_point[0];
      p.y = query_point[1];
      p.z = query_point[2];

      int n_found = model_->getNN(p, 1, nn_indices, nn_distances_sq);
      //int n_found = model_octree_.nearestKSearch(p, 1, nn_indices, nn_distances_sq);
     
      bool have_match = false;
      int match;

      // distance metric

      if (n_found == 1 && nn_distances_sq[0] < max_distance_sq_)
      {
        have_match = true;
        match = nn_indices[0];
      }

      // color correspondence discarding

      if (have_match && use_color_)
      {
        double color_diff = getColorDiff((*data_)[i].rgb, (*model_)[match].rgb);

        if (color_diff >= max_color_diff_sq_)
        {
          ++color_discarded;
          have_match = false;
        }
      }

      if (have_match)
      {
        corr_indices[i] = match; // match

        // set up the updated mahalanobis matrix here
        gsl_matrix_view C1 = gsl_matrix_view_array(&(*data_ )[i].C[0][0], 3, 3);
        gsl_matrix_view C2 = gsl_matrix_view_array(&(*model_)[match].C[0][0], 3, 3);
        gsl_matrix_view M = gsl_matrix_view_array(&mahalanobis[i][0][0], 3, 3);
        gsl_matrix_set_zero(&M.matrix);	    
        gsl_matrix_set_zero(gsl_temp);

        // M = R*C1  // using M as a temp variable here
        gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1., gsl_R, &C1.matrix, 1., &M.matrix);
        
        // temp = M*R' // move the temp value to 'temp' here
        gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1., &M.matrix, gsl_R, 0., gsl_temp);
        
        // temp += C2
        gsl_matrix_add(gsl_temp, &C2.matrix);
        // at this point temp = C2 + R*C1*R'
        
        // now invert temp to get the mahalanobis distance metric for gicp
        // M = temp^-1
        gsl_matrix_set_identity(&M.matrix); 
        gsl_linalg_cholesky_decomp(gsl_temp);

        for(int k = 0; k < 3; k++) 
          gsl_linalg_cholesky_svx(gsl_temp, &gsl_matrix_row(&M.matrix, k).vector);
        num_matches++;
      }
      else 
      {
        corr_indices[i] = -1; // no match
      }

      //printf ("\t[%d %d]\n", num_matches, color_discarded);
    }
     
    opt_data.num_matches = num_matches;

    // optimize transformation using the current assignment and Mahalanobis metrics
    dgc_transform_copy(t_last, t);
    opt_status = opt.Optimize(t, opt_data);

    if(params_.debug)
    {
      cout << "Optimizer converged in " << opt.Iterations() << " iterations." << endl;
      cout << "Status: " << opt.Status() << endl;

      std::ostringstream filename;
      filename << "t_" << iterations_ + 1 << ".tfm";
      dgc_transform_write(t, filename.str().c_str());
    }	

    // compute the delta from this iteration 
    delta = 0.;
    for(int k = 0; k < 4; k++) 
    {
      for(int l = 0; l < 4; l++) 
      {
        double ratio = 1;

        if(k < 3 && l < 3)  // rotation part of the transform
          ratio = 1./params_.epsilon_rot;
        else 
          ratio = 1./params_.epsilon;

        double c_delta = ratio*fabs(t_last[k][l] - t[k][l]);
        
        if(c_delta > delta)
          delta = c_delta;
      }
    }

    if(params_.debug)
      cout << "delta = " << delta << endl;
    
    // check convergence
    iterations_++;

   	if(iterations_ >= params_.max_iteration || delta < 1)
    {
      converged = true;
      //printf("converged at %d of %d\n", iteration, params_.max_iteration);
    }
  }
 
  if(mahalanobis != NULL) 
    delete [] mahalanobis;
  
  if(gsl_R != NULL) 
    gsl_matrix_free(gsl_R);
  
  if(gsl_temp != NULL) 
    gsl_matrix_free(gsl_temp);
}

double GICPAlign::getColorDiff(float c1, float c2)
{
  int rgb1 = *reinterpret_cast<int*>(&c1);
  int rgb2 = *reinterpret_cast<int*>(&c2);

  int r1 = ((rgb1 >> 16) & 0xff);
  int g1 = ((rgb1 >>  8) & 0xff);
  int b1 =  (rgb1        & 0xff);

  int r2 = ((rgb2 >> 16) & 0xff);
  int g2 = ((rgb2 >>  8) & 0xff);
  int b2 =  (rgb2        & 0xff);

  int dr = r1-r2;
  int dg = g1-g2;
  int db = b1-b2;

  return 0.33 * dr*dr + 0.33 * dg*dg + 0.33 * db*db;
}

} //namespace ccny_gicp

