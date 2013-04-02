#include "ccny_rgbd/rgbd_map_util.h"
#include "ccny_rgbd/types.h"

using namespace ccny_rgbd;

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    printf("error: usage is %s [path_to_keyframes]\n", argv[0]);
    return -1;
  }
  
  // path parameter
  std::string path = argv[1];
  
  // read in
  KeyframeVector keyframes;
  loadKeyframes(keyframes, path);
  
  prepareFeaturesForRANSAC(keyframes);
  
  /*
  // build dense association matrix
  cv::Mat dense_association_matrix;
  buildDenseAssociationMatrix(keyframes, dense_association_matrix);
  cv::Mat dense_association_matrix_uint;
  floatMatrixToUintMatrix(dense_association_matrix, dense_association_matrix_uint)
    
  // show
  cv::namedWindow("Dense association matrix", 0);
  cv::imshow("Dense association matrix", dense_association_matrix_uint);
  cv::waitKey(0);
  */
  
  // build brute force surf association matrix
  cv::Mat bf_surf_association_matrix;
  buildBruteForceSURFAssociationMatrix(keyframes, bf_surf_association_matrix);

    
  // show uint
  cv::Mat bf_surf_association_matrix_uint;
  floatMatrixToUintMatrix(bf_surf_association_matrix, bf_surf_association_matrix_uint);
  cv::namedWindow("BF SURF association matrix", 0);
  cv::imshow("BF SURF association matrix", bf_surf_association_matrix_uint);

   
  // show thresholded
  cv::Mat bf_surf_association_matrix_thresholded;
  thresholdMatrix(bf_surf_association_matrix, bf_surf_association_matrix_thresholded, 30);
  cv::namedWindow("BF SURF association matrix T", 0);
  cv::imshow("BF SURF association matrix T", bf_surf_association_matrix_thresholded);

  cv::waitKey(0);
    
  /*
  // build surf association matrix
  cv::Mat surf_association_matrix;
  buildSURFAssociationMatrix(keyframes, surf_association_matrix);
  cv::Mat surf_association_matrix_uint;
  floatMatrixToUintMatrix(surf_association_matrix, surf_association_matrix_uint);
    
  // show
  cv::namedWindow("SURF association matrix", 0);
  cv::imshow("SURF association matrix", surf_association_matrix_uint);
  cv::waitKey(0);
  */
  
  return 0;
}
