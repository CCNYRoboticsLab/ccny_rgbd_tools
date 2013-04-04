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

  // *********************************************************
  
  // build brute force surf association matrix
  cv::Mat surf_association_matrix_bf;
  buildSURFAssociationMatrixBruteForce(keyframes, surf_association_matrix_bf, 30);
  cv::namedWindow("surf_association_matrix_bf", 0);
  cv::imshow("surf_association_matrix_bf", surf_association_matrix_bf*255);

  // *********************************************************
  
  // build kdtree surf association matrix
  cv::Mat surf_association_matrix_tree;
  buildSURFAssociationMatrixTree(keyframes, surf_association_matrix_tree, 15, 15, 30);
  cv::namedWindow("surf_association_matrix_tree", 0);
  cv::imshow("surf_association_matrix_tree", surf_association_matrix_tree*255);
  
  // *********************************************************
  
  int false_pos, false_neg,total;
  compareAssociationMatrix(
    surf_association_matrix_bf, surf_association_matrix_tree,
    false_neg, false_pos, total);
  
  printf("Total: %d, FP: %d, FN: %d\n", total, false_pos, false_neg);
  cv::waitKey(0);
  
  return 0;
}
