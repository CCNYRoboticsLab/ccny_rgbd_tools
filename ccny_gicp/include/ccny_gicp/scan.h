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

#ifndef CCNY_GICP_SCAN_H
#define CCNY_GICP_SCAN_H

#include <vector>

typedef struct {
  float mat[3][3]; // 36 bytes, row major
  int transpose;   // 4
  int symmetric;   // 4
  int diagonal;    // 4
} dgc_matrix3d_t;  // = 48 bytes per matrix      

typedef struct {
  float vec[3];    // 12 bytes
} dgc_vector3d_c_t;  // 16 bytes per vector                                                                                                                                                                       

typedef struct {
  float vec[3];    // 12 bytes
} dgc_vector3d_r_t;  // 16 bytes per vector

typedef struct {
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
} dgc_pose_t, * dgc_pose_p;

class dgc_scan_t 
{
  public:
    std::vector<dgc_vector3d_c_t> points;
    std::vector<dgc_vector3d_c_t> norms;
    dgc_pose_t pose;
    
    int save(const char* filename);
    int load(const char *filename);
};

#endif // CCNY_GICP_SCAN_H
