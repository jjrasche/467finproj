#ifndef HOMOGRAPHY_BOTLAB_H
#define HOMOGRAPHY_BOTLAB_H

#include "common/zarray.h"
#include "common/matd.h"
#define NUM_CALIB_POINTS 5
#define NUM_DISTORT_POINTS 35

//char* matrix_format = "%15.5f";
extern float xy_plane_coords[NUM_DISTORT_POINTS * 2];

void initHomography();
matd_t* build_homography(zarray_t* pix_coords);
matd_t* dist_homography(int* pix_coords);

#endif
