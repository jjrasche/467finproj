#ifndef BLOB_DETECT_H
#define BLOB_DETECT_H

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "common/image_u32.h"
#include "common/zarray.h"
#include "common/zhash.h"
#include "common/matd.h"
#include "lcmtypes/blob_detector_ballpos_t.h"


typedef struct blob_detect_data blob_detect_data_t;
struct blob_detect_data
{
    int max_error[3];
    int rgb[3];
    int min_blob_size;
    int lines;
};

void find_diamonds_blob_detector(image_u32_t* im, float* xy0, float* xy1, float* ex0, float* ex1,
        int32_t* rgb_in, int32_t* max_error, zarray_t* blobs_out, int min_blob_size, int lines);

void find_balls_blob_detector(image_u32_t* im, float* xy0, float* xy1, float* ex0, float* ex1,
                              int32_t* rgb_in, int32_t* max_error, zarray_t* blobs_out, 
                              int min_blob_size, int lines);

void find_diamonds_blob_detector(image_u32_t* im, float* xy0, float* xy1, float* ex0, float* ex1,
        int32_t* rgb_in, int32_t* max_error, zarray_t* blobs_out, int min_blob_size, int lines);

void hsv_find_balls_blob_detector(image_u32_t* im, float* xy0, float* xy1, float* ex0, float* ex1,
                                    double* hsv_in, double* max_error, zarray_t* blobs_out, 
                                    int min_blob_size, int lines);

void hsv_find_diamonds_blob_detector(image_u32_t* im, float* xy0, float* xy1, float* ex0, float* ex1,
        double* hsv_in, double* max_error, zarray_t* blobs_out, int min_blob_size, int lines);

void blob_detect_td_rl(image_u32_t* im, float* xy0, float* xy1, float* ex0, float* ex1,
                              int32_t* rgb_in, int32_t* max_error, zarray_t* blobs_out, 
                              int min_blob_size, int lines);


void find_balls_template_matcher(image_u32_t* im, float* xy0, float* xy1, float* ex0, float* ex1, 
                                 image_u32_t* im_template, uint32_t max_error, zarray_t* blobs_out);


#endif
