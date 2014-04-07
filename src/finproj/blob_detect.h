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
#include "camer"


typedef struct blob_detect_data blob_detect_data_t;
struct blob_detect_data
{
    int max_error[3];
    int rgb[3];
    int min_blob_size;
    int lines;
};

typedef struct frame frame_t;
struct frame
{
  loc_t xy0;
  loc_t xy1;
  loc_t ex0;
  loc_t ex1;
};

typedef struct metric metrics_t;
struct metric
{
  hsv_t hsv;
  hsv_t error;
  int min_size;
  int lines;
};

void find_diamonds_blob_detector(image_u32_t* im, float* xy0, float* xy1, float* ex0, float* ex1,
        int32_t* rgb_in, int32_t* max_error, zarray_t* blobs_out, int min_blob_size, int lines);

void find_balls_blob_detector(image_u32_t* im, float* xy0, float* xy1, float* ex0, float* ex1,
                              int32_t* rgb_in, int32_t* max_error, zarray_t* blobs_out, 
                              int min_blob_size, int lines);

void find_diamonds_blob_detector(image_u32_t* im, float* xy0, float* xy1, float* ex0, float* ex1,
        int32_t* rgb_in, int32_t* max_error, zarray_t* blobs_out, int min_blob_size, int lines);

void hsv_find_balls_blob_detector(image_u32_t* im, frame_t frame, 
                            metrics_t metric, zarray_t* blobs_out);

void blob_detect_td_rl(image_u32_t* im, float* xy0, float* xy1, float* ex0, float* ex1,
                              int32_t* rgb_in, int32_t* max_error, zarray_t* blobs_out, 
                              int min_blob_size, int lines);


void find_balls_template_matcher(image_u32_t* im, float* xy0, float* xy1, float* ex0, float* ex1, 
                                 image_u32_t* im_template, uint32_t max_error, zarray_t* blobs_out);


#endif
