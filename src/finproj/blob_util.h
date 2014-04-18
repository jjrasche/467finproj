#ifndef BLOB_STUFF_H
#define BLOB_STUFF_H
#include "camera_util.h"

#define NUM_CHART_BLOBS 25
char* matrix_format;


int compare(const void* a, const void* b);

void hsv_find_balls_blob_detector(image_u32_t* im, frame_t frame, 
                            metrics_t metric, zarray_t* blobs_out, vx_buffer_t* buf);

void take_measurements(image_u32_t* im, vx_buffer_t* buf, metrics_t met);


#endif