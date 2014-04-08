#ifndef CAMERA_UTIL_H
#define CAMERA_UTIL_H

#include "custom_util.h" 
#include "math.h"

#define MAXDOT 180       // the oposite pointing direction 
#define MINMAGTOPLOT 10
#define PI 3.1415925
#define MIN3(x,y,z)  ((y) <= (z) ? \
                         ((x) <= (y) ? (x) : (y)) \
                     : \
                         ((x) <= (z) ? (x) : (z)))

#define MAX3(x,y,z)  ((y) >= (z) ? \
                         ((x) >= (y) ? (x) : (y)) \
                     : \
                         ((x) >= (z) ? (x) : (z)))

typedef struct grad grad_t;
struct grad
{
    double x;
    double y;
};


typedef struct loc loc_t;
struct loc
{
    int x;
    int y;
};

typedef struct vec vec_t;
struct vec
{
    double x;
    double y;
};

typedef struct g_node g_node_t;
struct g_node
{
    uint32_t id;
    uint32_t parent_id;
    g_node_t *parent_node;
    loc_t loc;
    grad_t grad;
};

typedef struct abgr abgr_t;
struct abgr
{
    uint8_t a;
    uint8_t b;
    uint8_t g;
    uint8_t r;
};

typedef struct pixel pixel_t;
struct pixel
{
    abgr_t abgr;
    grad_t grad;
    loc_t loc;
};

typedef struct line line_t;
struct line
{
    loc_t start;
    loc_t end;
    zarray_t* nodes;
};

typedef struct hsv hsv_t;
struct hsv
{
    double hue;
    double sat;
    double val;
};
typedef struct threshold_metric threshold_metrics_t;
struct threshold_metric
{   
    hsv_t obj_hsv;
    hsv_t color_error;
    double max_grad_diff;
    int min_size;
    double min_mag;
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


typedef struct node node_t;
struct node {
    uint32_t id;
    uint32_t parent_id;
    node_t *parent_node;
    int num_children;
};



image_u32_t* blur_image(image_u32_t* im, int num_passes);

void flip_image(image_u32_t* im, image_u32_t* flipped);

void convert_to_grad_image(image_u32_t* im, int bright);

void convert_to_grad_dir_image(image_u32_t* im, int bright, double mag); 

void capture_image(image_u32_t *im, int num);

void hsv_find_balls_blob_detector(image_u32_t* im, frame_t frame, 
                            metrics_t metric, zarray_t* blobs_out);

void test_build_line();

// passes in an image to form segementation image, if NULL does nothing
zarray_t* form_lines(image_u32_t*im, threshold_metrics_t thresh, image_u32_t* seg_image);

// caller needs to deallocate buf
void deep_copy_image (image_u32_t* in, image_u32_t* out);

// return int with average RGB values
int average_surrounding_pixels(image_u32_t* image, int x, int y);

//fills in white pixels with avg of surrounding 4 pixels
void interpolate_image(image_u32_t* image);

// assume (xd,yd) are distorted pixel values, with zero corresponding to image center
void scale_new_im(int xd, int yd, double a, double b, int *xyu);

//old method going from distorted to restored
void undistort_xy(image_u32_t* distort_im, image_u32_t* restor_im,
                     double b, double c);

//better, more understandable method
//iterates over empty restored image, and poles distorted image at
//location determined by b*r+c*r^2
void new_way(image_u32_t* distort_im, image_u32_t* restor_im,
                     double b, double c, double sat_change);

void undistort_testing(image_u32_t* distort_im, image_u32_t* restor_im,
                     double b, double c, double sat_change);

void rgb_to_hsv(uint32_t abgr, hsv_t* hsv);
uint32_t hsv_to_rgb(hsv_t hsv);

int changeSaturation(int abgr, double change);

#endif