#ifndef CAMERA_UTIL_H
#define CAMERA_UTIL_H

#include "custom_util.h" 
#include "math.h"

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
    int a;
    int b;
    int g;
    int r;
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


void convert_to_grad_image(image_u32_t* im, int add);

void capture_image(image_u32_t *im, int num);

void test_build_line();

zarray_t* form_lines(image_u32_t*im, double max_grad_diff, int min_size, 
                        hsv_t obj_hsv, hsv_t max_hsv_diff);

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

void rgb_to_hsv(uint32_t abgr, hsv_t hsv);

int changeSaturation(int abgr, double change);

#endif