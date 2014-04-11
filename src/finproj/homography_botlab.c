// #include "homography_botlab.h"
// #include "common/homography.h"
// #include "common/math_util.h"
// #include "common/image_u32.h"
// #include "common/zhash.h"
// #include "custom_util.h"

// char* matrix_format_b = "%15.5f";
// const double xy_floor_coords[NUM_CALIB_POINTS * 2] = {  30.5,      0,
//                                                         61.1,    -30.5, 
//                                                         61.1,      0,
//                                                         61.1,     30.5,
//                                                         91.6,      0,
//                                                                         };


// // form arguments for homography_compute and return the homography matrix
// matd_t* build_homography(zarray_t* pix_coords)
// {
//     // assemble correspondences
//     // in format: float [] { a, b, c, d } where plane_coords = [a b], pix_coords = [c d]
//     assert(zarray_size(pix_coords) == NUM_CALIB_POINTS);

//     zarray_t* correspondences = zarray_create(sizeof(float[4]));
//     printf("correspondences\n");
//     for(int i = 0; i < NUM_CALIB_POINTS; i++)
//     {
//         double *pix;
//         zarray_get(pix_coords, i, &pix);
//         float tmp[4] = {xy_floor_coords[i*2], xy_floor_coords[i*2+1], pix[0], pix[1]};
//         printf("%lf %lf %lf %lf \n", xy_floor_coords[i*2], xy_floor_coords[i*2+1], pix[0], pix[1]);
//         zarray_add(correspondences, &tmp);
//     }

//     matd_t * H = homography_compute(correspondences);
//     matd_print(H, matrix_format_b);

//     return(H);
// }




// float xy_plane_coords[NUM_DISTORT_POINTS * 2] = {  0,      0,
//                                                     4,      0, 
//                                                     8,      0,
//                                                     12,     0,
//                                                     16,     0,
//                                                     20,     0,
//                                                     24,     0,
//                                                     0,      4,
//                                                     4,      4, 
//                                                     8,      4,
//                                                     12,     4,
//                                                     16,     4,
//                                                     20,     4,
//                                                     24,     4,
//                                                     0,      8,
//                                                     4,      8, 
//                                                     8,      8,
//                                                     12,     8,
//                                                     16,     8,
//                                                     20,     8,
//                                                     24,     8,
//                                                     0,      12,
//                                                     4,      12, 
//                                                     8,      12,
//                                                     12,     12,
//                                                     16,     12,
//                                                     20,     12,
//                                                     24,     12,
//                                                     0,      16,
//                                                     4,      16, 
//                                                     8,      16,
//                                                     12,     16,
//                                                     16,     16,
//                                                     20,     16,
//                                                     24,     16
//                                                                 };

// // form arguments for homography_compute and return the homography matrix
// matd_t* dist_homography(int* pix)
// {
//     // assemble correspondences  
//     // in format: float [] { a, b, c, d } where x = [a b], y = [c d]   and   y = Hx
//     // here y = pix_coords  and  x = real_world plane coords

//     zarray_t* correspondences = zarray_create(sizeof(float[4]));
//     // printf("correspondences\n");
//     for(int i = 0; i < NUM_DISTORT_POINTS; i++)
//     {
//         float tmp[4] = {pix[i*2], pix[i*2+1], 
//                         xy_plane_coords[i*2], xy_plane_coords[i*2+1]};
//         // printf("%f %f %f %f \n", tmp[0], tmp[1], tmp[2], tmp[3]);
//         zarray_add(correspondences, &tmp);
//     }

//     matd_t * H = homography_compute(correspondences);
//     // matd_print(H, matrix_format_b);

//     return(H);
// }
