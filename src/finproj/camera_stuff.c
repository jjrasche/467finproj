#include "custom_util.h"    // This is where a lot of the internals live
#include "homography_botlab.h"

#include "vx/vx_ray3.h"

// #include "blob_detect.h"
#include "lcmtypes/blob_detector_ballpos_t.h"
#include "lcmtypes/blob_detector_ballpos_list_t.h"
#include "camera_util.h"

#include "stdlib.h"

#include "bot_map.h"
#include "bot_utils.h"

/* notes
initiate camera with this to calibrate for white balance every time
./camera --url pgusb://b09d010090f9ac?fidx=1&white-balance-manual=1&white-balance-red=405&white-balance-blue=714
./camera --url dc1394://b09d010090f9ac?fidx=1&white-balance-manual=1&white-balance-red=405&white-balance-blue=714

*/

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
typedef struct vx_event_handler event_handler_t;
char* matrix_format = "%15.5f";


typedef struct state state_t;
struct state
{
    char *url;
    int running;

    vx_application_t  vxapp;
    getopt_t         *gopt;
    parameter_gui_t  *pg;
    double *color_pos;
    int track;
    int run_homography;
    int homography_set;

    int rgb_lock;
    // int32_t tape_error[3];
    // int32_t tape_rgb[3];
    // int tape_min_blob_size;
    // int32_t target_error[3];
    // int32_t target_rgb[3];
    // int target_min_blob_size;

    double sat_change;
    double tape_error[3];
    double tape_hsv[3];
    int tape_min_blob_size;
    double target_error[3];
    double target_hsv[3];
    int target_min_blob_size;

    vx_world_t *camera_world;  // Where vx objects are live
    vx_layer_t *camera_layer;

    vx_world_t *map_world;  // Where vx objects are live
    vx_layer_t *map_layer;

    float angle;

    zarray_t* anchors;
    matd_t* homography_matrix;
    int clicks_to_calibration;

    bot_map_t* map;
    zarray_t* path;
    pid_control_t pid;

    pthread_mutex_t mutex;  // for accessing the arrays
    pthread_t animate_thread;
};

void my_param_changed(parameter_listener_t *pl, parameter_gui_t *pg, const char *name)
{
    pthread_mutex_lock(&mutex);
    state_t *state = pl->impl;

    if(!strcmp("lock", name)) {
       state->rgb_lock = pg_gb(pg, name);;
    }
    if (!strcmp("but1", name)) {
        printf("Begin Homography Calibration.\n");
        state->run_homography = 1;
    }
    if (!strcmp("but2", name)) {
        printf("Incrementing Angle by PI/6.\n");
        state->angle += M_PI/6;
    }
    if (!strcmp("but3", name)) {
        printf("Decrementing Angle by PI/6.\n");
        state->angle -= M_PI/6;
    }
    // tape test
    // if (!strcmp("tape_r", name)) {
    //     printf("tape_r = %d\n", pg_gi(pg, name));
    //     state->tape_rgb[0] = pg_gi(pg,name);
    // }
    // if (!strcmp("tape_r_err", name)) {
    //     printf("tape_r_err = %d\n", pg_gi(pg, name));
    //     state->tape_error[0] = pg_gi(pg,name);
    // }
    // if (!strcmp("tape_g", name)) {
    //     printf("tape_g = %d\n", pg_gi(pg, name));
    //     state->tape_rgb[1] = pg_gi(pg,name);
    // }
    // if (!strcmp("tape_g_err", name)) {
    //     printf("tape_g_err = %d\n", pg_gi(pg, name));
    //     state->tape_error[1] = pg_gi(pg,name);
    // }
    // if (!strcmp("tape_b", name)) {
    //     printf("tape_b = %d\n", pg_gi(pg, name));
    //     state->tape_rgb[2] = pg_gi(pg,name);
    // }
    // if (!strcmp("tape_b_err", name)) {
    //     printf("tape_b_err = %d\n", pg_gi(pg, name));
    //     state->tape_error[2] = pg_gi(pg,name);
    // }
    // if (!strcmp("tape_min_blob_size", name)) {
    //     printf("tape_min_blob_size = %d\n", pg_gi(pg, name));
    //     state->tape_min_blob_size = pg_gi(pg,name);
    // }     

    // // target test
    // if (!strcmp("target_r", name)) {
    //     printf("target_r = %d\n", pg_gi(pg, name));
    //     int old_val = state->target_rgb[0];
    //     state->target_rgb[0] = pg_gi(pg,name);
    //     if(state->rgb_lock) {
    //         state->target_rgb[1] = state->target_rgb[1] * 1.0f/((float)old_val/state->target_rgb[0]);
    //         //pg_si(pg, "taget_g", state->target_rgb[1]);

    //         state->target_rgb[2] = state->target_rgb[2] * 1.0f/((float)old_val/state->target_rgb[0]);
    //         //pg_si(pg, "taget_b", state->target_rgb[2]);
    //         printf("RGB: <%d, %d, %d>\n", state->target_rgb[0], state->target_rgb[1], state->target_rgb[2]);
    //     }
    // }
    // if (!strcmp("target_r_err", name)) {
    //     printf("target_r_err = %d\n", pg_gi(pg, name));
    //     state->target_error[0] = pg_gi(pg,name);
    // }
    // if (!strcmp("target_g", name)) {
    //     printf("target_g = %d\n", pg_gi(pg, name));
    //     int old_val = state->target_rgb[1];
    //     state->target_rgb[1] = pg_gi(pg,name);
    //     if(state->rgb_lock) {
    //         state->target_rgb[0] = state->target_rgb[0] * 1.0f/((float)old_val/state->target_rgb[1]);
    //         //pg_si(pg, "taget_r", state->target_rgb[0]);

    //         state->target_rgb[2] = state->target_rgb[2] * 1.0f/((float)old_val/state->target_rgb[1]);
    //         //pg_si(pg, "taget_b", state->target_rgb[2]);
    //         printf("RGB: <%d, %d, %d>\n", state->target_rgb[0], state->target_rgb[1], state->target_rgb[2]);
    //     }
    // }
    // if (!strcmp("target_g_err", name)) {
    //     printf("target_g_err = %d\n", pg_gi(pg, name));
    //     state->target_error[1] = pg_gi(pg,name);
    // }
    // if (!strcmp("target_b", name)) {
    //     printf("target_b = %d\n", pg_gi(pg, name));
    //     int old_val = state->target_rgb[2];
    //     state->target_rgb[2] = pg_gi(pg,name);
    //     if(state->rgb_lock) {
    //         state->target_rgb[0] = state->target_rgb[0] * 1.0f/((float)old_val/state->target_rgb[2]);
    //         //pg_si(pg, "taget_r", state->target_rgb[0]);
            
    //         state->target_rgb[1] = state->target_rgb[1] * 1.0f/((float)old_val/state->target_rgb[2]);
    //         //pg_si(pg, "taget_g", state->target_rgb[1]);
    //         printf("RGB: <%d, %d, %d>\n", state->target_rgb[0], state->target_rgb[1], state->target_rgb[2]);
    //     }
    // }
    // if (!strcmp("target_b_err", name)) {
    //     printf("target_b_err = %d\n", pg_gi(pg, name));
    //     state->target_error[2] = pg_gi(pg,name);
    // }
    // if (!strcmp("target_min_blob_size", name)) {
    //     printf("target_min_blob_size = %d\n", pg_gi(pg, name));
    //     state->target_min_blob_size = pg_gi(pg,name);
    // }    
    // if (!strcmp("sat_change", name)) {
    //     printf("sat_change = %lf\n", pg_gd(pg, name));
    //     state->sat_change = pg_gd(pg,name);
    // }
    if (!strcmp("tape_h", name)) {
        printf("tape_h = %lf\n", pg_gd(pg, name));
        state->tape_hsv[0] = pg_gd(pg,name);
    }
    if (!strcmp("tape_h_err", name)) {
        printf("tape_h_err = %lf\n", pg_gd(pg, name));
        state->tape_error[0] = pg_gd(pg,name);
    }
    if (!strcmp("tape_s", name)) {
        printf("tape_s = %lf\n", pg_gd(pg, name));
        state->tape_hsv[1] = pg_gd(pg,name);
    }
    if (!strcmp("tape_s_err", name)) {
        printf("tape_s_err = %lf\n", pg_gd(pg, name));
        state->tape_error[1] = pg_gd(pg,name);
    }
    if (!strcmp("tape_v", name)) {
        printf("tape_v = %lf\n", pg_gd(pg, name));
        state->tape_hsv[2] = pg_gd(pg,name);
    }
    if (!strcmp("tape_v_err", name)) {
        printf("tape_v_err = %lf\n", pg_gd(pg, name));
        state->tape_error[2] = pg_gd(pg,name);
    }
    if (!strcmp("tape_min_blob_size", name)) {
        printf("tape_min_blob_size = %d\n", pg_gi(pg, name));
        state->tape_min_blob_size = pg_gi(pg,name);
    }  


    // target test
    if (!strcmp("target_h", name)) {
        printf("target_h = %lf\n", pg_gd(pg, name));
        state->target_hsv[0] = pg_gd(pg,name);
    }
    if (!strcmp("target_h_err", name)) {
        printf("target_h_err = %lf\n", pg_gd(pg, name));
        state->target_error[0] = pg_gd(pg,name);
    }
    if (!strcmp("target_s", name)) {
        printf("target_s = %lf\n", pg_gd(pg, name));
        state->target_hsv[1] = pg_gd(pg,name);
    }
    if (!strcmp("target_s_err", name)) {
        printf("target_s_err = %lf\n", pg_gd(pg, name));
        state->target_error[1] = pg_gd(pg,name);
    }
    if (!strcmp("target_v", name)) {
        printf("target_v = %lf\n", pg_gd(pg, name));
        state->target_hsv[2] = pg_gd(pg,name);
    }
    if (!strcmp("target_v_err", name)) {
        printf("target_v_err = %lf\n", pg_gd(pg, name));
        state->target_error[2] = pg_gd(pg,name);
    }
    if (!strcmp("target_min_blob_size", name)) {
        printf("target_min_blob_size = %d\n", pg_gi(pg, name));
        state->target_min_blob_size = pg_gi(pg,name);
    }    
    pthread_mutex_unlock(&mutex);
}



// Event handler
static int custom_mouse_event(vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse) {
    state_t *state = vh->impl;

    // Handle the event how you see fit
    static int last_mouse = 0;
    static vx_mouse_event_t* last_mouse_event;
    
    //first event, ignore and store
    if (last_mouse == 0) {
        last_mouse_event = mouse;
        last_mouse++;
        return 0;
    }

    int diff_button = mouse->button_mask ^ last_mouse_event->button_mask; //which buttons changed?
    int button_up = diff_button & last_mouse_event->button_mask; // which button(s) just got released?

    int LEFT_BUT_PRESS = (button_up == VX_BUTTON1_MASK); //was the left button pressed?
    int RIGHT_BUT_PRESS = (button_up == VX_BUTTON3_MASK); //was the right button pressed?

    vx_ray3_t pos_ray;
    vx_camera_pos_compute_ray(pos, mouse->x, mouse->y, &pos_ray);
    double* pos_vec3 = calloc(1, 3 * sizeof(double));
    vx_ray3_intersect_xy(&pos_ray, 0, pos_vec3);

    if(LEFT_BUT_PRESS) {
        printf("Raw Mouse: %lf, %lf\n", mouse->x, mouse->y);
        printf("Pos: %lf, %lf\n", pos_vec3[0], pos_vec3[1]);

        pthread_mutex_lock(&mutex);
        //recalc homography
        if(state->run_homography) {
            state->homography_set = 0;
            if(state->clicks_to_calibration <= NUM_CALIB_POINTS) {
                if(state->clicks_to_calibration < NUM_CALIB_POINTS) {
                    zarray_add(state->anchors, &pos_vec3);
                    printf("added point %d of %d to to homography", state->clicks_to_calibration, NUM_CALIB_POINTS-1);
                    state->clicks_to_calibration++;
                    if(state->clicks_to_calibration == NUM_CALIB_POINTS) {
                        state->homography_matrix = build_homography(state->anchors);
                        state->homography_set = 1;
                        
                        //reset so homography can be recalculated
                        state->run_homography = 0;
                        state->clicks_to_calibration = 0;
                        zarray_clear(state->anchors);
                    }
                } 
            }                
        }
        //calc homography on selected point
        else if (state->homography_set) {
            double tmp[3] = {pos_vec3[0], pos_vec3[1], 1};
            matd_t* pix_matrix = matd_create_data(3,1,tmp);
            matd_print(pix_matrix, matrix_format);

            matd_t* xy_floor_coords = matd_op("M^-1*M",state->homography_matrix, pix_matrix);
            printf("XY_Floor_Matrix\n");
            matd_print(xy_floor_coords, matrix_format);

            double x_floor = MATD_EL(xy_floor_coords, 1, 0)/MATD_EL(xy_floor_coords, 2, 0);
            double y_floor = MATD_EL(xy_floor_coords, 0, 0)/MATD_EL(xy_floor_coords, 2, 0);

            printf("Floor Coords: %lf, %lf\n", x_floor, y_floor);

            state->color_pos = pos_vec3;
            state->track = 1;
        }
        pthread_mutex_unlock(&mutex);
    }

    if(RIGHT_BUT_PRESS) {
        printf("right pressed\n");
        pthread_mutex_lock(&mutex);
        state->path = Djikstra(state->map, 0, 0, 0, 60);
        pthread_mutex_unlock(&mutex);
    }

    //store this event
    last_mouse_event = mouse;
    return 0; // Returning 0 says that you have consumed the event. If the event is not consumed (return 1), then it is passed down the chain to the other event handlers.
}
static int custom_touch_event(vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_touch_event_t *mouse) {
    return 1;
}
static int custom_key_event(vx_event_handler_t *vh, vx_layer_t *vl, vx_key_event_t *key) {
    return 1;
}
void custom_destroy(vx_event_handler_t *vh) {
}


// create 10x10 image of pixel color 
void create_color_compare(uint32_t abgr, image_u32_t *im, uint32_t* buf, int size)
{
    im->width = size;
    im->stride = size;
    im->height = size;
    int bufsize = im->stride*im->height;
    
    for(int i = 0; i < bufsize; i++)
    {
        buf[i] = abgr;
    }
    im->buf = buf;
}


void add_color_swatch(state_t* state, image_u32_t *im)
{
    //print color if clicked
    if(state->track) {
        uint32_t idx = state->color_pos[1] * im->stride + state->color_pos[0];
        uint32_t abgr = im->buf[idx];
        uint8_t r = (abgr >> 0) & 0xff;
        uint8_t g = (abgr >> 8) & 0xff;
        uint8_t b = (abgr >> 16) & 0xff;
        printf("%d, %d, %d\n", r, g, b);
        image_u32_t *color_im = calloc(1, sizeof(image_u32_t));
        int size = 50;
        uint32_t buf[size*size];
        create_color_compare(abgr, color_im, buf, size);
        // if  color received, print a swatch of that color to screen
        vx_buffer_add_back(vx_world_get_buffer(state->camera_world, "image"),
                           vxo_chain(vxo_mat_translate3(-size, 0,0),
                             vxo_image_from_u32(color_im,
                                                       0,
                                                       VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER)));
        free(color_im);

    }
}

matd_t* z_mat_from_rads(float angle) {
    matd_t* rot_mat = matd_create(3,3);

    // Rotates about the Z axis
    MATD_EL(rot_mat, 0,0) = cos(angle);
    MATD_EL(rot_mat, 0,1) = sin(angle);
    MATD_EL(rot_mat, 1,0) = -sin(angle);
    MATD_EL(rot_mat, 1,1) = cos(angle);
    MATD_EL(rot_mat, 2,2) = 1;

    return rot_mat;
}

int blob_compare(const void* a, const void* b)
{
    double* aptr = (double*)a;
    double* bptr = (double*)b;

    if (aptr[0] < bptr[0])
        return -1;
    else if (aptr[0] > bptr[0])
        return 1;
    else
        return 0;
}

void* render_loop(void *data)
{
    int fps = 15;
    state_t *state = data;

    vx_buffer_t* map_buffer = vx_world_get_buffer(state->map_world, "map");

    // Set up the imagesource
    pthread_mutex_lock(&mutex);
    char str[150] = "dc1394://b09d010090f9ac?fidx=1&white-balance-manual=1&white-balance-red=405&white-balance-blue=714";
    image_source_t *isrc = image_source_open(str);
    pthread_mutex_unlock(&mutex);

    //exclude only 1 pixel
    float ex0[2] = {0,0};
    float ex1[2] = {1,1};               // TODO: don't exclude any pixels

    //decent values, but could be adjusted to be even better.
    double b = 1.216;
    double c = -.00055;

    if (isrc == NULL) {
        printf("Error opening device.\n");
    } else {

        for (int i = 0; i < isrc->num_formats(isrc); i++) {
            image_source_format_t ifmt;
            isrc->get_format(isrc, i, &ifmt);
            printf("%3d: %4d x %4d (%s)\n", i, ifmt.width, ifmt.height, ifmt.format);
        }
        isrc->start(isrc);
    }

    // Continue running until we are signaled otherwise. This happens
    // when the window is closed/Ctrl+C is received.
    while (state->running) {
        image_u32_t *im = NULL;
        // Get the most recent camera frame and render. it to screen.
        if (isrc != NULL) {
            image_source_data_t * frmd = calloc(1, sizeof(image_source_data_t));
            int res = isrc->get_frame(isrc, frmd);
            if (res < 0) {
                printf("get_frame fail: %d\n", res);
            } else {
                // Handle frame
                im = image_convert_u32(frmd);
                if (im != NULL) {

                    // distortion correction
                   // image_u32_t *distort_im = image_convert_u32(frmd);
                    image_u32_t *distort_im = im;
                    if (distort_im != NULL) {
                        int restor_im_dim[2];
                        scale_new_im(distort_im->width/2, distort_im->height/2, b, c, restor_im_dim);

                        image_u32_t *restor_im = image_u32_create(distort_im->width, distort_im->height);

                        uint32_t abgr_black = 0xff000000;
                        for(int i = 0; i < restor_im->height; i++) {
                            for(int j = 0; j < restor_im->width; j++) {
                                memcpy(&restor_im->buf[i * im->stride + j], &abgr_black, sizeof(uint32_t));
                            }
                        }

                        new_way(distort_im, restor_im, b, c, state->sat_change);  
                        im = restor_im;
                        image_u32_destroy(distort_im);
                    }

                    //mark every 10 cm with a circle
                    for (int i = 10; i < 51; i+=10) {
                        vx_buffer_add_back(vx_world_get_buffer(state->map_world, "map"),
                                    vxo_chain(vxo_mat_scale2(i, i),
                                        vxo_circle(vxo_lines_style(vx_red, 2.0f))));    
                    }

                    //look at only the bottom half of the image
                    //faster and ignores things that may be similar colored but not walls
                    //upper half of image b/c the image is flipped

                    float xy0[2] = {0, 2*im->height/3};
                    float xy1[2] = {im->width-1, im->height-1};
					//float xy0[2] = {0, 320};
					//float xy1[2] = {751, 479};

                    float fit_xy0[2] = {0, 0};
                    float fit_xy1[2] = {im->width-1, im->height-1};
                    // vx_layer_camera_fit2D(state->camera_layer, fit_xy0, fit_xy1, 0);

                    // tape detection
                    zarray_t* tape_locs = zarray_create(sizeof(blob_detector_ballpos_t));
                    pthread_mutex_lock(&mutex);

                    // int32_t tape_max_error[3] = {state->tape_error[0], state->tape_error[1], state->tape_error[2]};
					//int32_t tape_max_error[3] = {78, 78, 78};

                    // int32_t tape_rgb[3] = {state->tape_rgb[0], state->tape_rgb[1], state->tape_rgb[2]};
                    //int32_t tape_rgb[3] = {50, 75, 95};

					
					int tape_min_blob_size = state->tape_min_blob_size;
					//int tape_min_blob_size = 8; 
                    // find_balls_blob_detector(im, xy0, xy1, ex0, ex1, tape_rgb, tape_max_error, 
                                             // tape_locs, tape_min_blob_size, 1);
                    // double tape_max_error[3] = {state->tape_error[0], state->tape_error[1], state->tape_error[2]};
                    // int32_t tape_rgb[3] = {state->tape_rgb[0], state->tape_rgb[1], state->tape_rgb[2]};
                    double tape_hsv[3] = {state->tape_hsv[0], state->tape_hsv[1], state->tape_hsv[2]};
                    double tape_max_error[3] = {state->tape_error[0], state->tape_error[1], state->tape_error[2]};

                    hsv_find_balls_blob_detector(im, xy0, xy1, ex0, ex1, tape_hsv, tape_max_error, 
                                            tape_locs, state->tape_min_blob_size, 1);
                    pthread_mutex_unlock(&mutex);

                    for(int i = 0; i < zarray_size(tape_locs); i++) {
                        blob_detector_ballpos_t pos;

                        zarray_get(tape_locs, i, &pos);

                        if (state->homography_set) {
                            pthread_mutex_lock(&mutex);
                            double tmp[3] = {pos.position[0], abs(pos.position[1]-im->height), 1};
                            matd_t* pix_matrix = matd_create_data(3,1,tmp);

                            //Homographize!
                            matd_t* xy_floor_coords = matd_op("M^-1*M",state->homography_matrix, pix_matrix);

                            //rotate based on angle
                            matd_t* rotation_mat = z_mat_from_rads(M_PI * getOrientation() / 180);
                            matd_t* rotated_coords = matd_multiply(rotation_mat, xy_floor_coords);

                            //Scale down by scale factor
                            double x_floor = MATD_EL(rotated_coords, 1, 0)/MATD_EL(rotated_coords, 2, 0);
                            double y_floor = MATD_EL(rotated_coords, 0, 0)/MATD_EL(rotated_coords, 2, 0);

                            //add this blob to the map, shift to middle of map
                            set_cell_wall(state->map, x_floor+state->map->size/2, y_floor+state->map->size/2);
                            pthread_mutex_unlock(&mutex);

                            matd_destroy(pix_matrix);
                            matd_destroy(xy_floor_coords);
                            matd_destroy(rotation_mat);
                            matd_destroy(rotated_coords);
                        }

                        pthread_mutex_lock(&mutex);
                        vx_buffer_add_back(vx_world_get_buffer(state->camera_world, "image"),
                                    vxo_chain(vxo_mat_translate3(pos.position[0], abs(pos.position[1]-im->height), 1),
                                    vxo_mat_scale(2.0f),
                                    vxo_sphere(vxo_mesh_style(vx_blue))));
                        pthread_mutex_unlock(&mutex);
                    }
                    zarray_destroy(tape_locs);


                    // target detection
                    xy0[0] = 0;
					xy0[1] = 1*im->height/3;
                    //xy0[1] = 0;        // targets will be above floor level 
                    zarray_t* target_locs = zarray_create(sizeof(blob_detector_ballpos_t));
                    pthread_mutex_lock(&mutex);
                    // int target_max_error[3] = {state->target_error[0], state->target_error[1], state->target_error[2]};
                    // int target_rgb[3] = {state->target_rgb[0], state->target_rgb[1], state->target_rgb[2]};
                    // int target_rgb[3] = {state->target_rgb[0], state->target_rgb[1], state->target_rgb[2]};

                    // find_diamonds_blob_detector(im, xy0, xy1, ex0, ex1, target_rgb, target_max_error, 
                                            // target_locs, state->target_min_blob_size, 0);
                    double target_hsv[3] = {state->target_hsv[0], state->target_hsv[1], state->target_hsv[2]};
                    double target_max_error[3] = {state->target_error[0], state->target_error[1], state->target_error[2]};

                    hsv_find_diamonds_blob_detector(im, xy0, xy1, ex0, ex1, target_hsv, target_max_error, 
                                            target_locs, state->tape_min_blob_size, 0);
                    pthread_mutex_unlock(&mutex);

                    int sum_x = 0;
                    int num_targets = zarray_size(target_locs);
                    for(int i = 0; i < num_targets; i++) {
                        blob_detector_ballpos_t pos;

                        zarray_get(target_locs, i, &pos);
                        sum_x += pos.position[0];

                        pthread_mutex_lock(&mutex);
                        vx_buffer_add_back(vx_world_get_buffer(state->camera_world, "image"),
                                    vxo_chain(vxo_mat_translate3(pos.position[0], abs(pos.position[1]-im->height), 1),
                                    vxo_mat_scale(2.0f),
                                    vxo_sphere(vxo_mesh_style(vx_plum))));
                        pthread_mutex_unlock(&mutex);
                    }                    
                    
                    pthread_mutex_lock(&mutex);
                    vx_object_t *vim = vxo_image_from_u32(im, VXO_IMAGE_FLIPY, VX_TEX_MIN_FILTER);
                    vx_buffer_add_back(vx_world_get_buffer(state->camera_world, "image"),vim);


                    if (num_targets >= 1) {
                        //draw line based on mean
                        /*
                        int avg_x = sum_x/num_targets;
                        float points[6] = {avg_x, im->height, 1, avg_x, 0, 1};
                        */

                        //draw line based on median
                        zarray_sort(target_locs, blob_compare);
                        double med[2];
                        zarray_get(target_locs, zarray_size(target_locs)/2, med);
                        float points[6] = {med[0], im->height, 1, med[0], 0, 1};


                        int npoints = 2;
                        vx_resc_t *verts = vx_resc_copyf(points, npoints*3);
                        verts = vx_resc_copyf(points, npoints*3);
                        vx_buffer_add_back(vx_world_get_buffer(state->camera_world, "image"),
                                            vxo_lines(verts, npoints, GL_LINES, vxo_points_style(vx_red, 2.0f)));
                    }
                    zarray_destroy(target_locs);
                    // vx_object_t* obj = calloc(1, sizeof(vx_object_t));;
                    // blob_detect_data_t blob_data;
                    // blob_data.max_error[0] = state->target_error[0];
                    // blob_data.max_error[1] = state->target_error[1];
                    // blob_data.max_error[2] =  state->target_error[2];
                    // blob_data.rgb[0] = state->target_rgb[0];
                    // blob_data.rgb[1] = state->target_rgb[1];
                    // blob_data.rgb[2] = state->target_rgb[2];
                    // blob_data.min_blob_size = state->target_min_blob_size;

                    // int* pos = find_target_center(&blob_data, im);

                    // if either top or bottom corner not found, did not find center
                    // if(pos[2] != -1 && pos[4] != -1) {
                    //     float points[3] = {pos[0], pos[1], 1};
                    //     int npoints = 1;
                    //     vx_resc_t *verts = vx_resc_copyf(points, npoints*3);
                    //     obj = vxo_points(verts, npoints, vxo_points_style(vx_red, 3.0f));
                    //     vx_buffer_add_back(vx_world_get_buffer(state->camera_world, "image"), obj);

                    //     float points2[6] = {pos[2], pos[3], 1, pos[4], pos[5], 1};
                    //     npoints = 2;
                    //     verts = vx_resc_copyf(points2, npoints*3);
                    //     vx_buffer_add_back(vx_world_get_buffer(state->camera_world, "image"),
                    //                         vxo_lines(verts, npoints, GL_LINES, vxo_points_style(vx_red, 2.0f)));
                    // }

                    pthread_mutex_unlock(&mutex);
                }
            }
            fflush(stdout);
            isrc->release_frame(isrc, frmd);
        }

        image_u32_destroy(im);
        
        pthread_mutex_lock(&mutex);
        // add_color_swatch(state, im);
        vx_buffer_swap(vx_world_get_buffer(state->camera_world, "image"));

        render_bot_map(state->map, map_buffer);
        vx_buffer_swap(map_buffer);

        pthread_mutex_unlock(&mutex);

        if (state->track) {
            pthread_mutex_unlock(&mutex);
            usleep(150000);

            pthread_mutex_lock(&mutex);
            state->track = 0;
            pthread_mutex_unlock(&mutex);
        }

        usleep(1000000/fps);
    }

    if (isrc != NULL)
        isrc->stop(isrc);

    return NULL;
}
// This is intended to give you a starting point to work with for any program
// requiring a GUI. This handles all of the GTK and vx setup, allowing you to
// fill in the functionality with your own code.
int main(int argc, char **argv)
{
    custom_init(argc, argv);

    state_t *state = calloc(1, sizeof(state_t));

    state->rgb_lock = 0;

    //create map
    state->map = bot_map_create(1000, 1.0f, 10.0f*1000000);

    state->vxapp.display_started = custom_default_display_started;
    state->vxapp.display_finished = custom_default_display_finished;
    state->vxapp.impl = custom_default_implementation_create();

    // Initialize mouse event handler
    event_handler_t *my_event_handler = malloc(sizeof(event_handler_t));
    my_event_handler->dispatch_order = 0; 
    my_event_handler->impl = state;
    my_event_handler->mouse_event = custom_mouse_event;
    my_event_handler->key_event = custom_key_event;
    my_event_handler->touch_event = custom_touch_event;
    my_event_handler->destroy = custom_destroy;

    //state->vxapp.impl = eecs467_default_implementation_create_handler(state->camera_world, my_event_handler);

    //camera world/layer setup
    state->camera_world = vx_world_create();
    state->camera_layer = vx_layer_create(state->camera_world);
    float percent1[4] = {0.0f, 0.0f, 0.5f, 1.0f};
    custom_default_set_layer_viewport(&state->vxapp, state->camera_layer, percent1);

    //map world/layer setup
    state->angle = 0;
    state->map_world = vx_world_create();
    state->map_layer = vx_layer_create(state->map_world);
    float percent2[4] = {0.5f, 0.0f, 0.5f, 1.0f};
    custom_default_set_layer_viewport(&state->vxapp, state->map_layer, percent2);

    custom_default_add_layer(&state->vxapp, state->map_layer);
    custom_default_add_layer(&state->vxapp, state->camera_layer);
    vx_layer_add_event_handler(state->camera_layer, my_event_handler);
    vx_layer_add_event_handler(state->map_layer, my_event_handler);

    state->running = 1;

    state->sat_change = 1.0;

    // state->tape_error[0] = 8;
    // state->tape_error[1] = 8;
    // state->tape_error[2] = 8;
    // state->tape_min_blob_size = 0;
    // state->tape_rgb[0] = 50;
    // state->tape_rgb[1] = 75;
    // state->tape_rgb[2] = 95;

	//iniitalize bot utils
	utils_init();

    state->tape_error[0] = 10;
    state->tape_error[1] = .5;
    state->tape_error[2] = .5;
    state->tape_min_blob_size = 6;
    state->tape_hsv[0] = 180;
    state->tape_hsv[1] = 0.5;
    state->tape_hsv[2] = 0.5;


    // state->target_error[0] = 10;
    // state->target_error[1] = 10;
    // state->target_error[2] = 10;
    // state->target_min_blob_size = 0;
    // state->target_rgb[0] = 60;
    // state->target_rgb[1] = 110;
    // state->target_rgb[2] = 80;
    state->target_error[0] = 10;
    state->target_error[1] = .5;
    state->target_error[2] = .5;
    state->target_min_blob_size = 6;
    state->target_hsv[0] = 180;
    state->target_hsv[1] = 0.5;
    state->target_hsv[2] = 0.5;

    // turn these on and the stuff below off to be forced to calculate on start
    // state->run_homography = 0;
    // state->homography_set = 0;
    // state->anchors = zarray_create(sizeof(double*));
    
    //Homography calculated 5:18PM 3/3/14 with b=1.2.16, c = -.0055.
    const double h_data[3][3] = {//[3][3];
    {0.60201, 0.62505, 1.96377},
    {0.34545, 0.00470, -5.56394},
    {0.00147, -0.00000, 0.00496}
    };
    state->homography_set = 1;
    state->homography_matrix = matd_create_data(3, 3, *h_data);

    // Parse arguments from the command line, showing the help
    // screen if required
    state->gopt = getopt_create();
    getopt_add_bool(state->gopt, 'h', "homography", 0, "calculate homography");
    getopt_add_bool(state->gopt, 't', "tape", 0, "the sliders are used for tape");
    getopt_add_string(state->gopt, '\0', "url", "", "Camera URL");

    if (!getopt_parse(state->gopt, argc, argv, 1))
    {
        printf("Usage: %s [--url=CAMERAURL] [other options]\n\n", argv[0]);
        getopt_do_usage(state->gopt);
        exit(1);
    }

    // Set up the imagesource. This looks for a camera url specified on
    // the command line and, if none is found, enumerates a list of all
    // cameras imagesource can find and picks the first url it fidns.
    if (strncmp(getopt_get_string(state->gopt, "url"), "", 1)) {
        state->url = strdup(getopt_get_string(state->gopt, "url"));
        printf("URL: %s\n", state->url);
    } else {
        // No URL specified. Show all available and then
        // use the first

        zarray_t *urls = image_source_enumerate();
        printf("Cameras:\n");
        for (int i = 0; i < zarray_size(urls); i++) {
            char *url;
            zarray_get(urls, i, &url);
            printf("  %3d: %s\n", i, url);
        }

        if (zarray_size(urls) == 0) {
            printf("Found no cameras.\n");
            return -1;
        }

        zarray_get(urls, 0, &state->url);
    }

    // if asked to compute homography
        // in order r = 0, g = 1 , b = 2
    if(getopt_get_bool(state->gopt, "homography")) {
        state->run_homography = 1;
    }

    vx_remote_display_source_t *cxn = vx_remote_display_source_create(&state->vxapp);
    parameter_gui_t *pg = pg_create();

    pg_add_double_slider(pg, "sat_change", "Saturation", 0.0, 5.0, 1.0);
        // in order r = 0, g = 1 , b = 2
    // if(getopt_get_bool(state->gopt, "tape")) {
    //     pg_add_int_slider(pg, "tape_r", "Red", 0, 0xff, state->tape_rgb[0]);
    //     pg_add_int_slider(pg, "tape_r_err", "Red Error", 0, 0xff, state->tape_error[0]);
    //     pg_add_int_slider(pg, "tape_g", "Green", 0, 0xff, state->tape_rgb[1]);
    //     pg_add_int_slider(pg, "tape_g_err", "Green Error", 0, 0xff, state->tape_error[1]);
    //     pg_add_int_slider(pg, "tape_b", "BLE", 0, 0xff, state->tape_rgb[2]);
    //     pg_add_int_slider(pg, "tape_b_err", "BLUE Error", 0, 0xff, state->tape_error[2]);
    //     pg_add_int_slider(pg, "tape_min_blob_size", "tape Min Blob Size", 0, 1000, state->tape_min_blob_size);
    // }
    // else {
    //     pg_add_int_slider(pg, "target_r", "Red", 0, 0xff, state->target_rgb[0]);
    //     pg_add_int_slider(pg, "target_r_err", "Red Error", 0, 0xff, state->target_error[0]);
    //     pg_add_int_slider(pg, "target_g", "Green", 0, 0xff, state->target_rgb[1]);
    //     pg_add_int_slider(pg, "target_g_err", "Green Error", 0, 0xff, state->target_error[1]);
    //     pg_add_int_slider(pg, "target_b", "BLE", 0, 0xff, state->target_rgb[2]);
    //     pg_add_int_slider(pg, "target_b_err", "BLUE Error", 0, 0xff, state->target_error[2]);
    //     pg_add_int_slider(pg, "target_min_blob_size", "target Min Blob Size", 0, 1000, state->target_min_blob_size);

    //     pg_add_check_boxes(pg, "lock", "Lock rgb values", 0, NULL);
    // }

    if(getopt_get_bool(state->gopt, "tape")) {
        pg_add_double_slider(pg, "tape_h", "Hue", 0.00, 360, state->tape_hsv[0]);
        pg_add_double_slider(pg, "tape_h_err", "Hue Error", 0, 180, state->tape_error[0]);
        pg_add_double_slider(pg, "tape_s", "Saturation", 0.00, 1.00, state->tape_hsv[1]);
        pg_add_double_slider(pg, "tape_s_err", "Saturation Error", 0, 1, state->tape_error[1]);
        pg_add_double_slider(pg, "tape_v", "Value", 0.00, 1.00, state->tape_hsv[2]);
        pg_add_double_slider(pg, "tape_v_err", "Value Error", 0, 1, state->tape_error[2]);
        pg_add_double_slider(pg, "tape_min_blob_size", "tape Min Blob Size", 0, 1000, state->tape_min_blob_size);
    }
    else {
        pg_add_double_slider(pg, "target_h", "Hue", 0.00, 360, state->target_hsv[0]);
        pg_add_double_slider(pg, "target_h_err", "Hue Error", 0, 180, state->target_error[0]);
        pg_add_double_slider(pg, "target_s", "Saturation", 0.00, 1.00, state->target_hsv[1]);
        pg_add_double_slider(pg, "target_s_err", "Saturation Error", 0, 1, state->target_error[1]);
        pg_add_double_slider(pg, "target_v", "Value", 0.00, 1.00, state->target_hsv[2]);
        pg_add_double_slider(pg, "target_v_err", "Value Error", 0, 1, state->target_error[2]);
        pg_add_double_slider(pg, "target_min_blob_size", "target Min Blob Size", 0, 1000, state->target_min_blob_size);
    }

    pg_add_buttons(pg,
                   "but1", "Calibrate Homography",
                   "but2", "Add pi/6 to Angle",
                   "but3", "Subtract pi/6 from Angle",
                   NULL);

    parameter_listener_t *my_listener = calloc(1,sizeof(parameter_listener_t*));
    my_listener->impl = state;
    my_listener->param_changed = my_param_changed;
    pg_add_listener(pg, my_listener);

    state->pg = pg;

    pthread_create(&state->animate_thread, NULL, render_loop, state);


    custom_gui_run(&state->vxapp, state->pg, 800, 600);
    // Quit when GTK closes
    state->running = 0;

    pthread_join(state->animate_thread, NULL);
    vx_remote_display_source_destroy(cxn);

    // Cleanup
    //free(my_listener);

    vx_global_destroy();
    getopt_destroy(state->gopt);
}
