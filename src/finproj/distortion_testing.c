#include "custom_util.h" 
#include "math.h"   
#include "camera_util.h"
#include "homography_botlab.h"

//#define NUM_CALIB_POINTS 6
#define NUM_TEST_BLOBS 35
char* matrix_format = "%15.5f";

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
typedef struct vx_event_handler event_handler_t;

typedef struct state state_t;
struct state
{
    char *url;
    int running;

    vx_application_t  vxapp;
    getopt_t         *gopt;
    parameter_gui_t  *pg;


    hsv_t target_error;
    hsv_t target_hsv;
    int min_size;
    matd_t* homography_matrix;

    vx_world_t *world;  // Where vx objects are live
	vx_layer_t* layer;

    pthread_t animate_thread;
};

// Event handler
static int custom_mouse_event(vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse) {
    // state_t *state = vh->impl;

    // // Handle the event how you see fit
    // static int last_mouse = 0;
    // static vx_mouse_event_t* last_mouse_event;
    
    // //first event, ignore and store
    // if (last_mouse == 0) {
    //     last_mouse_event = mouse;
    //     last_mouse++;
    //     return 0;
    // }

    // int diff_button = mouse->button_mask ^ last_mouse_event->button_mask; //which buttons changed?
    // int button_up = diff_button & last_mouse_event->button_mask; // which button(s) just got released?

    // int LEFT_BUT_PRESS = (button_up == VX_BUTTON1_MASK); //was the left button pressed?
    // int RIGHT_BUT_PRESS = (button_up == VX_BUTTON3_MASK); //was the right button pressed?

    // vx_ray3_t pos_ray;
    // vx_camera_pos_compute_ray(pos, mouse->x, mouse->y, &pos_ray);
    // double* pos_vec3 = calloc(1, 3 * sizeof(double));
    // vx_ray3_intersect_xy(&pos_ray, 0, pos_vec3);

    // if(LEFT_BUT_PRESS) {

    // }
    // if(RIGHT_BUT_PRESS) {


    // }
    // //store this event
    // last_mouse_event = mouse;
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

void my_param_changed(parameter_listener_t *pl, parameter_gui_t *pg, const char *name)
{
    pthread_mutex_lock(&mutex);
    state_t *state = pl->impl;

    if (!strcmp("target_h", name)) {
        state->target_hsv.hue = pg_gd(pg,name);
        printf("hsv:(%lf, %lf, %lf)\n", state->target_hsv.hue,
                state->target_hsv.sat, state->target_hsv.val);
    }   
    if (!strcmp("target_s", name)) {
        state->target_hsv.sat = pg_gd(pg,name);
        printf("hsv:(%lf, %lf, %lf)\n", state->target_hsv.hue,
                state->target_hsv.sat, state->target_hsv.val);
    }   
    if (!strcmp("target_v", name)) {
        state->target_hsv.val = pg_gd(pg,name);
        printf("hsv:(%lf, %lf, %lf)\n", state->target_hsv.hue,
                state->target_hsv.sat, state->target_hsv.val);
    }   
    if (!strcmp("target_h_err", name)) {
        state->target_error.hue = pg_gd(pg,name);
    }   
    if (!strcmp("target_s_err", name)) {
        state->target_error.sat = pg_gd(pg,name);
    }   
    if (!strcmp("target_v_err", name)) {
        state->target_error.val = pg_gd(pg,name);
    }   
    if (!strcmp("min_size", name)) {
        state->min_size = pg_gi(pg,name);
    }    

    pthread_mutex_unlock(&mutex);
}

// return true if a is above or to the left of b
int compare(const void* a, const void* b)
{
    // note, this could be a source of error, So if homography 
    // goes crazy from small movements look here!! The sorting may be wrong
    int pixel_error_margin = 25;

    loc_t* c = (loc_t*)a;
    loc_t* d = (loc_t*)b;
    // same row
    if(abs(c->y - d->y) < pixel_error_margin) 
    {
        // shouldn't be two points within 2 pixels of each other
       // assert(abs(a_xy[0] - b_xy[0]) > pixel_error_margin);
        // a is right of b
        if(c->x > d->x)
            return(1);
    }   // a above b
    else if(c->y - d->y) 
        return(1);

    return(0);
}



// void* render_loop(void *data)
// {
//     pthread_mutex_lock(&mutex);
//     state_t *state = data;
//     vx_buffer_t *buf = vx_world_get_buffer(state->world, "blah");

//     vx_object_t * restor_obj;
//     vx_object_t * distort_obj;
//     image_u32_t *restor_im;

//     // Set up the imagesource
//     char str[150] = "dc1394://b09d010090f9ac?fidx=1&white-balance-manual=1&white-balance-red=405&white-balance-blue=714";
//     image_source_t *isrc = image_source_open(str);

//     if (isrc == NULL) {
//         printf("Error opening device.\n");
//     } else {
//         // Print out possible formats. If no format was specified in the
//         // url, then format 0 is picked by default.
//         // e.g. of setting the format parameter to format 2:
//         //
//         // --url=dc1394://bd91098db0as9?fidx=2
//         for (int i = 0; i < isrc->num_formats(isrc); i++) {
//             image_source_format_t ifmt;
//             isrc->get_format(isrc, i, &ifmt);
//             printf("%3d: %4d x %4d (%s)\n", i, ifmt.width, ifmt.height, ifmt.format);
//         }
//         isrc->start(isrc);
//     }

//     image_source_data_t* frmd = calloc(1, sizeof(image_source_data_t));;
//     while(state->running) {
//         double a = state->A;
//         double b = state->B;
//         double c = state->C;

//         pthread_mutex_unlock(&mutex);
        
//         // Get the most recent camera frame and render it to screen.
//         if (isrc != NULL) {
//             int res = isrc->get_frame(isrc, frmd);
//             if (res < 0) {
//                 printf("get_frame fail: %d\n", res);
//             }
//             else {
//                 image_u32_t *distort_im = image_convert_u32(frmd);
//                 if (distort_im != NULL) {

//                     int restor_im_dim[2];
//                     // scale_new_im(distort_im->width/2, distort_im->height/2, b, c, restor_im_dim);
//                     // restor_im = image_u32_create(2*restor_im_dim[0], 2*restor_im_dim[1]); 
//                     restor_im = image_u32_create(distort_im->width, distort_im->height);
//                     new_way(distort_im, restor_im, b, c, 1.0f);  

//                     float ex0[2] = {0,0};
//                     float ex1[2] = {1,1};   
//                     // float xy0[2] = {restor_im->width/4, restor_im->height/4};
//                     // float xy1[2] = {3*restor_im->width/4, 3*restor_im->height/4};
//                     float xy0[2] = {0, 0};
//                     float xy1[2] = {restor_im->width-1, restor_im->height-1};

//                     zarray_t* blobs = zarray_create(sizeof(blob_detector_ballpos_t));
//                     pthread_mutex_lock(&mutex);
//                     //take pic if clicked
//                     if(state->take_image) {
//                         capture_image(distort_im, state->pic_num);
//                         state->take_image = 0;
//                         state->pic_num++;
//                     }
//                     int32_t max_error[3] = {state->error[0], state->error[1], state->error[2]};
//                     int32_t rgb[3] = {state->rgb[0], state->rgb[1], state->rgb[2]};

//                     find_balls_blob_detector(restor_im, xy0, xy1, ex0, ex1, rgb, max_error, blobs, state->min_blob_size, 0);

//                     zarray_sort(blobs, compare);

//                     pthread_mutex_unlock(&mutex);

//                     if (restor_im != NULL) {
//                         vx_object_t *vim = vxo_image_from_u32(restor_im,
//                                             VXO_IMAGE_FLIPY,
//                                             VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);
//                         vx_buffer_add_back(buf, 
//                                         vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT,
//                                                         vxo_chain (vxo_mat_scale(1),
//                                                                     // vxo_mat_translate3 (0, 
//                                                                     // -restor_im->height, 0),
//                                                                     vim)));                                
//                         vx_buffer_add_back(buf, 
//                                         vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT,
//                                                         vxo_chain (vxo_mat_scale(2),
//                                                                     // vxo_mat_translate3 (0, 
//                                                                     // -restor_im->height, 0),
//                                                                     vxo_sphere(vxo_mesh_style(vx_blue)))));      
                        
//                     }
//                     // number of y pixels coord changes within each row summed 
//                     int distortion_error = 0;
//                     blob_detector_ballpos_t last_pos;


//                     for(int i = 0; i < zarray_size(blobs); i++) {
//                         blob_detector_ballpos_t pos;
//                         zarray_get(blobs, i, &pos);

//                         vx_buffer_add_back(buf,
//                                 vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT,
//                                         vxo_chain(vxo_mat_translate3(pos.position[0], abs(pos.position[1]-restor_im->height), 0),
//                                             vxo_mat_scale(3.0f),
//                                             vxo_circle(vxo_mesh_style(vx_red)))));
//                         if(i > 0)
//                         {
//                             if(!(abs(pos.position[1] - last_pos.position[1]) > 4))
//                             {
//                                 distortion_error += abs(pos.position[1] - last_pos.position[1]);
//                             }
//                         } 
//                         last_pos = pos;
//                     }
//                     // print_double_to_BL_screen((double)distortion_error, buf);
//                    // print_int_to_BR_screen(restor_im->height*restor_im->width, buf);
//                     // image_u32_write_pnm(distort_im, "/home/jjrasche/eecs467_botlab/src/botlab/distorted.pnm");

//                     zarray_destroy(blobs);
//                     image_u32_destroy(restor_im);
//                     image_u32_destroy(distort_im);
//                 }
//             }
//         }
//         vx_buffer_swap(buf);        

//         fflush(stdout);
//         isrc->release_frame(isrc, frmd);
//        // usleep(2000000);

//         pthread_mutex_lock(&mutex);
//     }



//     return NULL;
// }



void* render_loop(void *data)
{
    pthread_mutex_lock(&mutex);
    state_t *state = data;
    vx_buffer_t *buf = vx_world_get_buffer(state->world, "blah");
    int fps = 30;
    int static_test_done = 0;

    vx_object_t * restor_obj;
    vx_object_t * distort_obj;
    image_u32_t *restor_im;
    char image_file [150] = "/home/jjrasche/finalProject/src/finproj/homography.pnm";
    //"/home/jjrasche/eecs467_botlab/src/botlab/pic0.pnm";
    image_u32_t *distort_im = image_u32_create_from_pnm(image_file);
    matd_t * H;
    int pix_array[NUM_DISTORT_POINTS*2];

    double scale = 5.0;

    // -.0009 < c < .0001    total range = .001,   100 setps/ run 
    double step_size = .00001;
    double end_range = -.00056;
    double start_range = .00044;
    double range_size = start_range - end_range;
    double c = start_range;       // initial pos, want to sweep back and forth 3 times and average the outcome
    int num_steps = range_size/step_size;
    double scale_idx = num_steps/range_size;
    double compare_array[num_steps];
    int laps =0;
    int correct_num_blobs = 0;
   // assert((int)(range_size/step_size)+1 == 100);

    while(state->running) {
        // double a = state->A;
        // double b = state->B;
        //double c = state->C;
        int min_size = state->min_size;
        hsv_t max_hsv_diff = state->target_error;
        hsv_t hsv = state->target_hsv;
        pthread_mutex_unlock(&mutex);
        correct_num_blobs = 1;   



        // create restored image based on distortion factors
        // int restor_im_dim[2];
        // scale_new_im(distort_im->width/2, distort_im->height/2, 1.0, c, restor_im_dim);
        // restor_im = image_u32_create(2*restor_im_dim[0], 2*restor_im_dim[1]); 
        // undistort_testing(distort_im, restor_im, 1.0, c, 1.0f);  
        // double scaled_radius = sqrt((restor_im->width*restor_im->width) + 
        //     (restor_im->height * restor_im->height));

        zarray_t* blobs = zarray_create(sizeof(loc_t));

        frame_t frame = {{0,0}, {distort_im->width-1, distort_im->height-1},
                            {0,0}, {1,1}};
        metrics_t met;
        met.hsv = hsv;
        met.error = max_hsv_diff;
        met.min_size = min_size;
        met.lines = 0;

        hsv_find_balls_blob_detector(distort_im, frame, met, blobs);
        zarray_sort(blobs, compare);
        int size = zarray_size(blobs);

        if (distort_im != NULL) {
            vx_object_t *vim = vxo_image_from_u32(distort_im, VXO_IMAGE_FLIPY,
                                        VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);
            vx_buffer_add_back(buf, 
                             vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT, vim));                               
        }

        if(size!= NUM_TEST_BLOBS)  {
            printf("found %d of 35 blobs\n", size);  
            correct_num_blobs = 0;   
        }

        // // print original blobs and build zarray for homography 
        int idx = 0;
        for(int i = 0; i < size; i++) {
            loc_t pos;
            zarray_get(blobs, i, &pos);
            vx_buffer_add_back(buf,
                     vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT,
                            vxo_chain(vxo_mat_translate3(pos.x, abs(pos.y-distort_im->height), 0),
                                vxo_mat_scale(4.0f),
                                vxo_circle(vxo_mesh_style(vx_maroon)))));
            
            // printf("%d size:%d   (%d, %d)\n", i, zarray_size(blobs), pos.x, pos.y);
            if(correct_num_blobs) {
                pix_array[idx*2] = pos.x;
                pix_array[idx*2+1] = pos.y;
                idx++;
            }
            size+=.2f;
        }


        // create homography from plane coordinates and blobs  pix = H plane_coords
        // H = dist_homography(pix_array);

        // // iterate through estimated positions to make them pixels, plot and compare them 
        // idx = 0; 
        // double error = 0;
        // for(int i = 0; i < zarray_size(blobs); i++) {
        //         blob_detector_ballpos_t blob_pix;
        //         zarray_get(blobs, i, &blob_pix);

        //         // TODO: The homography should be in form "pix = H xy", why do I invert this ???
        //         double tmp[3] = {xy_plane_coords[idx*2], xy_plane_coords[idx*2+1], 1};
        //         matd_t* xy_matrix = matd_create_data(3,1,tmp);
        //         matd_t* pix_estimated = matd_op("(M^-1)*M",H, xy_matrix);

        //         // printf("xy position:(%lf, %lf),  actual:(%lf, %lf),  estimated:(%lf, %lf)\n", 
        //         //         tmp[0], tmp[1], blob_pix.position[0], blob_pix.position[1],
        //         //         MATD_EL(pix_estimated, 0, 0), MATD_EL(pix_estimated, 1, 0));
        //         // xy_coords = matd_op("",xy_coords, pix_matrix);

        //         // compare the estimated and blob positions scaled to the radius of the scaled image
        //         error += abs(blob_pix.position[0]-MATD_EL(pix_estimated, 0, 0))/scaled_radius;
        //         error += abs(blob_pix.position[1]-MATD_EL(pix_estimated, 1, 0))/scaled_radius;

        //         vx_buffer_add_back(buf,
        //                  vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT,
        //                         vxo_chain(vxo_mat_translate3(MATD_EL(pix_estimated, 0, 0), 
        //                                                         abs(MATD_EL(pix_estimated, 1, 0)-restor_im->height),0),
        //                             vxo_mat_scale(4.0f),
        //                             vxo_circle(vxo_mesh_style(vx_green)))));
        //         idx++;        // vx_buffer_swap(buf);
        // // pthread_cond_wait(&add_blobs,&mutex);
        // }
        // // vx_buffer_swap(buf);
        // // pthread_cond_wait(&add_blobs,&mutex);        // vx_buffer_swap(buf);
        // // pthread_cond_wait(&add_blobs,&mutex);
        // usleep(1000000/30);
        // // print_double_to_BL_screen(error, buf);

        // char str[60];
        // sprintf(str, "<<#000000>> c:%lf   error: %lf", c, error);
        // vx_object_t *text = vxo_text_create(VXO_TEXT_ANCHOR_BOTTOM_RIGHT, str); 
        // vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_RIGHT, text));

        // // iterate over c and find the best error 
        

        // c -= step_size;
        // if (c < end_range || c > start_range) {      // change directions 
        //     step_size = (-1)*step_size;
        //     laps++;
        // }
        // printf("idx: %d,    c:%lf\n", (int)(c*scale_idx-(end_range*scale_idx)), c);
        // compare_array[(int)(c*scale_idx-(end_range*scale_idx))] = error;


        // // pic the best one
        // if (laps > 1) {
        //     double best_error = 100000.0f;
        //     double best_c_value = 100.0f;
        //     // find best value
        //     for(int i = 0; i < num_steps; i++) {
        //         printf("%d,    %lf,      %lf\n", i,  (double)(i/scale_idx+end_range), compare_array[i]);
        //         if(compare_array[i] < .001) continue;        // a value I skipped because image didn't restore correctly
        //         if(compare_array[i] < best_error) {
        //             best_c_value = (double)(i/scale_idx+end_range);
        //             best_error = compare_array[i];
        //         }
        //     }
        //     printf("best value!! %lf\n", best_error);
        // }


        //TODO:  split display so can see undistorted and distorted images

        zarray_destroy(blobs);
        // image_u32_destroy(distort_im);
        vx_buffer_swap(buf);        

        pthread_mutex_lock(&mutex);
    }

   image_u32_destroy(distort_im);

    return NULL;
}


int main(int argc, char **argv)
{
    custom_init(argc, argv);

    state_t *state = calloc(1, sizeof(state_t));
    state->world = vx_world_create();
    state->layer = vx_layer_create(state->world);

    state->vxapp.display_started = custom_default_display_started;
    state->vxapp.display_finished = custom_default_display_finished;
    state->vxapp.impl = custom_default_implementation_create();
    custom_default_add_layer(&state->vxapp, state->layer);

    // Initialize mouse event handler
    event_handler_t *my_event_handler = malloc(sizeof(event_handler_t));
    my_event_handler->dispatch_order = 0; 
    my_event_handler->impl = state;
    my_event_handler->mouse_event = custom_mouse_event;
    my_event_handler->key_event = custom_key_event;
    my_event_handler->touch_event = custom_touch_event;
    my_event_handler->destroy = custom_destroy;
    vx_layer_add_event_handler(state->layer, my_event_handler);

    state->running = 1;

    // Parse arguments from the command line, showing the help
    // screen if required
    state->gopt = getopt_create();
    getopt_add_bool(state->gopt, 'h', "help", 0, "Show help");
    getopt_add_string(state->gopt, '\0', "url", "", "Camera URL");
    getopt_add_bool(state->gopt, 'p', "print", 0, "print values");


    if (!getopt_parse(state->gopt, argc, argv, 1) || getopt_get_bool(state->gopt, "help"))
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

    // Initialize this application as a remote display source. This allows
    // you to use remote displays to render your visualization. Also starts up
    // the animation thread, in which a render loop is run to update your
    // display.
    vx_remote_display_source_t *cxn = vx_remote_display_source_create(&state->vxapp);

    pthread_create(&state->animate_thread, NULL, render_loop, state);

    // Initialize a parameter gui
    parameter_gui_t *pg = pg_create();
    state->target_error.hue = 42.1;
    state->target_error.sat = .131;
    state->target_error.val = .068;
    state->target_hsv.hue = 180;
    state->target_hsv.sat = 0.249;
    state->target_hsv.val = 0.196;
    state->min_size = 40;
    pg_add_double_slider(pg, "target_h", "Hue", 0.00, 360, state->target_hsv.hue);
    pg_add_double_slider(pg, "target_h_err", "Hue Error", 0, 180, state->target_error.hue);
    pg_add_double_slider(pg, "target_s", "Saturation", 0.00, 1.00, state->target_hsv.sat);
    pg_add_double_slider(pg, "target_s_err", "Saturation Error", 0, 1, state->target_error.sat);
    pg_add_double_slider(pg, "target_v", "Value", 0.00, 1.00, state->target_hsv.val);
    pg_add_double_slider(pg, "target_v_err", "Value Error", 0, 1, state->target_error.val);
    pg_add_int_slider(pg, "min_size", "Min Blob Size", 0, 500, state->min_size);

    parameter_listener_t *my_listener = calloc(1,sizeof(parameter_listener_t*));
    my_listener->impl = state;
    my_listener->param_changed = my_param_changed;
    pg_add_listener(pg, my_listener);

    state->pg = pg;

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
