#include "custom_util.h" 
#include "math.h"   
#include "camera_util.h"
#include "homography_botlab.h"


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

    int32_t error[3];
    int32_t rgb[3];
    int min_blob_size;
    matd_t* homography_matrix;
    int print_click;
    int clicks_to_calibration;
    zarray_t* anchors;
    int pic_num;
    int take_image;

    vx_world_t *world;  // Where vx objects are live
    vx_layer_t* layer;

    int phase;

    pthread_t animate_thread;
};

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
 
        // if (state->clicks_to_calibration <= NUM_CALIB_POINTS) {
        //     if(state->clicks_to_calibration < NUM_CALIB_POINTS) {
        //         zarray_add(state->anchors, &pos_vec3);
        //         printf("added point %d of %d to to homography", state->clicks_to_calibration, NUM_CALIB_POINTS-1);
        //         state->clicks_to_calibration++;
        //         if(state->clicks_to_calibration == NUM_CALIB_POINTS) {
        //             state->homography_matrix = build_homography(state->anchors);
        //             state->homography_set = 1;
        //         }
        //     } 
        // }                
        // if (state->homography_set) {
        //     double tmp[3] = {pos_vec3[0], pos_vec3[1], 1};
        //     matd_t* pix_matrix = matd_create_data(3,1,tmp);
        //     matd_print(pix_matrix, matrix_format);

        //     matd_t* xy_pattern_coords = matd_op("M^-1*M",state->homography_matrix, pix_matrix);
        //     printf("XY_Floor_Matrix\n");
        //     matd_print(xy_pattern_coords, matrix_format);

        //     double x = MATD_EL(xy_pattern_coords, 1, 0)/MATD_EL(xy_pattern_coords, 2, 0);
        //     double y = MATD_EL(xy_pattern_coords, 0, 0)/MATD_EL(xy_pattern_coords, 2, 0);

        //     printf("Floor Coords: %lf, %lf\n", x, y);
        // }
        
        state->print_click = 1;
        pthread_mutex_unlock(&mutex);
    }
    if(RIGHT_BUT_PRESS) {
        pthread_mutex_lock(&mutex);
        printf("taking image %d", state->pic_num);
        state->take_image = 1;
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

void my_param_changed(parameter_listener_t *pl, parameter_gui_t *pg, const char *name)
{
    pthread_mutex_lock(&mutex);
    // state_t *state = pl->impl;
    pthread_mutex_unlock(&mutex);
}



void* render_loop(void *data)
{
    pthread_mutex_lock(&mutex);
    state_t *state = data;
    vx_buffer_t *buf = vx_world_get_buffer(state->world, "blah");

    // char str[150] = "dc1394://b09d010090f9ac?fidx=1&white-balance-manual=1&white-balance-red=405&white-balance-blue=714";
    image_source_t *isrc = image_source_open(state->url);

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

    image_source_data_t* frmd = calloc(1, sizeof(image_source_data_t));;
    while(state->running) {
        pthread_mutex_unlock(&mutex);
        
        // Get the most recent camera frame and render it to screen.
        if (isrc != NULL) {
            int res = isrc->get_frame(isrc, frmd);
            if (res < 0) {
                printf("get_frame fail: %d\n", res);
            }
            else {
                image_u32_t *im = image_convert_u32(frmd);
                if (im != NULL) {

 

                    vx_object_t *vim = vxo_image_from_u32(im, VXO_IMAGE_FLIPY, 0);
                    vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT, vim));



                    // vx_buffer_add_back(buf, 
                    //                 vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT,
                    //                                 vxo_chain (vxo_mat_scale(2),
                    //                                             // vxo_mat_translate3 (0, 
                    //                                             // -restor_im->height, 0),
                    //                                             vxo_sphere(vxo_mesh_style(vx_blue)))));      
                    
                
                    image_u32_destroy(im);
                }
            }
        }
        vx_buffer_swap(buf);        
        fflush(stdout);
        isrc->release_frame(isrc, frmd);
        pthread_mutex_lock(&mutex);
    }



    return NULL;
}



// This is intended to give you a starting point to work with for any program
// requiring a GUI. This handles all of the GTK and vx setup, allowing you to
// fill in the functionality with your own code.
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

    // state->target_error[0] = 10;
    // state->target_error[1] = .5;
    // state->target_error[2] = .5;
    // state->target_min_blob_size = 6;
    // state->target_hsv[0] = 180;
    // state->target_hsv[1] = 0.5;
    // state->target_hsv[2] = 0.5;

    
    //Homography calculated 5:18PM 3/3/14 with b=1.2.16, c = -.0055.
    // const double h_data[3][3] = {//[3][3];
    // {0.60201, 0.62505, 1.96377},
    // {0.34545, 0.00470, -5.56394},
    // {0.00147, -0.00000, 0.00496}
    // };
    // state->homography_set = 1;
    // state->homography_matrix = matd_create_data(3, 3, *h_data);

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


    vx_remote_display_source_t *cxn = vx_remote_display_source_create(&state->vxapp);
    parameter_gui_t *pg = pg_create();

    // pg_add_double_slider(pg, "target_h", "Hue", 0.00, 360, state->target_hsv[0]);
    // pg_add_double_slider(pg, "target_h_err", "Hue Error", 0, 180, state->target_error[0]);
    // pg_add_double_slider(pg, "target_s", "Saturation", 0.00, 1.00, state->target_hsv[1]);
    // pg_add_double_slider(pg, "target_s_err", "Saturation Error", 0, 1, state->target_error[1]);
    // pg_add_double_slider(pg, "target_v", "Value", 0.00, 1.00, state->target_hsv[2]);
    // pg_add_double_slider(pg, "target_v_err", "Value Error", 0, 1, state->target_error[2]);
    // pg_add_double_slider(pg, "target_min_blob_size", "target Min Blob Size", 0, 1000, state->target_min_blob_size);

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
