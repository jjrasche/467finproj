/*      Areas I got stuck
Problem:    lines were not printing
            what was happening: was changing colors based on modding the values in a 
            float[4] the last value represents transparency, and I set it to 0 every 16 
            iterations.
Solution:   mod 3 instead of 4 on indexing into the color array


Problem:    vertical lines on "box" test image where not showing up
            The gradient blobs were being created, but the algorithm to form
            the line was finding the slope of most vertical lines to be 0
Solution:   The root problem here was I was trying to get precision by subtracting
            large numbers, I changed the formula and while the slope was 
            infintesimally small, it was not 0 and true endpoints of the line were found


Problem:    zarray_get was asserting that I was past the last element in array
            when I was not
Solution:   I allocated a zarray of the wrong size (ball_pos_t) and was filling
            it with loc_t which were of different size and overwritting my stack
            I found this out by seeing that my state variable location was being
            overwritten

*/

#include "custom_util.h" 
#include "math.h"   
#include "camera_util.h"
#include "homography_botlab.h"


char* matrix_format = "%15.5f";
char image_name[100] = "/home/jjrasche/finalProject/src/finproj/pic0.pnm";

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
    double min_mag;
    double max_grad_diff;
    matd_t* homography_matrix;
    int print_click;
    int clicks_to_calibration;
    zarray_t* anchors;
    int pic_num;
    int take_image;
    int static_image;
    int grad_image;
    int grad_dir_image;
    int brightness;
    int add_lines;
    int zoom;
    int segment_image;
    int show_pix;

    vx_world_t *world;  // Where vx objects are live
    vx_layer_t* layer;    

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
    if (!strcmp("grad_error", name)) {
        state->max_grad_diff = pg_gd(pg,name);
    }    
    if (!strcmp("brightness", name)) {
        state->brightness = pg_gi(pg,name);
    }    
    if (!strcmp("min_mag", name)) {
        state->min_mag = pg_gd(pg,name);
    }    

    if (!strcmp("zoom", name)) {
        state->zoom = pg_gi(pg,name);
    }    

    if (!strcmp("add_lines", name)) {
        state->add_lines = pg_gb(pg,name);
        printf("pushed add_lines button\n");
    }   
    if (!strcmp("grad_image", name)) {
        state->grad_image = pg_gb(pg,name);
        printf("pushed grad_image button\n");
    }   
    if (!strcmp("static_image", name)) {
        int tmp = state->static_image;
        state->static_image = pg_gb(pg,name);
        printf("static_im: old:%d, new:%d\n", tmp, state->static_image);
    }   
    if (!strcmp("take_image", name)) {
        state->take_image = pg_gb(pg,name);
        printf("pushed take_image button\n");
    }   
    if (!strcmp("seg_image", name)) {
        int tmp = state->segment_image;
        state->segment_image = pg_gb(pg,name);
        printf("segment_image: old:%d, new:%d\n", tmp, state->segment_image);
    }   
    if (!strcmp("show_pix", name)) {
        int tmp = state->show_pix;
        state->show_pix = pg_gb(pg,name);
        printf("show_pix: old:%d, new:%d\n", tmp, state->show_pix);    
    }   
    if (!strcmp("grad_dir_image", name)) {
        int tmp = state->grad_dir_image;
        state->grad_dir_image = pg_gb(pg,name);
        printf("grad_dir_image: old:%d, new:%d\n", tmp, state->grad_dir_image); 
    }   

    pthread_mutex_unlock(&mutex);
}



void* render_loop(void *data)
{
    pthread_mutex_lock(&mutex);
    state_t *state = data;
    vx_buffer_t *buf = vx_world_get_buffer(state->world, "blah");
    int static_image = 0;

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
                image_u32_t *im;
                if(static_image == 1) {
                    im = image_u32_create_from_pnm(image_name);
                }
                else {
                    im = image_convert_u32(frmd);
                }
                if (im != NULL) {
                    image_u32_t* flipped = image_u32_create(im->width, im->height);
                    flip_image(im, flipped);

                    pthread_mutex_lock(&mutex);
                    double max_grad_diff = state->max_grad_diff;
                    int min_size = state->min_size;
                    int brightness = state->brightness;
                    int add_lines = state->add_lines;
                    int take_pic = state->take_image;
                    int set_grad_image = state->grad_image;
                    int set_dir_grad_image = state->grad_dir_image;
                    hsv_t max_hsv_diff = state->target_error;
                    hsv_t hsv = state->target_hsv;
                    static_image = state->static_image;
                    int zoom = state->zoom;
                    int seg = state->segment_image;
                    double min_mag = state->min_mag;
                    int show_pix = state->show_pix;
                    pthread_mutex_unlock(&mutex);

                    threshold_metrics_t thresh = {hsv, max_hsv_diff,max_grad_diff, 
                                                    min_size, min_mag};
                    zarray_t* lines = form_lines(flipped, thresh, NULL);

                    if(set_grad_image == 1) {              
                        convert_to_grad_image(flipped, brightness);
                        vx_object_t *grad_obj = vxo_image_from_u32(flipped, 0, 0);
                        vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT, grad_obj));
                    }
                    else if(set_dir_grad_image == 1) {
                       convert_to_grad_dir_image(flipped, brightness); 
                       vx_object_t *dir_grad_obj = vxo_image_from_u32(flipped, 0, 0);
                       vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT, dir_grad_obj));
                    }
                    else {
                        vx_object_t *vim = vxo_image_from_u32(flipped, 0, 0);
                        vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT, vim));
                    }


                    int num_lines = zarray_size(lines);
                    printf("num_lines = %d \n", num_lines);   
                    float color[4] = {0.3f, 0.6f, 0.9f, 1.0f};
                    double change = 0.3;
                    int npoints = num_lines * 2;
                    float points[npoints*3];
                    if(num_lines != 0) {
                        for(int i = 0; i < num_lines; i++) 
                        {
                            line_t l;
                            zarray_get(lines, i, &l);
                            points[6*i + 0] = l.start.x;
                            points[6*i + 1] = l.start.y;
                            points[6*i + 2] = 1;
                            points[6*i + 3] = l.end.x;
                            points[6*i + 4] = l.end.y;
                            points[6*i + 5] = 1;
                            // printf("line:%d start:(%d, %d)  end:(%d, %d) \n", 
                            //         i, l.start.x, l.start.y, l.end.x, l.end.y);

                            if(show_pix) {

                                if((color[i%3] + change) > 1) color[i%3] = 0;
                                else color[i%3]+= change;

                                for(int j = 0; j < zarray_size(l.nodes); j++)
                                {
                                    g_node_t* node;
                                    zarray_get(l.nodes, j, &node);
                                    vx_buffer_add_back(buf,
                                        vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT,
                                            vxo_chain(vxo_mat_translate3(node->loc.x, node->loc.y, 1),
                                                        vxo_mat_scale(1.0f),
                                                        vxo_circle(vxo_mesh_style(color)))));
                                    // printf("num:%d   loc:(%d, %d)  grad:(%lf, %lf)  color:(%f, %f, %f, %f)\n", i, 
                                    //         node->loc.x, node->loc.y, node->grad.x, node->grad.y,
                                    //         color[0], color[1], color[2], color[3]);
                                }
                            }
                            zarray_vmap(l.nodes, free);
                            zarray_destroy(l.nodes);
                        }
                    }
                    // usleep(3000000);

                    if(add_lines) {
                        vx_resc_t *verts = vx_resc_copyf(points, npoints*3);
                        vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT,
                                                        vxo_lines(verts, npoints, GL_LINES, 
                                                            vxo_points_style(vx_blue, 2.0f))));
                    }
                    if(take_pic == 1) {
                        pthread_mutex_lock(&mutex);
                        capture_image(im, 0);
                        state->take_image = 0;
                        pthread_mutex_unlock(&mutex);
                    }

                    image_u32_destroy(im);
                    image_u32_destroy(flipped);
                    zarray_destroy(lines);
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
    getopt_add_bool(state->gopt, 's', "static_image", 0, 
                    "use a static image rathe than update one");
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

    state->target_error.hue = 45;
    state->target_error.sat = .113;
    state->target_error.val = .25;
    state->target_hsv.hue = 171.5;
    state->target_hsv.sat = 0.148;
    state->target_hsv.val = 0.6;
    pg_add_double_slider(pg, "target_h", "Hue", 0.00, 360, state->target_hsv.hue);
    pg_add_double_slider(pg, "target_h_err", "Hue Error", 0, 180, state->target_error.hue);
    pg_add_double_slider(pg, "target_s", "Saturation", 0.00, 1.00, state->target_hsv.sat);
    pg_add_double_slider(pg, "target_s_err", "Saturation Error", 0, 1, state->target_error.sat);
    pg_add_double_slider(pg, "target_v", "Value", 0.00, 1.00, state->target_hsv.val);
    pg_add_double_slider(pg, "target_v_err", "Value Error", 0, 1, state->target_error.val);
   // pg_add_int_slider(pg, "zoom", "Zoom", 1, 20, state->zoom); 

    state->static_image = 1;
    state->take_image = 0;
    state->grad_dir_image = 0;
    state->min_size = 29;
    state->max_grad_diff = 13.34;        // in degrees
    state->brightness = 100;
    state->min_mag = 15.4; 
    pg_add_int_slider(pg, "brightness", "Bright", 50, 800, state->brightness);
    pg_add_int_slider(pg, "min_size", "Size", 0, 300, state->min_size);
    pg_add_double_slider(pg, "grad_error", "Grad Dir Error", 0, 90, state->max_grad_diff);
    pg_add_double_slider(pg, "min_mag", "Min Magnitude", 0, 700, state->min_mag);
    pg_add_check_boxes(pg,
                        "add_lines", "Lines", 0, 
                        "grad_image", "Show Grad", 0,
                        "grad_dir_image", "Show Grad Direction", 0,
                        "static_image", "Static Image", 1,
                        "take_image", "Take Image", 0,
                        // "seg_image", "Segmentation Image", 0,
                        "show_pix", "Show Pixels", 0,
                                            NULL);

    int pg_add_check_boxes(parameter_gui_t *pg, const char *name, const char * desc, int is_checked, ...) __attribute__((sentinel));

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




























// hsv -> rgb tester
    // double hue, sat, val;
    // while(1){
    //     printf("enter hue:\n");
    //     scanf("%lf", &hue);
    //     printf("enter sat:\n");
    //     scanf("%lf", &sat);
    //     printf("enter val:\n");
    //     scanf("%lf", &val);
    //     hsv_t temp = {hue, sat, val};
    //     uint32_t b =  hsv_to_rgb(temp);
    //     printf("hsv: (%lf, %lf, %lf)\n", hue, sat, val);
    // }