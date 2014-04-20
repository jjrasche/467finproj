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

Symptom:    zarray_remove was not remooving all it was supposed to
Problem:    was shifting all elements in array down when deleted, so was 
            moving past every other element in array

*/

#include "custom_util.h" 
#include "math.h"   
#include "camera_util.h"
#include "homography_botlab.h"
#include "blob_util.h"
#define NUM_CON_METHODS 4
#define NUM_HSV_CALIBS 4  // yellow highlights, black background, aluminum background, green testing 

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
    int blur_amount;
    double min_mag;
    double max_grad_diff;
    hsv_calib_t* hsv_calibrations;
    int hsv_calib_num;

    int static_image;
    int grad_image;
    int grad_dir_image;
    int brightness;
    int add_lines;
    int dilate_im;
    int zoom;
    int segment_image;
    int show_pix;
    int show_homography;
    int connection_method;
    int blob_detect;
    int qualify;
    double square_variation;
    double consensus_accuracy;
    int num_outliers;

    loc_t clicked_loc;
    zarray_t* image_array;
    int opened_im;
    int pic_num;
    int take_image;

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
        pthread_mutex_lock(&mutex);
        // printf("Raw Mouse: %lf, %lf\n", mouse->x, mouse->y);
        // printf("Pos: %lf, %lf\n", pos_vec3[0], pos_vec3[1]);
        state->clicked_loc.x = mouse->x;
        state->clicked_loc.y = mouse->y;
        pthread_mutex_unlock(&mutex);

    }
    if(RIGHT_BUT_PRESS) {
        // pthread_mutex_lock(&mutex);
        // printf("taking image %d", state->pic_num);
        // state->take_image = 1;
        // pthread_mutex_unlock(&mutex);

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
        state->hsv_calibrations[state->hsv_calib_num].hsv.hue = pg_gd(pg,name);
        // printf("hsv:(%lf, %lf, %lf)\n", state->target_hsv.hue,
                // state->target_hsv.sat, state->target_hsv.val);
    }   
    if (!strcmp("target_s", name)) {
        state->hsv_calibrations[state->hsv_calib_num].hsv.sat = pg_gd(pg,name);
        // printf("hsv:(%lf, %lf, %lf)\n", state->target_hsv.hue,
                // state->target_hsv.sat, state->target_hsv.val);
    }   
    if (!strcmp("target_v", name)) {
        state->hsv_calibrations[state->hsv_calib_num].hsv.val = pg_gd(pg,name);
        // printf("hsv:(%lf, %lf, %lf)\n", state->target_hsv.hue,
                // state->target_hsv.sat, state->target_hsv.val);
    }   
    if (!strcmp("target_h_err", name)) {
        state->hsv_calibrations[state->hsv_calib_num].error.hue = pg_gd(pg,name);
    }   
    if (!strcmp("target_s_err", name)) {
        state->hsv_calibrations[state->hsv_calib_num].error.sat = pg_gd(pg,name);
    }   
    if (!strcmp("target_v_err", name)) {
        state->hsv_calibrations[state->hsv_calib_num].error.val = pg_gd(pg,name);
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
    if (!strcmp("blur_amount", name)) {
        state->blur_amount = pg_gi(pg,name);
    }
    if (!strcmp("consensus_accuracy", name)) {
        state->consensus_accuracy = pg_gd(pg,name);
    }    
    if (!strcmp("num_outliers", name)) {
        state->num_outliers = pg_gi(pg,name);
    }    
    if (!strcmp("connection_method", name)) {
        state->connection_method = pg_gi(pg,name);
    }    
    if (!strcmp("blob_detect", name)) {
        state->blob_detect = pg_gb(pg,name);
    }   
    if (!strcmp("square_variation", name)) {
        state->square_variation = pg_gd(pg,name);
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
    if (!strcmp("dilate_im", name)) {
        state->dilate_im = pg_gb(pg,name);
    }   
    if (!strcmp("qualify", name)) {
        state->qualify = pg_gb(pg,name);
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
    if (!strcmp("show_homography", name)) {
        state->show_homography = pg_gb(pg,name);
    }   
    if (!strcmp("next_image", name)) {
        state->opened_im++;
        if(state->opened_im == zarray_size(state->image_array)) 
            state->opened_im = 0;
        printf("opened_image = %d    %d\n", state->opened_im, zarray_size(state->image_array));
    }   

    if (!strcmp("next_hsv_calib", name)) {
        state->hsv_calib_num++;
        if(state->hsv_calib_num == NUM_HSV_CALIBS) 
            state->hsv_calib_num = 0;
        printf("hsv_calib_num = %d\n", state->hsv_calib_num);
    }   

    pthread_mutex_unlock(&mutex);
}


void* render_loop(void *data)
{
    pthread_mutex_lock(&mutex);
    state_t *state = data;
    vx_buffer_t *buf = vx_world_get_buffer(state->world, "blah");
    int static_image = state->static_image;
    int blob_detect = state->blob_detect;

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
        
        // Get the most recent camera frame and render it to screen.
        if (isrc != NULL) {
            int res = isrc->get_frame(isrc, frmd);
            if (res < 0) {
                printf("get_frame fail: %d\n", res);
            }
            else {
                image_u32_t *im;
                if(static_image == 1) {
                    image_t image;
                    zarray_get(state->image_array, state->opened_im, &image);
                    im = image_u32_create_from_pnm(image.name);
                }
                else {
                    im = image_convert_u32(frmd);
                }
                if (im != NULL) {
                    double max_grad_diff = state->max_grad_diff;
                    int min_size = state->min_size;
                    int brightness = state->brightness;
                    int add_lines = state->add_lines;
                    int take_pic = state->take_image;
                    int set_grad_image = state->grad_image;
                    int set_dir_grad_image = state->grad_dir_image;
                    hsv_t max_hsv_diff = state->target_error;
                    hsv_t hsv = state->target_hsv;
                    hsv_calib_t* hsv_calib = state->hsv_calibrations;
                    static_image = state->static_image;
                    int zoom = state->zoom;
                    int seg = state->segment_image;
                    double min_mag = state->min_mag;
                    int show_pix = state->show_pix;
                    int homography = state->show_homography;
                    int blur_amount = state->blur_amount;
                    int connection_method = state->connection_method;
                    int dilate_im = state->dilate_im;
                    blob_detect = state->blob_detect;
                    double var = state->square_variation;
                    int qualify = state->qualify;
                    int num_outliers = state->num_outliers;
                    double conacc = state->consensus_accuracy;
                    pthread_mutex_unlock(&mutex);

                    image_u32_t* flipped = image_u32_create(im->width, im->height);
                    flip_image(im, flipped);
                    flipped = blur_image(flipped, blur_amount);

                    if(state->clicked_loc.x != -1) {
                        int idx = state->clicked_loc.y * flipped->stride + state->clicked_loc.x;
                        hsv_t hsv;
                        rgb_to_hsv(flipped->buf[idx], &hsv);
                        printf("hsv: (%lf, %lf, %lf) \n", hsv.hue, hsv.sat, hsv.val);
                        state->clicked_loc.x = -1;
                    }
                    if(take_pic == 1) {
                        pthread_mutex_lock(&mutex);
                        capture_image(state->image_array,im, state->pic_num);
                        state->take_image = 0;
                        state->pic_num++;
                        pthread_mutex_unlock(&mutex);
                    }
                    metrics_t met = {   .hsv_data = hsv_calib,//{max_hsv_diff.hue, max_hsv_diff.sat, max_hsv_diff.val},
                                        .min_size = min_size,
                                        .std_dev_from_square = var,
                                        .qualify = qualify,
                                        .lines = 0,
                                        .add_lines = add_lines,
                                        .dothis = dilate_im,
                                        .num_outliers = num_outliers,
                                        .consensus_accuracy = conacc
                                    };
                    // find and add to buffer all blobs that match a certain color
                    if(blob_detect) {
                        vx_object_t *vim = vxo_image_from_u32(flipped, 0, 0);
                        vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT, vim));

                        // finds matching blobs, connects lines, and builds homography
                        take_measurements(flipped, buf, met);

                        image_u32_destroy(im);
                        image_u32_destroy(flipped);
                        vx_buffer_swap(buf);
                        fflush(stdout);
                        isrc->release_frame(isrc, frmd);
                        pthread_mutex_lock(&mutex);
                        continue;
                    }

                    threshold_metrics_t thresh = {hsv, max_hsv_diff,max_grad_diff, 
                                                    min_size, min_mag, connection_method, 
                                                    dilate_im};
                    zarray_t* lines = form_lines(flipped, thresh, NULL);

                    if(set_grad_image == 1) {              
                        convert_to_grad_image(flipped, brightness, thresh);
                        vx_object_t *grad_obj = vxo_image_from_u32(flipped, 0, 0);
                        vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT, grad_obj));
                    }
                    else if(set_dir_grad_image == 1) {
                       convert_to_grad_dir_image(flipped, brightness, thresh); 
                       vx_object_t *dir_grad_obj = vxo_image_from_u32(flipped, 0, 0);
                       vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT, dir_grad_obj));
                    }
                    else {
                        vx_object_t *vim = vxo_image_from_u32(flipped, 0, 0);
                        vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT, vim));
                    }


                    int num_lines = zarray_size(lines);
                    // printf("num_lines = %d \n", num_lines);   
                    float color[4] = {0.3f, 0.6f, 0.9f, 1.0f};
                    double change = 0.1;
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
                                                        vxo_rect(vxo_mesh_style(color)))));
                                    // printf("num:%d   loc:(%d, %d)  grad:(%lf, %lf)  color:(%f, %f, %f, %f)\n", i, 
                                    //         node->loc.x, node->loc.y, node->grad.x, node->grad.y,
                                    //         color[0], color[1], color[2], color[3]);
                                    // vx_buffer_add_back(buf, vxo_rect(vxo_mesh_style(vx_red),
                                    //                                  vxo_lines_style(vx_yellow, 2.0f),
                                    //                                  vxo_points_style(vx_cyan, 5.0f)));
                                }
                            }
                            zarray_vmap(l.nodes, free);
                            zarray_destroy(l.nodes);
                        }
                    }

                    if(add_lines) {
                        vx_resc_t *verts = vx_resc_copyf(points, npoints*3);
                        vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT,
                                                        vxo_lines(verts, npoints, GL_LINES, 
                                                            vxo_points_style(vx_blue, 2.0f))));
                    }
                    if(homography) {
                        // metrics_t met = {   .hsv = hsv,//{hsv.hue, hsv.sat, hsv.val},
                        //                     .error = max_hsv_diff,//{max_hsv_diff.hue, max_hsv_diff.sat, max_hsv_diff.val},
                        //                     .min_size = min_size,
                        //                     .lines = 0
                        //                 };
                        // take_measurements(flipped, buf, met);
                    }

                    image_u32_destroy(im);
                    image_u32_destroy(flipped);
                    zarray_destroy(lines);
                }
                else {
                    printf("image is null\n");
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


void add_images(zarray_t* arr)
{
    add_image(arr, "/home/jjrasche/finalProject/src/finproj/pic0.pnm", 640, 480, 2);
    add_image(arr, "/home/jjrasche/finalProject/src/finproj/pic1.pnm", 640, 480, 2);
    add_image(arr, "/home/jjrasche/finalProject/src/finproj/face_4m.pnm", 640, 480, 2);
    add_image(arr, "/home/jjrasche/finalProject/src/finproj/face_6m.pnm", 640, 480, 2);
    add_image(arr, "/home/jjrasche/finalProject/src/finproj/face_10m.pnm", 640, 480, 2);
    add_image(arr, "/home/jjrasche/finalProject/src/finproj/corners0.pnm", 640, 480, 3);
    add_image(arr, "/home/jjrasche/finalProject/src/finproj/corners1.pnm", 640, 480, 2);
    add_image(arr, "/home/jjrasche/finalProject/src/finproj/corners2.pnm", 640, 480, 2);

    // add_image(arr, "/home/jjrasche/finalProject/src/finproj/pic2.pnm", 640, 480, 2);

    // add_image(arr, "/home/jjrasche/finalProject/src/finproj/test_images/1_5_meter_352x258.pnm", 355, 258, 1.5);
    // add_image(arr, "/home/jjrasche/finalProject/src/finproj/test_images/2_meter_352x258.pnm", 355, 258, 2);
    // add_image(arr, "/home/jjrasche/finalProject/src/finproj/test_images/3_meter_352x258.pnm", 355, 258, 3);
    // add_image(arr, "/home/jjrasche/finalProject/src/finproj/test_images/4_meter_352x258.pnm", 355, 258, 4);

    // add_image(arr, "/home/jjrasche/finalProject/src/finproj/test_images/1_5_meter_640x480.pnm", 640, 480, 1.5);
    // add_image(arr, "/home/jjrasche/finalProject/src/finproj/test_images/2_meter_640x480.pnm", 640, 480, 2);
    // add_image(arr, "/home/jjrasche/finalProject/src/finproj/test_images/3_meter_640x480.pnm", 640, 480, 3);
    // add_image(arr, "/home/jjrasche/finalProject/src/finproj/test_images/4_meter_640x480.pnm", 640, 480, 4);
    
    // add_image(arr, "/home/jjrasche/finalProject/src/finproj/test_images/1_5_meter_1280x720.pnm", 1280, 720, 1.5);
    // add_image(arr, "/home/jjrasche/finalProject/src/finproj/test_images/2_meter_1280x720.pnm", 1280, 720, 2);
    // add_image(arr, "/home/jjrasche/finalProject/src/finproj/test_images/3_meter_1280x720.pnm", 1280, 720, 3);
    // add_image(arr, "/home/jjrasche/finalProject/src/finproj/test_images/4_meter_1280x720.pnm", 1280, 720, 4);

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
    state->image_array = zarray_create(sizeof(image_t));
    add_images(state->image_array);

    image_t tmp;
    zarray_get(state->image_array, 0, &tmp);

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


    // hsv calibration setting (0,1,2,3) --> target color, black background, grey background, green testing
    hsv_calib_t calibs[NUM_HSV_CALIBS];

    calibs[TARGETCOLOR].hsv.hue = 65;
    calibs[TARGETCOLOR].hsv.sat = .81;
    calibs[TARGETCOLOR].hsv.val = .68;
    calibs[TARGETCOLOR].error.hue = 40;
    calibs[TARGETCOLOR].error.sat = .5;
    calibs[TARGETCOLOR].error.val = .4;

    // calibs[TARGETCOLOR].hsv.hue = 65;
    // calibs[TARGETCOLOR].hsv.sat = .6;
    // calibs[TARGETCOLOR].hsv.val = .75;
    // calibs[TARGETCOLOR].error.hue = 12;
    // calibs[TARGETCOLOR].error.sat = .25;
    // calibs[TARGETCOLOR].error.val = .15;

    calibs[BLACKBACKGROUND].hsv.hue = 180;
    calibs[BLACKBACKGROUND].hsv.sat = .1;
    calibs[BLACKBACKGROUND].hsv.val = .215;
    calibs[BLACKBACKGROUND].error.hue = 360;
    calibs[BLACKBACKGROUND].error.sat = .2;
    calibs[BLACKBACKGROUND].error.val = .2;

    calibs[ALUMINUMBACKGROUND].hsv.hue = 30;
    calibs[ALUMINUMBACKGROUND].hsv.sat = .15;
    calibs[ALUMINUMBACKGROUND].hsv.val = .6;
    calibs[ALUMINUMBACKGROUND].error.hue = 40;
    calibs[ALUMINUMBACKGROUND].error.sat = .3;
    calibs[ALUMINUMBACKGROUND].error.val = .2;

    calibs[GREENTESTING].hsv.hue = 120;
    calibs[GREENTESTING].hsv.sat = 1;
    calibs[GREENTESTING].hsv.val = .6;
    calibs[GREENTESTING].error.hue = 31.5;
    calibs[GREENTESTING].error.sat = .61;
    calibs[GREENTESTING].error.val = .62;

    state->hsv_calib_num = 0;
    state->hsv_calibrations = calibs;
    state->square_variation = 2;
    state->consensus_accuracy = 2.0;
    state->num_outliers = 10;
    pg_add_double_slider(pg, "target_h", "Hue", 0.00, 360, state->hsv_calibrations[TARGETCOLOR].hsv.hue);
    pg_add_double_slider(pg, "target_h_err", "Hue Error", 0, 360, state->hsv_calibrations[TARGETCOLOR].error.hue);
    pg_add_double_slider(pg, "target_s", "Saturation", 0.00, 1.00, state->hsv_calibrations[TARGETCOLOR].hsv.sat);
    pg_add_double_slider(pg, "target_s_err", "Saturation Error", 0, 1, state->hsv_calibrations[TARGETCOLOR].error.sat);
    pg_add_double_slider(pg, "target_v", "Value", 0.00, 1.00, state->hsv_calibrations[TARGETCOLOR].hsv.val);
    pg_add_double_slider(pg, "target_v_err", "Value Error", 0, 1, state->hsv_calibrations[TARGETCOLOR].error.val);
    // pg_add_double_slider(pg, "square_variation", "Sq Var", 0, 50, state->square_variation);
    pg_add_int_slider(pg, "num_outliers", "Outliers", 0, 50, state->num_outliers);
    pg_add_double_slider(pg, "consensus_accuracy", "ConAcc", 0, 50, state->consensus_accuracy);

   // pg_add_int_slider(pg, "zoom", "Zoom", 1, 20, state->zoom); 

    state->static_image = 1;
    state->take_image = 0;
    state->pic_num = 0;
    state->grad_dir_image = 0;
    state->show_homography = 0;
    state->max_grad_diff = 8;        // in degrees
    state->brightness = 20;
    state->min_mag = 6.5; 
    state->min_size = 200;
    state->blur_amount = 0;
    state->dilate_im = 0;
    state->connection_method = 4;
    state->qualify = 0;
    state->blob_detect = 1;
    // pg_add_int_slider(pg, "brightness", "Bright", 0, 150, state->brightness);
    // pg_add_int_slider(pg, "blur_amount", "Blur", 0, 10, state->blur_amount);
    // pg_add_double_slider(pg, "grad_error", "Grad Dir Error", 0, 360, state->max_grad_diff);
    // pg_add_double_slider(pg, "min_mag", "Min Magnitude", 0, 255, state->min_mag);
    pg_add_int_slider(pg, "min_size", "Size", 0, 1000, state->min_size);
    // pg_add_int_slider(pg, "connection_method", "CONMETH", 1, NUM_CON_METHODS, state->connection_method);

    pg_add_check_boxes(pg,
                        "add_lines", "Lines", 0, 
                        // "grad_image", "Show Grad", 0,
                        "dilate_im", "Dilate", 0, 
                        "blob_detect", "BD", 1, 
                        "qualify", "Q", 0, 
                        "grad_dir_image", "Show Grad Direction", 0,
                        "static_image", "Static Image", 1,
                        "take_image", "Take Image", 0,
                        "show_pix", "Show Pixels", 0,
                        "show_homography", "Show Homography", 0,

                                            NULL);
    pg_add_buttons(pg,
                   "next_image", "Next Image",
                   "next_hsv_calib", "Next HSV Calibration",
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






















    // // line intersection tester
    // loc_t l1p1 = {0,0};
    // loc_t l1p2 = {0 , 5};
    // loc_t l2p1 = {20, 0};
    // loc_t l2p2 = {0 , 20}; 
    // loc_t ans = get_line_intersection(l1p1, l1p2, l2p1, l2p2);
    // printf("int = (%d, %d)\n", ans.x, ans.y);
    // loc_t l3p1 = {5,0};
    // loc_t l3p2 = {5 , 10};  
    // ans = get_line_intersection(l1p1, l1p2, l3p1, l3p2);
    // printf("int = (%d, %d)\n", ans.x, ans.y);
    // return(0);





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
