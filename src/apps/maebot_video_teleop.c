
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm.h>
#include <signal.h>

#include "vx/vx.h"
#include "vx/vxo_drawables.h"
#include "vx/vx_remote_display_source.h"

#include "common/getopt.h"
#include "common/image_u8x3.h"
#include "common/timestamp.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "lcmtypes/maebot_command_t.h"

// XXX these need to be fixed based on actual spec
#define MAX_REVERSE_SPEED -32768
#define MAX_FORWARD_SPEED 32767

typedef struct
{
    vx_application_t app;
    vx_event_handler_t veh;

    int running;

    getopt_t * gopt;
    char * url;
    image_source_t *isrc;
    int fidx;

    lcm_t * lcm;

    pthread_mutex_t mutex;

    vx_world_t * vw;
    zhash_t *layer_map; // <display, layer>
} state_t;


static void display_finished(vx_application_t * app, vx_display_t * disp)
{
    state_t * state = app->impl;
    pthread_mutex_lock(&state->mutex);

    vx_layer_t * layer = NULL;

    // store a reference to the world and layer that we associate with each vx_display_t
    zhash_remove(state->layer_map, &disp, NULL, &layer);

    vx_layer_destroy(layer);

    pthread_mutex_unlock(&state->mutex);
}

static void display_started(vx_application_t * app, vx_display_t * disp)
{
    state_t * state = app->impl;

    vx_layer_t * layer = vx_layer_create(state->vw);
    vx_layer_set_display(layer, disp);
    vx_layer_add_event_handler(layer, &state->veh);

    pthread_mutex_lock(&state->mutex);
    // store a reference to the world and layer that we associate with each vx_display_t
    zhash_put(state->layer_map, &disp, &layer, NULL, NULL);
    pthread_mutex_unlock(&state->mutex);
}


void* run_camera(void * data)
{

    state_t * state = data;
    image_source_t *isrc = state->isrc;

    while (state->running) {

        image_u32_t *im = NULL;

        {
            image_source_data_t isdata;

            int res = isrc->get_frame(isrc, &isdata);
            if (!res) {
                im = image_convert_u32(&isdata);
            }
            if (res)
                goto error;

            isrc->release_frame(isrc, &isdata);
        }

        if (im != NULL) {

            vx_object_t * vo = vxo_image_from_u32(im, VXO_IMAGE_FLIPY, VX_TEX_MIN_FILTER);
            // XXX We may want to downsample the image eventually
            vx_buffer_t *vb = vx_world_get_buffer(state->vw, "image");
            vx_buffer_add_back(vb, vxo_pix_coords(VX_ORIGIN_TOP_LEFT,
                                                  vxo_chain (vxo_mat_translate3 (0, -im->height, 0),
                                                             vo)));
            vx_buffer_swap(vb);
        }

        image_u32_destroy(im);

    }

  error:
    isrc->stop(isrc);
    printf("exiting\n");
    return NULL;

}

static int touch_event (vx_event_handler_t * vh, vx_layer_t * vl, vx_camera_pos_t * pos, vx_touch_event_t * mouse)
{
    return 0;
}
static int mouse_event (vx_event_handler_t * vh, vx_layer_t * vl, vx_camera_pos_t * pos, vx_mouse_event_t * mouse)
{
    return 0;
}

static int key_event (vx_event_handler_t * vh, vx_layer_t * vl, vx_key_event_t * key)
{
    state_t *state = vh->impl;

    maebot_command_t cmd;
    memset(&cmd, 0, sizeof(maebot_command_t));

    cmd.timestamp = utime_now();


    if (!key->released) {
        if (key->key_code == 'w' || key->key_code == 'W') {
            // forward
            cmd.motor_left_speed = MAX_FORWARD_SPEED;
            cmd.motor_right_speed = MAX_FORWARD_SPEED;
        } else if (key->key_code == 'a' || key->key_code == 'A' ) {
            // turn left
            cmd.motor_left_speed = MAX_REVERSE_SPEED;
            cmd.motor_right_speed = MAX_FORWARD_SPEED;

        } else if (key->key_code == 's' || key->key_code == 'S') {
            // reverse
            cmd.motor_left_speed = MAX_REVERSE_SPEED;
            cmd.motor_right_speed = MAX_REVERSE_SPEED;
        } else if (key->key_code == 'd' || key->key_code == 'D') {
            // turn right
            cmd.motor_left_speed = MAX_REVERSE_SPEED;
            cmd.motor_right_speed = MAX_FORWARD_SPEED;
        }
    } else {
        // when key released, speeds default to 0
        cmd.motor_left_speed = MAX_REVERSE_SPEED;
        cmd.motor_right_speed = MAX_FORWARD_SPEED;
    }

    maebot_command_t_publish(state->lcm,  "MAEBOT_COMMAND", &cmd);
    return 0;
}

static void nodestroy (vx_event_handler_t * vh)
{
    // do nothing, since this event handler is statically allocated.
}

static state_t * global_state;
static void handler(int signum)
{
    switch (signum)
    {
        case SIGINT:
        case SIGQUIT:
            global_state->running = 0;
            break;
        default:
            break;
    }
}

int main(int argc, char ** argv)
{
    vx_global_init();



    state_t * state = calloc(1, sizeof(state_t));
    global_state = state;
    state->gopt = getopt_create();
    state->app.display_finished = display_finished;
    state->app.display_started = display_started;
    state->app.impl = state;
    state->veh.dispatch_order = -10;
    state->veh.touch_event = touch_event;
    state->veh.mouse_event = mouse_event;
    state->veh.key_event = key_event;
    state->veh.destroy = nodestroy;
    state->veh.impl = state;

    state->running = 1;
    state->lcm = lcm_create(NULL);
    state->vw = vx_world_create();
    pthread_mutex_init(&state->mutex, NULL);
    state->layer_map = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);

    signal(SIGINT, handler);
    signal(SIGQUIT, handler);


    getopt_add_bool(state->gopt, 'h', "--help", 0, "Show this help");

    if (!getopt_parse(state->gopt, argc, argv, 0)) {
        getopt_do_usage(state->gopt);
        exit(-1);
    }

    const zarray_t *args = getopt_get_extra_args(state->gopt);
    if (zarray_size(args) > 0) {
        zarray_get(args, 0, &state->url);
    } else {
        zarray_t *urls = image_source_enumerate();

        printf("Cameras:\n");
        for (int i = 0; i < zarray_size(urls); i++) {
            char *url;
            zarray_get(urls, i, &url);
            printf("  %3d: %s\n", i, url);
        }

        if (zarray_size(urls) == 0) {
            printf("No cameras found.\n");
            exit(0);
        }
        zarray_get(urls, 0, &state->url);
    }


    if (1) {
        vx_object_t *vt = vxo_text_create(VXO_TEXT_ANCHOR_TOP_RIGHT, "<<right,#0000ff>>Robot viewer!\n");
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "text");
        vx_buffer_add_back(vb, vxo_pix_coords(VX_ORIGIN_TOP_RIGHT,vt));
        vx_buffer_swap(vb);
    }


    vx_remote_display_source_t * remote = vx_remote_display_source_create(&state->app);



    state->isrc = image_source_open(state->url);
    if (state->isrc == NULL) {
        printf("Unable to open device %s\n", state->url);
        exit(-1);
    }

    image_source_t *isrc = state->isrc;

    if (isrc->start(isrc))
        exit(-1);
    run_camera(state);

    isrc->close(isrc);

    vx_remote_display_source_destroy(remote);
}