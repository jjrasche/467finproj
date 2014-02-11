
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <lcm/lcm.h>
#include "vx/vx.h"
#include "vx/vxo_drawables.h"
#include "vx/vx_remote_display_source.h"

typedef struct
{
    vx_application_t app;

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

    pthread_mutex_lock(&state->mutex);
    // store a reference to the world and layer that we associate with each vx_display_t
    zhash_put(state->layer_map, &disp, &layer, NULL, NULL);
    pthread_mutex_unlock(&state->mutex);
}


int main()
{
    vx_global_init();

    state_t * state = calloc(1, sizeof(state_t));
    state->app.display_finished = display_finished;
    state->app.display_started = display_started;
    state->app.impl = state;
    state->lcm = lcm_create(NULL);
    state->vw = vx_world_create();
    pthread_mutex_init(&state->mutex, NULL);
    state->layer_map = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);

    if (1) {
        vx_object_t *vt = vxo_text_create(VXO_TEXT_ANCHOR_TOP_RIGHT, "<<right,#0000ff>>Robot viewer!\n");
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "text");
        vx_buffer_add_back(vb, vxo_pix_coords(VX_ORIGIN_TOP_RIGHT,vt));
        vx_buffer_swap(vb);
    }


    vx_remote_display_source_t * remote = vx_remote_display_source_create(&state->app);

    // block forever
    while (1) sleep(1);

    vx_remote_display_source_destroy(remote);
}
