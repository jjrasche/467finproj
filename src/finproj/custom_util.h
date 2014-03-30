#ifndef EECS467_UTIL_H
#define EECS467_UTIL_H

// XXX This is evil
#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"

#include "vx/gtk/vx_gtk_display_source.h"
#include "vx/vx_remote_display_source.h"

#include "vx/vx_event_handler.h"

// drawables
#include "vx/vxo_drawables.h"

#include "common/getopt.h"
#include "common/image_u32.h"
#include "common/pg.h"
#include "common/zarray.h"

// imagesource
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

typedef struct custom_default_implementation custom_default_implementation_t;

custom_default_implementation_t *custom_default_implementation_create();

void custom_default_add_layer(vx_application_t* app, vx_layer_t* layer);
void custom_default_set_layer_viewport(vx_application_t* app, vx_layer_t* layer, 
		                               float* viewport_rel);

void custom_default_display_started(vx_application_t *app, vx_display_t *disp);
void custom_default_display_finished(vx_application_t *app, vx_display_t *disp);

void custom_init(int argc, char **argv);

// Blocks
void custom_gui_run(vx_application_t *app, parameter_gui_t *pg, int w, int h);

#endif
