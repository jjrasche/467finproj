#include "custom_util.h"

struct custom_default_implementation
{
    pthread_mutex_t mutex;

    // Single display for vxapp
    vx_display_t* display;

    // 1-1 layer-world
    zarray_t* layers;
	zhash_t* layer_rel_viewports;
};

custom_default_implementation_t *custom_default_implementation_create()
{
    //Initialize the default implementation
    custom_default_implementation_t *impl = calloc(1, sizeof(custom_default_implementation_t));

    impl->display = NULL;
    impl->layers = zarray_create(sizeof(vx_layer_t*));
	impl->layer_rel_viewports = zhash_create(sizeof(vx_layer_t*), 4 * sizeof(float), zhash_ptr_hash, 
			                                 zhash_ptr_equals);

    pthread_mutex_init(&impl->mutex, NULL);
    
    return impl;
}

void custom_default_add_layer(vx_application_t* app, vx_layer_t* layer) {
    custom_default_implementation_t *impl = app->impl;
    pthread_mutex_lock(&impl->mutex);
    vx_display_t* disp = impl->display;
    if(disp != NULL) {
        vx_layer_set_draw_order(layer, zarray_size(impl->layers));
        vx_layer_set_display(layer, disp);
    }
    zarray_add(impl->layers, &layer);
    pthread_mutex_unlock(&impl->mutex);
}

void custom_default_set_layer_viewport(vx_application_t* app, vx_layer_t* layer, 
		                               float* viewport_rel) {
    custom_default_implementation_t *impl = app->impl;

	zhash_put(impl->layer_rel_viewports, &layer, viewport_rel, NULL, NULL);
}

void custom_default_display_started(vx_application_t *app, vx_display_t *disp)
{
    custom_default_implementation_t *impl = app->impl;

    pthread_mutex_lock(&impl->mutex);

    for(int i = 0; i < zarray_size(impl->layers); i++) {
        vx_layer_t *layer = NULL;

        zarray_get(impl->layers, i, &layer);
       
        vx_layer_set_draw_order(layer, i);
        vx_layer_set_display(layer, disp);

		if(zhash_contains(impl->layer_rel_viewports, &layer)) {
			float *viewport = calloc(4, sizeof(float));
			zhash_get(impl->layer_rel_viewports, &layer, viewport);

			vx_layer_set_viewport_rel(layer, viewport);
		}
    }

    pthread_mutex_unlock(&impl->mutex);
}

void custom_default_display_finished(vx_application_t *app, vx_display_t *disp)
{
    custom_default_implementation_t *impl = app->impl;
    pthread_mutex_lock(&impl->mutex);

    for(int i = 0; i < zarray_size(impl->layers); i++) {
        vx_layer_t* layer = NULL;

        zarray_get(impl->layers, i, &layer);     
        
        vx_layer_destroy(layer);
    }

    zarray_destroy(impl->layers);

    pthread_mutex_unlock(&impl->mutex);
}

void custom_init(int argc, char **argv)
{
    // on newer GTK systems, this might generate an error/warning
    g_type_init();

    // Initialize GTK
    gdk_threads_init();
    gdk_threads_enter();
    gtk_init(&argc, &argv);

    vx_global_init();
}

void custom_gui_run(vx_application_t *app, parameter_gui_t *pg, int w, int h)
{
    // Creates a GTK window to wrap around our vx display canvas. The vx world
    // is rendered to the canvas widget, which acts as a viewport into your
    // virtual world.
    vx_gtk_display_source_t *appwrap = vx_gtk_display_source_create(app);
    GtkWidget *window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    GtkWidget *canvas = vx_gtk_display_source_get_widget(appwrap);
    GtkWidget *pgui = pg_get_widget(pg);
    gtk_window_set_default_size(GTK_WINDOW(window), w, h);

    // Pack a parameter gui and canvas into a vertical box
    GtkWidget *vbox = (GtkWidget*)gtk_vbox_new(0, 0);
    gtk_box_pack_start(GTK_BOX(vbox), canvas, 1, 1, 0);
    gtk_widget_show(canvas);    // XXX Show all causes errors!
    gtk_box_pack_start(GTK_BOX(vbox), pgui, 0, 0, 0);
    gtk_widget_show(pgui);

    gtk_container_add(GTK_CONTAINER(window), vbox);
    gtk_widget_show(window);
    gtk_widget_show(vbox);

    g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);

    gtk_main(); // Blocks as long as GTK window is open
    gdk_threads_leave();

    vx_gtk_display_source_destroy(appwrap);
}

