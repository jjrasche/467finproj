#include "eecs467_util.h"    // This is where a lot of the internals live

// It's good form for every application to keep its state in a struct.
typedef struct state state_t;
struct state
{
    char *url;
    int running;

    vx_application_t  vxapp;
    getopt_t         *gopt;
    parameter_gui_t  *pg;

    vx_world_t *world;  // Where vx objects are live

    zhash_t *layers;

    pthread_mutex_t mutex;  // for accessing the arrays
    pthread_t animate_thread;
};

// === Parameter listener =================================================
// This function is handled to the parameter gui (via a parameter listener)
// and handles events coming from the parameter gui. The parameter listener
// also holds a void* pointer to "impl", which can point to a struct holding
// state, etc if need be.
void my_param_changed(parameter_listener_t *pl, parameter_gui_t *pg, const char *name)
{
    if (!strcmp("sl1", name)) {
        printf("sl1 = %f\n", pg_gd(pg, name));
    } else if (!strcmp("sl2", name)) {
        printf("sl2 = %d\n", pg_gi(pg, name));
    } else if (!strcmp("cb1", name) | !strcmp("cb2", name)) {
        printf("%s = %d\n", name, pg_gb(pg, name));
    } else {
        printf("%s changed\n", name);
    }
}

// === Your code goes here ================================================
// The render loop handles your visualization updates. It is the function run
// by the animate_thread. It periodically renders the contents on the
// vx world contained by state
void* render_loop(void *data)
{
    int fps = 60;
    state_t *state = data;

    // Set up the imagesource
    image_source_t *isrc = image_source_open(state->url);

    if (isrc == NULL) {
        printf("Error opening device.\n");
    } else {
        // Print out possible formats. If no format was specified in the
        // url, then format 0 is picked by default.
        // e.g. of setting the format parameter to format 2:
        //
        // --url=dc1394://bd91098db0as9?fidx=2
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

        // Get the most recent camera frame and render it to screen.
        if (isrc != NULL) {
            image_source_data_t * frmd = calloc(1, sizeof(image_source_data_t));
            int res = isrc->get_frame(isrc, frmd);
            if (res < 0) {
                printf("get_frame fail: %d\n", res);
            } else {
                // Handle frame
                image_u32_t *im = image_convert_u32(frmd);
                if (im != NULL) {

                    vx_object_t *vim = vxo_image_from_u32(im,
                                                          VXO_IMAGE_FLIPY,
                                                          VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);

                    vx_buffer_add_back(vx_world_get_buffer(state->world, "image"),
                                       vxo_chain(vxo_mat_translate3(-im->width/2,-im->height/2,0),
                                                 vim));
                    vx_buffer_swap(vx_world_get_buffer(state->world, "image"));
                    image_u32_destroy(im);
                }
            }

            fflush(stdout);
            isrc->release_frame(isrc, frmd);
        }

        // Example rendering of vx primitives
        double rad = (vx_mtime() % 5000) * 2 * M_PI / 5e3;   // [0,2PI]
        double osc = ((vx_mtime() % 5000) / 5e3) * 2 - 1;    // [-1, 1]

        // Creates a blue box and applies a series of rigid body transformations
        // to it. A vxo_chain applies its arguments sequentially. In this case,
        // then, we rotate our coordinate frame by rad radians, as determined
        // by the current time above. Then, the origin of our coordinate frame
        // is translated 0 meters along its X-axis and 10 meters along its
        // Y-axis. Finally, a 1x1x1 cube (or box) is rendered centered at the
        // origin, and is rendered with the blue mesh style, meaning it has
        // solid, blue sides.
        vx_object_t *vo = vxo_chain(vxo_mat_rotate_z(rad),
                                    vxo_mat_translate2(0,10),
                                    vxo_sphere(vxo_mesh_style(vx_blue)));

        // Then, we add this object to a buffer awaiting a render order
        vx_buffer_add_back(vx_world_get_buffer(state->world, "rot-sphere"), vo);

        // Now we will render a red box that translates back and forth. This
        // time, there is no rotation of our coordinate frame, so the box will
        // just slide back and forth along the X axis. This box is rendered
        // with a red line style, meaning it will appear as a red wireframe,
        // in this case, with lines 2 px wide.
        vo = vxo_chain(vxo_mat_translate2(osc*5,0),
                       vxo_box(vxo_lines_style(vx_red, 2)));

        // We add this object to a different buffer so it may be rendered
        // separately if desired
        vx_buffer_add_back(vx_world_get_buffer(state->world, "osc-square"), vo);

        // Now, we update both buffers
        vx_buffer_swap(vx_world_get_buffer(state->world, "rot-sphere"));
        vx_buffer_swap(vx_world_get_buffer(state->world, "osc-square"));

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
    eecs467_init(argc, argv);

    state_t *state = calloc(1, sizeof(state_t));
    state->world = vx_world_create();

    state->vxapp.display_started = eecs467_default_display_started;
    state->vxapp.display_finished = eecs467_default_display_finished;
    state->vxapp.impl = eecs467_default_implementation_create(state->world);

    state->running = 1;

    // Parse arguments from the command line, showing the help
    // screen if required
    state->gopt = getopt_create();
    getopt_add_bool(state->gopt, 'h', "help", 0, "Show help");
    getopt_add_string(state->gopt, '\0', "url", "", "Camera URL");

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
    pg_add_double_slider(pg, "sl1", "Slider 1", 0, 100, 50);
    pg_add_int_slider(pg, "sl2", "Slider 2", 0, 100, 25);
    pg_add_check_boxes(pg,
                       "cb1", "Check Box 1", 0,
                       "cb2", "Check Box 2", 1,
                       NULL);
    pg_add_buttons(pg,
                   "but1", "Button 1",
                   "but2", "Button 2",
                   "but3", "Button 3",
                   NULL);

    parameter_listener_t *my_listener = calloc(1,sizeof(parameter_listener_t*));
    my_listener->impl = NULL;
    my_listener->param_changed = my_param_changed;
    pg_add_listener(pg, my_listener);

    state->pg = pg;

    eecs467_gui_run(&state->vxapp, state->pg, 800, 600);
    // Quit when GTK closes
    state->running = 0;

    pthread_join(state->animate_thread, NULL);
    vx_remote_display_source_destroy(cxn);

    // Cleanup
    //free(my_listener);

    vx_global_destroy();
    getopt_destroy(state->gopt);
}
