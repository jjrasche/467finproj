#ifndef PG_H
#define PG_H

#include <gtk/gtk.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct parameter_gui parameter_gui_t;
typedef struct parameter_listener parameter_listener_t;


struct parameter_listener
{
    void * impl;
    void (*param_changed)(parameter_listener_t * listener, parameter_gui_t * pg, const char * keyname);
};


// This is a pretty hacked together parameter gui implementation
// which is missing many of the features of ParameterGUI.java,
// but is a first attempt to make an API that is concise enough for
// inline use

parameter_gui_t * pg_create(void);
void pg_destroy(parameter_gui_t *pg);

void pg_add_listener(parameter_gui_t *pg, parameter_listener_t * listener);
void pg_remove_listener(parameter_gui_t *pg, parameter_listener_t * listener);

GtkWidget * pg_get_widget(parameter_gui_t *pg);

int pg_add_double_slider(parameter_gui_t *pg, const char *name, const char * desc,
                         double min, double max, double intial_value);

int pg_add_int_slider(parameter_gui_t *pg, const char *name, const char * desc,
                      int min, int max, int intial_value);

// Accepts an arbitrary number of (char*,int) paired arguments. Requires
// a NULL terminator.
// WARNING: you must be sure to provide the correct number of arguments
// (must be 0 mod 3 + 1 including NULL), or you will get a segfault or hang
int pg_add_check_boxes(parameter_gui_t *pg, const char *name, const char * desc, int is_checked, ...) __attribute__((sentinel));
int pg_add_buttons(parameter_gui_t *pg, const char *name, const char * desc, ...) __attribute__((sentinel));

// Get Double
double pg_gd(parameter_gui_t *pg, const char *name);

// Set Double
void pg_sd(parameter_gui_t *pg, const char *name, double value);

// Get Integer
int pg_gi(parameter_gui_t *pg, const char *name);

// Set Integer
void pg_si(parameter_gui_t *pg, const char *name, int value);

// Get Boolean
int pg_gb(parameter_gui_t *pg, const char *name);

// Set Boolean
void pg_sb(parameter_gui_t *pg, const char *name, int value);

#ifdef __cplusplus
}
#endif


#endif
