#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "pg.h"
#include "param_widget.h"


G_BEGIN_DECLS

#define GTK_TYPE_PARAM_GUI  gtk_param_gui_get_type()
#define GTK_PARAM_GUI(obj)  (G_TYPE_CHECK_INSTANCE_CAST ((obj), \
        GTK_TYPE_PARAM_GUI, ParameterGUI))
#define GTK_PARAM_GUI_CLASS(klass) (G_TYPE_CHECK_CLASS_CAST ((klass), \
            GTK_TYPE_PARAM_GUI, ParameterGUIClass))

typedef struct _ParameterGUI ParameterGUI;
typedef struct _ParameterGUIClass ParameterGUIClass;

G_END_DECLS

struct parameter_gui
{
    ParameterGUI * paramWidget;

    //zhash_t listeners; // <parameter_listener_t *, parameter_listener_t *>
};


// debuggine macros
#define ENABLE_DEBUG 0
#define DEBUG(...) do { if(ENABLE_DEBUG) fprintf(stderr, __VA_ARGS__); } while(0)
#define ERROR(...) fprintf(stderr, __VA_ARGS__)

#define SLIDER_STEP_COUNT 1000

// Forward Declarations
//static int add_check_boxes(parameter_gui_t *pg, const char *name, int *is_checked, ...) __attribute__((sentinel));
static double gtk_param_gui_get_double (ParameterGUI *pg, const char *name);
static int gtk_param_gui_get_int (ParameterGUI *pg, const char *name);
static int gtk_param_gui_get_boolean (ParameterGUI *pg, const char *name);

struct _ParameterGUI
{
    GtkVBox vbox;

    /* private */
    GHashTable *params;
    GHashTable *widget_to_param;

    GList *widgets;
};

struct _ParameterGUIClass
{
    GtkVBoxClass parent_class;
};

typedef struct _param_data {
    char *name;
    char * desc;
    GtkWidget *widget;

    GType data_type;

    double min_double;
    double max_double;
} param_data_t;

static param_data_t *param_data_new(const char *name, const char * desc, GtkWidget *widget, GType data_type)
{
    param_data_t *pdata = g_slice_new(param_data_t);
    pdata->name = strdup(name);
    pdata->desc = strdup(desc);
    pdata->widget = widget;
    pdata->data_type = data_type;

    return pdata;
}

static void param_data_free(param_data_t *pdata)
{
    free(pdata->name);
    free(pdata->desc);
    g_slice_free(param_data_t, pdata);
}

/** GTK required functions **/

enum {
    CHANGED_SIGNAL,
    LAST_SIGNAL
};

static void gtk_param_gui_class_init (ParameterGUIClass *klass);
static void gtk_param_gui_init (ParameterGUI *pg);
static void gtk_param_gui_finalize (GObject *obj);
static guint gtk_param_gui_signals[LAST_SIGNAL] = { 0 };

G_DEFINE_TYPE (ParameterGUI, gtk_param_gui, GTK_TYPE_VBOX)

static void gtk_param_gui_class_init (ParameterGUIClass *klass)
{
    GObjectClass *gobject_class = G_OBJECT_CLASS(klass);
    gobject_class->finalize = gtk_param_gui_finalize;

    gtk_param_gui_signals[CHANGED_SIGNAL] = g_signal_new("changed",
            G_TYPE_FROM_CLASS(klass),
            G_SIGNAL_RUN_FIRST | G_SIGNAL_ACTION,
            0,
            NULL,
            NULL,
            gtk_marshal_VOID__STRING,
            G_TYPE_NONE, 1, G_TYPE_STRING);
}

static void gtk_param_gui_init (ParameterGUI *pg)
{
    DEBUG ("%s:%d %s\n", __FILE__, __LINE__, __FUNCTION__);

    pg->widgets = NULL;
    pg->params = g_hash_table_new(g_str_hash, g_str_equal);
    pg->widget_to_param = g_hash_table_new_full(g_direct_hash, g_direct_equal,
            NULL, (GDestroyNotify)param_data_free);
}

static void gtk_param_gui_finalize (GObject *obj)
{
    DEBUG ("%s:%d %s\n", __FILE__, __LINE__, __FUNCTION__);

    G_OBJECT_CLASS (gtk_param_gui_parent_class)->finalize(obj);
}

static GtkWidget *gtk_param_gui_new (void)
{
    return GTK_WIDGET (g_object_new (GTK_TYPE_PARAM_GUI, NULL));
}

static int have_parameter_key (ParameterGUI *pg, const char *name)
{
    return g_hash_table_lookup (pg->params, name) != NULL;
}

static void generic_widget_changed (GtkWidget *w, gpointer user_data)
{
    ParameterGUI *pg = GTK_PARAM_GUI(user_data);
    param_data_t *pd =
        (param_data_t*) g_hash_table_lookup (pg->widget_to_param, w);
    g_signal_emit (G_OBJECT (pg),
            gtk_param_gui_signals[CHANGED_SIGNAL], 0, pd->name);

}

static param_data_t *add_row (ParameterGUI *pg, const char *name, const char * desc,
                              GtkWidget *w, const char *signal_name, GType data_type)
{
    GtkWidget *hb = gtk_hbox_new (FALSE, 0);
    GtkWidget *lbl = gtk_label_new (desc);
    gtk_box_pack_start (GTK_BOX (hb), lbl, FALSE, FALSE, 0);
    gtk_box_pack_start (GTK_BOX (hb), w, TRUE, TRUE, 0);
    gtk_box_pack_start (GTK_BOX (&pg->vbox), hb, TRUE, TRUE, 0);
    pg->widgets = g_list_append (pg->widgets, hb);
    pg->widgets = g_list_append (pg->widgets, lbl);
    pg->widgets = g_list_append (pg->widgets, w);

    g_hash_table_insert (pg->params, (gpointer)name, w);

    param_data_t *pdata = param_data_new(name, desc, w, data_type);
    g_hash_table_insert (pg->widget_to_param, w, pdata);

    if (signal_name && strlen (signal_name)) {
        g_signal_connect (G_OBJECT(w), signal_name,
                G_CALLBACK (generic_widget_changed), pg);
    }

    gtk_widget_show_all (hb);
    return pdata;
}

int pg_add_double_slider(parameter_gui_t *pg, const char *name,
                         const char * desc,
                         double min, double max, double var)
{
    ParameterGUI *gtkpg = GTK_PARAM_GUI(pg->paramWidget);

    if (have_parameter_key (gtkpg, name)) {
        ERROR("ERROR: parameter named '%s' already in the ParameterGUI", name);
        return -1;
    }

    if (min >= max || var < min || var > max) {
        ERROR("WARNING: param_widget_add_double - invalid args\n");
        return -1;
    }

    double inc = (max - min) / (double)SLIDER_STEP_COUNT;
    GtkWidget *w = gtk_hscale_new_with_range (min, max, inc);
    gtk_range_set_value (GTK_RANGE(w), var);

    param_data_t * pdata = add_row (gtkpg, name, desc, w, "value-changed", G_TYPE_DOUBLE);
    pdata->min_double = min;
    pdata->max_double = max;
    g_object_set_data (G_OBJECT(w), "data-type", "double");

    return 0;
}

int pg_add_int_slider(parameter_gui_t *pg, const char *name,
                      const char * desc,
                      int min, int max, int var)
{
    ParameterGUI *gtkpg = GTK_PARAM_GUI(pg->paramWidget);

    if (have_parameter_key (gtkpg, name)) {
        ERROR("ERROR: parameter named '%s' already in the ParameterGUI", name);
        return -1;
    }

    if (min >= max || var < min || var > max) {
        ERROR("WARNING: param_widget_int_slider - invalid args\n");
        return -1;
    }

    double inc = 1;
    GtkWidget *w = gtk_hscale_new_with_range (min, max, inc);
    gtk_range_set_value (GTK_RANGE(w), var);

    g_object_set_data (G_OBJECT(w), "data-type", "int");
    add_row (gtkpg, name, desc, w, "value-changed", G_TYPE_INT);
    return 0;
}

static int add_checkboxes_helper (ParameterGUI *pg, GtkBox *box,
                                  const char *name, const char * desc, int is_checked)
{
    if (have_parameter_key (pg, name)) {
        ERROR("ERROR: parameter named '%s' already in the ParameterGUI", name);
        return -1;
    }

    GtkWidget *cb = gtk_check_button_new_with_label (desc);
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (cb), is_checked);

    gtk_box_pack_start (GTK_BOX (box), cb, FALSE, FALSE, 0);

    pg->widgets = g_list_append (pg->widgets, cb);

    g_hash_table_insert (pg->params, (gpointer)name, cb);
    g_object_set_data (G_OBJECT(cb), "data-type", "boolean");

    param_data_t *pd = param_data_new(name, desc, cb, G_TYPE_BOOLEAN);
    g_hash_table_insert (pg->widget_to_param, cb, pd);

    g_signal_connect (G_OBJECT(cb), "toggled",
            G_CALLBACK (generic_widget_changed), pg);
    return 0;
}

int pg_add_check_boxes(parameter_gui_t *pg, const char *name, const char * desc,
                       int is_checked, ...)
{
    ParameterGUI *gtkpg = GTK_PARAM_GUI(pg->paramWidget);

    va_list va;
    va_start (va, is_checked);

    GtkBox *hb = GTK_BOX (gtk_hbox_new (FALSE, 0));

    while(1){

        if(add_checkboxes_helper(gtkpg, hb, name, desc, is_checked) != 0)
            return -1;

        name = va_arg(va, const char *);
        if(name == NULL || strlen(name) == 0)
            break; // all done

        desc = va_arg(va, const char *);
        is_checked = va_arg(va, int);
    }

    gtk_widget_show_all (GTK_WIDGET (hb));
    gtk_box_pack_start (GTK_BOX(&gtkpg->vbox), GTK_WIDGET(hb), TRUE, TRUE, 0);

    va_end(va);
    return 0;
}

static double gtk_param_gui_get_double (ParameterGUI *pg, const char *name)
{
    if(!have_parameter_key(pg, name)) {
        fprintf(stderr, "param_widget: invalid parameter [%s]\n", name);
        return 0;
    }

    GtkWidget *w = g_hash_table_lookup(pg->params, name);
    return (double)gtk_range_get_value (GTK_RANGE (w));
}

double pg_gd(parameter_gui_t *pg, const char *name)
{
    ParameterGUI *gtkpg = GTK_PARAM_GUI(pg->paramWidget);
    return gtk_param_gui_get_double(gtkpg, name);
}

void pg_sd(parameter_gui_t *pg, const char *name, double value)
{
    assert(0 && "Unimplemented");
}

static int gtk_param_gui_get_int (ParameterGUI *pg, const char *name)
{
    if(!have_parameter_key(pg, name)) {
        fprintf(stderr, "param_widget: invalid parameter [%s]\n", name);
        return 0;
    }

    GtkWidget *w = g_hash_table_lookup(pg->params, name);
    return (int)gtk_range_get_value (GTK_RANGE (w));
}

int pg_gi(parameter_gui_t *pg, const char *name)
{
    ParameterGUI *gtkpg = GTK_PARAM_GUI(pg->paramWidget);
    return gtk_param_gui_get_int(gtkpg, name);
}

void pg_si(parameter_gui_t *pg, const char *name, int value)
{
    assert(0 && "Unimplemented");
}

static int gtk_param_gui_get_boolean (ParameterGUI *pg, const char *name)
{
    if(!have_parameter_key(pg, name)) {
        fprintf(stderr, "param_widget: invalid parameter [%s]\n", name);
        return 0;
    }

    GtkWidget *w = g_hash_table_lookup(pg->params, name);
    return gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON (w));
}

int pg_gb(parameter_gui_t *pg, const char *name)
{
    ParameterGUI *gtkpg = GTK_PARAM_GUI(pg->paramWidget);
    return gtk_param_gui_get_boolean(gtkpg, name);
}

void pg_sb(parameter_gui_t *pg, const char *name, int value)
{
    assert(0 && "Unimplemented");
}


parameter_gui_t *pg_create(void)
{
    parameter_gui_t * pg = calloc(1,sizeof(parameter_gui_t));

    pg->paramWidget = GTK_PARAM_GUI(gtk_param_gui_new());
    g_object_ref_sink(pg->paramWidget); // convert floating refernce to actual reference

    return pg;
}


void pg_destroy(parameter_gui_t *pg)
{
    gtk_widget_destroy (GTK_WIDGET(pg->paramWidget));
    g_object_unref (pg->paramWidget);
}

GtkWidget * pg_get_widget(parameter_gui_t *pg)
{
    return  GTK_WIDGET(pg->paramWidget);

}