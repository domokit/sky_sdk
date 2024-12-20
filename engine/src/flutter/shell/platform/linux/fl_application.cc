// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_application.h"

#include <gtk/gtk.h>
#ifdef GDK_WINDOWING_X11
#include <gdk/gdkx.h>
#endif

#include "flutter/shell/platform/linux/public/flutter_linux/fl_dart_project.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_plugin_registry.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_view.h"

struct FlApplicationPrivate {
  // Arguments to pass to Dart.
  gchar** dart_entrypoint_arguments;
};

#define FL_APPLICATION_GET_PRIVATE(app)                        \
  ((FlApplicationPrivate*)fl_application_get_instance_private( \
      FL_APPLICATION(app)))

enum { SIGNAL_REGISTER_PLUGINS, SIGNAL_CREATE_WINDOW, LAST_SIGNAL };

static guint fl_application_signals[LAST_SIGNAL];

G_DEFINE_TYPE_WITH_CODE(FlApplication,
                        fl_application,
                        GTK_TYPE_APPLICATION,
                        G_ADD_PRIVATE(FlApplication))

// Called when the first frame is received.
static void first_frame_cb(FlApplication* self, FlView* view) {
  GtkWidget* window = gtk_widget_get_toplevel(GTK_WIDGET(view));

  // Show the main window.
  if (window != nullptr && GTK_IS_WINDOW(window)) {
    gtk_window_present(GTK_WINDOW(window));
  }
}

// Default implementation of FlApplication::register_plugins
static void fl_application_register_plugins(FlApplication* self,
                                            FlPluginRegistry* registry) {}

// Default implementation of FlApplication::create_window
static GtkWindow* fl_application_create_window(FlApplication* self,
                                               FlView* view) {
  GtkApplicationWindow* window =
      GTK_APPLICATION_WINDOW(gtk_application_window_new(GTK_APPLICATION(self)));

  // Use a header bar when running in GNOME as this is the common style used
  // by applications and is the setup most users will be using (e.g. Ubuntu
  // desktop).
  // If running on X and not using GNOME then just use a traditional title bar
  // in case the window manager does more exotic layout, e.g. tiling.
  // If running on Wayland assume the header bar will work (may need changing
  // if future cases occur).
  gboolean use_header_bar = TRUE;
#ifdef GDK_WINDOWING_X11
  GdkScreen* screen = gtk_window_get_screen(GTK_WINDOW(window));
  if (GDK_IS_X11_SCREEN(screen)) {
    const gchar* wm_name = gdk_x11_screen_get_window_manager_name(screen);
    if (g_strcmp0(wm_name, "GNOME Shell") != 0) {
      use_header_bar = FALSE;
    }
  }
#endif
  if (use_header_bar) {
    GtkHeaderBar* header_bar = GTK_HEADER_BAR(gtk_header_bar_new());
    gtk_widget_show(GTK_WIDGET(header_bar));
    gtk_header_bar_set_show_close_button(header_bar, TRUE);
    gtk_window_set_titlebar(GTK_WINDOW(window), GTK_WIDGET(header_bar));
  }

  gtk_container_add(GTK_CONTAINER(window), GTK_WIDGET(view));

  return GTK_WINDOW(window);
}

// Implements GApplication::activate.
static void fl_application_activate(GApplication* application) {
  FlApplication* self = FL_APPLICATION(application);
  FlApplicationPrivate* priv = FL_APPLICATION_GET_PRIVATE(self);

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  fl_dart_project_set_dart_entrypoint_arguments(
      project, priv->dart_entrypoint_arguments);

  FlView* view = fl_view_new(project);
  g_signal_connect_swapped(view, "first-frame", G_CALLBACK(first_frame_cb),
                           self);
  gtk_widget_show(GTK_WIDGET(view));

  GtkWindow* window;
  g_signal_emit(self, fl_application_signals[SIGNAL_CREATE_WINDOW], 0, view,
                &window);

  // Make the resources for the view so rendering can start.
  // We'll show the view when we have the first frame.
  gtk_widget_realize(GTK_WIDGET(view));

  g_signal_emit(self, fl_application_signals[SIGNAL_REGISTER_PLUGINS], 0,
                FL_PLUGIN_REGISTRY(view));
}

// Implements GApplication::local_command_line.
static gboolean fl_application_local_command_line(GApplication* application,
                                                  gchar*** arguments,
                                                  int* exit_status) {
  FlApplication* self = FL_APPLICATION(application);
  FlApplicationPrivate* priv = FL_APPLICATION_GET_PRIVATE(self);

  // Strip out the first argument as it is the binary name.
  priv->dart_entrypoint_arguments = g_strdupv(*arguments + 1);

  g_autoptr(GError) error = nullptr;
  if (!g_application_register(application, nullptr, &error)) {
    g_warning("Failed to register: %s", error->message);
    *exit_status = 1;
    return TRUE;
  }

  // This will only run on the primary instance or this instance with
  // G_APPLICATION_NON_UNIQUE
  g_application_activate(application);
  *exit_status = 0;

  return TRUE;
}

// Implements GObject::dispose.
static void fl_application_dispose(GObject* object) {
  FlApplication* self = FL_APPLICATION(object);
  FlApplicationPrivate* priv = FL_APPLICATION_GET_PRIVATE(self);

  g_clear_pointer(&priv->dart_entrypoint_arguments, g_strfreev);

  G_OBJECT_CLASS(fl_application_parent_class)->dispose(object);
}

static void fl_application_class_init(FlApplicationClass* klass) {
  G_APPLICATION_CLASS(klass)->activate = fl_application_activate;
  G_APPLICATION_CLASS(klass)->local_command_line =
      fl_application_local_command_line;
  G_OBJECT_CLASS(klass)->dispose = fl_application_dispose;

  klass->register_plugins = fl_application_register_plugins;
  klass->create_window = fl_application_create_window;

  fl_application_signals[SIGNAL_REGISTER_PLUGINS] = g_signal_new(
      "register-plugins", fl_application_get_type(), G_SIGNAL_RUN_LAST,
      G_STRUCT_OFFSET(FlApplicationClass, register_plugins), nullptr, nullptr,
      nullptr, G_TYPE_NONE, 1, fl_plugin_registry_get_type());
  fl_application_signals[SIGNAL_CREATE_WINDOW] = g_signal_new(
      "create-window", fl_application_get_type(), G_SIGNAL_RUN_LAST,
      G_STRUCT_OFFSET(FlApplicationClass, create_window),
      g_signal_accumulator_first_wins, nullptr, nullptr, GTK_TYPE_WINDOW, 1,
      fl_view_get_type());
}

static void fl_application_init(FlApplication* self) {}

G_MODULE_EXPORT
FlApplication* fl_application_new(const gchar* application_id,
                                  GApplicationFlags flags) {
  return FL_APPLICATION(g_object_new(fl_application_get_type(),
                                     "application-id", application_id, "flags",
                                     flags, nullptr));
}
