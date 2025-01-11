// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "my_application.h"

#include <flutter_linux/flutter_linux.h>

#ifdef GDK_WINDOWING_X11
#include <gdk/gdkx.h>
#endif

#include "flutter/generated_plugin_registrant.h"

struct _MyApplication {
  GtkApplication parent_instance;

  // Channel to receive platform view requests from Flutter.
  FlMethodChannel* platform_view_channel;

  // Main window.
  GtkWindow* window;

  // Current counter.
  int64_t counter;

  // Request in progress.
  FlMethodCall* method_call;

  // Native window requested by Flutter.
  GtkWindow* native_window;

  // Label to show count.
  GtkLabel* counter_label;
};

G_DEFINE_TYPE(MyApplication, my_application, GTK_TYPE_APPLICATION)

static void update_counter_label(MyApplication* self) {
  g_autofree gchar* text =
      g_strdup_printf("Button tapped %" G_GINT64_FORMAT " %s.", self->counter,
                      self->counter == 1 ? "time" : "times");
  gtk_label_set_text(self->counter_label, text);
}

static void button_clicked_cb(MyApplication* self) {
  self->counter++;
  update_counter_label(self);
}

static void native_window_delete_event_cb(MyApplication* self,
                                          gint response_id) {
  g_autoptr(FlValue) counter_value = fl_value_new_int(self->counter);
  fl_method_call_respond_success(self->method_call, counter_value, nullptr);
  g_clear_object(&self->method_call);
}

// Handle request to switch the view.
static void handle_switch_view(MyApplication* self, FlMethodCall* method_call) {
  FlValue* counter_value = fl_method_call_get_args(method_call);
  if (fl_value_get_type(counter_value) != FL_VALUE_TYPE_INT) {
    fl_method_call_respond_error(self->method_call, "Invalid args",
                                 "Invalid switchView args", nullptr, nullptr);
    return;
  }

  self->counter = fl_value_get_int(counter_value);
  self->method_call = FL_METHOD_CALL(g_object_ref(method_call));

  // Show the same UI in a native window.
  self->native_window = GTK_WINDOW(gtk_window_new(GTK_WINDOW_TOPLEVEL));
  gtk_window_set_transient_for(self->native_window, self->window);
  gtk_window_set_modal(self->native_window, TRUE);
  gtk_window_set_destroy_with_parent(self->native_window, TRUE);
  g_signal_connect_swapped(self->native_window, "delete-event",
                           G_CALLBACK(native_window_delete_event_cb), self);

  GtkWidget* box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 12);
  gtk_widget_set_margin_start(box, 24);
  gtk_widget_set_margin_end(box, 24);
  gtk_widget_set_margin_top(box, 24);
  gtk_widget_set_margin_bottom(box, 24);
  gtk_widget_show(box);
  gtk_container_add(GTK_CONTAINER(self->native_window), box);

  self->counter_label = GTK_LABEL(gtk_label_new(""));
  gtk_widget_show(GTK_WIDGET(self->counter_label));
  gtk_container_add(GTK_CONTAINER(box), GTK_WIDGET(self->counter_label));

  GtkWidget* button = gtk_button_new_with_label("+");
  gtk_style_context_add_class(gtk_widget_get_style_context(GTK_WIDGET(button)),
                              "circular");
  gtk_widget_set_halign(button, GTK_ALIGN_CENTER);
  gtk_widget_show(button);
  gtk_container_add(GTK_CONTAINER(box), button);
  g_signal_connect_swapped(button, "clicked", G_CALLBACK(button_clicked_cb),
                           self);

  update_counter_label(self);

  gtk_window_present(self->native_window);
}

// Handle platform view requests from Flutter.
static void platform_view_channel_method_cb(FlMethodChannel* channel,
                                            FlMethodCall* method_call,
                                            gpointer user_data) {
  MyApplication* self = MY_APPLICATION(user_data);

  const char* name = fl_method_call_get_name(method_call);
  if (g_str_equal(name, "switchView")) {
    handle_switch_view(self, method_call);
  } else {
    fl_method_call_respond_not_implemented(method_call, nullptr);
  }
}

// Implements GObject::dispose.
static void my_application_dispose(GObject* object) {
  MyApplication* self = MY_APPLICATION(object);

  g_clear_object(&self->platform_view_channel);

  G_OBJECT_CLASS(my_application_parent_class)->dispose(object);
}

// Implements GApplication::activate.
static void my_application_activate(GApplication* application) {
  MyApplication* self = MY_APPLICATION(application);
  self->window =
      GTK_WINDOW(gtk_application_window_new(GTK_APPLICATION(application)));

  gtk_window_set_default_size(self->window, 1280, 720);
  gtk_widget_show(GTK_WIDGET(self->window));

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  FlView* view = fl_view_new(project);
  gtk_widget_show(GTK_WIDGET(view));
  gtk_container_add(GTK_CONTAINER(self->window), GTK_WIDGET(view));

  // Create channel to handle platform view requests from Flutter.
  FlEngine* engine = fl_view_get_engine(view);
  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  self->platform_view_channel = fl_method_channel_new(
      fl_engine_get_binary_messenger(engine),
      "samples.flutter.io/platform_view", FL_METHOD_CODEC(codec));
  fl_method_channel_set_method_call_handler(self->platform_view_channel,
                                            platform_view_channel_method_cb,
                                            self, nullptr);

  fl_register_plugins(FL_PLUGIN_REGISTRY(view));

  gtk_widget_grab_focus(GTK_WIDGET(view));
}

static void my_application_class_init(MyApplicationClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = my_application_dispose;
  G_APPLICATION_CLASS(klass)->activate = my_application_activate;
}

static void my_application_init(MyApplication* self) {}

MyApplication* my_application_new() {
  return MY_APPLICATION(g_object_new(my_application_get_type(),
                                     "application-id", APPLICATION_ID, "flags",
                                     G_APPLICATION_NON_UNIQUE, nullptr));
}
