// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "my_application.h"

#include <flutter_linux/flutter_linux.h>

#include "my_texture.h"
#ifdef GDK_WINDOWING_X11
#include <gdk/gdkx.h>
#endif

#include "flutter/generated_plugin_registrant.h"

struct _MyApplication {
  GtkApplication parent_instance;

  // Channel to receive texture requests from Flutter.
  FlMethodChannel* texture_channel;

  // Texture we've created.
  MyTexture* texture;

  FlView* view;
};

G_DEFINE_TYPE(MyApplication, my_application, GTK_TYPE_APPLICATION)

// Handle request to create the texture.
static FlMethodResponse* handle_create(MyApplication* self,
                                       FlMethodCall* method_call) {
  if (self->texture != nullptr) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        "Error", "texture already created", nullptr));
  }

  FlValue* args = fl_method_call_get_args(method_call);
  if (fl_value_get_type(args) != FL_VALUE_TYPE_LIST ||
      fl_value_get_length(args) != 2) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        "Invalid args", "Invalid create args", nullptr));
  }
  FlValue* width_value = fl_value_get_list_value(args, 0);
  FlValue* height_value = fl_value_get_list_value(args, 1);
  if (fl_value_get_type(width_value) != FL_VALUE_TYPE_INT ||
      fl_value_get_type(height_value) != FL_VALUE_TYPE_INT) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        "Invalid args", "Invalid create args", nullptr));
  }

  FlEngine* engine = fl_view_get_engine(self->view);
  FlTextureRegistrar* texture_registrar =
      fl_engine_get_texture_registrar(engine);

  self->texture =
      my_texture_new(fl_value_get_int(width_value),
                     fl_value_get_int(height_value), 0x05, 0x53, 0xb1);
  if (!fl_texture_registrar_register_texture(texture_registrar,
                                             FL_TEXTURE(self->texture))) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        "Error", "Failed to register texture", nullptr));
  }

  // Return the texture ID to Flutter so it can use this texture.
  g_autoptr(FlValue) id =
      fl_value_new_int(fl_texture_get_id(FL_TEXTURE(self->texture)));
  return FL_METHOD_RESPONSE(fl_method_success_response_new(id));
}

// Handle request to set the texture color.
static FlMethodResponse* handle_set_color(MyApplication* self,
                                          FlMethodCall* method_call) {
  FlValue* args = fl_method_call_get_args(method_call);
  if (fl_value_get_type(args) != FL_VALUE_TYPE_LIST ||
      fl_value_get_length(args) != 3) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        "Invalid args", "Invalid setColor args", nullptr));
  }
  FlValue* r_value = fl_value_get_list_value(args, 0);
  FlValue* g_value = fl_value_get_list_value(args, 1);
  FlValue* b_value = fl_value_get_list_value(args, 2);
  if (fl_value_get_type(r_value) != FL_VALUE_TYPE_INT ||
      fl_value_get_type(g_value) != FL_VALUE_TYPE_INT ||
      fl_value_get_type(b_value) != FL_VALUE_TYPE_INT) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        "Invalid args", "Invalid setColor args", nullptr));
  }

  FlEngine* engine = fl_view_get_engine(self->view);
  FlTextureRegistrar* texture_registrar =
      fl_engine_get_texture_registrar(engine);

  // Redraw in requested color.
  my_texture_set_color(self->texture, fl_value_get_int(r_value),
                       fl_value_get_int(g_value), fl_value_get_int(b_value));

  // Notify Flutter the texture has changed.
  fl_texture_registrar_mark_texture_frame_available(texture_registrar,
                                                    FL_TEXTURE(self->texture));

  return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
}

// Handle texture requests from Flutter.
static void texture_channel_method_cb(FlMethodChannel* channel,
                                      FlMethodCall* method_call,
                                      gpointer user_data) {
  MyApplication* self = MY_APPLICATION(user_data);

  const char* name = fl_method_call_get_name(method_call);
  if (g_str_equal(name, "create")) {
    g_autoptr(FlMethodResponse) response = handle_create(self, method_call);
    fl_method_call_respond(method_call, response, NULL);
  } else if (g_str_equal(name, "setColor")) {
    g_autoptr(FlMethodResponse) response = handle_set_color(self, method_call);
    fl_method_call_respond(method_call, response, NULL);
  } else {
    fl_method_call_respond_not_implemented(method_call, NULL);
  }
}

// Implements GObject::dispose.
static void my_application_dispose(GObject* object) {
  MyApplication* self = MY_APPLICATION(object);

  g_clear_object(&self->texture_channel);
  g_clear_object(&self->texture);

  G_OBJECT_CLASS(my_application_parent_class)->dispose(object);
}

// Implements GApplication::activate.
static void my_application_activate(GApplication* application) {
  MyApplication* self = MY_APPLICATION(application);
  GtkWindow* window =
      GTK_WINDOW(gtk_application_window_new(GTK_APPLICATION(application)));

  gtk_window_set_default_size(window, 1280, 720);
  gtk_widget_show(GTK_WIDGET(window));

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  self->view = fl_view_new(project);
  gtk_widget_show(GTK_WIDGET(self->view));
  gtk_container_add(GTK_CONTAINER(window), GTK_WIDGET(self->view));

  // Create channel to handle texture requests from Flutter.
  FlEngine* engine = fl_view_get_engine(self->view);
  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  self->texture_channel = fl_method_channel_new(
      fl_engine_get_binary_messenger(engine), "samples.flutter.io/texture",
      FL_METHOD_CODEC(codec));
  fl_method_channel_set_method_call_handler(
      self->texture_channel, texture_channel_method_cb, self, nullptr);

  fl_register_plugins(FL_PLUGIN_REGISTRY(self->view));

  gtk_widget_grab_focus(GTK_WIDGET(self->view));
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
