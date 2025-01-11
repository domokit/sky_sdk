// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "my_application.h"

#include <flutter_linux/flutter_linux.h>
#include <math.h>
#include <upower.h>
#ifdef GDK_WINDOWING_X11
#include <gdk/gdkx.h>
#endif

#include "flutter/generated_plugin_registrant.h"

struct _MyApplication {
  GtkApplication parent_instance;
  char** dart_entrypoint_arguments;

  FlView* view;

  // Connection to UPower.
  UpClient* up_client;
  GPtrArray* battery_devices;

  // Channel for Dart code to request the battery information.
  FlMethodChannel* battery_channel;

  // Channel to send updates to Dart code about battery charging state.
  FlEventChannel* charging_channel;
  gchar* last_charge_event;
  bool emit_charge_events;
};

G_DEFINE_TYPE(MyApplication, my_application, GTK_TYPE_APPLICATION)

// Checks the charging state and emits an event if necessary.
static void update_charging_state(MyApplication* self) {
  if (!self->emit_charge_events) {
    return;
  }

  const gchar* charge_event = "discharging";
  for (guint i = 0; i < self->battery_devices->len; i++) {
    UpDevice* device = UP_DEVICE(g_ptr_array_index(self->battery_devices, i));

    guint state;
    g_object_get(device, "state", &state, nullptr);
    if (state == UP_DEVICE_STATE_CHARGING ||
        state == UP_DEVICE_STATE_FULLY_CHARGED) {
      charge_event = "charging";
    }
  }

  if (g_strcmp0(charge_event, self->last_charge_event) != 0) {
    g_autoptr(GError) error = nullptr;
    g_autoptr(FlValue) value = fl_value_new_string(charge_event);
    if (!fl_event_channel_send(self->charging_channel, value, nullptr,
                               &error)) {
      g_warning("Failed to send charging event: %s", error->message);
      return;
    }
    g_free(self->last_charge_event);
    self->last_charge_event = g_strdup(charge_event);
  }
}

// Called when a UPower device changes state.
static void up_device_state_changed_cb(MyApplication* self, GParamSpec* pspec,
                                       UpDevice* device) {
  update_charging_state(self);
}

// Called when UPower devices are added.
static void up_device_added_cb(MyApplication* self, UpDevice* device) {
  // Listen for state changes from battery_devices.
  guint kind;
  g_object_get(device, "kind", &kind, nullptr);
  if (kind == UP_DEVICE_KIND_BATTERY) {
    g_ptr_array_add(self->battery_devices, g_object_ref(device));
    g_signal_connect_swapped(device, "notify::state",
                             G_CALLBACK(up_device_state_changed_cb), self);
    up_device_state_changed_cb(self, nullptr, device);
  }
}

// Called when UPower devices are removed.
static void up_device_removed_cb(MyApplication* self, UpDevice* device) {
  g_ptr_array_remove(self->battery_devices, device);
  g_signal_handlers_disconnect_matched(
      device, G_SIGNAL_MATCH_FUNC, 0, 0, nullptr,
      reinterpret_cast<GClosure*>(up_device_state_changed_cb), nullptr);
}

// Gets the current battery level.
static FlMethodResponse* get_battery_level(MyApplication* self) {
  // Find the first available battery and use that.
  for (guint i = 0; i < self->battery_devices->len; i++) {
    UpDevice* device = UP_DEVICE(g_ptr_array_index(self->battery_devices, i));

    double percentage;
    g_object_get(device, "percentage", &percentage, nullptr);
    g_autoptr(FlValue) result =
        fl_value_new_int(static_cast<int64_t>(round(percentage)));
    return FL_METHOD_RESPONSE(fl_method_success_response_new(result));
  }

  return FL_METHOD_RESPONSE(fl_method_error_response_new(
      "NO_BATTERY", "Device does not have a battery.", nullptr));
}

// Called when the Dart code requests battery information.
static void battery_method_call_cb(FlMethodChannel* channel,
                                   FlMethodCall* method_call,
                                   gpointer user_data) {
  MyApplication* self = static_cast<MyApplication*>(user_data);

  g_autoptr(FlMethodResponse) response = nullptr;
  if (strcmp(fl_method_call_get_name(method_call), "getBatteryLevel") == 0) {
    response = get_battery_level(self);
  } else {
    response = FL_METHOD_RESPONSE(fl_method_not_implemented_response_new());
  }

  g_autoptr(GError) error = nullptr;
  if (!fl_method_call_respond(method_call, response, &error)) {
    g_warning("Failed to send response: %s", error->message);
  }
}

// Called when the Dart code starts listening for charging events.
static FlMethodErrorResponse* charging_listen_cb(FlEventChannel* channel,
                                                 FlValue* args,
                                                 gpointer user_data) {
  MyApplication* self = static_cast<MyApplication*>(user_data);

  self->emit_charge_events = true;
  update_charging_state(self);

  return nullptr;
}

// Called when the Dart code stops listening for charging events.
static FlMethodErrorResponse* charging_cancel_cb(FlEventChannel* channel,
                                                 FlValue* args,
                                                 gpointer user_data) {
  MyApplication* self = static_cast<MyApplication*>(user_data);

  self->emit_charge_events = false;

  return nullptr;
}

// Creates the platform channels this application provides.
static void create_channels(MyApplication* self) {
  FlEngine* engine = fl_view_get_engine(self->view);
  FlBinaryMessenger* messenger = fl_engine_get_binary_messenger(engine);
  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();

  self->battery_channel = fl_method_channel_new(
      messenger, "samples.flutter.io/battery", FL_METHOD_CODEC(codec));
  fl_method_channel_set_method_call_handler(
      self->battery_channel, battery_method_call_cb, self, nullptr);

  self->charging_channel = fl_event_channel_new(
      messenger, "samples.flutter.io/charging", FL_METHOD_CODEC(codec));
  fl_event_channel_set_stream_handlers(self->charging_channel,
                                       charging_listen_cb, charging_cancel_cb,
                                       self, nullptr);
}

// Implements GApplication::activate.
static void my_application_activate(GApplication* application) {
  MyApplication* self = MY_APPLICATION(application);
  GtkWindow* window =
      GTK_WINDOW(gtk_application_window_new(GTK_APPLICATION(application)));

  // Use a header bar when running in GNOME as this is the common style used
  // by applications and is the setup most users will be using (e.g. Ubuntu
  // desktop).
  // If running on X and not using GNOME then just use a traditional title bar
  // in case the window manager does more exotic layout, e.g. tiling.
  // If running on Wayland assume the header bar will work (may need changing
  // if future cases occur).
  gboolean use_header_bar = TRUE;
#ifdef GDK_WINDOWING_X11
  GdkScreen* screen = gtk_window_get_screen(window);
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
    gtk_header_bar_set_title(header_bar, "platform_channel");
    gtk_header_bar_set_show_close_button(header_bar, TRUE);
    gtk_window_set_titlebar(window, GTK_WIDGET(header_bar));
  } else {
    gtk_window_set_title(window, "platform_channel");
  }

  gtk_window_set_default_size(window, 1280, 720);
  gtk_widget_show(GTK_WIDGET(window));

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  fl_dart_project_set_dart_entrypoint_arguments(
      project, self->dart_entrypoint_arguments);

  // Connect to UPower.
  self->up_client = up_client_new();
  g_signal_connect_swapped(self->up_client, "device-added",
                           G_CALLBACK(up_device_added_cb), self);
  g_signal_connect_swapped(self->up_client, "device-removed",
                           G_CALLBACK(up_device_removed_cb), self);
#if UP_CHECK_VERSION(0, 99, 8)
  // up_client_get_devices was deprecated and replaced with
  // up_client_get_devices2 in libupower 0.99.8.
  g_autoptr(GPtrArray) devices = up_client_get_devices2(self->up_client);
#else
  g_autoptr(GPtrArray) devices = up_client_get_devices(self->up_client);
#endif
  for (guint i = 0; i < devices->len; i++) {
    UpDevice* device = UP_DEVICE(g_ptr_array_index(devices, i));
    up_device_added_cb(self, device);
  }

  self->view = fl_view_new(project);
  gtk_widget_show(GTK_WIDGET(self->view));
  gtk_container_add(GTK_CONTAINER(window), GTK_WIDGET(self->view));

  fl_register_plugins(FL_PLUGIN_REGISTRY(self->view));

  // Create application specific platform channels.
  create_channels(self);

  gtk_widget_grab_focus(GTK_WIDGET(self->view));
}

// Implements GApplication::local_command_line.
static gboolean my_application_local_command_line(GApplication* application,
                                                  gchar*** arguments,
                                                  int* exit_status) {
  MyApplication* self = MY_APPLICATION(application);
  // Strip out the first argument as it is the binary name.
  self->dart_entrypoint_arguments = g_strdupv(*arguments + 1);

  g_autoptr(GError) error = nullptr;
  if (!g_application_register(application, nullptr, &error)) {
    g_warning("Failed to register: %s", error->message);
    *exit_status = 1;
    return TRUE;
  }

  g_application_activate(application);
  *exit_status = 0;

  return TRUE;
}

// Implements GObject::dispose.
static void my_application_dispose(GObject* object) {
  MyApplication* self = MY_APPLICATION(object);

  for (guint i = 0; i < self->battery_devices->len; i++) {
    UpDevice* device = UP_DEVICE(g_ptr_array_index(self->battery_devices, i));
    g_signal_handlers_disconnect_matched(device, G_SIGNAL_MATCH_DATA, 0, 0,
                                         nullptr, nullptr, self);
  }
  g_signal_handlers_disconnect_matched(self->up_client, G_SIGNAL_MATCH_DATA, 0,
                                       0, nullptr, nullptr, self);

  g_clear_pointer(&self->dart_entrypoint_arguments, g_strfreev);
  g_clear_object(&self->up_client);
  g_clear_pointer(&self->battery_devices, g_ptr_array_unref);
  g_clear_object(&self->battery_channel);
  g_clear_object(&self->charging_channel);
  g_clear_pointer(&self->last_charge_event, g_free);
  G_OBJECT_CLASS(my_application_parent_class)->dispose(object);
}

static void my_application_class_init(MyApplicationClass* klass) {
  G_APPLICATION_CLASS(klass)->activate = my_application_activate;
  G_APPLICATION_CLASS(klass)->local_command_line =
      my_application_local_command_line;
  G_OBJECT_CLASS(klass)->dispose = my_application_dispose;
}

static void my_application_init(MyApplication* self) {
  self->battery_devices = g_ptr_array_new_with_free_func(g_object_unref);
}

MyApplication* my_application_new() {
  return MY_APPLICATION(g_object_new(my_application_get_type(),
                                     "application-id", APPLICATION_ID, "flags",
                                     G_APPLICATION_NON_UNIQUE, nullptr));
}
