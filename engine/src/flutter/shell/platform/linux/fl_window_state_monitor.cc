// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_window_state_monitor.h"

#include <gtk/gtk.h>

#include "flutter/shell/platform/linux/public/flutter_linux/fl_string_codec.h"

static constexpr const char* kFlutterLifecycleChannel = "flutter/lifecycle";

static constexpr const char* kAppLifecycleStateResumed =
    "AppLifecycleState.resumed";
static constexpr const char* kAppLifecycleStateInactive =
    "AppLifecycleState.inactive";
static constexpr const char* kAppLifecycleStateHidden =
    "AppLifecycleState.hidden";

struct _FlWindowStateMonitor {
  GObject parent_instance;

  // Messenger to communicate with engine.
  FlBinaryMessenger* messenger;

  // Window being monitored.
  GtkWindow* window;

  // Current state information.
  GdkWindowState window_state;

  // Signal connection ID for window-state-changed
  gulong window_state_event_cb_id;
};

G_DEFINE_TYPE(FlWindowStateMonitor, fl_window_state_monitor, G_TYPE_OBJECT);

static void send_lifecycle_state(FlWindowStateMonitor* self,
                                 const gchar* lifecycle_state) {
  g_autoptr(FlValue) value = fl_value_new_string(lifecycle_state);
  g_autoptr(FlStringCodec) codec = fl_string_codec_new();
  g_autoptr(GError) error = nullptr;
  g_autoptr(GBytes) message =
      fl_message_codec_encode_message(FL_MESSAGE_CODEC(codec), value, &error);
  if (message == nullptr) {
    g_warning("Failed to encoding lifecycle state message: %s", error->message);
    return;
  }

  fl_binary_messenger_send_on_channel(self->messenger, kFlutterLifecycleChannel,
                                      message, nullptr, nullptr, nullptr);
}

static gboolean is_hidden(GdkWindowState state) {
  return (state & GDK_WINDOW_STATE_WITHDRAWN) ||
         (state & GDK_WINDOW_STATE_ICONIFIED);
}

// Signal handler for GtkWindow::window-state-event
static gboolean window_state_event_cb(FlWindowStateMonitor* self,
                                      GdkEvent* event) {
  GdkWindowState state = event->window_state.new_window_state;
  GdkWindowState previous_state = self->window_state;
  self->window_state = state;
  bool was_visible = !is_hidden(previous_state);
  bool is_visible = !is_hidden(state);
  bool was_focused = (previous_state & GDK_WINDOW_STATE_FOCUSED);
  bool is_focused = (state & GDK_WINDOW_STATE_FOCUSED);

  if (was_visible != is_visible || was_focused != is_focused) {
    const gchar* lifecycle_state;
    if (is_visible) {
      lifecycle_state =
          is_focused ? kAppLifecycleStateResumed : kAppLifecycleStateInactive;
    } else {
      lifecycle_state = kAppLifecycleStateHidden;
    }

    send_lifecycle_state(self, lifecycle_state);
  }

  return FALSE;
}

static void fl_window_state_monitor_dispose(GObject* object) {
  FlWindowStateMonitor* self = FL_WINDOW_STATE_MONITOR(object);

  g_clear_object(&self->messenger);
  if (self->window_state_event_cb_id != 0) {
    g_signal_handler_disconnect(self->window, self->window_state_event_cb_id);
    self->window_state_event_cb_id = 0;
  }

  G_OBJECT_CLASS(fl_window_state_monitor_parent_class)->dispose(object);
}

static void fl_window_state_monitor_class_init(
    FlWindowStateMonitorClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_window_state_monitor_dispose;
}

static void fl_window_state_monitor_init(FlWindowStateMonitor* self) {}

FlWindowStateMonitor* fl_window_state_monitor_new(FlBinaryMessenger* messenger,
                                                  GtkWindow* window) {
  FlWindowStateMonitor* self = FL_WINDOW_STATE_MONITOR(
      g_object_new(fl_window_state_monitor_get_type(), nullptr));
  self->messenger = FL_BINARY_MESSENGER(g_object_ref(messenger));
  self->window = window;

  // Listen to window state changes.
  self->window_state_event_cb_id =
      g_signal_connect_swapped(self->window, "window-state-event",
                               G_CALLBACK(window_state_event_cb), self);
  self->window_state =
      gdk_window_get_state(gtk_widget_get_window(GTK_WIDGET(self->window)));

  return self;
}
