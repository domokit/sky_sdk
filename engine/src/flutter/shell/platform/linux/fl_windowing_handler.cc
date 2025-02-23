// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_windowing_handler.h"

#include "flutter/shell/platform/linux/fl_windowing_channel.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_view.h"

typedef struct {
  GWeakRef engine;

  FlWindowingChannel* channel;

  GHashTable* windows_by_view_id;
} FlWindowingHandlerPrivate;

enum { SIGNAL_CREATE_WINDOW, LAST_SIGNAL };

static guint signals[LAST_SIGNAL];

G_DEFINE_TYPE_WITH_PRIVATE(FlWindowingHandler,
                           fl_windowing_handler,
                           G_TYPE_OBJECT)

typedef struct {
  GtkWindow* window;
  FlView* view;
  guint first_frame_cb_id;
} WindowData;

static WindowData* window_data_new(GtkWindow* window, FlView* view) {
  WindowData* data = g_new0(WindowData, 1);
  data->window = GTK_WINDOW(g_object_ref(window));
  data->view = FL_VIEW(g_object_ref(view));
  return data;
}

static void window_data_free(WindowData* data) {
  g_signal_handler_disconnect(data->view, data->first_frame_cb_id);
  g_object_unref(data->window);
  g_object_unref(data->view);
  g_free(data);
}

// Called when the first frame is received.
static void first_frame_cb(FlView* view, WindowData* data) {
  gtk_window_present(data->window);
}

static WindowData* get_window_data(FlWindowingHandler* self, int64_t view_id) {
  FlWindowingHandlerPrivate* priv =
      reinterpret_cast<FlWindowingHandlerPrivate*>(
          fl_windowing_handler_get_instance_private(self));

  return static_cast<WindowData*>(
      g_hash_table_lookup(priv->windows_by_view_id, GINT_TO_POINTER(view_id)));
}

static GtkWindow* fl_windowing_handler_create_window(
    FlWindowingHandler* handler,
    FlView* view) {
  GtkWidget* window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

  gtk_widget_show(GTK_WIDGET(view));
  gtk_container_add(GTK_CONTAINER(window), GTK_WIDGET(view));

  return GTK_WINDOW(window);
}

static FlMethodResponse* create_regular(FlWindowingSize* size,
                                        FlWindowingSize* min_size,
                                        FlWindowingSize* max_size,
                                        const gchar* title,
                                        FlWindowState state,
                                        gpointer user_data) {
  FlWindowingHandler* self = FL_WINDOWING_HANDLER(user_data);
  FlWindowingHandlerPrivate* priv =
      reinterpret_cast<FlWindowingHandlerPrivate*>(
          fl_windowing_handler_get_instance_private(self));

  g_autoptr(FlEngine) engine = FL_ENGINE(g_weak_ref_get(&priv->engine));
  if (engine == nullptr) {
    return FL_METHOD_RESPONSE(
        fl_method_error_response_new("Internal error", "No engine", nullptr));
  }

  FlView* view = fl_view_new_for_engine(engine);

  GtkWindow* window;
  g_signal_emit(self, signals[SIGNAL_CREATE_WINDOW], 0, view, &window);
  if (window == nullptr) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        "Internal error", "Failed to create window", nullptr));
  }

  gtk_window_set_default_size(GTK_WINDOW(window), size->width, size->height);
  if (title != nullptr) {
    gtk_window_set_title(GTK_WINDOW(window), title);
  }
  switch (state) {
    case FL_WINDOW_STATE_MAXIMIZED:
      gtk_window_maximize(GTK_WINDOW(window));
      break;
    case FL_WINDOW_STATE_MINIMIZED:
      gtk_window_iconify(GTK_WINDOW(window));
      break;
    case FL_WINDOW_STATE_RESTORED:
    case FL_WINDOW_STATE_UNDEFINED:
      break;
  }

  GdkGeometry geometry;
  GdkWindowHints geometry_mask = static_cast<GdkWindowHints>(0);
  if (min_size != nullptr) {
    geometry.min_width = min_size->width;
    geometry.min_height = min_size->height;
    geometry_mask =
        static_cast<GdkWindowHints>(geometry_mask | GDK_HINT_MIN_SIZE);
  }
  if (max_size != nullptr) {
    geometry.max_width = max_size->width;
    geometry.max_height = max_size->height;
    geometry_mask =
        static_cast<GdkWindowHints>(geometry_mask | GDK_HINT_MAX_SIZE);
  }
  if (geometry_mask != 0) {
    gtk_window_set_geometry_hints(GTK_WINDOW(window), nullptr, &geometry,
                                  geometry_mask);
  }

  WindowData* data = window_data_new(GTK_WINDOW(window), view);
  data->first_frame_cb_id =
      g_signal_connect(view, "first-frame", G_CALLBACK(first_frame_cb), data);

  // Make the resources for the view so rendering can start.
  // We'll show the view when we have the first frame.
  gtk_widget_realize(GTK_WIDGET(view));

  g_hash_table_insert(priv->windows_by_view_id,
                      GINT_TO_POINTER(fl_view_get_id(view)), data);

  // We don't know the current size and dimensions, so just reflect back what
  // was requested.
  FlWindowingSize* initial_size = size;
  FlWindowState initial_state = state;
  if (initial_state == FL_WINDOW_STATE_UNDEFINED) {
    initial_state = FL_WINDOW_STATE_RESTORED;
  }

  return fl_windowing_channel_make_create_regular_response(
      fl_view_get_id(view), initial_size, initial_state);
}

static FlMethodResponse* modify_regular(int64_t view_id,
                                        FlWindowingSize* size,
                                        const gchar* title,
                                        FlWindowState state,
                                        gpointer user_data) {
  FlWindowingHandler* self = FL_WINDOWING_HANDLER(user_data);

  WindowData* data = get_window_data(self, view_id);
  if (data == nullptr) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        "Bad Arguments", "No window with given view ID", nullptr));
  }

  if (size != nullptr) {
    gtk_window_resize(data->window, size->width, size->height);
  }

  if (title != nullptr) {
    gtk_window_set_title(data->window, title);
  }

  GdkWindowState window_state;
  switch (state) {
    case FL_WINDOW_STATE_RESTORED:
      if (gtk_window_is_maximized(data->window)) {
        gtk_window_unmaximize(data->window);
      }
      window_state =
          gdk_window_get_state(gtk_widget_get_window(GTK_WIDGET(data->window)));
      if (window_state & GDK_WINDOW_STATE_ICONIFIED) {
        gtk_window_deiconify(data->window);
      }
      break;
    case FL_WINDOW_STATE_MAXIMIZED:
      gtk_window_maximize(data->window);
      break;
    case FL_WINDOW_STATE_MINIMIZED:
      gtk_window_iconify(data->window);
      break;
    case FL_WINDOW_STATE_UNDEFINED:
      break;
  }

  return fl_windowing_channel_make_modify_regular_response();
}

static FlMethodResponse* destroy_window(int64_t view_id, gpointer user_data) {
  FlWindowingHandler* self = FL_WINDOWING_HANDLER(user_data);
  FlWindowingHandlerPrivate* priv =
      reinterpret_cast<FlWindowingHandlerPrivate*>(
          fl_windowing_handler_get_instance_private(self));

  WindowData* data = get_window_data(self, view_id);
  if (data == nullptr) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        "Bad Arguments", "No window with given view ID", nullptr));
  }

  gtk_widget_destroy(GTK_WIDGET(data->window));

  g_hash_table_remove(priv->windows_by_view_id, GINT_TO_POINTER(view_id));

  return fl_windowing_channel_make_destroy_window_response();
}

static void fl_windowing_handler_dispose(GObject* object) {
  FlWindowingHandler* self = FL_WINDOWING_HANDLER(object);
  FlWindowingHandlerPrivate* priv =
      reinterpret_cast<FlWindowingHandlerPrivate*>(
          fl_windowing_handler_get_instance_private(self));

  g_weak_ref_clear(&priv->engine);
  g_clear_object(&priv->channel);
  g_clear_pointer(&priv->windows_by_view_id, g_hash_table_unref);

  G_OBJECT_CLASS(fl_windowing_handler_parent_class)->dispose(object);
}

static void fl_windowing_handler_class_init(FlWindowingHandlerClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_windowing_handler_dispose;

  klass->create_window = fl_windowing_handler_create_window;

  signals[SIGNAL_CREATE_WINDOW] = g_signal_new(
      "create-window", fl_windowing_handler_get_type(), G_SIGNAL_RUN_LAST,
      G_STRUCT_OFFSET(FlWindowingHandlerClass, create_window),
      g_signal_accumulator_first_wins, nullptr, nullptr, GTK_TYPE_WINDOW, 1,
      fl_view_get_type());
}

static void fl_windowing_handler_init(FlWindowingHandler* self) {
  FlWindowingHandlerPrivate* priv =
      reinterpret_cast<FlWindowingHandlerPrivate*>(
          fl_windowing_handler_get_instance_private(self));

  priv->windows_by_view_id =
      g_hash_table_new_full(g_direct_hash, g_direct_equal, nullptr,
                            reinterpret_cast<GDestroyNotify>(window_data_free));
}

static FlWindowingChannelVTable windowing_channel_vtable = {
    .create_regular = create_regular,
    .modify_regular = modify_regular,
    .destroy_window = destroy_window,
};

FlWindowingHandler* fl_windowing_handler_new(FlEngine* engine) {
  g_return_val_if_fail(FL_IS_ENGINE(engine), nullptr);

  FlWindowingHandler* self = FL_WINDOWING_HANDLER(
      g_object_new(fl_windowing_handler_get_type(), nullptr));
  FlWindowingHandlerPrivate* priv =
      reinterpret_cast<FlWindowingHandlerPrivate*>(
          fl_windowing_handler_get_instance_private(self));

  g_weak_ref_init(&priv->engine, engine);
  priv->channel = fl_windowing_channel_new(
      fl_engine_get_binary_messenger(engine), &windowing_channel_vtable, self);

  return self;
}
