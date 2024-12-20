// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_view.h"

#include <atk/atk.h>
#include <gtk/gtk-a11y.h>

#include <cstring>

#include "flutter/common/constants.h"
#include "flutter/shell/platform/linux/fl_accessible_node.h"
#include "flutter/shell/platform/linux/fl_engine_private.h"
#include "flutter/shell/platform/linux/fl_key_event.h"
#include "flutter/shell/platform/linux/fl_keyboard_handler.h"
#include "flutter/shell/platform/linux/fl_keyboard_manager.h"
#include "flutter/shell/platform/linux/fl_keyboard_view_delegate.h"
#include "flutter/shell/platform/linux/fl_plugin_registrar_private.h"
#include "flutter/shell/platform/linux/fl_pointer_manager.h"
#include "flutter/shell/platform/linux/fl_renderer_gdk.h"
#include "flutter/shell/platform/linux/fl_scrolling_manager.h"
#include "flutter/shell/platform/linux/fl_socket_accessible.h"
#include "flutter/shell/platform/linux/fl_text_input_handler.h"
#include "flutter/shell/platform/linux/fl_text_input_view_delegate.h"
#include "flutter/shell/platform/linux/fl_touch_manager.h"
#include "flutter/shell/platform/linux/fl_view_accessible.h"
#include "flutter/shell/platform/linux/fl_window_state_monitor.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_engine.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_plugin_registry.h"

struct _FlView {
  GtkBox parent_instance;

  // The widget rendering the Flutter view.
  GtkGLArea* gl_area;

  // Engine this view is showing.
  FlEngine* engine;

  // Signal subscription for engine restarts.
  guint on_pre_engine_restart_cb_id;

  // ID for this view.
  FlutterViewId view_id;

  // Object that performs the view rendering.
  FlRendererGdk* renderer;

  // Background color.
  GdkRGBA* background_color;

  // TRUE if have got the first frame to render.
  gboolean have_first_frame;

  // Monitor to track window state.
  FlWindowStateMonitor* window_state_monitor;

  // Manages scrolling events.
  FlScrollingManager* scrolling_manager;

  // Manages pointer events.
  FlPointerManager* pointer_manager;

  // Manages touch events.
  FlTouchManager* touch_manager;

  // Manages keyboard events.
  FlKeyboardManager* keyboard_manager;

  // Flutter system channel handlers.
  FlKeyboardHandler* keyboard_handler;
  FlTextInputHandler* text_input_handler;

  // Accessible tree from Flutter, exposed as an AtkPlug.
  FlViewAccessible* view_accessible;

  // Signal subscripton for cursor changes.
  guint cursor_changed_cb_id;

  GCancellable* cancellable;
};

enum { SIGNAL_FIRST_FRAME, LAST_SIGNAL };

static guint fl_view_signals[LAST_SIGNAL];

static void fl_renderable_iface_init(FlRenderableInterface* iface);

static void fl_view_plugin_registry_iface_init(
    FlPluginRegistryInterface* iface);

static void fl_view_keyboard_delegate_iface_init(
    FlKeyboardViewDelegateInterface* iface);

static void fl_view_text_input_delegate_iface_init(
    FlTextInputViewDelegateInterface* iface);

G_DEFINE_TYPE_WITH_CODE(
    FlView,
    fl_view,
    GTK_TYPE_BOX,
    G_IMPLEMENT_INTERFACE(fl_renderable_get_type(), fl_renderable_iface_init)
        G_IMPLEMENT_INTERFACE(fl_plugin_registry_get_type(),
                              fl_view_plugin_registry_iface_init)
            G_IMPLEMENT_INTERFACE(fl_keyboard_view_delegate_get_type(),
                                  fl_view_keyboard_delegate_iface_init)
                G_IMPLEMENT_INTERFACE(fl_text_input_view_delegate_get_type(),
                                      fl_view_text_input_delegate_iface_init))

// Emit the first frame signal in the main thread.
static gboolean first_frame_idle_cb(gpointer user_data) {
  FlView* self = FL_VIEW(user_data);

  g_signal_emit(self, fl_view_signals[SIGNAL_FIRST_FRAME], 0);

  return FALSE;
}

// Signal handler for GtkWidget::delete-event
static gboolean window_delete_event_cb(FlView* self) {
  fl_engine_request_app_exit(self->engine);
  // Stop the event from propagating.
  return TRUE;
}

// Initialize keyboard.
static void init_keyboard(FlView* self) {
  FlBinaryMessenger* messenger = fl_engine_get_binary_messenger(self->engine);

  GdkWindow* window =
      gtk_widget_get_window(gtk_widget_get_toplevel(GTK_WIDGET(self)));
  g_return_if_fail(GDK_IS_WINDOW(window));
  g_autoptr(GtkIMContext) im_context = gtk_im_multicontext_new();
  gtk_im_context_set_client_window(im_context, window);

  g_clear_object(&self->text_input_handler);
  self->text_input_handler = fl_text_input_handler_new(
      messenger, im_context, FL_TEXT_INPUT_VIEW_DELEGATE(self));
  g_clear_object(&self->keyboard_manager);
  self->keyboard_manager =
      fl_keyboard_manager_new(self->engine, FL_KEYBOARD_VIEW_DELEGATE(self));
  g_clear_object(&self->keyboard_handler);
  self->keyboard_handler =
      fl_keyboard_handler_new(messenger, self->keyboard_manager);
}

static void init_scrolling(FlView* self) {
  g_clear_object(&self->scrolling_manager);
  self->scrolling_manager =
      fl_scrolling_manager_new(self->engine, self->view_id);
}

static void init_touch(FlView* self) {
  g_clear_object(&self->touch_manager);
  self->touch_manager = fl_touch_manager_new(self->engine, self->view_id);
}

static FlutterPointerDeviceKind get_device_kind(GdkEvent* event) {
  GdkDevice* device = gdk_event_get_source_device(event);
  GdkInputSource source = gdk_device_get_source(device);
  switch (source) {
    case GDK_SOURCE_PEN:
    case GDK_SOURCE_ERASER:
    case GDK_SOURCE_CURSOR:
    case GDK_SOURCE_TABLET_PAD:
      return kFlutterPointerDeviceKindStylus;
    case GDK_SOURCE_TOUCHSCREEN:
      return kFlutterPointerDeviceKindTouch;
    case GDK_SOURCE_TOUCHPAD:  // trackpad device type is reserved for gestures
    case GDK_SOURCE_TRACKPOINT:
    case GDK_SOURCE_KEYBOARD:
    case GDK_SOURCE_MOUSE:
      return kFlutterPointerDeviceKindMouse;
  }
}

static gboolean get_mouse_button(GdkEvent* event, int64_t* button) {
  guint event_button = 0;
  gdk_event_get_button(event, &event_button);

  switch (event_button) {
    case 1:
      *button = kFlutterPointerButtonMousePrimary;
      return TRUE;
    case 2:
      *button = kFlutterPointerButtonMouseMiddle;
      return TRUE;
    case 3:
      *button = kFlutterPointerButtonMouseSecondary;
      return TRUE;
    default:
      return FALSE;
  }
}

// Called when the mouse cursor changes.
static void cursor_changed_cb(FlView* self) {
  FlMouseCursorHandler* handler =
      fl_engine_get_mouse_cursor_handler(self->engine);
  const gchar* cursor_name = fl_mouse_cursor_handler_get_cursor_name(handler);
  GdkWindow* window =
      gtk_widget_get_window(gtk_widget_get_toplevel(GTK_WIDGET(self)));
  g_autoptr(GdkCursor) cursor =
      gdk_cursor_new_from_name(gdk_window_get_display(window), cursor_name);
  gdk_window_set_cursor(window, cursor);
}

// Set the mouse cursor.
static void setup_cursor(FlView* self) {
  FlMouseCursorHandler* handler =
      fl_engine_get_mouse_cursor_handler(self->engine);

  self->cursor_changed_cb_id = g_signal_connect_swapped(
      handler, "cursor-changed", G_CALLBACK(cursor_changed_cb), self);
  cursor_changed_cb(self);
}

// Updates the engine with the current window metrics.
static void handle_geometry_changed(FlView* self) {
  GtkAllocation allocation;
  gtk_widget_get_allocation(GTK_WIDGET(self), &allocation);
  gint scale_factor = gtk_widget_get_scale_factor(GTK_WIDGET(self));
  fl_engine_send_window_metrics_event(
      self->engine, self->view_id, allocation.width * scale_factor,
      allocation.height * scale_factor, scale_factor);

  // Make sure the view has been realized and its size has been allocated before
  // waiting for a frame. `fl_view_realize()` and `fl_view_size_allocate()` may
  // be called in either order depending on the order in which the window is
  // shown and the view is added to a container in the app runner.
  //
  // Note: `gtk_widget_init()` initializes the size allocation to 1x1.
  if (allocation.width > 1 && allocation.height > 1 &&
      gtk_widget_get_realized(GTK_WIDGET(self))) {
    fl_renderer_wait_for_frame(FL_RENDERER(self->renderer),
                               allocation.width * scale_factor,
                               allocation.height * scale_factor);
  }
}

static void view_added_cb(GObject* object,
                          GAsyncResult* result,
                          gpointer user_data) {
  FlView* self = FL_VIEW(user_data);

  g_autoptr(GError) error = nullptr;
  if (!fl_engine_add_view_finish(FL_ENGINE(object), result, &error)) {
    if (g_error_matches(error, G_IO_ERROR, G_IO_ERROR_CANCELLED)) {
      return;
    }

    g_warning("Failed to add view: %s", error->message);
    // FIXME: Show on the GLArea
    return;
  }

  handle_geometry_changed(self);
}

// Called when the engine updates accessibility.
static void update_semantics_cb(FlEngine* engine,
                                const FlutterSemanticsUpdate2* update,
                                gpointer user_data) {
  FlView* self = FL_VIEW(user_data);

  fl_view_accessible_handle_update_semantics(self->view_accessible, update);
}

// Invoked by the engine right before the engine is restarted.
//
// This method should reset states to be as if the engine had just been started,
// which usually indicates the user has requested a hot restart (Shift-R in the
// Flutter CLI.)
static void on_pre_engine_restart_cb(FlView* self) {
  init_keyboard(self);
  init_scrolling(self);
  init_touch(self);
}

// Implements FlRenderable::redraw
static void fl_view_redraw(FlRenderable* renderable) {
  FlView* self = FL_VIEW(renderable);

  gtk_widget_queue_draw(GTK_WIDGET(self->gl_area));

  if (!self->have_first_frame) {
    self->have_first_frame = TRUE;
    // This is not the main thread, so the signal needs to be done via an idle
    // callback.
    g_idle_add(first_frame_idle_cb, self);
  }
}

// Implements FlRenderable::make_current
static void fl_view_make_current(FlRenderable* renderable) {
  FlView* self = FL_VIEW(renderable);
  gtk_gl_area_make_current(self->gl_area);
}

// Implements FlPluginRegistry::get_registrar_for_plugin.
static FlPluginRegistrar* fl_view_get_registrar_for_plugin(
    FlPluginRegistry* registry,
    const gchar* name) {
  FlView* self = FL_VIEW(registry);

  return fl_plugin_registrar_new(self,
                                 fl_engine_get_binary_messenger(self->engine),
                                 fl_engine_get_texture_registrar(self->engine));
}

static void fl_renderable_iface_init(FlRenderableInterface* iface) {
  iface->redraw = fl_view_redraw;
  iface->make_current = fl_view_make_current;
}

static void fl_view_plugin_registry_iface_init(
    FlPluginRegistryInterface* iface) {
  iface->get_registrar_for_plugin = fl_view_get_registrar_for_plugin;
}

static void fl_view_keyboard_delegate_iface_init(
    FlKeyboardViewDelegateInterface* iface) {
  iface->text_filter_key_press = [](FlKeyboardViewDelegate* view_delegate,
                                    FlKeyEvent* event) {
    FlView* self = FL_VIEW(view_delegate);
    return fl_text_input_handler_filter_keypress(self->text_input_handler,
                                                 event);
  };
}

static void fl_view_text_input_delegate_iface_init(
    FlTextInputViewDelegateInterface* iface) {
  iface->translate_coordinates = [](FlTextInputViewDelegate* delegate,
                                    gint view_x, gint view_y, gint* window_x,
                                    gint* window_y) {
    FlView* self = FL_VIEW(delegate);
    gtk_widget_translate_coordinates(GTK_WIDGET(self),
                                     gtk_widget_get_toplevel(GTK_WIDGET(self)),
                                     view_x, view_y, window_x, window_y);
  };
}

static void sync_modifier_if_needed(FlView* self, GdkEvent* event) {
  guint event_time = gdk_event_get_time(event);
  GdkModifierType event_state = static_cast<GdkModifierType>(0);
  gdk_event_get_state(event, &event_state);
  fl_keyboard_manager_sync_modifier_if_needed(self->keyboard_manager,
                                              event_state, event_time);
}

static void set_scrolling_position(FlView* self, gdouble x, gdouble y) {
  gint scale_factor = gtk_widget_get_scale_factor(GTK_WIDGET(self));
  fl_scrolling_manager_set_last_mouse_position(
      self->scrolling_manager, x * scale_factor, y * scale_factor);
}

// Signal handler for GtkWidget::button-press-event
static gboolean button_press_event_cb(FlView* self,
                                      GdkEventButton* button_event) {
  GdkEvent* event = reinterpret_cast<GdkEvent*>(button_event);

  // Flutter doesn't handle double and triple click events.
  GdkEventType event_type = gdk_event_get_event_type(event);
  if (event_type == GDK_DOUBLE_BUTTON_PRESS ||
      event_type == GDK_TRIPLE_BUTTON_PRESS) {
    return FALSE;
  }

  int64_t button;
  if (!get_mouse_button(event, &button)) {
    return FALSE;
  }

  gdouble x = 0.0, y = 0.0;
  gdk_event_get_coords(event, &x, &y);

  set_scrolling_position(self, x, y);
  sync_modifier_if_needed(self, event);

  gint scale_factor = gtk_widget_get_scale_factor(GTK_WIDGET(self));
  return fl_pointer_manager_handle_button_press(
      self->pointer_manager, gdk_event_get_time(event), get_device_kind(event),
      x * scale_factor, y * scale_factor, button);
}

// Signal handler for GtkWidget::button-release-event
static gboolean button_release_event_cb(FlView* self,
                                        GdkEventButton* button_event) {
  GdkEvent* event = reinterpret_cast<GdkEvent*>(button_event);

  int64_t button;
  if (!get_mouse_button(event, &button)) {
    return FALSE;
  }

  gdouble x = 0.0, y = 0.0;
  gdk_event_get_coords(event, &x, &y);

  set_scrolling_position(self, x, y);
  sync_modifier_if_needed(self, event);

  gint scale_factor = gtk_widget_get_scale_factor(GTK_WIDGET(self));
  return fl_pointer_manager_handle_button_release(
      self->pointer_manager, gdk_event_get_time(event), get_device_kind(event),
      x * scale_factor, y * scale_factor, button);
}

// Signal handler for GtkWidget::scroll-event
static gboolean scroll_event_cb(FlView* self, GdkEventScroll* event) {
  // TODO(robert-ancell): Update to use GtkEventControllerScroll when we can
  // depend on GTK 3.24.

  fl_scrolling_manager_handle_scroll_event(
      self->scrolling_manager, event,
      gtk_widget_get_scale_factor(GTK_WIDGET(self)));
  return TRUE;
}

static gboolean touch_event_cb(FlView* self, GdkEventTouch* event) {
  fl_touch_manager_handle_touch_event(
      self->touch_manager, event,
      gtk_widget_get_scale_factor(GTK_WIDGET(self)));
  return TRUE;
}

// Signal handler for GtkWidget::motion-notify-event
static gboolean motion_notify_event_cb(FlView* self,
                                       GdkEventMotion* motion_event) {
  GdkEvent* event = reinterpret_cast<GdkEvent*>(motion_event);
  sync_modifier_if_needed(self, event);

  // return if touch event
  auto event_type = gdk_event_get_event_type(event);
  if (event_type == GDK_TOUCH_BEGIN || event_type == GDK_TOUCH_UPDATE ||
      event_type == GDK_TOUCH_END || event_type == GDK_TOUCH_CANCEL) {
    return FALSE;
  }

  gdouble x = 0.0, y = 0.0;
  gdk_event_get_coords(event, &x, &y);
  gint scale_factor = gtk_widget_get_scale_factor(GTK_WIDGET(self));
  return fl_pointer_manager_handle_motion(
      self->pointer_manager, gdk_event_get_time(event), get_device_kind(event),
      x * scale_factor, y * scale_factor);
}

// Signal handler for GtkWidget::enter-notify-event
static gboolean enter_notify_event_cb(FlView* self,
                                      GdkEventCrossing* crossing_event) {
  GdkEvent* event = reinterpret_cast<GdkEvent*>(crossing_event);
  gdouble x = 0.0, y = 0.0;
  gdk_event_get_coords(event, &x, &y);
  gint scale_factor = gtk_widget_get_scale_factor(GTK_WIDGET(self));
  return fl_pointer_manager_handle_enter(
      self->pointer_manager, gdk_event_get_time(event), get_device_kind(event),
      x * scale_factor, y * scale_factor);
}

// Signal handler for GtkWidget::leave-notify-event
static gboolean leave_notify_event_cb(FlView* self,
                                      GdkEventCrossing* crossing_event) {
  if (crossing_event->mode != GDK_CROSSING_NORMAL) {
    return FALSE;
  }

  GdkEvent* event = reinterpret_cast<GdkEvent*>(crossing_event);
  gdouble x = 0.0, y = 0.0;
  gdk_event_get_coords(event, &x, &y);
  gint scale_factor = gtk_widget_get_scale_factor(GTK_WIDGET(self));
  return fl_pointer_manager_handle_leave(
      self->pointer_manager, gdk_event_get_time(event), get_device_kind(event),
      x * scale_factor, y * scale_factor);
}

static void gesture_rotation_begin_cb(FlView* self) {
  fl_scrolling_manager_handle_rotation_begin(self->scrolling_manager);
}

static void gesture_rotation_update_cb(FlView* self,
                                       gdouble rotation,
                                       gdouble delta) {
  fl_scrolling_manager_handle_rotation_update(self->scrolling_manager,
                                              rotation);
}

static void gesture_rotation_end_cb(FlView* self) {
  fl_scrolling_manager_handle_rotation_end(self->scrolling_manager);
}

static void gesture_zoom_begin_cb(FlView* self) {
  fl_scrolling_manager_handle_zoom_begin(self->scrolling_manager);
}

static void gesture_zoom_update_cb(FlView* self, gdouble scale) {
  fl_scrolling_manager_handle_zoom_update(self->scrolling_manager, scale);
}

static void gesture_zoom_end_cb(FlView* self) {
  fl_scrolling_manager_handle_zoom_end(self->scrolling_manager);
}

static GdkGLContext* create_context_cb(FlView* self) {
  fl_renderer_gdk_set_window(self->renderer,
                             gtk_widget_get_parent_window(GTK_WIDGET(self)));

  init_scrolling(self);
  init_touch(self);

  g_autoptr(GError) error = nullptr;
  if (!fl_renderer_gdk_create_contexts(self->renderer, &error)) {
    gtk_gl_area_set_error(self->gl_area, error);
    return nullptr;
  }

  return GDK_GL_CONTEXT(
      g_object_ref(fl_renderer_gdk_get_context(self->renderer)));
}

static void realize_cb(FlView* self) {
  g_autoptr(GError) error = nullptr;

  fl_renderer_make_current(FL_RENDERER(self->renderer));

  GError* gl_error = gtk_gl_area_get_error(self->gl_area);
  if (gl_error != NULL) {
    g_warning("Failed to initialize GLArea: %s", gl_error->message);
    return;
  }

  fl_renderer_setup(FL_RENDERER(self->renderer));

  GtkWidget* toplevel_window = gtk_widget_get_toplevel(GTK_WIDGET(self));

  self->window_state_monitor =
      fl_window_state_monitor_new(fl_engine_get_binary_messenger(self->engine),
                                  GTK_WINDOW(toplevel_window));

  // Handle requests by the user to close the application.
  g_signal_connect_swapped(toplevel_window, "delete-event",
                           G_CALLBACK(window_delete_event_cb), self);

  init_keyboard(self);

  fl_renderer_add_renderable(FL_RENDERER(self->renderer), self->view_id,
                             FL_RENDERABLE(self));

  if (!fl_engine_start(self->engine, &error)) {
    g_warning("Failed to start Flutter engine: %s", error->message);
    return;
  }

  setup_cursor(self);

  handle_geometry_changed(self);

  self->view_accessible = fl_view_accessible_new(self->engine);
  fl_socket_accessible_embed(
      FL_SOCKET_ACCESSIBLE(gtk_widget_get_accessible(GTK_WIDGET(self))),
      atk_plug_get_id(ATK_PLUG(self->view_accessible)));
}

static gboolean render_cb(FlView* self, GdkGLContext* context) {
  if (gtk_gl_area_get_error(self->gl_area) != NULL) {
    return FALSE;
  }

  int width = gtk_widget_get_allocated_width(GTK_WIDGET(self->gl_area));
  int height = gtk_widget_get_allocated_height(GTK_WIDGET(self->gl_area));
  gint scale_factor = gtk_widget_get_scale_factor(GTK_WIDGET(self->gl_area));
  fl_renderer_render(FL_RENDERER(self->renderer), self->view_id,
                     width * scale_factor, height * scale_factor,
                     self->background_color);

  return TRUE;
}

static void unrealize_cb(FlView* self) {
  g_autoptr(GError) error = nullptr;

  fl_renderer_make_current(FL_RENDERER(self->renderer));

  GError* gl_error = gtk_gl_area_get_error(self->gl_area);
  if (gl_error != NULL) {
    g_warning("Failed to uninitialize GLArea: %s", gl_error->message);
    return;
  }

  fl_renderer_cleanup(FL_RENDERER(self->renderer));
}

static void size_allocate_cb(FlView* self) {
  handle_geometry_changed(self);
}

static void fl_view_notify(GObject* object, GParamSpec* pspec) {
  FlView* self = FL_VIEW(object);

  if (strcmp(pspec->name, "scale-factor") == 0) {
    handle_geometry_changed(self);
  }

  if (G_OBJECT_CLASS(fl_view_parent_class)->notify != nullptr) {
    G_OBJECT_CLASS(fl_view_parent_class)->notify(object, pspec);
  }
}

static void fl_view_dispose(GObject* object) {
  FlView* self = FL_VIEW(object);

  g_cancellable_cancel(self->cancellable);

  if (self->engine != nullptr) {
    fl_engine_set_update_semantics_handler(self->engine, nullptr, nullptr,
                                           nullptr);

    FlMouseCursorHandler* handler =
        fl_engine_get_mouse_cursor_handler(self->engine);
    if (self->cursor_changed_cb_id != 0) {
      g_signal_handler_disconnect(handler, self->cursor_changed_cb_id);
      self->cursor_changed_cb_id = 0;
    }

    // Stop rendering.
    fl_renderer_remove_view(FL_RENDERER(self->renderer), self->view_id);

    // Release the view ID from the engine.
    fl_engine_remove_view(self->engine, self->view_id, nullptr, nullptr,
                          nullptr);
  }

  if (self->on_pre_engine_restart_cb_id != 0) {
    g_signal_handler_disconnect(self->engine,
                                self->on_pre_engine_restart_cb_id);
    self->on_pre_engine_restart_cb_id = 0;
  }

  g_clear_object(&self->engine);
  g_clear_object(&self->renderer);
  g_clear_pointer(&self->background_color, gdk_rgba_free);
  g_clear_object(&self->window_state_monitor);
  g_clear_object(&self->scrolling_manager);
  g_clear_object(&self->pointer_manager);
  g_clear_object(&self->touch_manager);
  g_clear_object(&self->keyboard_manager);
  g_clear_object(&self->keyboard_handler);
  g_clear_object(&self->view_accessible);
  g_clear_object(&self->cancellable);

  G_OBJECT_CLASS(fl_view_parent_class)->dispose(object);
}

// Implements GtkWidget::realize.
static void fl_view_realize(GtkWidget* widget) {
  FlView* self = FL_VIEW(widget);

  GTK_WIDGET_CLASS(fl_view_parent_class)->realize(widget);

  // Realize the child widgets.
  gtk_widget_realize(GTK_WIDGET(self->gl_area));
}

// Implements GtkWidget::key_press_event.
static gboolean fl_view_key_press_event(GtkWidget* widget,
                                        GdkEventKey* key_event) {
  FlView* self = FL_VIEW(widget);

  GdkEvent* event = reinterpret_cast<GdkEvent*>(key_event);
  return fl_keyboard_manager_handle_event(
      self->keyboard_manager,
      fl_key_event_new_from_gdk_event(gdk_event_copy(event)));
}

// Implements GtkWidget::key_release_event.
static gboolean fl_view_key_release_event(GtkWidget* widget,
                                          GdkEventKey* key_event) {
  FlView* self = FL_VIEW(widget);
  GdkEvent* event = reinterpret_cast<GdkEvent*>(key_event);
  return fl_keyboard_manager_handle_event(
      self->keyboard_manager,
      fl_key_event_new_from_gdk_event(gdk_event_copy(event)));
}

static void fl_view_class_init(FlViewClass* klass) {
  GObjectClass* object_class = G_OBJECT_CLASS(klass);
  object_class->notify = fl_view_notify;
  object_class->dispose = fl_view_dispose;

  GtkWidgetClass* widget_class = GTK_WIDGET_CLASS(klass);
  widget_class->realize = fl_view_realize;
  widget_class->key_press_event = fl_view_key_press_event;
  widget_class->key_release_event = fl_view_key_release_event;

  fl_view_signals[SIGNAL_FIRST_FRAME] =
      g_signal_new("first-frame", fl_view_get_type(), G_SIGNAL_RUN_LAST, 0,
                   NULL, NULL, NULL, G_TYPE_NONE, 0);

  gtk_widget_class_set_accessible_type(GTK_WIDGET_CLASS(klass),
                                       fl_socket_accessible_get_type());
}

static void fl_view_init(FlView* self) {
  self->cancellable = g_cancellable_new();

  gtk_widget_set_can_focus(GTK_WIDGET(self), TRUE);

  self->view_id = -1;

  GdkRGBA default_background = {
      .red = 0.0, .green = 0.0, .blue = 0.0, .alpha = 1.0};
  self->background_color = gdk_rgba_copy(&default_background);

  GtkWidget* event_box = gtk_event_box_new();
  gtk_widget_set_hexpand(event_box, TRUE);
  gtk_widget_set_vexpand(event_box, TRUE);
  gtk_container_add(GTK_CONTAINER(self), event_box);
  gtk_widget_show(event_box);
  gtk_widget_add_events(event_box,
                        GDK_POINTER_MOTION_MASK | GDK_BUTTON_PRESS_MASK |
                            GDK_BUTTON_RELEASE_MASK | GDK_SCROLL_MASK |
                            GDK_SMOOTH_SCROLL_MASK | GDK_TOUCH_MASK);

  g_signal_connect_swapped(event_box, "button-press-event",
                           G_CALLBACK(button_press_event_cb), self);
  g_signal_connect_swapped(event_box, "button-release-event",
                           G_CALLBACK(button_release_event_cb), self);
  g_signal_connect_swapped(event_box, "scroll-event",
                           G_CALLBACK(scroll_event_cb), self);
  g_signal_connect_swapped(event_box, "motion-notify-event",
                           G_CALLBACK(motion_notify_event_cb), self);
  g_signal_connect_swapped(event_box, "enter-notify-event",
                           G_CALLBACK(enter_notify_event_cb), self);
  g_signal_connect_swapped(event_box, "leave-notify-event",
                           G_CALLBACK(leave_notify_event_cb), self);
  GtkGesture* zoom = gtk_gesture_zoom_new(event_box);
  g_signal_connect_swapped(zoom, "begin", G_CALLBACK(gesture_zoom_begin_cb),
                           self);
  g_signal_connect_swapped(zoom, "scale-changed",
                           G_CALLBACK(gesture_zoom_update_cb), self);
  g_signal_connect_swapped(zoom, "end", G_CALLBACK(gesture_zoom_end_cb), self);
  GtkGesture* rotate = gtk_gesture_rotate_new(event_box);
  g_signal_connect_swapped(rotate, "begin",
                           G_CALLBACK(gesture_rotation_begin_cb), self);
  g_signal_connect_swapped(rotate, "angle-changed",
                           G_CALLBACK(gesture_rotation_update_cb), self);
  g_signal_connect_swapped(rotate, "end", G_CALLBACK(gesture_rotation_end_cb),
                           self);
  g_signal_connect_swapped(event_box, "touch-event", G_CALLBACK(touch_event_cb),
                           self);

  self->gl_area = GTK_GL_AREA(gtk_gl_area_new());
  gtk_gl_area_set_has_alpha(self->gl_area, TRUE);
  gtk_widget_show(GTK_WIDGET(self->gl_area));
  gtk_container_add(GTK_CONTAINER(event_box), GTK_WIDGET(self->gl_area));
  g_signal_connect_swapped(self->gl_area, "render", G_CALLBACK(render_cb),
                           self);

  g_signal_connect_swapped(self, "size-allocate", G_CALLBACK(size_allocate_cb),
                           self);
}

G_MODULE_EXPORT FlView* fl_view_new(FlDartProject* project) {
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  FlView* self = FL_VIEW(g_object_new(fl_view_get_type(), nullptr));

  self->view_id = flutter::kFlutterImplicitViewId;
  self->engine = FL_ENGINE(g_object_ref(engine));
  FlRenderer* renderer = fl_engine_get_renderer(engine);
  g_assert(FL_IS_RENDERER_GDK(renderer));
  self->renderer = FL_RENDERER_GDK(g_object_ref(renderer));

  self->pointer_manager = fl_pointer_manager_new(self->view_id, engine);

  fl_engine_set_update_semantics_handler(self->engine, update_semantics_cb,
                                         self, nullptr);
  self->on_pre_engine_restart_cb_id =
      g_signal_connect_swapped(engine, "on-pre-engine-restart",
                               G_CALLBACK(on_pre_engine_restart_cb), self);

  g_signal_connect_swapped(self->gl_area, "create-context",
                           G_CALLBACK(create_context_cb), self);
  g_signal_connect_swapped(self->gl_area, "realize", G_CALLBACK(realize_cb),
                           self);
  g_signal_connect_swapped(self->gl_area, "unrealize", G_CALLBACK(unrealize_cb),
                           self);

  return self;
}

G_MODULE_EXPORT FlView* fl_view_new_for_engine(FlEngine* engine) {
  FlView* self = FL_VIEW(g_object_new(fl_view_get_type(), nullptr));

  self->engine = FL_ENGINE(g_object_ref(engine));
  FlRenderer* renderer = fl_engine_get_renderer(engine);
  g_assert(FL_IS_RENDERER_GDK(renderer));
  self->renderer = FL_RENDERER_GDK(g_object_ref(renderer));

  self->on_pre_engine_restart_cb_id =
      g_signal_connect_swapped(engine, "on-pre-engine-restart",
                               G_CALLBACK(on_pre_engine_restart_cb), self);

  self->view_id = fl_engine_add_view(self->engine, 1, 1, 1.0, self->cancellable,
                                     view_added_cb, self);
  fl_renderer_add_renderable(FL_RENDERER(self->renderer), self->view_id,
                             FL_RENDERABLE(self));

  self->pointer_manager = fl_pointer_manager_new(self->view_id, engine);

  setup_cursor(self);

  return self;
}

G_MODULE_EXPORT FlEngine* fl_view_get_engine(FlView* self) {
  g_return_val_if_fail(FL_IS_VIEW(self), nullptr);
  return self->engine;
}

G_MODULE_EXPORT
int64_t fl_view_get_id(FlView* self) {
  g_return_val_if_fail(FL_IS_VIEW(self), -1);
  return self->view_id;
}

G_MODULE_EXPORT void fl_view_set_background_color(FlView* self,
                                                  const GdkRGBA* color) {
  g_return_if_fail(FL_IS_VIEW(self));
  gdk_rgba_free(self->background_color);
  self->background_color = gdk_rgba_copy(color);
}
