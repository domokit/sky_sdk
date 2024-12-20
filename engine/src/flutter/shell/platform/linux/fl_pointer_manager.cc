// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_pointer_manager.h"

#include "flutter/shell/platform/linux/fl_engine_private.h"

static constexpr int kMicrosecondsPerMillisecond = 1000;

struct _FlPointerManager {
  GObject parent_instance;

  // Engine to send pointer events to.
  GWeakRef engine;

  // ID to mark events with.
  FlutterViewId view_id;

  // TRUE if the mouse pointer is inside the view, used for generating missing
  // add events.
  gboolean pointer_inside;

  // Pointer button state recorded for sending status updates.
  int64_t button_state;
};

G_DEFINE_TYPE(FlPointerManager, fl_pointer_manager, G_TYPE_OBJECT);

// Generates a mouse pointer event if the pointer appears inside the window.
static void ensure_pointer_added(FlPointerManager* self,
                                 guint event_time,
                                 FlutterPointerDeviceKind device_kind,
                                 gdouble x,
                                 gdouble y) {
  if (self->pointer_inside) {
    return;
  }
  self->pointer_inside = TRUE;

  g_autoptr(FlEngine) engine = FL_ENGINE(g_weak_ref_get(&self->engine));
  if (engine == nullptr) {
    return;
  }

  fl_engine_send_mouse_pointer_event(
      engine, self->view_id, kAdd, event_time * kMicrosecondsPerMillisecond, x,
      y, device_kind, 0, 0, self->button_state);
}

static void fl_pointer_manager_dispose(GObject* object) {
  FlPointerManager* self = FL_POINTER_MANAGER(object);

  g_weak_ref_clear(&self->engine);

  G_OBJECT_CLASS(fl_pointer_manager_parent_class)->dispose(object);
}

static void fl_pointer_manager_class_init(FlPointerManagerClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_pointer_manager_dispose;
}

static void fl_pointer_manager_init(FlPointerManager* self) {}

FlPointerManager* fl_pointer_manager_new(FlutterViewId view_id,
                                         FlEngine* engine) {
  FlPointerManager* self =
      FL_POINTER_MANAGER(g_object_new(fl_pointer_manager_get_type(), nullptr));

  self->view_id = view_id;
  g_weak_ref_init(&self->engine, engine);

  return self;
}

gboolean fl_pointer_manager_handle_button_press(
    FlPointerManager* self,
    guint event_time,
    FlutterPointerDeviceKind device_kind,
    gdouble x,
    gdouble y,
    int64_t button) {
  g_return_val_if_fail(FL_IS_POINTER_MANAGER(self), FALSE);

  ensure_pointer_added(self, event_time, device_kind, x, y);

  // Drop the event if Flutter already thinks the button is down.
  if ((self->button_state & button) != 0) {
    return FALSE;
  }

  int old_button_state = self->button_state;
  FlutterPointerPhase phase = kMove;
  self->button_state ^= button;
  phase = old_button_state == 0 ? kDown : kMove;

  g_autoptr(FlEngine) engine = FL_ENGINE(g_weak_ref_get(&self->engine));
  if (engine == nullptr) {
    return FALSE;
  }

  fl_engine_send_mouse_pointer_event(
      engine, self->view_id, phase, event_time * kMicrosecondsPerMillisecond, x,
      y, device_kind, 0, 0, self->button_state);

  return TRUE;
}

gboolean fl_pointer_manager_handle_button_release(
    FlPointerManager* self,
    guint event_time,
    FlutterPointerDeviceKind device_kind,
    gdouble x,
    gdouble y,
    int64_t button) {
  g_return_val_if_fail(FL_IS_POINTER_MANAGER(self), FALSE);

  // Drop the event if Flutter already thinks the button is up.
  if ((self->button_state & button) == 0) {
    return FALSE;
  }

  FlutterPointerPhase phase = kMove;
  self->button_state ^= button;

  phase = self->button_state == 0 ? kUp : kMove;

  g_autoptr(FlEngine) engine = FL_ENGINE(g_weak_ref_get(&self->engine));
  if (engine == nullptr) {
    return FALSE;
  }

  fl_engine_send_mouse_pointer_event(
      engine, self->view_id, phase, event_time * kMicrosecondsPerMillisecond, x,
      y, device_kind, 0, 0, self->button_state);

  return TRUE;
}

gboolean fl_pointer_manager_handle_motion(FlPointerManager* self,
                                          guint event_time,
                                          FlutterPointerDeviceKind device_kind,
                                          gdouble x,
                                          gdouble y) {
  g_return_val_if_fail(FL_IS_POINTER_MANAGER(self), FALSE);

  g_autoptr(FlEngine) engine = FL_ENGINE(g_weak_ref_get(&self->engine));
  if (engine == nullptr) {
    return FALSE;
  }

  ensure_pointer_added(self, event_time, device_kind, x, y);

  fl_engine_send_mouse_pointer_event(
      engine, self->view_id, self->button_state != 0 ? kMove : kHover,
      event_time * kMicrosecondsPerMillisecond, x, y, device_kind, 0, 0,
      self->button_state);

  return TRUE;
}

gboolean fl_pointer_manager_handle_enter(FlPointerManager* self,
                                         guint event_time,
                                         FlutterPointerDeviceKind device_kind,
                                         gdouble x,
                                         gdouble y) {
  g_return_val_if_fail(FL_IS_POINTER_MANAGER(self), FALSE);

  g_autoptr(FlEngine) engine = FL_ENGINE(g_weak_ref_get(&self->engine));
  if (engine == nullptr) {
    return FALSE;
  }

  ensure_pointer_added(self, event_time, device_kind, x, y);

  return TRUE;
}

gboolean fl_pointer_manager_handle_leave(FlPointerManager* self,
                                         guint event_time,
                                         FlutterPointerDeviceKind device_kind,
                                         gdouble x,
                                         gdouble y) {
  g_return_val_if_fail(FL_IS_POINTER_MANAGER(self), FALSE);

  g_autoptr(FlEngine) engine = FL_ENGINE(g_weak_ref_get(&self->engine));
  if (engine == nullptr) {
    return FALSE;
  }

  // Don't remove pointer while button is down; In case of dragging outside of
  // window with mouse grab active Gtk will send another leave notify on
  // release.
  if (self->pointer_inside && self->button_state == 0) {
    fl_engine_send_mouse_pointer_event(engine, self->view_id, kRemove,
                                       event_time * kMicrosecondsPerMillisecond,
                                       x, y, device_kind, 0, 0,
                                       self->button_state);
    self->pointer_inside = FALSE;
  }

  return TRUE;
}
