// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_key_event.h"

struct _FlKeyEvent {
  GObject parent_instance;

  // Time in milliseconds.
  guint32 time;

  // True if is a press event, otherwise a release event.
  gboolean is_press;

  // Hardware keycode.
  guint16 keycode;

  // Keyval.
  guint keyval;

  // Modifier state.
  GdkModifierType state;

  // Keyboard group.
  guint8 group;

  // The original event.
  GdkEvent* origin;
};

G_DEFINE_TYPE(FlKeyEvent, fl_key_event, G_TYPE_OBJECT)

FlKeyEvent* fl_key_event_new(guint32 time,
                             gboolean is_press,
                             guint16 keycode,
                             guint keyval,
                             GdkModifierType state,
                             guint8 group) {
  FlKeyEvent* self =
      FL_KEY_EVENT(g_object_new(fl_key_event_get_type(), nullptr));

  self->time = time;
  self->is_press = is_press;
  self->keycode = keycode;
  self->keyval = keyval;
  self->state = state;
  self->group = group;

  return self;
}

FlKeyEvent* fl_key_event_new_from_gdk_event(GdkEvent* event) {
  FlKeyEvent* self =
      FL_KEY_EVENT(g_object_new(fl_key_event_get_type(), nullptr));

  GdkEventType type = gdk_event_get_event_type(event);
  g_return_val_if_fail(type == GDK_KEY_PRESS || type == GDK_KEY_RELEASE,
                       nullptr);

  guint16 keycode = 0;
  gdk_event_get_keycode(event, &keycode);
  guint keyval = 0;
  gdk_event_get_keyval(event, &keyval);
  GdkModifierType state = static_cast<GdkModifierType>(0);
  gdk_event_get_state(event, &state);

  self->time = gdk_event_get_time(event);
  self->is_press = type == GDK_KEY_PRESS;
  self->keycode = keycode;
  self->keyval = keyval;
  self->state = state;
  self->group = event->key.group;
  self->origin = event;

  return self;
}

guint32 fl_key_event_get_time(FlKeyEvent* self) {
  g_return_val_if_fail(FL_IS_KEY_EVENT(self), 0);
  return self->time;
}

gboolean fl_key_event_get_is_press(FlKeyEvent* self) {
  g_return_val_if_fail(FL_IS_KEY_EVENT(self), FALSE);
  return self->is_press;
}

guint16 fl_key_event_get_keycode(FlKeyEvent* self) {
  g_return_val_if_fail(FL_IS_KEY_EVENT(self), 0);
  return self->keycode;
}

guint fl_key_event_get_keyval(FlKeyEvent* self) {
  g_return_val_if_fail(FL_IS_KEY_EVENT(self), 0);
  return self->keyval;
}

GdkModifierType fl_key_event_get_state(FlKeyEvent* self) {
  g_return_val_if_fail(FL_IS_KEY_EVENT(self), static_cast<GdkModifierType>(0));
  return self->state;
}

guint8 fl_key_event_get_group(FlKeyEvent* self) {
  g_return_val_if_fail(FL_IS_KEY_EVENT(self), 0);
  return self->group;
}

GdkEvent* fl_key_event_get_origin(FlKeyEvent* self) {
  g_return_val_if_fail(FL_IS_KEY_EVENT(self), nullptr);
  return self->origin;
}

uint64_t fl_key_event_hash(FlKeyEvent* self) {
  g_return_val_if_fail(FL_IS_KEY_EVENT(self), 0);

  // Combine the event timestamp, the type of event, and the hardware keycode
  // (scan code) of the event to come up with a unique id for this event that
  // can be derived solely from the event data itself, so that we can identify
  // whether or not we have seen this event already.
  guint64 type =
      static_cast<uint64_t>(self->is_press ? GDK_KEY_PRESS : GDK_KEY_RELEASE);
  guint64 keycode = static_cast<uint64_t>(self->keycode);
  return (self->time & 0xffffffff) | ((type & 0xffff) << 32) |
         ((keycode & 0xffff) << 48);
}

static void fl_key_event_dispose(GObject* object) {
  FlKeyEvent* self = FL_KEY_EVENT(object);

  g_clear_pointer(&self->origin, gdk_event_free);

  G_OBJECT_CLASS(fl_key_event_parent_class)->dispose(object);
}

static void fl_key_event_class_init(FlKeyEventClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_key_event_dispose;
}

static void fl_key_event_init(FlKeyEvent* self) {}
