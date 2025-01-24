// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_KEY_EVENT_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_KEY_EVENT_H_

#include <stdint.h>

#include <gdk/gdk.h>

G_DECLARE_FINAL_TYPE(FlKeyEvent, fl_key_event, FL, KEY_EVENT, GObject);

/**
 * FlKeyEvent:
 * A struct that stores information from GdkEvent.
 *
 * This is a class only used within the GTK embedding, created by
 * FlView and consumed by FlKeyboardHandler. It is not sent to
 * the embedder.
 *
 * This object contains information from GdkEvent as well as an origin event
 * object, so that Flutter can create an event object in unit tests even after
 * migrating to GDK 4.0 which stops supporting creating GdkEvent.
 */

FlKeyEvent* fl_key_event_new(guint32 time,
                             gboolean is_press,
                             guint16 keycode,
                             guint keyval,
                             GdkModifierType state,
                             guint8 group);

/**
 * fl_key_event_new_from_gdk_event:
 * @event: the #GdkEvent this #FlKeyEvent is based on.
 *
 * Create a new #FlKeyEvent based on a #GdkEvent.
 *
 * Returns: a new #FlKeyEvent.
 */
FlKeyEvent* fl_key_event_new_from_gdk_event(GdkEvent* event);

guint32 fl_key_event_get_time(FlKeyEvent* event);

gboolean fl_key_event_get_is_press(FlKeyEvent* event);

guint16 fl_key_event_get_keycode(FlKeyEvent* event);

guint fl_key_event_get_keyval(FlKeyEvent* event);

GdkModifierType fl_key_event_get_state(FlKeyEvent* event);

guint8 fl_key_event_get_group(FlKeyEvent* event);

GdkEvent* fl_key_event_get_origin(FlKeyEvent* event);

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_KEY_EVENT_H_
