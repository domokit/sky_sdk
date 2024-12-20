// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_POINTER_MANAGER_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_POINTER_MANAGER_H_

#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_engine.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlPointerManager,
                     fl_pointer_manager,
                     FL,
                     POINTER_MANAGER,
                     GObject);

/**
 * fl_pointer_manager_new:
 * @view_id: view ID to report events for.
 * @engine: an #FlEngine.
 *
 * Create a new #FlPointerManager.
 *
 * Returns: a new #FlPointerManager.
 */
FlPointerManager* fl_pointer_manager_new(FlutterViewId view_id,
                                         FlEngine* engine);

/**
 * fl_pointer_manager_handle_button_press:
 * @manager: an #FlPointerManager.
 * @event_time: event time in milliseconds.
 * @device_kind: kind of device generating the event.
 * @x: x co-ordinate of event.
 * @y: y co-ordinate of event.
 * @button: button being pressed.
 *
 * Returns %TRUE if this event was handled.
 */
gboolean fl_pointer_manager_handle_button_press(
    FlPointerManager* manager,
    guint event_time,
    FlutterPointerDeviceKind device_kind,
    gdouble x,
    gdouble y,
    int64_t button);

/**
 * fl_pointer_manager_handle_button_release:
 * @manager: an #FlPointerManager.
 * @event_time: event time in milliseconds.
 * @device_kind: kind of device generating the event.
 * @x: x co-ordinate of event.
 * @y: y co-ordinate of event.
 * @button: button being released.
 *
 * Returns %TRUE if this event was handled.
 */
gboolean fl_pointer_manager_handle_button_release(
    FlPointerManager* manager,
    guint event_time,
    FlutterPointerDeviceKind device_kind,
    gdouble x,
    gdouble y,
    int64_t button);

/**
 * fl_pointer_manager_handle_motion:
 * @manager: an #FlPointerManager.
 * @event_time: event time in milliseconds.
 * @device_kind: kind of device generating the event.
 * @x: x co-ordinate of event.
 * @y: y co-ordinate of event.
 *
 * Returns %TRUE if this event was handled.
 */
gboolean fl_pointer_manager_handle_motion(FlPointerManager* manager,
                                          guint event_time,
                                          FlutterPointerDeviceKind device_kind,
                                          gdouble x,
                                          gdouble y);

/**
 * fl_pointer_manager_handle_enter:
 * @manager: an #FlPointerManager.
 * @event_time: event time in milliseconds.
 * @device_kind: kind of device generating the event.
 * @x: x co-ordinate of event.
 * @y: y co-ordinate of event.
 *
 * Returns %TRUE if this event was handled.
 */
gboolean fl_pointer_manager_handle_enter(FlPointerManager* manager,
                                         guint event_time,
                                         FlutterPointerDeviceKind device_kind,
                                         gdouble x,
                                         gdouble y);

/**
 * fl_pointer_manager_handle_leave:
 * @manager: an #FlPointerManager.
 * @event_time: event time in milliseconds.
 * @device_kind: kind of device generating the event.
 * @x: x co-ordinate of event.
 * @y: y co-ordinate of event.
 *
 * Returns %TRUE if this event was handled.
 */
gboolean fl_pointer_manager_handle_leave(FlPointerManager* manager,
                                         guint event_time,
                                         FlutterPointerDeviceKind device_kind,
                                         gdouble x,
                                         gdouble y);
G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_POINTER_MANAGER_H_
