// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_WINDOW_STATE_MONITOR_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_WINDOW_STATE_MONITOR_H_

#include <gtk/gtk.h>

#include "flutter/shell/platform/linux/public/flutter_linux/fl_engine.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlWindowStateMonitor,
                     fl_window_state_monitor,
                     FL,
                     WINDOW_STATE_MONITOR,
                     GObject);

/**
 * FlWindowStateMonitor:
 *
 * Monitors a GtkWindow and reports state change events to the Flutter engine.
 */

/**
 * fl_window_state_monitor_new:
 * @messenger: an #FlBinaryMessenger.
 * @window: a #GtkWindow.
 *
 * Creates a new window state manager to monitor @window and report events to
 * @messenger.
 *
 * Returns: a new #FlWindowStateMonitor.
 */
FlWindowStateMonitor* fl_window_state_monitor_new(FlBinaryMessenger* messenger,
                                                  GtkWindow* window);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_WINDOW_STATE_MONITOR_H_
