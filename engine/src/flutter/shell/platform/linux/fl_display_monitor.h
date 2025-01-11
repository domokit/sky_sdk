// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_DISPLAY_MONITOR_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_DISPLAY_MONITOR_H_

#include <gdk/gdk.h>

#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_engine.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlDisplayMonitor,
                     fl_display_monitor,
                     FL,
                     DISPLAY_MONITOR,
                     GObject);

/**
 * fl_display_monitor_new:
 * @engine: engine to update.
 * @display: display to monitor.
 *
 * Creates a new object to keep the engine updated with the currently used
 * displays. In GDK, a display is called a "monitor".
 *
 * Returns: a new #FlDisplayMontior.
 */
FlDisplayMonitor* fl_display_monitor_new(FlEngine* engine, GdkDisplay* display);

/**
 * fl_display_monitor_start:
 * @monitor: an #FlDisplayMonitor.
 *
 * Start monitoring for display changes.
 */
void fl_display_monitor_start(FlDisplayMonitor* monitor);

/**
 * fl_display_monitor_get_display_id:
 * @monitor: an #FlDisplayMonitor.
 * @gdk_monitor: GDK monitor to get display ID for.
 *
 * Get the ID Flutter is using for a given monitor.
 *
 * Returns: an ID or 0 if unknown.
 */
FlutterEngineDisplayId fl_display_monitor_get_display_id(
    FlDisplayMonitor* monitor,
    GdkMonitor* gdk_monitor);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_DISPLAY_MONITOR_H_
