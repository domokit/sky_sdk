// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/testing/mock_window.h"

using namespace flutter::testing;

static MockWindow* mock = nullptr;

MockWindow::MockWindow() {
  mock = this;
}

GdkDisplay* gdk_display_get_default() {
  return GDK_DISPLAY(g_object_new(gdk_display_get_type(), nullptr));
}

void gdk_display_beep(GdkDisplay* display) {}

GdkWindowState gdk_window_get_state(GdkWindow* window) {
  return mock->gdk_window_get_state(window);
}

GdkDisplay* gdk_window_get_display(GdkWindow* window) {
  return GDK_DISPLAY(g_object_new(gdk_display_get_type(), nullptr));
}

int gdk_display_get_n_monitors(GdkDisplay* display) {
  return 1;
}

GdkMonitor* gdk_display_get_monitor(GdkDisplay* display, int n) {
  return GDK_MONITOR(g_object_new(gdk_monitor_get_type(), nullptr));
}

GdkMonitor* gdk_display_get_monitor_at_window(GdkDisplay* display,
                                              GdkWindow* window) {
  return nullptr;
}

GdkCursor* gdk_cursor_new_from_name(GdkDisplay* display, const gchar* name) {
  return nullptr;
}

void gdk_window_set_cursor(GdkWindow* window, GdkCursor* cursor) {}
