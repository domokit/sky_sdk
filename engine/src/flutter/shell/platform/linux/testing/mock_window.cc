// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <dlfcn.h>

#include "flutter/shell/platform/linux/testing/mock_window.h"

using namespace flutter::testing;

static MockWindow* mock = nullptr;

MockWindow::MockWindow() {
  mock = this;
}

MockWindow::~MockWindow() {
  if (mock == this) {
    mock = nullptr;
  }
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

GtkWidget* gtk_window_new(GtkWindowType type) {
  GtkWindow* window = GTK_WINDOW(g_object_new(gtk_window_get_type(), nullptr));
  mock->gtk_window_new(window, type);
  return GTK_WIDGET(window);
}

void gtk_window_set_default_size(GtkWindow* window, gint width, gint height) {
  mock->gtk_window_set_default_size(window, width, height);
}

void gtk_window_set_title(GtkWindow* window, const gchar* title) {
  mock->gtk_window_set_title(window, title);
}

void gtk_window_set_geometry_hints(GtkWindow* window,
                                   GtkWidget* widget,
                                   GdkGeometry* geometry,
                                   GdkWindowHints geometry_mask) {
  mock->gtk_window_set_geometry_hints(window, widget, geometry, geometry_mask);
}

void gtk_window_resize(GtkWindow* window, gint width, gint height) {
  mock->gtk_window_resize(window, width, height);
}

void gtk_window_maximize(GtkWindow* window) {
  mock->gtk_window_maximize(window);
}

void gtk_window_unmaximize(GtkWindow* window) {
  mock->gtk_window_unmaximize(window);
}

gboolean gtk_window_is_maximized(GtkWindow* window) {
  return mock->gtk_window_is_maximized(window);
}

void gtk_window_iconify(GtkWindow* window) {
  mock->gtk_window_iconify(window);
}

void gtk_window_deiconify(GtkWindow* window) {
  mock->gtk_window_deiconify(window);
}

void gtk_widget_show(GtkWidget* widget) {}

void gtk_widget_destroy(GtkWidget* widget) {
  mock->gtk_widget_destroy(widget);
}

void fl_gtk_widget_destroy(GtkWidget* widget) {
  void (*destroy)(GtkWidget*) = reinterpret_cast<void (*)(GtkWidget*)>(
      dlsym(RTLD_NEXT, "gtk_widget_destroy"));
  destroy(widget);
}
