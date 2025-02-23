// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_WINDOW_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_WINDOW_H_

#include "gmock/gmock.h"

#include <gtk/gtk.h>

namespace flutter {
namespace testing {

class MockWindow {
 public:
  MockWindow();
  ~MockWindow();

  MOCK_METHOD(GdkWindowState, gdk_window_get_state, (GdkWindow * window));
  MOCK_METHOD(void, gtk_window_new, (GtkWindow * window, GtkWindowType type));
  MOCK_METHOD(void,
              gtk_window_set_default_size,
              (GtkWindow * window, gint width, gint height));
  MOCK_METHOD(void,
              gtk_window_set_title,
              (GtkWindow * window, const gchar* title));
  MOCK_METHOD(void,
              gtk_window_set_geometry_hints,
              (GtkWindow * window,
               GtkWidget* widget,
               GdkGeometry* geometry,
               GdkWindowHints geometry_mask));
  MOCK_METHOD(void,
              gtk_window_resize,
              (GtkWindow * window, gint width, gint height));
  MOCK_METHOD(void, gtk_window_maximize, (GtkWindow * window));
  MOCK_METHOD(void, gtk_window_unmaximize, (GtkWindow * window));
  MOCK_METHOD(gboolean, gtk_window_is_maximized, (GtkWindow * window));
  MOCK_METHOD(void, gtk_window_iconify, (GtkWindow * window));
  MOCK_METHOD(void, gtk_window_deiconify, (GtkWindow * window));
  MOCK_METHOD(void, gtk_widget_destroy, (GtkWidget * widget));
};

}  // namespace testing
}  // namespace flutter

// Calls original gtk_widget_destroy.
void fl_gtk_widget_destroy(GtkWidget* widget);

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_WINDOW_H_
