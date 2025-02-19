// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_IM_CONTEXT_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_IM_CONTEXT_H_

#include <gtk/gtk.h>

#include "gmock/gmock.h"

namespace flutter {
namespace testing {

class MockIMContext {
 public:
  MockIMContext();
  ~MockIMContext();

  MOCK_METHOD(void,
              gtk_im_context_set_client_window,
              (GtkIMContext * context, GdkWindow* window));
  MOCK_METHOD(void,
              gtk_im_context_get_preedit_string,
              (GtkIMContext * context,
               gchar** str,
               PangoAttrList** attrs,
               gint* cursor_pos));
  MOCK_METHOD(gboolean,
              gtk_im_context_filter_keypress,
              (GtkIMContext * context, GdkEventKey* event));
  MOCK_METHOD(gboolean, gtk_im_context_focus_in, (GtkIMContext * context));
  MOCK_METHOD(void, gtk_im_context_focus_out, (GtkIMContext * context));
  MOCK_METHOD(void,
              gtk_im_context_set_cursor_location,
              (GtkIMContext * context, const GdkRectangle* area));
  MOCK_METHOD(
      void,
      gtk_im_context_set_surrounding,
      (GtkIMContext * context, const gchar* text, gint len, gint cursor_index));
  MOCK_METHOD(gboolean,
              gtk_widget_translate_coordinates,
              (GtkWidget * src_widget,
               GtkWidget* dest_widget,
               gint src_x,
               gint src_y,
               gint* dest_x,
               gint* dest_y));
  MOCK_METHOD(GtkWidget*, gtk_widget_get_toplevel, (GtkWidget * widget));
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_IM_CONTEXT_H_
