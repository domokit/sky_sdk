// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/testing/mock_im_context.h"

using namespace flutter::testing;

static MockIMContext* mock = nullptr;

MockIMContext::MockIMContext() {
  mock = this;
}

MockIMContext::~MockIMContext() {
  if (mock == this) {
    mock = nullptr;
  }
}

void gtk_im_context_set_client_window(GtkIMContext* context,
                                      GdkWindow* window) {
  if (mock != nullptr) {
    mock->gtk_im_context_set_client_window(context, window);
  }
}

void gtk_im_context_get_preedit_string(GtkIMContext* context,
                                       gchar** str,
                                       PangoAttrList** attrs,
                                       gint* cursor_pos) {
  if (mock != nullptr) {
    mock->gtk_im_context_get_preedit_string(context, str, attrs, cursor_pos);
  }
}

gboolean gtk_im_context_filter_keypress(GtkIMContext* context,
                                        GdkEventKey* event) {
  if (mock == nullptr) {
    return TRUE;
  }

  return mock->gtk_im_context_filter_keypress(context, event);
}

void gtk_im_context_focus_in(GtkIMContext* context) {
  if (mock != nullptr) {
    mock->gtk_im_context_focus_in(context);
  }
}

void gtk_im_context_focus_out(GtkIMContext* context) {
  if (mock != nullptr) {
    mock->gtk_im_context_focus_out(context);
  }
}

void gtk_im_context_set_cursor_location(GtkIMContext* context,
                                        const GdkRectangle* area) {
  if (mock != nullptr) {
    mock->gtk_im_context_set_cursor_location(context, area);
  }
}

void gtk_im_context_set_surrounding(GtkIMContext* context,
                                    const gchar* text,
                                    gint len,
                                    gint cursor_index) {
  if (mock != nullptr) {
    mock->gtk_im_context_set_surrounding(context, text, len, cursor_index);
  }
}

gboolean gtk_widget_translate_coordinates(GtkWidget* src_widget,
                                          GtkWidget* dest_widget,
                                          gint src_x,
                                          gint src_y,
                                          gint* dest_x,
                                          gint* dest_y) {
  if (mock == nullptr) {
    *dest_x = src_x;
    *dest_y = src_y;
    return TRUE;
  }

  return mock->gtk_widget_translate_coordinates(src_widget, dest_widget, src_x,
                                                src_y, dest_x, dest_y);
}

GtkWidget* gtk_widget_get_toplevel(GtkWidget* widget) {
  return widget;
}
