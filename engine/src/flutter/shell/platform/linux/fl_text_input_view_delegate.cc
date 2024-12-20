// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_text_input_view_delegate.h"

G_DEFINE_INTERFACE(FlTextInputViewDelegate,
                   fl_text_input_view_delegate,
                   G_TYPE_OBJECT)

static void fl_text_input_view_delegate_default_init(
    FlTextInputViewDelegateInterface* iface) {}

void fl_text_input_view_delegate_translate_coordinates(
    FlTextInputViewDelegate* self,
    gint view_x,
    gint view_y,
    gint* window_x,
    gint* window_y) {
  g_return_if_fail(FL_IS_TEXT_INPUT_VIEW_DELEGATE(self));

  FL_TEXT_INPUT_VIEW_DELEGATE_GET_IFACE(self)->translate_coordinates(
      self, view_x, view_y, window_x, window_y);
}
