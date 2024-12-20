// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_keyboard_view_delegate.h"

G_DEFINE_INTERFACE(FlKeyboardViewDelegate,
                   fl_keyboard_view_delegate,
                   G_TYPE_OBJECT)

static void fl_keyboard_view_delegate_default_init(
    FlKeyboardViewDelegateInterface* iface) {}

gboolean fl_keyboard_view_delegate_text_filter_key_press(
    FlKeyboardViewDelegate* self,
    FlKeyEvent* event) {
  g_return_val_if_fail(FL_IS_KEYBOARD_VIEW_DELEGATE(self), false);
  g_return_val_if_fail(event != nullptr, false);

  return FL_KEYBOARD_VIEW_DELEGATE_GET_IFACE(self)->text_filter_key_press(
      self, event);
}
