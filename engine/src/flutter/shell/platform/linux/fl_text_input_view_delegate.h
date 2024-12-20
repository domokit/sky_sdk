// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_TEXT_INPUT_VIEW_DELEGATE_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_TEXT_INPUT_VIEW_DELEGATE_H_

#include <gdk/gdk.h>

#include "flutter/shell/platform/embedder/embedder.h"

G_BEGIN_DECLS

G_DECLARE_INTERFACE(FlTextInputViewDelegate,
                    fl_text_input_view_delegate,
                    FL,
                    TEXT_INPUT_VIEW_DELEGATE,
                    GObject);

/**
 * FlTextInputViewDelegate:
 *
 * An interface for a class that provides `FlTextInputHandler` with
 * view-related features.
 *
 * This interface is typically implemented by `FlView`.
 */

struct _FlTextInputViewDelegateInterface {
  GTypeInterface g_iface;

  void (*translate_coordinates)(FlTextInputViewDelegate* delegate,
                                gint view_x,
                                gint view_y,
                                gint* window_x,
                                gint* window_y);
};

void fl_text_input_view_delegate_translate_coordinates(
    FlTextInputViewDelegate* delegate,
    gint view_x,
    gint view_y,
    gint* window_x,
    gint* window_y);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_TEXT_INPUT_VIEW_DELEGATE_H_
