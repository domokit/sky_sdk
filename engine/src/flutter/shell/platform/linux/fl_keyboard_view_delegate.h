// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_KEYBOARD_VIEW_DELEGATE_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_KEYBOARD_VIEW_DELEGATE_H_

#include <gdk/gdk.h>
#include <cinttypes>
#include <functional>
#include <memory>

#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/linux/fl_key_event.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_binary_messenger.h"

G_BEGIN_DECLS

G_DECLARE_INTERFACE(FlKeyboardViewDelegate,
                    fl_keyboard_view_delegate,
                    FL,
                    KEYBOARD_VIEW_DELEGATE,
                    GObject);

/**
 * FlKeyboardViewDelegate:
 *
 * An interface for a class that provides `FlKeyboardHandler` with
 * platform-related features.
 *
 * This interface is typically implemented by `FlView`.
 */

struct _FlKeyboardViewDelegateInterface {
  GTypeInterface g_iface;

  gboolean (*text_filter_key_press)(FlKeyboardViewDelegate* delegate,
                                    FlKeyEvent* event);
};

/**
 * fl_keyboard_view_delegate_text_filter_key_press:
 *
 * Handles `FlKeyboardHandler`'s request to check if the GTK text input IM
 * filter would like to handle a GDK event.
 *
 * The ownership of the `event` is kept by the keyboard handler.
 */
gboolean fl_keyboard_view_delegate_text_filter_key_press(
    FlKeyboardViewDelegate* delegate,
    FlKeyEvent* event);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_KEYBOARD_VIEW_DELEGATE_H_
