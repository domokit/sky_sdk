// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_KEYBOARD_HANDLER_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_KEYBOARD_HANDLER_H_

#include <gdk/gdk.h>

#include "flutter/shell/platform/linux/fl_keyboard_manager.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_binary_messenger.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlKeyboardHandler,
                     fl_keyboard_handler,
                     FL,
                     KEYBOARD_HANDLER,
                     GObject);

/**
 * FlKeyboardHandler:
 *
 * Provides the channel to receive keyboard requests from the Dart code.
 */

/**
 * fl_keyboard_handler_new:
 * @messenger: a #FlBinaryMessenger.
 * @keyboard_manager: a #FlKeyboardManager.
 *
 * Create a new #FlKeyboardHandler.
 *
 * Returns: a new #FlKeyboardHandler.
 */
FlKeyboardHandler* fl_keyboard_handler_new(FlBinaryMessenger* messenger,
                                           FlKeyboardManager* keyboard_manager);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_KEYBOARD_HANDLER_H_
