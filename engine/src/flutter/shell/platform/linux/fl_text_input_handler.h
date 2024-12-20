// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_TEXT_INPUT_HANDLER_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_TEXT_INPUT_HANDLER_H_

#include <gtk/gtk.h>

#include "flutter/shell/platform/linux/fl_key_event.h"
#include "flutter/shell/platform/linux/fl_text_input_view_delegate.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_binary_messenger.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlTextInputHandler,
                     fl_text_input_handler,
                     FL,
                     TEXT_INPUT_HANDLER,
                     GObject);

/**
 * FlTextInputHandler:
 *
 * #FlTextInputHandler is a handler that implements the shell side
 * of SystemChannels.textInput from the Flutter services library.
 */

/**
 * fl_text_input_handler_new:
 * @messenger: an #FlBinaryMessenger.
 * @im_context: (allow-none): a #GtkIMContext.
 * @view_delegate: an #FlTextInputViewDelegate.
 *
 * Creates a new handler that implements SystemChannels.textInput from the
 * Flutter services library.
 *
 * Returns: a new #FlTextInputHandler.
 */
FlTextInputHandler* fl_text_input_handler_new(
    FlBinaryMessenger* messenger,
    GtkIMContext* im_context,
    FlTextInputViewDelegate* view_delegate);

/**
 * fl_text_input_handler_filter_keypress
 * @handler: an #FlTextInputHandler.
 * @event: a #FlKeyEvent
 *
 * Process a Gdk key event.
 *
 * Returns: %TRUE if the event was used.
 */
gboolean fl_text_input_handler_filter_keypress(FlTextInputHandler* handler,
                                               FlKeyEvent* event);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_TEXT_INPUT_HANDLER_H_
