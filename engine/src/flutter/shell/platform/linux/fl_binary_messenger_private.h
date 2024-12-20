// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_BINARY_MESSENGER_PRIVATE_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_BINARY_MESSENGER_PRIVATE_H_

#include <glib-object.h>

#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_engine.h"

G_BEGIN_DECLS

/**
 * fl_binary_messenger_new:
 * @engine: The #FlEngine to communicate with.
 *
 * Creates a new #FlBinaryMessenger. The binary messenger will take control of
 * the engines platform message handler.
 *
 * Returns: a new #FlBinaryMessenger.
 */
FlBinaryMessenger* fl_binary_messenger_new(FlEngine* engine);

/**
 * fl_binary_messenger_handle_message:
 * @messenger: an #FlBinaryMessenger.
 * @channel: channel message received on.
 * @message: message data.
 * @response_handle: handle to provide to
 * fl_engine_send_platform_message_response().
 *
 * Handles a message received from the engine. Available for testing purposes.
 *
 * Returns: %TRUE if the message is handled.
 */
gboolean fl_binary_messenger_handle_message(
    FlBinaryMessenger* messenger,
    const gchar* channel,
    GBytes* message,
    const FlutterPlatformMessageResponseHandle* response_handle);

/**
 * fl_binary_messenger_shutdown:
 * @messenger: an #FlBinaryMessenger.
 *
 * Shutdown the messenger closing any open channels.
 */
void fl_binary_messenger_shutdown(FlBinaryMessenger* messenger);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_BINARY_MESSENGER_PRIVATE_H_
