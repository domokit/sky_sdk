// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_METHOD_CHANNEL_PRIVATE_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_METHOD_CHANNEL_PRIVATE_H_

#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_channel.h"

#include "flutter/shell/platform/linux/public/flutter_linux/fl_binary_messenger.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_response.h"

G_BEGIN_DECLS

/**
 * fl_method_channel_respond:
 * @channel: an #FlMethodChannel.
 * @response_handle: an #FlBinaryMessengerResponseHandle.
 * @response: an #FlMethodResponse.
 * @error: (allow-none): #GError location to store the error occurring, or %NULL
 * to ignore.
 *
 * Responds to a method call.
 *
 * Returns: %TRUE on success.
 */
gboolean fl_method_channel_respond(
    FlMethodChannel* channel,
    FlBinaryMessengerResponseHandle* response_handle,
    FlMethodResponse* response,
    GError** error);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_METHOD_CHANNEL_PRIVATE_H_
