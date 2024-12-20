// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_METHOD_CALL_PRIVATE_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_METHOD_CALL_PRIVATE_H_

#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_call.h"

#include "flutter/shell/platform/linux/public/flutter_linux/fl_binary_messenger.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_channel.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_value.h"

G_BEGIN_DECLS

/**
 * fl_method_call_new:
 * @name: a method name.
 * @args: arguments provided to a method.
 * @channel: channel call received on.
 * @response_handle: handle to respond with.
 *
 * Creates a method call.
 *
 * Returns: a new #FlMethodCall.
 */
FlMethodCall* fl_method_call_new(
    const gchar* name,
    FlValue* args,
    FlMethodChannel* channel,
    FlBinaryMessengerResponseHandle* response_handle);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_METHOD_CALL_PRIVATE_H_
