// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_PLATFORM_HANDLER_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_PLATFORM_HANDLER_H_

#include "flutter/shell/platform/linux/public/flutter_linux/fl_binary_messenger.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlPlatformHandler,
                     fl_platform_handler,
                     FL,
                     PLATFORM_HANDLER,
                     GObject);

/**
 * FlPlatformHandler:
 *
 * #FlPlatformHandler is a handler that implements the shell side
 * of SystemChannels.platform from the Flutter services library.
 */

/**
 * fl_platform_handler_new:
 * @messenger: an #FlBinaryMessenger
 *
 * Creates a new handler that implements SystemChannels.platform from the
 * Flutter services library.
 *
 * Returns: a new #FlPlatformHandler
 */
FlPlatformHandler* fl_platform_handler_new(FlBinaryMessenger* messenger);

/**
 * fl_platform_handler_request_app_exit:
 * @handler: an #FlPlatformHandler
 *
 * Request the application exits (i.e. due to the window being requested to be
 * closed).
 *
 * Calling this will only send an exit request to the framework if the framework
 * has already indicated that it is ready to receive requests by sending a
 * "System.initializationComplete" method call on the platform channel. Calls
 * before initialization is complete will result in an immediate exit.
 */
void fl_platform_handler_request_app_exit(FlPlatformHandler* handler);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_PLATFORM_HANDLER_H_
