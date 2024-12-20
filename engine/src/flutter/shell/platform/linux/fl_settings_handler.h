// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_SETTINGS_HANDLER_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_SETTINGS_HANDLER_H_

#include "flutter/shell/platform/linux/fl_settings.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_engine.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlSettingsHandler,
                     fl_settings_handler,
                     FL,
                     SETTINGS_HANDLER,
                     GObject);

/**
 * FlSettingsHandler:
 *
 * #FlSettingsHandler is a handler that implements the Flutter user settings
 * channel.
 */

/**
 * fl_settings_handler_new:
 * @messenger: an #FlEngine
 *
 * Creates a new handler that sends user settings to the Flutter engine.
 *
 * Returns: a new #FlSettingsHandler
 */
FlSettingsHandler* fl_settings_handler_new(FlEngine* engine);

/**
 * fl_settings_handler_start:
 * @handler: an #FlSettingsHandler.
 *
 * Sends the current settings to the engine and updates when they change.
 */
void fl_settings_handler_start(FlSettingsHandler* handler,
                               FlSettings* settings);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_SETTINGS_HANDLER_H_
