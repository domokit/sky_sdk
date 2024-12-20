// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_PLUGIN_REGISTRAR_PRIVATE_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_PLUGIN_REGISTRAR_PRIVATE_H_

#include "flutter/shell/platform/linux/public/flutter_linux/fl_binary_messenger.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_plugin_registrar.h"

G_BEGIN_DECLS

/**
 * fl_plugin_registrar_new:
 * @view: (allow-none): the #FlView that is being plugged into or %NULL for
 * headless mode.
 * @messenger: the #FlBinaryMessenger to communicate with.
 * @texture_registrar: the #FlTextureRegistrar to communicate with.
 *
 * Creates a new #FlPluginRegistrar.
 *
 * Returns: a new #FlPluginRegistrar.
 */
FlPluginRegistrar* fl_plugin_registrar_new(
    FlView* view,
    FlBinaryMessenger* messenger,
    FlTextureRegistrar* texture_registrar);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_PLUGIN_REGISTRAR_PRIVATE_H_
