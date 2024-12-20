// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_PLUGIN_REGISTRAR_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_PLUGIN_REGISTRAR_H_

#include "flutter/shell/platform/linux/public/flutter_linux/fl_binary_messenger.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_plugin_registrar.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_texture_registrar.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlMockPluginRegistrar,
                     fl_mock_plugin_registrar,
                     FL,
                     MOCK_PLUGIN_REGISTRAR,
                     GObject)

FlPluginRegistrar* fl_mock_plugin_registrar_new(
    FlBinaryMessenger* messenger,
    FlTextureRegistrar* texture_registrar);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_PLUGIN_REGISTRAR_H_
