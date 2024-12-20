// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_TEXTURE_REGISTRAR_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_TEXTURE_REGISTRAR_H_

#include "flutter/shell/platform/linux/fl_texture_registrar_private.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlMockTextureRegistrar,
                     fl_mock_texture_registrar,
                     FL,
                     MOCK_TEXTURE_REGISTRAR,
                     GObject)

FlMockTextureRegistrar* fl_mock_texture_registrar_new();

FlTexture* fl_mock_texture_registrar_get_texture(
    FlMockTextureRegistrar* registrar);

gboolean fl_mock_texture_registrar_get_frame_available(
    FlMockTextureRegistrar* registrar);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_TEXTURE_REGISTRAR_H_
