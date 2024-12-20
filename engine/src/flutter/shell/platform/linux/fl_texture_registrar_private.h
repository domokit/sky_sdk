// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_TEXTURE_REGISTRAR_PRIVATE_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_TEXTURE_REGISTRAR_PRIVATE_H_

#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_engine.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_texture_registrar.h"

G_BEGIN_DECLS

/**
 * fl_texture_registrar_new:
 * @engine: an #FlEngine.
 *
 * Creates a new #FlTextureRegistrar.
 *
 * Returns: a new #FlTextureRegistrar.
 */
FlTextureRegistrar* fl_texture_registrar_new(FlEngine* engine);

/**
 * fl_texture_registrar_lookup_texture:
 * @registrar: an #FlTextureRegistrar.
 * @texture_id: ID of texture.
 *
 * Looks for the texture with the given ID.
 *
 * Returns: an #FlTexture or %NULL if no texture with this ID.
 */
FlTexture* fl_texture_registrar_lookup_texture(FlTextureRegistrar* registrar,
                                               int64_t texture_id);

/**
 * fl_texture_registrar_shutdown:
 * @registrar: an #FlTextureRegistrar.
 *
 * Shutdown the registrary and unregister any textures.
 */
void fl_texture_registrar_shutdown(FlTextureRegistrar* registrar);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_TEXTURE_REGISTRAR_PRIVATE_H_
