// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_TEXTURE_PRIVATE_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_TEXTURE_PRIVATE_H_

#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_texture.h"

G_BEGIN_DECLS

/**
 * fl_texture_set_id:
 * @texture: an #FlTexture.
 * @id: a texture ID.
 *
 * Set the ID for a texture.
 */
void fl_texture_set_id(FlTexture* texture, int64_t id);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_TEXTURE_PRIVATE_H_
