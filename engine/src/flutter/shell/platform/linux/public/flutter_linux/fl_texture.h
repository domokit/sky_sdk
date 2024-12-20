// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_TEXTURE_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_TEXTURE_H_

#if !defined(__FLUTTER_LINUX_INSIDE__) && !defined(FLUTTER_LINUX_COMPILATION)
#error "Only <flutter_linux/flutter_linux.h> can be included directly."
#endif

#include <glib-object.h>
#include <gmodule.h>
#include <stdint.h>

G_BEGIN_DECLS

G_MODULE_EXPORT
G_DECLARE_INTERFACE(FlTexture, fl_texture, FL, TEXTURE, GObject)

/**
 * FlTexture:
 *
 * #FlTexture represents a texture.
 *
 * You can derive #FlTextureGL for populating hardware-accelerated textures or
 * instantiate #FlPixelBufferTexture for populating pixel buffers. Do NOT
 * directly implement this interface.
 */

struct _FlTextureInterface {
  GTypeInterface g_iface;

  void (*set_id)(FlTexture* texture, int64_t id);

  int64_t (*get_id)(FlTexture* texture);
};

/**
 * fl_texture_get_id:
 * @texture: a #FlTexture.
 *
 * Get the ID for this texture, which can be passed to Flutter code to refer to
 * this texture.
 *
 * Returns: a texture ID.
 */
int64_t fl_texture_get_id(FlTexture* texture);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_TEXTURE_H_
