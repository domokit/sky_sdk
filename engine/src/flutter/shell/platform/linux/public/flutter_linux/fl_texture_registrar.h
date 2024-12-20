// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_TEXTURE_REGISTRAR_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_TEXTURE_REGISTRAR_H_

#if !defined(__FLUTTER_LINUX_INSIDE__) && !defined(FLUTTER_LINUX_COMPILATION)
#error "Only <flutter_linux/flutter_linux.h> can be included directly."
#endif

#include <glib-object.h>
#include <gmodule.h>
#include <stdint.h>

#include "fl_texture.h"

G_BEGIN_DECLS

G_MODULE_EXPORT
G_DECLARE_INTERFACE(FlTextureRegistrar,
                    fl_texture_registrar,
                    FL,
                    TEXTURE_REGISTRAR,
                    GObject)

struct _FlTextureRegistrarInterface {
  GTypeInterface parent_iface;

  gboolean (*register_texture)(FlTextureRegistrar* registrar,
                               FlTexture* texture);

  FlTexture* (*lookup_texture)(FlTextureRegistrar* registrar, int64_t id);

  gboolean (*mark_texture_frame_available)(FlTextureRegistrar* registrar,
                                           FlTexture* texture);

  gboolean (*unregister_texture)(FlTextureRegistrar* registrar,
                                 FlTexture* texture);

  void (*shutdown)(FlTextureRegistrar* registrar);
};

/**
 * FlTextureRegistrar:
 *
 * #FlTextureRegistrar is used when registering textures.
 *
 * Flutter Framework accesses your texture by the related unique texture ID. To
 * draw your texture in Dart, you should add Texture widget in your widget tree
 * with the same texture ID. Use platform channels to send this unique texture
 * ID to the Dart side.
 */

/**
 * fl_texture_registrar_register_texture:
 * @registrar: an #FlTextureRegistrar.
 * @texture: an #FlTexture for registration.
 *
 * Registers a texture.
 *
 * Returns: %TRUE on success.
 */
gboolean fl_texture_registrar_register_texture(FlTextureRegistrar* registrar,
                                               FlTexture* texture);

/**
 * fl_texture_registrar_mark_texture_frame_available:
 * @registrar: an #FlTextureRegistrar.
 * @texture: the texture that has a frame available.
 *
 * Notifies the flutter engine that the texture object has updated and needs to
 * be rerendered.
 *
 * Returns: %TRUE on success.
 */
gboolean fl_texture_registrar_mark_texture_frame_available(
    FlTextureRegistrar* registrar,
    FlTexture* texture);

/**
 * fl_texture_registrar_unregister_texture:
 * @registrar: an #FlTextureRegistrar.
 * @texture: the texture being unregistered.
 *
 * Unregisters an existing texture object.
 *
 * Returns: %TRUE on success.
 */
gboolean fl_texture_registrar_unregister_texture(FlTextureRegistrar* registrar,
                                                 FlTexture* texture);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_TEXTURE_REGISTRAR_H_
