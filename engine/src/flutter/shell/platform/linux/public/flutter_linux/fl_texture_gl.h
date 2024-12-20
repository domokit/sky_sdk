// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_TEXTURE_GL_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_TEXTURE_GL_H_

#if !defined(__FLUTTER_LINUX_INSIDE__) && !defined(FLUTTER_LINUX_COMPILATION)
#error "Only <flutter_linux/flutter_linux.h> can be included directly."
#endif

#include <glib-object.h>
#include <gmodule.h>
#include <stdint.h>
#include "fl_texture.h"

G_BEGIN_DECLS

G_MODULE_EXPORT
G_DECLARE_DERIVABLE_TYPE(FlTextureGL, fl_texture_gl, FL, TEXTURE_GL, GObject)

/**
 * FlTextureGL:
 *
 * #FlTextureGL is an abstract class that represents an OpenGL texture.
 *
 * If you want to render textures in other OpenGL context, create and use the
 * #GdkGLContext by calling gdk_window_create_gl_context () with the #GdkWindow
 * of #FlView. The context will be shared with the one used by Flutter.
 *
 * The following example shows how to implement an #FlTextureGL.
 * ![<!-- language="C" -->
 *   #include <epoxy/gl.h>
 *
 *   struct _MyTextureGL {
 *     FlTextureGL parent_instance;
 *
 *     GLuint texture_id;
 *   };
 *
 *   G_DEFINE_TYPE(MyTextureGL,
 *                 my_texture_gl,
 *                 fl_texture_gl_get_type ())
 *
 *   static gboolean
 *   my_texture_gl_populate (FlTextureGL *texture,
 *                           uint32_t *target,
 *                           uint32_t *name,
 *                           uint32_t *width,
 *                           uint32_t *height,
 *                           GError **error) {
 *     MyTextureGL *self = MY_TEXTURE_GL (texture);
 *     if (self->texture_id == 0) {
 *       glGenTextures (1, &self->texture_id);
 *       glBindTexture (GL_TEXTURE_2D, self->texture_id);
 *       // further configuration here.
 *     } else {
 *       glBindTexture (GL_TEXTURE_2D, self->texture_id);
 *     }
 *
 *     // For example, we render pixel buffer here.
 *     // Note that Flutter only accepts textures in GL_RGBA8 format.
 *     static char buffer[] = { 0x1f, 0x2f, 0x3f, 0x4f };  // 1x1 pixel.
 *     glTexImage2D (GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA,
 *                   GL_UNSIGNED_BYTE, buffer);
 *
 *     *target = GL_TEXTURE_2D;
 *     *name = self->texture_id;
 *     *width = 1;
 *     *height = 1;
 *
 *     return TRUE;
 *   }
 *
 *   static void my_texture_class_init(MyTextureClass* klass) {
 *     FL_TEXTURE_GL_CLASS(klass)->populate = my_texture_gl_populate;
 *   }
 *
 *   static void my_texture_init(MyTexture* self) {}
 * ]|
 */

struct _FlTextureGLClass {
  GObjectClass parent_class;

  /**
   * Virtual method called when Flutter populates this texture. The OpenGL
   * context used by Flutter has been already set.
   * @texture: an #FlTexture.
   * @target: texture target (example GL_TEXTURE_2D or GL_TEXTURE_RECTANGLE).
   * @name: (out): name of texture.
   * @width: (inout): width of the texture in pixels.
   * @height: (inout): height of the texture in pixels.
   * @error: (allow-none): #GError location to store the error occurring, or
   * %NULL to ignore.
   *
   * Returns: %TRUE on success.
   */
  gboolean (*populate)(FlTextureGL* texture,
                       uint32_t* target,
                       uint32_t* name,
                       uint32_t* width,
                       uint32_t* height,
                       GError** error);
};

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_TEXTURE_GL_H_
