// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_RENDERER_GDK_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_RENDERER_GDK_H_

#include "flutter/shell/platform/linux/fl_renderer.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlRendererGdk,
                     fl_renderer_gdk,
                     FL,
                     RENDERER_GDK,
                     FlRenderer)

/**
 * FlRendererGdk:
 *
 * #FlRendererGdk is an implementation of #FlRenderer that renders by OpenGL ES.
 */

/**
 * fl_renderer_gdk_new:
 *
 * Creates an object that allows Flutter to render by OpenGL ES.
 *
 * Returns: a new #FlRendererGdk.
 */
FlRendererGdk* fl_renderer_gdk_new();

/**
 * fl_renderer_gdk_set_window:
 * @renderer: an #FlRendererGdk.
 * @window: the window that is being rendered on.
 *
 * Set the window that is being rendered on. This is only called once when the
 * window is available.
 */
void fl_renderer_gdk_set_window(FlRendererGdk* renderer, GdkWindow* window);

/**
 * fl_renderer_gdk_create_contexts:
 * @renderer: an #FlRendererGdk.
 * @error: (allow-none): #GError location to store the error occurring, or
 * %NULL to ignore.
 *
 * Create rendering contexts.
 *
 * Returns: %TRUE if contexts were created, %FALSE if there was an error.
 */
gboolean fl_renderer_gdk_create_contexts(FlRendererGdk* renderer,
                                         GError** error);

/**
 * fl_renderer_gdk_get_context:
 * @renderer: an #FlRendererGdk.
 *
 * Returns: the main context used for rendering.
 */
GdkGLContext* fl_renderer_gdk_get_context(FlRendererGdk* renderer);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_RENDERER_GDK_H_
