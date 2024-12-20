// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_RENDERABLE_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_RENDERABLE_H_

#include <gdk/gdk.h>

#include "flutter/shell/platform/embedder/embedder.h"

G_BEGIN_DECLS

G_DECLARE_INTERFACE(FlRenderable, fl_renderable, FL, RENDERABLE, GObject);

/**
 * FlRenderable:
 *
 * An interface for a class that can render views from #FlRenderer.
 *
 * This interface is typically implemented by #FlView and is provided to make
 * #FlRenderer easier to test.
 */

struct _FlRenderableInterface {
  GTypeInterface g_iface;

  void (*redraw)(FlRenderable* renderable);
  void (*make_current)(FlRenderable* renderable);
};

/**
 * fl_renderable_redraw:
 * @renderable: an #FlRenderable
 *
 * Indicate the renderable needs to redraw. When ready, the renderable should
 * call fl_renderer_draw().
 */
void fl_renderable_redraw(FlRenderable* renderable);

/**
 * fl_renderable_make_current:
 * @renderable: an #FlRenderable
 *
 * Make this renderable the current OpenGL context.
 */
void fl_renderable_make_current(FlRenderable* renderable);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_RENDERABLE_H_
