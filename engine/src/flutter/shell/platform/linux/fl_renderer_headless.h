// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_RENDERER_HEADLESS_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_RENDERER_HEADLESS_H_

#include "flutter/shell/platform/linux/fl_renderer.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlRendererHeadless,
                     fl_renderer_headless,
                     FL,
                     RENDERER_HEADLESS,
                     FlRenderer)

/**
 * FlRendererHeadless:
 *
 * #FlRendererHeadless is an implementation of #FlRenderer that works without a
 * display.
 */

/**
 * fl_renderer_headless_new:
 *
 * Creates an object that allows Flutter to operate without a display.
 *
 * Returns: a new #FlRendererHeadless.
 */
FlRendererHeadless* fl_renderer_headless_new();

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_RENDERER_HEADLESS_H_
