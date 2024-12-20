// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_VIEW_ACCESSIBLE_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_VIEW_ACCESSIBLE_H_

#if !defined(__FLUTTER_LINUX_INSIDE__) && !defined(FLUTTER_LINUX_COMPILATION)
#error "Only <flutter_linux/flutter_linux.h> can be included directly."
#endif

#include <atk/atk.h>

#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_engine.h"

G_BEGIN_DECLS

// ATK g_autoptr macros weren't added until 2.37. Add them manually.
// https://gitlab.gnome.org/GNOME/atk/-/issues/10
#if !ATK_CHECK_VERSION(2, 37, 0)
G_DEFINE_AUTOPTR_CLEANUP_FUNC(AtkPlug, g_object_unref)
#endif

G_DECLARE_FINAL_TYPE(FlViewAccessible,
                     fl_view_accessible,
                     FL,
                     VIEW_ACCESSIBLE,
                     AtkPlug)

/**
 * FlViewAccessible:
 *
 * #FlViewAccessible is an object that exposes accessibility information for an
 * #FlView.
 */

/**
 * fl_view_accessible_new:
 *
 * Creates a new accessibility object that exposes Flutter accessibility
 * information to ATK.
 *
 * Returns: a new #FlViewAccessible.
 */
FlViewAccessible* fl_view_accessible_new(FlEngine* engine);

/**
 * fl_view_accessible_handle_update_semantics:
 * @accessible: an #FlViewAccessible.
 * @update: semantic update information.
 *
 * Handle a semantics update from Flutter.
 */
void fl_view_accessible_handle_update_semantics(
    FlViewAccessible* accessible,
    const FlutterSemanticsUpdate2* update);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_VIEW_ACCESSIBLE_H_
