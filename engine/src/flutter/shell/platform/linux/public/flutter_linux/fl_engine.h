// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_ENGINE_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_ENGINE_H_

#if !defined(__FLUTTER_LINUX_INSIDE__) && !defined(FLUTTER_LINUX_COMPILATION)
#error "Only <flutter_linux/flutter_linux.h> can be included directly."
#endif

#include <glib-object.h>
#include <gmodule.h>

#include "fl_binary_messenger.h"
#include "fl_dart_project.h"
#include "fl_texture_registrar.h"

G_BEGIN_DECLS

G_MODULE_EXPORT
G_DECLARE_FINAL_TYPE(FlEngine, fl_engine, FL, ENGINE, GObject)

/**
 * FlEngine:
 *
 * #FlEngine is an object that contains a running Flutter engine.
 */

/**
 * fl_engine_new:
 * @project: an #FlDartProject.
 *
 * Creates new Flutter engine.
 *
 * Returns: a new #FlEngine.
 */
FlEngine* fl_engine_new(FlDartProject* project);

/**
 * fl_engine_new_headless:
 * @project: an #FlDartProject.
 *
 * Creates new Flutter engine running in headless mode.
 *
 * Returns: a new #FlEngine.
 */
FlEngine* fl_engine_new_headless(FlDartProject* project);

/**
 * fl_engine_get_binary_messenger:
 * @engine: an #FlEngine.
 *
 * Gets the messenger to communicate with this engine.
 *
 * Returns: an #FlBinaryMessenger.
 */
FlBinaryMessenger* fl_engine_get_binary_messenger(FlEngine* engine);

/**
 * fl_engine_get_texture_registrar:
 * @engine: an #FlEngine.
 *
 * Gets the texture registrar for registering textures.
 *
 * Returns: an #FlTextureRegistrar.
 */
FlTextureRegistrar* fl_engine_get_texture_registrar(FlEngine* engine);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_ENGINE_H_
