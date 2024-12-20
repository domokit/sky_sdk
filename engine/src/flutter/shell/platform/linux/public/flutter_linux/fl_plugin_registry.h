// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_PLUGIN_REGISTRY_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_PLUGIN_REGISTRY_H_

#if !defined(__FLUTTER_LINUX_INSIDE__) && !defined(FLUTTER_LINUX_COMPILATION)
#error "Only <flutter_linux/flutter_linux.h> can be included directly."
#endif

#include <glib-object.h>
#include <gmodule.h>

#include "fl_plugin_registrar.h"

G_BEGIN_DECLS

G_MODULE_EXPORT
G_DECLARE_INTERFACE(FlPluginRegistry,
                    fl_plugin_registry,
                    FL,
                    PLUGIN_REGISTRY,
                    GObject)

/**
 * FlPluginRegistry:
 *
 * #FlPluginRegistry vends #FlPluginRegistrar objects for named plugins.
 */

struct _FlPluginRegistryInterface {
  GTypeInterface g_iface;

  /**
   * FlPluginRegistry::get_registrar_for_plugin:
   * @registry: an #FlPluginRegistry.
   * @name: plugin name.
   *
   * Gets the plugin registrar for the plugin with @name.
   *
   * Returns: (transfer full): an #FlPluginRegistrar.
   */
  FlPluginRegistrar* (*get_registrar_for_plugin)(FlPluginRegistry* registry,
                                                 const gchar* name);
};

/**
 * fl_plugin_registry_get_registrar_for_plugin:
 * @registry: an #FlPluginRegistry.
 * @name: plugin name.
 *
 * Gets the plugin registrar for the plugin with @name.
 *
 * Returns: (transfer full): an #FlPluginRegistrar.
 */
FlPluginRegistrar* fl_plugin_registry_get_registrar_for_plugin(
    FlPluginRegistry* registry,
    const gchar* name);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_PLUGIN_REGISTRY_H_
