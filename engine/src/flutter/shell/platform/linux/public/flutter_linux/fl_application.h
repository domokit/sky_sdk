// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_APPLICATION_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_APPLICATION_H_

#if !defined(__FLUTTER_LINUX_INSIDE__) && !defined(FLUTTER_LINUX_COMPILATION)
#error "Only <flutter_linux/flutter_linux.h> can be included directly."
#endif

#include "fl_plugin_registry.h"
#include "fl_view.h"

#include <gmodule.h>
#include <gtk/gtk.h>

G_BEGIN_DECLS

G_MODULE_EXPORT
G_DECLARE_DERIVABLE_TYPE(FlApplication,
                         fl_application,
                         FL,
                         APPLICATION,
                         GtkApplication);

/**
 * FlApplicationClass:
 * @register_plugins: invoked when plugins should be registered.
 */
struct _FlApplicationClass {
  GtkApplicationClass parent_class;

  /**
   * FlApplication::register_plugins:
   * @application: the application
   * @registry: registry to use.
   *
   * The ::register_plugins signal is emitted when plugins can be registered.
   */
  void (*register_plugins)(FlApplication* application,
                           FlPluginRegistry* registry);

  /**
   * FlApplication::create_window:
   * @application: the application
   * @view: the view to add to this window.
   *
   * The ::create_window signal is emitted when a needs to be created for a
   * view. By handling this signal the application can create the appropriate
   * window for the given view and set any window properties or additional
   * widgets required.
   *
   * If this signal is not handled a standard GTK window will be created.
   */
  GtkWindow* (*create_window)(FlApplication* application, FlView* view);
};

/**
 * FlApplication:
 *
 * #Flutter-based application with the GTK embedder.
 *
 * Provides default behaviour for basic Flutter applications.
 */

/**
 * fl_application_new:
 * @application_id: (allow-none): The application ID or %NULL.
 * @flags: The application flags.
 *
 * Creates a new Flutter-based application.
 *
 * Returns: a new #FlApplication
 */
FlApplication* fl_application_new(const gchar* application_id,
                                  GApplicationFlags flags);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_APPLICATION_H_
