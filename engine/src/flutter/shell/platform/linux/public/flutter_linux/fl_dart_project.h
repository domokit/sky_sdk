// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_DART_PROJECT_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_DART_PROJECT_H_

#include <glib-object.h>
#include <gmodule.h>

#if !defined(__FLUTTER_LINUX_INSIDE__) && !defined(FLUTTER_LINUX_COMPILATION)
#error "Only <flutter_linux/flutter_linux.h> can be included directly."
#endif

G_BEGIN_DECLS

G_MODULE_EXPORT
G_DECLARE_FINAL_TYPE(FlDartProject, fl_dart_project, FL, DART_PROJECT, GObject)

/**
 * FlDartProject:
 *
 * #FlDartProject represents a Dart project. It is used to provide information
 * about the application when creating an #FlView.
 */

/**
 * fl_dart_project_new:
 *
 * Creates a Flutter project for the currently running executable. The following
 * data files are required relative to the location of the executable:
 * - data/flutter_assets/ (as built by the Flutter tool).
 * - data/icudtl.dat (provided as a resource by the Flutter tool).
 * - lib/libapp.so (as built by the Flutter tool when in AOT mode).
 *
 * Returns: a new #FlDartProject.
 */
FlDartProject* fl_dart_project_new();

/**
 * fl_dart_project_set_aot_library_path:
 * @project: an #FlDartProject.
 * @path: the absolute path to the AOT library in the Flutter application.
 *
 * Sets the path to the AOT library in the Flutter application, which is
 * the path to libapp.so. By default this is lib/libapp.so relative to the
 * executable directory.
 */
void fl_dart_project_set_aot_library_path(FlDartProject* project,
                                          const gchar* path);

/**
 * fl_dart_project_get_aot_library_path:
 * @project: an #FlDartProject.
 *
 * Gets the path to the AOT library in the Flutter application.
 *
 * Returns: (type filename): an absolute file path, e.g.
 * "/projects/my_dart_project/lib/libapp.so".
 */
const gchar* fl_dart_project_get_aot_library_path(FlDartProject* project);

/**
 * fl_dart_project_set_assets_path:
 * @project: an #FlDartProject.
 * @path: the absolute path to the assets directory.
 *
 * Sets the path to the directory containing the assets used in the Flutter
 * application. By default, this is the data/flutter_assets subdirectory
 * relative to the executable directory.
 */
void fl_dart_project_set_assets_path(FlDartProject* project, gchar* path);

/**
 * fl_dart_project_get_assets_path:
 * @project: an #FlDartProject.
 *
 * Gets the path to the directory containing the assets used in the Flutter
 * application.
 *
 * Returns: (type filename): an absolute directory path, e.g.
 * "/projects/my_dart_project/data/flutter_assets".
 */
const gchar* fl_dart_project_get_assets_path(FlDartProject* project);

/**
 * fl_dart_project_set_icu_data_path:
 * @project: an #FlDartProject.
 * @path: the absolute path to the ICU data file.
 *
 * Sets the path to the ICU data file used in the Flutter application. By
 * default, this is data/icudtl.dat relative to the executable directory.
 */
void fl_dart_project_set_icu_data_path(FlDartProject* project, gchar* path);

/**
 * fl_dart_project_get_icu_data_path:
 * @project: an #FlDartProject.
 *
 * Gets the path to the ICU data file in the Flutter application.
 *
 * Returns: (type filename): an absolute file path, e.g.
 * "/projects/my_dart_project/data/icudtl.dat".
 */
const gchar* fl_dart_project_get_icu_data_path(FlDartProject* project);

/**
 * fl_dart_project_set_dart_entrypoint_arguments:
 * @project: an #FlDartProject.
 * @argv: a pointer to a NULL-terminated array of C strings containing the
 * command line arguments.
 *
 * Sets the command line arguments to be passed through to the Dart
 * entrypoint function.
 */
void fl_dart_project_set_dart_entrypoint_arguments(FlDartProject* project,
                                                   char** argv);

/**
 * fl_dart_project_get_dart_entrypoint_arguments:
 * @project: an #FlDartProject.
 *
 * Gets the command line arguments to be passed through to the Dart entrypoint
 * function.
 *
 * Returns: a NULL-terminated array of strings containing the command line
 * arguments to be passed to the Dart entrypoint.
 */
gchar** fl_dart_project_get_dart_entrypoint_arguments(FlDartProject* project);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_DART_PROJECT_H_
