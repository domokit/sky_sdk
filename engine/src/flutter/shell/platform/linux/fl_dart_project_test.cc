// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_dart_project.h"

#include <gmodule.h>

#include <cstdlib>

#include "gtest/gtest.h"

TEST(FlDartProjectTest, GetPaths) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autofree gchar* exe_path = g_file_read_link("/proc/self/exe", nullptr);
  ASSERT_TRUE(exe_path != nullptr);
  g_autofree gchar* dir = g_path_get_dirname(exe_path);
  g_autofree gchar* expected_aot_library_path =
      g_build_filename(dir, "lib", "libapp.so", nullptr);
  EXPECT_STREQ(fl_dart_project_get_aot_library_path(project),
               expected_aot_library_path);
  g_autofree gchar* expected_assets_path =
      g_build_filename(dir, "data", "flutter_assets", nullptr);
  EXPECT_STREQ(fl_dart_project_get_assets_path(project), expected_assets_path);
  g_autofree gchar* expected_icu_data_path =
      g_build_filename(dir, "data", "icudtl.dat", nullptr);
  EXPECT_STREQ(fl_dart_project_get_icu_data_path(project),
               expected_icu_data_path);
}

TEST(FlDartProjectTest, OverrideAotLibraryPath) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();

  char aot_library_path[] = "/normal/tuesday/night/for/shia/labeouf";
  fl_dart_project_set_aot_library_path(project, aot_library_path);
  EXPECT_STREQ(fl_dart_project_get_aot_library_path(project), aot_library_path);
}

TEST(FlDartProjectTest, OverrideAssetsPath) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();

  char assets_path[] = "/normal/tuesday/night/for/shia/labeouf";
  fl_dart_project_set_assets_path(project, assets_path);
  EXPECT_STREQ(fl_dart_project_get_assets_path(project), assets_path);
}

TEST(FlDartProjectTest, OverrideIcuDataPath) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();

  char icu_data_path[] = "/living/in/the/woods/icudtl.dat";
  fl_dart_project_set_icu_data_path(project, icu_data_path);
  EXPECT_STREQ(fl_dart_project_get_icu_data_path(project), icu_data_path);
}

TEST(FlDartProjectTest, DartEntrypointArgs) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();

  char** retrieved_args =
      fl_dart_project_get_dart_entrypoint_arguments(project);

  EXPECT_EQ(retrieved_args, nullptr);

  GPtrArray* args_array = g_ptr_array_new();
  g_ptr_array_add(args_array, const_cast<char*>("arg_one"));
  g_ptr_array_add(args_array, const_cast<char*>("arg_two"));
  g_ptr_array_add(args_array, const_cast<char*>("arg_three"));
  g_ptr_array_add(args_array, nullptr);
  gchar** args = reinterpret_cast<gchar**>(g_ptr_array_free(args_array, false));

  fl_dart_project_set_dart_entrypoint_arguments(project, args);

  retrieved_args = fl_dart_project_get_dart_entrypoint_arguments(project);

  // FlDartProject should have done a deep copy of the args
  EXPECT_NE(retrieved_args, args);

  EXPECT_EQ(g_strv_length(retrieved_args), 3U);
}
