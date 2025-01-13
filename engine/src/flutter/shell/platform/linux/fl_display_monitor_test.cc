// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_display_monitor.h"
#include "flutter/shell/platform/embedder/test_utils/proc_table_replacement.h"
#include "flutter/shell/platform/linux/fl_engine_private.h"

#include "gtest/gtest.h"

TEST(FlDisplayMonitorTest, Test) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  bool called = false;
  fl_engine_get_embedder_api(engine)->NotifyDisplayUpdate = MOCK_ENGINE_PROC(
      NotifyDisplayUpdate,
      ([&called](auto engine, FlutterEngineDisplaysUpdateType update_type,
                 const FlutterEngineDisplay* displays, size_t displays_length) {
        called = true;

        EXPECT_EQ(displays_length, 1u);

        return kSuccess;
      }));

  g_autoptr(FlDisplayMonitor) monitor =
      fl_display_monitor_new(engine, gdk_display_get_default());
  EXPECT_FALSE(called);
  fl_display_monitor_start(monitor);
  EXPECT_TRUE(called);
}
