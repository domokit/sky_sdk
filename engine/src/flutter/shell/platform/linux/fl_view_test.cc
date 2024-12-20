// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_view.h"
#include "flutter/shell/platform/embedder/test_utils/proc_table_replacement.h"
#include "flutter/shell/platform/linux/fl_engine_private.h"
#include "flutter/shell/platform/linux/testing/fl_test.h"
#include "flutter/shell/platform/linux/testing/fl_test_gtk_logs.h"

#include "gtest/gtest.h"

static void first_frame_cb(FlView* view, gboolean* first_frame_emitted) {
  *first_frame_emitted = TRUE;
}

TEST(FlViewTest, GetEngine) {
  flutter::testing::fl_ensure_gtk_init();
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  FlView* view = fl_view_new(project);

  // Check the engine is immediately available (i.e. before the widget is
  // realized).
  FlEngine* engine = fl_view_get_engine(view);
  EXPECT_NE(engine, nullptr);
}

TEST(FlViewTest, StateUpdateDoesNotHappenInInit) {
  flutter::testing::fl_ensure_gtk_init();
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  FlView* view = fl_view_new(project);
  // Check that creating a view doesn't try to query the window state in
  // initialization, causing a critical log to be issued.
  EXPECT_EQ(
      flutter::testing::fl_get_received_gtk_log_levels() & G_LOG_LEVEL_CRITICAL,
      (GLogLevelFlags)0x0);

  (void)view;
}

TEST(FlViewTest, FirstFrameSignal) {
  flutter::testing::fl_ensure_gtk_init();

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  FlView* view = fl_view_new(project);
  gboolean first_frame_emitted = FALSE;
  g_signal_connect(view, "first-frame", G_CALLBACK(first_frame_cb),
                   &first_frame_emitted);

  EXPECT_FALSE(first_frame_emitted);

  fl_renderable_redraw(FL_RENDERABLE(view));

  // Signal is emitted in idle, clear the main loop.
  while (g_main_context_iteration(g_main_context_default(), FALSE)) {
    // Repeat until nothing to iterate on.
  }

  // Check view has detected frame.
  EXPECT_TRUE(first_frame_emitted);
}

// Check secondary view is registered with engine.
TEST(FlViewTest, SecondaryView) {
  flutter::testing::fl_ensure_gtk_init();

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  FlView* implicit_view = fl_view_new(project);

  FlEngine* engine = fl_view_get_engine(implicit_view);

  FlutterViewId view_id = -1;
  fl_engine_get_embedder_api(engine)->AddView = MOCK_ENGINE_PROC(
      AddView, ([&view_id](auto engine, const FlutterAddViewInfo* info) {
        view_id = info->view_id;
        FlutterAddViewResult result = {
            .struct_size = sizeof(FlutterAddViewResult),
            .added = true,
            .user_data = info->user_data};
        info->add_view_callback(&result);
        return kSuccess;
      }));

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));

  FlView* secondary_view = fl_view_new_for_engine(engine);
  EXPECT_EQ(view_id, fl_view_get_id(secondary_view));
}

// Check secondary view that fails registration.
TEST(FlViewTest, SecondaryViewError) {
  flutter::testing::fl_ensure_gtk_init();

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  FlView* implicit_view = fl_view_new(project);

  FlEngine* engine = fl_view_get_engine(implicit_view);

  FlutterViewId view_id = -1;
  fl_engine_get_embedder_api(engine)->AddView = MOCK_ENGINE_PROC(
      AddView, ([&view_id](auto engine, const FlutterAddViewInfo* info) {
        view_id = info->view_id;
        return kInvalidArguments;
      }));

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));

  FlView* secondary_view = fl_view_new_for_engine(engine);
  EXPECT_EQ(view_id, fl_view_get_id(secondary_view));
}

// Check views are deregistered on destruction.
TEST(FlViewTest, ViewDestroy) {
  flutter::testing::fl_ensure_gtk_init();

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  FlView* implicit_view = fl_view_new(project);

  FlEngine* engine = fl_view_get_engine(implicit_view);

  g_autoptr(GPtrArray) removed_views = g_ptr_array_new();
  fl_engine_get_embedder_api(engine)->RemoveView = MOCK_ENGINE_PROC(
      RemoveView,
      ([removed_views](auto engine, const FlutterRemoveViewInfo* info) {
        g_ptr_array_add(removed_views, GINT_TO_POINTER(info->view_id));
        return kSuccess;
      }));

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));

  FlView* secondary_view = fl_view_new_for_engine(engine);

  int64_t implicit_view_id = fl_view_get_id(implicit_view);
  int64_t secondary_view_id = fl_view_get_id(secondary_view);

  gtk_widget_destroy(GTK_WIDGET(secondary_view));
  gtk_widget_destroy(GTK_WIDGET(implicit_view));

  EXPECT_EQ(removed_views->len, 2u);
  EXPECT_EQ(GPOINTER_TO_INT(g_ptr_array_index(removed_views, 0)),
            secondary_view_id);
  EXPECT_EQ(GPOINTER_TO_INT(g_ptr_array_index(removed_views, 1)),
            implicit_view_id);
}

// Check views deregistered with errors works.
TEST(FlViewTest, ViewDestroyError) {
  flutter::testing::fl_ensure_gtk_init();

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  FlView* implicit_view = fl_view_new(project);

  FlEngine* engine = fl_view_get_engine(implicit_view);

  fl_engine_get_embedder_api(engine)->RemoveView = MOCK_ENGINE_PROC(
      RemoveView, ([](auto engine, const FlutterRemoveViewInfo* info) {
        return kInvalidArguments;
      }));

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));

  FlView* secondary_view = fl_view_new_for_engine(engine);

  gtk_widget_destroy(GTK_WIDGET(secondary_view));
  gtk_widget_destroy(GTK_WIDGET(implicit_view));
}
