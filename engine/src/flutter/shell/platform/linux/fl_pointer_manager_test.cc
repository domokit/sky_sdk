// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_pointer_manager.h"
#include "flutter/shell/platform/embedder/test_utils/proc_table_replacement.h"
#include "flutter/shell/platform/linux/fl_engine_private.h"

#include "gtest/gtest.h"

TEST(FlPointerManagerTest, EnterLeave) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  std::vector<FlutterPointerEvent> pointer_events;
  fl_engine_get_embedder_api(engine)->SendPointerEvent = MOCK_ENGINE_PROC(
      SendPointerEvent,
      ([&pointer_events](auto engine, const FlutterPointerEvent* events,
                         size_t events_count) {
        for (size_t i = 0; i < events_count; i++) {
          pointer_events.push_back(events[i]);
        }

        return kSuccess;
      }));

  g_autoptr(FlPointerManager) manager = fl_pointer_manager_new(42, engine);
  fl_pointer_manager_handle_enter(manager, 1234, kFlutterPointerDeviceKindMouse,
                                  1.0, 2.0);
  fl_pointer_manager_handle_leave(manager, 1235, kFlutterPointerDeviceKindMouse,
                                  3.0, 4.0);

  EXPECT_EQ(pointer_events.size(), 2u);

  EXPECT_EQ(pointer_events[0].timestamp, 1234000u);
  EXPECT_EQ(pointer_events[0].x, 1.0);
  EXPECT_EQ(pointer_events[0].y, 2.0);
  EXPECT_EQ(pointer_events[0].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[0].buttons, 0);
  EXPECT_EQ(pointer_events[0].view_id, 42);

  EXPECT_EQ(pointer_events[1].timestamp, 1235000u);
  EXPECT_EQ(pointer_events[1].x, 3.0);
  EXPECT_EQ(pointer_events[1].y, 4.0);
  EXPECT_EQ(pointer_events[1].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[1].buttons, 0);
  EXPECT_EQ(pointer_events[1].view_id, 42);
}

TEST(FlPointerManagerTest, EnterEnter) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  std::vector<FlutterPointerEvent> pointer_events;
  fl_engine_get_embedder_api(engine)->SendPointerEvent = MOCK_ENGINE_PROC(
      SendPointerEvent,
      ([&pointer_events](auto engine, const FlutterPointerEvent* events,
                         size_t events_count) {
        for (size_t i = 0; i < events_count; i++) {
          pointer_events.push_back(events[i]);
        }

        return kSuccess;
      }));

  g_autoptr(FlPointerManager) manager = fl_pointer_manager_new(42, engine);
  fl_pointer_manager_handle_enter(manager, 1234, kFlutterPointerDeviceKindMouse,
                                  1.0, 2.0);
  // Duplicate enter is ignored
  fl_pointer_manager_handle_enter(manager, 1235, kFlutterPointerDeviceKindMouse,
                                  3.0, 4.0);

  EXPECT_EQ(pointer_events.size(), 1u);

  EXPECT_EQ(pointer_events[0].timestamp, 1234000u);
  EXPECT_EQ(pointer_events[0].x, 1.0);
  EXPECT_EQ(pointer_events[0].y, 2.0);
  EXPECT_EQ(pointer_events[0].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[0].buttons, 0);
  EXPECT_EQ(pointer_events[0].view_id, 42);
}

TEST(FlPointerManagerTest, EnterLeaveLeave) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  std::vector<FlutterPointerEvent> pointer_events;
  fl_engine_get_embedder_api(engine)->SendPointerEvent = MOCK_ENGINE_PROC(
      SendPointerEvent,
      ([&pointer_events](auto engine, const FlutterPointerEvent* events,
                         size_t events_count) {
        for (size_t i = 0; i < events_count; i++) {
          pointer_events.push_back(events[i]);
        }

        return kSuccess;
      }));

  g_autoptr(FlPointerManager) manager = fl_pointer_manager_new(42, engine);
  fl_pointer_manager_handle_enter(manager, 1234, kFlutterPointerDeviceKindMouse,
                                  1.0, 2.0);
  fl_pointer_manager_handle_leave(manager, 1235, kFlutterPointerDeviceKindMouse,
                                  3.0, 4.0);
  // Duplicate leave is ignored
  fl_pointer_manager_handle_leave(manager, 1235, kFlutterPointerDeviceKindMouse,
                                  5.0, 6.0);

  EXPECT_EQ(pointer_events.size(), 2u);

  EXPECT_EQ(pointer_events[0].timestamp, 1234000u);
  EXPECT_EQ(pointer_events[0].x, 1.0);
  EXPECT_EQ(pointer_events[0].y, 2.0);
  EXPECT_EQ(pointer_events[0].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[0].buttons, 0);
  EXPECT_EQ(pointer_events[0].view_id, 42);

  EXPECT_EQ(pointer_events[1].timestamp, 1235000u);
  EXPECT_EQ(pointer_events[1].x, 3.0);
  EXPECT_EQ(pointer_events[1].y, 4.0);
  EXPECT_EQ(pointer_events[1].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[1].buttons, 0);
  EXPECT_EQ(pointer_events[1].view_id, 42);
}

TEST(FlPointerManagerTest, EnterButtonPress) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  std::vector<FlutterPointerEvent> pointer_events;
  fl_engine_get_embedder_api(engine)->SendPointerEvent = MOCK_ENGINE_PROC(
      SendPointerEvent,
      ([&pointer_events](auto engine, const FlutterPointerEvent* events,
                         size_t events_count) {
        for (size_t i = 0; i < events_count; i++) {
          pointer_events.push_back(events[i]);
        }

        return kSuccess;
      }));

  g_autoptr(FlPointerManager) manager = fl_pointer_manager_new(42, engine);
  fl_pointer_manager_handle_enter(manager, 1234, kFlutterPointerDeviceKindMouse,
                                  1.0, 2.0);
  fl_pointer_manager_handle_button_press(
      manager, 1235, kFlutterPointerDeviceKindMouse, 4.0, 8.0,
      kFlutterPointerButtonMousePrimary);

  EXPECT_EQ(pointer_events.size(), 2u);

  EXPECT_EQ(pointer_events[0].timestamp, 1234000u);
  EXPECT_EQ(pointer_events[0].x, 1.0);
  EXPECT_EQ(pointer_events[0].y, 2.0);
  EXPECT_EQ(pointer_events[0].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[0].buttons, 0);
  EXPECT_EQ(pointer_events[0].view_id, 42);

  EXPECT_EQ(pointer_events[1].timestamp, 1235000u);
  EXPECT_EQ(pointer_events[1].x, 4.0);
  EXPECT_EQ(pointer_events[1].y, 8.0);
  EXPECT_EQ(pointer_events[1].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[1].buttons, kFlutterPointerButtonMousePrimary);
  EXPECT_EQ(pointer_events[1].view_id, 42);
}

TEST(FlPointerManagerTest, NoEnterButtonPress) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  std::vector<FlutterPointerEvent> pointer_events;
  fl_engine_get_embedder_api(engine)->SendPointerEvent = MOCK_ENGINE_PROC(
      SendPointerEvent,
      ([&pointer_events](auto engine, const FlutterPointerEvent* events,
                         size_t events_count) {
        for (size_t i = 0; i < events_count; i++) {
          pointer_events.push_back(events[i]);
        }

        return kSuccess;
      }));

  g_autoptr(FlPointerManager) manager = fl_pointer_manager_new(42, engine);
  fl_pointer_manager_handle_button_press(
      manager, 1234, kFlutterPointerDeviceKindMouse, 4.0, 8.0,
      kFlutterPointerButtonMousePrimary);

  EXPECT_EQ(pointer_events.size(), 2u);

  // Synthetic enter events
  EXPECT_EQ(pointer_events[0].timestamp, 1234000u);
  EXPECT_EQ(pointer_events[0].x, 4.0);
  EXPECT_EQ(pointer_events[0].y, 8.0);
  EXPECT_EQ(pointer_events[0].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[0].buttons, 0);
  EXPECT_EQ(pointer_events[0].view_id, 42);

  EXPECT_EQ(pointer_events[1].timestamp, 1234000u);
  EXPECT_EQ(pointer_events[1].x, 4.0);
  EXPECT_EQ(pointer_events[1].y, 8.0);
  EXPECT_EQ(pointer_events[1].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[1].buttons, kFlutterPointerButtonMousePrimary);
  EXPECT_EQ(pointer_events[1].view_id, 42);
}

TEST(FlPointerManagerTest, ButtonPressButtonRelease) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  std::vector<FlutterPointerEvent> pointer_events;
  fl_engine_get_embedder_api(engine)->SendPointerEvent = MOCK_ENGINE_PROC(
      SendPointerEvent,
      ([&pointer_events](auto engine, const FlutterPointerEvent* events,
                         size_t events_count) {
        for (size_t i = 0; i < events_count; i++) {
          pointer_events.push_back(events[i]);
        }

        return kSuccess;
      }));

  g_autoptr(FlPointerManager) manager = fl_pointer_manager_new(42, engine);
  fl_pointer_manager_handle_button_press(
      manager, 1234, kFlutterPointerDeviceKindMouse, 4.0, 8.0,
      kFlutterPointerButtonMousePrimary);
  fl_pointer_manager_handle_button_release(
      manager, 1235, kFlutterPointerDeviceKindMouse, 5.0, 9.0,
      kFlutterPointerButtonMousePrimary);

  EXPECT_EQ(pointer_events.size(), 3u);

  // Ignore first synthetic enter event
  EXPECT_EQ(pointer_events[1].timestamp, 1234000u);
  EXPECT_EQ(pointer_events[1].x, 4.0);
  EXPECT_EQ(pointer_events[1].y, 8.0);
  EXPECT_EQ(pointer_events[1].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[1].buttons, kFlutterPointerButtonMousePrimary);
  EXPECT_EQ(pointer_events[1].view_id, 42);
  EXPECT_EQ(pointer_events[2].timestamp, 1235000u);
  EXPECT_EQ(pointer_events[2].x, 5.0);
  EXPECT_EQ(pointer_events[2].y, 9.0);
  EXPECT_EQ(pointer_events[2].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[2].buttons, 0);
  EXPECT_EQ(pointer_events[2].view_id, 42);
}

TEST(FlPointerManagerTest, ButtonPressButtonReleaseThreeButtons) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  std::vector<FlutterPointerEvent> pointer_events;
  fl_engine_get_embedder_api(engine)->SendPointerEvent = MOCK_ENGINE_PROC(
      SendPointerEvent,
      ([&pointer_events](auto engine, const FlutterPointerEvent* events,
                         size_t events_count) {
        for (size_t i = 0; i < events_count; i++) {
          pointer_events.push_back(events[i]);
        }

        return kSuccess;
      }));

  g_autoptr(FlPointerManager) manager = fl_pointer_manager_new(42, engine);
  // Press buttons 1-2-3, release 3-2-1
  fl_pointer_manager_handle_button_press(
      manager, 1234, kFlutterPointerDeviceKindMouse, 1.0, 2.0,
      kFlutterPointerButtonMousePrimary);
  fl_pointer_manager_handle_button_press(
      manager, 1235, kFlutterPointerDeviceKindMouse, 3.0, 4.0,
      kFlutterPointerButtonMouseSecondary);
  fl_pointer_manager_handle_button_press(manager, 1236,
                                         kFlutterPointerDeviceKindMouse, 5.0,
                                         6.0, kFlutterPointerButtonMouseMiddle);
  fl_pointer_manager_handle_button_release(
      manager, 1237, kFlutterPointerDeviceKindMouse, 7.0, 8.0,
      kFlutterPointerButtonMouseMiddle);
  fl_pointer_manager_handle_button_release(
      manager, 1238, kFlutterPointerDeviceKindMouse, 9.0, 10.0,
      kFlutterPointerButtonMouseSecondary);
  fl_pointer_manager_handle_button_release(
      manager, 1239, kFlutterPointerDeviceKindMouse, 11.0, 12.0,
      kFlutterPointerButtonMousePrimary);

  EXPECT_EQ(pointer_events.size(), 7u);

  // Ignore first synthetic enter event
  EXPECT_EQ(pointer_events[1].timestamp, 1234000u);
  EXPECT_EQ(pointer_events[1].x, 1.0);
  EXPECT_EQ(pointer_events[1].y, 2.0);
  EXPECT_EQ(pointer_events[1].buttons, kFlutterPointerButtonMousePrimary);
  EXPECT_EQ(pointer_events[2].timestamp, 1235000u);
  EXPECT_EQ(pointer_events[2].x, 3.0);
  EXPECT_EQ(pointer_events[2].y, 4.0);
  EXPECT_EQ(pointer_events[2].buttons, kFlutterPointerButtonMousePrimary |
                                           kFlutterPointerButtonMouseSecondary);
  EXPECT_EQ(pointer_events[3].timestamp, 1236000u);
  EXPECT_EQ(pointer_events[3].x, 5.0);
  EXPECT_EQ(pointer_events[3].y, 6.0);
  EXPECT_EQ(pointer_events[3].buttons, kFlutterPointerButtonMousePrimary |
                                           kFlutterPointerButtonMouseSecondary |
                                           kFlutterPointerButtonMouseMiddle);
  EXPECT_EQ(pointer_events[4].timestamp, 1237000u);
  EXPECT_EQ(pointer_events[4].x, 7.0);
  EXPECT_EQ(pointer_events[4].y, 8.0);
  EXPECT_EQ(pointer_events[4].buttons, kFlutterPointerButtonMousePrimary |
                                           kFlutterPointerButtonMouseSecondary);
  EXPECT_EQ(pointer_events[5].timestamp, 1238000u);
  EXPECT_EQ(pointer_events[5].x, 9.0);
  EXPECT_EQ(pointer_events[5].y, 10.0);
  EXPECT_EQ(pointer_events[5].buttons, kFlutterPointerButtonMousePrimary);
  EXPECT_EQ(pointer_events[6].timestamp, 1239000u);
  EXPECT_EQ(pointer_events[6].x, 11.0);
  EXPECT_EQ(pointer_events[6].y, 12.0);
  EXPECT_EQ(pointer_events[6].buttons, 0);
}

TEST(FlPointerManagerTest, ButtonPressButtonPressButtonRelease) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  std::vector<FlutterPointerEvent> pointer_events;
  fl_engine_get_embedder_api(engine)->SendPointerEvent = MOCK_ENGINE_PROC(
      SendPointerEvent,
      ([&pointer_events](auto engine, const FlutterPointerEvent* events,
                         size_t events_count) {
        for (size_t i = 0; i < events_count; i++) {
          pointer_events.push_back(events[i]);
        }

        return kSuccess;
      }));

  g_autoptr(FlPointerManager) manager = fl_pointer_manager_new(42, engine);
  fl_pointer_manager_handle_button_press(
      manager, 1234, kFlutterPointerDeviceKindMouse, 4.0, 8.0,
      kFlutterPointerButtonMousePrimary);
  // Ignore duplicate press
  fl_pointer_manager_handle_button_press(
      manager, 1234, kFlutterPointerDeviceKindMouse, 6.0, 10.0,
      kFlutterPointerButtonMousePrimary);
  fl_pointer_manager_handle_button_release(
      manager, 1235, kFlutterPointerDeviceKindMouse, 5.0, 9.0,
      kFlutterPointerButtonMousePrimary);

  EXPECT_EQ(pointer_events.size(), 3u);

  // Ignore first synthetic enter event
  EXPECT_EQ(pointer_events[1].timestamp, 1234000u);
  EXPECT_EQ(pointer_events[1].x, 4.0);
  EXPECT_EQ(pointer_events[1].y, 8.0);
  EXPECT_EQ(pointer_events[1].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[1].buttons, kFlutterPointerButtonMousePrimary);
  EXPECT_EQ(pointer_events[1].view_id, 42);
  EXPECT_EQ(pointer_events[2].timestamp, 1235000u);
  EXPECT_EQ(pointer_events[2].x, 5.0);
  EXPECT_EQ(pointer_events[2].y, 9.0);
  EXPECT_EQ(pointer_events[2].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[2].buttons, 0);
  EXPECT_EQ(pointer_events[2].view_id, 42);
}

TEST(FlPointerManagerTest, ButtonPressButtonReleaseButtonRelease) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  std::vector<FlutterPointerEvent> pointer_events;
  fl_engine_get_embedder_api(engine)->SendPointerEvent = MOCK_ENGINE_PROC(
      SendPointerEvent,
      ([&pointer_events](auto engine, const FlutterPointerEvent* events,
                         size_t events_count) {
        for (size_t i = 0; i < events_count; i++) {
          pointer_events.push_back(events[i]);
        }

        return kSuccess;
      }));

  g_autoptr(FlPointerManager) manager = fl_pointer_manager_new(42, engine);
  fl_pointer_manager_handle_button_press(
      manager, 1234, kFlutterPointerDeviceKindMouse, 4.0, 8.0,
      kFlutterPointerButtonMousePrimary);
  fl_pointer_manager_handle_button_release(
      manager, 1235, kFlutterPointerDeviceKindMouse, 5.0, 9.0,
      kFlutterPointerButtonMousePrimary);
  // Ignore duplicate release
  fl_pointer_manager_handle_button_release(
      manager, 1235, kFlutterPointerDeviceKindMouse, 6.0, 10.0,
      kFlutterPointerButtonMousePrimary);

  EXPECT_EQ(pointer_events.size(), 3u);

  // Ignore first synthetic enter event
  EXPECT_EQ(pointer_events[1].timestamp, 1234000u);
  EXPECT_EQ(pointer_events[1].x, 4.0);
  EXPECT_EQ(pointer_events[1].y, 8.0);
  EXPECT_EQ(pointer_events[1].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[1].buttons, kFlutterPointerButtonMousePrimary);
  EXPECT_EQ(pointer_events[1].view_id, 42);
  EXPECT_EQ(pointer_events[2].timestamp, 1235000u);
  EXPECT_EQ(pointer_events[2].x, 5.0);
  EXPECT_EQ(pointer_events[2].y, 9.0);
  EXPECT_EQ(pointer_events[2].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[2].buttons, 0);
  EXPECT_EQ(pointer_events[2].view_id, 42);
}

TEST(FlPointerManagerTest, NoButtonPressButtonRelease) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  std::vector<FlutterPointerEvent> pointer_events;
  fl_engine_get_embedder_api(engine)->SendPointerEvent = MOCK_ENGINE_PROC(
      SendPointerEvent,
      ([&pointer_events](auto engine, const FlutterPointerEvent* events,
                         size_t events_count) {
        for (size_t i = 0; i < events_count; i++) {
          pointer_events.push_back(events[i]);
        }

        return kSuccess;
      }));

  g_autoptr(FlPointerManager) manager = fl_pointer_manager_new(42, engine);
  // Release without associated press, will be ignored
  fl_pointer_manager_handle_button_release(
      manager, 1235, kFlutterPointerDeviceKindMouse, 5.0, 9.0,
      kFlutterPointerButtonMousePrimary);

  EXPECT_EQ(pointer_events.size(), 0u);
}

TEST(FlPointerManagerTest, Motion) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  std::vector<FlutterPointerEvent> pointer_events;
  fl_engine_get_embedder_api(engine)->SendPointerEvent = MOCK_ENGINE_PROC(
      SendPointerEvent,
      ([&pointer_events](auto engine, const FlutterPointerEvent* events,
                         size_t events_count) {
        for (size_t i = 0; i < events_count; i++) {
          pointer_events.push_back(events[i]);
        }

        return kSuccess;
      }));

  g_autoptr(FlPointerManager) manager = fl_pointer_manager_new(42, engine);
  fl_pointer_manager_handle_motion(manager, 1234,
                                   kFlutterPointerDeviceKindMouse, 1.0, 2.0);
  fl_pointer_manager_handle_motion(manager, 1235,
                                   kFlutterPointerDeviceKindMouse, 3.0, 4.0);
  fl_pointer_manager_handle_motion(manager, 1236,
                                   kFlutterPointerDeviceKindMouse, 5.0, 6.0);

  EXPECT_EQ(pointer_events.size(), 4u);

  // Ignore first synthetic enter event
  EXPECT_EQ(pointer_events[1].timestamp, 1234000u);
  EXPECT_EQ(pointer_events[1].x, 1.0);
  EXPECT_EQ(pointer_events[1].y, 2.0);
  EXPECT_EQ(pointer_events[1].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[1].buttons, 0);
  EXPECT_EQ(pointer_events[1].view_id, 42);
  EXPECT_EQ(pointer_events[2].timestamp, 1235000u);
  EXPECT_EQ(pointer_events[2].x, 3.0);
  EXPECT_EQ(pointer_events[2].y, 4.0);
  EXPECT_EQ(pointer_events[2].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[2].buttons, 0);
  EXPECT_EQ(pointer_events[2].view_id, 42);
  EXPECT_EQ(pointer_events[3].timestamp, 1236000u);
  EXPECT_EQ(pointer_events[3].x, 5.0);
  EXPECT_EQ(pointer_events[3].y, 6.0);
  EXPECT_EQ(pointer_events[3].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[3].buttons, 0);
  EXPECT_EQ(pointer_events[3].view_id, 42);
}

TEST(FlPointerManagerTest, Drag) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  std::vector<FlutterPointerEvent> pointer_events;
  fl_engine_get_embedder_api(engine)->SendPointerEvent = MOCK_ENGINE_PROC(
      SendPointerEvent,
      ([&pointer_events](auto engine, const FlutterPointerEvent* events,
                         size_t events_count) {
        for (size_t i = 0; i < events_count; i++) {
          pointer_events.push_back(events[i]);
        }

        return kSuccess;
      }));

  g_autoptr(FlPointerManager) manager = fl_pointer_manager_new(42, engine);
  fl_pointer_manager_handle_motion(manager, 1234,
                                   kFlutterPointerDeviceKindMouse, 1.0, 2.0);
  fl_pointer_manager_handle_button_press(
      manager, 1235, kFlutterPointerDeviceKindMouse, 3.0, 4.0,
      kFlutterPointerButtonMousePrimary);
  fl_pointer_manager_handle_motion(manager, 1236,
                                   kFlutterPointerDeviceKindMouse, 5.0, 6.0);
  fl_pointer_manager_handle_button_release(
      manager, 1237, kFlutterPointerDeviceKindMouse, 7.0, 8.0,
      kFlutterPointerButtonMousePrimary);
  fl_pointer_manager_handle_motion(manager, 1238,
                                   kFlutterPointerDeviceKindMouse, 9.0, 10.0);

  EXPECT_EQ(pointer_events.size(), 6u);

  // Ignore first synthetic enter event
  EXPECT_EQ(pointer_events[1].timestamp, 1234000u);
  EXPECT_EQ(pointer_events[1].x, 1.0);
  EXPECT_EQ(pointer_events[1].y, 2.0);
  EXPECT_EQ(pointer_events[1].buttons, 0);
  EXPECT_EQ(pointer_events[1].view_id, 42);
  EXPECT_EQ(pointer_events[2].timestamp, 1235000u);
  EXPECT_EQ(pointer_events[2].x, 3.0);
  EXPECT_EQ(pointer_events[2].y, 4.0);
  EXPECT_EQ(pointer_events[2].buttons, kFlutterPointerButtonMousePrimary);
  EXPECT_EQ(pointer_events[2].view_id, 42);
  EXPECT_EQ(pointer_events[3].timestamp, 1236000u);
  EXPECT_EQ(pointer_events[3].x, 5.0);
  EXPECT_EQ(pointer_events[3].y, 6.0);
  EXPECT_EQ(pointer_events[3].buttons, kFlutterPointerButtonMousePrimary);
  EXPECT_EQ(pointer_events[3].view_id, 42);
  EXPECT_EQ(pointer_events[4].timestamp, 1237000u);
  EXPECT_EQ(pointer_events[4].x, 7.0);
  EXPECT_EQ(pointer_events[4].y, 8.0);
  EXPECT_EQ(pointer_events[4].buttons, 0);
  EXPECT_EQ(pointer_events[4].view_id, 42);
  EXPECT_EQ(pointer_events[5].timestamp, 1238000u);
  EXPECT_EQ(pointer_events[5].x, 9.0);
  EXPECT_EQ(pointer_events[5].y, 10.0);
  EXPECT_EQ(pointer_events[5].buttons, 0);
  EXPECT_EQ(pointer_events[5].view_id, 42);
}

TEST(FlPointerManagerTest, DeviceKind) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  std::vector<FlutterPointerEvent> pointer_events;
  fl_engine_get_embedder_api(engine)->SendPointerEvent = MOCK_ENGINE_PROC(
      SendPointerEvent,
      ([&pointer_events](auto engine, const FlutterPointerEvent* events,
                         size_t events_count) {
        for (size_t i = 0; i < events_count; i++) {
          pointer_events.push_back(events[i]);
        }

        return kSuccess;
      }));

  g_autoptr(FlPointerManager) manager = fl_pointer_manager_new(42, engine);
  fl_pointer_manager_handle_enter(manager, 1234,
                                  kFlutterPointerDeviceKindTrackpad, 1.0, 2.0);
  fl_pointer_manager_handle_button_press(
      manager, 1235, kFlutterPointerDeviceKindTrackpad, 1.0, 2.0,
      kFlutterPointerButtonMousePrimary);
  fl_pointer_manager_handle_motion(manager, 1238,
                                   kFlutterPointerDeviceKindTrackpad, 3.0, 4.0);
  fl_pointer_manager_handle_button_release(
      manager, 1237, kFlutterPointerDeviceKindTrackpad, 3.0, 4.0,
      kFlutterPointerButtonMousePrimary);
  fl_pointer_manager_handle_leave(manager, 1235,
                                  kFlutterPointerDeviceKindTrackpad, 3.0, 4.0);

  EXPECT_EQ(pointer_events.size(), 5u);

  EXPECT_EQ(pointer_events[0].device_kind, kFlutterPointerDeviceKindTrackpad);
  EXPECT_EQ(pointer_events[1].device_kind, kFlutterPointerDeviceKindTrackpad);
  EXPECT_EQ(pointer_events[2].device_kind, kFlutterPointerDeviceKindTrackpad);
  EXPECT_EQ(pointer_events[3].device_kind, kFlutterPointerDeviceKindTrackpad);
  EXPECT_EQ(pointer_events[4].device_kind, kFlutterPointerDeviceKindTrackpad);
}
