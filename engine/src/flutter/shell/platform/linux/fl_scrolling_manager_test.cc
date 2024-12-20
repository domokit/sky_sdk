// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_scrolling_manager.h"
#include "flutter/shell/platform/embedder/test_utils/proc_table_replacement.h"
#include "flutter/shell/platform/linux/fl_engine_private.h"

#include <cstring>
#include <vector>

#include "gtest/gtest.h"

// Disgusting hack but could not find any way to create a GdkDevice
struct _FakeGdkDevice {
  GObject parent_instance;
  gchar* name;
  GdkInputSource source;
};
GdkDevice* makeFakeDevice(GdkInputSource source) {
  _FakeGdkDevice* device =
      static_cast<_FakeGdkDevice*>(g_malloc0(sizeof(_FakeGdkDevice)));
  device->source = source;
  // Bully the type checker
  (reinterpret_cast<GTypeInstance*>(device))->g_class =
      static_cast<GTypeClass*>(g_malloc0(sizeof(GTypeClass)));
  (reinterpret_cast<GTypeInstance*>(device))->g_class->g_type = GDK_TYPE_DEVICE;
  return reinterpret_cast<GdkDevice*>(device);
}

TEST(FlScrollingManagerTest, DiscreteDirectional) {
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

  g_autoptr(FlScrollingManager) manager = fl_scrolling_manager_new(engine, 0);

  GdkDevice* mouse = makeFakeDevice(GDK_SOURCE_MOUSE);
  GdkEventScroll* event =
      reinterpret_cast<GdkEventScroll*>(gdk_event_new(GDK_SCROLL));
  event->time = 1;
  event->x = 4.0;
  event->y = 8.0;
  event->device = mouse;
  event->direction = GDK_SCROLL_UP;
  fl_scrolling_manager_handle_scroll_event(manager, event, 1.0);
  EXPECT_EQ(pointer_events.size(), 1u);
  EXPECT_EQ(pointer_events[0].x, 4.0);
  EXPECT_EQ(pointer_events[0].y, 8.0);
  EXPECT_EQ(pointer_events[0].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[0].timestamp,
            1000lu);  // Milliseconds -> Microseconds
  EXPECT_EQ(pointer_events[0].scroll_delta_x, 0);
  EXPECT_EQ(pointer_events[0].scroll_delta_y, 53 * -1.0);
  event->direction = GDK_SCROLL_DOWN;
  fl_scrolling_manager_handle_scroll_event(manager, event, 1.0);
  EXPECT_EQ(pointer_events.size(), 2u);
  EXPECT_EQ(pointer_events[1].x, 4.0);
  EXPECT_EQ(pointer_events[1].y, 8.0);
  EXPECT_EQ(pointer_events[1].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[1].timestamp,
            1000lu);  // Milliseconds -> Microseconds
  EXPECT_EQ(pointer_events[1].scroll_delta_x, 0);
  EXPECT_EQ(pointer_events[1].scroll_delta_y, 53 * 1.0);
  event->direction = GDK_SCROLL_LEFT;
  fl_scrolling_manager_handle_scroll_event(manager, event, 1.0);
  EXPECT_EQ(pointer_events.size(), 3u);
  EXPECT_EQ(pointer_events[2].x, 4.0);
  EXPECT_EQ(pointer_events[2].y, 8.0);
  EXPECT_EQ(pointer_events[2].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[2].timestamp,
            1000lu);  // Milliseconds -> Microseconds
  EXPECT_EQ(pointer_events[2].scroll_delta_x, 53 * -1.0);
  EXPECT_EQ(pointer_events[2].scroll_delta_y, 0);
  event->direction = GDK_SCROLL_RIGHT;
  fl_scrolling_manager_handle_scroll_event(manager, event, 1.0);
  EXPECT_EQ(pointer_events.size(), 4u);
  EXPECT_EQ(pointer_events[3].x, 4.0);
  EXPECT_EQ(pointer_events[3].y, 8.0);
  EXPECT_EQ(pointer_events[3].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[3].timestamp,
            1000lu);  // Milliseconds -> Microseconds
  EXPECT_EQ(pointer_events[3].scroll_delta_x, 53 * 1.0);
  EXPECT_EQ(pointer_events[3].scroll_delta_y, 0);
}

TEST(FlScrollingManagerTest, DiscreteScrolling) {
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

  g_autoptr(FlScrollingManager) manager = fl_scrolling_manager_new(engine, 0);

  GdkDevice* mouse = makeFakeDevice(GDK_SOURCE_MOUSE);
  GdkEventScroll* event =
      reinterpret_cast<GdkEventScroll*>(gdk_event_new(GDK_SCROLL));
  event->time = 1;
  event->x = 4.0;
  event->y = 8.0;
  event->delta_x = 1.0;
  event->delta_y = 2.0;
  event->device = mouse;
  event->direction = GDK_SCROLL_SMOOTH;
  fl_scrolling_manager_handle_scroll_event(manager, event, 1.0);
  EXPECT_EQ(pointer_events.size(), 1u);
  EXPECT_EQ(pointer_events[0].x, 4.0);
  EXPECT_EQ(pointer_events[0].y, 8.0);
  EXPECT_EQ(pointer_events[0].device_kind, kFlutterPointerDeviceKindMouse);
  EXPECT_EQ(pointer_events[0].timestamp,
            1000lu);  // Milliseconds -> Microseconds
  EXPECT_EQ(pointer_events[0].scroll_delta_x, 53 * 1.0);
  EXPECT_EQ(pointer_events[0].scroll_delta_y, 53 * 2.0);
}

TEST(FlScrollingManagerTest, Panning) {
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

  g_autoptr(FlScrollingManager) manager = fl_scrolling_manager_new(engine, 0);

  GdkDevice* touchpad = makeFakeDevice(GDK_SOURCE_TOUCHPAD);
  GdkEventScroll* event =
      reinterpret_cast<GdkEventScroll*>(gdk_event_new(GDK_SCROLL));
  event->time = 1;
  event->x = 4.0;
  event->y = 8.0;
  event->delta_x = 1.0;
  event->delta_y = 2.0;
  event->device = touchpad;
  event->direction = GDK_SCROLL_SMOOTH;
  fl_scrolling_manager_handle_scroll_event(manager, event, 1.0);
  EXPECT_EQ(pointer_events.size(), 2u);
  EXPECT_EQ(pointer_events[0].x, 4.0);
  EXPECT_EQ(pointer_events[0].y, 8.0);
  EXPECT_EQ(pointer_events[0].timestamp,
            1000lu);  // Milliseconds -> Microseconds
  EXPECT_EQ(pointer_events[0].phase, kPanZoomStart);
  EXPECT_EQ(pointer_events[1].x, 4.0);
  EXPECT_EQ(pointer_events[1].y, 8.0);
  EXPECT_EQ(pointer_events[1].timestamp,
            1000lu);  // Milliseconds -> Microseconds
  EXPECT_EQ(pointer_events[1].phase, kPanZoomUpdate);
  EXPECT_EQ(pointer_events[1].pan_x, 53 * -1.0);  // directions get swapped
  EXPECT_EQ(pointer_events[1].pan_y, 53 * -2.0);
  EXPECT_EQ(pointer_events[1].scale, 1.0);
  EXPECT_EQ(pointer_events[1].rotation, 0.0);
  fl_scrolling_manager_handle_scroll_event(manager, event, 1.0);
  EXPECT_EQ(pointer_events.size(), 3u);
  EXPECT_EQ(pointer_events[2].x, 4.0);
  EXPECT_EQ(pointer_events[2].y, 8.0);
  EXPECT_EQ(pointer_events[2].timestamp,
            1000lu);  // Milliseconds -> Microseconds
  EXPECT_EQ(pointer_events[2].phase, kPanZoomUpdate);
  EXPECT_EQ(pointer_events[2].pan_x, 53 * -2.0);  // directions get swapped
  EXPECT_EQ(pointer_events[2].pan_y, 53 * -4.0);
  EXPECT_EQ(pointer_events[2].scale, 1.0);
  EXPECT_EQ(pointer_events[2].rotation, 0.0);
  event->is_stop = true;
  fl_scrolling_manager_handle_scroll_event(manager, event, 1.0);
  EXPECT_EQ(pointer_events.size(), 4u);
  EXPECT_EQ(pointer_events[3].x, 4.0);
  EXPECT_EQ(pointer_events[3].y, 8.0);
  EXPECT_EQ(pointer_events[3].timestamp,
            1000lu);  // Milliseconds -> Microseconds
  EXPECT_EQ(pointer_events[3].phase, kPanZoomEnd);
}

TEST(FlScrollingManagerTest, Zooming) {
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

  g_autoptr(FlScrollingManager) manager = fl_scrolling_manager_new(engine, 0);

  size_t time_start = g_get_real_time();
  fl_scrolling_manager_handle_zoom_begin(manager);
  EXPECT_EQ(pointer_events.size(), 1u);
  EXPECT_EQ(pointer_events[0].x, 0);
  EXPECT_EQ(pointer_events[0].y, 0);
  EXPECT_EQ(pointer_events[0].phase, kPanZoomStart);
  EXPECT_GE(pointer_events[0].timestamp, time_start);
  fl_scrolling_manager_handle_zoom_update(manager, 1.1);
  EXPECT_EQ(pointer_events.size(), 2u);
  EXPECT_EQ(pointer_events[1].x, 0);
  EXPECT_EQ(pointer_events[1].y, 0);
  EXPECT_EQ(pointer_events[1].phase, kPanZoomUpdate);
  EXPECT_GE(pointer_events[1].timestamp, pointer_events[0].timestamp);
  EXPECT_EQ(pointer_events[1].pan_x, 0);
  EXPECT_EQ(pointer_events[1].pan_y, 0);
  EXPECT_EQ(pointer_events[1].scale, 1.1);
  EXPECT_EQ(pointer_events[1].rotation, 0);
  fl_scrolling_manager_handle_zoom_end(manager);
  EXPECT_EQ(pointer_events.size(), 3u);
  EXPECT_EQ(pointer_events[2].x, 0);
  EXPECT_EQ(pointer_events[2].y, 0);
  EXPECT_EQ(pointer_events[2].phase, kPanZoomEnd);
  EXPECT_GE(pointer_events[2].timestamp, pointer_events[1].timestamp);
}

TEST(FlScrollingManagerTest, Rotating) {
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

  g_autoptr(FlScrollingManager) manager = fl_scrolling_manager_new(engine, 0);

  size_t time_start = g_get_real_time();
  fl_scrolling_manager_handle_rotation_begin(manager);
  EXPECT_EQ(pointer_events.size(), 1u);
  EXPECT_EQ(pointer_events[0].x, 0);
  EXPECT_EQ(pointer_events[0].y, 0);
  EXPECT_EQ(pointer_events[0].phase, kPanZoomStart);
  EXPECT_GE(pointer_events[0].timestamp, time_start);
  fl_scrolling_manager_handle_rotation_update(manager, 0.5);
  EXPECT_EQ(pointer_events.size(), 2u);
  EXPECT_EQ(pointer_events[1].x, 0);
  EXPECT_EQ(pointer_events[1].y, 0);
  EXPECT_EQ(pointer_events[1].phase, kPanZoomUpdate);
  EXPECT_GE(pointer_events[1].timestamp, pointer_events[0].timestamp);
  EXPECT_EQ(pointer_events[1].pan_x, 0);
  EXPECT_EQ(pointer_events[1].pan_y, 0);
  EXPECT_EQ(pointer_events[1].scale, 1.0);
  EXPECT_EQ(pointer_events[1].rotation, 0.5);
  fl_scrolling_manager_handle_rotation_end(manager);
  EXPECT_EQ(pointer_events.size(), 3u);
  EXPECT_EQ(pointer_events[2].x, 0);
  EXPECT_EQ(pointer_events[2].y, 0);
  EXPECT_EQ(pointer_events[2].phase, kPanZoomEnd);
  EXPECT_GE(pointer_events[2].timestamp, pointer_events[1].timestamp);
}

TEST(FlScrollingManagerTest, SynchronizedZoomingAndRotating) {
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

  g_autoptr(FlScrollingManager) manager = fl_scrolling_manager_new(engine, 0);

  size_t time_start = g_get_real_time();
  fl_scrolling_manager_handle_zoom_begin(manager);
  EXPECT_EQ(pointer_events.size(), 1u);
  EXPECT_EQ(pointer_events[0].x, 0);
  EXPECT_EQ(pointer_events[0].y, 0);
  EXPECT_EQ(pointer_events[0].phase, kPanZoomStart);
  EXPECT_GE(pointer_events[0].timestamp, time_start);
  fl_scrolling_manager_handle_zoom_update(manager, 1.1);
  EXPECT_EQ(pointer_events.size(), 2u);
  EXPECT_EQ(pointer_events[1].x, 0);
  EXPECT_EQ(pointer_events[1].y, 0);
  EXPECT_EQ(pointer_events[1].phase, kPanZoomUpdate);
  EXPECT_GE(pointer_events[1].timestamp, pointer_events[0].timestamp);
  EXPECT_EQ(pointer_events[1].pan_x, 0);
  EXPECT_EQ(pointer_events[1].pan_y, 0);
  EXPECT_EQ(pointer_events[1].scale, 1.1);
  EXPECT_EQ(pointer_events[1].rotation, 0);
  fl_scrolling_manager_handle_rotation_begin(manager);
  EXPECT_EQ(pointer_events.size(), 2u);
  fl_scrolling_manager_handle_rotation_update(manager, 0.5);
  EXPECT_EQ(pointer_events.size(), 3u);
  EXPECT_EQ(pointer_events[2].x, 0);
  EXPECT_EQ(pointer_events[2].y, 0);
  EXPECT_EQ(pointer_events[2].phase, kPanZoomUpdate);
  EXPECT_GE(pointer_events[2].timestamp, pointer_events[1].timestamp);
  EXPECT_EQ(pointer_events[2].pan_x, 0);
  EXPECT_EQ(pointer_events[2].pan_y, 0);
  EXPECT_EQ(pointer_events[2].scale, 1.1);
  EXPECT_EQ(pointer_events[2].rotation, 0.5);
  fl_scrolling_manager_handle_zoom_end(manager);
  // End event should only be sent after both zoom and rotate complete.
  EXPECT_EQ(pointer_events.size(), 3u);
  fl_scrolling_manager_handle_rotation_end(manager);
  EXPECT_EQ(pointer_events.size(), 4u);
  EXPECT_EQ(pointer_events[3].x, 0);
  EXPECT_EQ(pointer_events[3].y, 0);
  EXPECT_EQ(pointer_events[3].phase, kPanZoomEnd);
  EXPECT_GE(pointer_events[3].timestamp, pointer_events[2].timestamp);
}

// Make sure that zoom and rotate sequences which don't end at the same time
// don't cause any problems.
TEST(FlScrollingManagerTest, UnsynchronizedZoomingAndRotating) {
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

  g_autoptr(FlScrollingManager) manager = fl_scrolling_manager_new(engine, 0);

  size_t time_start = g_get_real_time();
  fl_scrolling_manager_handle_zoom_begin(manager);
  EXPECT_EQ(pointer_events.size(), 1u);
  EXPECT_EQ(pointer_events[0].x, 0);
  EXPECT_EQ(pointer_events[0].y, 0);
  EXPECT_EQ(pointer_events[0].phase, kPanZoomStart);
  EXPECT_GE(pointer_events[0].timestamp, time_start);
  fl_scrolling_manager_handle_zoom_update(manager, 1.1);
  EXPECT_EQ(pointer_events.size(), 2u);
  EXPECT_EQ(pointer_events[1].x, 0);
  EXPECT_EQ(pointer_events[1].y, 0);
  EXPECT_EQ(pointer_events[1].phase, kPanZoomUpdate);
  EXPECT_GE(pointer_events[1].timestamp, pointer_events[0].timestamp);
  EXPECT_EQ(pointer_events[1].pan_x, 0);
  EXPECT_EQ(pointer_events[1].pan_y, 0);
  EXPECT_EQ(pointer_events[1].scale, 1.1);
  EXPECT_EQ(pointer_events[1].rotation, 0);
  fl_scrolling_manager_handle_rotation_begin(manager);
  EXPECT_EQ(pointer_events.size(), 2u);
  fl_scrolling_manager_handle_rotation_update(manager, 0.5);
  EXPECT_EQ(pointer_events.size(), 3u);
  EXPECT_EQ(pointer_events[2].x, 0);
  EXPECT_EQ(pointer_events[2].y, 0);
  EXPECT_EQ(pointer_events[2].phase, kPanZoomUpdate);
  EXPECT_GE(pointer_events[2].timestamp, pointer_events[1].timestamp);
  EXPECT_EQ(pointer_events[2].pan_x, 0);
  EXPECT_EQ(pointer_events[2].pan_y, 0);
  EXPECT_EQ(pointer_events[2].scale, 1.1);
  EXPECT_EQ(pointer_events[2].rotation, 0.5);
  fl_scrolling_manager_handle_zoom_end(manager);
  EXPECT_EQ(pointer_events.size(), 3u);
  fl_scrolling_manager_handle_rotation_update(manager, 1.0);
  EXPECT_EQ(pointer_events.size(), 4u);
  EXPECT_EQ(pointer_events[3].x, 0);
  EXPECT_EQ(pointer_events[3].y, 0);
  EXPECT_EQ(pointer_events[3].phase, kPanZoomUpdate);
  EXPECT_GE(pointer_events[3].timestamp, pointer_events[2].timestamp);
  EXPECT_EQ(pointer_events[3].pan_x, 0);
  EXPECT_EQ(pointer_events[3].pan_y, 0);
  EXPECT_EQ(pointer_events[3].scale, 1.1);
  EXPECT_EQ(pointer_events[3].rotation, 1.0);
  fl_scrolling_manager_handle_rotation_end(manager);
  EXPECT_EQ(pointer_events.size(), 5u);
  EXPECT_EQ(pointer_events[4].x, 0);
  EXPECT_EQ(pointer_events[4].y, 0);
  EXPECT_EQ(pointer_events[4].phase, kPanZoomEnd);
  EXPECT_GE(pointer_events[4].timestamp, pointer_events[3].timestamp);
}
