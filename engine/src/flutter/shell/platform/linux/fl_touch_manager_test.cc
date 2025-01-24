// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_touch_manager.h"
#include "flutter/shell/platform/embedder/test_utils/proc_table_replacement.h"
#include "flutter/shell/platform/linux/fl_engine_private.h"

#include <gdk/gdkwayland.h>
#include <cstring>
#include <vector>

#include "gtest/gtest.h"

TEST(FlTouchManagerTest, TouchEvents) {
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

  g_autoptr(FlTouchManager) manager = fl_touch_manager_new(engine, 0);

  GdkDevice* touchscreen =
      GDK_DEVICE(g_object_new(gdk_wayland_device_get_type(), "input-source",
                              GDK_SOURCE_TOUCHSCREEN, nullptr));
  GdkEventTouch* event =
      reinterpret_cast<GdkEventTouch*>(gdk_event_new(GDK_TOUCH_BEGIN));
  event->time = 1;
  event->x = 4.0;
  event->y = 8.0;
  event->device = touchscreen;
  fl_touch_manager_handle_touch_event(manager, event, 1.0);
  EXPECT_EQ(pointer_events.size(), 2u);
  EXPECT_EQ(pointer_events[0].x, 4.0);
  EXPECT_EQ(pointer_events[0].y, 8.0);
  EXPECT_EQ(pointer_events[0].device_kind, kFlutterPointerDeviceKindTouch);
  EXPECT_EQ(pointer_events[0].timestamp,
            1000lu);  // Milliseconds -> Microseconds
  EXPECT_EQ(pointer_events[0].phase, kAdd);

  EXPECT_EQ(pointer_events[1].x, 4.0);
  EXPECT_EQ(pointer_events[1].y, 8.0);
  EXPECT_EQ(pointer_events[1].device_kind, kFlutterPointerDeviceKindTouch);
  EXPECT_EQ(pointer_events[1].timestamp,
            1000lu);  // Milliseconds -> Microseconds
  EXPECT_EQ(pointer_events[1].phase, kDown);

  event->type = GDK_TOUCH_UPDATE;
  fl_touch_manager_handle_touch_event(manager, event, 1.0);
  EXPECT_EQ(pointer_events.size(), 3u);
  EXPECT_EQ(pointer_events[2].x, 4.0);
  EXPECT_EQ(pointer_events[2].y, 8.0);
  EXPECT_EQ(pointer_events[2].device_kind, kFlutterPointerDeviceKindTouch);
  EXPECT_EQ(pointer_events[2].timestamp,
            1000lu);  // Milliseconds -> Microseconds
  EXPECT_EQ(pointer_events[2].phase, kMove);

  event->type = GDK_TOUCH_END;
  fl_touch_manager_handle_touch_event(manager, event, 1.0);
  EXPECT_EQ(pointer_events.size(), 5u);
  EXPECT_EQ(pointer_events[3].x, 4.0);
  EXPECT_EQ(pointer_events[3].y, 8.0);
  EXPECT_EQ(pointer_events[3].device_kind, kFlutterPointerDeviceKindTouch);
  EXPECT_EQ(pointer_events[3].timestamp,
            1000lu);  // Milliseconds -> Microseconds
  EXPECT_EQ(pointer_events[3].phase, kUp);

  EXPECT_EQ(pointer_events[4].x, 4.0);
  EXPECT_EQ(pointer_events[4].y, 8.0);
  EXPECT_EQ(pointer_events[4].device_kind, kFlutterPointerDeviceKindTouch);
  EXPECT_EQ(pointer_events[4].timestamp,
            1000lu);  // Milliseconds -> Microseconds
  EXPECT_EQ(pointer_events[4].phase, kRemove);
}