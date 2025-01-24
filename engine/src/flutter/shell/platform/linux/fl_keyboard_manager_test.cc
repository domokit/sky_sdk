// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_keyboard_manager.h"

#include <cstring>
#include <vector>

#include "flutter/shell/platform/embedder/test_utils/key_codes.g.h"
#include "flutter/shell/platform/embedder/test_utils/proc_table_replacement.h"
#include "flutter/shell/platform/linux/fl_engine_private.h"
#include "flutter/shell/platform/linux/key_mapping.h"
#include "flutter/shell/platform/linux/testing/fl_mock_binary_messenger.h"
#include "flutter/shell/platform/linux/testing/mock_keymap.h"

#include "gtest/gtest.h"

#define VERIFY_DOWN(OUT_LOGICAL, OUT_CHAR)                                  \
  EXPECT_EQ(static_cast<CallRecord*>(g_ptr_array_index(call_records, 0))    \
                ->event_type,                                               \
            kFlutterKeyEventTypeDown);                                      \
  EXPECT_EQ(static_cast<CallRecord*>(g_ptr_array_index(call_records, 0))    \
                ->event_logical,                                            \
            (OUT_LOGICAL));                                                 \
  EXPECT_STREQ(static_cast<CallRecord*>(g_ptr_array_index(call_records, 0)) \
                   ->event_character,                                       \
               (OUT_CHAR));                                                 \
  EXPECT_EQ(static_cast<CallRecord*>(g_ptr_array_index(call_records, 0))    \
                ->event_synthesized,                                        \
            false);                                                         \
  g_ptr_array_set_size(call_records, 0)

typedef struct {
  FlutterKeyEventType event_type;
  uint64_t event_physical;
  uint64_t event_logical;
  gchar* event_character;
  bool event_synthesized;
  FlutterKeyEventCallback callback;
  void* callback_user_data;
} CallRecord;

static CallRecord* call_record_new(const FlutterKeyEvent* event,
                                   FlutterKeyEventCallback callback,
                                   void* callback_user_data) {
  CallRecord* record = g_new0(CallRecord, 1);
  record->event_type = event->type;
  record->event_physical = event->physical;
  record->event_logical = event->logical;
  record->event_character = g_strdup(event->character);
  record->event_synthesized = event->synthesized;
  record->callback = callback;
  record->callback_user_data = callback_user_data;
  return record;
}

static void call_record_free(CallRecord* record) {
  g_free(record->event_character);
  g_free(record);
}

static void call_record_respond(CallRecord* record, bool handled) {
  if (record->callback != nullptr) {
    record->callback(handled, record->callback_user_data);
  }
}

namespace {
using ::flutter::testing::keycodes::kLogicalAltLeft;
using ::flutter::testing::keycodes::kLogicalBracketLeft;
using ::flutter::testing::keycodes::kLogicalComma;
using ::flutter::testing::keycodes::kLogicalControlLeft;
using ::flutter::testing::keycodes::kLogicalDigit1;
using ::flutter::testing::keycodes::kLogicalKeyA;
using ::flutter::testing::keycodes::kLogicalKeyB;
using ::flutter::testing::keycodes::kLogicalKeyM;
using ::flutter::testing::keycodes::kLogicalKeyQ;
using ::flutter::testing::keycodes::kLogicalMetaLeft;
using ::flutter::testing::keycodes::kLogicalMinus;
using ::flutter::testing::keycodes::kLogicalParenthesisRight;
using ::flutter::testing::keycodes::kLogicalSemicolon;
using ::flutter::testing::keycodes::kLogicalShiftLeft;
using ::flutter::testing::keycodes::kLogicalUnderscore;

using ::flutter::testing::keycodes::kPhysicalAltLeft;
using ::flutter::testing::keycodes::kPhysicalControlLeft;
using ::flutter::testing::keycodes::kPhysicalKeyA;
using ::flutter::testing::keycodes::kPhysicalKeyB;
using ::flutter::testing::keycodes::kPhysicalMetaLeft;
using ::flutter::testing::keycodes::kPhysicalShiftLeft;

constexpr guint16 kKeyCodeKeyA = 0x26u;
constexpr guint16 kKeyCodeKeyM = 0x3au;
constexpr guint16 kKeyCodeDigit1 = 0x0au;
constexpr guint16 kKeyCodeMinus = 0x14u;
constexpr guint16 kKeyCodeSemicolon = 0x2fu;
constexpr guint16 kKeyCodeKeyLeftBracket = 0x22u;

// All key clues for a keyboard layout.
//
// The index is (keyCode * 2 + hasShift), where each value is the character for
// this key (GTK only supports UTF-16.) Since the maximum keycode of interest
// is 128, it has a total of 256 entries..
typedef std::array<uint32_t, 256> MockGroupLayoutData;
typedef std::vector<const MockGroupLayoutData*> MockLayoutData;

extern const MockLayoutData kLayoutRussian;
extern const MockLayoutData kLayoutFrench;

TEST(FlKeyboardManagerTest, EngineNoResponseChannelHandled) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  // Channel handles all events.
  fl_mock_binary_messenger_set_json_message_channel(
      messenger, "flutter/keyevent",
      [](FlMockBinaryMessenger* messenger, GTask* task, FlValue* message,
         gpointer user_data) {
        g_autoptr(FlValue) return_value = fl_value_new_map();
        fl_value_set_string_take(return_value, "handled",
                                 fl_value_new_bool(TRUE));
        return fl_value_ref(return_value);
      },
      nullptr);

  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlKeyboardManager) manager = fl_keyboard_manager_new(engine);

  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  // Don't handle first event - async call never completes.
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent, ([](auto engine, const FlutterKeyEvent* event,
                        FlutterKeyEventCallback callback,
                        void* user_data) { return kSuccess; }));
  g_autoptr(FlKeyEvent) event1 = fl_key_event_new(
      0, TRUE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  gboolean first_event_completed = FALSE;
  fl_keyboard_manager_handle_event(
      manager, event1, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean* first_event_completed = static_cast<gboolean*>(user_data);
        *first_event_completed = TRUE;
      },
      &first_event_completed);

  // Handle second event.
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent, ([](auto engine, const FlutterKeyEvent* event,
                        FlutterKeyEventCallback callback, void* user_data) {
        callback(true, user_data);
        return kSuccess;
      }));
  g_autoptr(FlKeyEvent) event2 = fl_key_event_new(
      0, FALSE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);
  fl_keyboard_manager_handle_event(
      manager, event2, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(FlKeyEvent) redispatched_event = nullptr;
        EXPECT_TRUE(fl_keyboard_manager_handle_event_finish(
            FL_KEYBOARD_MANAGER(object), result, &redispatched_event, nullptr));
        EXPECT_EQ(redispatched_event, nullptr);
        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);
  g_main_loop_run(loop);

  EXPECT_FALSE(first_event_completed);

  // Passes if the cleanup does not crash.
}

TEST(FlKeyboardManagerTest, EngineHandledChannelNotHandledSync) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();

  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlKeyboardManager) manager = fl_keyboard_manager_new(engine);

  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  // Handle channel and embedder calls synchronously.
  fl_mock_binary_messenger_set_json_message_channel(
      messenger, "flutter/keyevent",
      [](FlMockBinaryMessenger* messenger, GTask* task, FlValue* message,
         gpointer user_data) {
        g_autoptr(FlValue) return_value = fl_value_new_map();
        fl_value_set_string_take(return_value, "handled",
                                 fl_value_new_bool(FALSE));
        return fl_value_ref(return_value);
      },
      nullptr);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent, ([](auto engine, const FlutterKeyEvent* event,
                        FlutterKeyEventCallback callback, void* user_data) {
        callback(true, user_data);
        return kSuccess;
      }));

  g_autoptr(FlKeyEvent) event = fl_key_event_new(
      0, TRUE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);
  fl_keyboard_manager_handle_event(
      manager, event, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(FlKeyEvent) redispatched_event = nullptr;
        EXPECT_TRUE(fl_keyboard_manager_handle_event_finish(
            FL_KEYBOARD_MANAGER(object), result, &redispatched_event, nullptr));
        EXPECT_EQ(redispatched_event, nullptr);
        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);
  g_main_loop_run(loop);
}

TEST(FlKeyboardManagerTest, EngineNotHandledChannelHandledSync) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();

  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlKeyboardManager) manager = fl_keyboard_manager_new(engine);

  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  // Handle channel and embedder calls synchronously.
  fl_mock_binary_messenger_set_json_message_channel(
      messenger, "flutter/keyevent",
      [](FlMockBinaryMessenger* messenger, GTask* task, FlValue* message,
         gpointer user_data) {
        g_autoptr(FlValue) return_value = fl_value_new_map();
        fl_value_set_string_take(return_value, "handled",
                                 fl_value_new_bool(TRUE));
        return fl_value_ref(return_value);
      },
      nullptr);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent, ([](auto engine, const FlutterKeyEvent* event,
                        FlutterKeyEventCallback callback, void* user_data) {
        callback(false, user_data);
        return kSuccess;
      }));

  g_autoptr(FlKeyEvent) event = fl_key_event_new(
      0, TRUE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);
  fl_keyboard_manager_handle_event(
      manager, event, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(FlKeyEvent) redispatched_event = nullptr;
        EXPECT_TRUE(fl_keyboard_manager_handle_event_finish(
            FL_KEYBOARD_MANAGER(object), result, &redispatched_event, nullptr));
        EXPECT_EQ(redispatched_event, nullptr);
        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);
  g_main_loop_run(loop);
}

TEST(FlKeyboardManagerTest, EngineHandledChannelHandledSync) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();

  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlKeyboardManager) manager = fl_keyboard_manager_new(engine);

  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  // Handle channel and embedder calls synchronously.
  fl_mock_binary_messenger_set_json_message_channel(
      messenger, "flutter/keyevent",
      [](FlMockBinaryMessenger* messenger, GTask* task, FlValue* message,
         gpointer user_data) {
        g_autoptr(FlValue) return_value = fl_value_new_map();
        fl_value_set_string_take(return_value, "handled",
                                 fl_value_new_bool(TRUE));
        return fl_value_ref(return_value);
      },
      nullptr);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent, ([](auto engine, const FlutterKeyEvent* event,
                        FlutterKeyEventCallback callback, void* user_data) {
        callback(true, user_data);
        return kSuccess;
      }));

  g_autoptr(FlKeyEvent) event = fl_key_event_new(
      0, TRUE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);
  fl_keyboard_manager_handle_event(
      manager, event, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(FlKeyEvent) redispatched_event = nullptr;
        EXPECT_TRUE(fl_keyboard_manager_handle_event_finish(
            FL_KEYBOARD_MANAGER(object), result, &redispatched_event, nullptr));
        EXPECT_EQ(redispatched_event, nullptr);
        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);
  g_main_loop_run(loop);
}

TEST(FlKeyboardManagerTest, EngineNotHandledChannelNotHandledSync) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();

  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlKeyboardManager) manager = fl_keyboard_manager_new(engine);

  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  // Handle channel and embedder calls synchronously.
  fl_mock_binary_messenger_set_json_message_channel(
      messenger, "flutter/keyevent",
      [](FlMockBinaryMessenger* messenger, GTask* task, FlValue* message,
         gpointer user_data) {
        g_autoptr(FlValue) return_value = fl_value_new_map();
        fl_value_set_string_take(return_value, "handled",
                                 fl_value_new_bool(FALSE));
        return fl_value_ref(return_value);
      },
      nullptr);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent, ([](auto engine, const FlutterKeyEvent* event,
                        FlutterKeyEventCallback callback, void* user_data) {
        callback(false, user_data);
        return kSuccess;
      }));

  g_autoptr(FlKeyEvent) event = fl_key_event_new(
      0, TRUE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);
  fl_keyboard_manager_handle_event(
      manager, event, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(FlKeyEvent) redispatched_event = nullptr;
        EXPECT_TRUE(fl_keyboard_manager_handle_event_finish(
            FL_KEYBOARD_MANAGER(object), result, &redispatched_event, nullptr));
        EXPECT_NE(redispatched_event, nullptr);
        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);
  g_main_loop_run(loop);
}

static void channel_respond(FlMockBinaryMessenger* messenger,
                            GTask* task,
                            gboolean handled) {
  g_autoptr(FlValue) value = fl_value_new_map();
  fl_value_set_string_take(value, "handled", fl_value_new_bool(handled));
  fl_mock_binary_messenger_json_message_channel_respond(messenger, task, value);
}

TEST(FlKeyboardManagerTest, EngineHandledChannelNotHandledAsync) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();

  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlKeyboardManager) manager = fl_keyboard_manager_new(engine);

  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  // Handle channel and embedder calls asynchronously.
  g_autoptr(GPtrArray) channel_calls =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_mock_binary_messenger_set_json_message_channel(
      messenger, "flutter/keyevent",
      [](FlMockBinaryMessenger* messenger, GTask* task, FlValue* message,
         gpointer user_data) {
        GPtrArray* channel_calls = static_cast<GPtrArray*>(user_data);
        g_ptr_array_add(channel_calls, g_object_ref(task));
        // Will respond async
        return static_cast<FlValue*>(nullptr);
      },
      channel_calls);
  g_autoptr(GPtrArray) embedder_call_records = g_ptr_array_new_with_free_func(
      reinterpret_cast<GDestroyNotify>(call_record_free));
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent, ([&embedder_call_records](
                         auto engine, const FlutterKeyEvent* event,
                         FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(embedder_call_records,
                        call_record_new(event, callback, user_data));
        return kSuccess;
      }));

  g_autoptr(FlKeyEvent) event = fl_key_event_new(
      0, TRUE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);
  fl_keyboard_manager_handle_event(
      manager, event, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(FlKeyEvent) redispatched_event = nullptr;
        EXPECT_TRUE(fl_keyboard_manager_handle_event_finish(
            FL_KEYBOARD_MANAGER(object), result, &redispatched_event, nullptr));
        EXPECT_EQ(redispatched_event, nullptr);
        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  EXPECT_EQ(embedder_call_records->len, 1u);
  EXPECT_EQ(channel_calls->len, 1u);

  call_record_respond(
      static_cast<CallRecord*>(g_ptr_array_index(embedder_call_records, 0)),
      true);
  channel_respond(messenger,
                  static_cast<GTask*>(g_ptr_array_index(channel_calls, 0)),
                  FALSE);
  g_main_loop_run(loop);
}

TEST(FlKeyboardManagerTest, EngineNotHandledChannelHandledAsync) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();

  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlKeyboardManager) manager = fl_keyboard_manager_new(engine);

  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  // Handle channel and embedder calls asynchronously.
  g_autoptr(GPtrArray) channel_calls =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_mock_binary_messenger_set_json_message_channel(
      messenger, "flutter/keyevent",
      [](FlMockBinaryMessenger* messenger, GTask* task, FlValue* message,
         gpointer user_data) {
        GPtrArray* channel_calls = static_cast<GPtrArray*>(user_data);
        g_ptr_array_add(channel_calls, g_object_ref(task));
        // Will respond async
        return static_cast<FlValue*>(nullptr);
      },
      channel_calls);
  g_autoptr(GPtrArray) embedder_call_records = g_ptr_array_new_with_free_func(
      reinterpret_cast<GDestroyNotify>(call_record_free));
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent, ([&embedder_call_records](
                         auto engine, const FlutterKeyEvent* event,
                         FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(embedder_call_records,
                        call_record_new(event, callback, user_data));
        return kSuccess;
      }));

  g_autoptr(FlKeyEvent) event = fl_key_event_new(
      0, TRUE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);
  fl_keyboard_manager_handle_event(
      manager, event, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(FlKeyEvent) redispatched_event = nullptr;
        EXPECT_TRUE(fl_keyboard_manager_handle_event_finish(
            FL_KEYBOARD_MANAGER(object), result, &redispatched_event, nullptr));
        EXPECT_EQ(redispatched_event, nullptr);
        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  EXPECT_EQ(embedder_call_records->len, 1u);
  EXPECT_EQ(channel_calls->len, 1u);

  call_record_respond(
      static_cast<CallRecord*>(g_ptr_array_index(embedder_call_records, 0)),
      false);
  channel_respond(messenger,
                  static_cast<GTask*>(g_ptr_array_index(channel_calls, 0)),
                  TRUE);
  g_main_loop_run(loop);
}

TEST(FlKeyboardManagerTest, EngineHandledChannelHandledAsync) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();

  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlKeyboardManager) manager = fl_keyboard_manager_new(engine);

  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  // Handle channel and embedder calls asynchronously.
  g_autoptr(GPtrArray) channel_calls =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_mock_binary_messenger_set_json_message_channel(
      messenger, "flutter/keyevent",
      [](FlMockBinaryMessenger* messenger, GTask* task, FlValue* message,
         gpointer user_data) {
        GPtrArray* channel_calls = static_cast<GPtrArray*>(user_data);
        g_ptr_array_add(channel_calls, g_object_ref(task));
        // Will respond async
        return static_cast<FlValue*>(nullptr);
      },
      channel_calls);
  g_autoptr(GPtrArray) embedder_call_records = g_ptr_array_new_with_free_func(
      reinterpret_cast<GDestroyNotify>(call_record_free));
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent, ([&embedder_call_records](
                         auto engine, const FlutterKeyEvent* event,
                         FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(embedder_call_records,
                        call_record_new(event, callback, user_data));
        return kSuccess;
      }));

  g_autoptr(FlKeyEvent) event = fl_key_event_new(
      0, TRUE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);
  fl_keyboard_manager_handle_event(
      manager, event, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(FlKeyEvent) redispatched_event = nullptr;
        EXPECT_TRUE(fl_keyboard_manager_handle_event_finish(
            FL_KEYBOARD_MANAGER(object), result, &redispatched_event, nullptr));
        EXPECT_EQ(redispatched_event, nullptr);
        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  EXPECT_EQ(embedder_call_records->len, 1u);
  EXPECT_EQ(channel_calls->len, 1u);

  call_record_respond(
      static_cast<CallRecord*>(g_ptr_array_index(embedder_call_records, 0)),
      true);
  channel_respond(messenger,
                  static_cast<GTask*>(g_ptr_array_index(channel_calls, 0)),
                  TRUE);
  g_main_loop_run(loop);
}

TEST(FlKeyboardManagerTest, EngineNotHandledChannelNotHandledAsync) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();

  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlKeyboardManager) manager = fl_keyboard_manager_new(engine);

  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  // Handle channel and embedder calls asynchronously.
  g_autoptr(GPtrArray) channel_calls =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_mock_binary_messenger_set_json_message_channel(
      messenger, "flutter/keyevent",
      [](FlMockBinaryMessenger* messenger, GTask* task, FlValue* message,
         gpointer user_data) {
        GPtrArray* channel_calls = static_cast<GPtrArray*>(user_data);
        g_ptr_array_add(channel_calls, g_object_ref(task));
        // Will respond async
        return static_cast<FlValue*>(nullptr);
      },
      channel_calls);
  g_autoptr(GPtrArray) embedder_call_records = g_ptr_array_new_with_free_func(
      reinterpret_cast<GDestroyNotify>(call_record_free));
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent, ([&embedder_call_records](
                         auto engine, const FlutterKeyEvent* event,
                         FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(embedder_call_records,
                        call_record_new(event, callback, user_data));
        return kSuccess;
      }));

  g_autoptr(FlKeyEvent) event = fl_key_event_new(
      0, TRUE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);
  fl_keyboard_manager_handle_event(
      manager, event, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(FlKeyEvent) redispatched_event = nullptr;
        EXPECT_TRUE(fl_keyboard_manager_handle_event_finish(
            FL_KEYBOARD_MANAGER(object), result, &redispatched_event, nullptr));
        EXPECT_NE(redispatched_event, nullptr);
        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  EXPECT_EQ(embedder_call_records->len, 1u);
  EXPECT_EQ(channel_calls->len, 1u);

  call_record_respond(
      static_cast<CallRecord*>(g_ptr_array_index(embedder_call_records, 0)),
      false);
  channel_respond(messenger,
                  static_cast<GTask*>(g_ptr_array_index(channel_calls, 0)),
                  FALSE);
  g_main_loop_run(loop);
}

TEST(FlKeyboardManagerTest, CorrectLogicalKeyForLayouts) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  g_autoptr(FlKeyboardManager) manager = fl_keyboard_manager_new(engine);

  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  g_autoptr(GPtrArray) call_records = g_ptr_array_new_with_free_func(
      reinterpret_cast<GDestroyNotify>(call_record_free));
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&call_records](auto engine, const FlutterKeyEvent* event,
                       FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(call_records,
                        call_record_new(event, callback, user_data));
        return kSuccess;
      }));

  auto sendTap = [&](guint8 keycode, guint keyval, guint8 group) {
    g_autoptr(FlKeyEvent) event1 = fl_key_event_new(
        0, TRUE, keycode, keyval, static_cast<GdkModifierType>(0), group);
    fl_keyboard_manager_handle_event(manager, event1, nullptr, nullptr,
                                     nullptr);
    g_autoptr(FlKeyEvent) event2 = fl_key_event_new(
        0, FALSE, keycode, keyval, static_cast<GdkModifierType>(0), group);
    fl_keyboard_manager_handle_event(manager, event2, nullptr, nullptr,
                                     nullptr);
  };

  /* US keyboard layout */

  sendTap(kKeyCodeKeyA, GDK_KEY_a, 0);  // KeyA
  VERIFY_DOWN(kLogicalKeyA, "a");

  sendTap(kKeyCodeKeyA, GDK_KEY_A, 0);  // Shift-KeyA
  VERIFY_DOWN(kLogicalKeyA, "A");

  sendTap(kKeyCodeDigit1, GDK_KEY_1, 0);  // Digit1
  VERIFY_DOWN(kLogicalDigit1, "1");

  sendTap(kKeyCodeDigit1, GDK_KEY_exclam, 0);  // Shift-Digit1
  VERIFY_DOWN(kLogicalDigit1, "!");

  sendTap(kKeyCodeMinus, GDK_KEY_minus, 0);  // Minus
  VERIFY_DOWN(kLogicalMinus, "-");

  sendTap(kKeyCodeMinus, GDK_KEY_underscore, 0);  // Shift-Minus
  VERIFY_DOWN(kLogicalUnderscore, "_");

  /* French keyboard layout, group 3, which is when the input method is showing
   * "Fr" */

  fl_keyboard_manager_set_lookup_key_handler(
      manager,
      [](const GdkKeymapKey* key, gpointer user_data) {
        MockLayoutData* layout_data = static_cast<MockLayoutData*>(user_data);
        guint8 group = static_cast<guint8>(key->group);
        EXPECT_LT(group, layout_data->size());
        const MockGroupLayoutData* group_layout = (*layout_data)[group];
        EXPECT_NE(group_layout, nullptr);
        EXPECT_TRUE(key->level == 0 || key->level == 1);
        bool shift = key->level == 1;
        return (*group_layout)[key->keycode * 2 + shift];
      },
      reinterpret_cast<gpointer>(const_cast<MockLayoutData*>(&kLayoutFrench)));

  sendTap(kKeyCodeKeyA, GDK_KEY_q, 3);  // KeyA
  VERIFY_DOWN(kLogicalKeyQ, "q");

  sendTap(kKeyCodeKeyA, GDK_KEY_Q, 3);  // Shift-KeyA
  VERIFY_DOWN(kLogicalKeyQ, "Q");

  sendTap(kKeyCodeSemicolon, GDK_KEY_m, 3);  // ; but prints M
  VERIFY_DOWN(kLogicalKeyM, "m");

  sendTap(kKeyCodeKeyM, GDK_KEY_comma, 3);  // M but prints ,
  VERIFY_DOWN(kLogicalComma, ",");

  sendTap(kKeyCodeDigit1, GDK_KEY_ampersand, 3);  // Digit1
  VERIFY_DOWN(kLogicalDigit1, "&");

  sendTap(kKeyCodeDigit1, GDK_KEY_1, 3);  // Shift-Digit1
  VERIFY_DOWN(kLogicalDigit1, "1");

  sendTap(kKeyCodeMinus, GDK_KEY_parenright, 3);  // Minus
  VERIFY_DOWN(kLogicalParenthesisRight, ")");

  sendTap(kKeyCodeMinus, GDK_KEY_degree, 3);  // Shift-Minus
  VERIFY_DOWN(static_cast<uint32_t>(L'°'), "°");

  /* French keyboard layout, group 0, which is pressing the "extra key for
   * triggering input method" key once after switching to French IME. */

  sendTap(kKeyCodeKeyA, GDK_KEY_a, 0);  // KeyA
  VERIFY_DOWN(kLogicalKeyA, "a");

  sendTap(kKeyCodeDigit1, GDK_KEY_1, 0);  // Digit1
  VERIFY_DOWN(kLogicalDigit1, "1");

  /* Russian keyboard layout, group 2 */
  fl_keyboard_manager_set_lookup_key_handler(
      manager,
      [](const GdkKeymapKey* key, gpointer user_data) {
        MockLayoutData* layout_data = static_cast<MockLayoutData*>(user_data);
        guint8 group = static_cast<guint8>(key->group);
        EXPECT_LT(group, layout_data->size());
        const MockGroupLayoutData* group_layout = (*layout_data)[group];
        EXPECT_NE(group_layout, nullptr);
        EXPECT_TRUE(key->level == 0 || key->level == 1);
        bool shift = key->level == 1;
        return (*group_layout)[key->keycode * 2 + shift];
      },
      reinterpret_cast<gpointer>(const_cast<MockLayoutData*>(&kLayoutRussian)));

  sendTap(kKeyCodeKeyA, GDK_KEY_Cyrillic_ef, 2);  // KeyA
  VERIFY_DOWN(kLogicalKeyA, "ф");

  sendTap(kKeyCodeDigit1, GDK_KEY_1, 2);  // Shift-Digit1
  VERIFY_DOWN(kLogicalDigit1, "1");

  sendTap(kKeyCodeKeyLeftBracket, GDK_KEY_Cyrillic_ha, 2);
  VERIFY_DOWN(kLogicalBracketLeft, "х");

  /* Russian keyboard layout, group 0 */
  sendTap(kKeyCodeKeyA, GDK_KEY_a, 0);  // KeyA
  VERIFY_DOWN(kLogicalKeyA, "a");

  sendTap(kKeyCodeKeyLeftBracket, GDK_KEY_bracketleft, 0);
  VERIFY_DOWN(kLogicalBracketLeft, "[");
}

TEST(FlKeyboardManagerTest, SynthesizeModifiersIfNeeded) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  g_autoptr(FlKeyboardManager) manager = fl_keyboard_manager_new(engine);

  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  g_autoptr(GPtrArray) call_records = g_ptr_array_new_with_free_func(
      reinterpret_cast<GDestroyNotify>(call_record_free));
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&call_records](auto engine, const FlutterKeyEvent* event,
                       FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(call_records,
                        call_record_new(event, callback, user_data));
        return kSuccess;
      }));

  auto verifyModifierIsSynthesized = [&](GdkModifierType mask,
                                         uint64_t physical, uint64_t logical) {
    // Modifier is pressed.
    guint state = mask;
    fl_keyboard_manager_sync_modifier_if_needed(manager, state, 1000);
    EXPECT_EQ(call_records->len, 1u);
    CallRecord* record =
        static_cast<CallRecord*>(g_ptr_array_index(call_records, 0));
    EXPECT_EQ(record->event_type, kFlutterKeyEventTypeDown);
    EXPECT_EQ(record->event_physical, physical);
    EXPECT_EQ(record->event_logical, logical);
    EXPECT_STREQ(record->event_character, NULL);
    EXPECT_EQ(record->event_synthesized, true);
    // Modifier is released.
    state = state ^ mask;
    fl_keyboard_manager_sync_modifier_if_needed(manager, state, 1001);
    EXPECT_EQ(call_records->len, 2u);
    record = static_cast<CallRecord*>(g_ptr_array_index(call_records, 1));
    EXPECT_EQ(record->event_type, kFlutterKeyEventTypeUp);
    EXPECT_EQ(record->event_physical, physical);
    EXPECT_EQ(record->event_logical, logical);
    EXPECT_STREQ(record->event_character, NULL);
    EXPECT_EQ(record->event_synthesized, true);
    g_ptr_array_set_size(call_records, 0);
  };

  // No modifiers pressed.
  guint state = 0;
  fl_keyboard_manager_sync_modifier_if_needed(manager, state, 1000);
  EXPECT_EQ(call_records->len, 0u);
  g_ptr_array_set_size(call_records, 0);

  // Press and release each modifier once.
  verifyModifierIsSynthesized(GDK_CONTROL_MASK, kPhysicalControlLeft,
                              kLogicalControlLeft);
  verifyModifierIsSynthesized(GDK_META_MASK, kPhysicalMetaLeft,
                              kLogicalMetaLeft);
  verifyModifierIsSynthesized(GDK_MOD1_MASK, kPhysicalAltLeft, kLogicalAltLeft);
  verifyModifierIsSynthesized(GDK_SHIFT_MASK, kPhysicalShiftLeft,
                              kLogicalShiftLeft);
}

TEST(FlKeyboardManagerTest, GetPressedState) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlKeyboardManager) manager = fl_keyboard_manager_new(engine);

  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  // Dispatch a key event.
  fl_mock_binary_messenger_set_json_message_channel(
      messenger, "flutter/keyevent",
      [](FlMockBinaryMessenger* messenger, GTask* task, FlValue* message,
         gpointer user_data) {
        FlValue* response = fl_value_new_map();
        fl_value_set_string_take(response, "handled", fl_value_new_bool(FALSE));
        return response;
      },
      nullptr);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent, ([](auto engine, const FlutterKeyEvent* event,
                        FlutterKeyEventCallback callback, void* user_data) {
        callback(false, user_data);
        return kSuccess;
      }));
  g_autoptr(FlKeyEvent) event = fl_key_event_new(
      0, TRUE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);
  fl_keyboard_manager_handle_event(
      manager, event, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(FlKeyEvent) redispatched_event = nullptr;
        EXPECT_TRUE(fl_keyboard_manager_handle_event_finish(
            FL_KEYBOARD_MANAGER(object), result, &redispatched_event, nullptr));
        EXPECT_NE(redispatched_event, nullptr);
        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);
  g_main_loop_run(loop);

  GHashTable* pressed_state = fl_keyboard_manager_get_pressed_state(manager);
  EXPECT_EQ(g_hash_table_size(pressed_state), 1u);

  gpointer physical_key =
      g_hash_table_lookup(pressed_state, uint64_to_gpointer(kPhysicalKeyA));
  EXPECT_EQ(gpointer_to_uint64(physical_key), kLogicalKeyA);
}

// The following layout data is generated using DEBUG_PRINT_LAYOUT.

const MockGroupLayoutData kLayoutRussian0{
    // +0x0  Shift   +0x1    Shift   +0x2    Shift   +0x3    Shift
    0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff,  // 0x00
    0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff,  // 0x04
    0x0000, 0xffff, 0xffff, 0x0031, 0x0031, 0x0021, 0x0032, 0x0040,  // 0x08
    0x0033, 0x0023, 0x0034, 0x0024, 0x0035, 0x0025, 0x0036, 0x005e,  // 0x0c
    0x0037, 0x0026, 0x0038, 0x002a, 0x0039, 0x0028, 0x0030, 0x0029,  // 0x10
    0x002d, 0x005f, 0x003d, 0x002b, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x14
    0x0071, 0x0051, 0x0077, 0x0057, 0x0065, 0x0045, 0x0072, 0x0052,  // 0x18
    0x0074, 0x0054, 0x0079, 0x0059, 0x0075, 0x0055, 0x0069, 0x0049,  // 0x1c
    0x006f, 0x004f, 0x0070, 0x0050, 0x005b, 0x007b, 0x005d, 0x007d,  // 0x20
    0xffff, 0xffff, 0xffff, 0x0061, 0x0061, 0x0041, 0x0073, 0x0053,  // 0x24
    0x0064, 0x0044, 0x0066, 0x0046, 0x0067, 0x0047, 0x0068, 0x0048,  // 0x28
    0x006a, 0x004a, 0x006b, 0x004b, 0x006c, 0x004c, 0x003b, 0x003a,  // 0x2c
    0x0027, 0x0022, 0x0060, 0x007e, 0xffff, 0x005c, 0x005c, 0x007c,  // 0x30
    0x007a, 0x005a, 0x0078, 0x0058, 0x0063, 0x0043, 0x0076, 0x0056,  // 0x34
    0x0062, 0x0042, 0x006e, 0x004e, 0x006d, 0x004d, 0x002c, 0x003c,  // 0x38
    0x002e, 0x003e, 0x002f, 0x003f, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x3c
    0xffff, 0xffff, 0x0020, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x40
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x44
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x48
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x4c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x50
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x54
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x58
    0xffff, 0xffff, 0x0000, 0xffff, 0x003c, 0x003e, 0xffff, 0xffff,  // 0x5c
    0xffff, 0xffff, 0x0000, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x60
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x0000, 0xffff,  // 0x64
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x68
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x6c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x70
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x74
    0x0000, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x78
    0xffff, 0xffff, 0xffff, 0x00b1, 0x00b1, 0xffff, 0xffff, 0xffff,  // 0x7c
};

const MockGroupLayoutData kLayoutRussian2{{
    // +0x0  Shift   +0x1    Shift   +0x2    Shift   +0x3    Shift
    0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff,  // 0x00
    0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff,  // 0x04
    0xffff, 0x0031, 0x0021, 0x0000, 0x0031, 0x0021, 0x0032, 0x0022,  // 0x08
    0x0033, 0x06b0, 0x0034, 0x003b, 0x0035, 0x0025, 0x0036, 0x003a,  // 0x0c
    0x0037, 0x003f, 0x0038, 0x002a, 0x0039, 0x0028, 0x0030, 0x0029,  // 0x10
    0x002d, 0x005f, 0x003d, 0x002b, 0x0071, 0x0051, 0x0000, 0x0000,  // 0x14
    0x06ca, 0x06ea, 0x06c3, 0x06e3, 0x06d5, 0x06f5, 0x06cb, 0x06eb,  // 0x18
    0x06c5, 0x06e5, 0x06ce, 0x06ee, 0x06c7, 0x06e7, 0x06db, 0x06fb,  // 0x1c
    0x06dd, 0x06fd, 0x06da, 0x06fa, 0x06c8, 0x06e8, 0x06df, 0x06ff,  // 0x20
    0x0061, 0x0041, 0x0041, 0x0000, 0x06c6, 0x06e6, 0x06d9, 0x06f9,  // 0x24
    0x06d7, 0x06f7, 0x06c1, 0x06e1, 0x06d0, 0x06f0, 0x06d2, 0x06f2,  // 0x28
    0x06cf, 0x06ef, 0x06cc, 0x06ec, 0x06c4, 0x06e4, 0x06d6, 0x06f6,  // 0x2c
    0x06dc, 0x06fc, 0x06a3, 0x06b3, 0x007c, 0x0000, 0x005c, 0x002f,  // 0x30
    0x06d1, 0x06f1, 0x06de, 0x06fe, 0x06d3, 0x06f3, 0x06cd, 0x06ed,  // 0x34
    0x06c9, 0x06e9, 0x06d4, 0x06f4, 0x06d8, 0x06f8, 0x06c2, 0x06e2,  // 0x38
    0x06c0, 0x06e0, 0x002e, 0x002c, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x3c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x40
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x44
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x48
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x4c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x50
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x54
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x58
    0xffff, 0xffff, 0x003c, 0x003e, 0x002f, 0x007c, 0xffff, 0xffff,  // 0x5c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x60
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x64
    0xffff, 0xffff, 0xffff, 0xffff, 0x0000, 0xffff, 0xffff, 0x0000,  // 0x68
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x6c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x70
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x74
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x00b1,  // 0x78
    0x00b1, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x7c
}};

const MockGroupLayoutData kLayoutFrench0 = {
    // +0x0  Shift   +0x1    Shift   +0x2    Shift   +0x3    Shift
    0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff,  // 0x00
    0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff,  // 0x04
    0x0000, 0xffff, 0xffff, 0x0031, 0x0031, 0x0021, 0x0032, 0x0040,  // 0x08
    0x0033, 0x0023, 0x0034, 0x0024, 0x0035, 0x0025, 0x0036, 0x005e,  // 0x0c
    0x0037, 0x0026, 0x0038, 0x002a, 0x0039, 0x0028, 0x0030, 0x0029,  // 0x10
    0x002d, 0x005f, 0x003d, 0x002b, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x14
    0x0071, 0x0051, 0x0077, 0x0057, 0x0065, 0x0045, 0x0072, 0x0052,  // 0x18
    0x0074, 0x0054, 0x0079, 0x0059, 0x0075, 0x0055, 0x0069, 0x0049,  // 0x1c
    0x006f, 0x004f, 0x0070, 0x0050, 0x005b, 0x007b, 0x005d, 0x007d,  // 0x20
    0xffff, 0xffff, 0xffff, 0x0061, 0x0061, 0x0041, 0x0073, 0x0053,  // 0x24
    0x0064, 0x0044, 0x0066, 0x0046, 0x0067, 0x0047, 0x0068, 0x0048,  // 0x28
    0x006a, 0x004a, 0x006b, 0x004b, 0x006c, 0x004c, 0x003b, 0x003a,  // 0x2c
    0x0027, 0x0022, 0x0060, 0x007e, 0xffff, 0x005c, 0x005c, 0x007c,  // 0x30
    0x007a, 0x005a, 0x0078, 0x0058, 0x0063, 0x0043, 0x0076, 0x0056,  // 0x34
    0x0062, 0x0042, 0x006e, 0x004e, 0x006d, 0x004d, 0x002c, 0x003c,  // 0x38
    0x002e, 0x003e, 0x002f, 0x003f, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x3c
    0xffff, 0xffff, 0x0020, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x40
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x44
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x48
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x4c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x50
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x54
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x58
    0xffff, 0xffff, 0x0000, 0xffff, 0x003c, 0x003e, 0xffff, 0xffff,  // 0x5c
    0xffff, 0xffff, 0x0000, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x60
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x0000, 0xffff,  // 0x64
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x68
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x6c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x70
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x74
    0x0000, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x78
    0xffff, 0xffff, 0xffff, 0x00b1, 0x00b1, 0xffff, 0xffff, 0xffff,  // 0x7c
};

const MockGroupLayoutData kLayoutFrench3 = {
    // +0x0  Shift   +0x1    Shift   +0x2    Shift   +0x3    Shift
    0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff,  // 0x00
    0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff,  // 0x04
    0x0000, 0xffff, 0x0000, 0x0000, 0x0026, 0x0031, 0x00e9, 0x0032,  // 0x08
    0x0022, 0x0033, 0x0027, 0x0034, 0x0028, 0x0035, 0x002d, 0x0036,  // 0x0c
    0x00e8, 0x0037, 0x005f, 0x0038, 0x00e7, 0x0039, 0x00e0, 0x0030,  // 0x10
    0x0029, 0x00b0, 0x003d, 0x002b, 0x0000, 0x0000, 0x0061, 0x0041,  // 0x14
    0x0061, 0x0041, 0x007a, 0x005a, 0x0065, 0x0045, 0x0072, 0x0052,  // 0x18
    0x0074, 0x0054, 0x0079, 0x0059, 0x0075, 0x0055, 0x0069, 0x0049,  // 0x1c
    0x006f, 0x004f, 0x0070, 0x0050, 0xffff, 0xffff, 0x0024, 0x00a3,  // 0x20
    0x0041, 0x0000, 0x0000, 0x0000, 0x0071, 0x0051, 0x0073, 0x0053,  // 0x24
    0x0064, 0x0044, 0x0066, 0x0046, 0x0067, 0x0047, 0x0068, 0x0048,  // 0x28
    0x006a, 0x004a, 0x006b, 0x004b, 0x006c, 0x004c, 0x006d, 0x004d,  // 0x2c
    0x00f9, 0x0025, 0x00b2, 0x007e, 0x0000, 0x0000, 0x002a, 0x00b5,  // 0x30
    0x0077, 0x0057, 0x0078, 0x0058, 0x0063, 0x0043, 0x0076, 0x0056,  // 0x34
    0x0062, 0x0042, 0x006e, 0x004e, 0x002c, 0x003f, 0x003b, 0x002e,  // 0x38
    0x003a, 0x002f, 0x0021, 0x00a7, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x3c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x40
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x44
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x48
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x4c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x50
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x54
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x58
    0xffff, 0x003c, 0x0000, 0xffff, 0x003c, 0x003e, 0xffff, 0xffff,  // 0x5c
    0xffff, 0xffff, 0x0000, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x60
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x0000, 0xffff,  // 0x64
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x68
    0xffff, 0x0000, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x6c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x70
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x74
    0x0000, 0xffff, 0xffff, 0xffff, 0xffff, 0x00b1, 0x00b1, 0xffff,  // 0x78
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x7c
};

const MockLayoutData kLayoutRussian{&kLayoutRussian0, nullptr,
                                    &kLayoutRussian2};
const MockLayoutData kLayoutFrench{&kLayoutFrench0, nullptr, nullptr,
                                   &kLayoutFrench3};

}  // namespace
