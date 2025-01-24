// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_keyboard_handler.h"

#include "flutter/shell/platform/embedder/test_utils/key_codes.g.h"
#include "flutter/shell/platform/embedder/test_utils/proc_table_replacement.h"
#include "flutter/shell/platform/linux/fl_engine_private.h"
#include "flutter/shell/platform/linux/fl_method_codec_private.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_standard_method_codec.h"
#include "flutter/shell/platform/linux/testing/fl_mock_binary_messenger.h"
#include "flutter/shell/platform/linux/testing/mock_keymap.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

static constexpr char kKeyboardChannelName[] = "flutter/keyboard";
static constexpr char kGetKeyboardStateMethod[] = "getKeyboardState";

using ::flutter::testing::keycodes::kLogicalKeyA;
using ::flutter::testing::keycodes::kPhysicalKeyA;

constexpr guint16 kKeyCodeKeyA = 0x26u;

TEST(FlKeyboardHandlerTest, KeyboardChannelGetPressedState) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlKeyboardManager) manager = fl_keyboard_manager_new(engine);

  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  g_autoptr(FlKeyboardHandler) handler =
      fl_keyboard_handler_new(FL_BINARY_MESSENGER(messenger), manager);
  EXPECT_NE(handler, nullptr);

  // Send key event to set pressed state.
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
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);
  g_autoptr(FlKeyEvent) event = fl_key_event_new(
      0, TRUE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  fl_keyboard_manager_handle_event(
      manager, event, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(FlKeyEvent) redispatched_event = nullptr;
        EXPECT_TRUE(fl_keyboard_manager_handle_event_finish(
            FL_KEYBOARD_MANAGER(object), result, &redispatched_event, nullptr));
        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);
  g_main_loop_run(loop);

  gboolean called = FALSE;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, kKeyboardChannelName, kGetKeyboardStateMethod, nullptr,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;

        EXPECT_TRUE(FL_IS_METHOD_SUCCESS_RESPONSE(response));

        g_autoptr(FlValue) expected_result = fl_value_new_map();
        fl_value_set_take(expected_result, fl_value_new_int(kPhysicalKeyA),
                          fl_value_new_int(kLogicalKeyA));
        EXPECT_TRUE(fl_value_equal(fl_method_success_response_get_result(
                                       FL_METHOD_SUCCESS_RESPONSE(response)),
                                   expected_result));
      },
      &called);

  EXPECT_TRUE(called);
}
