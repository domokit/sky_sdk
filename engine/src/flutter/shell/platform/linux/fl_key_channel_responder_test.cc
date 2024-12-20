// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_key_channel_responder.h"

#include "gtest/gtest.h"

#include "flutter/shell/platform/linux/fl_binary_messenger_private.h"
#include "flutter/shell/platform/linux/testing/fl_mock_binary_messenger.h"

typedef struct {
  const gchar* expected_message;
  gboolean handled;
} KeyEventData;

static FlValue* key_event_cb(FlMockBinaryMessenger* messenger,
                             FlValue* message,
                             gpointer user_data) {
  KeyEventData* data = static_cast<KeyEventData*>(user_data);

  g_autofree gchar* message_string = fl_value_to_string(message);
  EXPECT_STREQ(message_string, data->expected_message);

  FlValue* response = fl_value_new_map();
  fl_value_set_string_take(response, "handled",
                           fl_value_new_bool(data->handled));

  free(data);

  return response;
}

static void set_key_event_channel(FlMockBinaryMessenger* messenger,
                                  const gchar* expected_message,
                                  gboolean handled) {
  KeyEventData* data = g_new0(KeyEventData, 1);
  data->expected_message = expected_message;
  data->handled = handled;
  fl_mock_binary_messenger_set_json_message_channel(
      messenger, "flutter/keyevent", key_event_cb, data);
}

// Test sending a letter "A";
TEST(FlKeyChannelResponderTest, SendKeyEvent) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlKeyChannelResponder) responder =
      fl_key_channel_responder_new(FL_BINARY_MESSENGER(messenger));

  set_key_event_channel(
      messenger,
      "{type: keydown, keymap: linux, scanCode: 4, toolkit: gtk, keyCode: 65, "
      "modifiers: 0, unicodeScalarValues: 65}",
      FALSE);
  g_autoptr(FlKeyEvent) event1 = fl_key_event_new(
      12345, TRUE, 0x04, GDK_KEY_A, static_cast<GdkModifierType>(0), 0);
  fl_key_channel_responder_handle_event(
      responder, event1, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_channel_responder_handle_event_finish(
            FL_KEY_CHANNEL_RESPONDER(object), result, &handled, nullptr));
        EXPECT_FALSE(handled);
        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);
  g_main_loop_run(loop);

  set_key_event_channel(
      messenger,
      "{type: keyup, keymap: linux, scanCode: 4, toolkit: gtk, keyCode: 65, "
      "modifiers: 0, unicodeScalarValues: 65}",
      FALSE);
  g_autoptr(FlKeyEvent) event2 = fl_key_event_new(
      23456, FALSE, 0x04, GDK_KEY_A, static_cast<GdkModifierType>(0), 0);
  fl_key_channel_responder_handle_event(
      responder, event2, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_channel_responder_handle_event_finish(
            FL_KEY_CHANNEL_RESPONDER(object), result, &handled, nullptr));
        EXPECT_FALSE(handled);
        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);
  g_main_loop_run(loop);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

void test_lock_event(guint key_code,
                     const char* down_expected,
                     const char* up_expected) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlKeyChannelResponder) responder =
      fl_key_channel_responder_new(FL_BINARY_MESSENGER(messenger));

  set_key_event_channel(messenger, down_expected, FALSE);
  g_autoptr(FlKeyEvent) event1 = fl_key_event_new(
      12345, TRUE, 0x04, key_code, static_cast<GdkModifierType>(0), 0);
  fl_key_channel_responder_handle_event(
      responder, event1, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_channel_responder_handle_event_finish(
            FL_KEY_CHANNEL_RESPONDER(object), result, &handled, nullptr));
        EXPECT_FALSE(handled);
        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);
  g_main_loop_run(loop);

  set_key_event_channel(messenger, up_expected, FALSE);
  g_autoptr(FlKeyEvent) event2 = fl_key_event_new(
      12346, FALSE, 0x04, key_code, static_cast<GdkModifierType>(0), 0);
  fl_key_channel_responder_handle_event(
      responder, event2, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_channel_responder_handle_event_finish(
            FL_KEY_CHANNEL_RESPONDER(object), result, &handled, nullptr));
        EXPECT_FALSE(handled);
        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);
  g_main_loop_run(loop);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

// Test sending a "NumLock" keypress.
TEST(FlKeyChannelResponderTest, SendNumLockKeyEvent) {
  test_lock_event(GDK_KEY_Num_Lock,
                  "{type: keydown, keymap: linux, scanCode: 4, toolkit: gtk, "
                  "keyCode: 65407, modifiers: 16}",
                  "{type: keyup, keymap: linux, scanCode: 4, toolkit: gtk, "
                  "keyCode: 65407, modifiers: 0}");
}

// Test sending a "CapsLock" keypress.
TEST(FlKeyChannelResponderTest, SendCapsLockKeyEvent) {
  test_lock_event(GDK_KEY_Caps_Lock,
                  "{type: keydown, keymap: linux, scanCode: 4, toolkit: gtk, "
                  "keyCode: 65509, modifiers: 2}",
                  "{type: keyup, keymap: linux, scanCode: 4, toolkit: gtk, "
                  "keyCode: 65509, modifiers: 0}");
}

// Test sending a "ShiftLock" keypress.
TEST(FlKeyChannelResponderTest, SendShiftLockKeyEvent) {
  test_lock_event(GDK_KEY_Shift_Lock,
                  "{type: keydown, keymap: linux, scanCode: 4, toolkit: gtk, "
                  "keyCode: 65510, modifiers: 2}",
                  "{type: keyup, keymap: linux, scanCode: 4, toolkit: gtk, "
                  "keyCode: 65510, modifiers: 0}");
}

TEST(FlKeyChannelResponderTest, TestKeyEventHandledByFramework) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlKeyChannelResponder) responder =
      fl_key_channel_responder_new(FL_BINARY_MESSENGER(messenger));

  set_key_event_channel(
      messenger,
      "{type: keydown, keymap: linux, scanCode: 4, toolkit: gtk, "
      "keyCode: 65, modifiers: 0, unicodeScalarValues: 65}",
      TRUE);
  g_autoptr(FlKeyEvent) event = fl_key_event_new(
      12345, TRUE, 0x04, GDK_KEY_A, static_cast<GdkModifierType>(0), 0);
  fl_key_channel_responder_handle_event(
      responder, event, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_channel_responder_handle_event_finish(
            FL_KEY_CHANNEL_RESPONDER(object), result, &handled, nullptr));
        EXPECT_TRUE(handled);
        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);
  g_main_loop_run(loop);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlKeyChannelResponderTest, UseSpecifiedLogicalKey) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlKeyChannelResponder) responder =
      fl_key_channel_responder_new(FL_BINARY_MESSENGER(messenger));

  set_key_event_channel(
      messenger,
      "{type: keydown, keymap: linux, scanCode: 4, toolkit: gtk, "
      "keyCode: 65, modifiers: 0, unicodeScalarValues: 65, "
      "specifiedLogicalKey: 888}",
      TRUE);
  g_autoptr(FlKeyEvent) event = fl_key_event_new(
      12345, TRUE, 0x04, GDK_KEY_A, static_cast<GdkModifierType>(0), 0);
  fl_key_channel_responder_handle_event(
      responder, event, 888, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_channel_responder_handle_event_finish(
            FL_KEY_CHANNEL_RESPONDER(object), result, &handled, nullptr));
        EXPECT_TRUE(handled);
        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);
  g_main_loop_run(loop);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}
