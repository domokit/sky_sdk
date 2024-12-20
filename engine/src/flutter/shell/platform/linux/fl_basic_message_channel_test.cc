// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Included first as it collides with the X11 headers.
#include "gtest/gtest.h"

#include "flutter/shell/platform/linux/public/flutter_linux/fl_basic_message_channel.h"

#include "flutter/shell/platform/linux/public/flutter_linux/fl_message_codec.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_standard_message_codec.h"
#include "flutter/shell/platform/linux/testing/fl_mock_binary_messenger.h"

// Checks sending a message without a response works.
// MOCK_ENGINE_PROC is leaky by design
// NOLINTBEGIN(clang-analyzer-core.StackAddressEscape)
TEST(FlBasicMessageChannelTest, SendMessageWithoutResponse) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  gboolean called = FALSE;
  fl_mock_binary_messenger_set_standard_message_channel(
      messenger, "test",
      [](FlMockBinaryMessenger* messenger, FlValue* message,
         gpointer user_data) {
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;

        EXPECT_NE(message, nullptr);
        EXPECT_EQ(fl_value_get_type(message), FL_VALUE_TYPE_STRING);
        EXPECT_STREQ(fl_value_get_string(message), "Marco!");

        // No response.
        return static_cast<FlValue*>(nullptr);
      },
      &called);

  g_autoptr(FlStandardMessageCodec) codec = fl_standard_message_codec_new();
  g_autoptr(FlBasicMessageChannel) channel = fl_basic_message_channel_new(
      FL_BINARY_MESSENGER(messenger), "test", FL_MESSAGE_CODEC(codec));
  g_autoptr(FlValue) message = fl_value_new_string("Marco!");
  fl_basic_message_channel_send(channel, message, nullptr, nullptr, nullptr);

  EXPECT_TRUE(called);
}
// NOLINTEND(clang-analyzer-core.StackAddressEscape)

// Checks sending a message with a response works.
TEST(FlBasicMessageChannelTest, SendMessageWithResponse) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  fl_mock_binary_messenger_set_standard_message_channel(
      messenger, "test",
      [](FlMockBinaryMessenger* messenger, FlValue* message,
         gpointer user_data) {
        EXPECT_NE(message, nullptr);
        EXPECT_EQ(fl_value_get_type(message), FL_VALUE_TYPE_STRING);
        EXPECT_STREQ(fl_value_get_string(message), "Marco!");

        return fl_value_new_string("Polo!");
      },
      nullptr);

  g_autoptr(FlStandardMessageCodec) codec = fl_standard_message_codec_new();
  g_autoptr(FlBasicMessageChannel) channel = fl_basic_message_channel_new(
      FL_BINARY_MESSENGER(messenger), "test", FL_MESSAGE_CODEC(codec));
  g_autoptr(FlValue) message = fl_value_new_string("Marco!");
  fl_basic_message_channel_send(
      channel, message, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(GError) error = nullptr;
        g_autoptr(FlValue) message = fl_basic_message_channel_send_finish(
            FL_BASIC_MESSAGE_CHANNEL(object), result, &error);
        EXPECT_NE(message, nullptr);
        EXPECT_EQ(error, nullptr);

        EXPECT_EQ(fl_value_get_type(message), FL_VALUE_TYPE_STRING);
        EXPECT_STREQ(fl_value_get_string(message), "Polo!");

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  g_main_loop_run(loop);
}

// Checks the engine reporting a send failure is handled.
TEST(FlBasicMessageChannelTest, SendFailure) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  fl_mock_binary_messenger_set_error_channel(messenger, "test", 42, "Error");

  g_autoptr(FlStandardMessageCodec) codec = fl_standard_message_codec_new();
  g_autoptr(FlBasicMessageChannel) channel = fl_basic_message_channel_new(
      FL_BINARY_MESSENGER(messenger), "test", FL_MESSAGE_CODEC(codec));
  g_autoptr(FlValue) message = fl_value_new_string("Hello World!");
  fl_basic_message_channel_send(
      channel, message, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(GError) error = nullptr;
        g_autoptr(FlValue) message = fl_basic_message_channel_send_finish(
            FL_BASIC_MESSAGE_CHANNEL(object), result, &error);
        EXPECT_EQ(message, nullptr);
        EXPECT_NE(error, nullptr);
        EXPECT_EQ(error->code, 42);
        EXPECT_STREQ(error->message, "Error");

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  g_main_loop_run(loop);
}

// Checks the shell able to receive and respond to messages from the engine.
TEST(FlBasicMessageChannelTest, ReceiveMessage) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();

  // Listen for messages from the engine.
  g_autoptr(FlStandardMessageCodec) codec = fl_standard_message_codec_new();
  g_autoptr(FlBasicMessageChannel) messages_channel =
      fl_basic_message_channel_new(FL_BINARY_MESSENGER(messenger), "test",
                                   FL_MESSAGE_CODEC(codec));
  fl_basic_message_channel_set_message_handler(
      messages_channel,
      [](FlBasicMessageChannel* channel, FlValue* message,
         FlBasicMessageChannelResponseHandle* response_handle,
         gpointer user_data) {
        EXPECT_NE(message, nullptr);
        EXPECT_EQ(fl_value_get_type(message), FL_VALUE_TYPE_STRING);
        EXPECT_STREQ(fl_value_get_string(message), "Marco!");

        g_autoptr(GError) error = nullptr;
        g_autoptr(FlValue) response = fl_value_new_string("Polo!");
        EXPECT_TRUE(fl_basic_message_channel_respond(channel, response_handle,
                                                     response, &error));
        EXPECT_EQ(error, nullptr);
      },
      nullptr, nullptr);

  // Trigger the engine to send a message.
  g_autoptr(FlValue) message = fl_value_new_string("Marco!");
  gboolean called = FALSE;
  fl_mock_binary_messenger_send_standard_message(
      messenger, "test", message,
      [](FlMockBinaryMessenger* messenger, FlValue* response,
         gpointer user_data) {
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;

        EXPECT_NE(response, nullptr);
        EXPECT_EQ(fl_value_get_type(response), FL_VALUE_TYPE_STRING);
        EXPECT_STREQ(fl_value_get_string(response), "Polo!");
      },
      &called);

  EXPECT_TRUE(called);
}

// Checks sending a null message with a response works.
TEST(FlBasicMessageChannelTest, SendNullMessageWithResponse) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  fl_mock_binary_messenger_set_standard_message_channel(
      messenger, "test",
      [](FlMockBinaryMessenger* messenger, FlValue* message,
         gpointer user_data) { return fl_value_new_null(); },
      nullptr);

  g_autoptr(FlStandardMessageCodec) codec = fl_standard_message_codec_new();
  g_autoptr(FlBasicMessageChannel) channel = fl_basic_message_channel_new(
      FL_BINARY_MESSENGER(messenger), "test", FL_MESSAGE_CODEC(codec));
  fl_basic_message_channel_send(
      channel, nullptr, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(GError) error = nullptr;
        g_autoptr(FlValue) message = fl_basic_message_channel_send_finish(
            FL_BASIC_MESSAGE_CHANNEL(object), result, &error);
        EXPECT_NE(message, nullptr);
        EXPECT_EQ(error, nullptr);

        EXPECT_EQ(fl_value_get_type(message), FL_VALUE_TYPE_NULL);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  g_main_loop_run(loop);
}

// Checks sending a message with a custom type generates an error.
TEST(FlBasicMessageChannelTest, CustomType) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  fl_mock_binary_messenger_set_standard_message_channel(
      messenger, "test",
      [](FlMockBinaryMessenger* messenger, FlValue* message,
         gpointer user_data) { return fl_value_new_null(); },
      nullptr);

  g_autoptr(FlStandardMessageCodec) codec = fl_standard_message_codec_new();
  g_autoptr(FlBasicMessageChannel) channel = fl_basic_message_channel_new(
      FL_BINARY_MESSENGER(messenger), "test", FL_MESSAGE_CODEC(codec));
  g_autoptr(FlValue) message = fl_value_new_custom(42, nullptr, nullptr);
  fl_basic_message_channel_send(
      channel, message, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(GError) error = nullptr;
        g_autoptr(FlValue) message = fl_basic_message_channel_send_finish(
            FL_BASIC_MESSAGE_CHANNEL(object), result, &error);
        EXPECT_EQ(message, nullptr);
        EXPECT_NE(error, nullptr);
        EXPECT_STREQ(error->message, "Custom value not implemented");

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  g_main_loop_run(loop);
}
