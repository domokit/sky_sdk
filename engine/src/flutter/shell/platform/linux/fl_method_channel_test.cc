// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Included first as it collides with the X11 headers.
#include "gtest/gtest.h"

#include "flutter/shell/platform/linux/fl_method_codec_private.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_channel.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_standard_method_codec.h"
#include "flutter/shell/platform/linux/testing/fl_mock_binary_messenger.h"

// Checks if invoking a method returns a value.
TEST(FlMethodChannelTest, InvokeMethod) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  fl_mock_binary_messenger_set_standard_method_channel(
      messenger, "test",
      [](FlMockBinaryMessenger* messenger, GTask* task, const gchar* name,
         FlValue* args, gpointer user_data) {
        EXPECT_STREQ(name, "Test");
        EXPECT_EQ(fl_value_get_type(args), FL_VALUE_TYPE_STRING);
        EXPECT_STREQ(fl_value_get_string(args), "Marco!");
        g_autoptr(FlValue) return_value = fl_value_new_string("Polo!");
        return FL_METHOD_RESPONSE(fl_method_success_response_new(return_value));
      },
      nullptr);

  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  g_autoptr(FlMethodChannel) channel = fl_method_channel_new(
      FL_BINARY_MESSENGER(messenger), "test", FL_METHOD_CODEC(codec));

  g_autoptr(FlValue) args = fl_value_new_string("Marco!");
  fl_method_channel_invoke_method(
      channel, "Test", args, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(GError) error = nullptr;
        g_autoptr(FlMethodResponse) response =
            fl_method_channel_invoke_method_finish(FL_METHOD_CHANNEL(object),
                                                   result, &error);
        EXPECT_NE(response, nullptr);
        EXPECT_EQ(error, nullptr);

        FlValue* r = fl_method_response_get_result(response, &error);
        EXPECT_NE(r, nullptr);
        EXPECT_EQ(error, nullptr);

        EXPECT_EQ(fl_value_get_type(r), FL_VALUE_TYPE_STRING);
        EXPECT_STREQ(fl_value_get_string(r), "Polo!");

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  g_main_loop_run(loop);
}

// Checks if a method can be invoked with nullptr for arguments.
TEST(FlMethodChannelTest, InvokeMethodNullptrArgsMessage) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  fl_mock_binary_messenger_set_standard_method_channel(
      messenger, "test",
      [](FlMockBinaryMessenger* messenger, GTask* task, const gchar* name,
         FlValue* args, gpointer user_data) {
        EXPECT_STREQ(name, "Test");
        EXPECT_EQ(fl_value_get_type(args), FL_VALUE_TYPE_NULL);
        return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
      },
      nullptr);

  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  g_autoptr(FlMethodChannel) channel = fl_method_channel_new(
      FL_BINARY_MESSENGER(messenger), "test", FL_METHOD_CODEC(codec));

  fl_method_channel_invoke_method(
      channel, "Test", nullptr, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(GError) error = nullptr;
        g_autoptr(FlMethodResponse) response =
            fl_method_channel_invoke_method_finish(FL_METHOD_CHANNEL(object),
                                                   result, &error);
        EXPECT_NE(response, nullptr);
        EXPECT_EQ(error, nullptr);

        FlValue* r = fl_method_response_get_result(response, &error);
        EXPECT_NE(r, nullptr);
        EXPECT_EQ(error, nullptr);
        EXPECT_EQ(fl_value_get_type(r), FL_VALUE_TYPE_NULL);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  g_main_loop_run(loop);
}

// Checks if an error response from a method call is handled.
TEST(FlMethodChannelTest, InvokeMethodError) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  fl_mock_binary_messenger_set_standard_method_channel(
      messenger, "test",
      [](FlMockBinaryMessenger* messenger, GTask* task, const gchar* name,
         FlValue* args, gpointer user_data) {
        EXPECT_STREQ(name, "Test");
        g_autoptr(FlValue) details = fl_value_new_string("DETAILS");
        return FL_METHOD_RESPONSE(
            fl_method_error_response_new("CODE", "MESSAGE", details));
      },
      nullptr);

  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  g_autoptr(FlMethodChannel) channel = fl_method_channel_new(
      FL_BINARY_MESSENGER(messenger), "test", FL_METHOD_CODEC(codec));

  fl_method_channel_invoke_method(
      channel, "Test", nullptr, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(GError) error = nullptr;
        g_autoptr(FlMethodResponse) response =
            fl_method_channel_invoke_method_finish(FL_METHOD_CHANNEL(object),
                                                   result, &error);
        EXPECT_NE(response, nullptr);
        EXPECT_EQ(error, nullptr);

        EXPECT_TRUE(FL_IS_METHOD_ERROR_RESPONSE(response));
        EXPECT_STREQ(fl_method_error_response_get_code(
                         FL_METHOD_ERROR_RESPONSE(response)),
                     "CODE");
        EXPECT_STREQ(fl_method_error_response_get_message(
                         FL_METHOD_ERROR_RESPONSE(response)),
                     "MESSAGE");
        FlValue* details = fl_method_error_response_get_details(
            FL_METHOD_ERROR_RESPONSE(response));
        EXPECT_NE(details, nullptr);
        EXPECT_EQ(fl_value_get_type(details), FL_VALUE_TYPE_STRING);
        EXPECT_STREQ(fl_value_get_string(details), "DETAILS");

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  g_main_loop_run(loop);
}

// Checks if a not implemeneted response from a method call is handled.
TEST(FlMethodChannelTest, InvokeMethodNotImplemented) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  fl_mock_binary_messenger_set_standard_method_channel(
      messenger, "test",
      [](FlMockBinaryMessenger* messenger, GTask* task, const gchar* name,
         FlValue* args, gpointer user_data) {
        EXPECT_STREQ(name, "Test");
        return FL_METHOD_RESPONSE(fl_method_not_implemented_response_new());
      },
      nullptr);

  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  g_autoptr(FlMethodChannel) channel = fl_method_channel_new(
      FL_BINARY_MESSENGER(messenger), "test", FL_METHOD_CODEC(codec));

  fl_method_channel_invoke_method(
      channel, "Test", nullptr, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(GError) error = nullptr;
        g_autoptr(FlMethodResponse) response =
            fl_method_channel_invoke_method_finish(FL_METHOD_CHANNEL(object),
                                                   result, &error);
        EXPECT_NE(response, nullptr);
        EXPECT_EQ(error, nullptr);

        EXPECT_TRUE(FL_IS_METHOD_NOT_IMPLEMENTED_RESPONSE(response));

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  g_main_loop_run(loop);
}

// Checks if an engine failure calling a method call is handled.
TEST(FlMethodChannelTest, InvokeMethodFailure) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  fl_mock_binary_messenger_set_error_channel(messenger, "test", 42, "ERROR");

  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  g_autoptr(FlMethodChannel) channel = fl_method_channel_new(
      FL_BINARY_MESSENGER(messenger), "test", FL_METHOD_CODEC(codec));

  fl_method_channel_invoke_method(
      channel, "Test", nullptr, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(GError) error = nullptr;
        g_autoptr(FlMethodResponse) response =
            fl_method_channel_invoke_method_finish(FL_METHOD_CHANNEL(object),
                                                   result, &error);
        EXPECT_EQ(response, nullptr);
        EXPECT_NE(error, nullptr);

        EXPECT_EQ(error->code, 42);
        EXPECT_STREQ(error->message, "ERROR");

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  g_main_loop_run(loop);
}

// Checks the shell able to receive and respond to method calls from the engine.
TEST(FlMethodChannelTest, ReceiveMethodCallRespondSuccess) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();

  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  g_autoptr(FlMethodChannel) channel = fl_method_channel_new(
      FL_BINARY_MESSENGER(messenger), "test", FL_METHOD_CODEC(codec));
  fl_method_channel_set_method_call_handler(
      channel,
      [](FlMethodChannel* channel, FlMethodCall* method_call,
         gpointer user_data) {
        EXPECT_STREQ(fl_method_call_get_name(method_call), "Test");
        EXPECT_EQ(fl_value_get_type(fl_method_call_get_args(method_call)),
                  FL_VALUE_TYPE_STRING);
        EXPECT_STREQ(fl_value_get_string(fl_method_call_get_args(method_call)),
                     "Marco!");

        g_autoptr(FlValue) result = fl_value_new_string("Polo!");
        g_autoptr(GError) error = nullptr;
        EXPECT_TRUE(
            fl_method_call_respond_success(method_call, result, &error));
        EXPECT_EQ(error, nullptr);
      },
      nullptr, nullptr);

  // Trigger the engine to make a method call.
  g_autoptr(FlValue) args = fl_value_new_string("Marco!");
  gboolean called = FALSE;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "test", "Test", args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;

        EXPECT_TRUE(FL_IS_METHOD_SUCCESS_RESPONSE(response));
        FlValue* result = fl_method_success_response_get_result(
            FL_METHOD_SUCCESS_RESPONSE(response));
        EXPECT_EQ(fl_value_get_type(result), FL_VALUE_TYPE_STRING);
        EXPECT_STREQ(fl_value_get_string(result), "Polo!");
      },
      &called);
  EXPECT_TRUE(called);
}

// Checks the shell able to receive and respond to method calls from the engine.
TEST(FlMethodChannelTest, ReceiveMethodCallRespondError) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();

  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  g_autoptr(FlMethodChannel) channel = fl_method_channel_new(
      FL_BINARY_MESSENGER(messenger), "test", FL_METHOD_CODEC(codec));
  fl_method_channel_set_method_call_handler(
      channel,
      [](FlMethodChannel* channel, FlMethodCall* method_call,
         gpointer user_data) {
        EXPECT_STREQ(fl_method_call_get_name(method_call), "Test");
        EXPECT_EQ(fl_value_get_type(fl_method_call_get_args(method_call)),
                  FL_VALUE_TYPE_STRING);
        EXPECT_STREQ(fl_value_get_string(fl_method_call_get_args(method_call)),
                     "Marco!");

        g_autoptr(FlValue) details = fl_value_new_string("DETAILS");
        g_autoptr(GError) error = nullptr;
        EXPECT_TRUE(fl_method_call_respond_error(method_call, "CODE", "MESSAGE",
                                                 details, &error));
        EXPECT_EQ(error, nullptr);
      },
      nullptr, nullptr);

  // Trigger the engine to make a method call.
  g_autoptr(FlValue) args = fl_value_new_string("Marco!");
  gboolean called = FALSE;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "test", "Test", args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;

        EXPECT_TRUE(FL_IS_METHOD_ERROR_RESPONSE(response));
        EXPECT_STREQ(fl_method_error_response_get_code(
                         FL_METHOD_ERROR_RESPONSE(response)),
                     "CODE");
        EXPECT_STREQ(fl_method_error_response_get_message(
                         FL_METHOD_ERROR_RESPONSE(response)),
                     "MESSAGE");
        FlValue* details = fl_method_error_response_get_details(
            FL_METHOD_ERROR_RESPONSE(response));
        EXPECT_EQ(fl_value_get_type(details), FL_VALUE_TYPE_STRING);
        EXPECT_STREQ(fl_value_get_string(details), "DETAILS");
      },
      &called);
  EXPECT_TRUE(called);
}

// Checks the shell able to receive and respond to method calls from the engine.
TEST(FlMethodChannelTest, ReceiveMethodCallRespondNotImplemented) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();

  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  g_autoptr(FlMethodChannel) channel = fl_method_channel_new(
      FL_BINARY_MESSENGER(messenger), "test", FL_METHOD_CODEC(codec));
  fl_method_channel_set_method_call_handler(
      channel,
      [](FlMethodChannel* channel, FlMethodCall* method_call,
         gpointer user_data) {
        EXPECT_STREQ(fl_method_call_get_name(method_call), "Test");
        EXPECT_EQ(fl_value_get_type(fl_method_call_get_args(method_call)),
                  FL_VALUE_TYPE_STRING);
        EXPECT_STREQ(fl_value_get_string(fl_method_call_get_args(method_call)),
                     "Marco!");

        g_autoptr(GError) error = nullptr;
        EXPECT_TRUE(
            fl_method_call_respond_not_implemented(method_call, &error));
        EXPECT_EQ(error, nullptr);
      },
      nullptr, nullptr);

  // Trigger the engine to make a method call.
  g_autoptr(FlValue) args = fl_value_new_string("Marco!");
  gboolean called = FALSE;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "test", "Test", args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;

        EXPECT_TRUE(FL_IS_METHOD_NOT_IMPLEMENTED_RESPONSE(response));
      },
      &called);
  EXPECT_TRUE(called);
}

// A test method codec that always generates errors on responses.
G_DECLARE_FINAL_TYPE(TestMethodCodec,
                     test_method_codec,
                     TEST,
                     METHOD_CODEC,
                     FlMethodCodec)

struct _TestMethodCodec {
  FlMethodCodec parent_instance;

  FlStandardMethodCodec* wrapped_codec;
};

G_DEFINE_TYPE(TestMethodCodec, test_method_codec, fl_method_codec_get_type())

static void test_method_codec_dispose(GObject* object) {
  TestMethodCodec* self = TEST_METHOD_CODEC(object);

  g_clear_object(&self->wrapped_codec);

  G_OBJECT_CLASS(test_method_codec_parent_class)->dispose(object);
}

// Implements FlMethodCodec::encode_method_call.
static GBytes* test_method_codec_encode_method_call(FlMethodCodec* codec,
                                                    const gchar* name,
                                                    FlValue* args,
                                                    GError** error) {
  EXPECT_TRUE(TEST_IS_METHOD_CODEC(codec));
  TestMethodCodec* self = TEST_METHOD_CODEC(codec);
  return fl_method_codec_encode_method_call(
      FL_METHOD_CODEC(self->wrapped_codec), name, args, error);
}

// Implements FlMethodCodec::decode_method_call.
static gboolean test_method_codec_decode_method_call(FlMethodCodec* codec,
                                                     GBytes* message,
                                                     gchar** name,
                                                     FlValue** args,
                                                     GError** error) {
  EXPECT_TRUE(TEST_IS_METHOD_CODEC(codec));
  TestMethodCodec* self = TEST_METHOD_CODEC(codec);
  return fl_method_codec_decode_method_call(
      FL_METHOD_CODEC(self->wrapped_codec), message, name, args, error);
}

// Implements FlMethodCodec::encode_success_envelope.
static GBytes* test_method_codec_encode_success_envelope(FlMethodCodec* codec,
                                                         FlValue* result,
                                                         GError** error) {
  g_set_error(error, FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_FAILED,
              "Unsupported type");
  return nullptr;
}

// Implements FlMethodCodec::encode_error_envelope.
static GBytes* test_method_codec_encode_error_envelope(FlMethodCodec* codec,
                                                       const gchar* code,
                                                       const gchar* message,
                                                       FlValue* details,
                                                       GError** error) {
  g_set_error(error, FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_FAILED,
              "Unsupported type");
  return nullptr;
}

// Implements FlMethodCodec::encode_decode_response.
static FlMethodResponse* test_method_codec_decode_response(FlMethodCodec* codec,
                                                           GBytes* message,
                                                           GError** error) {
  EXPECT_TRUE(TEST_IS_METHOD_CODEC(codec));
  TestMethodCodec* self = TEST_METHOD_CODEC(codec);
  return fl_method_codec_decode_response(FL_METHOD_CODEC(self->wrapped_codec),
                                         message, error);
}

static void test_method_codec_class_init(TestMethodCodecClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = test_method_codec_dispose;
  FL_METHOD_CODEC_CLASS(klass)->encode_method_call =
      test_method_codec_encode_method_call;
  FL_METHOD_CODEC_CLASS(klass)->decode_method_call =
      test_method_codec_decode_method_call;
  FL_METHOD_CODEC_CLASS(klass)->encode_success_envelope =
      test_method_codec_encode_success_envelope;
  FL_METHOD_CODEC_CLASS(klass)->encode_error_envelope =
      test_method_codec_encode_error_envelope;
  FL_METHOD_CODEC_CLASS(klass)->decode_response =
      test_method_codec_decode_response;
}

static void test_method_codec_init(TestMethodCodec* self) {
  self->wrapped_codec = fl_standard_method_codec_new();
}

TestMethodCodec* test_method_codec_new() {
  return TEST_METHOD_CODEC(g_object_new(test_method_codec_get_type(), nullptr));
}

// Checks error correctly handled if provide an unsupported arg in a method call
// response.
TEST(FlMethodChannelTest, ReceiveMethodCallRespondSuccessError) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();

  g_autoptr(TestMethodCodec) codec = test_method_codec_new();
  g_autoptr(FlMethodChannel) channel = fl_method_channel_new(
      FL_BINARY_MESSENGER(messenger), "test", FL_METHOD_CODEC(codec));
  gboolean called = FALSE;
  fl_method_channel_set_method_call_handler(
      channel,
      [](FlMethodChannel* channel, FlMethodCall* method_call,
         gpointer user_data) {
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;

        g_autoptr(FlValue) result = fl_value_new_int(42);
        g_autoptr(GError) response_error = nullptr;
        EXPECT_FALSE(fl_method_call_respond_success(method_call, result,
                                                    &response_error));
        EXPECT_NE(response_error, nullptr);
        EXPECT_STREQ(response_error->message, "Unsupported type");

        // Respond to stop a warning occurring about not responding.
        fl_method_call_respond_not_implemented(method_call, nullptr);
      },
      &called, nullptr);

  // Trigger the engine to make a method call.
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "test", "Test", nullptr,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {},
      nullptr);

  EXPECT_TRUE(called);
}

// Checks error correctly handled if provide an unsupported arg in a method call
// response.
TEST(FlMethodChannelTest, ReceiveMethodCallRespondErrorError) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();

  g_autoptr(TestMethodCodec) codec = test_method_codec_new();
  g_autoptr(FlMethodChannel) channel = fl_method_channel_new(
      FL_BINARY_MESSENGER(messenger), "test", FL_METHOD_CODEC(codec));
  gboolean called = FALSE;
  fl_method_channel_set_method_call_handler(
      channel,
      [](FlMethodChannel* channel, FlMethodCall* method_call,
         gpointer user_data) {
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;

        g_autoptr(FlValue) details = fl_value_new_int(42);
        g_autoptr(GError) response_error = nullptr;
        EXPECT_FALSE(fl_method_call_respond_error(method_call, "error", "ERROR",
                                                  details, &response_error));
        EXPECT_NE(response_error, nullptr);
        EXPECT_STREQ(response_error->message, "Unsupported type");
      },
      &called, nullptr);

  // Trigger the engine to make a method call.
  fl_mock_binary_messenger_invoke_standard_method(messenger, "test", "Test",
                                                  nullptr, nullptr, nullptr);

  EXPECT_TRUE(called);
}

// Make sure that the following steps will work properly:
//
// 1. Register a method channel.
// 2. Dispose the method channel, and it's unregistered.
// 3. Register a new channel with the same name.
//
// This is a regression test to https://github.com/flutter/flutter/issues/90817.
TEST(FlMethodChannelTest, ReplaceADisposedMethodChannel) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();

  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();

  // Register the first channel and test if it works.
  FlMethodChannel* channel1 = fl_method_channel_new(
      FL_BINARY_MESSENGER(messenger), "test", FL_METHOD_CODEC(codec));
  int first_count = 0;
  fl_method_channel_set_method_call_handler(
      channel1,
      [](FlMethodChannel* channel, FlMethodCall* method_call,
         gpointer user_data) {
        int* first_count = static_cast<int*>(user_data);
        (*first_count)++;

        EXPECT_TRUE(
            fl_method_call_respond_success(method_call, nullptr, nullptr));
      },
      &first_count, nullptr);

  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "test", "Test", nullptr,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {},
      nullptr);
  EXPECT_EQ(first_count, 1);

  // Dispose the first channel.
  g_object_unref(channel1);

  // Register the second channel and test if it works.
  g_autoptr(FlMethodChannel) channel2 = fl_method_channel_new(
      FL_BINARY_MESSENGER(messenger), "test", FL_METHOD_CODEC(codec));
  int second_count = 0;
  fl_method_channel_set_method_call_handler(
      channel2,
      [](FlMethodChannel* channel, FlMethodCall* method_call,
         gpointer user_data) {
        int* second_count = static_cast<int*>(user_data);
        (*second_count)++;

        EXPECT_TRUE(
            fl_method_call_respond_success(method_call, nullptr, nullptr));
      },
      &second_count, nullptr);

  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "test", "Test", nullptr,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {},
      nullptr);
  EXPECT_EQ(first_count, 1);
  EXPECT_EQ(second_count, 1);
}

// Make sure that the following steps will work properly:
//
// 1. Register a method channel.
// 2. Register the same name with a new channel.
// 3. Dispose the previous method channel.
//
// This is a regression test to https://github.com/flutter/flutter/issues/90817.
TEST(FlMethodChannelTest, DisposeAReplacedMethodChannel) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();

  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();

  // Register the first channel and test if it works.
  FlMethodChannel* channel1 = fl_method_channel_new(
      FL_BINARY_MESSENGER(messenger), "test", FL_METHOD_CODEC(codec));
  int first_count = 0;
  fl_method_channel_set_method_call_handler(
      channel1,
      [](FlMethodChannel* channel, FlMethodCall* method_call,
         gpointer user_data) {
        int* first_count = static_cast<int*>(user_data);
        (*first_count)++;

        EXPECT_TRUE(
            fl_method_call_respond_success(method_call, nullptr, nullptr));
      },
      &first_count, nullptr);

  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "test", "Test", nullptr,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {},
      nullptr);
  EXPECT_EQ(first_count, 1);

  // Register a new channel to the same name.
  g_autoptr(FlMethodChannel) channel2 = fl_method_channel_new(
      FL_BINARY_MESSENGER(messenger), "test", FL_METHOD_CODEC(codec));
  int second_count = 0;
  fl_method_channel_set_method_call_handler(
      channel2,
      [](FlMethodChannel* channel, FlMethodCall* method_call,
         gpointer user_data) {
        int* second_count = static_cast<int*>(user_data);
        (*second_count)++;

        EXPECT_TRUE(
            fl_method_call_respond_success(method_call, nullptr, nullptr));
      },
      &second_count, nullptr);

  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "test", "Test", nullptr,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {},
      nullptr);
  EXPECT_EQ(first_count, 1);
  EXPECT_EQ(second_count, 1);

  // Dispose the first channel. The new channel should keep working.
  g_object_unref(channel1);

  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "test", "Test", nullptr,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {},
      nullptr);
  EXPECT_EQ(first_count, 1);
  EXPECT_EQ(second_count, 2);
}

// Checks invoking a method with a custom type generates an error.
TEST(FlMethodChannelTest, CustomType) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  fl_mock_binary_messenger_set_standard_method_channel(
      messenger, "test",
      [](FlMockBinaryMessenger* messenger, GTask* task, const gchar* name,
         FlValue* args, gpointer user_data) {
        return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
      },
      nullptr);

  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  g_autoptr(FlMethodChannel) channel = fl_method_channel_new(
      FL_BINARY_MESSENGER(messenger), "test", FL_METHOD_CODEC(codec));

  g_autoptr(FlValue) args = fl_value_new_custom(42, nullptr, nullptr);
  fl_method_channel_invoke_method(
      channel, "Test", args, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(GError) error = nullptr;
        g_autoptr(FlMethodResponse) response =
            fl_method_channel_invoke_method_finish(FL_METHOD_CHANNEL(object),
                                                   result, &error);
        EXPECT_EQ(response, nullptr);
        EXPECT_NE(error, nullptr);
        EXPECT_STREQ(error->message, "Custom value not implemented");

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  g_main_loop_run(loop);
}
