// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Included first as it collides with the X11 headers.
#include "gtest/gtest.h"

#include <pthread.h>
#include <cstring>

#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/embedder/test_utils/proc_table_replacement.h"
#include "flutter/shell/platform/linux/fl_binary_messenger_private.h"
#include "flutter/shell/platform/linux/fl_engine_private.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_binary_messenger.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_standard_method_codec.h"

// Checks can send a message.
TEST(FlBinaryMessengerTest, Send) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  FlutterDataCallback response_callback;
  void* response_callback_user_data;
  fl_engine_get_embedder_api(engine)->PlatformMessageCreateResponseHandle =
      MOCK_ENGINE_PROC(
          PlatformMessageCreateResponseHandle,
          ([&response_callback, &response_callback_user_data](
               auto engine, FlutterDataCallback data_callback, void* user_data,
               FlutterPlatformMessageResponseHandle** response_out) {
            response_callback = data_callback;
            response_callback_user_data = user_data;
            return kSuccess;
          }));
  fl_engine_get_embedder_api(engine)->SendPlatformMessage = MOCK_ENGINE_PROC(
      SendPlatformMessage,
      ([&response_callback, &response_callback_user_data](
           auto engine, const FlutterPlatformMessage* message) {
        EXPECT_STREQ(message->channel, "test");
        g_autofree gchar* text =
            g_strndup(reinterpret_cast<const gchar*>(message->message),
                      message->message_size);
        EXPECT_STREQ(text, "Marco!");

        const gchar* response = "Polo!";
        response_callback(reinterpret_cast<const uint8_t*>(response),
                          strlen(response), response_callback_user_data);

        return kSuccess;
      }));

  g_autoptr(FlBinaryMessenger) messenger = fl_binary_messenger_new(engine);
  const char* text = "Marco!";
  g_autoptr(GBytes) message = g_bytes_new(text, strlen(text));
  fl_binary_messenger_send_on_channel(
      messenger, "test", message, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(GError) error = nullptr;
        g_autoptr(GBytes) message = fl_binary_messenger_send_on_channel_finish(
            FL_BINARY_MESSENGER(object), result, &error);
        EXPECT_NE(message, nullptr);
        EXPECT_EQ(error, nullptr);

        g_autofree gchar* text = g_strndup(
            static_cast<const gchar*>(g_bytes_get_data(message, nullptr)),
            g_bytes_get_size(message));
        EXPECT_STREQ(text, "Polo!");

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  g_main_loop_run(loop);
}

// Checks sending nullptr for a message works.
TEST(FlBinaryMessengerTest, SendNullptr) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  bool called = false;
  fl_engine_get_embedder_api(engine)->SendPlatformMessage = MOCK_ENGINE_PROC(
      SendPlatformMessage,
      ([&called](auto engine, const FlutterPlatformMessage* message) {
        called = true;

        EXPECT_STREQ(message->channel, "test");
        // Note we don't check message->message as it could be nullptr or a
        // pointer to an buffer - either way it wouldn't be accessed.
        EXPECT_EQ(message->message_size, static_cast<size_t>(0));

        return kSuccess;
      }));

  g_autoptr(FlBinaryMessenger) messenger = fl_binary_messenger_new(engine);
  fl_binary_messenger_send_on_channel(messenger, "test", nullptr, nullptr,
                                      nullptr, nullptr);
  EXPECT_TRUE(called);
}

// Checks sending a zero length message works.
TEST(FlBinaryMessengerTest, SendEmpty) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  bool called = false;
  fl_engine_get_embedder_api(engine)->SendPlatformMessage = MOCK_ENGINE_PROC(
      SendPlatformMessage,
      ([&called](auto engine, const FlutterPlatformMessage* message) {
        called = true;

        EXPECT_STREQ(message->channel, "test");
        EXPECT_EQ(message->message_size, static_cast<size_t>(0));

        return kSuccess;
      }));
  g_autoptr(FlBinaryMessenger) messenger = fl_binary_messenger_new(engine);
  g_autoptr(GBytes) message = g_bytes_new(nullptr, 0);
  fl_binary_messenger_send_on_channel(messenger, "test", message, nullptr,
                                      nullptr, nullptr);
  EXPECT_TRUE(called);
}

// Checks the engine returning a nullptr message work.
TEST(FlBinaryMessengerTest, NullptrResponse) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  FlutterDataCallback response_callback;
  void* response_callback_user_data;
  fl_engine_get_embedder_api(engine)->PlatformMessageCreateResponseHandle =
      MOCK_ENGINE_PROC(
          PlatformMessageCreateResponseHandle,
          ([&response_callback, &response_callback_user_data](
               auto engine, FlutterDataCallback data_callback, void* user_data,
               FlutterPlatformMessageResponseHandle** response_out) {
            response_callback = data_callback;
            response_callback_user_data = user_data;
            return kSuccess;
          }));
  fl_engine_get_embedder_api(engine)->SendPlatformMessage = MOCK_ENGINE_PROC(
      SendPlatformMessage,
      ([&response_callback, &response_callback_user_data](
           auto engine, const FlutterPlatformMessage* message) {
        EXPECT_STREQ(message->channel, "test");
        g_autofree gchar* text =
            g_strndup(reinterpret_cast<const gchar*>(message->message),
                      message->message_size);
        EXPECT_STREQ(text, "Hello World!");

        response_callback(nullptr, 0, response_callback_user_data);

        return kSuccess;
      }));

  g_autoptr(FlBinaryMessenger) messenger = fl_binary_messenger_new(engine);
  const char* text = "Hello World!";
  g_autoptr(GBytes) message = g_bytes_new(text, strlen(text));
  fl_binary_messenger_send_on_channel(
      messenger, "test", message, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(GError) error = nullptr;
        g_autoptr(GBytes) message = fl_binary_messenger_send_on_channel_finish(
            FL_BINARY_MESSENGER(object), result, &error);
        EXPECT_NE(message, nullptr);
        EXPECT_EQ(error, nullptr);

        EXPECT_EQ(g_bytes_get_size(message), static_cast<gsize>(0));

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  g_main_loop_run(loop);
}

// Checks the engine reporting a send failure is handled.
TEST(FlBinaryMessengerTest, SendFailure) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  fl_engine_get_embedder_api(engine)->SendPlatformMessage =
      MOCK_ENGINE_PROC(SendPlatformMessage,
                       ([](auto engine, const FlutterPlatformMessage* message) {
                         EXPECT_STREQ(message->channel, "test");
                         return kInternalInconsistency;
                       }));

  g_autoptr(FlBinaryMessenger) messenger = fl_binary_messenger_new(engine);
  fl_binary_messenger_send_on_channel(
      messenger, "test", nullptr, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(GError) error = nullptr;
        g_autoptr(GBytes) message = fl_binary_messenger_send_on_channel_finish(
            FL_BINARY_MESSENGER(object), result, &error);
        EXPECT_EQ(message, nullptr);
        EXPECT_NE(error, nullptr);
        EXPECT_STREQ(error->message, "Failed to send platform messages");

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  g_main_loop_run(loop);
}

// Checks can receive a message.
TEST(FlBinaryMessengerTest, Receive) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  bool called = false;
  fl_engine_get_embedder_api(engine)->SendPlatformMessageResponse =
      MOCK_ENGINE_PROC(
          SendPlatformMessageResponse,
          ([&called](auto engine,
                     const FlutterPlatformMessageResponseHandle* handle,
                     const uint8_t* data, size_t data_length) {
            called = true;

            int fake_handle = *reinterpret_cast<const int*>(handle);
            EXPECT_EQ(fake_handle, 42);

            g_autofree gchar* text =
                g_strndup(reinterpret_cast<const gchar*>(data), data_length);
            EXPECT_STREQ(text, "Polo!");

            return kSuccess;
          }));

  FlBinaryMessenger* messenger = fl_engine_get_binary_messenger(engine);

  // Listen for message.
  fl_binary_messenger_set_message_handler_on_channel(
      messenger, "test",
      [](FlBinaryMessenger* messenger, const gchar* channel, GBytes* message,
         FlBinaryMessengerResponseHandle* response_handle, gpointer user_data) {
        g_autofree gchar* text = g_strndup(
            static_cast<const gchar*>(g_bytes_get_data(message, nullptr)),
            g_bytes_get_size(message));
        EXPECT_STREQ(text, "Marco!");

        const char* response_text = "Polo!";
        g_autoptr(GBytes) response =
            g_bytes_new(response_text, strlen(response_text));
        g_autoptr(GError) error = nullptr;
        EXPECT_TRUE(fl_binary_messenger_send_response(
            messenger, response_handle, response, &error));
        EXPECT_EQ(error, nullptr);
      },
      nullptr, nullptr);

  // Send message from engine.
  const char* message_text = "Marco!";
  g_autoptr(GBytes) message = g_bytes_new(message_text, strlen(message_text));
  int fake_handle = 42;
  fl_binary_messenger_handle_message(
      messenger, "test", message,
      reinterpret_cast<const FlutterPlatformMessageResponseHandle*>(
          &fake_handle));

  EXPECT_TRUE(called);
}

// Checks receieved messages can be responded to on a thread.
TEST(FlBinaryMessengerTest, ReceiveRespondThread) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  fl_engine_get_embedder_api(engine)->SendPlatformMessageResponse =
      MOCK_ENGINE_PROC(
          SendPlatformMessageResponse,
          ([&loop](auto engine,
                   const FlutterPlatformMessageResponseHandle* handle,
                   const uint8_t* data, size_t data_length) {
            int fake_handle = *reinterpret_cast<const int*>(handle);
            EXPECT_EQ(fake_handle, 42);

            g_autofree gchar* text =
                g_strndup(reinterpret_cast<const gchar*>(data), data_length);
            EXPECT_STREQ(text, "Polo!");

            g_main_loop_quit(loop);

            return kSuccess;
          }));

  FlBinaryMessenger* messenger = fl_engine_get_binary_messenger(engine);

  // Listen for message.
  fl_binary_messenger_set_message_handler_on_channel(
      messenger, "test",
      [](FlBinaryMessenger* messenger, const gchar* channel, GBytes* message,
         FlBinaryMessengerResponseHandle* response_handle, gpointer user_data) {
        g_autofree gchar* text = g_strndup(
            static_cast<const gchar*>(g_bytes_get_data(message, nullptr)),
            g_bytes_get_size(message));
        EXPECT_STREQ(text, "Marco!");

        // Respond on a thread.
        typedef struct {
          FlBinaryMessenger* messenger;
          FlBinaryMessengerResponseHandle* response_handle;
        } ThreadData;
        ThreadData* data = g_new0(ThreadData, 1);
        data->messenger =
            static_cast<FlBinaryMessenger*>(g_object_ref(messenger));
        data->response_handle = static_cast<FlBinaryMessengerResponseHandle*>(
            g_object_ref(response_handle));
        g_autoptr(GThread) thread = g_thread_new(
            nullptr,
            [](gpointer user_data) {
              g_autofree ThreadData* data = static_cast<ThreadData*>(user_data);
              g_autoptr(FlBinaryMessenger) messenger = data->messenger;
              g_autoptr(FlBinaryMessengerResponseHandle) response_handle =
                  data->response_handle;

              const char* response_text = "Polo!";
              g_autoptr(GBytes) response =
                  g_bytes_new(response_text, strlen(response_text));
              g_autoptr(GError) error = nullptr;
              EXPECT_TRUE(fl_binary_messenger_send_response(
                  data->messenger, data->response_handle, response, &error));
              EXPECT_EQ(error, nullptr);

              return static_cast<gpointer>(nullptr);
            },
            data);
      },
      nullptr, nullptr);

  // Send message from engine.
  const char* message_text = "Marco!";
  g_autoptr(GBytes) message = g_bytes_new(message_text, strlen(message_text));
  int fake_handle = 42;
  fl_binary_messenger_handle_message(
      messenger, "test", message,
      reinterpret_cast<const FlutterPlatformMessageResponseHandle*>(
          &fake_handle));

  g_main_loop_run(loop);
}

// MOCK_ENGINE_PROC is leaky by design.
// NOLINTBEGIN(clang-analyzer-core.StackAddressEscape)

// Checks if the 'resize' command is sent and is well-formed.
TEST(FlBinaryMessengerTest, ResizeChannel) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  bool called = false;

  FlutterEngineSendPlatformMessageFnPtr old_handler =
      fl_engine_get_embedder_api(engine)->SendPlatformMessage;
  fl_engine_get_embedder_api(engine)->SendPlatformMessage = MOCK_ENGINE_PROC(
      SendPlatformMessage,
      ([&called, old_handler](auto engine,
                              const FlutterPlatformMessage* message) {
        // Expect to receive a message on the "control" channel.
        if (strcmp(message->channel, "dev.flutter/channel-buffers") != 0) {
          return old_handler(engine, message);
        }

        called = true;

        // The expected content was created from the following Dart code:
        //   MethodCall call = MethodCall('resize', ['flutter/test',3]);
        //   StandardMethodCodec().encodeMethodCall(call).buffer.asUint8List();
        const int expected_message_size = 29;
        EXPECT_EQ(message->message_size,
                  static_cast<size_t>(expected_message_size));
        int expected[expected_message_size] = {
            7,   6,   114, 101, 115, 105, 122, 101, 12,  2,
            7,   12,  102, 108, 117, 116, 116, 101, 114, 47,
            116, 101, 115, 116, 3,   3,   0,   0,   0};
        for (size_t i = 0; i < expected_message_size; i++) {
          EXPECT_EQ(message->message[i], expected[i]);
        }

        return kSuccess;
      }));

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  g_autoptr(FlBinaryMessenger) messenger = fl_binary_messenger_new(engine);
  fl_binary_messenger_resize_channel(messenger, "flutter/test", 3);

  EXPECT_TRUE(called);
}

// Checks if the 'overflow' command is sent and is well-formed.
TEST(FlBinaryMessengerTest, WarnsOnOverflowChannel) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  bool called = false;

  FlutterEngineSendPlatformMessageFnPtr old_handler =
      fl_engine_get_embedder_api(engine)->SendPlatformMessage;
  fl_engine_get_embedder_api(engine)->SendPlatformMessage = MOCK_ENGINE_PROC(
      SendPlatformMessage,
      ([&called, old_handler](auto engine,
                              const FlutterPlatformMessage* message) {
        // Expect to receive a message on the "control" channel.
        if (strcmp(message->channel, "dev.flutter/channel-buffers") != 0) {
          return old_handler(engine, message);
        }

        called = true;

        // The expected content was created from the following Dart code:
        //   MethodCall call = MethodCall('overflow',['flutter/test', true]);
        //   StandardMethodCodec().encodeMethodCall(call).buffer.asUint8List();
        const int expected_message_size = 27;
        EXPECT_EQ(message->message_size,
                  static_cast<size_t>(expected_message_size));
        int expected[expected_message_size] = {
            7,   8,   111, 118, 101, 114, 102, 108, 111, 119, 12,  2,   7, 12,
            102, 108, 117, 116, 116, 101, 114, 47,  116, 101, 115, 116, 1};
        for (size_t i = 0; i < expected_message_size; i++) {
          EXPECT_EQ(message->message[i], expected[i]);
        }

        return kSuccess;
      }));

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  g_autoptr(FlBinaryMessenger) messenger = fl_binary_messenger_new(engine);
  fl_binary_messenger_set_warns_on_channel_overflow(messenger, "flutter/test",
                                                    false);

  EXPECT_TRUE(called);
}

// Checks if error returned when invoking a command on the control channel
// are handled.
TEST(FlBinaryMessengerTest, ControlChannelErrorResponse) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  g_autoptr(FlBinaryMessenger) messenger = fl_binary_messenger_new(engine);
  bool called = false;
  FlutterEngineSendPlatformMessageFnPtr old_handler =
      fl_engine_get_embedder_api(engine)->SendPlatformMessage;
  fl_engine_get_embedder_api(engine)->SendPlatformMessage = MOCK_ENGINE_PROC(
      SendPlatformMessage,
      ([&called, old_handler, loop](auto engine,
                                    const FlutterPlatformMessage* message) {
        // Expect to receive a message on the "control" channel.
        if (strcmp(message->channel, "dev.flutter/channel-buffers") != 0) {
          return old_handler(engine, message);
        }

        called = true;

        // Register a callback to quit the main loop when binary messenger work
        // ends.
        g_idle_add(
            [](gpointer user_data) {
              g_main_loop_quit(static_cast<GMainLoop*>(user_data));
              return FALSE;
            },
            loop);

        // Simulates an internal error.
        return kInvalidArguments;
      }));

  fl_binary_messenger_set_warns_on_channel_overflow(messenger, "flutter/test",
                                                    false);

  EXPECT_TRUE(called);

  g_main_loop_run(loop);
}

// NOLINTEND(clang-analyzer-core.StackAddressEscape)

TEST(FlBinaryMessengerTest, DeletingEngineClearsHandlers) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  FlBinaryMessenger* messenger = fl_engine_get_binary_messenger(engine);

  // Add handler to check the destroy_notify is called.
  gboolean destroy_notify_called = FALSE;
  fl_binary_messenger_set_message_handler_on_channel(
      messenger, "test",
      [](FlBinaryMessenger* messenger, const gchar* channel, GBytes* message,
         FlBinaryMessengerResponseHandle* response_handle,
         gpointer user_data) {},
      &destroy_notify_called,
      [](gpointer user_data) { *static_cast<gboolean*>(user_data) = TRUE; });

  g_clear_object(&engine);

  ASSERT_TRUE(destroy_notify_called);
}
