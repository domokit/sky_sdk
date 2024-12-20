// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_platform_channel.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_channel.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_standard_method_codec.h"
#include "flutter/shell/platform/linux/testing/fl_mock_binary_messenger.h"

#include "gtest/gtest.h"

TEST(FlPlatformChannelTest, ExitResponse) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  fl_mock_binary_messenger_set_json_method_channel(
      messenger, "flutter/platform",
      [](FlMockBinaryMessenger* messenger, const gchar* name, FlValue* args,
         gpointer user_data) {
        EXPECT_STREQ(name, "System.requestAppExit");
        g_autoptr(FlValue) return_value = fl_value_new_map();
        fl_value_set_string_take(return_value, "response",
                                 fl_value_new_string("exit"));
        return FL_METHOD_RESPONSE(fl_method_success_response_new(return_value));
      },
      nullptr);

  FlPlatformChannelVTable vtable;
  g_autoptr(FlPlatformChannel) channel =
      fl_platform_channel_new(FL_BINARY_MESSENGER(messenger), &vtable, nullptr);
  fl_platform_channel_system_request_app_exit(
      channel, FL_PLATFORM_CHANNEL_EXIT_TYPE_REQUIRED, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        g_autoptr(GError) error = nullptr;
        FlPlatformChannelExitResponse response;
        gboolean success = fl_platform_channel_system_request_app_exit_finish(
            object, result, &response, &error);

        EXPECT_TRUE(success);
        EXPECT_EQ(response, FL_PLATFORM_CHANNEL_EXIT_RESPONSE_EXIT);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  g_main_loop_run(loop);
}
