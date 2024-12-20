// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <gtk/gtk.h>

#include "flutter/shell/platform/linux/fl_binary_messenger_private.h"
#include "flutter/shell/platform/linux/fl_platform_handler.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_codec.h"
#include "flutter/shell/platform/linux/testing/fl_mock_binary_messenger.h"
#include "flutter/shell/platform/linux/testing/fl_test.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

G_DECLARE_FINAL_TYPE(FlTestApplication,
                     fl_test_application,
                     FL,
                     TEST_APPLICATION,
                     GtkApplication)

struct _FlTestApplication {
  GtkApplication parent_instance;
  gboolean* dispose_called;
};

G_DEFINE_TYPE(FlTestApplication,
              fl_test_application,
              gtk_application_get_type())

static void fl_test_application_startup(GApplication* application) {
  G_APPLICATION_CLASS(fl_test_application_parent_class)->startup(application);

  // Add a window to this application, which will hold a reference to the
  // application and stop it disposing. See
  // https://gitlab.gnome.org/GNOME/gtk/-/issues/6190
  gtk_application_window_new(GTK_APPLICATION(application));
}

static void fl_test_application_activate(GApplication* application) {
  G_APPLICATION_CLASS(fl_test_application_parent_class)->activate(application);

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlPlatformHandler) handler =
      fl_platform_handler_new(FL_BINARY_MESSENGER(messenger));
  EXPECT_NE(handler, nullptr);

  // Request app exit.
  gboolean called = FALSE;
  g_autoptr(FlValue) args = fl_value_new_map();
  fl_value_set_string_take(args, "type", fl_value_new_string("required"));
  fl_mock_binary_messenger_invoke_json_method(
      messenger, "flutter/platform", "System.exitApplication", args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;

        EXPECT_TRUE(FL_IS_METHOD_SUCCESS_RESPONSE(response));

        g_autoptr(FlValue) expected_result = fl_value_new_map();
        fl_value_set_string_take(expected_result, "response",
                                 fl_value_new_string("exit"));
        EXPECT_TRUE(fl_value_equal(fl_method_success_response_get_result(
                                       FL_METHOD_SUCCESS_RESPONSE(response)),
                                   expected_result));
      },
      &called);
  EXPECT_TRUE(called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

static void fl_test_application_dispose(GObject* object) {
  FlTestApplication* self = FL_TEST_APPLICATION(object);

  *self->dispose_called = true;

  G_OBJECT_CLASS(fl_test_application_parent_class)->dispose(object);
}

static void fl_test_application_class_init(FlTestApplicationClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_test_application_dispose;
  G_APPLICATION_CLASS(klass)->startup = fl_test_application_startup;
  G_APPLICATION_CLASS(klass)->activate = fl_test_application_activate;
}

static void fl_test_application_init(FlTestApplication* self) {}

FlTestApplication* fl_test_application_new(gboolean* dispose_called) {
  FlTestApplication* self = FL_TEST_APPLICATION(
      g_object_new(fl_test_application_get_type(), nullptr));

  // Don't try and register on D-Bus.
  g_application_set_application_id(G_APPLICATION(self), "dev.flutter.GtkTest");
  g_application_set_flags(G_APPLICATION(self), G_APPLICATION_NON_UNIQUE);

  // Added to stop compiler complaining about an unused function.
  FL_IS_TEST_APPLICATION(self);

  self->dispose_called = dispose_called;

  return self;
}

TEST(FlPlatformHandlerTest, PlaySound) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlPlatformHandler) handler =
      fl_platform_handler_new(FL_BINARY_MESSENGER(messenger));
  EXPECT_NE(handler, nullptr);

  gboolean called = FALSE;
  g_autoptr(FlValue) args = fl_value_new_string("SystemSoundType.alert");
  fl_mock_binary_messenger_invoke_json_method(
      messenger, "flutter/platform", "SystemSound.play", args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;

        EXPECT_TRUE(FL_IS_METHOD_SUCCESS_RESPONSE(response));

        g_autoptr(FlValue) expected_result = fl_value_new_null();
        EXPECT_TRUE(fl_value_equal(fl_method_success_response_get_result(
                                       FL_METHOD_SUCCESS_RESPONSE(response)),
                                   expected_result));
      },
      &called);
  EXPECT_TRUE(called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlPlatformHandlerTest, ExitApplication) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlPlatformHandler) handler =
      fl_platform_handler_new(FL_BINARY_MESSENGER(messenger));
  EXPECT_NE(handler, nullptr);

  // Indicate that the binding is initialized.
  gboolean called = FALSE;
  fl_mock_binary_messenger_invoke_json_method(
      messenger, "flutter/platform", "System.initializationComplete", nullptr,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;

        EXPECT_TRUE(FL_IS_METHOD_SUCCESS_RESPONSE(response));

        g_autoptr(FlValue) expected_result = fl_value_new_null();
        EXPECT_TRUE(fl_value_equal(fl_method_success_response_get_result(
                                       FL_METHOD_SUCCESS_RESPONSE(response)),
                                   expected_result));
      },
      &called);
  EXPECT_TRUE(called);

  gboolean request_exit_called = FALSE;
  fl_mock_binary_messenger_set_json_method_channel(
      messenger, "flutter/platform",
      [](FlMockBinaryMessenger* messenger, const gchar* name, FlValue* args,
         gpointer user_data) {
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;

        EXPECT_STREQ(name, "System.requestAppExit");

        g_autoptr(FlValue) expected_args = fl_value_new_map();
        fl_value_set_string_take(expected_args, "type",
                                 fl_value_new_string("cancelable"));
        EXPECT_TRUE(fl_value_equal(args, expected_args));

        // Cancel so it doesn't try and exit this app (i.e. the current test)
        g_autoptr(FlValue) result = fl_value_new_map();
        fl_value_set_string_take(result, "response",
                                 fl_value_new_string("cancel"));
        return FL_METHOD_RESPONSE(fl_method_success_response_new(result));
      },
      &request_exit_called);

  g_autoptr(FlValue) args = fl_value_new_map();
  fl_value_set_string_take(args, "type", fl_value_new_string("cancelable"));
  fl_mock_binary_messenger_invoke_json_method(
      messenger, "flutter/platform", "System.exitApplication", args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        EXPECT_TRUE(FL_IS_METHOD_SUCCESS_RESPONSE(response));

        g_autoptr(FlValue) expected_result = fl_value_new_map();
        fl_value_set_string_take(expected_result, "response",
                                 fl_value_new_string("cancel"));
        EXPECT_TRUE(fl_value_equal(fl_method_success_response_get_result(
                                       FL_METHOD_SUCCESS_RESPONSE(response)),
                                   expected_result));

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  g_main_loop_run(loop);

  EXPECT_TRUE(request_exit_called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlPlatformHandlerTest, ExitApplicationDispose) {
  gtk_init(0, nullptr);

  gboolean dispose_called = false;
  FlTestApplication* application = fl_test_application_new(&dispose_called);

  // Run the application, it will quit after startup.
  g_application_run(G_APPLICATION(application), 0, nullptr);

  EXPECT_FALSE(dispose_called);
  g_object_unref(application);
  EXPECT_TRUE(dispose_called);
}
