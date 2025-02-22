// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_windowing_handler.h"
#include "flutter/shell/platform/linux/fl_binary_messenger_private.h"
#include "flutter/shell/platform/linux/fl_engine_private.h"
#include "flutter/shell/platform/linux/testing/fl_mock_binary_messenger.h"
#include "flutter/shell/platform/linux/testing/fl_test_gtk_logs.h"
#include "flutter/shell/platform/linux/testing/mock_window.h"
#include "flutter/testing/testing.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

static void set_size_arg(FlValue* args,
                         const gchar* name,
                         double width,
                         double height) {
  g_autoptr(FlValue) size_value = fl_value_new_list();
  fl_value_append_take(size_value, fl_value_new_float(width));
  fl_value_append_take(size_value, fl_value_new_float(height));
  fl_value_set_string(args, name, size_value);
}

static FlValue* make_create_regular_args(double width, double height) {
  FlValue* args = fl_value_new_map();
  set_size_arg(args, "size", width, height);
  return args;
}

static int64_t parse_create_regular_response(FlMethodResponse* response) {
  EXPECT_TRUE(FL_IS_METHOD_SUCCESS_RESPONSE(response));

  FlValue* result = fl_method_success_response_get_result(
      FL_METHOD_SUCCESS_RESPONSE(response));
  EXPECT_EQ(fl_value_get_type(result), FL_VALUE_TYPE_MAP);

  FlValue* view_id_value = fl_value_lookup_string(result, "viewId");
  EXPECT_NE(view_id_value, nullptr);
  EXPECT_EQ(fl_value_get_type(view_id_value), FL_VALUE_TYPE_INT);
  int64_t view_id = fl_value_get_int(view_id_value);
  EXPECT_GT(view_id, 0);

  FlValue* size_value = fl_value_lookup_string(result, "size");
  EXPECT_NE(size_value, nullptr);
  EXPECT_EQ(fl_value_get_type(size_value), FL_VALUE_TYPE_LIST);
  EXPECT_EQ(fl_value_get_length(size_value), 2u);
  EXPECT_EQ(fl_value_get_type(fl_value_get_list_value(size_value, 0)),
            FL_VALUE_TYPE_FLOAT);
  EXPECT_EQ(fl_value_get_type(fl_value_get_list_value(size_value, 1)),
            FL_VALUE_TYPE_FLOAT);

  FlValue* state_value = fl_value_lookup_string(result, "state");
  EXPECT_NE(state_value, nullptr);
  EXPECT_EQ(fl_value_get_type(state_value), FL_VALUE_TYPE_STRING);

  return view_id;
}

static FlValue* make_modify_regular_args(int64_t view_id) {
  FlValue* args = fl_value_new_map();
  fl_value_set_string_take(args, "viewId", fl_value_new_int(view_id));
  return args;
}

static FlValue* make_destroy_window_args(int64_t view_id) {
  FlValue* args = fl_value_new_map();
  fl_value_set_string_take(args, "viewId", fl_value_new_int(view_id));
  return args;
}

TEST(FlWindowingHandlerTest, CreateRegular) {
  flutter::testing::fl_ensure_gtk_init();
  ::testing::NiceMock<flutter::testing::MockWindow> mock_window;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlWindowingHandler) handler = fl_windowing_handler_new(engine);

  EXPECT_CALL(mock_window, gtk_window_new);
  EXPECT_CALL(mock_window, gtk_window_set_default_size(::testing::_, 800, 600));

  g_autoptr(FlValue) args = make_create_regular_args(800, 600);

  gboolean called = FALSE;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "createRegular", args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        parse_create_regular_response(response);
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;
      },
      &called);
  EXPECT_TRUE(called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlWindowingHandlerTest, CreateRegularMinSize) {
  flutter::testing::fl_ensure_gtk_init();
  ::testing::NiceMock<flutter::testing::MockWindow> mock_window;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlWindowingHandler) handler = fl_windowing_handler_new(engine);

  EXPECT_CALL(mock_window, gtk_window_new);
  EXPECT_CALL(mock_window,
              gtk_window_set_geometry_hints(
                  ::testing::_, nullptr,
                  ::testing::Pointee(::testing::AllOf(
                      ::testing::Field(&GdkGeometry::min_width, 100),
                      ::testing::Field(&GdkGeometry::min_height, 200))),
                  GDK_HINT_MIN_SIZE));

  g_autoptr(FlValue) args = make_create_regular_args(800, 600);
  set_size_arg(args, "minSize", 100, 200);

  gboolean called = FALSE;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "createRegular", args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        parse_create_regular_response(response);
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;
      },
      &called);
  EXPECT_TRUE(called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlWindowingHandlerTest, CreateRegularMaxSize) {
  flutter::testing::fl_ensure_gtk_init();
  ::testing::NiceMock<flutter::testing::MockWindow> mock_window;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlWindowingHandler) handler = fl_windowing_handler_new(engine);

  EXPECT_CALL(mock_window, gtk_window_new);
  EXPECT_CALL(mock_window,
              gtk_window_set_geometry_hints(
                  ::testing::_, nullptr,
                  ::testing::Pointee(::testing::AllOf(
                      ::testing::Field(&GdkGeometry::max_width, 1000),
                      ::testing::Field(&GdkGeometry::max_height, 2000))),
                  GDK_HINT_MAX_SIZE));

  g_autoptr(FlValue) args = make_create_regular_args(800, 600);
  set_size_arg(args, "maxSize", 1000, 2000);

  gboolean called = FALSE;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "createRegular", args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        parse_create_regular_response(response);
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;
      },
      &called);
  EXPECT_TRUE(called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlWindowingHandlerTest, CreateRegularWithTitle) {
  flutter::testing::fl_ensure_gtk_init();
  ::testing::NiceMock<flutter::testing::MockWindow> mock_window;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlWindowingHandler) handler = fl_windowing_handler_new(engine);

  EXPECT_CALL(mock_window, gtk_window_new);
  EXPECT_CALL(mock_window,
              gtk_window_set_title(::testing::_, ::testing::StrEq("TITLE")));

  g_autoptr(FlValue) args = make_create_regular_args(800, 600);
  fl_value_set_string_take(args, "title", fl_value_new_string("TITLE"));

  gboolean called = FALSE;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "createRegular", args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        parse_create_regular_response(response);
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;
      },
      &called);
  EXPECT_TRUE(called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlWindowingHandlerTest, CreateRegularMaximized) {
  flutter::testing::fl_ensure_gtk_init();
  ::testing::NiceMock<flutter::testing::MockWindow> mock_window;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlWindowingHandler) handler = fl_windowing_handler_new(engine);

  EXPECT_CALL(mock_window, gtk_window_new);
  EXPECT_CALL(mock_window, gtk_window_maximize(::testing::_));

  g_autoptr(FlValue) args = make_create_regular_args(800, 600);
  fl_value_set_string_take(args, "state",
                           fl_value_new_string("WindowState.maximized"));

  gboolean called = FALSE;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "createRegular", args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        parse_create_regular_response(response);
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;
      },
      &called);
  EXPECT_TRUE(called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlWindowingHandlerTest, CreateRegularMinimized) {
  flutter::testing::fl_ensure_gtk_init();
  ::testing::NiceMock<flutter::testing::MockWindow> mock_window;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlWindowingHandler) handler = fl_windowing_handler_new(engine);

  EXPECT_CALL(mock_window, gtk_window_new);
  EXPECT_CALL(mock_window, gtk_window_iconify(::testing::_));

  g_autoptr(FlValue) args = make_create_regular_args(800, 600);
  fl_value_set_string_take(args, "state",
                           fl_value_new_string("WindowState.minimized"));

  gboolean called = FALSE;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "createRegular", args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        parse_create_regular_response(response);
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;
      },
      &called);
  EXPECT_TRUE(called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlWindowingHandlerTest, ModifyRegularSize) {
  flutter::testing::fl_ensure_gtk_init();
  ::testing::NiceMock<flutter::testing::MockWindow> mock_window;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlWindowingHandler) handler = fl_windowing_handler_new(engine);

  EXPECT_CALL(mock_window, gtk_window_new);
  EXPECT_CALL(mock_window, gtk_window_resize(::testing::_, 1024, 768));

  g_autoptr(FlValue) create_args = make_create_regular_args(800, 600);

  int64_t view_id = -1;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "createRegular", create_args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        int64_t* view_id = static_cast<int64_t*>(user_data);
        *view_id = parse_create_regular_response(response);
      },
      &view_id);
  EXPECT_GT(view_id, 0);

  g_autoptr(FlValue) modify_args = make_modify_regular_args(view_id);
  set_size_arg(modify_args, "size", 1024, 768);

  gboolean modify_called = FALSE;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "modifyRegular", modify_args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        EXPECT_TRUE(FL_IS_METHOD_SUCCESS_RESPONSE(response));
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;
      },
      &modify_called);
  EXPECT_TRUE(modify_called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlWindowingHandlerTest, ModifyRegularTitle) {
  flutter::testing::fl_ensure_gtk_init();
  ::testing::NiceMock<flutter::testing::MockWindow> mock_window;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlWindowingHandler) handler = fl_windowing_handler_new(engine);

  EXPECT_CALL(mock_window, gtk_window_new);
  EXPECT_CALL(mock_window,
              gtk_window_set_title(::testing::_, ::testing::StrEq("TITLE")));

  g_autoptr(FlValue) create_args = make_create_regular_args(800, 600);

  int64_t view_id = -1;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "createRegular", create_args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        int64_t* view_id = static_cast<int64_t*>(user_data);
        *view_id = parse_create_regular_response(response);
      },
      &view_id);
  EXPECT_GT(view_id, 0);

  g_autoptr(FlValue) modify_args = make_modify_regular_args(view_id);
  fl_value_set_string_take(modify_args, "title", fl_value_new_string("TITLE"));

  gboolean modify_called = FALSE;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "modifyRegular", modify_args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        EXPECT_TRUE(FL_IS_METHOD_SUCCESS_RESPONSE(response));
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;
      },
      &modify_called);
  EXPECT_TRUE(modify_called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlWindowingHandlerTest, ModifyRegularMaximize) {
  flutter::testing::fl_ensure_gtk_init();
  ::testing::NiceMock<flutter::testing::MockWindow> mock_window;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlWindowingHandler) handler = fl_windowing_handler_new(engine);

  EXPECT_CALL(mock_window, gtk_window_new);

  g_autoptr(FlValue) create_args = make_create_regular_args(800, 600);

  int64_t view_id = -1;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "createRegular", create_args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        int64_t* view_id = static_cast<int64_t*>(user_data);
        *view_id = parse_create_regular_response(response);
      },
      &view_id);
  EXPECT_GT(view_id, 0);

  EXPECT_CALL(mock_window, gtk_window_maximize(::testing::_));

  g_autoptr(FlValue) modify_args = make_modify_regular_args(view_id);
  fl_value_set_string_take(modify_args, "state",
                           fl_value_new_string("WindowState.maximized"));

  gboolean modify_called = FALSE;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "modifyRegular", modify_args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        EXPECT_TRUE(FL_IS_METHOD_SUCCESS_RESPONSE(response));
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;
      },
      &modify_called);
  EXPECT_TRUE(modify_called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlWindowingHandlerTest, ModifyRegularUnmaximize) {
  flutter::testing::fl_ensure_gtk_init();
  ::testing::NiceMock<flutter::testing::MockWindow> mock_window;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlWindowingHandler) handler = fl_windowing_handler_new(engine);

  EXPECT_CALL(mock_window, gtk_window_new);
  EXPECT_CALL(mock_window, gtk_window_maximize(::testing::_));

  g_autoptr(FlValue) create_args = make_create_regular_args(800, 600);
  fl_value_set_string_take(create_args, "state",
                           fl_value_new_string("WindowState.maximized"));

  int64_t view_id = -1;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "createRegular", create_args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        int64_t* view_id = static_cast<int64_t*>(user_data);
        *view_id = parse_create_regular_response(response);
      },
      &view_id);
  EXPECT_GT(view_id, 0);

  EXPECT_CALL(mock_window, gtk_window_is_maximized(::testing::_))
      .WillOnce(::testing::Return(TRUE));
  EXPECT_CALL(mock_window, gtk_window_unmaximize(::testing::_));

  g_autoptr(FlValue) modify_args = make_modify_regular_args(view_id);
  fl_value_set_string_take(modify_args, "state",
                           fl_value_new_string("WindowState.restored"));

  gboolean modify_called = FALSE;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "modifyRegular", modify_args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        EXPECT_TRUE(FL_IS_METHOD_SUCCESS_RESPONSE(response));
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;
      },
      &modify_called);
  EXPECT_TRUE(modify_called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlWindowingHandlerTest, ModifyRegularMinimize) {
  flutter::testing::fl_ensure_gtk_init();
  ::testing::NiceMock<flutter::testing::MockWindow> mock_window;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlWindowingHandler) handler = fl_windowing_handler_new(engine);

  EXPECT_CALL(mock_window, gtk_window_new);

  g_autoptr(FlValue) create_args = make_create_regular_args(800, 600);

  int64_t view_id = -1;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "createRegular", create_args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        int64_t* view_id = static_cast<int64_t*>(user_data);
        *view_id = parse_create_regular_response(response);
      },
      &view_id);
  EXPECT_GT(view_id, 0);

  EXPECT_CALL(mock_window, gtk_window_iconify(::testing::_));

  g_autoptr(FlValue) modify_args = make_modify_regular_args(view_id);
  fl_value_set_string_take(modify_args, "state",
                           fl_value_new_string("WindowState.minimized"));

  gboolean modify_called = FALSE;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "modifyRegular", modify_args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        EXPECT_TRUE(FL_IS_METHOD_SUCCESS_RESPONSE(response));
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;
      },
      &modify_called);
  EXPECT_TRUE(modify_called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlWindowingHandlerTest, ModifyRegularUnminimize) {
  flutter::testing::fl_ensure_gtk_init();
  ::testing::NiceMock<flutter::testing::MockWindow> mock_window;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlWindowingHandler) handler = fl_windowing_handler_new(engine);

  EXPECT_CALL(mock_window, gtk_window_new);
  EXPECT_CALL(mock_window, gtk_window_iconify(::testing::_));

  g_autoptr(FlValue) create_args = make_create_regular_args(800, 600);
  fl_value_set_string_take(create_args, "state",
                           fl_value_new_string("WindowState.minimized"));

  int64_t view_id = -1;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "createRegular", create_args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        int64_t* view_id = static_cast<int64_t*>(user_data);
        *view_id = parse_create_regular_response(response);
      },
      &view_id);
  EXPECT_GT(view_id, 0);

  EXPECT_CALL(mock_window, gdk_window_get_state(::testing::_))
      .WillOnce(::testing::Return(GDK_WINDOW_STATE_ICONIFIED));
  EXPECT_CALL(mock_window, gtk_window_deiconify(::testing::_));

  g_autoptr(FlValue) modify_args = make_modify_regular_args(view_id);
  fl_value_set_string_take(modify_args, "state",
                           fl_value_new_string("WindowState.restored"));

  gboolean modify_called = FALSE;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "modifyRegular", modify_args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        EXPECT_TRUE(FL_IS_METHOD_SUCCESS_RESPONSE(response));
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;
      },
      &modify_called);
  EXPECT_TRUE(modify_called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlWindowingHandlerTest, ModifyUnknownWindow) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlWindowingHandler) handler = fl_windowing_handler_new(engine);

  g_autoptr(FlValue) args = make_modify_regular_args(99);

  gboolean called = FALSE;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "modifyRegular", args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        EXPECT_TRUE(FL_IS_METHOD_ERROR_RESPONSE(response));
        EXPECT_STREQ(fl_method_error_response_get_code(
                         FL_METHOD_ERROR_RESPONSE(response)),
                     "Bad Arguments");
        EXPECT_STREQ(fl_method_error_response_get_message(
                         FL_METHOD_ERROR_RESPONSE(response)),
                     "No window with given view ID");
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;
      },
      &called);
  EXPECT_TRUE(called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlWindowingHandlerTest, DestroyWindow) {
  flutter::testing::fl_ensure_gtk_init();
  ::testing::NiceMock<flutter::testing::MockWindow> mock_window;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlWindowingHandler) handler = fl_windowing_handler_new(engine);

  EXPECT_CALL(mock_window, gtk_window_new);
  EXPECT_CALL(mock_window, gtk_widget_destroy);

  g_autoptr(FlValue) create_args = make_create_regular_args(800, 600);

  int64_t view_id = -1;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "createRegular", create_args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        int64_t* view_id = static_cast<int64_t*>(user_data);
        *view_id = parse_create_regular_response(response);
      },
      &view_id);
  EXPECT_GT(view_id, 0);

  g_autoptr(FlValue) destroy_args = make_destroy_window_args(view_id);
  gboolean destroy_called = FALSE;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "destroyWindow", destroy_args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        EXPECT_TRUE(FL_IS_METHOD_SUCCESS_RESPONSE(response));
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;
      },
      &destroy_called);
  EXPECT_TRUE(destroy_called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlWindowingHandlerTest, DestroyUnknownWindow) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      fl_engine_new_with_binary_messenger(FL_BINARY_MESSENGER(messenger));
  g_autoptr(FlWindowingHandler) handler = fl_windowing_handler_new(engine);

  g_autoptr(FlValue) args = make_destroy_window_args(99);
  gboolean called = FALSE;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, "flutter/windowing", "destroyWindow", args,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        EXPECT_TRUE(FL_IS_METHOD_ERROR_RESPONSE(response));
        EXPECT_STREQ(fl_method_error_response_get_code(
                         FL_METHOD_ERROR_RESPONSE(response)),
                     "Bad Arguments");
        EXPECT_STREQ(fl_method_error_response_get_message(
                         FL_METHOD_ERROR_RESPONSE(response)),
                     "No window with given view ID");
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;
      },
      &called);
  EXPECT_TRUE(called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}
