// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <utility>

#include "flutter/shell/platform/linux/fl_binary_messenger_private.h"
#include "flutter/shell/platform/linux/fl_method_codec_private.h"
#include "flutter/shell/platform/linux/fl_text_input_handler.h"
#include "flutter/shell/platform/linux/testing/fl_mock_binary_messenger.h"
#include "flutter/shell/platform/linux/testing/fl_test.h"
#include "flutter/shell/platform/linux/testing/mock_im_context.h"
#include "flutter/shell/platform/linux/testing/mock_text_input_view_delegate.h"
#include "flutter/testing/testing.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

static FlValue* build_map(std::map<const gchar*, FlValue*> args) {
  FlValue* value = fl_value_new_map();
  for (auto it = args.begin(); it != args.end(); ++it) {
    fl_value_set_string_take(value, it->first, it->second);
  }
  return value;
}

static FlValue* build_list(std::vector<FlValue*> args) {
  FlValue* value = fl_value_new_list();
  for (auto it = args.begin(); it != args.end(); ++it) {
    fl_value_append_take(value, *it);
  }
  return value;
}

struct InputConfig {
  int64_t client_id = -1;
  const gchar* input_type = "TextInputType.text";
  const gchar* input_action = "TextInputAction.none";
  gboolean enable_delta_model = false;
};

static FlValue* build_input_config(InputConfig config) {
  return build_list({
      fl_value_new_int(config.client_id),
      build_map({
          {"inputAction", fl_value_new_string(config.input_action)},
          {"inputType", build_map({
                            {"name", fl_value_new_string(config.input_type)},
                        })},
          {"enableDeltaModel", fl_value_new_bool(config.enable_delta_model)},
      }),
  });
}

struct EditingState {
  const gchar* text = "";
  int selection_base = -1;
  int selection_extent = -1;
  int composing_base = -1;
  int composing_extent = -1;
};

static FlValue* build_editing_state(EditingState state) {
  return build_map({
      {"text", fl_value_new_string(state.text)},
      {"selectionBase", fl_value_new_int(state.selection_base)},
      {"selectionExtent", fl_value_new_int(state.selection_extent)},
      {"selectionAffinity", fl_value_new_string("TextAffinity.downstream")},
      {"selectionIsDirectional", fl_value_new_bool(false)},
      {"composingBase", fl_value_new_int(state.composing_base)},
      {"composingExtent", fl_value_new_int(state.composing_extent)},
  });
}

struct EditingDelta {
  const gchar* old_text = "";
  const gchar* delta_text = "";
  int delta_start = -1;
  int delta_end = -1;
  int selection_base = -1;
  int selection_extent = -1;
  int composing_base = -1;
  int composing_extent = -1;
};

static FlValue* build_editing_delta(EditingDelta delta) {
  return build_map({
      {"oldText", fl_value_new_string(delta.old_text)},
      {"deltaText", fl_value_new_string(delta.delta_text)},
      {"deltaStart", fl_value_new_int(delta.delta_start)},
      {"deltaEnd", fl_value_new_int(delta.delta_end)},
      {"selectionBase", fl_value_new_int(delta.selection_base)},
      {"selectionExtent", fl_value_new_int(delta.selection_extent)},
      {"selectionAffinity", fl_value_new_string("TextAffinity.downstream")},
      {"selectionIsDirectional", fl_value_new_bool(false)},
      {"composingBase", fl_value_new_int(delta.composing_base)},
      {"composingExtent", fl_value_new_int(delta.composing_extent)},
  });
}

static void set_client(FlMockBinaryMessenger* messenger, InputConfig config) {
  gboolean called = FALSE;
  g_autoptr(FlValue) args = build_input_config(config);
  fl_mock_binary_messenger_invoke_json_method(
      messenger, "flutter/textinput", "TextInput.setClient", args,
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
}

static void set_editing_state(FlMockBinaryMessenger* messenger,
                              EditingState state) {
  gboolean called = FALSE;
  g_autoptr(FlValue) args = build_editing_state(state);
  fl_mock_binary_messenger_invoke_json_method(
      messenger, "flutter/textinput", "TextInput.setEditingState", args,
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
}

static void send_key_event(FlTextInputHandler* handler,
                           gint keyval,
                           gint state = 0) {
  GdkEvent* gdk_event = gdk_event_new(GDK_KEY_PRESS);
  gdk_event->key.keyval = keyval;
  gdk_event->key.state = state;
  g_autoptr(FlKeyEvent) key_event = fl_key_event_new_from_gdk_event(gdk_event);
  fl_text_input_handler_filter_keypress(handler, key_event);
}

TEST(FlTextInputHandlerTest, MessageHandler) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  ::testing::NiceMock<flutter::testing::MockIMContext> context;
  ::testing::NiceMock<flutter::testing::MockTextInputViewDelegate> delegate;

  g_autoptr(FlTextInputHandler) handler = fl_text_input_handler_new(
      FL_BINARY_MESSENGER(messenger), context, delegate);
  EXPECT_NE(handler, nullptr);

  EXPECT_TRUE(
      fl_mock_binary_messenger_has_handler(messenger, "flutter/textinput"));

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlTextInputHandlerTest, SetClient) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  ::testing::NiceMock<flutter::testing::MockIMContext> context;
  ::testing::NiceMock<flutter::testing::MockTextInputViewDelegate> delegate;

  g_autoptr(FlTextInputHandler) handler = fl_text_input_handler_new(
      FL_BINARY_MESSENGER(messenger), context, delegate);
  EXPECT_NE(handler, nullptr);

  set_client(messenger, {.client_id = 1});

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlTextInputHandlerTest, Show) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  ::testing::NiceMock<flutter::testing::MockIMContext> context;
  ::testing::NiceMock<flutter::testing::MockTextInputViewDelegate> delegate;

  g_autoptr(FlTextInputHandler) handler = fl_text_input_handler_new(
      FL_BINARY_MESSENGER(messenger), context, delegate);
  EXPECT_NE(handler, nullptr);

  EXPECT_CALL(context,
              gtk_im_context_focus_in(::testing::Eq<GtkIMContext*>(context)));

  gboolean called = FALSE;
  fl_mock_binary_messenger_invoke_json_method(
      messenger, "flutter/textinput", "TextInput.show", nullptr,
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

TEST(FlTextInputHandlerTest, Hide) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  ::testing::NiceMock<flutter::testing::MockIMContext> context;
  ::testing::NiceMock<flutter::testing::MockTextInputViewDelegate> delegate;

  g_autoptr(FlTextInputHandler) handler = fl_text_input_handler_new(
      FL_BINARY_MESSENGER(messenger), context, delegate);
  EXPECT_NE(handler, nullptr);

  EXPECT_CALL(context,
              gtk_im_context_focus_out(::testing::Eq<GtkIMContext*>(context)));

  gboolean called = FALSE;
  fl_mock_binary_messenger_invoke_json_method(
      messenger, "flutter/textinput", "TextInput.hide", nullptr,
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

TEST(FlTextInputHandlerTest, ClearClient) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  ::testing::NiceMock<flutter::testing::MockIMContext> context;
  ::testing::NiceMock<flutter::testing::MockTextInputViewDelegate> delegate;

  g_autoptr(FlTextInputHandler) handler = fl_text_input_handler_new(
      FL_BINARY_MESSENGER(messenger), context, delegate);
  EXPECT_NE(handler, nullptr);

  gboolean called = FALSE;
  fl_mock_binary_messenger_invoke_json_method(
      messenger, "flutter/textinput", "TextInput.clearClient", nullptr,
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

TEST(FlTextInputHandlerTest, PerformAction) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  ::testing::NiceMock<flutter::testing::MockIMContext> context;
  ::testing::NiceMock<flutter::testing::MockTextInputViewDelegate> delegate;

  g_autoptr(FlTextInputHandler) handler = fl_text_input_handler_new(
      FL_BINARY_MESSENGER(messenger), context, delegate);
  EXPECT_NE(handler, nullptr);

  set_client(messenger, {
                            .client_id = 1,
                            .input_type = "TextInputType.multiline",
                            .input_action = "TextInputAction.newline",
                        });
  set_editing_state(messenger, {
                                   .text = "Flutter",
                                   .selection_base = 7,
                                   .selection_extent = 7,
                               });

  // Client will update editing state and perform action
  int call_count = 0;
  fl_mock_binary_messenger_set_json_method_channel(
      messenger, "flutter/textinput",
      [](FlMockBinaryMessenger* messenger, GTask* task, const gchar* name,
         FlValue* args, gpointer user_data) {
        int* call_count = static_cast<int*>(user_data);

        if (strcmp(name, "TextInputClient.updateEditingState") == 0) {
          g_autoptr(FlValue) expected_args = build_list({
              fl_value_new_int(1),  // client_id
              build_editing_state({
                  .text = "Flutter\n",
                  .selection_base = 8,
                  .selection_extent = 8,
              }),
          });
          EXPECT_TRUE(fl_value_equal(args, expected_args));
          EXPECT_EQ(*call_count, 0);
          (*call_count)++;
        } else if (strcmp(name, "TextInputClient.performAction") == 0) {
          g_autoptr(FlValue) expected_args = build_list({
              fl_value_new_int(1),  // client_id
              fl_value_new_string("TextInputAction.newline"),
          });
          EXPECT_TRUE(fl_value_equal(args, expected_args));
          EXPECT_EQ(*call_count, 1);
          (*call_count)++;
        }

        return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
      },
      &call_count);

  send_key_event(handler, GDK_KEY_Return);
  EXPECT_EQ(call_count, 2);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

// Regression test for https://github.com/flutter/flutter/issues/125879.
TEST(FlTextInputHandlerTest, MultilineWithSendAction) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  ::testing::NiceMock<flutter::testing::MockIMContext> context;
  ::testing::NiceMock<flutter::testing::MockTextInputViewDelegate> delegate;

  g_autoptr(FlTextInputHandler) handler = fl_text_input_handler_new(
      FL_BINARY_MESSENGER(messenger), context, delegate);
  EXPECT_NE(handler, nullptr);

  set_client(messenger, {
                            .client_id = 1,
                            .input_type = "TextInputType.multiline",
                            .input_action = "TextInputAction.send",
                        });
  set_editing_state(messenger, {
                                   .text = "Flutter",
                                   .selection_base = 7,
                                   .selection_extent = 7,
                               });

  // Because the input action is not set to TextInputAction.newline, the next
  // expected call is "TextInputClient.performAction". If the input action was
  // set to TextInputAction.newline the next call would be
  // "TextInputClient.updateEditingState" (this case is tested in the test named
  // 'PerformAction').
  int call_count = 0;
  fl_mock_binary_messenger_set_json_method_channel(
      messenger, "flutter/textinput",
      [](FlMockBinaryMessenger* messenger, GTask* task, const gchar* name,
         FlValue* args, gpointer user_data) {
        int* call_count = static_cast<int*>(user_data);

        EXPECT_STREQ(name, "TextInputClient.performAction");
        g_autoptr(FlValue) expected_args = nullptr;
        switch (*call_count) {
          case 0:
            // Perform action.
            expected_args = build_list({
                fl_value_new_int(1),  // client_id
                fl_value_new_string("TextInputAction.send"),
            });
            break;
          default:
            g_assert_not_reached();
            break;
        }
        EXPECT_TRUE(fl_value_equal(args, expected_args));
        (*call_count)++;

        return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
      },
      &call_count);

  send_key_event(handler, GDK_KEY_Return);
  EXPECT_EQ(call_count, 1);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlTextInputHandlerTest, MoveCursor) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  ::testing::NiceMock<flutter::testing::MockIMContext> context;
  ::testing::NiceMock<flutter::testing::MockTextInputViewDelegate> delegate;

  g_autoptr(FlTextInputHandler) handler = fl_text_input_handler_new(
      FL_BINARY_MESSENGER(messenger), context, delegate);
  EXPECT_NE(handler, nullptr);

  set_client(messenger, {.client_id = 1});
  set_editing_state(messenger, {
                                   .text = "Flutter",
                                   .selection_base = 4,
                                   .selection_extent = 4,
                               });

  int call_count = 0;
  fl_mock_binary_messenger_set_json_method_channel(
      messenger, "flutter/textinput",
      [](FlMockBinaryMessenger* messenger, GTask* task, const gchar* name,
         FlValue* args, gpointer user_data) {
        int* call_count = static_cast<int*>(user_data);

        EXPECT_STREQ(name, "TextInputClient.updateEditingState");
        g_autoptr(FlValue) expected_args = nullptr;
        switch (*call_count) {
          case 0:
            // move cursor to beginning
            expected_args = build_list({
                fl_value_new_int(1),  // client_id
                build_editing_state({
                    .text = "Flutter",
                    .selection_base = 0,
                    .selection_extent = 0,
                }),
            });
            break;
          case 1:
            // move cursor to end
            expected_args = build_list({
                fl_value_new_int(1),  // client_id
                build_editing_state({
                    .text = "Flutter",
                    .selection_base = 7,
                    .selection_extent = 7,
                }),
            });
            break;
          default:
            g_assert_not_reached();
            break;
        }
        EXPECT_TRUE(fl_value_equal(args, expected_args));
        (*call_count)++;

        return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
      },
      &call_count);

  send_key_event(handler, GDK_KEY_Home);
  send_key_event(handler, GDK_KEY_End);
  EXPECT_EQ(call_count, 2);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlTextInputHandlerTest, Select) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  ::testing::NiceMock<flutter::testing::MockIMContext> context;
  ::testing::NiceMock<flutter::testing::MockTextInputViewDelegate> delegate;

  g_autoptr(FlTextInputHandler) handler = fl_text_input_handler_new(
      FL_BINARY_MESSENGER(messenger), context, delegate);
  EXPECT_NE(handler, nullptr);

  set_client(messenger, {.client_id = 1});
  set_editing_state(messenger, {
                                   .text = "Flutter",
                                   .selection_base = 4,
                                   .selection_extent = 4,
                               });

  int call_count = 0;
  fl_mock_binary_messenger_set_json_method_channel(
      messenger, "flutter/textinput",
      [](FlMockBinaryMessenger* messenger, GTask* task, const gchar* name,
         FlValue* args, gpointer user_data) {
        int* call_count = static_cast<int*>(user_data);

        EXPECT_STREQ(name, "TextInputClient.updateEditingState");
        g_autoptr(FlValue) expected_args = nullptr;
        switch (*call_count) {
          case 0:
            // select to end
            expected_args = build_list({
                fl_value_new_int(1),  // client_id
                build_editing_state({
                    .text = "Flutter",
                    .selection_base = 4,
                    .selection_extent = 7,
                }),
            });
            break;
          case 1:
            // select to beginning
            expected_args = build_list({
                fl_value_new_int(1),  // client_id
                build_editing_state({
                    .text = "Flutter",
                    .selection_base = 4,
                    .selection_extent = 0,
                }),
            });
            break;
          default:
            g_assert_not_reached();
            break;
        }
        EXPECT_TRUE(fl_value_equal(args, expected_args));
        (*call_count)++;

        return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
      },
      &call_count);

  send_key_event(handler, GDK_KEY_End, GDK_SHIFT_MASK);
  send_key_event(handler, GDK_KEY_Home, GDK_SHIFT_MASK);
  EXPECT_EQ(call_count, 2);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlTextInputHandlerTest, Composing) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  ::testing::NiceMock<flutter::testing::MockIMContext> context;
  ::testing::NiceMock<flutter::testing::MockTextInputViewDelegate> delegate;

  g_autoptr(FlTextInputHandler) handler = fl_text_input_handler_new(
      FL_BINARY_MESSENGER(messenger), context, delegate);
  EXPECT_NE(handler, nullptr);

  // update
  EXPECT_CALL(context,
              gtk_im_context_get_preedit_string(
                  ::testing::Eq<GtkIMContext*>(context),
                  ::testing::A<gchar**>(), ::testing::_, ::testing::A<gint*>()))
      .WillOnce(
          ::testing::DoAll(::testing::SetArgPointee<1>(g_strdup("Flutter")),
                           ::testing::SetArgPointee<3>(0)));

  int call_count = 0;
  fl_mock_binary_messenger_set_json_method_channel(
      messenger, "flutter/textinput",
      [](FlMockBinaryMessenger* messenger, GTask* task, const gchar* name,
         FlValue* args, gpointer user_data) {
        int* call_count = static_cast<int*>(user_data);

        EXPECT_STREQ(name, "TextInputClient.updateEditingState");
        g_autoptr(FlValue) expected_args = nullptr;
        switch (*call_count) {
          case 0:
            expected_args = build_list({
                fl_value_new_int(-1),  // client_id
                build_editing_state({
                    .text = "Flutter",
                    .selection_base = 0,
                    .selection_extent = 0,
                    .composing_base = 0,
                    .composing_extent = 7,
                }),
            });
            break;
          case 1:
            // commit
            expected_args = build_list({
                fl_value_new_int(-1),  // client_id
                build_editing_state({
                    .text = "engine",
                    .selection_base = 6,
                    .selection_extent = 6,
                }),
            });
            break;
          case 2:
            // end
            expected_args = build_list({
                fl_value_new_int(-1),  // client_id
                build_editing_state({
                    .text = "engine",
                    .selection_base = 6,
                    .selection_extent = 6,
                }),
            });
            break;
          default:
            g_assert_not_reached();
            break;
        }
        EXPECT_TRUE(fl_value_equal(args, expected_args));
        (*call_count)++;

        return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
      },
      &call_count);

  g_signal_emit_by_name(context, "preedit-start", nullptr);
  g_signal_emit_by_name(context, "preedit-changed", nullptr);
  g_signal_emit_by_name(context, "commit", "engine", nullptr);
  g_signal_emit_by_name(context, "preedit-end", nullptr);
  EXPECT_EQ(call_count, 3);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlTextInputHandlerTest, SurroundingText) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  ::testing::NiceMock<flutter::testing::MockIMContext> context;
  ::testing::NiceMock<flutter::testing::MockTextInputViewDelegate> delegate;

  g_autoptr(FlTextInputHandler) handler = fl_text_input_handler_new(
      FL_BINARY_MESSENGER(messenger), context, delegate);
  EXPECT_NE(handler, nullptr);

  set_client(messenger, {.client_id = 1});
  set_editing_state(messenger, {
                                   .text = "Flutter",
                                   .selection_base = 3,
                                   .selection_extent = 3,
                               });

  // retrieve
  EXPECT_CALL(context, gtk_im_context_set_surrounding(
                           ::testing::Eq<GtkIMContext*>(context),
                           ::testing::StrEq("Flutter"), 7, 3));

  gboolean retrieved = false;
  g_signal_emit_by_name(context, "retrieve-surrounding", &retrieved, nullptr);
  EXPECT_TRUE(retrieved);

  int call_count = 0;
  fl_mock_binary_messenger_set_json_method_channel(
      messenger, "flutter/textinput",
      [](FlMockBinaryMessenger* messenger, GTask* task, const gchar* name,
         FlValue* args, gpointer user_data) {
        int* call_count = static_cast<int*>(user_data);

        EXPECT_STREQ(name, "TextInputClient.updateEditingState");
        g_autoptr(FlValue) expected_args = nullptr;
        switch (*call_count) {
          case 0:
            // delete
            expected_args = build_list({
                fl_value_new_int(1),  // client_id
                build_editing_state({
                    .text = "Flutr",
                    .selection_base = 3,
                    .selection_extent = 3,
                }),
            });
            break;
          default:
            g_assert_not_reached();
            break;
        }
        EXPECT_TRUE(fl_value_equal(args, expected_args));
        (*call_count)++;

        return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
      },
      &call_count);

  gboolean deleted = false;
  g_signal_emit_by_name(context, "delete-surrounding", 1, 2, &deleted, nullptr);
  EXPECT_TRUE(deleted);
  EXPECT_EQ(call_count, 1);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlTextInputHandlerTest, SetMarkedTextRect) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  ::testing::NiceMock<flutter::testing::MockIMContext> context;
  ::testing::NiceMock<flutter::testing::MockTextInputViewDelegate> delegate;

  g_autoptr(FlTextInputHandler) handler = fl_text_input_handler_new(
      FL_BINARY_MESSENGER(messenger), context, delegate);
  EXPECT_NE(handler, nullptr);

  g_signal_emit_by_name(context, "preedit-start", nullptr);

  // set editable size and transform
  g_autoptr(FlValue) size_and_transform = build_map({
      {
          "transform",
          build_list({
              fl_value_new_float(1),
              fl_value_new_float(2),
              fl_value_new_float(3),
              fl_value_new_float(4),
              fl_value_new_float(5),
              fl_value_new_float(6),
              fl_value_new_float(7),
              fl_value_new_float(8),
              fl_value_new_float(9),
              fl_value_new_float(10),
              fl_value_new_float(11),
              fl_value_new_float(12),
              fl_value_new_float(13),
              fl_value_new_float(14),
              fl_value_new_float(15),
              fl_value_new_float(16),
          }),
      },
  });
  gboolean called = FALSE;
  fl_mock_binary_messenger_invoke_json_method(
      messenger, "flutter/textinput", "TextInput.setEditableSizeAndTransform",
      size_and_transform,
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

  EXPECT_CALL(delegate, fl_text_input_view_delegate_translate_coordinates(
                            ::testing::Eq<FlTextInputViewDelegate*>(delegate),
                            ::testing::Eq(27), ::testing::Eq(32), ::testing::_,
                            ::testing::_))
      .WillOnce(::testing::DoAll(::testing::SetArgPointee<3>(123),
                                 ::testing::SetArgPointee<4>(456)));

  EXPECT_CALL(context, gtk_im_context_set_cursor_location(
                           ::testing::Eq<GtkIMContext*>(context),
                           ::testing::Pointee(::testing::AllOf(
                               ::testing::Field(&GdkRectangle::x, 123),
                               ::testing::Field(&GdkRectangle::y, 456),
                               ::testing::Field(&GdkRectangle::width, 0),
                               ::testing::Field(&GdkRectangle::height, 0)))));

  // set marked text rect
  g_autoptr(FlValue) rect = build_map({
      {"x", fl_value_new_float(1)},
      {"y", fl_value_new_float(2)},
      {"width", fl_value_new_float(3)},
      {"height", fl_value_new_float(4)},
  });
  called = FALSE;
  fl_mock_binary_messenger_invoke_json_method(
      messenger, "flutter/textinput", "TextInput.setMarkedTextRect", rect,
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

TEST(FlTextInputHandlerTest, TextInputTypeNone) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  ::testing::NiceMock<flutter::testing::MockIMContext> context;
  ::testing::NiceMock<flutter::testing::MockTextInputViewDelegate> delegate;

  g_autoptr(FlTextInputHandler) handler = fl_text_input_handler_new(
      FL_BINARY_MESSENGER(messenger), context, delegate);
  EXPECT_NE(handler, nullptr);

  set_client(messenger, {
                            .client_id = 1,
                            .input_type = "TextInputType.none",
                        });

  EXPECT_CALL(context,
              gtk_im_context_focus_in(::testing::Eq<GtkIMContext*>(context)))
      .Times(0);
  EXPECT_CALL(context,
              gtk_im_context_focus_out(::testing::Eq<GtkIMContext*>(context)));

  gboolean called = FALSE;
  fl_mock_binary_messenger_invoke_json_method(
      messenger, "flutter/textinput", "TextInput.show", nullptr,
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

TEST(FlTextInputHandlerTest, TextEditingDelta) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  ::testing::NiceMock<flutter::testing::MockIMContext> context;
  ::testing::NiceMock<flutter::testing::MockTextInputViewDelegate> delegate;

  g_autoptr(FlTextInputHandler) handler = fl_text_input_handler_new(
      FL_BINARY_MESSENGER(messenger), context, delegate);
  EXPECT_NE(handler, nullptr);

  set_client(messenger, {
                            .client_id = 1,
                            .enable_delta_model = true,
                        });
  set_editing_state(messenger, {
                                   .text = "Flutter",
                                   .selection_base = 7,
                                   .selection_extent = 7,
                               });

  // update editing state with deltas
  int call_count = 0;
  fl_mock_binary_messenger_set_json_method_channel(
      messenger, "flutter/textinput",
      [](FlMockBinaryMessenger* messenger, GTask* task, const gchar* name,
         FlValue* args, gpointer user_data) {
        int* call_count = static_cast<int*>(user_data);

        EXPECT_STREQ(name, "TextInputClient.updateEditingStateWithDeltas");
        g_autoptr(FlValue) expected_args = nullptr;
        switch (*call_count) {
          case 0:
            expected_args = build_list({
                fl_value_new_int(1),  // client_id
                build_map({{
                    "deltas",
                    build_list({
                        build_editing_delta({
                            .old_text = "Flutter",
                            .delta_text = "Flutter",
                            .delta_start = 7,
                            .delta_end = 7,
                            .selection_base = 0,
                            .selection_extent = 0,
                        }),
                    }),
                }}),
            });
            break;
          default:
            g_assert_not_reached();
            break;
        }
        EXPECT_TRUE(fl_value_equal(args, expected_args));
        (*call_count)++;

        return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
      },
      &call_count);

  send_key_event(handler, GDK_KEY_Home);
  EXPECT_EQ(call_count, 1);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlTextInputHandlerTest, ComposingDelta) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  ::testing::NiceMock<flutter::testing::MockIMContext> context;
  ::testing::NiceMock<flutter::testing::MockTextInputViewDelegate> delegate;

  g_autoptr(FlTextInputHandler) handler = fl_text_input_handler_new(
      FL_BINARY_MESSENGER(messenger), context, delegate);
  EXPECT_NE(handler, nullptr);

  // set config
  set_client(messenger, {
                            .client_id = 1,
                            .enable_delta_model = true,
                        });

  g_signal_emit_by_name(context, "preedit-start", nullptr);

  // update
  EXPECT_CALL(context,
              gtk_im_context_get_preedit_string(
                  ::testing::Eq<GtkIMContext*>(context),
                  ::testing::A<gchar**>(), ::testing::_, ::testing::A<gint*>()))
      .WillOnce(
          ::testing::DoAll(::testing::SetArgPointee<1>(g_strdup("Flutter ")),
                           ::testing::SetArgPointee<3>(8)));

  int call_count = 0;
  fl_mock_binary_messenger_set_json_method_channel(
      messenger, "flutter/textinput",
      [](FlMockBinaryMessenger* messenger, GTask* task, const gchar* name,
         FlValue* args, gpointer user_data) {
        int* call_count = static_cast<int*>(user_data);

        EXPECT_STREQ(name, "TextInputClient.updateEditingStateWithDeltas");
        g_autoptr(FlValue) expected_args = nullptr;
        switch (*call_count) {
          case 0:
            expected_args = build_list({
                fl_value_new_int(1),  // client_id
                build_map({{
                    "deltas",
                    build_list({
                        build_editing_delta({
                            .old_text = "",
                            .delta_text = "Flutter ",
                            .delta_start = 0,
                            .delta_end = 0,
                            .selection_base = 8,
                            .selection_extent = 8,
                            .composing_base = 0,
                            .composing_extent = 8,
                        }),
                    }),
                }}),
            });
            break;
          case 1:
            // commit
            expected_args = build_list({
                fl_value_new_int(1),  // client_id
                build_map({{
                    "deltas",
                    build_list({
                        build_editing_delta({
                            .old_text = "Flutter ",
                            .delta_text = "Flutter engine",
                            .delta_start = 0,
                            .delta_end = 8,
                            .selection_base = 14,
                            .selection_extent = 14,
                            .composing_base = -1,
                            .composing_extent = -1,
                        }),
                    }),
                }}),
            });
            break;
          case 2:
            // end
            expected_args = build_list({
                fl_value_new_int(1),  // client_id
                build_map({{
                    "deltas",
                    build_list({
                        build_editing_delta({
                            .old_text = "Flutter engine",
                            .selection_base = 14,
                            .selection_extent = 14,
                        }),
                    }),
                }}),
            });
            break;
          default:
            g_assert_not_reached();
            break;
        }
        EXPECT_TRUE(fl_value_equal(args, expected_args));
        (*call_count)++;

        return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
      },
      &call_count);

  g_signal_emit_by_name(context, "preedit-changed", nullptr);
  g_signal_emit_by_name(context, "commit", "Flutter engine", nullptr);
  g_signal_emit_by_name(context, "preedit-end", nullptr);
  EXPECT_EQ(call_count, 3);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}

TEST(FlTextInputHandlerTest, NonComposingDelta) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  ::testing::NiceMock<flutter::testing::MockIMContext> context;
  ::testing::NiceMock<flutter::testing::MockTextInputViewDelegate> delegate;

  g_autoptr(FlTextInputHandler) handler = fl_text_input_handler_new(
      FL_BINARY_MESSENGER(messenger), context, delegate);
  EXPECT_NE(handler, nullptr);

  // set config
  set_client(messenger, {
                            .client_id = 1,
                            .enable_delta_model = true,
                        });

  int call_count = 0;
  fl_mock_binary_messenger_set_json_method_channel(
      messenger, "flutter/textinput",
      [](FlMockBinaryMessenger* messenger, GTask* task, const gchar* name,
         FlValue* args, gpointer user_data) {
        int* call_count = static_cast<int*>(user_data);

        EXPECT_STREQ(name, "TextInputClient.updateEditingStateWithDeltas");
        g_autoptr(FlValue) expected_args = nullptr;
        switch (*call_count) {
          case 0:
            // commit F
            expected_args = build_list({
                fl_value_new_int(1),  // client_id
                build_map({{
                    "deltas",
                    build_list({
                        build_editing_delta({
                            .old_text = "",
                            .delta_text = "F",
                            .delta_start = 0,
                            .delta_end = 0,
                            .selection_base = 1,
                            .selection_extent = 1,
                            .composing_base = -1,
                            .composing_extent = -1,
                        }),
                    }),
                }}),
            });
            break;
          case 1:
            // commit l
            expected_args = build_list({
                fl_value_new_int(1),  // client_id
                build_map({{
                    "deltas",
                    build_list({
                        build_editing_delta({
                            .old_text = "F",
                            .delta_text = "l",
                            .delta_start = 1,
                            .delta_end = 1,
                            .selection_base = 2,
                            .selection_extent = 2,
                            .composing_base = -1,
                            .composing_extent = -1,
                        }),
                    }),
                }}),
            });
            break;
          case 2:
            // commit u
            expected_args = build_list({
                fl_value_new_int(1),  // client_id
                build_map({{
                    "deltas",
                    build_list({
                        build_editing_delta({
                            .old_text = "Fl",
                            .delta_text = "u",
                            .delta_start = 2,
                            .delta_end = 2,
                            .selection_base = 3,
                            .selection_extent = 3,
                            .composing_base = -1,
                            .composing_extent = -1,
                        }),
                    }),
                }}),
            });
            break;
          case 3:
            // commit t
            expected_args = build_list({
                fl_value_new_int(1),  // client_id
                build_map({{
                    "deltas",
                    build_list({
                        build_editing_delta({
                            .old_text = "Flu",
                            .delta_text = "t",
                            .delta_start = 3,
                            .delta_end = 3,
                            .selection_base = 4,
                            .selection_extent = 4,
                            .composing_base = -1,
                            .composing_extent = -1,
                        }),
                    }),
                }}),
            });
            break;
          case 4:
            // commit t again
            expected_args = build_list({
                fl_value_new_int(1),  // client_id
                build_map({{
                    "deltas",
                    build_list({
                        build_editing_delta({
                            .old_text = "Flut",
                            .delta_text = "t",
                            .delta_start = 4,
                            .delta_end = 4,
                            .selection_base = 5,
                            .selection_extent = 5,
                            .composing_base = -1,
                            .composing_extent = -1,
                        }),
                    }),
                }}),
            });
            break;
          case 5:
            // commit e
            expected_args = build_list({
                fl_value_new_int(1),  // client_id
                build_map({{
                    "deltas",
                    build_list({
                        build_editing_delta({
                            .old_text = "Flutt",
                            .delta_text = "e",
                            .delta_start = 5,
                            .delta_end = 5,
                            .selection_base = 6,
                            .selection_extent = 6,
                            .composing_base = -1,
                            .composing_extent = -1,
                        }),
                    }),
                }}),
            });
            break;
          case 6:
            // commit r
            expected_args = build_list({
                fl_value_new_int(1),  // client_id
                build_map({{
                    "deltas",
                    build_list({
                        build_editing_delta({
                            .old_text = "Flutte",
                            .delta_text = "r",
                            .delta_start = 6,
                            .delta_end = 6,
                            .selection_base = 7,
                            .selection_extent = 7,
                            .composing_base = -1,
                            .composing_extent = -1,
                        }),
                    }),
                }}),
            });
            break;
          default:
            g_assert_not_reached();
            break;
        }
        EXPECT_TRUE(fl_value_equal(args, expected_args));
        (*call_count)++;

        return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
      },
      &call_count);

  g_signal_emit_by_name(context, "commit", "F", nullptr);
  g_signal_emit_by_name(context, "commit", "l", nullptr);
  g_signal_emit_by_name(context, "commit", "u", nullptr);
  g_signal_emit_by_name(context, "commit", "t", nullptr);
  g_signal_emit_by_name(context, "commit", "t", nullptr);
  g_signal_emit_by_name(context, "commit", "e", nullptr);
  g_signal_emit_by_name(context, "commit", "r", nullptr);
  EXPECT_EQ(call_count, 7);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}
