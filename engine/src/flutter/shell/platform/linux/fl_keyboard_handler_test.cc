// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_keyboard_handler.h"

#include "flutter/shell/platform/linux/fl_binary_messenger_private.h"
#include "flutter/shell/platform/linux/fl_method_codec_private.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_standard_method_codec.h"
#include "flutter/shell/platform/linux/testing/fl_mock_binary_messenger.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

static constexpr char kKeyboardChannelName[] = "flutter/keyboard";
static constexpr char kGetKeyboardStateMethod[] = "getKeyboardState";
static constexpr uint64_t kMockPhysicalKey = 42;
static constexpr uint64_t kMockLogicalKey = 42;

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlMockKeyboardHandlerDelegate,
                     fl_mock_keyboard_handler_delegate,
                     FL,
                     MOCK_KEYBOARD_HANDLER_DELEGATE,
                     GObject);

G_END_DECLS

struct _FlMockKeyboardHandlerDelegate {
  GObject parent_instance;
};

static void fl_mock_keyboard_handler_delegate_keyboard_view_delegate_iface_init(
    FlKeyboardViewDelegateInterface* iface);

G_DEFINE_TYPE_WITH_CODE(
    FlMockKeyboardHandlerDelegate,
    fl_mock_keyboard_handler_delegate,
    G_TYPE_OBJECT,
    G_IMPLEMENT_INTERFACE(
        fl_keyboard_view_delegate_get_type(),
        fl_mock_keyboard_handler_delegate_keyboard_view_delegate_iface_init))

static void fl_mock_keyboard_handler_delegate_init(
    FlMockKeyboardHandlerDelegate* self) {}

static void fl_mock_keyboard_handler_delegate_class_init(
    FlMockKeyboardHandlerDelegateClass* klass) {}

static void fl_mock_keyboard_handler_delegate_keyboard_view_delegate_iface_init(
    FlKeyboardViewDelegateInterface* iface) {}

static FlMockKeyboardHandlerDelegate* fl_mock_keyboard_handler_delegate_new() {
  FlMockKeyboardHandlerDelegate* self = FL_MOCK_KEYBOARD_HANDLER_DELEGATE(
      g_object_new(fl_mock_keyboard_handler_delegate_get_type(), nullptr));

  // Added to stop compiler complaining about an unused function.
  FL_IS_MOCK_KEYBOARD_HANDLER_DELEGATE(self);

  return self;
}

TEST(FlKeyboardHandlerTest, KeyboardChannelGetPressedState) {
  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  g_autoptr(FlEngine) engine =
      FL_ENGINE(g_object_new(fl_engine_get_type(), "binary-messenger",
                             FL_BINARY_MESSENGER(messenger), nullptr));
  g_autoptr(FlMockKeyboardHandlerDelegate) view_delegate =
      fl_mock_keyboard_handler_delegate_new();
  g_autoptr(FlKeyboardManager) manager =
      fl_keyboard_manager_new(engine, FL_KEYBOARD_VIEW_DELEGATE(view_delegate));
  fl_keyboard_manager_set_get_pressed_state_handler(
      manager,
      [](gpointer user_data) {
        GHashTable* result = g_hash_table_new(g_direct_hash, g_direct_equal);
        g_hash_table_insert(result,
                            reinterpret_cast<gpointer>(kMockPhysicalKey),
                            reinterpret_cast<gpointer>(kMockLogicalKey));

        return result;
      },
      nullptr);
  g_autoptr(FlKeyboardHandler) handler =
      fl_keyboard_handler_new(FL_BINARY_MESSENGER(messenger), manager);
  EXPECT_NE(handler, nullptr);

  gboolean called = FALSE;
  fl_mock_binary_messenger_invoke_standard_method(
      messenger, kKeyboardChannelName, kGetKeyboardStateMethod, nullptr,
      [](FlMockBinaryMessenger* messenger, FlMethodResponse* response,
         gpointer user_data) {
        gboolean* called = static_cast<gboolean*>(user_data);
        *called = TRUE;

        EXPECT_TRUE(FL_IS_METHOD_SUCCESS_RESPONSE(response));

        g_autoptr(FlValue) expected_result = fl_value_new_map();
        fl_value_set_take(expected_result, fl_value_new_int(kMockPhysicalKey),
                          fl_value_new_int(kMockLogicalKey));
        EXPECT_TRUE(fl_value_equal(fl_method_success_response_get_result(
                                       FL_METHOD_SUCCESS_RESPONSE(response)),
                                   expected_result));
      },
      &called);
  EXPECT_TRUE(called);

  fl_binary_messenger_shutdown(FL_BINARY_MESSENGER(messenger));
}
