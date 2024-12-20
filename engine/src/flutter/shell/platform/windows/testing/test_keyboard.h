// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_TEST_KEYBOARD_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_TEST_KEYBOARD_H_

#include <windows.h>

#include <functional>
#include <string>

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/windows/testing/engine_modifier.h"
#include "flutter/shell/platform/windows/testing/wm_builders.h"

#include "gtest/gtest.h"

struct _FlutterPlatformMessageResponseHandle {
  FlutterDesktopBinaryReply callback;
  void* user_data;
};

namespace flutter {
namespace testing {
::testing::AssertionResult _EventEquals(const char* expr_event,
                                        const char* expr_expected,
                                        const FlutterKeyEvent& event,
                                        const FlutterKeyEvent& expected);

// Clone string onto the heap.
//
// If #string is nullptr, returns nullptr. Otherwise, the returned pointer must
// be freed with delete[].
char* clone_string(const char* string);

// Creates a valid Windows LPARAM for WM_KEYDOWN and WM_CHAR from parameters
// given.
//
// While |CreateKeyEventLparam| is flexible, it's recommended to use dedicated
// functions in wm_builders.h, such as |WmKeyDownInfo|.
LPARAM CreateKeyEventLparam(USHORT scancode,
                            bool extended,
                            bool was_down,
                            USHORT repeat_count = 1,
                            bool context_code = 0,
                            bool transition_state = 1);

class MockKeyResponseController {
 public:
  using ResponseCallback = std::function<void(bool)>;
  using EmbedderCallbackHandler =
      std::function<void(const FlutterKeyEvent*, ResponseCallback)>;
  using ChannelCallbackHandler = std::function<void(ResponseCallback)>;
  using TextInputCallbackHandler =
      std::function<void(std::unique_ptr<rapidjson::Document>)>;

  MockKeyResponseController()
      : channel_response_(ChannelRespondFalse),
        embedder_response_(EmbedderRespondFalse),
        text_input_response_(
            [](std::unique_ptr<rapidjson::Document> document) {}) {}

  void SetChannelResponse(ChannelCallbackHandler handler) {
    channel_response_ = std::move(handler);
  }

  void SetEmbedderResponse(EmbedderCallbackHandler handler) {
    embedder_response_ = std::move(handler);
  }

  void SetTextInputResponse(TextInputCallbackHandler handler) {
    text_input_response_ = std::move(handler);
  }

  void HandleChannelMessage(ResponseCallback callback) {
    channel_response_(callback);
  }

  void HandleEmbedderMessage(const FlutterKeyEvent* event,
                             ResponseCallback callback) {
    embedder_response_(event, std::move(callback));
  }

  void HandleTextInputMessage(std::unique_ptr<rapidjson::Document> document) {
    text_input_response_(std::move(document));
  }

 private:
  EmbedderCallbackHandler embedder_response_;
  ChannelCallbackHandler channel_response_;
  TextInputCallbackHandler text_input_response_;

  static void ChannelRespondFalse(ResponseCallback callback) {
    callback(false);
  }

  static void EmbedderRespondFalse(const FlutterKeyEvent* event,
                                   ResponseCallback callback) {
    callback(false);
  }

  FML_DISALLOW_COPY_AND_ASSIGN(MockKeyResponseController);
};

void MockEmbedderApiForKeyboard(
    EngineModifier& modifier,
    std::shared_ptr<MockKeyResponseController> response_controller);

// Simulate a message queue for WM messages.
//
// Subclasses must implement |Win32SendMessage| for how dispatched messages are
// processed.
class MockMessageQueue {
 protected:
  // Push a message to the message queue without dispatching it.
  void PushBack(const Win32Message* message);

  // Dispatch the first message of the message queue and return its result.
  //
  // This method asserts that the queue is not empty.
  LRESULT DispatchFront();

  // Peak the next message in the message queue.
  //
  // See Win32's |PeekMessage| for documentation.
  BOOL Win32PeekMessage(LPMSG lpMsg,
                        UINT wMsgFilterMin,
                        UINT wMsgFilterMax,
                        UINT wRemoveMsg);

  // Simulate dispatching a message to the system.
  virtual LRESULT Win32SendMessage(UINT const message,
                                   WPARAM const wparam,
                                   LPARAM const lparam) = 0;

  std::list<Win32Message> _pending_messages;
  std::list<Win32Message> _sent_messages;
};

}  // namespace testing
}  // namespace flutter

// Expect the |_target| FlutterKeyEvent has the required properties.
#define EXPECT_EVENT_EQUALS(_target, _type, _physical, _logical, _character, \
                            _synthesized)                                    \
  EXPECT_PRED_FORMAT2(                                                       \
      _EventEquals, _target,                                                 \
      (FlutterKeyEvent{                                                      \
          /* struct_size = */ sizeof(FlutterKeyEvent),                       \
          /* timestamp = */ 0,                                               \
          /* type = */ _type,                                                \
          /* physical = */ _physical,                                        \
          /* logical = */ _logical,                                          \
          /* character = */ _character,                                      \
          /* synthesized = */ _synthesized,                                  \
          /* device_type = */ kFlutterKeyEventDeviceTypeKeyboard,            \
      }));

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_TEST_KEYBOARD_H_
