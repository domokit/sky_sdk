// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
#include "flutter/shell/platform/windows/keyboard_key_handler.h"

#include <rapidjson/document.h>
#include <map>
#include <memory>
#include "flutter/shell/platform/common/client_wrapper/include/flutter/method_result_functions.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/standard_message_codec.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/standard_method_codec.h"
#include "flutter/shell/platform/embedder/test_utils/key_codes.g.h"
#include "flutter/shell/platform/embedder/test_utils/proc_table_replacement.h"
#include "flutter/shell/platform/windows/keyboard_utils.h"
#include "flutter/shell/platform/windows/testing/engine_modifier.h"
#include "flutter/shell/platform/windows/testing/test_binary_messenger.h"

#include "flutter/fml/macros.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

namespace {

static constexpr char kChannelName[] = "flutter/keyboard";
static constexpr char kGetKeyboardStateMethod[] = "getKeyboardState";

constexpr SHORT kStateMaskToggled = 0x01;
constexpr SHORT kStateMaskPressed = 0x80;

class TestFlutterKeyEvent : public FlutterKeyEvent {
 public:
  TestFlutterKeyEvent(const FlutterKeyEvent& src,
                      FlutterKeyEventCallback callback,
                      void* user_data)
      : character_str(src.character), callback(callback), user_data(user_data) {
    struct_size = src.struct_size;
    timestamp = src.timestamp;
    type = src.type;
    physical = src.physical;
    logical = src.logical;
    character = character_str.c_str();
    synthesized = src.synthesized;
  }

  TestFlutterKeyEvent(TestFlutterKeyEvent&& source)
      : FlutterKeyEvent(source),
        callback(std::move(source.callback)),
        user_data(source.user_data) {
    character = character_str.c_str();
  }

  FlutterKeyEventCallback callback;
  void* user_data;

 private:
  const std::string character_str;
};

class TestKeystate {
 public:
  void Set(int virtual_key, bool pressed, bool toggled_on = false) {
    state_[virtual_key] = (pressed ? kStateMaskPressed : 0) |
                          (toggled_on ? kStateMaskToggled : 0);
  }

  SHORT Get(int virtual_key) { return state_[virtual_key]; }

  KeyboardKeyEmbedderHandler::GetKeyStateHandler Getter() {
    return [this](int virtual_key) { return Get(virtual_key); };
  }

 private:
  std::map<int, SHORT> state_;
};

UINT DefaultMapVkToScan(UINT virtual_key, bool extended) {
  return MapVirtualKey(virtual_key,
                       extended ? MAPVK_VK_TO_VSC_EX : MAPVK_VK_TO_VSC);
}

static constexpr int kHandledScanCode = 20;
static constexpr int kHandledScanCode2 = 22;
static constexpr int kUnhandledScanCode = 21;

constexpr uint64_t kScanCodeShiftRight = 0x36;
constexpr uint64_t kScanCodeControl = 0x1D;
constexpr uint64_t kScanCodeAltLeft = 0x38;

constexpr uint64_t kScanCodeKeyA = 0x1e;
constexpr uint64_t kVirtualKeyA = 0x41;

typedef std::function<void(bool)> Callback;
typedef std::function<void(Callback&)> CallbackHandler;
void dont_respond(Callback& callback) {}
void respond_true(Callback& callback) {
  callback(true);
}
void respond_false(Callback& callback) {
  callback(false);
}

// A testing |KeyHandlerDelegate| that records all calls
// to |KeyboardHook| and can be customized with whether
// the hook is handled in async.
class MockKeyHandlerDelegate
    : public KeyboardKeyHandler::KeyboardKeyHandlerDelegate {
 public:
  class KeyboardHookCall {
   public:
    int delegate_id;
    int key;
    int scancode;
    int action;
    char32_t character;
    bool extended;
    bool was_down;
    std::function<void(bool)> callback;
  };

  // Create a |MockKeyHandlerDelegate|.
  //
  // The |delegate_id| is an arbitrary ID to tell between delegates
  // that will be recorded in |KeyboardHookCall|.
  //
  // The |hook_history| will store every call to |KeyboardHookCall| that are
  // handled asynchronously.
  //
  // The |is_async| is a function that the class calls upon every
  // |KeyboardHookCall| to decide whether the call is handled asynchronously.
  // Defaults to always returning true (async).
  MockKeyHandlerDelegate(int delegate_id,
                         std::list<KeyboardHookCall>* hook_history)
      : delegate_id(delegate_id),
        hook_history(hook_history),
        callback_handler(dont_respond) {}
  virtual ~MockKeyHandlerDelegate() = default;

  virtual void KeyboardHook(int key,
                            int scancode,
                            int action,
                            char32_t character,
                            bool extended,
                            bool was_down,
                            std::function<void(bool)> callback) {
    hook_history->push_back(KeyboardHookCall{
        .delegate_id = delegate_id,
        .key = key,
        .scancode = scancode,
        .character = character,
        .extended = extended,
        .was_down = was_down,
        .callback = std::move(callback),
    });
    callback_handler(hook_history->back().callback);
  }

  virtual void SyncModifiersIfNeeded(int modifiers_state) {
    // Do Nothing
  }

  virtual std::map<uint64_t, uint64_t> GetPressedState() {
    std::map<uint64_t, uint64_t> Empty_State;
    return Empty_State;
  }

  CallbackHandler callback_handler;
  int delegate_id;
  std::list<KeyboardHookCall>* hook_history;

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(MockKeyHandlerDelegate);
};

enum KeyEventResponse {
  kNoResponse,
  kHandled,
  kUnhandled,
};

static KeyEventResponse key_event_response = kNoResponse;

void OnKeyEventResult(bool handled) {
  key_event_response = handled ? kHandled : kUnhandled;
}

void SimulateKeyboardMessage(TestBinaryMessenger* messenger,
                             const std::string& method_name,
                             std::unique_ptr<EncodableValue> arguments,
                             MethodResult<EncodableValue>* result_handler) {
  MethodCall<> call(method_name, std::move(arguments));
  auto message = StandardMethodCodec::GetInstance().EncodeMethodCall(call);

  EXPECT_TRUE(messenger->SimulateEngineMessage(
      kChannelName, message->data(), message->size(),
      [&result_handler](const uint8_t* reply, size_t reply_size) {
        StandardMethodCodec::GetInstance().DecodeAndProcessResponseEnvelope(
            reply, reply_size, result_handler);
      }));
}

}  // namespace

using namespace ::flutter::testing::keycodes;

TEST(KeyboardKeyHandlerTest, SingleDelegateWithAsyncResponds) {
  std::list<MockKeyHandlerDelegate::KeyboardHookCall> hook_history;

  TestBinaryMessenger messenger([](const std::string& channel,
                                   const uint8_t* message, size_t message_size,
                                   BinaryReply reply) {});
  KeyboardKeyHandler handler(&messenger);

  // Add one delegate
  auto delegate = std::make_unique<MockKeyHandlerDelegate>(1, &hook_history);
  handler.AddDelegate(std::move(delegate));

  /// Test 1: One event that is handled by the framework

  // Dispatch a key event
  handler.KeyboardHook(64, kHandledScanCode, WM_KEYDOWN, L'a', false, true,
                       OnKeyEventResult);
  EXPECT_EQ(key_event_response, kNoResponse);
  EXPECT_EQ(hook_history.size(), 1);
  EXPECT_EQ(hook_history.back().delegate_id, 1);
  EXPECT_EQ(hook_history.back().scancode, kHandledScanCode);
  EXPECT_EQ(hook_history.back().was_down, true);

  EXPECT_EQ(key_event_response, kNoResponse);
  hook_history.back().callback(true);
  EXPECT_EQ(key_event_response, kHandled);

  key_event_response = kNoResponse;
  hook_history.clear();

  /// Test 2: Two events that are unhandled by the framework

  handler.KeyboardHook(64, kHandledScanCode, WM_KEYDOWN, L'a', false, false,
                       OnKeyEventResult);
  EXPECT_EQ(key_event_response, kNoResponse);
  EXPECT_EQ(hook_history.size(), 1);
  EXPECT_EQ(hook_history.back().delegate_id, 1);
  EXPECT_EQ(hook_history.back().scancode, kHandledScanCode);
  EXPECT_EQ(hook_history.back().was_down, false);

  // Dispatch another key event
  handler.KeyboardHook(65, kHandledScanCode2, WM_KEYUP, L'b', false, true,
                       OnKeyEventResult);
  EXPECT_EQ(key_event_response, kNoResponse);
  EXPECT_EQ(hook_history.size(), 2);
  EXPECT_EQ(hook_history.back().delegate_id, 1);
  EXPECT_EQ(hook_history.back().scancode, kHandledScanCode2);
  EXPECT_EQ(hook_history.back().was_down, true);

  // Resolve the second event first to test out-of-order response
  hook_history.back().callback(false);
  EXPECT_EQ(key_event_response, kUnhandled);
  key_event_response = kNoResponse;

  // Resolve the first event then
  hook_history.front().callback(false);
  EXPECT_EQ(key_event_response, kUnhandled);

  hook_history.clear();
  key_event_response = kNoResponse;
}

TEST(KeyboardKeyHandlerTest, SingleDelegateWithSyncResponds) {
  std::list<MockKeyHandlerDelegate::KeyboardHookCall> hook_history;

  TestBinaryMessenger messenger([](const std::string& channel,
                                   const uint8_t* message, size_t message_size,
                                   BinaryReply reply) {});
  KeyboardKeyHandler handler(&messenger);
  // Add one delegate
  auto delegate = std::make_unique<MockKeyHandlerDelegate>(1, &hook_history);
  CallbackHandler& delegate_handler = delegate->callback_handler;
  handler.AddDelegate(std::move(delegate));

  /// Test 1: One event that is handled by the framework

  // Dispatch a key event
  delegate_handler = respond_true;
  handler.KeyboardHook(64, kHandledScanCode, WM_KEYDOWN, L'a', false, false,
                       OnKeyEventResult);
  EXPECT_EQ(key_event_response, kHandled);
  EXPECT_EQ(hook_history.size(), 1);
  EXPECT_EQ(hook_history.back().delegate_id, 1);
  EXPECT_EQ(hook_history.back().scancode, kHandledScanCode);
  EXPECT_EQ(hook_history.back().was_down, false);

  hook_history.clear();

  /// Test 2: An event unhandled by the framework

  delegate_handler = respond_false;
  handler.KeyboardHook(64, kHandledScanCode, WM_KEYDOWN, L'a', false, false,
                       OnKeyEventResult);
  EXPECT_EQ(key_event_response, kUnhandled);
  EXPECT_EQ(hook_history.size(), 1);
  EXPECT_EQ(hook_history.back().delegate_id, 1);
  EXPECT_EQ(hook_history.back().scancode, kHandledScanCode);
  EXPECT_EQ(hook_history.back().was_down, false);

  hook_history.clear();
  key_event_response = kNoResponse;
}

TEST(KeyboardKeyHandlerTest, HandlerGetPressedState) {
  TestKeystate key_state;

  TestBinaryMessenger messenger([](const std::string& channel,
                                   const uint8_t* message, size_t message_size,
                                   BinaryReply reply) {});
  KeyboardKeyHandler handler(&messenger);

  std::unique_ptr<KeyboardKeyEmbedderHandler> embedder_handler =
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [](const FlutterKeyEvent& event, FlutterKeyEventCallback callback,
             void* user_data) {},
          key_state.Getter(), DefaultMapVkToScan);
  handler.AddDelegate(std::move(embedder_handler));

  // Dispatch a key event.
  handler.KeyboardHook(kVirtualKeyA, kScanCodeKeyA, WM_KEYDOWN, 'a', false,
                       false, OnKeyEventResult);

  std::map<uint64_t, uint64_t> pressed_state = handler.GetPressedState();
  EXPECT_EQ(pressed_state.size(), 1);
  EXPECT_EQ(pressed_state.at(kPhysicalKeyA), kLogicalKeyA);
}

TEST(KeyboardKeyHandlerTest, KeyboardChannelGetPressedState) {
  TestKeystate key_state;
  TestBinaryMessenger messenger;
  KeyboardKeyHandler handler(&messenger);

  std::unique_ptr<KeyboardKeyEmbedderHandler> embedder_handler =
      std::make_unique<KeyboardKeyEmbedderHandler>(
          [](const FlutterKeyEvent& event, FlutterKeyEventCallback callback,
             void* user_data) {},
          key_state.Getter(), DefaultMapVkToScan);
  handler.AddDelegate(std::move(embedder_handler));
  handler.InitKeyboardChannel();

  // Dispatch a key event.
  handler.KeyboardHook(kVirtualKeyA, kScanCodeKeyA, WM_KEYDOWN, 'a', false,
                       false, OnKeyEventResult);

  bool success = false;

  MethodResultFunctions<> result_handler(
      [&success](const EncodableValue* result) {
        success = true;
        auto& map = std::get<EncodableMap>(*result);
        EXPECT_EQ(map.size(), 1);
        EncodableValue physical_value(static_cast<long long>(kPhysicalKeyA));
        EncodableValue logical_value(static_cast<long long>(kLogicalKeyA));
        EXPECT_EQ(map.at(physical_value), logical_value);
      },
      nullptr, nullptr);

  SimulateKeyboardMessage(&messenger, kGetKeyboardStateMethod, nullptr,
                          &result_handler);
  EXPECT_TRUE(success);
}

}  // namespace testing
}  // namespace flutter
