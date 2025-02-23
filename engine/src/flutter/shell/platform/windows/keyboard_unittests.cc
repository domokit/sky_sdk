// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/logging.h"
#include "flutter/shell/platform/common/json_message_codec.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/embedder/test_utils/key_codes.g.h"
#include "flutter/shell/platform/windows/flutter_windows_engine.h"
#include "flutter/shell/platform/windows/flutter_windows_view.h"
#include "flutter/shell/platform/windows/keyboard_key_channel_handler.h"
#include "flutter/shell/platform/windows/keyboard_key_embedder_handler.h"
#include "flutter/shell/platform/windows/keyboard_key_handler.h"
#include "flutter/shell/platform/windows/keyboard_manager.h"
#include "flutter/shell/platform/windows/testing/engine_modifier.h"
#include "flutter/shell/platform/windows/testing/flutter_windows_engine_builder.h"
#include "flutter/shell/platform/windows/testing/mock_window_binding_handler.h"
#include "flutter/shell/platform/windows/testing/test_keyboard.h"
#include "flutter/shell/platform/windows/testing/windows_test.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

#include <functional>
#include <list>
#include <vector>

using testing::_;
using testing::Invoke;
using testing::Return;
using namespace ::flutter::testing::keycodes;

namespace flutter {
namespace testing {

namespace {

constexpr SHORT kStateMaskToggled = 0x01;
constexpr SHORT kStateMaskPressed = 0x80;

constexpr uint64_t kScanCodeBackquote = 0x29;
constexpr uint64_t kScanCodeKeyA = 0x1e;
constexpr uint64_t kScanCodeKeyB = 0x30;
constexpr uint64_t kScanCodeKeyE = 0x12;
constexpr uint64_t kScanCodeKeyF = 0x21;
constexpr uint64_t kScanCodeKeyO = 0x18;
constexpr uint64_t kScanCodeKeyQ = 0x10;
constexpr uint64_t kScanCodeKeyW = 0x11;
constexpr uint64_t kScanCodeDigit1 = 0x02;
constexpr uint64_t kScanCodeDigit2 = 0x03;
constexpr uint64_t kScanCodeDigit6 = 0x07;
// constexpr uint64_t kScanCodeNumpad1 = 0x4f;
// constexpr uint64_t kScanCodeNumLock = 0x45;
constexpr uint64_t kScanCodeControl = 0x1d;
constexpr uint64_t kScanCodeMetaLeft = 0x5b;
constexpr uint64_t kScanCodeMetaRight = 0x5c;
constexpr uint64_t kScanCodeAlt = 0x38;
constexpr uint64_t kScanCodeShiftLeft = 0x2a;
constexpr uint64_t kScanCodeShiftRight = 0x36;
constexpr uint64_t kScanCodeBracketLeft = 0x1a;
constexpr uint64_t kScanCodeArrowLeft = 0x4b;
constexpr uint64_t kScanCodeEnter = 0x1c;
constexpr uint64_t kScanCodeBackspace = 0x0e;

constexpr uint64_t kVirtualDigit1 = 0x31;
constexpr uint64_t kVirtualKeyA = 0x41;
constexpr uint64_t kVirtualKeyB = 0x42;
constexpr uint64_t kVirtualKeyE = 0x45;
constexpr uint64_t kVirtualKeyF = 0x46;
constexpr uint64_t kVirtualKeyO = 0x4f;
constexpr uint64_t kVirtualKeyQ = 0x51;
constexpr uint64_t kVirtualKeyW = 0x57;

constexpr bool kSynthesized = true;
constexpr bool kNotSynthesized = false;

typedef UINT (*MapVirtualKeyLayout)(UINT uCode, UINT uMapType);
typedef std::function<UINT(UINT)> MapVirtualKeyToChar;

UINT LayoutDefault(UINT uCode, UINT uMapType) {
  return MapVirtualKey(uCode, uMapType);
}

UINT LayoutFrench(UINT uCode, UINT uMapType) {
  switch (uMapType) {
    case MAPVK_VK_TO_CHAR:
      switch (uCode) {
        case 0xDD:
          return 0x8000005E;
        default:
          return MapVirtualKey(uCode, MAPVK_VK_TO_CHAR);
      }
    default:
      return MapVirtualKey(uCode, uMapType);
  }
}

class TestKeyboardManager : public KeyboardManager {
 public:
  explicit TestKeyboardManager(WindowDelegate* delegate)
      : KeyboardManager(delegate) {}

  bool DuringRedispatch() { return during_redispatch_; }

 protected:
  void RedispatchEvent(std::unique_ptr<PendingEvent> event) override {
    FML_DCHECK(!during_redispatch_)
        << "RedispatchEvent called while already redispatching an event";
    during_redispatch_ = true;
    KeyboardManager::RedispatchEvent(std::move(event));
    during_redispatch_ = false;
  }

 private:
  bool during_redispatch_ = false;

  FML_DISALLOW_COPY_AND_ASSIGN(TestKeyboardManager);
};

// Injecting this kind of keyboard change means that a key state (the true
// state for a key, typically a modifier) should be changed.
struct KeyStateChange {
  uint32_t key;
  bool pressed;
  bool toggled_on;
};

// Injecting this kind of keyboard change does not make any changes to the
// keyboard system, but indicates that a forged event is expected here, and
// that `KeyStateChange`s after this will be applied only after the forged
// event.
//
// See `IsKeyDownAltRight` for explaination for foged events.
struct ExpectForgedMessage {
  explicit ExpectForgedMessage(Win32Message message) : message(message){};

  Win32Message message;
};

struct KeyboardChange {
  // The constructors are intentionally for implicit conversion.

  KeyboardChange(Win32Message message) : type(kMessage) {
    content.message = message;
  }

  KeyboardChange(KeyStateChange change) : type(kKeyStateChange) {
    content.key_state_change = change;
  }

  KeyboardChange(ExpectForgedMessage forged_message)
      : type(kExpectForgedMessage) {
    content.expected_forged_message = forged_message.message;
  }

  enum Type {
    kMessage,
    kKeyStateChange,
    kExpectForgedMessage,
  } type;

  union {
    Win32Message message;
    KeyStateChange key_state_change;
    Win32Message expected_forged_message;
  } content;
};

class TestKeystate {
 public:
  void Set(uint32_t virtual_key, bool pressed, bool toggled_on = false) {
    state_[virtual_key] = (pressed ? kStateMaskPressed : 0) |
                          (toggled_on ? kStateMaskToggled : 0);
  }

  SHORT Get(uint32_t virtual_key) { return state_[virtual_key]; }

 private:
  std::map<uint32_t, SHORT> state_;
};

class MockKeyboardManagerDelegate : public KeyboardManager::WindowDelegate,
                                    protected MockMessageQueue {
 public:
  MockKeyboardManagerDelegate(WindowBindingHandlerDelegate* view,
                              MapVirtualKeyToChar map_vk_to_char)
      : view_(view), map_vk_to_char_(std::move(map_vk_to_char)) {
    keyboard_manager_ = std::make_unique<TestKeyboardManager>(this);
  }
  virtual ~MockKeyboardManagerDelegate() {}

  // |KeyboardManager::WindowDelegate|
  void OnKey(int key,
             int scancode,
             int action,
             char32_t character,
             bool extended,
             bool was_down,
             KeyEventCallback callback) override {
    view_->OnKey(key, scancode, action, character, extended, was_down,
                 callback);
  }

  // |KeyboardManager::WindowDelegate|
  void OnText(const std::u16string& text) override { view_->OnText(text); }

  SHORT GetKeyState(int virtual_key) { return key_state_.Get(virtual_key); }

  void InjectKeyboardChanges(std::vector<KeyboardChange> changes) {
    // First queue all messages to enable peeking.
    for (const KeyboardChange& change : changes) {
      switch (change.type) {
        case KeyboardChange::kMessage:
          PushBack(&change.content.message);
          break;
        default:
          break;
      }
    }
    for (const KeyboardChange& change : changes) {
      switch (change.type) {
        case KeyboardChange::kMessage:
          DispatchFront();
          break;
        case KeyboardChange::kExpectForgedMessage:
          forged_message_expectations_.push_back(ForgedMessageExpectation{
              .message = change.content.expected_forged_message,
          });
          break;
        case KeyboardChange::kKeyStateChange: {
          const KeyStateChange& state_change = change.content.key_state_change;
          if (forged_message_expectations_.empty()) {
            key_state_.Set(state_change.key, state_change.pressed,
                           state_change.toggled_on);
          } else {
            forged_message_expectations_.back()
                .state_changes_afterwards.push_back(state_change);
          }
          break;
        }
        default:
          FML_LOG(FATAL) << "Unhandled KeyboardChange type " << change.type;
      }
    }
  }

  std::list<Win32Message>& RedispatchedMessages() {
    return redispatched_messages_;
  }

 protected:
  BOOL Win32PeekMessage(LPMSG lpMsg,
                        UINT wMsgFilterMin,
                        UINT wMsgFilterMax,
                        UINT wRemoveMsg) override {
    return MockMessageQueue::Win32PeekMessage(lpMsg, wMsgFilterMin,
                                              wMsgFilterMax, wRemoveMsg);
  }

  uint32_t Win32MapVkToChar(uint32_t virtual_key) override {
    return map_vk_to_char_(virtual_key);
  }

  // This method is called for each message injected by test cases with
  // `tester.InjectMessages`.
  LRESULT Win32SendMessage(UINT const message,
                           WPARAM const wparam,
                           LPARAM const lparam) override {
    return keyboard_manager_->HandleMessage(message, wparam, lparam)
               ? 0
               : kWmResultDefault;
  }

  // This method is called when the keyboard manager redispatches messages
  // or forges messages (such as CtrlLeft up right before AltGr up).
  UINT Win32DispatchMessage(UINT Msg, WPARAM wParam, LPARAM lParam) override {
    bool handled = keyboard_manager_->HandleMessage(Msg, wParam, lParam);
    if (keyboard_manager_->DuringRedispatch()) {
      redispatched_messages_.push_back(Win32Message{
          .message = Msg,
          .wParam = wParam,
          .lParam = lParam,
      });
      EXPECT_FALSE(handled);
    } else {
      EXPECT_FALSE(forged_message_expectations_.empty());
      ForgedMessageExpectation expectation =
          forged_message_expectations_.front();
      forged_message_expectations_.pop_front();
      EXPECT_EQ(expectation.message.message, Msg);
      EXPECT_EQ(expectation.message.wParam, wParam);
      EXPECT_EQ(expectation.message.lParam, lParam);
      if (expectation.message.expected_result != kWmResultDontCheck) {
        EXPECT_EQ(expectation.message.expected_result,
                  handled ? kWmResultZero : kWmResultDefault);
      }
      for (const KeyStateChange& change :
           expectation.state_changes_afterwards) {
        key_state_.Set(change.key, change.pressed, change.toggled_on);
      }
    }
    return 0;
  }

 private:
  struct ForgedMessageExpectation {
    Win32Message message;
    std::list<KeyStateChange> state_changes_afterwards;
  };

  WindowBindingHandlerDelegate* view_;
  std::unique_ptr<TestKeyboardManager> keyboard_manager_;
  std::list<ForgedMessageExpectation> forged_message_expectations_;
  MapVirtualKeyToChar map_vk_to_char_;
  TestKeystate key_state_;
  std::list<Win32Message> redispatched_messages_;

  FML_DISALLOW_COPY_AND_ASSIGN(MockKeyboardManagerDelegate);
};

typedef struct {
  enum {
    kKeyCallOnKey,
    kKeyCallOnText,
    kKeyCallTextMethodCall,
  } type;

  // Only one of the following fields should be assigned.
  FlutterKeyEvent key_event;     // For kKeyCallOnKey
  std::u16string text;           // For kKeyCallOnText
  std::string text_method_call;  // For kKeyCallTextMethodCall
} KeyCall;

// A FlutterWindowsView that spies on text.
class TestFlutterWindowsView : public FlutterWindowsView {
 public:
  TestFlutterWindowsView(FlutterWindowsEngine* engine,
                         std::unique_ptr<WindowBindingHandler> window,
                         std::function<void(KeyCall)> on_key_call)
      : on_key_call_(on_key_call),
        FlutterWindowsView(kImplicitViewId, engine, std::move(window)) {}

  void OnText(const std::u16string& text) override {
    on_key_call_(KeyCall{
        .type = KeyCall::kKeyCallOnText,
        .text = text,
    });
  }

 private:
  std::function<void(KeyCall)> on_key_call_;

  FML_DISALLOW_COPY_AND_ASSIGN(TestFlutterWindowsView);
};

class KeyboardTester {
 public:
  using ResponseHandler =
      std::function<void(MockKeyResponseController::ResponseCallback)>;

  explicit KeyboardTester(WindowsTestContext& context)
      : callback_handler_(RespondValue(false)),
        map_virtual_key_layout_(LayoutDefault) {
    engine_ = GetTestEngine(context);
    view_ = std::make_unique<TestFlutterWindowsView>(
        engine_.get(),
        // The WindowBindingHandler is used for window size and such, and
        // doesn't affect keyboard.
        std::make_unique<::testing::NiceMock<MockWindowBindingHandler>>(),
        [this](KeyCall key_call) { key_calls.push_back(key_call); });

    EngineModifier modifier{engine_.get()};
    modifier.SetImplicitView(view_.get());
    modifier.InitializeKeyboard();

    window_ = std::make_unique<MockKeyboardManagerDelegate>(
        view_.get(), [this](UINT virtual_key) -> SHORT {
          return map_virtual_key_layout_(virtual_key, MAPVK_VK_TO_CHAR);
        });
  }

  TestFlutterWindowsView& GetView() { return *view_; }
  MockKeyboardManagerDelegate& GetWindow() { return *window_; }

  // Reset the keyboard by invoking the engine restart handler.
  void ResetKeyboard() { EngineModifier{engine_.get()}.Restart(); }

  // Set all events to be handled (true) or unhandled (false).
  void Responding(bool response) { callback_handler_ = RespondValue(response); }

  // Manually handle event callback of the onKeyData embedder API.
  //
  // On every onKeyData call, the |handler| will be invoked with the target
  // key data and the result callback. Immediately calling the callback with
  // a boolean is equivalent to setting |Responding| with the boolean. However,
  // |LateResponding| allows storing the callback to call later.
  void LateResponding(
      MockKeyResponseController::EmbedderCallbackHandler handler) {
    callback_handler_ = std::move(handler);
  }

  void SetLayout(MapVirtualKeyLayout layout) {
    map_virtual_key_layout_ = layout == nullptr ? LayoutDefault : layout;
  }

  void InjectKeyboardChanges(std::vector<KeyboardChange> changes) {
    FML_DCHECK(window_ != nullptr);
    window_->InjectKeyboardChanges(std::move(changes));
  }

  // Simulates receiving a platform message from the framework.
  void InjectPlatformMessage(const char* channel,
                             const char* method,
                             const char* args) {
    rapidjson::Document args_doc;
    args_doc.Parse(args);
    FML_DCHECK(!args_doc.HasParseError());

    rapidjson::Document message_doc(rapidjson::kObjectType);
    auto& allocator = message_doc.GetAllocator();
    message_doc.AddMember("method", rapidjson::Value(method, allocator),
                          allocator);
    message_doc.AddMember("args", args_doc, allocator);

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    message_doc.Accept(writer);

    std::unique_ptr<std::vector<uint8_t>> data =
        JsonMessageCodec::GetInstance().EncodeMessage(message_doc);
    FlutterPlatformMessageResponseHandle response_handle;
    const FlutterPlatformMessage message = {
        sizeof(FlutterPlatformMessage),  // struct_size
        channel,                         // channel
        data->data(),                    // message
        data->size(),                    // message_size
        &response_handle,                // response_handle
    };
    view_->GetEngine()->HandlePlatformMessage(&message);
  }

  // Get the number of redispatched messages since the last clear, then clear
  // the counter.
  size_t RedispatchedMessageCountAndClear() {
    auto& messages = window_->RedispatchedMessages();
    size_t count = messages.size();
    messages.clear();
    return count;
  }

  void clear_key_calls() {
    for (KeyCall& key_call : key_calls) {
      if (key_call.type == KeyCall::kKeyCallOnKey &&
          key_call.key_event.character != nullptr) {
        delete[] key_call.key_event.character;
      }
    }
    key_calls.clear();
  }

  std::vector<KeyCall> key_calls;

 private:
  std::unique_ptr<FlutterWindowsEngine> engine_;
  std::unique_ptr<TestFlutterWindowsView> view_;
  std::unique_ptr<MockKeyboardManagerDelegate> window_;
  MockKeyResponseController::EmbedderCallbackHandler callback_handler_;
  MapVirtualKeyLayout map_virtual_key_layout_;

  // Returns an engine instance configured with dummy project path values, and
  // overridden methods for sending platform messages, so that the engine can
  // respond as if the framework were connected.
  std::unique_ptr<FlutterWindowsEngine> GetTestEngine(
      WindowsTestContext& context) {
    FlutterWindowsEngineBuilder builder{context};

    builder.SetCreateKeyboardHandlerCallbacks(
        [this](int virtual_key) -> SHORT {
          // `window_` is not initialized yet when this callback is first
          // called.
          return window_ ? window_->GetKeyState(virtual_key) : 0;
        },
        [this](UINT virtual_key, bool extended) -> SHORT {
          return map_virtual_key_layout_(
              virtual_key, extended ? MAPVK_VK_TO_VSC_EX : MAPVK_VK_TO_VSC);
        });

    auto engine = builder.Build();

    EngineModifier modifier(engine.get());

    auto key_response_controller =
        std::make_shared<MockKeyResponseController>();
    key_response_controller->SetEmbedderResponse(
        [&key_calls = key_calls, &callback_handler = callback_handler_](
            const FlutterKeyEvent* event,
            MockKeyResponseController::ResponseCallback callback) {
          FlutterKeyEvent clone_event = *event;
          clone_event.character = event->character == nullptr
                                      ? nullptr
                                      : clone_string(event->character);
          key_calls.push_back(KeyCall{
              .type = KeyCall::kKeyCallOnKey,
              .key_event = clone_event,
          });
          callback_handler(event, callback);
        });
    key_response_controller->SetTextInputResponse(
        [&key_calls =
             key_calls](std::unique_ptr<rapidjson::Document> document) {
          rapidjson::StringBuffer buffer;
          rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
          document->Accept(writer);
          key_calls.push_back(KeyCall{
              .type = KeyCall::kKeyCallTextMethodCall,
              .text_method_call = buffer.GetString(),
          });
        });
    MockEmbedderApiForKeyboard(modifier, key_response_controller);

    engine->Run();

    return engine;
  }

  static MockKeyResponseController::EmbedderCallbackHandler RespondValue(
      bool value) {
    return [value](const FlutterKeyEvent* event,
                   MockKeyResponseController::ResponseCallback callback) {
      callback(value);
    };
  }

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(KeyboardTester);
};

class KeyboardTest : public WindowsTest {
 public:
  KeyboardTest() = default;
  virtual ~KeyboardTest() = default;

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(KeyboardTest);
};

}  // namespace

// Define compound `expect` in macros. If they're defined in functions, the
// stacktrace wouldn't print where the function is called in the unit tests.

#define EXPECT_CALL_IS_EVENT(_key_call, _type, _physical, _logical,    \
                             _character, _synthesized)                 \
  EXPECT_EQ(_key_call.type, KeyCall::kKeyCallOnKey);                   \
  EXPECT_EVENT_EQUALS(_key_call.key_event, _type, _physical, _logical, \
                      _character, _synthesized);

#define EXPECT_CALL_IS_TEXT(_key_call, u16_string)    \
  EXPECT_EQ(_key_call.type, KeyCall::kKeyCallOnText); \
  EXPECT_EQ(_key_call.text, u16_string);

#define EXPECT_CALL_IS_TEXT_METHOD_CALL(_key_call, json_string) \
  EXPECT_EQ(_key_call.type, KeyCall::kKeyCallTextMethodCall);   \
  EXPECT_STREQ(_key_call.text_method_call.c_str(), json_string);

TEST_F(KeyboardTest, LowerCaseAHandled) {
  KeyboardTester tester{GetContext()};
  tester.Responding(true);

  // US Keyboard layout

  // Press A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{'a', kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalKeyA, kLogicalKeyA, "a", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 0);

  // Release A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalKeyA, kLogicalKeyA, "", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 0);
}

TEST_F(KeyboardTest, LowerCaseAUnhandled) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // US Keyboard layout

  // Press A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{'a', kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalKeyA, kLogicalKeyA, "a", kNotSynthesized);
  EXPECT_CALL_IS_TEXT(tester.key_calls[1], u"a");
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  // Release A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalKeyA, kLogicalKeyA, "", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);
}

TEST_F(KeyboardTest, ArrowLeftHandled) {
  KeyboardTester tester{GetContext()};
  tester.Responding(true);

  // US Keyboard layout

  // Press ArrowLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{VK_LEFT, kScanCodeArrowLeft, kExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalArrowLeft, kLogicalArrowLeft, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 0);

  // Release ArrowLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{VK_LEFT, kScanCodeArrowLeft, kExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalArrowLeft, kLogicalArrowLeft, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 0);
}

TEST_F(KeyboardTest, ArrowLeftUnhandled) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // US Keyboard layout

  // Press ArrowLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{VK_LEFT, kScanCodeArrowLeft, kExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalArrowLeft, kLogicalArrowLeft, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Release ArrowLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{VK_LEFT, kScanCodeArrowLeft, kExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalArrowLeft, kLogicalArrowLeft, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);
}

TEST_F(KeyboardTest, ShiftLeftUnhandled) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // US Keyboard layout

  // Press ShiftLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LSHIFT, true, false},
      WmKeyDownInfo{VK_SHIFT, kScanCodeShiftLeft, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalShiftLeft, kLogicalShiftLeft, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Hold ShiftLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{VK_SHIFT, kScanCodeShiftLeft, kNotExtended, kWasDown}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeRepeat,
                       kPhysicalShiftLeft, kLogicalShiftLeft, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Release ShiftLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LSHIFT, false, true},
      WmKeyUpInfo{VK_SHIFT, kScanCodeShiftLeft, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalShiftLeft, kLogicalShiftLeft, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);
}

TEST_F(KeyboardTest, ShiftRightUnhandled) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // US Keyboard layout

  // Press ShiftRight
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_RSHIFT, true, false},
      WmKeyDownInfo{VK_SHIFT, kScanCodeShiftRight, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalShiftRight, kLogicalShiftRight, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Release ShiftRight
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_RSHIFT, false, true},
      WmKeyUpInfo{VK_SHIFT, kScanCodeShiftRight, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalShiftRight, kLogicalShiftRight, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);
}

TEST_F(KeyboardTest, CtrlLeftUnhandled) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // US Keyboard layout

  // Press CtrlLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LCONTROL, true, false},
      WmKeyDownInfo{VK_CONTROL, kScanCodeControl, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalControlLeft, kLogicalControlLeft, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Release CtrlLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LCONTROL, false, true},
      WmKeyUpInfo{VK_SHIFT, kScanCodeControl, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalControlLeft, kLogicalControlLeft, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);
}

TEST_F(KeyboardTest, CtrlRightUnhandled) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // US Keyboard layout

  // Press CtrlRight
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_RCONTROL, true, false},
      WmKeyDownInfo{VK_CONTROL, kScanCodeControl, kExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalControlRight, kLogicalControlRight, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Release CtrlRight
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_RCONTROL, false, true},
      WmKeyUpInfo{VK_CONTROL, kScanCodeControl, kExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalControlRight, kLogicalControlRight, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);
}

TEST_F(KeyboardTest, AltLeftUnhandled) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // US Keyboard layout

  // Press AltLeft. AltLeft is a SysKeyDown event.
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LMENU, true, false},
      WmSysKeyDownInfo{VK_MENU, kScanCodeAlt, kNotExtended, kWasUp}.Build(
          kWmResultDefault)});  // Always pass to the default WndProc.

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalAltLeft, kLogicalAltLeft, "", kNotSynthesized);
  tester.clear_key_calls();
  // Don't redispatch sys messages.
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 0);

  // Release AltLeft. AltLeft is a SysKeyUp event.
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LMENU, false, true},
      WmSysKeyUpInfo{VK_MENU, kScanCodeAlt, kNotExtended}.Build(
          kWmResultDefault)});  // Always pass to the default WndProc.

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalAltLeft, kLogicalAltLeft, "", kNotSynthesized);
  tester.clear_key_calls();
  // Don't redispatch sys messages.
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 0);
}

TEST_F(KeyboardTest, AltRightUnhandled) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // US Keyboard layout

  // Press AltRight. AltRight is a SysKeyDown event.
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_RMENU, true, false},
      WmSysKeyDownInfo{VK_MENU, kScanCodeAlt, kExtended, kWasUp}.Build(
          kWmResultDefault)});  // Always pass to the default WndProc.

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalAltRight, kLogicalAltRight, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  // Don't redispatch sys messages.
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 0);

  // Release AltRight. AltRight is a SysKeyUp event.
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_RMENU, false, true},
      WmSysKeyUpInfo{VK_MENU, kScanCodeAlt, kExtended}.Build(
          kWmResultDefault)});  // Always pass to the default WndProc.

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalAltRight, kLogicalAltRight, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  // Don't redispatch sys messages.
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 0);
}

TEST_F(KeyboardTest, MetaLeftUnhandled) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // US Keyboard layout

  // Press MetaLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LWIN, true, false},
      WmKeyDownInfo{VK_LWIN, kScanCodeMetaLeft, kExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalMetaLeft, kLogicalMetaLeft, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Release MetaLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LWIN, false, true},
      WmKeyUpInfo{VK_LWIN, kScanCodeMetaLeft, kExtended}.Build(kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalMetaLeft, kLogicalMetaLeft, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);
}

TEST_F(KeyboardTest, MetaRightUnhandled) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // US Keyboard layout

  // Press MetaRight
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_RWIN, true, false},
      WmKeyDownInfo{VK_RWIN, kScanCodeMetaRight, kExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalMetaRight, kLogicalMetaRight, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Release MetaRight
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_RWIN, false, true},
      WmKeyUpInfo{VK_RWIN, kScanCodeMetaRight, kExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalMetaRight, kLogicalMetaRight, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);
}

// Press and hold A. This should generate a repeat event.
TEST_F(KeyboardTest, RepeatA) {
  KeyboardTester tester{GetContext()};
  tester.Responding(true);

  // Press A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{'a', kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  // Hold A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended, kWasDown}.Build(
          kWmResultZero),
      WmCharInfo{'a', kScanCodeKeyA, kNotExtended, kWasDown}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalKeyA, kLogicalKeyA, "a", kNotSynthesized);
  EXPECT_CALL_IS_EVENT(tester.key_calls[1], kFlutterKeyEventTypeRepeat,
                       kPhysicalKeyA, kLogicalKeyA, "a", kNotSynthesized);
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 0);
}

// Press A, hot restart the engine, and hold A.
// This should reset the keyboard's state and generate
// two separate key down events.
TEST_F(KeyboardTest, RestartClearsKeyboardState) {
  KeyboardTester tester{GetContext()};
  tester.Responding(true);

  // Press A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{'a', kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  // Reset the keyboard's state.
  tester.ResetKeyboard();

  // Hold A. Notice the message declares the key is already down, however, the
  // the keyboard does not send a repeat event as its state was reset.
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended, kWasDown}.Build(
          kWmResultZero),
      WmCharInfo{'a', kScanCodeKeyA, kNotExtended, kWasDown}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalKeyA, kLogicalKeyA, "a", kNotSynthesized);
  EXPECT_CALL_IS_EVENT(tester.key_calls[1], kFlutterKeyEventTypeDown,
                       kPhysicalKeyA, kLogicalKeyA, "a", kNotSynthesized);
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 0);
}

// Press Shift-A. This is special because Win32 gives 'A' as character for the
// KeyA press.
TEST_F(KeyboardTest, ShiftLeftKeyA) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // US Keyboard layout

  // Press ShiftLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LSHIFT, true, true},
      WmKeyDownInfo{VK_SHIFT, kScanCodeShiftLeft, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalShiftLeft, kLogicalShiftLeft, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Press A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{'A', kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalKeyA, kLogicalKeyA, "A", kNotSynthesized);
  EXPECT_CALL_IS_TEXT(tester.key_calls[1], u"A");
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  // Release ShiftLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LSHIFT, false, true},
      WmKeyUpInfo{VK_SHIFT, kScanCodeShiftLeft, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalShiftLeft, kLogicalShiftLeft, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Release A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalKeyA, kLogicalKeyA, "", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);
}

// Press Ctrl-A. This is special because Win32 gives 0x01 as character for the
// KeyA press.
TEST_F(KeyboardTest, CtrlLeftKeyA) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // US Keyboard layout

  // Press ControlLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LCONTROL, true, true},
      WmKeyDownInfo{VK_CONTROL, kScanCodeControl, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalControlLeft, kLogicalControlLeft, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Press A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{0x01, kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalKeyA, kLogicalKeyA, "", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  // Release A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalKeyA, kLogicalKeyA, "", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Release ControlLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LCONTROL, false, true},
      WmKeyUpInfo{VK_CONTROL, kScanCodeControl, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalControlLeft, kLogicalControlLeft, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);
}

// Press Ctrl-1. This is special because it yields no WM_CHAR for the 1.
TEST_F(KeyboardTest, CtrlLeftDigit1) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // US Keyboard layout

  // Press ControlLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LCONTROL, true, true},
      WmKeyDownInfo{VK_CONTROL, kScanCodeControl, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalControlLeft, kLogicalControlLeft, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Press 1
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualDigit1, kScanCodeDigit1, kNotExtended, kWasUp}
          .Build(kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalDigit1, kLogicalDigit1, "", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Release 1
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{kVirtualDigit1, kScanCodeDigit1, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalDigit1, kLogicalDigit1, "", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Release ControlLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LCONTROL, false, true},
      WmKeyUpInfo{VK_CONTROL, kScanCodeControl, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalControlLeft, kLogicalControlLeft, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);
}

// Press 1 on a French keyboard. This is special because it yields WM_CHAR
// with char_code '&'.
TEST_F(KeyboardTest, Digit1OnFrenchLayout) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  tester.SetLayout(LayoutFrench);

  // Press 1
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualDigit1, kScanCodeDigit1, kNotExtended, kWasUp}
          .Build(kWmResultZero),
      WmCharInfo{'&', kScanCodeDigit1, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalDigit1, kLogicalDigit1, "&", kNotSynthesized);
  EXPECT_CALL_IS_TEXT(tester.key_calls[1], u"&");
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  // Release 1
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{kVirtualDigit1, kScanCodeDigit1, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalDigit1, kLogicalDigit1, "", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);
}

// This tests AltGr-Q on a German keyboard, which should print '@'.
TEST_F(KeyboardTest, AltGrModifiedKey) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // German Keyboard layout

  // Press AltGr, which Win32 precedes with a ContrlLeft down.
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LCONTROL, true, true},
      WmKeyDownInfo{VK_LCONTROL, kScanCodeControl, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      KeyStateChange{VK_RMENU, true, true},
      WmKeyDownInfo{VK_MENU, kScanCodeAlt, kExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalControlLeft, kLogicalControlLeft, "",
                       kNotSynthesized);
  EXPECT_CALL_IS_EVENT(tester.key_calls[1], kFlutterKeyEventTypeDown,
                       kPhysicalAltRight, kLogicalAltRight, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  // Press Q
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualKeyQ, kScanCodeKeyQ, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{'@', kScanCodeKeyQ, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalKeyQ, kLogicalKeyQ, "@", kNotSynthesized);
  EXPECT_CALL_IS_TEXT(tester.key_calls[1], u"@");
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  // Release Q
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{kVirtualKeyQ, kScanCodeKeyQ, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalKeyQ, kLogicalKeyQ, "", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Release AltGr. Win32 doesn't dispatch ControlLeft up. Instead Flutter will
  // forge one. The AltGr is a system key, therefore will be handled by Win32's
  // default WndProc.
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LCONTROL, false, true},
      ExpectForgedMessage{
          WmKeyUpInfo{VK_CONTROL, kScanCodeControl, kNotExtended}.Build(
              kWmResultZero)},
      KeyStateChange{VK_RMENU, false, true},
      WmSysKeyUpInfo{VK_MENU, kScanCodeAlt, kExtended}.Build(
          kWmResultDefault)});

  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalControlLeft, kLogicalControlLeft, "",
                       kNotSynthesized);
  EXPECT_CALL_IS_EVENT(tester.key_calls[1], kFlutterKeyEventTypeUp,
                       kPhysicalAltRight, kLogicalAltRight, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  // The sys key up must not be redispatched. The forged ControlLeft up will.
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);
}

// Test the following two key sequences at the same time:
//
// 1. Tap AltGr, then tap AltGr.
// 2. Tap AltGr, hold CtrlLeft, tap AltGr, release CtrlLeft.
//
// The two sequences are indistinguishable until the very end when a CtrlLeft
// up event might or might not follow.
//
//   Sequence 1: CtrlLeft down, AltRight down, AltRight up
//   Sequence 2: CtrlLeft down, AltRight down, AltRight up, CtrlLeft up
//
// This is because pressing AltGr alone causes Win32 to send a fake "CtrlLeft
// down" event first (see |IsKeyDownAltRight| for detailed explanation).
TEST_F(KeyboardTest, AltGrTwice) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // 1. AltGr down.

  // The key down event causes a ControlLeft down and a AltRight (extended
  // AltLeft) down.
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LCONTROL, true, true},
      WmKeyDownInfo{VK_CONTROL, kScanCodeControl, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      KeyStateChange{VK_RMENU, true, true},
      WmKeyDownInfo{VK_MENU, kScanCodeAlt, kExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalControlLeft, kLogicalControlLeft, "",
                       kNotSynthesized);
  EXPECT_CALL_IS_EVENT(tester.key_calls[1], kFlutterKeyEventTypeDown,
                       kPhysicalAltRight, kLogicalAltRight, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  // 2. AltGr up.

  // The key up event only causes a AltRight (extended AltLeft) up.
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LCONTROL, false, true},
      ExpectForgedMessage{
          WmKeyUpInfo{VK_CONTROL, kScanCodeControl, kNotExtended}.Build(
              kWmResultZero)},
      KeyStateChange{VK_RMENU, false, true},
      WmSysKeyUpInfo{VK_MENU, kScanCodeAlt, kExtended}.Build(
          kWmResultDefault)});
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalControlLeft, kLogicalControlLeft, "",
                       kNotSynthesized);
  EXPECT_CALL_IS_EVENT(tester.key_calls[1], kFlutterKeyEventTypeUp,
                       kPhysicalAltRight, kLogicalAltRight, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  // The sys key up must not be redispatched. The forged ControlLeft up will.
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // 3. AltGr down (or: ControlLeft down then AltRight down.)

  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LCONTROL, true, false},
      WmKeyDownInfo{VK_CONTROL, kScanCodeControl, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      KeyStateChange{VK_RMENU, true, true},
      WmKeyDownInfo{VK_MENU, kScanCodeAlt, kExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalControlLeft, kLogicalControlLeft, "",
                       kNotSynthesized);
  EXPECT_CALL_IS_EVENT(tester.key_calls[1], kFlutterKeyEventTypeDown,
                       kPhysicalAltRight, kLogicalAltRight, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  // 4. AltGr up.

  // The key up event only causes a AltRight (extended AltLeft) up.
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LCONTROL, false, false},
      ExpectForgedMessage{
          WmKeyUpInfo{VK_CONTROL, kScanCodeControl, kNotExtended}.Build(
              kWmResultZero)},
      KeyStateChange{VK_RMENU, false, false},
      WmSysKeyUpInfo{VK_MENU, kScanCodeAlt, kExtended}.Build(
          kWmResultDefault)});
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalControlLeft, kLogicalControlLeft, "",
                       kNotSynthesized);
  EXPECT_CALL_IS_EVENT(tester.key_calls[1], kFlutterKeyEventTypeUp,
                       kPhysicalAltRight, kLogicalAltRight, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  // The sys key up must not be redispatched. The forged ControlLeft up will.
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // 5. For key sequence 2: a real ControlLeft up.
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{VK_CONTROL, kScanCodeControl, kNotExtended}.Build(
          kWmResultZero)});
  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown, 0, 0, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 0);
}

// This tests dead key ^ then E on a French keyboard, which should be combined
// into ê.
TEST_F(KeyboardTest, DeadKeyThatCombines) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  tester.SetLayout(LayoutFrench);

  // Press ^¨ (US: Left bracket)
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{0xDD, kScanCodeBracketLeft, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmDeadCharInfo{'^', kScanCodeBracketLeft, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalBracketLeft, kLogicalBracketRight, "^",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  // Release ^¨
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{0xDD, kScanCodeBracketLeft, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalBracketLeft, kLogicalBracketRight, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Press E
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualKeyE, kScanCodeKeyE, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{0xEA, kScanCodeKeyE, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalKeyE, kLogicalKeyE, "ê", kNotSynthesized);
  EXPECT_CALL_IS_TEXT(tester.key_calls[1], u"ê");
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  // Release E
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{kVirtualKeyE, kScanCodeKeyE, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalKeyE, kLogicalKeyE, "", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);
}

// This tests dead key ^ then E on a US INTL keyboard, which should be combined
// into ê.
//
// It is different from French AZERTY because the character that the ^ key is
// mapped to does not contain the dead key character somehow.
TEST_F(KeyboardTest, DeadKeyWithoutDeadMaskThatCombines) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // Press ShiftLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LSHIFT, true, true},
      WmKeyDownInfo{VK_SHIFT, kScanCodeShiftLeft, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalShiftLeft, kLogicalShiftLeft, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Press 6^
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{'6', kScanCodeDigit6, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmDeadCharInfo{'^', kScanCodeDigit6, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalDigit6, kLogicalDigit6, "6", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  // Release 6^
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{'6', kScanCodeDigit6, kNotExtended}.Build(kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalDigit6, kLogicalDigit6, "", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Release ShiftLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LSHIFT, false, true},
      WmKeyUpInfo{VK_SHIFT, kScanCodeShiftLeft, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalShiftLeft, kLogicalShiftLeft, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Press E
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualKeyE, kScanCodeKeyE, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{0xEA, kScanCodeKeyE, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalKeyE, kLogicalKeyE, "ê", kNotSynthesized);
  EXPECT_CALL_IS_TEXT(tester.key_calls[1], u"ê");
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  // Release E
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{kVirtualKeyE, kScanCodeKeyE, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalKeyE, kLogicalKeyE, "", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);
}

// This tests dead key ^ then & (US: 1) on a French keyboard, which do not
// combine and should output "^&".
TEST_F(KeyboardTest, DeadKeyThatDoesNotCombine) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  tester.SetLayout(LayoutFrench);

  // Press ^¨ (US: Left bracket)
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{0xDD, kScanCodeBracketLeft, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmDeadCharInfo{'^', kScanCodeBracketLeft, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalBracketLeft, kLogicalBracketRight, "^",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  // Release ^¨
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{0xDD, kScanCodeBracketLeft, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalBracketLeft, kLogicalBracketRight, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Press 1
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualDigit1, kScanCodeDigit1, kNotExtended, kWasUp}
          .Build(kWmResultZero),
      WmCharInfo{'^', kScanCodeDigit1, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{'&', kScanCodeDigit1, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 3);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalDigit1, kLogicalDigit1, "^", kNotSynthesized);
  EXPECT_CALL_IS_TEXT(tester.key_calls[1], u"^");
  EXPECT_CALL_IS_TEXT(tester.key_calls[2], u"&");
  tester.clear_key_calls();
  // TODO(dkwingsmt): This count should probably be 3. Currently the '^'
  // message is redispatched due to being part of the KeyDown session, which is
  // not handled by the framework, while the '&' message is not redispatched
  // for being a standalone message. We should resolve this inconsistency.
  // https://github.com/flutter/flutter/issues/98306
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  // Release 1
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{kVirtualDigit1, kScanCodeDigit1, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalDigit1, kLogicalDigit1, "", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);
}

// This tests dead key `, then dead key `, then e.
//
// It should output ``e, instead of `è.
TEST_F(KeyboardTest, DeadKeyTwiceThenLetter) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // US INTL layout.

  // Press `
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{0xC0, kScanCodeBackquote, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmDeadCharInfo{'`', kScanCodeBackquote, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalBackquote, kLogicalBackquote, "`",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  // Release `
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{0xC0, kScanCodeBackquote, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalBackquote, kLogicalBackquote, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Press ` again.
  // The response should be slow.
  std::vector<MockKeyResponseController::ResponseCallback> recorded_callbacks;
  tester.LateResponding(
      [&recorded_callbacks](
          const FlutterKeyEvent* event,
          MockKeyResponseController::ResponseCallback callback) {
        recorded_callbacks.push_back(callback);
      });

  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{0xC0, kScanCodeBackquote, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{'`', kScanCodeBackquote, kNotExtended, kWasUp, kBeingReleased,
                 kNoContext, 1, /*bit25*/ true}
          .Build(kWmResultZero),
      WmCharInfo{'`', kScanCodeBackquote, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(recorded_callbacks.size(), 1);
  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalBackquote, kLogicalBackquote, "`",
                       kNotSynthesized);
  tester.clear_key_calls();
  // Key down event responded with false.
  recorded_callbacks.front()(false);
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_TEXT(tester.key_calls[0], u"`");
  EXPECT_CALL_IS_TEXT(tester.key_calls[1], u"`");
  tester.clear_key_calls();
  // TODO(dkwingsmt): This count should probably be 3. See the comment above
  // that is marked with the same issue.
  // https://github.com/flutter/flutter/issues/98306
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  tester.Responding(false);

  // Release `
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{0xC0, kScanCodeBackquote, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalBackquote, kLogicalBackquote, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);
}

// This tests when the resulting character needs to be combined with surrogates.
TEST_F(KeyboardTest, MultibyteCharacter) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // Gothic Keyboard layout. (We need a layout that yields non-BMP characters
  // without IME, which is actually very rare.)

  // Press key W of a US keyboard, which should yield character '𐍅'.
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualKeyW, kScanCodeKeyW, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{0xd800, kScanCodeKeyW, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{0xdf45, kScanCodeKeyW, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  const char* st = tester.key_calls[0].key_event.character;

  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalKeyW, kLogicalKeyW, "𐍅", kNotSynthesized);
  EXPECT_CALL_IS_TEXT(tester.key_calls[1], u"𐍅");
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 3);

  // Release W
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{kVirtualKeyW, kScanCodeKeyW, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalKeyW, kLogicalKeyW, "", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);
}

TEST_F(KeyboardTest, SynthesizeModifiers) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // Two dummy events used to trigger synthesization.
  Win32Message event1 =
      WmKeyDownInfo{VK_BACK, kScanCodeBackspace, kNotExtended, kWasUp}.Build(
          kWmResultZero);
  Win32Message event2 =
      WmKeyUpInfo{VK_BACK, kScanCodeBackspace, kNotExtended}.Build(
          kWmResultZero);

  // ShiftLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LSHIFT, true, true}, event1});
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalShiftLeft, kLogicalShiftLeft, "", kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LSHIFT, false, true}, event2});
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalShiftLeft, kLogicalShiftLeft, "", kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // ShiftRight
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_RSHIFT, true, true}, event1});
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalShiftRight, kLogicalShiftRight, "",
                       kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_RSHIFT, false, true}, event2});
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalShiftRight, kLogicalShiftRight, "",
                       kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // ControlLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LCONTROL, true, true}, event1});
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalControlLeft, kLogicalControlLeft, "",
                       kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LCONTROL, false, true}, event2});
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalControlLeft, kLogicalControlLeft, "",
                       kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // ControlRight
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_RCONTROL, true, true}, event1});
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalControlRight, kLogicalControlRight, "",
                       kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_RCONTROL, false, true}, event2});
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalControlRight, kLogicalControlRight, "",
                       kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // AltLeft
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LMENU, true, true}, event1});
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalAltLeft, kLogicalAltLeft, "", kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LMENU, false, true}, event2});
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalAltLeft, kLogicalAltLeft, "", kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // AltRight
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_RMENU, true, true}, event1});
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalAltRight, kLogicalAltRight, "", kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_RMENU, false, true}, event2});
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalAltRight, kLogicalAltRight, "", kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // MetaLeft
  tester.InjectKeyboardChanges(
      std::vector<KeyboardChange>{KeyStateChange{VK_LWIN, true, true}, event1});
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalMetaLeft, kLogicalMetaLeft, "", kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LWIN, false, true}, event2});
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalMetaLeft, kLogicalMetaLeft, "", kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // MetaRight
  tester.InjectKeyboardChanges(
      std::vector<KeyboardChange>{KeyStateChange{VK_RWIN, true, true}, event1});
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalMetaRight, kLogicalMetaRight, "", kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_RWIN, false, true}, event2});
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalMetaRight, kLogicalMetaRight, "", kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // CapsLock, phase 0 -> 2 -> 0.
  // (For phases, see |SynchronizeCriticalToggledStates|.)
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_CAPITAL, false, true}, event1});
  EXPECT_EQ(tester.key_calls.size(), 3);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalCapsLock, kLogicalCapsLock, "", kSynthesized);
  EXPECT_CALL_IS_EVENT(tester.key_calls[1], kFlutterKeyEventTypeUp,
                       kPhysicalCapsLock, kLogicalCapsLock, "", kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_CAPITAL, false, false}, event2});
  EXPECT_EQ(tester.key_calls.size(), 3);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalCapsLock, kLogicalCapsLock, "", kSynthesized);
  EXPECT_CALL_IS_EVENT(tester.key_calls[1], kFlutterKeyEventTypeUp,
                       kPhysicalCapsLock, kLogicalCapsLock, "", kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // ScrollLock, phase 0 -> 1 -> 3
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_SCROLL, true, true}, event1});
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalScrollLock, kLogicalScrollLock, "",
                       kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_SCROLL, true, false}, event2});
  EXPECT_EQ(tester.key_calls.size(), 3);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalScrollLock, kLogicalScrollLock, "",
                       kSynthesized);
  EXPECT_CALL_IS_EVENT(tester.key_calls[1], kFlutterKeyEventTypeDown,
                       kPhysicalScrollLock, kLogicalScrollLock, "",
                       kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // NumLock, phase 0 -> 3 -> 2
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_NUMLOCK, true, false}, event1});
  // TODO(dkwingsmt): Synthesizing from phase 0 to 3 should yield a full key
  // tap and a key down. Fix the algorithm so that the following result becomes
  // 4 keycalls with an extra pair of key down and up.
  // https://github.com/flutter/flutter/issues/98533
  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalNumLock, kLogicalNumLock, "", kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_NUMLOCK, false, true}, event2});
  EXPECT_EQ(tester.key_calls.size(), 4);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalNumLock, kLogicalNumLock, "", kSynthesized);
  EXPECT_CALL_IS_EVENT(tester.key_calls[1], kFlutterKeyEventTypeDown,
                       kPhysicalNumLock, kLogicalNumLock, "", kSynthesized);
  EXPECT_CALL_IS_EVENT(tester.key_calls[2], kFlutterKeyEventTypeUp,
                       kPhysicalNumLock, kLogicalNumLock, "", kSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);
}

// Pressing extended keys during IME events should work properly by not sending
// any events.
//
// Regression test for https://github.com/flutter/flutter/issues/95888 .
TEST_F(KeyboardTest, ImeExtendedEventsAreIgnored) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // US Keyboard layout.

  // There should be preceding key events to make the keyboard into IME mode.
  // Omit them in this test since they are not relavent.

  // Press CtrlRight in IME mode.
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_RCONTROL, true, false},
      WmKeyDownInfo{VK_PROCESSKEY, kScanCodeControl, kExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown, 0, 0, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 0);
}

// Ensures that synthesization works correctly when a Shift key is pressed and
// (only) its up event is labeled as an IME event (VK_PROCESSKEY).
//
// Regression test for https://github.com/flutter/flutter/issues/104169. These
// are real messages recorded when pressing Shift-2 using Microsoft Pinyin IME
// on Win 10 Enterprise, which crashed the app before the fix.
TEST_F(KeyboardTest, UpOnlyImeEventsAreCorrectlyHandled) {
  KeyboardTester tester{GetContext()};
  tester.Responding(true);

  // US Keyboard layout.

  // Press CtrlRight in IME mode.
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      KeyStateChange{VK_LSHIFT, true, false},
      WmKeyDownInfo{VK_SHIFT, kScanCodeShiftLeft, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmKeyDownInfo{VK_PROCESSKEY, kScanCodeDigit2, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      KeyStateChange{VK_LSHIFT, false, true},
      WmKeyUpInfo{VK_PROCESSKEY, kScanCodeShiftLeft, kNotExtended}.Build(
          kWmResultZero),
      WmKeyUpInfo{'2', kScanCodeDigit2, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 4);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalShiftLeft, kLogicalShiftLeft, "",
                       kNotSynthesized);
  EXPECT_CALL_IS_EVENT(tester.key_calls[1], kFlutterKeyEventTypeDown, 0, 0, "",
                       kNotSynthesized);
  EXPECT_CALL_IS_EVENT(tester.key_calls[2], kFlutterKeyEventTypeUp,
                       kPhysicalShiftLeft, kLogicalShiftLeft, "",
                       kNotSynthesized);
  EXPECT_CALL_IS_EVENT(tester.key_calls[3], kFlutterKeyEventTypeDown, 0, 0, "",
                       kNotSynthesized);
  tester.clear_key_calls();
}

// Regression test for a crash in an earlier implementation.
//
// In real life, the framework responds slowly. The next real event might
// arrive earlier than the framework response, and if the 2nd event has an
// identical hash as the one waiting for response, an earlier implementation
// will crash upon the response.
TEST_F(KeyboardTest, SlowFrameworkResponse) {
  KeyboardTester tester{GetContext()};

  std::vector<MockKeyResponseController::ResponseCallback> recorded_callbacks;

  // Store callbacks to manually call them.
  tester.LateResponding(
      [&recorded_callbacks](
          const FlutterKeyEvent* event,
          MockKeyResponseController::ResponseCallback callback) {
        recorded_callbacks.push_back(callback);
      });

  // Press A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{'a', kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  // Hold A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended, kWasDown}.Build(
          kWmResultZero),
      WmCharInfo{'a', kScanCodeKeyA, kNotExtended, kWasDown}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalKeyA, kLogicalKeyA, "a", kNotSynthesized);
  EXPECT_EQ(recorded_callbacks.size(), 1);
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 0);

  // The first response.
  recorded_callbacks.front()(false);

  EXPECT_EQ(tester.key_calls.size(), 3);
  EXPECT_EQ(recorded_callbacks.size(), 2);
  EXPECT_CALL_IS_TEXT(tester.key_calls[1], u"a");
  EXPECT_CALL_IS_EVENT(tester.key_calls[2], kFlutterKeyEventTypeRepeat,
                       kPhysicalKeyA, kLogicalKeyA, "a", kNotSynthesized);
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  // The second response.
  recorded_callbacks.back()(false);

  EXPECT_EQ(tester.key_calls.size(), 4);
  EXPECT_CALL_IS_TEXT(tester.key_calls[3], u"a");
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);
}

// Regression test for https://github.com/flutter/flutter/issues/84210.
//
// When the framework response is slow during a sequence of identical messages,
// make sure the real messages are not mistaken as redispatched messages,
// in order to not mess up the order of events.
//
// In this test we use:
//
//   KeyA down, KeyA up, (down event responded with false), KeyA down, KeyA up,
//
// The code must not take the 2nd real key down events as a redispatched event.
TEST_F(KeyboardTest, SlowFrameworkResponseForIdenticalEvents) {
  KeyboardTester tester{GetContext()};
  std::vector<MockKeyResponseController::ResponseCallback> recorded_callbacks;

  // Store callbacks to manually call them.
  tester.LateResponding(
      [&recorded_callbacks](
          const FlutterKeyEvent* event,
          MockKeyResponseController::ResponseCallback callback) {
        recorded_callbacks.push_back(callback);
      });

  // Press A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{'a', kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalKeyA, kLogicalKeyA, "a", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 0);

  // Release A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 0);
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 0);

  // The first down event responded with false.
  EXPECT_EQ(recorded_callbacks.size(), 1);
  recorded_callbacks.front()(false);

  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_TEXT(tester.key_calls[0], u"a");
  EXPECT_CALL_IS_EVENT(tester.key_calls[1], kFlutterKeyEventTypeUp,
                       kPhysicalKeyA, kLogicalKeyA, "", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  // Press A again
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{'a', kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  // Nothing more was dispatched because the first up event hasn't been
  // responded yet.
  EXPECT_EQ(recorded_callbacks.size(), 2);
  EXPECT_EQ(tester.key_calls.size(), 0);
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 0);

  // The first up event responded with false, which was redispatched, and caused
  // the down event to be dispatched.
  recorded_callbacks.back()(false);
  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalKeyA, kLogicalKeyA, "a", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(recorded_callbacks.size(), 3);
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Release A again
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 0);
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 0);
}

TEST_F(KeyboardTest, TextInputSubmit) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // US Keyboard layout

  tester.InjectPlatformMessage(
      "flutter/textinput", "TextInput.setClient",
      R"|([108, {"inputAction": "TextInputAction.none", "viewId": 0}])|");

  // Press Enter
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{VK_RETURN, kScanCodeEnter, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{'\n', kScanCodeEnter, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalEnter, kLogicalEnter, "", kNotSynthesized);
  EXPECT_CALL_IS_TEXT_METHOD_CALL(
      tester.key_calls[1],
      "{"
      R"|("method":"TextInputClient.performAction",)|"
      R"|("args":[108,"TextInputAction.none"])|"
      "}");
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  // Release Enter
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{VK_RETURN, kScanCodeEnter, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalEnter, kLogicalEnter, "", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Make sure OnText is not obstructed after pressing Enter.
  //
  // Regression test for https://github.com/flutter/flutter/issues/97706.

  // Press A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{'a', kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalKeyA, kLogicalKeyA, "a", kNotSynthesized);
  EXPECT_CALL_IS_TEXT(tester.key_calls[1], u"a");
  tester.clear_key_calls();

  // Release A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalKeyA, kLogicalKeyA, "", kNotSynthesized);
  tester.clear_key_calls();
}

TEST_F(KeyboardTest, VietnameseTelexAddDiacriticWithFastResponse) {
  // In this test, the user presses the folloing keys:
  //
  //   Key         Current text
  //  ===========================
  //   A           a
  //   F           à
  //
  // And the Backspace event is responded immediately.

  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  // US Keyboard layout

  // Press A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{'a', kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalKeyA, kLogicalKeyA, "a", kNotSynthesized);
  EXPECT_CALL_IS_TEXT(tester.key_calls[1], u"a");
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  // Release A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalKeyA, kLogicalKeyA, "", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  // Press F, which is translated to:
  //
  // Backspace down, char & up, then VK_PACKET('à').
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{VK_BACK, kScanCodeBackspace, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{0x8, kScanCodeBackspace, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmKeyUpInfo{VK_BACK, kScanCodeBackspace, kNotExtended}.Build(
          kWmResultZero),
      WmKeyDownInfo{VK_PACKET, 0, kNotExtended, kWasUp}.Build(kWmResultDefault),
      WmCharInfo{0xe0 /*'à'*/, 0, kNotExtended, kWasUp}.Build(kWmResultZero),
      WmKeyUpInfo{VK_PACKET, 0, kNotExtended}.Build(kWmResultDefault)});

  EXPECT_EQ(tester.key_calls.size(), 3);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalBackspace, kLogicalBackspace, "",
                       kNotSynthesized);
  EXPECT_CALL_IS_EVENT(tester.key_calls[1], kFlutterKeyEventTypeUp,
                       kPhysicalBackspace, kLogicalBackspace, "",
                       kNotSynthesized);
  EXPECT_CALL_IS_TEXT(tester.key_calls[2], u"à");
  tester.clear_key_calls();
  // TODO(dkwingsmt): This count should probably be 4. Currently the CHAR 0x8
  // message is redispatched due to being part of the KeyDown session, which is
  // not handled by the framework, while the 'à' message is not redispatched
  // for being a standalone message. We should resolve this inconsistency.
  // https://github.com/flutter/flutter/issues/98306
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 3);

  // Release F
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{kVirtualKeyF, kScanCodeKeyF, kNotExtended,
                  /* overwrite_prev_state_0 */ true}
          .Build(kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown, 0, 0, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 0);
}

void VietnameseTelexAddDiacriticWithSlowResponse(WindowsTestContext& context,
                                                 bool backspace_response) {
  // In this test, the user presses the following keys:
  //
  //   Key         Current text
  //  ===========================
  //   A           a
  //   F           à
  //
  // And the Backspace down event is responded slowly with `backspace_response`.

  KeyboardTester tester{context};
  tester.Responding(false);

  // US Keyboard layout

  // Press A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{'a', kScanCodeKeyA, kNotExtended, kWasUp}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 2);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalKeyA, kLogicalKeyA, "a", kNotSynthesized);
  EXPECT_CALL_IS_TEXT(tester.key_calls[1], u"a");
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 2);

  // Release A
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{kVirtualKeyA, kScanCodeKeyA, kNotExtended}.Build(
          kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalKeyA, kLogicalKeyA, "", kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  std::vector<MockKeyResponseController::ResponseCallback> recorded_callbacks;
  tester.LateResponding(
      [&recorded_callbacks](
          const FlutterKeyEvent* event,
          MockKeyResponseController::ResponseCallback callback) {
        recorded_callbacks.push_back(callback);
      });

  // Press F, which is translated to:
  //
  // Backspace down, char & up, VK_PACKET('à').
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{VK_BACK, kScanCodeBackspace, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmCharInfo{0x8, kScanCodeBackspace, kNotExtended, kWasUp}.Build(
          kWmResultZero),
      WmKeyUpInfo{VK_BACK, kScanCodeBackspace, kNotExtended}.Build(
          kWmResultZero),
      WmKeyDownInfo{VK_PACKET, 0, kNotExtended, kWasUp}.Build(kWmResultDefault),
      WmCharInfo{0xe0 /*'à'*/, 0, kNotExtended, kWasUp}.Build(kWmResultZero),
      WmKeyUpInfo{VK_PACKET, 0, kNotExtended}.Build(kWmResultDefault)});

  // The Backspace event has not responded yet, therefore the char message must
  // hold. This is because when the framework is handling the Backspace event,
  // it will send a setEditingState message that updates the text state that has
  // the last character deleted  (denoted by `string1`). Processing the char
  // message before then will cause the final text to set to `string1`.
  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown,
                       kPhysicalBackspace, kLogicalBackspace, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 0);

  EXPECT_EQ(recorded_callbacks.size(), 1);
  recorded_callbacks[0](backspace_response);

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeUp,
                       kPhysicalBackspace, kLogicalBackspace, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(),
            backspace_response ? 0 : 2);

  recorded_callbacks[1](false);
  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_TEXT(tester.key_calls[0], u"à");
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 1);

  tester.Responding(false);

  // Release F
  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyUpInfo{kVirtualKeyF, kScanCodeKeyF, kNotExtended,
                  /* overwrite_prev_state_0 */ true}
          .Build(kWmResultZero)});

  EXPECT_EQ(tester.key_calls.size(), 1);
  EXPECT_CALL_IS_EVENT(tester.key_calls[0], kFlutterKeyEventTypeDown, 0, 0, "",
                       kNotSynthesized);
  tester.clear_key_calls();
  EXPECT_EQ(tester.RedispatchedMessageCountAndClear(), 0);
}

TEST_F(KeyboardTest, VietnameseTelexAddDiacriticWithSlowFalseResponse) {
  VietnameseTelexAddDiacriticWithSlowResponse(GetContext(), false);
}

TEST_F(KeyboardTest, VietnameseTelexAddDiacriticWithSlowTrueResponse) {
  VietnameseTelexAddDiacriticWithSlowResponse(GetContext(), true);
}

// Ensure that the scancode-less key events issued by Narrator
// when toggling caps lock don't violate assert statements.
TEST_F(KeyboardTest, DoubleCapsLock) {
  KeyboardTester tester{GetContext()};
  tester.Responding(false);

  tester.InjectKeyboardChanges(std::vector<KeyboardChange>{
      WmKeyDownInfo{VK_CAPITAL, 0, kNotExtended}.Build(),
      WmKeyUpInfo{VK_CAPITAL, 0, kNotExtended}.Build()});

  tester.clear_key_calls();
}

}  // namespace testing
}  // namespace flutter
