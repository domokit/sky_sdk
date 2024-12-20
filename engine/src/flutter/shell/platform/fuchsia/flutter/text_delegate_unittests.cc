// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "text_delegate.h"

#include <fuchsia/ui/input/cpp/fidl.h>
#include <fuchsia/ui/input3/cpp/fidl.h>
#include <fuchsia/ui/views/cpp/fidl.h>
#include <gtest/gtest.h>
#include <lib/async-loop/cpp/loop.h>
#include <lib/async-loop/default.h>
#include <lib/fidl/cpp/binding.h>
#include <lib/fidl/cpp/binding_set.h>
#include <lib/zx/eventpair.h>

#include "tests/fakes/platform_message.h"

#include "flutter/lib/ui/window/platform_message.h"

#include <memory>

namespace flutter_runner::testing {

// Convert a |PlatformMessage| to string for ease of testing.
static std::string MessageToString(PlatformMessage& message) {
  const char* data = reinterpret_cast<const char*>(message.data().GetMapping());
  return std::string(data, message.data().GetSize());
}

// Fake |KeyboardService| implementation. Only responsibility is to remember
// what it was called with.
class FakeKeyboardService : public fuchsia::ui::input3::Keyboard {
 public:
  // |fuchsia.ui.input3/Keyboard.AddListener|
  virtual void AddListener(
      fuchsia::ui::views::ViewRef,
      fidl::InterfaceHandle<fuchsia::ui::input3::KeyboardListener> listener,
      fuchsia::ui::input3::Keyboard::AddListenerCallback callback) {
    listener_ = listener.Bind();
    callback();
  }

  fidl::InterfacePtr<fuchsia::ui::input3::KeyboardListener> listener_;
};

// Fake ImeService implementation. Only responsibility is to remember what
// it was called with.
class FakeImeService : public fuchsia::ui::input::ImeService {
 public:
  virtual void GetInputMethodEditor(
      fuchsia::ui::input::KeyboardType keyboard_type,
      fuchsia::ui::input::InputMethodAction action,
      fuchsia::ui::input::TextInputState input_state,
      fidl::InterfaceHandle<fuchsia::ui::input::InputMethodEditorClient> client,
      fidl::InterfaceRequest<fuchsia::ui::input::InputMethodEditor> ime) {
    keyboard_type_ = std::move(keyboard_type);
    action_ = std::move(action);
    input_state_ = std::move(input_state);
    client_ = client.Bind();
    ime_ = std::move(ime);
  }

  virtual void ShowKeyboard() { keyboard_shown_ = true; }

  virtual void HideKeyboard() { keyboard_shown_ = false; }

  bool IsKeyboardShown() { return keyboard_shown_; }

  bool keyboard_shown_ = false;

  fuchsia::ui::input::KeyboardType keyboard_type_;
  fuchsia::ui::input::InputMethodAction action_;
  fuchsia::ui::input::TextInputState input_state_;
  fidl::InterfacePtr<fuchsia::ui::input::InputMethodEditorClient> client_;
  fidl::InterfaceRequest<fuchsia::ui::input::InputMethodEditor> ime_;
};

class TextDelegateTest : public ::testing::Test {
 protected:
  TextDelegateTest()
      : loop_(&kAsyncLoopConfigAttachToCurrentThread),
        keyboard_service_binding_(&keyboard_service_),
        ime_service_binding_(&ime_service_) {
    fidl::InterfaceHandle<fuchsia::ui::input3::Keyboard> keyboard_handle;
    auto keyboard_request = keyboard_handle.NewRequest();
    keyboard_service_binding_.Bind(keyboard_request.TakeChannel());

    fidl::InterfaceHandle<fuchsia::ui::input::ImeService> ime_service_handle;
    ime_service_binding_.Bind(ime_service_handle.NewRequest().TakeChannel());

    fuchsia::ui::views::ViewRefControl view_ref_control;
    fuchsia::ui::views::ViewRef view_ref;
    auto status = zx::eventpair::create(
        /*options*/ 0u, &view_ref_control.reference, &view_ref.reference);
    ZX_ASSERT(status == ZX_OK);
    view_ref.reference.replace(ZX_RIGHTS_BASIC, &view_ref.reference);

    text_delegate_ = std::make_unique<TextDelegate>(
        std::move(view_ref), std::move(ime_service_handle),
        std::move(keyboard_handle),
        // Should this be accessed through a weak pointer?
        [this](std::unique_ptr<flutter::PlatformMessage> message) {
          last_message_ = std::move(message);
        });

    // TextDelegate has some async initialization that needs to happen.
    RunLoopUntilIdle();
  }

  // Runs the event loop until all scheduled events are spent.
  void RunLoopUntilIdle() { loop_.RunUntilIdle(); }

  void TearDown() override {
    loop_.Quit();
    ASSERT_EQ(loop_.ResetQuit(), 0);
  }

  async::Loop loop_;

  FakeKeyboardService keyboard_service_;
  fidl::Binding<fuchsia::ui::input3::Keyboard> keyboard_service_binding_;

  FakeImeService ime_service_;
  fidl::Binding<fuchsia::ui::input::ImeService> ime_service_binding_;

  // Unit under test.
  std::unique_ptr<TextDelegate> text_delegate_;

  std::unique_ptr<flutter::PlatformMessage> last_message_;
};

// Goes through several steps of a text edit protocol. These are hard to test
// in isolation because the text edit protocol depends on the correct method
// invocation sequence. The text editor is initialized with the editing
// parameters, and we verify that the correct input action is parsed out. We
// then exercise showing and hiding the keyboard, as well as a text state
// update.
TEST_F(TextDelegateTest, ActivateIme) {
  auto fake_platform_message_response = FakePlatformMessageResponse::Create();
  {
    // Initialize the editor. Without this initialization, the protocol code
    // will crash.
    const auto set_client_msg = R"(
      {
        "method": "TextInput.setClient",
        "args": [
           7,
           {
             "inputType": {
               "name": "TextInputType.multiline",
               "signed":null,
               "decimal":null
             },
             "readOnly": false,
             "obscureText": false,
             "autocorrect":true,
             "smartDashesType":"1",
             "smartQuotesType":"1",
             "enableSuggestions":true,
             "enableInteractiveSelection":true,
             "actionLabel":null,
             "inputAction":"TextInputAction.newline",
             "textCapitalization":"TextCapitalization.none",
             "keyboardAppearance":"Brightness.dark",
             "enableIMEPersonalizedLearning":true,
             "enableDeltaModel":false
          }
       ]
      }
    )";
    auto message = fake_platform_message_response->WithMessage(
        kTextInputChannel, set_client_msg);
    text_delegate_->HandleFlutterTextInputChannelPlatformMessage(
        std::move(message));
    RunLoopUntilIdle();
    EXPECT_EQ(ime_service_.action_,
              fuchsia::ui::input::InputMethodAction::NEWLINE);
    EXPECT_FALSE(ime_service_.IsKeyboardShown());
  }

  {
    // Verify that showing keyboard results in the correct platform effect.
    const auto set_client_msg = R"(
      {
        "method": "TextInput.show"
      }
    )";
    auto message = fake_platform_message_response->WithMessage(
        kTextInputChannel, set_client_msg);
    text_delegate_->HandleFlutterTextInputChannelPlatformMessage(
        std::move(message));
    RunLoopUntilIdle();
    EXPECT_TRUE(ime_service_.IsKeyboardShown());
  }

  {
    // Verify that hiding keyboard results in the correct platform effect.
    const auto set_client_msg = R"(
      {
        "method": "TextInput.hide"
      }
    )";
    auto message = fake_platform_message_response->WithMessage(
        kTextInputChannel, set_client_msg);
    text_delegate_->HandleFlutterTextInputChannelPlatformMessage(
        std::move(message));
    RunLoopUntilIdle();
    EXPECT_FALSE(ime_service_.IsKeyboardShown());
  }

  {
    // Update the editing state from the Fuchsia platform side.
    fuchsia::ui::input::TextInputState state = {
        .revision = 42,
        .text = "Foo",
        .selection = fuchsia::ui::input::TextSelection{},
        .composing = fuchsia::ui::input::TextRange{},
    };
    auto input_event = std::make_unique<fuchsia::ui::input::InputEvent>();
    ime_service_.client_->DidUpdateState(std::move(state),
                                         std::move(input_event));
    RunLoopUntilIdle();
    EXPECT_EQ(
        R"({"method":"TextInputClient.updateEditingState","args":[7,{"text":"Foo","selectionBase":0,"selectionExtent":0,"selectionAffinity":"TextAffinity.upstream","selectionIsDirectional":true,"composingBase":-1,"composingExtent":-1}]})",
        MessageToString(*last_message_));
  }

  {
    // Notify Flutter that the action key has been pressed.
    ime_service_.client_->OnAction(fuchsia::ui::input::InputMethodAction::DONE);
    RunLoopUntilIdle();
    EXPECT_EQ(
        R"({"method":"TextInputClient.performAction","args":[7,"TextInputAction.done"]})",
        MessageToString(*last_message_));
  }
}

// Hands a few typical |KeyEvent|s to the text delegate. Regular key events are
// handled, "odd" key events are rejected (not handled).  "Handling" a key event
// means converting it to an appropriate |PlatformMessage| and forwarding it.
TEST_F(TextDelegateTest, OnAction) {
  {
    // A sensible key event is converted into a platform message.
    fuchsia::ui::input3::KeyEvent key_event;
    *key_event.mutable_type() = fuchsia::ui::input3::KeyEventType::PRESSED;
    *key_event.mutable_key() = fuchsia::input::Key::A;
    key_event.mutable_key_meaning()->set_codepoint('a');

    fuchsia::ui::input3::KeyEventStatus status;
    keyboard_service_.listener_->OnKeyEvent(
        std::move(key_event), [&status](fuchsia::ui::input3::KeyEventStatus s) {
          status = std::move(s);
        });
    RunLoopUntilIdle();
    EXPECT_EQ(fuchsia::ui::input3::KeyEventStatus::HANDLED, status);
    EXPECT_EQ(
        R"({"type":"keydown","keymap":"fuchsia","hidUsage":458756,"codePoint":97,"modifiers":0})",
        MessageToString(*last_message_));
  }

  {
    // SYNC event is not handled.
    // This is currently expected, though we may need to change that behavior.
    fuchsia::ui::input3::KeyEvent key_event;
    *key_event.mutable_type() = fuchsia::ui::input3::KeyEventType::SYNC;

    fuchsia::ui::input3::KeyEventStatus status;
    keyboard_service_.listener_->OnKeyEvent(
        std::move(key_event), [&status](fuchsia::ui::input3::KeyEventStatus s) {
          status = std::move(s);
        });
    RunLoopUntilIdle();
    EXPECT_EQ(fuchsia::ui::input3::KeyEventStatus::NOT_HANDLED, status);
  }

  {
    // CANCEL event is not handled.
    // This is currently expected, though we may need to change that behavior.
    fuchsia::ui::input3::KeyEvent key_event;
    *key_event.mutable_type() = fuchsia::ui::input3::KeyEventType::CANCEL;

    fuchsia::ui::input3::KeyEventStatus status;
    keyboard_service_.listener_->OnKeyEvent(
        std::move(key_event), [&status](fuchsia::ui::input3::KeyEventStatus s) {
          status = std::move(s);
        });
    RunLoopUntilIdle();
    EXPECT_EQ(fuchsia::ui::input3::KeyEventStatus::NOT_HANDLED, status);
  }
}

}  // namespace flutter_runner::testing
