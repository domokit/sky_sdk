// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "text_delegate.h"

#include <fuchsia/ui/input/cpp/fidl.h>
#include <fuchsia/ui/input3/cpp/fidl.h>
#include <fuchsia/ui/views/cpp/fidl.h>
#include <lib/fidl/cpp/binding.h>

#include "flutter/fml/logging.h"
#include "flutter/fml/mapping.h"
#include "flutter/lib/ui/window/platform_message.h"
#include "flutter/shell/platform/fuchsia/flutter/keyboard.h"
#include "flutter/shell/platform/fuchsia/runtime/dart/utils/inlines.h"
#include "third_party/rapidjson/include/rapidjson/document.h"
#include "third_party/rapidjson/include/rapidjson/stringbuffer.h"
#include "third_party/rapidjson/include/rapidjson/writer.h"

#include "logging.h"

namespace flutter_runner {

static constexpr char kInputActionKey[] = "inputAction";

// See: https://api.flutter.dev/flutter/services/TextInputAction.html
// Only the actions relevant for Fuchsia are listed here.
static constexpr char kTextInputActionDone[] = "TextInputAction.done";
static constexpr char kTextInputActionNewline[] = "TextInputAction.newline";
static constexpr char kTextInputActionGo[] = "TextInputAction.go";
static constexpr char kTextInputActionNext[] = "TextInputAction.next";
static constexpr char kTextInputActionPrevious[] = "TextInputAction.previous";
static constexpr char kTextInputActionNone[] = "TextInputAction.none";
static constexpr char kTextInputActionSearch[] = "TextInputAction.search";
static constexpr char kTextInputActionSend[] = "TextInputAction.send";
static constexpr char kTextInputActionUnspecified[] =
    "TextInputAction.unspecified";

// Converts Flutter TextInputAction to Fuchsia action enum.
static fuchsia::ui::input::InputMethodAction IntoInputMethodAction(
    const std::string action_string) {
  if (action_string == kTextInputActionNewline) {
    return fuchsia::ui::input::InputMethodAction::NEWLINE;
  } else if (action_string == kTextInputActionDone) {
    return fuchsia::ui::input::InputMethodAction::DONE;
  } else if (action_string == kTextInputActionGo) {
    return fuchsia::ui::input::InputMethodAction::GO;
  } else if (action_string == kTextInputActionNext) {
    return fuchsia::ui::input::InputMethodAction::NEXT;
  } else if (action_string == kTextInputActionPrevious) {
    return fuchsia::ui::input::InputMethodAction::PREVIOUS;
  } else if (action_string == kTextInputActionNone) {
    return fuchsia::ui::input::InputMethodAction::NONE;
  } else if (action_string == kTextInputActionSearch) {
    return fuchsia::ui::input::InputMethodAction::SEARCH;
  } else if (action_string == kTextInputActionSend) {
    return fuchsia::ui::input::InputMethodAction::SEND;
  } else if (action_string == kTextInputActionUnspecified) {
    return fuchsia::ui::input::InputMethodAction::UNSPECIFIED;
  }
  // If this message comes along it means we should really add the missing 'if'
  // above.
  FML_VLOG(1) << "unexpected action_string: " << action_string;
  // Substituting DONE for an unexpected action string will probably be OK.
  return fuchsia::ui::input::InputMethodAction::DONE;
}

// Converts the Fuchsia action enum into Flutter TextInputAction.
static const std::string IntoTextInputAction(
    fuchsia::ui::input::InputMethodAction action) {
  if (action == fuchsia::ui::input::InputMethodAction::NEWLINE) {
    return kTextInputActionNewline;
  } else if (action == fuchsia::ui::input::InputMethodAction::DONE) {
    return kTextInputActionDone;
  } else if (action == fuchsia::ui::input::InputMethodAction::GO) {
    return kTextInputActionGo;
  } else if (action == fuchsia::ui::input::InputMethodAction::NEXT) {
    return kTextInputActionNext;
  } else if (action == fuchsia::ui::input::InputMethodAction::PREVIOUS) {
    return kTextInputActionPrevious;
  } else if (action == fuchsia::ui::input::InputMethodAction::NONE) {
    return kTextInputActionNone;
  } else if (action == fuchsia::ui::input::InputMethodAction::SEARCH) {
    return kTextInputActionSearch;
  } else if (action == fuchsia::ui::input::InputMethodAction::SEND) {
    return kTextInputActionSend;
  } else if (action == fuchsia::ui::input::InputMethodAction::UNSPECIFIED) {
    return kTextInputActionUnspecified;
  }
  // If this message comes along it means we should really add the missing 'if'
  // above.
  FML_VLOG(1) << "unexpected action: " << static_cast<uint32_t>(action);
  // Substituting "done" for an unexpected text input action will probably
  // be OK.
  return kTextInputActionDone;
}

// TODO(fxbug.dev/8868): Terminate engine if Fuchsia system FIDL connections
// have error.
template <class T>
void SetInterfaceErrorHandler(fidl::InterfacePtr<T>& interface,
                              std::string name) {
  interface.set_error_handler([name](zx_status_t status) {
    FML_LOG(ERROR) << "Interface error on: " << name << ", status: " << status;
  });
}
template <class T>
void SetInterfaceErrorHandler(fidl::Binding<T>& binding, std::string name) {
  binding.set_error_handler([name](zx_status_t status) {
    FML_LOG(ERROR) << "Binding error on: " << name << ", status: " << status;
  });
}

TextDelegate::TextDelegate(
    fuchsia::ui::views::ViewRef view_ref,
    fuchsia::ui::input::ImeServiceHandle ime_service,
    fuchsia::ui::input3::KeyboardHandle keyboard,
    std::function<void(std::unique_ptr<flutter::PlatformMessage>)>
        dispatch_callback)
    : dispatch_callback_(dispatch_callback),
      ime_client_(this),
      text_sync_service_(ime_service.Bind()),
      keyboard_listener_binding_(this),
      keyboard_(keyboard.Bind()) {
  // Register all error handlers.
  SetInterfaceErrorHandler(ime_, "Input Method Editor");
  SetInterfaceErrorHandler(ime_client_, "IME Client");
  SetInterfaceErrorHandler(text_sync_service_, "Text Sync Service");
  SetInterfaceErrorHandler(keyboard_listener_binding_, "Keyboard Listener");
  SetInterfaceErrorHandler(keyboard_, "Keyboard");

  // Configure keyboard listener.
  keyboard_->AddListener(std::move(view_ref),
                         keyboard_listener_binding_.NewBinding(), [] {});
}

void TextDelegate::ActivateIme() {
  ActivateIme(requested_text_action_.value_or(
      fuchsia::ui::input::InputMethodAction::DONE));
}

void TextDelegate::ActivateIme(fuchsia::ui::input::InputMethodAction action) {
  FML_DCHECK(last_text_state_.has_value());

  requested_text_action_ = action;
  text_sync_service_->GetInputMethodEditor(
      fuchsia::ui::input::KeyboardType::TEXT,  // keyboard type
      action,                                  // input method action
      last_text_state_.value(),                // initial state
      ime_client_.NewBinding(),                // client
      ime_.NewRequest()                        // editor
  );
}

void TextDelegate::DeactivateIme() {
  if (ime_) {
    text_sync_service_->HideKeyboard();
    ime_ = nullptr;
  }
  if (ime_client_.is_bound()) {
    ime_client_.Unbind();
  }
}

// |fuchsia::ui::input::InputMethodEditorClient|
void TextDelegate::DidUpdateState(
    fuchsia::ui::input::TextInputState state,
    std::unique_ptr<fuchsia::ui::input::InputEvent> input_event) {
  rapidjson::Document document;
  auto& allocator = document.GetAllocator();
  rapidjson::Value encoded_state(rapidjson::kObjectType);
  encoded_state.AddMember("text", state.text, allocator);
  encoded_state.AddMember("selectionBase", state.selection.base, allocator);
  encoded_state.AddMember("selectionExtent", state.selection.extent, allocator);
  switch (state.selection.affinity) {
    case fuchsia::ui::input::TextAffinity::UPSTREAM:
      encoded_state.AddMember("selectionAffinity",
                              rapidjson::Value("TextAffinity.upstream"),
                              allocator);
      break;
    case fuchsia::ui::input::TextAffinity::DOWNSTREAM:
      encoded_state.AddMember("selectionAffinity",
                              rapidjson::Value("TextAffinity.downstream"),
                              allocator);
      break;
  }
  encoded_state.AddMember("selectionIsDirectional", true, allocator);
  encoded_state.AddMember("composingBase", state.composing.start, allocator);
  encoded_state.AddMember("composingExtent", state.composing.end, allocator);

  rapidjson::Value args(rapidjson::kArrayType);
  args.PushBack(current_text_input_client_, allocator);
  args.PushBack(encoded_state, allocator);

  document.SetObject();
  document.AddMember("method",
                     rapidjson::Value("TextInputClient.updateEditingState"),
                     allocator);
  document.AddMember("args", args, allocator);

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);

  const uint8_t* data = reinterpret_cast<const uint8_t*>(buffer.GetString());
  dispatch_callback_(std::make_unique<flutter::PlatformMessage>(
      kTextInputChannel,                                 // channel
      fml::MallocMapping::Copy(data, buffer.GetSize()),  // message
      nullptr)                                           // response
  );
  last_text_state_ = std::move(state);
}

// |fuchsia::ui::input::InputMethodEditorClient|
void TextDelegate::OnAction(fuchsia::ui::input::InputMethodAction action) {
  rapidjson::Document document;
  auto& allocator = document.GetAllocator();

  rapidjson::Value args(rapidjson::kArrayType);
  args.PushBack(current_text_input_client_, allocator);

  const std::string action_string = IntoTextInputAction(action);
  args.PushBack(rapidjson::Value{}.SetString(action_string.c_str(),
                                             action_string.length()),
                allocator);

  document.SetObject();
  document.AddMember(
      "method", rapidjson::Value("TextInputClient.performAction"), allocator);
  document.AddMember("args", args, allocator);

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);

  const uint8_t* data = reinterpret_cast<const uint8_t*>(buffer.GetString());
  dispatch_callback_(std::make_unique<flutter::PlatformMessage>(
      kTextInputChannel,                                 // channel
      fml::MallocMapping::Copy(data, buffer.GetSize()),  // message
      nullptr)                                           // response
  );
}

// Channel handler for kTextInputChannel
bool TextDelegate::HandleFlutterTextInputChannelPlatformMessage(
    std::unique_ptr<flutter::PlatformMessage> message) {
  FML_DCHECK(message->channel() == kTextInputChannel);
  const auto& data = message->data();

  rapidjson::Document document;
  document.Parse(reinterpret_cast<const char*>(data.GetMapping()),
                 data.GetSize());
  if (document.HasParseError() || !document.IsObject()) {
    return false;
  }
  auto root = document.GetObject();
  auto method = root.FindMember("method");
  if (method == root.MemberEnd() || !method->value.IsString()) {
    return false;
  }

  if (method->value == "TextInput.show") {
    if (ime_) {
      text_sync_service_->ShowKeyboard();
    }
  } else if (method->value == "TextInput.hide") {
    if (ime_) {
      text_sync_service_->HideKeyboard();
    }
  } else if (method->value == "TextInput.setClient") {
    // Sample "setClient" message:
    //
    // {
    //   "method": "TextInput.setClient",
    //   "args": [
    //      7,
    //      {
    //        "inputType": {
    //          "name": "TextInputType.multiline",
    //          "signed":null,
    //          "decimal":null
    //        },
    //        "readOnly": false,
    //        "obscureText": false,
    //        "autocorrect":true,
    //        "smartDashesType":"1",
    //        "smartQuotesType":"1",
    //        "enableSuggestions":true,
    //        "enableInteractiveSelection":true,
    //        "actionLabel":null,
    //        "inputAction":"TextInputAction.newline",
    //        "textCapitalization":"TextCapitalization.none",
    //        "keyboardAppearance":"Brightness.dark",
    //        "enableIMEPersonalizedLearning":true,
    //        "enableDeltaModel":false
    //     }
    //  ]
    // }

    current_text_input_client_ = 0;
    DeactivateIme();
    auto args = root.FindMember("args");
    if (args == root.MemberEnd() || !args->value.IsArray() ||
        args->value.Size() != 2)
      return false;
    const auto& configuration = args->value[1];
    if (!configuration.IsObject()) {
      return false;
    }
    // TODO(abarth): Read the keyboard type from the configuration.
    current_text_input_client_ = args->value[0].GetInt();

    auto initial_text_input_state = fuchsia::ui::input::TextInputState{};
    initial_text_input_state.text = "";
    last_text_state_ = std::move(initial_text_input_state);

    const auto configuration_object = configuration.GetObject();
    if (!configuration_object.HasMember(kInputActionKey)) {
      return false;
    }
    const auto& action_object = configuration_object[kInputActionKey];
    if (!action_object.IsString()) {
      return false;
    }
    const auto action_string =
        std::string(action_object.GetString(), action_object.GetStringLength());
    ActivateIme(IntoInputMethodAction(std::move(action_string)));
  } else if (method->value == "TextInput.setEditingState") {
    if (ime_) {
      auto args_it = root.FindMember("args");
      if (args_it == root.MemberEnd() || !args_it->value.IsObject()) {
        return false;
      }
      const auto& args = args_it->value;
      fuchsia::ui::input::TextInputState state;
      state.text = "";
      // TODO(abarth): Deserialize state.
      auto text = args.FindMember("text");
      if (text != args.MemberEnd() && text->value.IsString()) {
        state.text = text->value.GetString();
      }
      auto selection_base = args.FindMember("selectionBase");
      if (selection_base != args.MemberEnd() && selection_base->value.IsInt()) {
        state.selection.base = selection_base->value.GetInt();
      }
      auto selection_extent = args.FindMember("selectionExtent");
      if (selection_extent != args.MemberEnd() &&
          selection_extent->value.IsInt()) {
        state.selection.extent = selection_extent->value.GetInt();
      }
      auto selection_affinity = args.FindMember("selectionAffinity");
      if (selection_affinity != args.MemberEnd() &&
          selection_affinity->value.IsString() &&
          selection_affinity->value == "TextAffinity.upstream") {
        state.selection.affinity = fuchsia::ui::input::TextAffinity::UPSTREAM;
      } else {
        state.selection.affinity = fuchsia::ui::input::TextAffinity::DOWNSTREAM;
      }
      // We ignore selectionIsDirectional because that concept doesn't exist on
      // Fuchsia.
      auto composing_base = args.FindMember("composingBase");
      if (composing_base != args.MemberEnd() && composing_base->value.IsInt()) {
        state.composing.start = composing_base->value.GetInt();
      }
      auto composing_extent = args.FindMember("composingExtent");
      if (composing_extent != args.MemberEnd() &&
          composing_extent->value.IsInt()) {
        state.composing.end = composing_extent->value.GetInt();
      }
      ime_->SetState(std::move(state));
    }
  } else if (method->value == "TextInput.clearClient") {
    current_text_input_client_ = 0;
    last_text_state_ = std::nullopt;
    requested_text_action_ = std::nullopt;
    DeactivateIme();
  } else if (method->value == "TextInput.setCaretRect" ||
             method->value == "TextInput.setEditableSizeAndTransform" ||
             method->value == "TextInput.setMarkedTextRect" ||
             method->value == "TextInput.setStyle") {
    // We don't have these methods implemented and they get
    // sent a lot during text input, so we create an empty case for them
    // here to avoid "Unknown flutter/textinput method TextInput.*"
    // log spam.
    //
    // TODO(fxb/101619): We should implement these.
  } else {
    FML_LOG(ERROR) << "Unknown " << message->channel() << " method "
                   << method->value.GetString();
  }
  // Complete with an empty response.
  return false;
}

// |fuchsia::ui:input3::KeyboardListener|
void TextDelegate::OnKeyEvent(
    fuchsia::ui::input3::KeyEvent key_event,
    fuchsia::ui::input3::KeyboardListener::OnKeyEventCallback callback) {
  const char* type = nullptr;
  switch (key_event.type()) {
    case fuchsia::ui::input3::KeyEventType::PRESSED:
      type = "keydown";
      break;
    case fuchsia::ui::input3::KeyEventType::RELEASED:
      type = "keyup";
      break;
    case fuchsia::ui::input3::KeyEventType::SYNC:
      // SYNC means the key was pressed while focus was not on this application.
      // This should possibly behave like PRESSED in the future, though it
      // doesn't hurt to ignore it today.
    case fuchsia::ui::input3::KeyEventType::CANCEL:
      // CANCEL means the key was released while focus was not on this
      // application.
      // This should possibly behave like RELEASED in the future to ensure that
      // a key is not repeated forever if it is pressed while focus is lost.
    default:
      break;
  }
  if (type == nullptr) {
    FML_VLOG(1) << "Unknown key event phase.";
    callback(fuchsia::ui::input3::KeyEventStatus::NOT_HANDLED);
    return;
  }
  keyboard_translator_.ConsumeEvent(std::move(key_event));

  rapidjson::Document document;
  auto& allocator = document.GetAllocator();
  document.SetObject();
  document.AddMember("type", rapidjson::Value(type, strlen(type)), allocator);
  document.AddMember("keymap", rapidjson::Value("fuchsia"), allocator);
  document.AddMember("hidUsage", keyboard_translator_.LastHIDUsage(),
                     allocator);
  document.AddMember("codePoint", keyboard_translator_.LastCodePoint(),
                     allocator);
  document.AddMember("modifiers", keyboard_translator_.Modifiers(), allocator);
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);

  const uint8_t* data = reinterpret_cast<const uint8_t*>(buffer.GetString());
  dispatch_callback_(std::make_unique<flutter::PlatformMessage>(
      kKeyEventChannel,                                  // channel
      fml::MallocMapping::Copy(data, buffer.GetSize()),  // data
      nullptr)                                           // response
  );
  callback(fuchsia::ui::input3::KeyEventStatus::HANDLED);
}
}  // namespace flutter_runner
