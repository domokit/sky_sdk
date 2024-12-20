// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/keyboard_key_handler.h"

#include <windows.h>

#include "flutter/fml/logging.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/standard_method_codec.h"
#include "flutter/shell/platform/windows/keyboard_utils.h"

namespace flutter {

namespace {

// The maximum number of pending events to keep before
// emitting a warning on the console about unhandled events.
static constexpr int kMaxPendingEvents = 1000;

// The name of the channel for keyboard state queries.
static constexpr char kChannelName[] = "flutter/keyboard";

static constexpr char kGetKeyboardStateMethod[] = "getKeyboardState";

}  // namespace

KeyboardKeyHandler::KeyboardKeyHandlerDelegate::~KeyboardKeyHandlerDelegate() =
    default;

KeyboardKeyHandler::KeyboardKeyHandler(flutter::BinaryMessenger* messenger)
    : last_sequence_id_(1),
      channel_(std::make_unique<MethodChannel<EncodableValue>>(
          messenger,
          kChannelName,
          &StandardMethodCodec::GetInstance())) {}

KeyboardKeyHandler::~KeyboardKeyHandler() = default;

void KeyboardKeyHandler::InitKeyboardChannel() {
  channel_->SetMethodCallHandler(
      [this](const MethodCall<EncodableValue>& call,
             std::unique_ptr<MethodResult<EncodableValue>> result) {
        HandleMethodCall(call, std::move(result));
      });
}

void KeyboardKeyHandler::HandleMethodCall(
    const MethodCall<EncodableValue>& method_call,
    std::unique_ptr<MethodResult<EncodableValue>> result) {
  const std::string& method = method_call.method_name();
  if (method.compare(kGetKeyboardStateMethod) == 0) {
    EncodableMap value;
    const auto& pressed_state = GetPressedState();
    for (const auto& pressed_key : pressed_state) {
      EncodableValue physical_value(static_cast<long long>(pressed_key.first));
      EncodableValue logical_value(static_cast<long long>(pressed_key.second));
      value[physical_value] = logical_value;
    }
    result->Success(EncodableValue(value));
  } else {
    result->NotImplemented();
  }
}

void KeyboardKeyHandler::AddDelegate(
    std::unique_ptr<KeyboardKeyHandlerDelegate> delegate) {
  delegates_.push_back(std::move(delegate));
}

void KeyboardKeyHandler::SyncModifiersIfNeeded(int modifiers_state) {
  // Only call SyncModifierIfNeeded on the key embedder handler.
  auto& key_embedder_handler = delegates_.front();
  key_embedder_handler->SyncModifiersIfNeeded(modifiers_state);
}

std::map<uint64_t, uint64_t> KeyboardKeyHandler::GetPressedState() {
  // The embedder responder is the first element in delegates_.
  auto& key_embedder_handler = delegates_.front();
  return key_embedder_handler->GetPressedState();
}

void KeyboardKeyHandler::KeyboardHook(int key,
                                      int scancode,
                                      int action,
                                      char32_t character,
                                      bool extended,
                                      bool was_down,
                                      KeyEventCallback callback) {
  std::unique_ptr<PendingEvent> incoming = std::make_unique<PendingEvent>();

  uint64_t sequence_id = ++last_sequence_id_;
  incoming->sequence_id = sequence_id;
  incoming->unreplied = delegates_.size();
  incoming->any_handled = false;
  incoming->callback = std::move(callback);

  if (pending_responds_.size() > kMaxPendingEvents) {
    FML_LOG(ERROR)
        << "There are " << pending_responds_.size()
        << " keyboard events that have not yet received a response from the "
        << "framework. Are responses being sent?";
  }
  pending_responds_.push_back(std::move(incoming));

  for (const auto& delegate : delegates_) {
    delegate->KeyboardHook(key, scancode, action, character, extended, was_down,
                           [sequence_id, this](bool handled) {
                             ResolvePendingEvent(sequence_id, handled);
                           });
  }

  // |ResolvePendingEvent| might trigger redispatching synchronously,
  // which might occur before |KeyboardHook| is returned. This won't
  // make events out of order though, because |KeyboardHook| will always
  // return true at this time, preventing this event from affecting
  // others.
}

void KeyboardKeyHandler::ResolvePendingEvent(uint64_t sequence_id,
                                             bool handled) {
  // Find the pending event
  for (auto iter = pending_responds_.begin(); iter != pending_responds_.end();
       ++iter) {
    if ((*iter)->sequence_id == sequence_id) {
      PendingEvent& event = **iter;
      event.any_handled = event.any_handled || handled;
      event.unreplied -= 1;
      FML_DCHECK(event.unreplied >= 0)
          << "Pending events must have unreplied count > 0";
      // If all delegates have replied, report if any of them handled the event.
      if (event.unreplied == 0) {
        std::unique_ptr<PendingEvent> event_ptr = std::move(*iter);
        pending_responds_.erase(iter);
        event.callback(event.any_handled);
      }
      // Return here; |iter| can't do ++ after erase.
      return;
    }
  }
  // The pending event should always be found.
  FML_LOG(FATAL) << "Could not find pending key event for sequence ID "
                 << sequence_id;
}

}  // namespace flutter
