// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_KEYBOARD_KEY_HANDLER_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_KEYBOARD_KEY_HANDLER_H_

#include <windows.h>
#include <deque>
#include <map>
#include <memory>
#include <string>

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/binary_messenger.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/encodable_value.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/method_channel.h"
#include "flutter/shell/platform/windows/keyboard_handler_base.h"

namespace flutter {

// Handles key events.
//
// This class detects whether an incoming event is a redispatched one,
// dispatches native events to delegates and collect their responses,
// and redispatches events unhandled by Flutter back to the system.
// See |KeyboardHook| for more information about dispatching.
//
// This class owns multiple |KeyboardKeyHandlerDelegate|s, which
// implements the exact behavior to asynchronously handle events. In
// reality, this design is only to support sending events through
// "channel" (RawKeyEvent) and "embedder" (KeyEvent) simultaneously,
// the former of which shall be removed after the deprecation window
// of the RawKeyEvent system.
class KeyboardKeyHandler : public KeyboardHandlerBase {
 public:
  // An interface for concrete definition of how to asynchronously handle key
  // events.
  class KeyboardKeyHandlerDelegate {
   public:
    // Defines how to how to asynchronously handle key events.
    //
    // |KeyboardHook| should invoke |callback| with the response (whether the
    // event is handled) later for exactly once.
    virtual void KeyboardHook(int key,
                              int scancode,
                              int action,
                              char32_t character,
                              bool extended,
                              bool was_down,
                              KeyEventCallback callback) = 0;

    virtual void SyncModifiersIfNeeded(int modifiers_state) = 0;

    virtual std::map<uint64_t, uint64_t> GetPressedState() = 0;

    virtual ~KeyboardKeyHandlerDelegate();
  };

  // Create a |KeyboardKeyHandler| by specifying the messenger
  // through which the messages are sent.
  explicit KeyboardKeyHandler(flutter::BinaryMessenger* messenger);

  ~KeyboardKeyHandler();

  // Init the keyboard channel used to answer to pressed state queries.
  void InitKeyboardChannel();

  // Add a delegate that handles events received by |KeyboardHook|.
  void AddDelegate(std::unique_ptr<KeyboardKeyHandlerDelegate> delegate);

  // Synthesize modifier keys events if needed.
  void SyncModifiersIfNeeded(int modifiers_state) override;

  // Handles a key event.
  //
  // Returns whether this handler claims to handle the event, which is true if
  // and only if the event is a non-synthesized event.
  //
  // Windows requires a synchronous response of whether a key event should be
  // handled, while the query to Flutter is always asynchronous. This is
  // resolved by the "redispatching" algorithm: by default, the response to a
  // fresh event is always true. The event is then sent to the framework.
  // If the framework later decides not to handle the event, this class will
  // create an identical event and dispatch it to the system, and remember all
  // synthesized events. The fist time an exact event (by |ComputeEventHash|) is
  // received in the future, the new event is considered a synthesized one,
  // causing |KeyboardHook| to return false to fall back to other keyboard
  // handlers.
  //
  // Whether a non-synthesized event is considered handled by the framework is
  // decided by dispatching the event to all delegates, simultaneously,
  // unconditionally, in insertion order, and collecting their responses later.
  // It's not supported to prevent any delegates to process the events, because
  // in reality this will only support 2 hardcoded delegates, and only to
  // continue supporting the legacy API (channel) during the deprecation window,
  // after which the channel delegate should be removed.
  //
  // Inherited from |KeyboardHandlerBase|.
  void KeyboardHook(int key,
                    int scancode,
                    int action,
                    char32_t character,
                    bool extended,
                    bool was_down,
                    KeyEventCallback callback) override;

  // Returns the keyboard pressed state.
  //
  // Returns the keyboard pressed state. The dictionary contains one entry per
  // pressed keys, mapping from the logical key to the physical key.
  std::map<uint64_t, uint64_t> GetPressedState() override;

 private:
  struct PendingEvent {
    // Self-incrementing ID attached to an event sent to the framework.
    uint64_t sequence_id;
    // The number of delegates that haven't replied.
    size_t unreplied;
    // Whether any replied delegates reported true (handled).
    bool any_handled;

    // Where to report the delegates' result to.
    //
    // Typically a callback provided by KeyboardManager32.
    KeyEventCallback callback;
  };

  void ResolvePendingEvent(uint64_t sequence_id, bool handled);

  // Called when a method is called on |channel_|;
  void HandleMethodCall(
      const flutter::MethodCall<EncodableValue>& method_call,
      std::unique_ptr<flutter::MethodResult<EncodableValue>> result);

  std::vector<std::unique_ptr<KeyboardKeyHandlerDelegate>> delegates_;

  // The queue of key events that have been sent to the framework but have not
  // yet received a response.
  std::deque<std::unique_ptr<PendingEvent>> pending_responds_;

  // The sequence_id attached to the last event sent to the framework.
  uint64_t last_sequence_id_;

  // The Flutter system channel for keyboard state messages.
  std::unique_ptr<flutter::MethodChannel<EncodableValue>> channel_;

  FML_DISALLOW_COPY_AND_ASSIGN(KeyboardKeyHandler);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_KEYBOARD_KEY_HANDLER_H_
