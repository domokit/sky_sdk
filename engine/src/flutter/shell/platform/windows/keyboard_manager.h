// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_KEYBOARD_MANAGER_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_KEYBOARD_MANAGER_H_

#include <windows.h>

#include <atomic>
#include <deque>
#include <functional>
#include <map>

#include "flutter/fml/macros.h"

namespace flutter {

// Handles keyboard and text messages on Win32.
//
// |KeyboardManager| consumes raw Win32 messages related to key and chars, and
// converts them to key calls (|WindowDelegate::OnKey|) and possibly text calls
// (|WindowDelegate::OnText|).
//
// |KeyboardManager| requires a |WindowDelegate| to define how to access Win32
// system calls (to allow mocking) and where to send the results of key calls
// and text calls to.
//
// Typically, |KeyboardManager| is owned by a |FlutterWindow|, which also
// implements the window delegate. The key calls and text calls are forwarded to
// the |FlutterWindow|, and consequently, to the |FlutterWindowsView|.
//
// ## Terminology
//
// The keyboard system uses the following terminology (which is different
// than Win32's terminology):
//
//  * Message: An invocation of |WndProc|, which consists of an
//    action, an lparam, and a wparam.
//  * Action: The type of a message.
//  * Session: One to three messages that should be processed together, such
//    as a key down message followed by char messages.
//  * Event: A FlutterKeyEvent/ui.KeyData sent to the framework.
//  * Call: A call to |WindowDelegate::OnKey| or |WindowDelegate::OnText|,
//    which contains semi-processed messages.
class KeyboardManager {
 public:
  // Define how the keyboard manager accesses Win32 system calls (to allow
  // mocking) and sends key calls and text calls.
  //
  // Typically implemented by |Window|.
  class WindowDelegate {
   public:
    using KeyEventCallback = std::function<void(bool)>;

    virtual ~WindowDelegate() = default;

    // Called when text input occurs.
    virtual void OnText(const std::u16string& text) = 0;

    // Called when raw keyboard input occurs.
    //
    // The `callback` must be called exactly once.
    virtual void OnKey(int key,
                       int scancode,
                       int action,
                       char32_t character,
                       bool extended,
                       bool was_down,
                       KeyEventCallback callback) = 0;

    // Win32's PeekMessage.
    //
    // Used to process key messages.
    virtual BOOL Win32PeekMessage(LPMSG lpMsg,
                                  UINT wMsgFilterMin,
                                  UINT wMsgFilterMax,
                                  UINT wRemoveMsg) = 0;

    // Win32's MapVirtualKey(*, MAPVK_VK_TO_CHAR).
    //
    // Used to process key messages.
    virtual uint32_t Win32MapVkToChar(uint32_t virtual_key) = 0;

    // Win32's |SendMessage|.
    //
    // Used to synthesize key messages.
    virtual UINT Win32DispatchMessage(UINT Msg,
                                      WPARAM wParam,
                                      LPARAM lParam) = 0;
  };

  using KeyEventCallback = WindowDelegate::KeyEventCallback;

  explicit KeyboardManager(WindowDelegate* delegate);

  // Processes Win32 messages related to keyboard and text.
  //
  // All messages related to keyboard and text should be sent here without
  // pre-processing, including WM_{SYS,}KEY{DOWN,UP} and WM_{SYS,}{DEAD,}CHAR.
  // Other message types will trigger assertion error.
  //
  // |HandleMessage| returns true if Flutter keyboard system decides to handle
  // this message synchronously. It doesn't mean that the Flutter framework
  // handles it, which is reported asynchronously later. Not handling this
  // message here usually means that this message is a redispatched message,
  // but there are other rare cases too. |Window| should forward unhandled
  // messages to |DefWindowProc|.
  bool HandleMessage(UINT const message,
                     WPARAM const wparam,
                     LPARAM const lparam);

 protected:
  struct Win32Message {
    UINT action;
    WPARAM wparam;
    LPARAM lparam;

    bool IsHighSurrogate() const { return IS_HIGH_SURROGATE(wparam); }

    bool IsLowSurrogate() const { return IS_LOW_SURROGATE(wparam); }

    bool IsGeneralKeyDown() const {
      return action == WM_KEYDOWN || action == WM_SYSKEYDOWN;
    }
  };

  struct PendingEvent {
    WPARAM key;
    uint8_t scancode;
    UINT action;
    char32_t character;
    bool extended;
    bool was_down;

    std::vector<Win32Message> session;
  };

  virtual void RedispatchEvent(std::unique_ptr<PendingEvent> event);

 private:
  using OnKeyCallback =
      std::function<void(std::unique_ptr<PendingEvent>, bool)>;

  struct PendingText {
    bool ready;
    std::u16string content;
    bool placeholder = false;
  };

  // Resume processing the pending events.
  //
  // If there is at least one pending event and no event is being processed,
  // the oldest pending event will be handed over to |PerformProcessEvent|.
  // After the event is processed, the next pending event will be automatically
  // started, until there are no pending events left.
  //
  // Otherwise, this call is a no-op.
  void ProcessNextEvent();

  // Process an event and call `callback` when it's completed.
  //
  // The `callback` is constructed by |ProcessNextEvent| to start the next
  // event, and must be called exactly once.
  void PerformProcessEvent(std::unique_ptr<PendingEvent> event,
                           std::function<void()> callback);

  // Handle the result of |WindowDelegate::OnKey|, possibly dispatching the text
  // result to |WindowDelegate::OnText| and then redispatching.
  //
  // The `pending_text` is either a valid iterator of `pending_texts`, or its
  // end(). In the latter case, this OnKey message does not contain a text.
  void HandleOnKeyResult(std::unique_ptr<PendingEvent> event,
                         bool framework_handled);

  // Dispatch the text content of a |PendingEvent| to |WindowDelegate::OnText|.
  //
  // If the content is empty of invalid, |WindowDelegate::OnText| will not be
  // called.
  void DispatchText(const PendingEvent& event);

  // Returns the type of the next WM message.
  //
  // The parameters limits the range of interested messages. See Win32's
  // |PeekMessage| for information.
  //
  // If there's no message, returns 0.
  UINT PeekNextMessageType(UINT wMsgFilterMin, UINT wMsgFilterMax);

  // Find an event in the redispatch list that matches the given one.
  //
  // If an matching event is found, removes the matching event from the
  // redispatch list, and returns true. Otherwise, returns false;
  bool RemoveRedispatchedMessage(UINT action, WPARAM wparam, LPARAM lparam);

  WindowDelegate* window_delegate_;

  // Keeps track of all messages during the current session.
  //
  // At the end of a session, it is moved to the `PendingEvent`, which is
  // passed to `WindowDelegate::OnKey`.
  std::vector<Win32Message> current_session_;

  // Whether the last event is a CtrlLeft key down.
  //
  // This is used to resolve a corner case described in |IsKeyDownAltRight|.
  bool last_key_is_ctrl_left_down;

  // The scancode of the last met CtrlLeft down.
  //
  // This is used to resolve a corner case described in |IsKeyDownAltRight|.
  uint8_t ctrl_left_scancode;

  // Whether a CtrlLeft up should be synthesized upon the next AltRight up.
  //
  // This is used to resolve a corner case described in |IsKeyDownAltRight|.
  bool should_synthesize_ctrl_left_up;

  // Store the messages coming from |HandleMessage|.
  //
  // Only one message is processed at a time. The next one will not start
  // until the framework has responded to the previous message.
  std::deque<std::unique_ptr<PendingEvent>> pending_events_;

  // Whether a message is being processed.
  std::atomic<bool> processing_event_;

  // The queue of messages that have been redispatched to the system but have
  // not yet been received for a second time.
  std::deque<Win32Message> pending_redispatches_;

  FML_DISALLOW_COPY_AND_ASSIGN(KeyboardManager);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_KEYBOARD_MANAGER_H_
