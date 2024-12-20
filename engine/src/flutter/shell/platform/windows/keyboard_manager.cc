// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <memory>
#include <string>

#include "flutter/fml/logging.h"
#include "flutter/shell/platform/windows/keyboard_manager.h"
#include "flutter/shell/platform/windows/keyboard_utils.h"

namespace flutter {

namespace {

// The maximum number of pending events to keep before
// emitting a warning on the console about unhandled events.
constexpr int kMaxPendingEvents = 1000;

// Returns true if this key is an AltRight key down event.
//
// This is used to resolve an issue where an AltGr press causes CtrlLeft to hang
// when pressed, as reported in https://github.com/flutter/flutter/issues/78005.
//
// When AltGr is pressed (in a supporting layout such as Spanish), Win32 first
// fires a fake CtrlLeft down event, then an AltRight down event.
// This is significant because this fake CtrlLeft down event will not be paired
// with a up event, which is fine until Flutter redispatches the CtrlDown
// event, which Win32 then interprets as a real event, leaving both Win32 and
// the Flutter framework thinking that CtrlLeft is still pressed.
//
// To resolve this, Flutter recognizes this fake CtrlLeft down event using the
// following AltRight down event. Flutter then forges a CtrlLeft key up event
// immediately after the corresponding AltRight key up event.
//
// One catch is that it is impossible to distinguish the fake CtrlLeft down
// from a normal CtrlLeft down (followed by a AltRight down), since they
// contain the exactly same information, including the GetKeyState result.
// Fortunately, this will require the two events to occur *really* close, which
// would be rare, and a misrecognition would only cause a minor consequence
// where the CtrlLeft is released early; the later, real, CtrlLeft up event will
// be ignored.
bool IsKeyDownAltRight(int action, int virtual_key, bool extended) {
  return virtual_key == VK_RMENU && extended &&
         (action == WM_KEYDOWN || action == WM_SYSKEYDOWN);
}

// Returns true if this key is a key up event of AltRight.
//
// This is used to assist a corner case described in |IsKeyDownAltRight|.
bool IsKeyUpAltRight(int action, int virtual_key, bool extended) {
  return virtual_key == VK_RMENU && extended &&
         (action == WM_KEYUP || action == WM_SYSKEYUP);
}

// Returns true if this key is a key down event of CtrlLeft.
//
// This is used to assist a corner case described in |IsKeyDownAltRight|.
bool IsKeyDownCtrlLeft(int action, int virtual_key) {
  return virtual_key == VK_LCONTROL &&
         (action == WM_KEYDOWN || action == WM_SYSKEYDOWN);
}

// Returns if a character sent by Win32 is a dead key.
bool IsDeadKey(uint32_t ch) {
  return (ch & kDeadKeyCharMask) != 0;
}

char32_t CodePointFromSurrogatePair(wchar_t high, wchar_t low) {
  return 0x10000 + ((static_cast<char32_t>(high) & 0x000003FF) << 10) +
         (low & 0x3FF);
}

uint16_t ResolveKeyCode(uint16_t original, bool extended, uint8_t scancode) {
  switch (original) {
    case VK_SHIFT:
    case VK_LSHIFT:
      return MapVirtualKey(scancode, MAPVK_VSC_TO_VK_EX);
    case VK_MENU:
    case VK_LMENU:
      return extended ? VK_RMENU : VK_LMENU;
    case VK_CONTROL:
    case VK_LCONTROL:
      return extended ? VK_RCONTROL : VK_LCONTROL;
    default:
      return original;
  }
}

bool IsPrintable(uint32_t c) {
  constexpr char32_t kMinPrintable = ' ';
  constexpr char32_t kDelete = 0x7F;
  return c >= kMinPrintable && c != kDelete;
}

bool IsSysAction(UINT action) {
  return action == WM_SYSKEYDOWN || action == WM_SYSKEYUP ||
         action == WM_SYSCHAR || action == WM_SYSDEADCHAR;
}

}  // namespace

KeyboardManager::KeyboardManager(WindowDelegate* delegate)
    : window_delegate_(delegate),
      last_key_is_ctrl_left_down(false),
      should_synthesize_ctrl_left_up(false),
      processing_event_(false) {}

void KeyboardManager::RedispatchEvent(std::unique_ptr<PendingEvent> event) {
  for (const Win32Message& message : event->session) {
    // Never redispatch sys keys, because their original messages have been
    // passed to the system default processor.
    if (IsSysAction(message.action)) {
      continue;
    }
    pending_redispatches_.push_back(message);
    UINT result = window_delegate_->Win32DispatchMessage(
        message.action, message.wparam, message.lparam);
    if (result != 0) {
      FML_LOG(ERROR) << "Unable to synthesize event for keyboard event.";
    }
  }
  if (pending_redispatches_.size() > kMaxPendingEvents) {
    FML_LOG(ERROR)
        << "There are " << pending_redispatches_.size()
        << " keyboard events that have not yet received a response from the "
        << "framework. Are responses being sent?";
  }
}

bool KeyboardManager::RemoveRedispatchedMessage(UINT const action,
                                                WPARAM const wparam,
                                                LPARAM const lparam) {
  for (auto iter = pending_redispatches_.begin();
       iter != pending_redispatches_.end(); ++iter) {
    if (action == iter->action && wparam == iter->wparam) {
      pending_redispatches_.erase(iter);
      return true;
    }
  }
  return false;
}

bool KeyboardManager::HandleMessage(UINT const action,
                                    WPARAM const wparam,
                                    LPARAM const lparam) {
  if (RemoveRedispatchedMessage(action, wparam, lparam)) {
    return false;
  }
  switch (action) {
    case WM_DEADCHAR:
    case WM_SYSDEADCHAR:
    case WM_CHAR:
    case WM_SYSCHAR: {
      const Win32Message message =
          Win32Message{.action = action, .wparam = wparam, .lparam = lparam};
      current_session_.push_back(message);

      char32_t code_point;
      if (message.IsHighSurrogate()) {
        // A high surrogate is always followed by a low surrogate.  Process the
        // session later and consider this message as handled.
        return true;
      } else if (message.IsLowSurrogate()) {
        const Win32Message* last_message =
            current_session_.size() <= 1
                ? nullptr
                : &current_session_[current_session_.size() - 2];
        if (last_message == nullptr || !last_message->IsHighSurrogate()) {
          return false;
        }
        // A low surrogate always follows a high surrogate, marking the end of
        // a char session. Process the session after the if clause.
        code_point =
            CodePointFromSurrogatePair(last_message->wparam, message.wparam);
      } else {
        // A non-surrogate character always appears alone. Process the session
        // after the if clause.
        code_point = static_cast<wchar_t>(message.wparam);
      }

      // If this char message is preceded by a key down message, then dispatch
      // the key down message as a key down event first, and only dispatch the
      // OnText if the key down event is not handled.
      if (current_session_.front().IsGeneralKeyDown()) {
        const Win32Message first_message = current_session_.front();
        const uint8_t scancode = (lparam >> 16) & 0xff;
        const uint16_t key_code = first_message.wparam;
        const bool extended = ((lparam >> 24) & 0x01) == 0x01;
        const bool was_down = lparam & 0x40000000;
        // Certain key combinations yield control characters as WM_CHAR's
        // lParam. For example, 0x01 for Ctrl-A. Filter these characters. See
        // https://docs.microsoft.com/en-us/windows/win32/learnwin32/accelerator-tables
        char32_t character;
        if (action == WM_DEADCHAR || action == WM_SYSDEADCHAR) {
          // Mask the resulting char with kDeadKeyCharMask anyway, because in
          // rare cases the bit is *not* set (US INTL Shift-6 circumflex, see
          // https://github.com/flutter/flutter/issues/92654 .)
          character =
              window_delegate_->Win32MapVkToChar(key_code) | kDeadKeyCharMask;
        } else {
          character = IsPrintable(code_point) ? code_point : 0;
        }
        auto event = std::make_unique<PendingEvent>(PendingEvent{
            .key = key_code,
            .scancode = scancode,
            .action = static_cast<UINT>(action == WM_SYSCHAR ? WM_SYSKEYDOWN
                                                             : WM_KEYDOWN),
            .character = character,
            .extended = extended,
            .was_down = was_down,
            .session = std::move(current_session_),
        });

        pending_events_.push_back(std::move(event));
        ProcessNextEvent();

        // SYS messages must not be consumed by `HandleMessage` so that they are
        // forwarded to the system.
        return !IsSysAction(action);
      }

      // If the charcter session is not preceded by a key down message,
      // mark PendingEvent::action as WM_CHAR, informing |PerformProcessEvent|
      // to dispatch the text content immediately.
      //
      // Only WM_CHAR should be treated as characters. WM_SYS*CHAR are not part
      // of text input, and WM_DEADCHAR will be incorporated into a later
      // WM_CHAR with the full character.
      if (action == WM_CHAR) {
        auto event = std::make_unique<PendingEvent>(PendingEvent{
            .action = WM_CHAR,
            .character = code_point,
            .session = std::move(current_session_),
        });
        pending_events_.push_back(std::move(event));
        ProcessNextEvent();
      }
      return true;
    }

    case WM_KEYDOWN:
    case WM_SYSKEYDOWN:
    case WM_KEYUP:
    case WM_SYSKEYUP: {
      if (wparam == VK_PACKET) {
        return false;
      }

      const uint8_t scancode = (lparam >> 16) & 0xff;
      const bool extended = ((lparam >> 24) & 0x01) == 0x01;
      // If the key is a modifier, get its side.
      const uint16_t key_code = ResolveKeyCode(wparam, extended, scancode);
      const bool was_down = lparam & 0x40000000;

      // Detect a pattern of key events in order to forge a CtrlLeft up event.
      // See |IsKeyDownAltRight| for explanation.
      if (IsKeyDownAltRight(action, key_code, extended)) {
        if (last_key_is_ctrl_left_down) {
          should_synthesize_ctrl_left_up = true;
        }
      }
      if (IsKeyDownCtrlLeft(action, key_code)) {
        last_key_is_ctrl_left_down = true;
        ctrl_left_scancode = scancode;
        should_synthesize_ctrl_left_up = false;
      } else {
        last_key_is_ctrl_left_down = false;
      }
      if (IsKeyUpAltRight(action, key_code, extended)) {
        if (should_synthesize_ctrl_left_up) {
          should_synthesize_ctrl_left_up = false;
          const LPARAM lParam =
              (1 /* repeat_count */ << 0) | (ctrl_left_scancode << 16) |
              (0 /* extended */ << 24) | (1 /* prev_state */ << 30) |
              (1 /* transition */ << 31);
          window_delegate_->Win32DispatchMessage(WM_KEYUP, VK_CONTROL, lParam);
        }
      }

      current_session_.clear();
      current_session_.push_back(
          Win32Message{.action = action, .wparam = wparam, .lparam = lparam});
      const bool is_keydown_message =
          (action == WM_KEYDOWN || action == WM_SYSKEYDOWN);
      // Check if this key produces a character by peeking if this key down
      // message has a following char message. Certain key messages are not
      // followed by char messages even though `MapVirtualKey` returns a valid
      // character (such as Ctrl + Digit, see
      // https://github.com/flutter/flutter/issues/85587 ).
      unsigned int character = window_delegate_->Win32MapVkToChar(wparam);
      UINT next_key_action = PeekNextMessageType(WM_KEYFIRST, WM_KEYLAST);
      bool has_char_action =
          (next_key_action == WM_DEADCHAR ||
           next_key_action == WM_SYSDEADCHAR || next_key_action == WM_CHAR ||
           next_key_action == WM_SYSCHAR);
      if (character > 0 && is_keydown_message && has_char_action) {
        // This key down message has a following char message. Process this
        // session in the char message, because the character for the key call
        // should be decided by the char events. Consider this message as
        // handled.
        return true;
      }

      // This key down message is not followed by a char message. Conclude this
      // session.
      auto event = std::make_unique<PendingEvent>(PendingEvent{
          .key = key_code,
          .scancode = scancode,
          .action = action,
          .character = 0,
          .extended = extended,
          .was_down = was_down,
          .session = std::move(current_session_),
      });
      pending_events_.push_back(std::move(event));
      ProcessNextEvent();
      // SYS messages must not be consumed by `HandleMessage` so that they are
      // forwarded to the system.
      return !IsSysAction(action);
    }
    default:
      FML_LOG(FATAL) << "No event handler for keyboard event with action "
                     << action;
  }
  return false;
}

void KeyboardManager::ProcessNextEvent() {
  if (processing_event_ || pending_events_.empty()) {
    return;
  }
  processing_event_ = true;
  auto pending_event = std::move(pending_events_.front());
  pending_events_.pop_front();
  PerformProcessEvent(std::move(pending_event), [this] {
    FML_DCHECK(processing_event_);
    processing_event_ = false;
    ProcessNextEvent();
  });
}

void KeyboardManager::PerformProcessEvent(std::unique_ptr<PendingEvent> event,
                                          std::function<void()> callback) {
  // PendingEvent::action being WM_CHAR means this is a char message without
  // a preceding key message, and should be dispatched immediately.
  if (event->action == WM_CHAR) {
    DispatchText(*event);
    callback();
    return;
  }

  // A unique_ptr can't be sent into a lambda without C++23's
  // move_only_function. Until then, `event` is sent as a raw pointer, hoping
  // WindowDelegate::OnKey to correctly call it once and only once.
  PendingEvent* event_p = event.release();
  window_delegate_->OnKey(
      event_p->key, event_p->scancode, event_p->action, event_p->character,
      event_p->extended, event_p->was_down,
      [this, event_p, callback = std::move(callback)](bool handled) {
        HandleOnKeyResult(std::unique_ptr<PendingEvent>(event_p), handled);
        callback();
      });
}

void KeyboardManager::HandleOnKeyResult(std::unique_ptr<PendingEvent> event,
                                        bool framework_handled) {
  const UINT last_action = event->session.back().action;
  // SYS messages must not be redispached, and their text content is not
  // dispatched either.
  bool handled = framework_handled || IsSysAction(last_action);

  if (handled) {
    return;
  }

  // Only WM_CHAR should be treated as characters. WM_SYS*CHAR are not part of
  // text input, and WM_DEADCHAR will be incorporated into a later WM_CHAR with
  // the full character.
  if (last_action == WM_CHAR) {
    DispatchText(*event);
  }

  RedispatchEvent(std::move(event));
}

void KeyboardManager::DispatchText(const PendingEvent& event) {
  // Check if the character is printable based on the last wparam, which works
  // even if the last wparam is a low surrogate, because the only unprintable
  // keys defined by `IsPrintable` are certain characters at lower ASCII range.
  // These ASCII control characters are sent as WM_CHAR events for all control
  // key shortcuts.
  FML_DCHECK(!event.session.empty());
  bool is_printable = IsPrintable(event.session.back().wparam);
  bool valid = event.character != 0 && is_printable;
  if (valid) {
    auto text = EncodeUtf16(event.character);
    window_delegate_->OnText(text);
  }
}

UINT KeyboardManager::PeekNextMessageType(UINT wMsgFilterMin,
                                          UINT wMsgFilterMax) {
  MSG next_message;
  BOOL has_msg = window_delegate_->Win32PeekMessage(
      &next_message, wMsgFilterMin, wMsgFilterMax, PM_NOREMOVE);
  if (!has_msg) {
    return 0;
  }
  return next_message.message;
}

}  // namespace flutter
