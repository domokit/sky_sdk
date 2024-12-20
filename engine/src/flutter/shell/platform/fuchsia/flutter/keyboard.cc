// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/fuchsia/flutter/keyboard.h"

#include <fuchsia/input/cpp/fidl.h>
#include <fuchsia/ui/input/cpp/fidl.h>
#include <fuchsia/ui/input3/cpp/fidl.h>

#include <iostream>

namespace flutter_runner {

using fuchsia::input::Key;
using fuchsia::ui::input::kModifierCapsLock;
using fuchsia::ui::input::kModifierLeftAlt;
using fuchsia::ui::input::kModifierLeftControl;
using fuchsia::ui::input::kModifierLeftShift;
using fuchsia::ui::input::kModifierNone;
using fuchsia::ui::input::kModifierRightAlt;
using fuchsia::ui::input::kModifierRightControl;
using fuchsia::ui::input::kModifierRightShift;
using fuchsia::ui::input3::KeyEvent;
using fuchsia::ui::input3::KeyEventType;

namespace {

// A simple keymap from a QWERTY keyboard to code points. A value 0 means no
// code point has been assigned for the respective keypress. Column 0 is the
// code point without a level modifier active, and Column 1 is the code point
// with a level modifier (e.g. Shift key) active.
static const uint32_t QWERTY_TO_CODE_POINTS[][2] = {
    // 0x00
    {},
    {},
    {},
    {},
    // 0x04,
    {'a', 'A'},
    {'b', 'B'},
    {'c', 'C'},
    {'d', 'D'},
    // 0x08
    {'e', 'E'},
    {'f', 'F'},
    {'g', 'G'},
    {'h', 'H'},
    // 0x0c
    {'i', 'I'},
    {'j', 'J'},
    {'k', 'K'},
    {'l', 'L'},
    // 0x10
    {'m', 'M'},
    {'n', 'N'},
    {'o', 'O'},
    {'p', 'P'},
    // 0x14
    {'q', 'Q'},
    {'r', 'R'},
    {'s', 'S'},
    {'t', 'T'},
    // 0x18
    {'u', 'U'},
    {'v', 'V'},
    {'w', 'W'},
    {'x', 'X'},
    // 0x1c
    {'y', 'Y'},
    {'z', 'Z'},
    {'1', '!'},
    {'2', '@'},
    // 0x20
    {'3', '#'},
    {'4', '$'},
    {'5', '%'},
    {'6', '^'},
    // 0x24
    {'7', '&'},
    {'8', '*'},
    {'9', '('},
    {'0', ')'},
    // 0x28
    {},
    {},
    {},
    {},
    // 0x2c
    {' ', ' '},
    {'-', '_'},
    {'=', '+'},
    {'[', '{'},
    // 0x30
    {']', '}'},
    {'\\', '|'},
    {},
    {';', ':'},
    // 0x34
    {'\'', '"'},
    {'`', '~'},
    {',', '<'},
    {'.', '>'},
    // 0x38
    {'/', '?'},
    {},
    {},
    {},
    // 0x3c
    {},
    {},
    {},
    {},
    // 0x40
    {},
    {},
    {},
    {},
    // 0x44
    {},
    {},
    {},
    {},
    // 0x48
    {},
    {},
    {},
    {},
    // 0x4c
    {},
    {},
    {},
    {},
    // 0x50
    {},
    {},
    {},
    {},
    // 0x54
    {'/', 0},
    {'*', 0},
    {'-', 0},
    {'+', 0},
    // 0x58
    {},
    {'1', 0},
    {'2', 0},
    {'3', 0},
    // 0x5c
    {'4', 0},
    {'5', 0},
    {'6', 0},
    {'7', 0},
    // 0x60
    {'8', 0},
    {'9', 0},
    {'0', 0},
    {'.', 0},
};

}  // namespace

Keyboard::Keyboard()
    : any_events_received_(false),
      stateful_caps_lock_(false),
      left_shift_(false),
      right_shift_(false),
      left_alt_(false),
      right_alt_(false),
      left_ctrl_(false),
      right_ctrl_(false),
      last_event_() {}

bool Keyboard::ConsumeEvent(KeyEvent event) {
  if (!event.has_type()) {
    return false;
  }
  if (!event.has_key() && !event.has_key_meaning()) {
    return false;
  }
  // Check if the time sequence of the events is correct.
  last_event_ = std::move(event);
  any_events_received_ = true;

  if (!event.has_key()) {
    // The key only has key meaning.  Key meaning currently can not
    // update the modifier state, so we just short-circuit the table
    // below.
    return true;
  }

  const Key& key = last_event_.key();
  const KeyEventType& event_type = last_event_.type();
  switch (event_type) {
    // For modifier keys, a SYNC is the same as a press.
    case KeyEventType::SYNC:
      switch (key) {
        case Key::CAPS_LOCK:
          stateful_caps_lock_ = true;
          break;
        case Key::LEFT_ALT:
          left_alt_ = true;
          break;
        case Key::LEFT_CTRL:
          left_ctrl_ = true;
          break;
        case Key::LEFT_SHIFT:
          left_shift_ = true;
          break;
        case Key::RIGHT_ALT:
          right_alt_ = true;
          break;
        case Key::RIGHT_CTRL:
          right_ctrl_ = true;
          break;
        case Key::RIGHT_SHIFT:
          right_shift_ = true;
          break;
        default:
          // no-op
          break;
      }
      break;
    case KeyEventType::PRESSED:
      switch (key) {
        case Key::CAPS_LOCK:
          stateful_caps_lock_ = !stateful_caps_lock_;
          break;
        case Key::LEFT_ALT:
          left_alt_ = true;
          break;
        case Key::LEFT_CTRL:
          left_ctrl_ = true;
          break;
        case Key::LEFT_SHIFT:
          left_shift_ = true;
          break;
        case Key::RIGHT_ALT:
          right_alt_ = true;
          break;
        case Key::RIGHT_CTRL:
          right_ctrl_ = true;
          break;
        case Key::RIGHT_SHIFT:
          right_shift_ = true;
          break;
        default:
          // No-op
          break;
      }
      break;
    case KeyEventType::RELEASED:
      switch (key) {
        case Key::CAPS_LOCK:
          // No-op.
          break;
        case Key::LEFT_ALT:
          left_alt_ = false;
          break;
        case Key::LEFT_CTRL:
          left_ctrl_ = false;
          break;
        case Key::LEFT_SHIFT:
          left_shift_ = false;
          break;
        case Key::RIGHT_ALT:
          right_alt_ = false;
          break;
        case Key::RIGHT_CTRL:
          right_ctrl_ = false;
          break;
        case Key::RIGHT_SHIFT:
          right_shift_ = false;
          break;
        default:
          // No-op
          break;
      }
      break;
    case KeyEventType::CANCEL:
      // No-op?
      break;
    default:
      // No-op
      break;
  }
  return true;
}

bool Keyboard::IsShift() {
  return left_shift_ | right_shift_ | stateful_caps_lock_;
}

bool Keyboard::IsKeys() {
  return LastHIDUsagePage() == 0x7;
}

uint32_t Keyboard::Modifiers() {
  return kModifierNone + (kModifierLeftShift * left_shift_) +
         (kModifierLeftAlt * left_alt_) + (kModifierLeftControl * left_ctrl_) +
         (kModifierRightShift * right_shift_) +
         (kModifierRightAlt * right_alt_) +
         (kModifierRightControl * right_ctrl_) +
         (kModifierCapsLock * stateful_caps_lock_);
}

uint32_t Keyboard::LastCodePoint() {
  // If the key has a meaning, and if the meaning is a code point, always have
  // that code point take precedence over any other keymap.
  if (last_event_.has_key_meaning()) {
    const auto& key_meaning = last_event_.key_meaning();
    if (key_meaning.is_codepoint()) {
      return key_meaning.codepoint();
    }
  }
  static const int qwerty_map_size =
      sizeof(QWERTY_TO_CODE_POINTS) / sizeof(QWERTY_TO_CODE_POINTS[0]);
  if (!IsKeys()) {
    return 0;
  }
  const auto usage = LastHIDUsageID();
  if (usage < qwerty_map_size) {
    return QWERTY_TO_CODE_POINTS[usage][IsShift() & 1];
  }
  // Any other keys don't have a code point.
  return 0;
}

uint32_t Keyboard::GetLastKey() {
  // For logical key determination, the physical key does not matter as long
  // as code point is set.
  // https://github.com/flutter/flutter/blob/570e39d38b799e91abe4f73f120ce494049c4ff0/packages/flutter/lib/src/services/raw_keyboard_fuchsia.dart#L71
  // It is not quite clear what happens to the physical key, though:
  // https://github.com/flutter/flutter/blob/570e39d38b799e91abe4f73f120ce494049c4ff0/packages/flutter/lib/src/services/raw_keyboard_fuchsia.dart#L88
  if (!last_event_.has_key()) {
    return 0;
  }
  return static_cast<uint32_t>(last_event_.key());
}

uint32_t Keyboard::LastHIDUsage() {
  return GetLastKey() & 0xFFFFFFFF;
}

uint16_t Keyboard::LastHIDUsageID() {
  return GetLastKey() & 0xFFFF;
}

uint16_t Keyboard::LastHIDUsagePage() {
  return (GetLastKey() >> 16) & 0xFFFF;
}

}  // namespace flutter_runner
