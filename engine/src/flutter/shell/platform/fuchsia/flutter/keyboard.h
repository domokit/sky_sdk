// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_KEYBOARD_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_KEYBOARD_H_

#include <fuchsia/ui/input3/cpp/fidl.h>

namespace flutter_runner {

// Keyboard handles the keyboard signals from fuchsia.ui.input3.  Specifically,
// input3 has no notion of a code point, and does not track stateful versions
// of the modifier keys.
class Keyboard final {
 public:
  explicit Keyboard();

  // Consumes the given keyboard event.  Keyboard will adjust the modifier
  // state based on the info given in the event.  Returns true if the event has
  // been integrated into the internal state successfully, or false otherwise.
  bool ConsumeEvent(fuchsia::ui::input3::KeyEvent event);

  // Gets the currently active modifier keys.
  uint32_t Modifiers();

  // Gets the last encountered code point.  The reported code point depends on
  // the state of the modifier keys.
  uint32_t LastCodePoint();

  // Gets the last encountered HID usage.  This is a 32-bit number, with the
  // upper 16 bits equal to `LastHidUsagePage()`, and the lower 16 bits equal
  // to `LastHIDUsageID()`.
  //
  // The key corresponding to A will have the usage 0x7004. This function will
  // return 0x7004 in that case.
  uint32_t LastHIDUsage();

  // Gets the last encountered HID usage page.
  //
  // The key corresponding to A will have the usage 0x7004. This function will
  // return 0x7 in that case.
  uint16_t LastHIDUsagePage();

  // Gets the last encountered HID usage ID.
  //
  // The key corresponding to A will have the usage 0x7004. This function will
  // return 0x4 in that case.
  uint16_t LastHIDUsageID();

 private:
  // Return true if any level shift is active.
  bool IsShift();

  // Returns true if the last key event was about a key that may have a code
  // point associated.
  bool IsKeys();

  // Returns the value of the last key as a uint32_t.
  // If there isn't such a value (as in the case of on-screen keyboards), this
  // will return a 0;
  uint32_t GetLastKey();

  // Set to false until any event is received.
  bool any_events_received_ : 1;

  // The flags below show the state of the keyboard modifiers after the last
  // event has been processed.  Stateful keys remain in the same state after
  // a release and require an additional press to toggle.
  bool stateful_caps_lock_ : 1;
  bool left_shift_ : 1;
  bool right_shift_ : 1;
  bool left_alt_ : 1;
  bool right_alt_ : 1;
  bool left_ctrl_ : 1;
  bool right_ctrl_ : 1;

  // The last received key event.  If any_events_received_ is not set, this is
  // not valid.
  fuchsia::ui::input3::KeyEvent last_event_;
};

}  // namespace flutter_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_KEYBOARD_H_
