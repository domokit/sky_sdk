// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_KEYBOARD_UTILS_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_KEYBOARD_UTILS_H_

#include <stdint.h>

#include <string>

namespace flutter {

constexpr int kShift = 1 << 0;
constexpr int kControl = 1 << 3;
constexpr int kScanCodeShiftLeft = 0x2a;
constexpr int kScanCodeShiftRight = 0x36;
constexpr int kKeyCodeShiftLeft = 0xa0;
constexpr int kScanCodeControlLeft = 0x1d;
constexpr int kScanCodeControlRight = 0xe01d;
constexpr int kKeyCodeControlLeft = 0xa2;

// The bit of a mapped character in a WM_KEYDOWN message that indicates the
// character is a dead key.
//
// When a dead key is pressed, the WM_KEYDOWN's lParam is mapped to a special
// value: the "normal character" | 0x80000000.  For example, when pressing
// "dead key caret" (one that makes the following e into ê), its mapped
// character is 0x8000005E. "Reverting" it gives 0x5E, which is character '^'.
constexpr int kDeadKeyCharMask = 0x80000000;

// Revert the "character" for a dead key to its normal value, or the argument
// unchanged otherwise.
inline uint32_t UndeadChar(uint32_t ch) {
  return ch & ~kDeadKeyCharMask;
}

// Encode a Unicode codepoint into a UTF-16 string.
//
// If the codepoint is invalid, this function throws an assertion error, and
// returns an empty string.
std::u16string EncodeUtf16(char32_t character);

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_KEYBOARD_UTILS_H_
