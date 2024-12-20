// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_KEYCODEMAP_INTERNAL_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_KEYCODEMAP_INTERNAL_H_

#import <Cocoa/Cocoa.h>
#include <cinttypes>
#include <vector>

namespace flutter {

/**
 * Maps macOS-specific key code values representing |PhysicalKeyboardKey|.
 *
 * MacOS doesn't provide a scan code, but a virtual keycode to represent a physical key.
 */
extern const NSDictionary* keyCodeToPhysicalKey;

/**
 * A map from macOS key codes to Flutter's logical key values.
 *
 * This is used to derive logical keys that can't or shouldn't be derived from
 * |charactersIgnoringModifiers|.
 */
extern const NSDictionary* keyCodeToLogicalKey;

// Several mask constants. See KeyCodeMap.g.mm for their descriptions.

/**
 * Mask for the 32-bit value portion of the key code.
 */
extern const uint64_t kValueMask;

/**
 * The plane value for keys which have a Unicode representation.
 */
extern const uint64_t kUnicodePlane;

/**
 * The plane value for the private keys defined by the macOS embedding.
 */
extern const uint64_t kMacosPlane;

/**
 * Map |NSEvent.keyCode| to its corresponding bitmask of NSEventModifierFlags.
 *
 * This does not include CapsLock, for it is handled specially.
 */
extern const NSDictionary* keyCodeToModifierFlag;

/**
 * Map a bit of bitmask of NSEventModifierFlags to its corresponding
 * |NSEvent.keyCode|.
 *
 * This does not include CapsLock, for it is handled specially.
 */
extern const NSDictionary* modifierFlagToKeyCode;

/**
 * The physical key for CapsLock, which needs special handling.
 */
extern const uint64_t kCapsLockPhysicalKey;

/**
 * The logical key for CapsLock, which needs special handling.
 */
extern const uint64_t kCapsLockLogicalKey;

/**
 * Bits in |NSEvent.modifierFlags| indicating whether a modifier key is pressed.
 *
 * These constants are not written in the official documentation, but derived
 * from experiments. This is currently the only way to know whether a one-side
 * modifier key (such as ShiftLeft) is pressed, instead of the general combined
 * modifier state (such as Shift).
 */
typedef enum {
  kModifierFlagControlLeft = 0x1,
  kModifierFlagShiftLeft = 0x2,
  kModifierFlagShiftRight = 0x4,
  kModifierFlagMetaLeft = 0x8,
  kModifierFlagMetaRight = 0x10,
  kModifierFlagAltLeft = 0x20,
  kModifierFlagAltRight = 0x40,
  kModifierFlagControlRight = 0x200,
} ModifierFlag;

/**
 * A character that Flutter wants to derive layout for, and guides on how to
 * derive it.
 */
typedef struct {
  // The key code for a key that prints `keyChar` in the US keyboard layout.
  uint16_t keyCode;

  // The printable string to derive logical key for.
  uint64_t keyChar;

  // If the goal is mandatory, the keyboard manager will make sure to find a
  // logical key for this character, falling back to the US keyboard layout.
  bool mandatory;
} LayoutGoal;

/**
 * All keys that Flutter wants to derive layout for, and guides on how to derive
 * them.
 */
extern const std::vector<LayoutGoal> kLayoutGoals;

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_KEYCODEMAP_INTERNAL_H_
