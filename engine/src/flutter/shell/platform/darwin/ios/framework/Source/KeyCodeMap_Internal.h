// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_KEYCODEMAP_INTERNAL_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_KEYCODEMAP_INTERNAL_H_

#import <UIKit/UIKit.h>
#include <map>
#include <set>

/**
 * Maps iOS-specific key code values representing |PhysicalKeyboardKey|.
 *
 * MacOS doesn't provide a scan code, but a virtual keycode to represent a
 * physical key.
 */
// NOLINTNEXTLINE(readability-identifier-naming)
extern const std::map<uint32_t, uint64_t> keyCodeToPhysicalKey;

/**
 * A map from iOS key codes to Flutter's logical key values.
 *
 * This is used to derive logical keys that can't or shouldn't be derived from
 * |charactersIgnoringModifiers|.
 */
// NOLINTNEXTLINE(readability-identifier-naming)
extern const std::map<uint32_t, uint64_t> keyCodeToLogicalKey;

/**
 * Maps iOS specific string values of nonvisible keys to logical keys.
 *
 * TODO(dkwingsmt): Change this getter function to a global variable. I tried to
 * do this but the unit test on CI threw errors saying "message sent to
 * deallocated instance" on the NSDictionary.
 *
 * See:
 * https://developer.apple.com/documentation/uikit/uikeycommand/input_strings_for_special_keys?language=objc
 */
extern NSDictionary<NSString*, NSNumber*>* specialKeyMapping;

// Several mask constants. See KeyCodeMap.g.mm for their descriptions.

extern const uint64_t kValueMask;
extern const uint64_t kUnicodePlane;
extern const uint64_t kIosPlane;

/**
 * The physical key for CapsLock, which needs special handling.
 */
extern const uint64_t kCapsLockPhysicalKey;

/**
 * The logical key for CapsLock, which needs special handling.
 */
extern const uint64_t kCapsLockLogicalKey;

/**
 * Bits in |UIKey.modifierFlags| indicating whether a modifier key is pressed.
 */
typedef enum {
  // These sided flags are not in any official Apple docs, they are derived from
  // experiments.
  kModifierFlagControlLeft = 0x1,
  kModifierFlagShiftLeft = 0x2,
  kModifierFlagShiftRight = 0x4,
  kModifierFlagMetaLeft = 0x8,
  kModifierFlagMetaRight = 0x10,
  kModifierFlagAltLeft = 0x20,
  kModifierFlagAltRight = 0x40,
  kModifierFlagControlRight = 0x2000,

  // These are equivalent to non-sided iOS values.
  kModifierFlagCapsLock = UIKeyModifierAlphaShift,  // 0x010000
  kModifierFlagShiftAny = UIKeyModifierShift,       // 0x020000
  kModifierFlagControlAny = UIKeyModifierControl,   // 0x040000
  kModifierFlagAltAny = UIKeyModifierAlternate,     // 0x080000
  kModifierFlagMetaAny = UIKeyModifierCommand,      // 0x100000
  kModifierFlagNumPadKey = UIKeyModifierNumericPad  // 0x200000
} ModifierFlag;

/**
 * A mask of all the modifier flags that represent a modifier being pressed, but
 * not whether it is the left or right modifier.
 */
constexpr uint32_t kModifierFlagAnyMask =
    kModifierFlagShiftAny | kModifierFlagControlAny | kModifierFlagAltAny | kModifierFlagMetaAny;

/**
 * A mask of the modifier flags that represent only left or right modifier
 * keys, and not the generic "Any" mask.
 */
constexpr uint32_t kModifierFlagSidedMask = kModifierFlagControlLeft | kModifierFlagShiftLeft |
                                            kModifierFlagShiftRight | kModifierFlagMetaLeft |
                                            kModifierFlagMetaRight | kModifierFlagAltLeft |
                                            kModifierFlagAltRight | kModifierFlagControlRight;

/**
 * Map |UIKey.keyCode| to the matching sided modifier in UIEventModifierFlags.
 */
// NOLINTNEXTLINE(readability-identifier-naming)
extern const std::map<uint32_t, ModifierFlag> keyCodeToModifierFlag;

/**
 * Map a bit of bitmask of sided modifiers in UIEventModifierFlags to their
 * corresponding |UIKey.keyCode|.
 */
// NOLINTNEXTLINE(readability-identifier-naming)
extern const std::map<ModifierFlag, uint32_t> modifierFlagToKeyCode;

/**
 * Maps a sided modifier key to the corresponding flag matching either side of
 * that type of modifier.
 */
// NOLINTNEXTLINE(readability-identifier-naming)
extern const std::map<ModifierFlag, ModifierFlag> sidedModifierToAny;

/**
 * Maps a non-sided modifier key to the corresponding flag matching the left key
 * of that type of modifier.
 */
// NOLINTNEXTLINE(readability-identifier-naming)
extern const std::map<ModifierFlag, ModifierFlag> anyModifierToLeft;

/**
 * A set of keycodes corresponding to function keys.
 */
// NOLINTNEXTLINE(readability-identifier-naming)
extern const std::set<uint32_t> functionKeyCodes;

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_KEYCODEMAP_INTERNAL_H_
