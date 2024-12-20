// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'hardware_keyboard.dart';
library;

import 'package:flutter/foundation.dart';

import 'keyboard_maps.g.dart';
import 'raw_keyboard.dart';

export 'package:flutter/foundation.dart' show DiagnosticPropertiesBuilder;

export 'keyboard_key.g.dart' show LogicalKeyboardKey, PhysicalKeyboardKey;

// Android sets the 0x80000000 bit on a character to indicate that it is a
// combining character, so we use this mask to remove that bit to make it a
// valid Unicode character again.
const int _kCombiningCharacterMask = 0x7fffffff;

/// Platform-specific key event data for Android.
///
/// This class is deprecated and will be removed. Platform specific key event
/// data will no longer be available. See [KeyEvent] for what is available.
///
/// This object contains information about key events obtained from Android's
/// `KeyEvent` interface.
///
/// See also:
///
/// * [RawKeyboard], which uses this interface to expose key data.
@Deprecated(
  'Platform specific key event data is no longer available. See KeyEvent for what is available. '
  'This feature was deprecated after v3.18.0-2.0.pre.',
)
class RawKeyEventDataAndroid extends RawKeyEventData {
  /// Creates a key event data structure specific for Android.
  @Deprecated(
    'Platform specific key event data is no longer available. See KeyEvent for what is available. '
    'This feature was deprecated after v3.18.0-2.0.pre.',
  )
  const RawKeyEventDataAndroid({
    this.flags = 0,
    this.codePoint = 0,
    this.plainCodePoint = 0,
    this.keyCode = 0,
    this.scanCode = 0,
    this.metaState = 0,
    this.eventSource = 0,
    this.vendorId = 0,
    this.productId = 0,
    this.deviceId = 0,
    this.repeatCount = 0,
  });

  /// The current set of additional flags for this event.
  ///
  /// Flags indicate things like repeat state, etc.
  ///
  /// See <https://developer.android.com/reference/android/view/KeyEvent.html#getFlags()>
  /// for more information.
  final int flags;

  /// The Unicode code point represented by the key event, if any.
  ///
  /// If there is no Unicode code point, this value is zero.
  ///
  /// Dead keys are represented as Unicode combining characters.
  ///
  /// See <https://developer.android.com/reference/android/view/KeyEvent.html#getUnicodeChar()>
  /// for more information.
  final int codePoint;

  /// The Unicode code point represented by the key event, if any, without
  /// regard to any modifier keys which are currently pressed.
  ///
  /// If there is no Unicode code point, this value is zero.
  ///
  /// Dead keys are represented as Unicode combining characters.
  ///
  /// This is the result of calling KeyEvent.getUnicodeChar(0) on Android.
  ///
  /// See <https://developer.android.com/reference/android/view/KeyEvent.html#getUnicodeChar(int)>
  /// for more information.
  final int plainCodePoint;

  /// The hardware key code corresponding to this key event.
  ///
  /// This is the physical key that was pressed, not the Unicode character.
  /// See [codePoint] for the Unicode character.
  ///
  /// See <https://developer.android.com/reference/android/view/KeyEvent.html#getKeyCode()>
  /// for more information.
  final int keyCode;

  /// The hardware scan code id corresponding to this key event.
  ///
  /// These values are not reliable and vary from device to device, so this
  /// information is mainly useful for debugging.
  ///
  /// See <https://developer.android.com/reference/android/view/KeyEvent.html#getScanCode()>
  /// for more information.
  final int scanCode;

  /// The modifiers that were present when the key event occurred.
  ///
  /// See <https://developer.android.com/reference/android/view/KeyEvent.html#getMetaState()>
  /// for the numerical values of the [metaState]. Many of these constants are
  /// also replicated as static constants in this class.
  ///
  /// See also:
  ///
  ///  * [modifiersPressed], which returns a Map of currently pressed modifiers
  ///    and their keyboard side.
  ///  * [isModifierPressed], to see if a specific modifier is pressed.
  ///  * [isControlPressed], to see if a CTRL key is pressed.
  ///  * [isShiftPressed], to see if a SHIFT key is pressed.
  ///  * [isAltPressed], to see if an ALT key is pressed.
  ///  * [isMetaPressed], to see if a META key is pressed.
  final int metaState;

  /// The source of the event.
  ///
  /// See <https://developer.android.com/reference/android/view/KeyEvent.html#getSource()>
  /// for the numerical values of the `source`. Many of these constants are also
  /// replicated as static constants in this class.
  final int eventSource;

  /// The vendor ID of the device that produced the event.
  ///
  /// See <https://developer.android.com/reference/android/view/InputDevice.html#getVendorId()>
  /// for the numerical values of the [vendorId].
  final int vendorId;

  /// The product ID of the device that produced the event.
  ///
  /// See <https://developer.android.com/reference/android/view/InputDevice.html#getProductId()>
  /// for the numerical values of the [productId].
  final int productId;

  /// The ID of the device that produced the event.
  ///
  /// See https://developer.android.com/reference/android/view/InputDevice.html#getId()
  final int deviceId;

  /// The repeat count of the event.
  ///
  /// See <https://developer.android.com/reference/android/view/KeyEvent#getRepeatCount()>
  /// for more information.
  final int repeatCount;

  // The source code that indicates that an event came from a joystick.
  // from https://developer.android.com/reference/android/view/InputDevice.html#SOURCE_JOYSTICK
  static const int _sourceJoystick = 0x01000010;

  // Android only reports a single code point for the key label.
  @override
  String get keyLabel =>
      plainCodePoint == 0 ? '' : String.fromCharCode(plainCodePoint & _kCombiningCharacterMask);

  @override
  PhysicalKeyboardKey get physicalKey {
    if (kAndroidToPhysicalKey.containsKey(scanCode)) {
      return kAndroidToPhysicalKey[scanCode]!;
    }

    // Android sends DPAD_UP, etc. as the keyCode for joystick DPAD events, but
    // it doesn't set the scanCode for those, so we have to detect this, and set
    // our own DPAD physical keys. The logical key will still match "arrowUp",
    // etc.
    if (eventSource & _sourceJoystick == _sourceJoystick) {
      final LogicalKeyboardKey? foundKey = kAndroidToLogicalKey[keyCode];
      if (foundKey == LogicalKeyboardKey.arrowUp) {
        return PhysicalKeyboardKey.arrowUp;
      }
      if (foundKey == LogicalKeyboardKey.arrowDown) {
        return PhysicalKeyboardKey.arrowDown;
      }
      if (foundKey == LogicalKeyboardKey.arrowLeft) {
        return PhysicalKeyboardKey.arrowLeft;
      }
      if (foundKey == LogicalKeyboardKey.arrowRight) {
        return PhysicalKeyboardKey.arrowRight;
      }
    }
    return PhysicalKeyboardKey(LogicalKeyboardKey.androidPlane + scanCode);
  }

  @override
  LogicalKeyboardKey get logicalKey {
    // Look to see if the keyCode is a printable number pad key, so that a
    // difference between regular keys (e.g. "=") and the number pad version
    // (e.g. the "=" on the number pad) can be determined.
    final LogicalKeyboardKey? numPadKey = kAndroidNumPadMap[keyCode];
    if (numPadKey != null) {
      return numPadKey;
    }

    // If it has a non-control-character label, then either return the existing
    // constant, or construct a new Unicode-based key from it. Don't mark it as
    // autogenerated, since the label uniquely identifies an ID from the Unicode
    // plane.
    if (keyLabel.isNotEmpty && !LogicalKeyboardKey.isControlCharacter(keyLabel)) {
      final int combinedCodePoint = plainCodePoint & _kCombiningCharacterMask;
      final int keyId =
          LogicalKeyboardKey.unicodePlane | (combinedCodePoint & LogicalKeyboardKey.valueMask);
      return LogicalKeyboardKey.findKeyByKeyId(keyId) ?? LogicalKeyboardKey(keyId);
    }

    // Look to see if the keyCode is one we know about and have a mapping for.
    final LogicalKeyboardKey? newKey = kAndroidToLogicalKey[keyCode];
    if (newKey != null) {
      return newKey;
    }

    return LogicalKeyboardKey(keyCode | LogicalKeyboardKey.androidPlane);
  }

  bool _isLeftRightModifierPressed(KeyboardSide side, int anyMask, int leftMask, int rightMask) {
    if (metaState & anyMask == 0) {
      return false;
    }
    return switch (side) {
      KeyboardSide.any => true,
      KeyboardSide.all => (metaState & leftMask != 0) && (metaState & rightMask != 0),
      KeyboardSide.left => metaState & leftMask != 0,
      KeyboardSide.right => metaState & rightMask != 0,
    };
  }

  @override
  bool isModifierPressed(ModifierKey key, {KeyboardSide side = KeyboardSide.any}) {
    return switch (key) {
      ModifierKey.controlModifier => _isLeftRightModifierPressed(
        side,
        modifierControl,
        modifierLeftControl,
        modifierRightControl,
      ),
      ModifierKey.shiftModifier => _isLeftRightModifierPressed(
        side,
        modifierShift,
        modifierLeftShift,
        modifierRightShift,
      ),
      ModifierKey.altModifier => _isLeftRightModifierPressed(
        side,
        modifierAlt,
        modifierLeftAlt,
        modifierRightAlt,
      ),
      ModifierKey.metaModifier => _isLeftRightModifierPressed(
        side,
        modifierMeta,
        modifierLeftMeta,
        modifierRightMeta,
      ),
      ModifierKey.capsLockModifier => metaState & modifierCapsLock != 0,
      ModifierKey.numLockModifier => metaState & modifierNumLock != 0,
      ModifierKey.scrollLockModifier => metaState & modifierScrollLock != 0,
      ModifierKey.functionModifier => metaState & modifierFunction != 0,
      ModifierKey.symbolModifier => metaState & modifierSym != 0,
    };
  }

  @override
  KeyboardSide? getModifierSide(ModifierKey key) {
    KeyboardSide? findSide(int anyMask, int leftMask, int rightMask) {
      final int combinedMask = leftMask | rightMask;
      final int combined = metaState & combinedMask;
      if (combined == leftMask) {
        return KeyboardSide.left;
      } else if (combined == rightMask) {
        return KeyboardSide.right;
      } else if (combined == combinedMask) {
        return KeyboardSide.all;
      }
      // If the platform code sets the "any" modifier, but not a specific side,
      // then we return "all", assuming that there is only one of that modifier
      // key on the keyboard.
      if (metaState & anyMask != 0) {
        return KeyboardSide.all;
      }
      return null;
    }

    switch (key) {
      case ModifierKey.controlModifier:
        return findSide(modifierControl, modifierLeftControl, modifierRightControl);
      case ModifierKey.shiftModifier:
        return findSide(modifierShift, modifierLeftShift, modifierRightShift);
      case ModifierKey.altModifier:
        return findSide(modifierAlt, modifierLeftAlt, modifierRightAlt);
      case ModifierKey.metaModifier:
        return findSide(modifierMeta, modifierLeftMeta, modifierRightMeta);
      case ModifierKey.capsLockModifier:
      case ModifierKey.numLockModifier:
      case ModifierKey.scrollLockModifier:
      case ModifierKey.functionModifier:
      case ModifierKey.symbolModifier:
        return KeyboardSide.all;
    }
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<int>('flags', flags));
    properties.add(DiagnosticsProperty<int>('codePoint', codePoint));
    properties.add(DiagnosticsProperty<int>('plainCodePoint', plainCodePoint));
    properties.add(DiagnosticsProperty<int>('keyCode', keyCode));
    properties.add(DiagnosticsProperty<int>('scanCode', scanCode));
    properties.add(DiagnosticsProperty<int>('metaState', metaState));
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is RawKeyEventDataAndroid &&
        other.flags == flags &&
        other.codePoint == codePoint &&
        other.plainCodePoint == plainCodePoint &&
        other.keyCode == keyCode &&
        other.scanCode == scanCode &&
        other.metaState == metaState;
  }

  @override
  int get hashCode => Object.hash(flags, codePoint, plainCodePoint, keyCode, scanCode, metaState);

  // Modifier key masks.

  /// No modifier keys are pressed in the [metaState] field.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierNone = 0;

  /// This mask is used to check the [metaState] field to test whether one of
  /// the ALT modifier keys is pressed.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierAlt = 0x02;

  /// This mask is used to check the [metaState] field to test whether the left
  /// ALT modifier key is pressed.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierLeftAlt = 0x10;

  /// This mask is used to check the [metaState] field to test whether the right
  /// ALT modifier key is pressed.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierRightAlt = 0x20;

  /// This mask is used to check the [metaState] field to test whether one of
  /// the SHIFT modifier keys is pressed.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierShift = 0x01;

  /// This mask is used to check the [metaState] field to test whether the left
  /// SHIFT modifier key is pressed.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierLeftShift = 0x40;

  /// This mask is used to check the [metaState] field to test whether the right
  /// SHIFT modifier key is pressed.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierRightShift = 0x80;

  /// This mask is used to check the [metaState] field to test whether the SYM
  /// modifier key is pressed.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierSym = 0x04;

  /// This mask is used to check the [metaState] field to test whether the
  /// Function modifier key (Fn) is pressed.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierFunction = 0x08;

  /// This mask is used to check the [metaState] field to test whether one of
  /// the CTRL modifier keys is pressed.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierControl = 0x1000;

  /// This mask is used to check the [metaState] field to test whether the left
  /// CTRL modifier key is pressed.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierLeftControl = 0x2000;

  /// This mask is used to check the [metaState] field to test whether the right
  /// CTRL modifier key is pressed.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierRightControl = 0x4000;

  /// This mask is used to check the [metaState] field to test whether one of
  /// the META modifier keys is pressed.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierMeta = 0x10000;

  /// This mask is used to check the [metaState] field to test whether the left
  /// META modifier key is pressed.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierLeftMeta = 0x20000;

  /// This mask is used to check the [metaState] field to test whether the right
  /// META modifier key is pressed.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierRightMeta = 0x40000;

  /// This mask is used to check the [metaState] field to test whether the CAPS
  /// LOCK modifier key is on.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierCapsLock = 0x100000;

  /// This mask is used to check the [metaState] field to test whether the NUM
  /// LOCK modifier key is on.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierNumLock = 0x200000;

  /// This mask is used to check the [metaState] field to test whether the
  /// SCROLL LOCK modifier key is on.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierScrollLock = 0x400000;
}
