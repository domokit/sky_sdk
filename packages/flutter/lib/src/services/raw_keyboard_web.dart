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

String? _unicodeChar(String key) {
  if (key.length == 1) {
    return key.substring(0, 1);
  }
  return null;
}

/// Platform-specific key event data for Web.
///
/// This class is DEPRECATED. Platform specific key event data will no longer
/// available. See [KeyEvent] for what is available.
///
/// See also:
///
///  * [RawKeyboard], which uses this interface to expose key data.
@Deprecated(
  'Platform specific key event data is no longer available. See KeyEvent for what is available. '
  'This feature was deprecated after v3.18.0-2.0.pre.',
)
@immutable
class RawKeyEventDataWeb extends RawKeyEventData {
  /// Creates a key event data structure specific for Web.
  @Deprecated(
    'Platform specific key event data is no longer available. See KeyEvent for what is available. '
    'This feature was deprecated after v3.18.0-2.0.pre.',
  )
  const RawKeyEventDataWeb({
    required this.code,
    required this.key,
    this.location = 0,
    this.metaState = modifierNone,
    this.keyCode = 0,
  });

  /// The `KeyboardEvent.code` corresponding to this event.
  ///
  /// The [code] represents a physical key on the keyboard, a value that isn't
  /// altered by keyboard layout or the state of the modifier keys.
  ///
  /// See <https://developer.mozilla.org/en-US/docs/Web/API/KeyboardEvent/code>
  /// for more information.
  final String code;

  /// The `KeyboardEvent.key` corresponding to this event.
  ///
  /// The [key] represents the key pressed by the user, taking into
  /// consideration the state of modifier keys such as Shift as well as the
  /// keyboard locale and layout.
  ///
  /// See <https://developer.mozilla.org/en-US/docs/Web/API/KeyboardEvent/key>
  /// for more information.
  final String key;

  /// The `KeyboardEvent.location` corresponding to this event.
  ///
  /// The [location] represents the location of the key on the keyboard or other
  /// input device, such as left or right modifier keys, or Numpad keys.
  ///
  /// See <https://developer.mozilla.org/en-US/docs/Web/API/KeyboardEvent/location>
  /// for more information.
  final int location;

  /// The modifiers that were present when the key event occurred.
  ///
  /// See `lib/src/engine/keyboard.dart` in the web engine for the numerical
  /// values of the [metaState]. These constants are also replicated as static
  /// constants in this class.
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

  /// The `KeyboardEvent.keyCode` corresponding to this event.
  ///
  /// See <https://developer.mozilla.org/en-US/docs/Web/API/KeyboardEvent/keyCode>
  /// for more information.
  final int keyCode;

  @override
  String get keyLabel => key == 'Unidentified' ? '' : _unicodeChar(key) ?? '';

  @override
  PhysicalKeyboardKey get physicalKey {
    return kWebToPhysicalKey[code] ??
        PhysicalKeyboardKey(LogicalKeyboardKey.webPlane + code.hashCode);
  }

  @override
  LogicalKeyboardKey get logicalKey {
    // Look to see if the keyCode is a key based on location. Typically they are
    // numpad keys (versus main area keys) and left/right modifiers.
    final LogicalKeyboardKey? maybeLocationKey = kWebLocationMap[key]?[location];
    if (maybeLocationKey != null) {
      return maybeLocationKey;
    }

    // Look to see if the [key] is one we know about and have a mapping for.
    final LogicalKeyboardKey? newKey = kWebToLogicalKey[key];
    if (newKey != null) {
      return newKey;
    }

    final bool isPrintable = key.length == 1;
    if (isPrintable) {
      return LogicalKeyboardKey(key.toLowerCase().codeUnitAt(0));
    }

    // This is a non-printable key that we don't know about, so we mint a new
    // key from `code`. Don't mint with `key`, because the `key` will always be
    // "Unidentified" .
    return LogicalKeyboardKey(code.hashCode + LogicalKeyboardKey.webPlane);
  }

  @override
  bool isModifierPressed(ModifierKey key, {KeyboardSide side = KeyboardSide.any}) {
    return switch (key) {
      ModifierKey.controlModifier => metaState & modifierControl != 0,
      ModifierKey.shiftModifier => metaState & modifierShift != 0,
      ModifierKey.altModifier => metaState & modifierAlt != 0,
      ModifierKey.metaModifier => metaState & modifierMeta != 0,
      ModifierKey.numLockModifier => metaState & modifierNumLock != 0,
      ModifierKey.capsLockModifier => metaState & modifierCapsLock != 0,
      ModifierKey.scrollLockModifier => metaState & modifierScrollLock != 0,
      // On Web, the browser doesn't report the state of the FN and SYM modifiers.
      ModifierKey.functionModifier || ModifierKey.symbolModifier => false,
    };
  }

  @override
  KeyboardSide getModifierSide(ModifierKey key) {
    // On Web, we don't distinguish the sides of modifier keys. Both left shift
    // and right shift, for example, are reported as the "Shift" modifier.
    //
    // See <https://developer.mozilla.org/en-US/docs/Web/API/KeyboardEvent/getModifierState>
    // for more information.
    return KeyboardSide.any;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<String>('code', code));
    properties.add(DiagnosticsProperty<String>('key', key));
    properties.add(DiagnosticsProperty<int>('location', location));
    properties.add(DiagnosticsProperty<int>('metaState', metaState));
    properties.add(DiagnosticsProperty<int>('keyCode', keyCode));
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is RawKeyEventDataWeb &&
        other.code == code &&
        other.key == key &&
        other.location == location &&
        other.metaState == metaState &&
        other.keyCode == keyCode;
  }

  @override
  int get hashCode => Object.hash(code, key, location, metaState, keyCode);

  // Modifier key masks.

  /// No modifier keys are pressed in the [metaState] field.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierNone = 0;

  /// This mask is used to check the [metaState] field to test whether one of
  /// the SHIFT modifier keys is pressed.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierShift = 0x01;

  /// This mask is used to check the [metaState] field to test whether one of
  /// the ALT modifier keys is pressed.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierAlt = 0x02;

  /// This mask is used to check the [metaState] field to test whether one of
  /// the CTRL modifier keys is pressed.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierControl = 0x04;

  /// This mask is used to check the [metaState] field to test whether one of
  /// the META modifier keys is pressed.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierMeta = 0x08;

  /// This mask is used to check the [metaState] field to test whether the NUM
  /// LOCK modifier key is on.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierNumLock = 0x10;

  /// This mask is used to check the [metaState] field to test whether the CAPS
  /// LOCK modifier key is on.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierCapsLock = 0x20;

  /// This mask is used to check the [metaState] field to test whether the
  /// SCROLL LOCK modifier key is on.
  ///
  /// Use this value if you need to decode the [metaState] field yourself, but
  /// it's much easier to use [isModifierPressed] if you just want to know if
  /// a modifier is pressed.
  static const int modifierScrollLock = 0x40;
}
