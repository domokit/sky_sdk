// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/cupertino.dart';
/// @docImport 'package:flutter/material.dart';
/// @docImport 'package:flutter_test/flutter_test.dart';
library;

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';

import 'binding.dart';
import 'debug.dart';
import 'raw_keyboard.dart';
import 'raw_keyboard_android.dart';
import 'system_channels.dart';

export 'dart:ui' show KeyData;

export 'package:flutter/foundation.dart' show DiagnosticPropertiesBuilder;

export 'keyboard_key.g.dart' show LogicalKeyboardKey, PhysicalKeyboardKey;

// When using _keyboardDebug, always call it like so:
//
// assert(_keyboardDebug(() => 'Blah $foo'));
//
// It needs to be inside the assert in order to be removed in release mode, and
// it needs to use a closure to generate the string in order to avoid string
// interpolation when debugPrintKeyboardEvents is false.
//
// It will throw a StateError if you try to call it when the app is in release
// mode.
bool _keyboardDebug(String Function() messageFunc, [Iterable<Object> Function()? detailsFunc]) {
  if (kReleaseMode) {
    throw StateError(
      '_keyboardDebug was called in Release mode, which means they are called '
      'without being wrapped in an assert. Always call _keyboardDebug like so:\n'
      r"  assert(_keyboardDebug(() => 'Blah $foo'));",
    );
  }
  if (!debugPrintKeyboardEvents) {
    return true;
  }
  debugPrint('KEYBOARD: ${messageFunc()}');
  final Iterable<Object> details = detailsFunc?.call() ?? const <Object>[];
  if (details.isNotEmpty) {
    for (final Object detail in details) {
      debugPrint('    $detail');
    }
  }
  // Return true so that it can be used inside of an assert.
  return true;
}

/// Represents a lock mode of a keyboard, such as [KeyboardLockMode.capsLock].
///
/// A lock mode locks some of a keyboard's keys into a distinct mode of operation,
/// depending on the lock settings selected. The status of the mode is toggled
/// with each key down of its corresponding logical key. A [KeyboardLockMode]
/// object is used to query whether this mode is enabled on the keyboard.
///
/// Only a limited number of modes are supported, which are enumerated as
/// static members of this class. Manual constructing of this class is
/// prohibited.
enum KeyboardLockMode {
  /// Represents the number lock mode on the keyboard.
  ///
  /// On supporting systems, enabling number lock mode usually allows key
  /// presses of the number pad to input numbers, instead of acting as up, down,
  /// left, right, page up, end, etc.
  numLock._(LogicalKeyboardKey.numLock),

  /// Represents the scrolling lock mode on the keyboard.
  ///
  /// On supporting systems and applications (such as a spreadsheet), enabling
  /// scrolling lock mode usually allows key presses of the cursor keys to
  /// scroll the document instead of the cursor.
  scrollLock._(LogicalKeyboardKey.scrollLock),

  /// Represents the capital letters lock mode on the keyboard.
  ///
  /// On supporting systems, enabling capital lock mode allows key presses of
  /// the letter keys to input uppercase letters instead of lowercase.
  capsLock._(LogicalKeyboardKey.capsLock);

  // KeyboardLockMode has a fixed pool of supported keys, enumerated as static
  // members of this class, therefore constructing is prohibited.
  const KeyboardLockMode._(this.logicalKey);

  /// The logical key that triggers this lock mode.
  final LogicalKeyboardKey logicalKey;

  static final Map<int, KeyboardLockMode> _knownLockModes = <int, KeyboardLockMode>{
    numLock.logicalKey.keyId: numLock,
    scrollLock.logicalKey.keyId: scrollLock,
    capsLock.logicalKey.keyId: capsLock,
  };

  /// Returns the [KeyboardLockMode] constant from the logical key, or
  /// null, if not found.
  static KeyboardLockMode? findLockByLogicalKey(LogicalKeyboardKey logicalKey) =>
      _knownLockModes[logicalKey.keyId];
}

/// Defines the interface for keyboard key events.
///
/// The [KeyEvent] provides a universal model for key event information from a
/// hardware keyboard across platforms.
///
/// See also:
///
///  * [HardwareKeyboard] for full introduction to key event model and handling.
///  * [KeyDownEvent], a subclass for events representing the user pressing a
///    key.
///  * [KeyRepeatEvent], a subclass for events representing the user holding a
///    key, causing repeated events.
///  * [KeyUpEvent], a subclass for events representing the user releasing a
///    key.
@immutable
abstract class KeyEvent with Diagnosticable {
  /// Create a const KeyEvent by providing each field.
  const KeyEvent({
    required this.physicalKey,
    required this.logicalKey,
    this.character,
    required this.timeStamp,
    this.deviceType = ui.KeyEventDeviceType.keyboard,
    this.synthesized = false,
  });

  /// Returns an object representing the physical location of this key.
  ///
  /// A [PhysicalKeyboardKey] represents a USB HID code sent from the keyboard,
  /// ignoring the key map, modifier keys (like SHIFT), and the label on the key.
  ///
  /// [PhysicalKeyboardKey]s are used to describe and test for keys in a
  /// particular location. A [PhysicalKeyboardKey] may have a name, but the name
  /// is a mnemonic ("keyA" is easier to remember than 0x70004), derived from the
  /// key's effect on a QWERTY keyboard. The name does not represent the key's
  /// effect whatsoever (a physical "keyA" can be the Q key on an AZERTY
  /// keyboard).
  ///
  /// For instance, if you wanted to make a game where the key to the right of
  /// the CAPS LOCK key made the player move left, you would be comparing a
  /// physical key with [PhysicalKeyboardKey.keyA], since that is the key next to
  /// the CAPS LOCK key on a QWERTY keyboard. This would return the same thing
  /// even on an AZERTY keyboard where the key next to the CAPS LOCK produces a
  /// "Q" when pressed.
  ///
  /// If you want to make your app respond to a key with a particular character
  /// on it regardless of location of the key, use [KeyEvent.logicalKey] instead.
  ///
  /// Also, even though physical keys are defined with USB HID codes, their
  /// values are not necessarily the same HID codes produced by the hardware and
  /// presented to the driver. On most platforms, Flutter has to map the
  /// platform representation back to a HID code because the original HID
  /// code is not provided. USB HID was chosen because it is a well-defined
  /// standard for referring to keys such as those a Flutter app may encounter.
  ///
  /// See also:
  ///
  ///  * [logicalKey] for the non-location specific key generated by this event.
  ///  * [character] for the character generated by this keypress (if any).
  final PhysicalKeyboardKey physicalKey;

  /// Returns an object representing the logical key that was pressed.
  ///
  /// {@template flutter.services.KeyEvent.logicalKey}
  /// This method takes into account the key map and modifier keys (like SHIFT)
  /// to determine which logical key to return.
  ///
  /// If you are looking for the character produced by a key event, use
  /// [KeyEvent.character] instead.
  ///
  /// If you are collecting text strings, use the [TextField] or
  /// [CupertinoTextField] widgets, since those automatically handle many of the
  /// complexities of managing keyboard input, like showing a soft keyboard or
  /// interacting with an input method editor (IME).
  /// {@endtemplate}
  final LogicalKeyboardKey logicalKey;

  /// Returns the Unicode character (grapheme cluster) completed by this
  /// keystroke, if any.
  ///
  /// This will only return a character if this keystroke, combined with any
  /// preceding keystroke(s), generates a character, and only on a "key down"
  /// event. It will return null if no character has been generated by the
  /// keystroke (e.g. a "dead" or "combining" key), or if the corresponding key
  /// is a key without a visual representation, such as a modifier key or a
  /// control key. It will also return null if this is a "key up" event.
  ///
  /// This can return multiple Unicode code points, since some characters (more
  /// accurately referred to as grapheme clusters) are made up of more than one
  /// code point.
  ///
  /// The [character] doesn't take into account edits by an input method editor
  /// (IME), or manage the visibility of the soft keyboard on touch devices. For
  /// composing text, use the [TextField] or [CupertinoTextField] widgets, since
  /// those automatically handle many of the complexities of managing keyboard
  /// input.
  ///
  /// The [character] is not available on [KeyUpEvent]s.
  final String? character;

  /// Time of event, relative to an arbitrary start point.
  ///
  /// All events share the same timeStamp origin.
  final Duration timeStamp;

  /// The source device type for the key event.
  ///
  /// Not all platforms supply an accurate type.
  ///
  /// Defaults to [ui.KeyEventDeviceType.keyboard].
  final ui.KeyEventDeviceType deviceType;

  /// Whether this event is synthesized by Flutter to synchronize key states.
  ///
  /// An non-[synthesized] event is converted from a native event, and a native
  /// event can only be converted to one non-[synthesized] event. Some properties
  /// might be changed during the conversion (for example, a native repeat event
  /// might be converted to a Flutter down event when necessary.)
  ///
  /// A [synthesized] event is created without a source native event in order to
  /// synchronize key states. For example, if the native platform shows that a
  /// shift key that was previously held has been released somehow without the
  /// key up event dispatched (probably due to loss of focus), a synthesized key
  /// up event will be added to regularized the event stream.
  ///
  /// For detailed introduction to the regularized event model, see
  /// [HardwareKeyboard].
  ///
  /// Defaults to false.
  final bool synthesized;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<PhysicalKeyboardKey>('physicalKey', physicalKey));
    properties.add(DiagnosticsProperty<LogicalKeyboardKey>('logicalKey', logicalKey));
    properties.add(StringProperty('character', character));
    properties.add(DiagnosticsProperty<Duration>('timeStamp', timeStamp));
    properties.add(FlagProperty('synthesized', value: synthesized, ifTrue: 'synthesized'));
  }
}

/// An event indicating that the user has pressed a key down on the keyboard.
///
/// See also:
///
///  * [KeyRepeatEvent], a key event representing the user
///    holding a key, causing repeated events.
///  * [KeyUpEvent], a key event representing the user
///    releasing a key.
///  * [HardwareKeyboard], which produces this event.
class KeyDownEvent extends KeyEvent {
  /// Creates a key event that represents the user pressing a key.
  const KeyDownEvent({
    required super.physicalKey,
    required super.logicalKey,
    super.character,
    required super.timeStamp,
    super.synthesized,
    super.deviceType,
  });
}

/// An event indicating that the user has released a key on the keyboard.
///
/// See also:
///
///  * [KeyDownEvent], a key event representing the user
///    pressing a key.
///  * [KeyRepeatEvent], a key event representing the user
///    holding a key, causing repeated events.
///  * [HardwareKeyboard], which produces this event.
class KeyUpEvent extends KeyEvent {
  /// Creates a key event that represents the user pressing a key.
  const KeyUpEvent({
    required super.physicalKey,
    required super.logicalKey,
    required super.timeStamp,
    super.synthesized,
    super.deviceType,
  });
}

/// An event indicating that the user has been holding a key on the keyboard
/// and causing repeated events.
///
/// Repeat events are not guaranteed and are provided only if supported by the
/// underlying platform.
///
/// See also:
///
///  * [KeyDownEvent], a key event representing the user
///    pressing a key.
///  * [KeyUpEvent], a key event representing the user
///    releasing a key.
///  * [HardwareKeyboard], which produces this event.
class KeyRepeatEvent extends KeyEvent {
  /// Creates a key event that represents the user pressing a key.
  const KeyRepeatEvent({
    required super.physicalKey,
    required super.logicalKey,
    super.character,
    required super.timeStamp,
    super.deviceType,
  });
}

/// The signature for [HardwareKeyboard.addHandler], a callback to decide whether
/// the entire framework handles a key event.
typedef KeyEventCallback = bool Function(KeyEvent event);

/// Manages key events from hardware keyboards.
///
/// [HardwareKeyboard] manages all key events of the Flutter application from
/// hardware keyboards (in contrast to on-screen keyboards). It receives key
/// data from the native platform, dispatches key events to registered
/// handlers, and records the keyboard state.
///
/// To stay notified whenever keys are pressed, held, or released, add a
/// handler with [addHandler]. To only be notified when a specific part of the
/// app is focused, use a [Focus] widget's `onFocusChanged` attribute instead
/// of [addHandler]. Handlers should be removed with [removeHandler] when
/// notification is no longer necessary, or when the handler is being disposed.
///
/// To query whether a key is being held, or a lock mode is enabled, use
/// [physicalKeysPressed], [logicalKeysPressed], or [lockModesEnabled].
/// These states will have been updated with the event when used during a key
/// event handler.
///
/// The singleton [HardwareKeyboard] instance is held by the [ServicesBinding]
/// as [ServicesBinding.keyboard], and can be conveniently accessed using the
/// [HardwareKeyboard.instance] static accessor.
///
/// ## Event model
///
/// Flutter uses a universal event model ([KeyEvent]) and key options
/// ([LogicalKeyboardKey] and [PhysicalKeyboardKey]) regardless of the native
/// platform, while preserving platform-specific features as much as
/// possible.
///
/// [HardwareKeyboard] guarantees that the key model is "regularized": The key
/// event stream consists of "key tap sequences", where a key tap sequence is
/// defined as one [KeyDownEvent], zero or more [KeyRepeatEvent]s, and one
/// [KeyUpEvent] in order, all with the same physical key and logical key.
///
/// Example:
///
///  * Tap and hold key A, US layout:
///     * KeyDownEvent(physicalKey: keyA, logicalKey: keyA, character: "a")
///     * KeyRepeatEvent(physicalKey: keyA, logicalKey: keyA, character: "a")
///     * KeyUpEvent(physicalKey: keyA, logicalKey: keyA)
///  * Press ShiftLeft, tap key A, then release ShiftLeft, US layout:
///     * KeyDownEvent(physicalKey: shiftLeft, logicalKey: shiftLeft)
///     * KeyDownEvent(physicalKey: keyA, logicalKey: keyA, character: "A")
///     * KeyRepeatEvent(physicalKey: keyA, logicalKey: keyA, character: "A")
///     * KeyUpEvent(physicalKey: keyA, logicalKey: keyA)
///     * KeyUpEvent(physicalKey: shiftLeft, logicalKey: shiftLeft)
///  * Tap key Q, French layout:
///     * KeyDownEvent(physicalKey: keyA, logicalKey: keyQ, character: "q")
///     * KeyUpEvent(physicalKey: keyA, logicalKey: keyQ)
///  * Tap CapsLock:
///     * KeyDownEvent(physicalKey: capsLock, logicalKey: capsLock)
///     * KeyUpEvent(physicalKey: capsLock, logicalKey: capsLock)
///
/// When the Flutter application starts, all keys are released, and all lock
/// modes are disabled. Upon key events, [HardwareKeyboard] will update its
/// states, then dispatch callbacks: [KeyDownEvent]s and [KeyUpEvent]s set
/// or reset the pressing state, while [KeyDownEvent]s also toggle lock modes.
///
/// Flutter will try to synchronize with the ground truth of keyboard states
/// using synthesized events ([KeyEvent.synthesized]), subject to the
/// availability of the platform. The desynchronization can be caused by
/// non-empty initial state or a change in the focused window or application.
/// For example, if CapsLock is enabled when the application starts, then
/// immediately before the first key event, a synthesized [KeyDownEvent] and
/// [KeyUpEvent] of CapsLock will be dispatched.
///
/// The resulting event stream does not map one-to-one to the native key event
/// stream. Some native events might be skipped, while some events might be
/// synthesized and do not correspond to native events. Synthesized events will
/// be indicated by [KeyEvent.synthesized].
///
/// Example:
///
///  * Flutter starts with CapsLock on, the first press of keyA:
///     * KeyDownEvent(physicalKey: capsLock, logicalKey: capsLock, synthesized: true)
///     * KeyUpEvent(physicalKey: capsLock, logicalKey: capsLock, synthesized: true)
///     * KeyDownEvent(physicalKey: keyA, logicalKey: keyA, character: "a")
///  * While holding ShiftLeft, lose window focus, release shiftLeft, then focus
///    back and press keyA:
///     * KeyUpEvent(physicalKey: shiftLeft, logicalKey: shiftLeft, synthesized: true)
///     * KeyDownEvent(physicalKey: keyA, logicalKey: keyA, character: "a")
///
/// Flutter does not distinguish between multiple keyboards. Flutter will
/// process all events as if they come from a single keyboard, and try to
/// resolve any conflicts and provide a regularized key event stream, which
/// can deviate from the ground truth.
///
/// See also:
///
///  * [KeyDownEvent], [KeyRepeatEvent], and [KeyUpEvent], the classes used to
///    describe specific key events.
///  * [instance], the singleton instance of this class.
class HardwareKeyboard {
  /// Provides convenient access to the current [HardwareKeyboard] singleton from
  /// the [ServicesBinding] instance.
  static HardwareKeyboard get instance => ServicesBinding.instance.keyboard;

  final Map<PhysicalKeyboardKey, LogicalKeyboardKey> _pressedKeys =
      <PhysicalKeyboardKey, LogicalKeyboardKey>{};

  /// The set of [PhysicalKeyboardKey]s that are pressed.
  ///
  /// If called from a key event handler, the result will already include the effect
  /// of the event.
  ///
  /// See also:
  ///
  ///  * [logicalKeysPressed], which tells if a logical key is being pressed.
  Set<PhysicalKeyboardKey> get physicalKeysPressed => _pressedKeys.keys.toSet();

  /// The set of [LogicalKeyboardKey]s that are pressed.
  ///
  /// If called from a key event handler, the result will already include the effect
  /// of the event.
  ///
  /// See also:
  ///
  ///  * [physicalKeysPressed], which tells if a physical key is being pressed.
  Set<LogicalKeyboardKey> get logicalKeysPressed => _pressedKeys.values.toSet();

  /// Returns the logical key that corresponds to the given pressed physical key.
  ///
  /// Returns null if the physical key is not currently pressed.
  LogicalKeyboardKey? lookUpLayout(PhysicalKeyboardKey physicalKey) => _pressedKeys[physicalKey];

  final Set<KeyboardLockMode> _lockModes = <KeyboardLockMode>{};

  /// The set of [KeyboardLockMode] that are enabled.
  ///
  /// Lock keys, such as CapsLock, are logical keys that toggle their
  /// respective boolean states on key down events. Such flags are usually used
  /// as modifier to other keys or events.
  ///
  /// If called from a key event handler, the result will already include the effect
  /// of the event.
  Set<KeyboardLockMode> get lockModesEnabled => _lockModes;

  /// Returns true if the given [LogicalKeyboardKey] is pressed, according to
  /// the [HardwareKeyboard].
  bool isLogicalKeyPressed(LogicalKeyboardKey key) => _pressedKeys.values.contains(key);

  /// Returns true if the given [PhysicalKeyboardKey] is pressed, according to
  /// the [HardwareKeyboard].
  bool isPhysicalKeyPressed(PhysicalKeyboardKey key) => _pressedKeys.containsKey(key);

  /// Returns true if a logical CTRL modifier key is pressed, regardless of
  /// which side of the keyboard it is on.
  ///
  /// Use [isLogicalKeyPressed] if you need to know which control key was
  /// pressed.
  bool get isControlPressed {
    return isLogicalKeyPressed(LogicalKeyboardKey.controlLeft) ||
        isLogicalKeyPressed(LogicalKeyboardKey.controlRight);
  }

  /// Returns true if a logical SHIFT modifier key is pressed, regardless of
  /// which side of the keyboard it is on.
  ///
  /// Use [isLogicalKeyPressed] if you need to know which shift key was pressed.
  bool get isShiftPressed {
    return isLogicalKeyPressed(LogicalKeyboardKey.shiftLeft) ||
        isLogicalKeyPressed(LogicalKeyboardKey.shiftRight);
  }

  /// Returns true if a logical ALT modifier key is pressed, regardless of which
  /// side of the keyboard it is on.
  ///
  /// The `AltGr` key that appears on some keyboards is considered to be the
  /// same as [LogicalKeyboardKey.altRight] on some platforms (notably Android).
  /// On platforms that can distinguish between `altRight` and `altGr`, a press
  /// of `AltGr` will not return true here, and will need to be tested for
  /// separately.
  ///
  /// Use [isLogicalKeyPressed] if you need to know which alt key was pressed.
  bool get isAltPressed {
    return isLogicalKeyPressed(LogicalKeyboardKey.altLeft) ||
        isLogicalKeyPressed(LogicalKeyboardKey.altRight);
  }

  /// Returns true if a logical META modifier key is pressed, regardless of
  /// which side of the keyboard it is on.
  ///
  /// Use [isLogicalKeyPressed] if you need to know which meta key was pressed.
  bool get isMetaPressed {
    return isLogicalKeyPressed(LogicalKeyboardKey.metaLeft) ||
        isLogicalKeyPressed(LogicalKeyboardKey.metaRight);
  }

  void _assertEventIsRegular(KeyEvent event) {
    assert(() {
      const String common =
          'If this occurs in real application, please report this '
          'bug to Flutter. If this occurs in unit tests, please ensure that '
          "simulated events follow Flutter's event model as documented in "
          '`HardwareKeyboard`. This was the event: ';
      if (event is KeyDownEvent) {
        assert(
          !_pressedKeys.containsKey(event.physicalKey),
          'A ${event.runtimeType} is dispatched, but the state shows that the physical '
          'key is already pressed. $common$event',
        );
      } else if (event is KeyRepeatEvent || event is KeyUpEvent) {
        assert(
          _pressedKeys.containsKey(event.physicalKey),
          'A ${event.runtimeType} is dispatched, but the state shows that the physical '
          'key is not pressed. $common$event',
        );
        assert(
          _pressedKeys[event.physicalKey] == event.logicalKey,
          'A ${event.runtimeType} is dispatched, but the state shows that the physical '
          'key is pressed on a different logical key. $common$event '
          'and the recorded logical key ${_pressedKeys[event.physicalKey]}',
        );
      } else {
        assert(false, 'Unexpected key event class ${event.runtimeType}');
      }
      return true;
    }());
  }

  List<KeyEventCallback> _handlers = <KeyEventCallback>[];
  bool _duringDispatch = false;
  List<KeyEventCallback>? _modifiedHandlers;

  /// Register a listener that is called every time a hardware key event
  /// occurs.
  ///
  /// All registered handlers will be invoked in order regardless of
  /// their return value. The return value indicates whether Flutter
  /// "handles" the event. If any handler returns true, the event
  /// will not be propagated to other native components in the add-to-app
  /// scenario.
  ///
  /// If an object added a handler, it must remove the handler before it is
  /// disposed.
  ///
  /// If used during event dispatching, the addition will not take effect
  /// until after the dispatching.
  ///
  /// See also:
  ///
  ///  * [removeHandler], which removes the handler.
  void addHandler(KeyEventCallback handler) {
    if (_duringDispatch) {
      _modifiedHandlers ??= <KeyEventCallback>[..._handlers];
      _modifiedHandlers!.add(handler);
    } else {
      _handlers.add(handler);
    }
  }

  /// Stop calling the given listener every time a hardware key event
  /// occurs.
  ///
  /// The `handler` argument must be [identical] to the one used in
  /// [addHandler]. If multiple exist, the first one will be removed.
  /// If none is found, then this method is a no-op.
  ///
  /// If used during event dispatching, the removal will not take effect
  /// until after the event has been dispatched.
  void removeHandler(KeyEventCallback handler) {
    if (_duringDispatch) {
      _modifiedHandlers ??= <KeyEventCallback>[..._handlers];
      _modifiedHandlers!.remove(handler);
    } else {
      _handlers.remove(handler);
    }
  }

  /// Query the engine and update _pressedKeys accordingly to the engine answer.
  //
  /// Both the framework and the engine maintain a state of the current pressed
  /// keys. There are edge cases, related to startup and restart, where the framework
  /// needs to resynchronize its keyboard state.
  Future<void> syncKeyboardState() async {
    final Map<int, int>? keyboardState = await SystemChannels.keyboard.invokeMapMethod<int, int>(
      'getKeyboardState',
    );
    if (keyboardState != null) {
      for (final int key in keyboardState.keys) {
        final PhysicalKeyboardKey physicalKey = PhysicalKeyboardKey(key);
        final LogicalKeyboardKey logicalKey = LogicalKeyboardKey(keyboardState[key]!);
        _pressedKeys[physicalKey] = logicalKey;
      }
    }
  }

  bool _dispatchKeyEvent(KeyEvent event) {
    // This dispatching could have used the same algorithm as [ChangeNotifier],
    // but since 1) it shouldn't be necessary to support reentrantly
    // dispatching, 2) there shouldn't be many handlers (most apps should use
    // only 1, this function just uses a simpler algorithm.
    assert(!_duringDispatch, 'Nested keyboard dispatching is not supported');
    _duringDispatch = true;
    bool handled = false;
    for (final KeyEventCallback handler in _handlers) {
      try {
        final bool thisResult = handler(event);
        handled = handled || thisResult;
      } catch (exception, stack) {
        InformationCollector? collector;
        assert(() {
          collector = () => <DiagnosticsNode>[DiagnosticsProperty<KeyEvent>('Event', event)];
          return true;
        }());
        FlutterError.reportError(
          FlutterErrorDetails(
            exception: exception,
            stack: stack,
            library: 'services library',
            context: ErrorDescription('while processing a key handler'),
            informationCollector: collector,
          ),
        );
      }
    }
    _duringDispatch = false;
    if (_modifiedHandlers != null) {
      _handlers = _modifiedHandlers!;
      _modifiedHandlers = null;
    }
    return handled;
  }

  List<String> _debugPressedKeysDetails() {
    return <String>[
      if (_pressedKeys.isEmpty)
        'Empty'
      else
        for (final PhysicalKeyboardKey physicalKey in _pressedKeys.keys)
          '$physicalKey: ${_pressedKeys[physicalKey]}',
    ];
  }

  /// Process a new [KeyEvent] by recording the state changes and dispatching
  /// to handlers.
  bool handleKeyEvent(KeyEvent event) {
    assert(_keyboardDebug(() => 'Key event received: $event'));
    assert(
      _keyboardDebug(() => 'Pressed state before processing the event:', _debugPressedKeysDetails),
    );
    _assertEventIsRegular(event);
    final PhysicalKeyboardKey physicalKey = event.physicalKey;
    final LogicalKeyboardKey logicalKey = event.logicalKey;
    if (event is KeyDownEvent) {
      _pressedKeys[physicalKey] = logicalKey;
      final KeyboardLockMode? lockMode = KeyboardLockMode.findLockByLogicalKey(event.logicalKey);
      if (lockMode != null) {
        if (_lockModes.contains(lockMode)) {
          _lockModes.remove(lockMode);
        } else {
          _lockModes.add(lockMode);
        }
      }
    } else if (event is KeyUpEvent) {
      _pressedKeys.remove(physicalKey);
    } else if (event is KeyRepeatEvent) {
      // Empty
    }

    assert(
      _keyboardDebug(() => 'Pressed state after processing the event:', _debugPressedKeysDetails),
    );
    return _dispatchKeyEvent(event);
  }

  /// Clear all keyboard states and additional handlers.
  ///
  /// All handlers are removed except for the first one, which is added by
  /// [ServicesBinding].
  ///
  /// This is used by the testing framework to make sure that tests are hermetic.
  @visibleForTesting
  void clearState() {
    _pressedKeys.clear();
    _lockModes.clear();
    _handlers.clear();
    assert(_modifiedHandlers == null);
  }
}

/// The mode in which information of key messages is delivered.
///
/// This enum is deprecated and will be removed. There is no direct substitute
/// planned, since this enum will no longer be necessary once [RawKeyEvent] and
/// associated APIs are removed.
///
/// Different platforms use different methods, classes, and models to inform the
/// framework of native key events, which is called "transit mode".
///
/// The framework must determine which transit mode the current platform
/// implements and behave accordingly (such as transforming and synthesizing
/// events if necessary). Unit tests related to keyboard might also want to
/// simulate platform of each transit mode.
///
/// The transit mode of the current platform is inferred by [KeyEventManager] at
/// the start of the application.
///
/// See also:
///
/// * [KeyEventManager], which infers the transit mode of the current platform
///   and guides how key messages are dispatched.
/// * [debugKeyEventSimulatorTransitModeOverride], overrides the transit mode
///   used to simulate key events.
/// * [KeySimulatorTransitModeVariant], an easier way to set
///   [debugKeyEventSimulatorTransitModeOverride] in widget tests.
@Deprecated(
  'No longer supported. Transit mode is always key data only. '
  'This feature was deprecated after v3.18.0-2.0.pre.',
)
enum KeyDataTransitMode {
  /// Key event information is delivered as raw key data.
  ///
  /// Raw key data is platform's native key event information sent in JSON
  /// through a method channel, which is then interpreted as a platform subclass
  /// of [RawKeyEventData].
  ///
  /// If the current transit mode is [rawKeyData], the raw key data is converted
  /// to both [KeyMessage.events] and [KeyMessage.rawEvent].
  @Deprecated(
    'No longer supported. Transit mode is always key data only. '
    'This feature was deprecated after v3.18.0-2.0.pre.',
  )
  rawKeyData,

  /// Key event information is delivered as converted key data, followed by raw
  /// key data.
  ///
  /// Key data ([ui.KeyData]) is a standardized event stream converted from
  /// platform's native key event information, sent through the embedder API.
  /// Its event model is described in [HardwareKeyboard].
  ///
  /// Raw key data is platform's native key event information sent in JSON
  /// through a method channel. It is interpreted by subclasses of
  /// [RawKeyEventData].
  ///
  /// If the current transit mode is [keyDataThenRawKeyData], then the
  /// [KeyEventManager] will use the [ui.KeyData] for [KeyMessage.events], and
  /// the raw data for [KeyMessage.rawEvent].
  @Deprecated(
    'No longer supported. Transit mode is always key data only. '
    'This feature was deprecated after v3.18.0-2.0.pre.',
  )
  keyDataThenRawKeyData,
}

/// The assembled information converted from a native key message.
///
/// This class is deprecated, and will be removed. There is no direct substitute
/// planned, since this class will no longer be necessary once [RawKeyEvent] and
/// associated APIs are removed.
///
/// Native key messages, produced by physically pressing or releasing keyboard
/// keys, are translated into two different event streams in Flutter:
///
/// * The [KeyEvent] stream, represented by [KeyMessage.events] (recommended).
/// * The [RawKeyEvent] stream, represented by [KeyMessage.rawEvent] (legacy, to
///   be deprecated).
///
/// Either the [KeyEvent] stream or the [RawKeyEvent] stream alone provides a
/// complete description of the keyboard messages, but in different event
/// models. Flutter is still transitioning from the legacy model to the new
/// model, therefore it dispatches both streams simultaneously until the
/// transition is completed. [KeyMessage] is used to bundle the stream segments
/// of both models from a native key message together for the convenience of
/// propagation.
///
/// Typically, an application either processes [KeyMessage.events] or
/// [KeyMessage.rawEvent], not both. For example, handling a [KeyMessage], means
/// handling each event in [KeyMessage.events].
///
/// In advanced cases, a widget needs to process both streams at the same time.
/// For example, [FocusNode] has an `onKey` that dispatches [RawKeyEvent]s and
/// an `onKeyEvent` that dispatches [KeyEvent]s. To processes a [KeyMessage], it
/// first calls `onKeyEvent` with each [KeyEvent] of [events], and then `onKey`
/// with [rawEvent]. All callbacks are invoked regardless of their
/// [KeyEventResult]. Their results are combined into the result of the node
/// using [combineKeyEventResults].
///
/// ```dart
/// void handleMessage(FocusNode node, KeyMessage message) {
///   final List<KeyEventResult> results = <KeyEventResult>[];
///   if (node.onKeyEvent != null) {
///     for (final KeyEvent event in message.events) {
///       results.add(node.onKeyEvent!(node, event));
///     }
///   }
///   if (node.onKey != null && message.rawEvent != null) {
///     results.add(node.onKey!(node, message.rawEvent!));
///   }
///   final KeyEventResult result = combineKeyEventResults(results);
///   // Progress based on `result`...
/// }
/// ```
@Deprecated(
  'No longer supported. Once RawKeyEvent is removed, it will no longer be needed. '
  'This feature was deprecated after v3.18.0-2.0.pre.',
)
@immutable
class KeyMessage {
  /// Create a [KeyMessage] by providing all information.
  ///
  /// The [events] might be empty.
  @Deprecated(
    'No longer supported. Once RawKeyEvent is removed, will no longer be needed. '
    'This feature was deprecated after v3.18.0-2.0.pre.',
  )
  const KeyMessage(this.events, this.rawEvent);

  /// The list of [KeyEvent]s converted from the native key message.
  ///
  /// A native key message is converted into multiple [KeyEvent]s in a regular
  /// event model. The [events] might contain zero or any number of
  /// [KeyEvent]s.
  ///
  /// See also:
  ///
  ///  * [HardwareKeyboard], which describes the regular event model.
  ///  * [HardwareKeyboard.addHandler], [KeyboardListener], [Focus.onKeyEvent],
  ///    where [KeyEvent]s are commonly used.
  final List<KeyEvent> events;

  /// The native key message in the form of a raw key event.
  ///
  /// A native key message is sent to the framework in JSON and represented
  /// in a platform-specific form as [RawKeyEventData] and a platform-neutral
  /// form as [RawKeyEvent]. Their stream is not as regular as [KeyEvent]'s,
  /// but keeps as much native information and structure as possible.
  ///
  /// The [rawEvent] field might be empty, for example, when the event
  /// converting system dispatches solitary synthesized events.
  ///
  /// The [rawEvent] field will be deprecated in the future.
  ///
  /// See also:
  ///
  ///  * [RawKeyboard.addListener], [RawKeyboardListener], [Focus.onKey],
  ///    where [RawKeyEvent]s are commonly used.
  final RawKeyEvent? rawEvent;

  @override
  String toString() {
    return 'KeyMessage($events)';
  }
}

/// The signature for [KeyEventManager.keyMessageHandler].
///
/// A [KeyMessageHandler] processes a [KeyMessage] and returns whether the
/// message is considered handled. Handled messages should not be propagated to
/// other native components.
///
/// This message handler signature is deprecated, and will be removed. There is
/// no direct substitute planned, since this handler type will no longer be
/// necessary once [RawKeyEvent] and associated APIs are removed.
@Deprecated(
  'No longer supported. Once KeyMessage is removed, will no longer be needed. '
  'This feature was deprecated after v3.18.0-2.0.pre.',
)
typedef KeyMessageHandler = bool Function(KeyMessage message);

/// A singleton class that processes key messages from the platform and
/// dispatches converted messages accordingly.
///
/// This class is deprecated, and will be removed. There is no direct substitute
/// planned, since this class will no longer be necessary once [RawKeyEvent] and
/// associated APIs are removed.
///
/// [KeyEventManager] receives platform key messages by [handleKeyData] and
/// [handleRawKeyMessage], sends converted events to [HardwareKeyboard] and
/// [RawKeyboard] for recording keeping, and then dispatches the [KeyMessage] to
/// [keyMessageHandler], the global message handler.
///
/// [KeyEventManager] is typically created, owned, and invoked by
/// [ServicesBinding].
///
/// ## On embedder implementation
///
/// Currently, Flutter has two sets of key event API pathways running in
/// parallel.
///
/// * The "hardware key event" pathway receives messages from the
///   "flutter/keydata" message channel (embedder API
///   `FlutterEngineSendKeyEvent`) and dispatches [KeyEvent] to
///   [HardwareKeyboard] and some methods such as [Focus.onKeyEvent].
/// * The deprecated "raw key event" pathway receives messages from the
///   "flutter/keyevent" message channel ([SystemChannels.keyEvent]) and
///   dispatches [RawKeyEvent] to [RawKeyboard] and [Focus.onKey] as well as
///   similar methods. This pathway will be removed at a future date.
///
/// [KeyEventManager] resolves cross-platform compatibility of keyboard
/// implementations, since legacy platforms might have not implemented the new
/// key data API and only send raw key data on each key message.
/// [KeyEventManager] recognizes the platform support by detecting whether a
/// message comes from platform channel "flutter/keyevent" before one from
/// "flutter/keydata", or vice versa, at the beginning of the app.
///
/// * If a "flutter/keydata" message is received first, then this platform is
///   considered a modern platform. The hardware key events are stored, and
///   dispatched only when a raw key message is received.
/// * If a "flutter/keyevent" message is received first, then this platform is
///   considered a legacy platform. The raw key event is transformed into a
///   hardware key event at best effort. No messages from "flutter/keydata" are
///   expected. This behavior has been deprecated, and will be removed at a
///   future date.
///
/// Therefore, to correctly implement a platform that supports
/// `FlutterEngineSendKeyEvent`, the platform must ensure that
/// `FlutterEngineSendKeyEvent` is called before sending a message to
/// "flutter/keyevent" at the beginning of the app, and every physical key event
/// is ended with a "flutter/keyevent" message.
@Deprecated(
  'No longer supported. Once RawKeyEvent is removed, will no longer be needed. '
  'This feature was deprecated after v3.18.0-2.0.pre.',
)
class KeyEventManager {
  /// Create an instance.
  ///
  /// This is typically only called by [ServicesBinding].
  @Deprecated(
    'No longer supported. Once RawKeyEvent is removed, will no longer be needed. '
    'This feature was deprecated after v3.18.0-2.0.pre.',
  )
  KeyEventManager(this._hardwareKeyboard, this._rawKeyboard);

  /// The global entrance which handles all key events sent to Flutter.
  ///
  /// This handler is deprecated and will be removed. Use
  /// [HardwareKeyboard.addHandler]/[HardwareKeyboard.removeHandler] instead.
  ///
  /// Typical applications use [WidgetsBinding], where this field is set by the
  /// focus system (see `FocusManager`) on startup to a function that dispatches
  /// incoming events to the focus system, including `FocusNode.onKey`,
  /// `FocusNode.onKeyEvent`, and `Shortcuts`. In this case, the application
  /// does not need to read, assign, or invoke this value.
  ///
  /// For advanced uses, the application can "patch" this callback. See below
  /// for details.
  ///
  /// ## Handlers and event results
  ///
  /// Roughly speaking, Flutter processes each native key event with the
  /// following phases:
  ///
  /// 1. Platform-side pre-filtering, sometimes used for IME.
  /// 2. The key event system.
  /// 3. The text input system.
  /// 4. Other native components (possibly non-Flutter).
  ///
  /// Each phase will conclude with a boolean called an "event result". If the
  /// result is true, this phase _handles_ the event and prevents the event from
  /// being propagated to the next phase. This mechanism allows shortcuts such
  /// as "Ctrl-C" to not generate a text "C" in the text field, or shortcuts
  /// that are not handled by any components to trigger special alerts (such as
  /// the "bonk" noise on macOS).
  ///
  /// In the second phase, known as "the key event system", the event is
  /// dispatched to several destinations: [RawKeyboard]'s listeners,
  /// [HardwareKeyboard]'s handlers, and [keyMessageHandler]. All destinations
  /// will always receive the event regardless of the handlers' results. If any
  /// handler's result is true, then the overall result of the second phase is
  /// true, and event propagation is stopped.
  ///
  /// See also:
  ///
  /// * [RawKeyboard.addListener], which adds a raw keyboard listener.
  /// * [RawKeyboardListener], which is also implemented by adding a raw
  ///   keyboard listener.
  /// * [HardwareKeyboard.addHandler], which adds a hardware keyboard handler.
  ///
  /// ## Advanced usages: Manual assignment or patching
  ///
  /// If you are not using the focus system to manage focus, set this attribute
  /// to a [KeyMessageHandler] that returns true if the propagation on the
  /// platform should not be continued. If this field is null, key events will
  /// be assumed to not have been handled by Flutter, a result of "false".
  ///
  /// Even if you are using the focus system, you might also want to do more
  /// than the focus system allows. In these cases, you can _patch_
  /// [keyMessageHandler] by setting it to a callback that performs your tasks
  /// and calls the original callback in between (or not at all.)
  ///
  /// Patching [keyMessageHandler] can not be reverted. You should always assume
  /// that another component might have patched it before you and after you.
  /// This means that you might want to write your own global notification
  /// manager, to which callbacks can be added and removed.
  ///
  /// You should not patch [keyMessageHandler] until the `FocusManager` has
  /// assigned its callback. This is assured during any time within the widget
  /// lifecycle (such as `initState`), or after calling
  /// `WidgetManager.instance`.
  ///
  /// {@tool dartpad}
  /// This example shows how to process key events that are not
  /// handled by any focus handler (such as `Shortcuts`) by patching
  /// [keyMessageHandler].
  ///
  /// The app prints out any key events that are not handled by the app body.
  /// Try typing something in the first text field. These key presses are not
  /// handled by `Shortcuts` and will be sent to the fallback handler and
  /// printed out. Now try some text shortcuts, such as Ctrl+A. The KeyA press
  /// is handled as a shortcut, and is not sent to the fallback handler and so
  /// is not printed out.
  ///
  /// The key widget is `FallbackKeyEventRegistrar`, a necessity class to allow
  /// reversible patching. `FallbackFocus` and `FallbackFocusNode` are also
  /// useful to recognize the widget tree's structure. `FallbackDemo` is an
  /// example of using them in an app.
  ///
  /// ** See code in examples/api/lib/widgets/hardware_keyboard/key_event_manager.0.dart **
  /// {@end-tool}
  ///
  /// See also:
  ///
  /// * [HardwareKeyboard.addHandler], which accepts multiple global handlers to
  ///   process [KeyEvent]s
  @Deprecated(
    'No longer supported. Once RawKeyEvent is removed, will no longer be needed. '
    'This feature was deprecated after v3.18.0-2.0.pre.',
  )
  KeyMessageHandler? keyMessageHandler;

  final HardwareKeyboard _hardwareKeyboard;
  final RawKeyboard _rawKeyboard;

  // The [KeyDataTransitMode] of the current platform.
  //
  // The `_transitMode` is guaranteed to be non-null during key event callbacks.
  //
  // The `_transitMode` is null before the first event, after which it is inferred
  // and will not change throughout the lifecycle of the application.
  //
  // The `_transitMode` can be reset to null in tests using [clearState].
  KeyDataTransitMode? _transitMode;

  // The accumulated [KeyEvent]s since the last time the message is dispatched.
  //
  // If _transitMode is [KeyDataTransitMode.keyDataThenRawKeyData], then [KeyEvent]s
  // are directly received from [handleKeyData]. If _transitMode is
  // [KeyDataTransitMode.rawKeyData], then [KeyEvent]s are converted from the raw
  // key data of [handleRawKeyMessage]. Either way, the accumulated key
  // events are only dispatched along with the next [KeyMessage] when a
  // dispatchable [RawKeyEvent] is available.
  final List<KeyEvent> _keyEventsSinceLastMessage = <KeyEvent>[];

  // When a RawKeyDownEvent is skipped ([RawKeyEventData.shouldDispatchEvent]
  // is false), its physical key will be recorded here, so that its up event
  // can also be properly skipped.
  final Set<PhysicalKeyboardKey> _skippedRawKeysPressed = <PhysicalKeyboardKey>{};

  /// Dispatch a key data to global and leaf listeners.
  ///
  /// This method is the handler to the global `onKeyData` API.
  ///
  /// This handler is deprecated, and will be removed. Use
  /// [HardwareKeyboard.addHandler] instead.
  @Deprecated(
    'No longer supported. Use HardwareKeyboard.instance.addHandler instead. '
    'This feature was deprecated after v3.18.0-2.0.pre.',
  )
  bool handleKeyData(ui.KeyData data) {
    _transitMode ??= KeyDataTransitMode.keyDataThenRawKeyData;
    switch (_transitMode!) {
      case KeyDataTransitMode.rawKeyData:
        assert(false, 'Should never encounter KeyData when transitMode is rawKeyData.');
        return false;
      case KeyDataTransitMode.keyDataThenRawKeyData:
        // Having 0 as the physical and logical ID indicates an empty key data
        // (the only occasion either field can be 0,) transmitted to ensure
        // that the transit mode is correctly inferred. These events should be
        // ignored.
        if (data.physical == 0 && data.logical == 0) {
          return false;
        }
        assert(data.physical != 0 && data.logical != 0);
        final KeyEvent event = _eventFromData(data);
        if (data.synthesized && _keyEventsSinceLastMessage.isEmpty) {
          // Dispatch the event instantly if both conditions are met:
          //
          // - The event is synthesized, therefore the result does not matter.
          // - The current queue is empty, therefore the order does not matter.
          //
          // This allows solitary synthesized `KeyEvent`s to be dispatched,
          // since they won't be followed by `RawKeyEvent`s.
          _hardwareKeyboard.handleKeyEvent(event);
          _dispatchKeyMessage(<KeyEvent>[event], null);
        } else {
          // Otherwise, postpone key event dispatching until the next raw
          // event. Normal key presses always send 0 or more `KeyEvent`s first,
          // then 1 `RawKeyEvent`.
          _keyEventsSinceLastMessage.add(event);
        }
        return false;
    }
  }

  bool _dispatchKeyMessage(List<KeyEvent> keyEvents, RawKeyEvent? rawEvent) {
    if (keyMessageHandler != null) {
      final KeyMessage message = KeyMessage(keyEvents, rawEvent);
      try {
        return keyMessageHandler!(message);
      } catch (exception, stack) {
        InformationCollector? collector;
        assert(() {
          collector =
              () => <DiagnosticsNode>[DiagnosticsProperty<KeyMessage>('KeyMessage', message)];
          return true;
        }());
        FlutterError.reportError(
          FlutterErrorDetails(
            exception: exception,
            stack: stack,
            library: 'services library',
            context: ErrorDescription('while processing the key message handler'),
            informationCollector: collector,
          ),
        );
      }
    }
    return false;
  }

  /// Handles a raw key message.
  ///
  /// This method is the handler to [SystemChannels.keyEvent], processing the
  /// JSON form of the native key message and returns the responds for the
  /// channel.
  ///
  /// This handler is deprecated, and will be removed. Use
  /// [HardwareKeyboard.addHandler] instead.
  @Deprecated(
    'No longer supported. Use HardwareKeyboard.instance.addHandler instead. '
    'This feature was deprecated after v3.18.0-2.0.pre.',
  )
  Future<Map<String, dynamic>> handleRawKeyMessage(dynamic message) async {
    if (_transitMode == null) {
      _transitMode = KeyDataTransitMode.rawKeyData;
      // Convert raw events using a listener so that conversion only occurs if
      // the raw event should be dispatched.
      _rawKeyboard.addListener(_convertRawEventAndStore);
    }
    final RawKeyEvent rawEvent = RawKeyEvent.fromMessage(message as Map<String, dynamic>);

    bool shouldDispatch = true;
    if (rawEvent is RawKeyDownEvent) {
      if (!rawEvent.data.shouldDispatchEvent()) {
        shouldDispatch = false;
        _skippedRawKeysPressed.add(rawEvent.physicalKey);
      } else {
        _skippedRawKeysPressed.remove(rawEvent.physicalKey);
      }
    } else if (rawEvent is RawKeyUpEvent) {
      if (_skippedRawKeysPressed.contains(rawEvent.physicalKey)) {
        _skippedRawKeysPressed.remove(rawEvent.physicalKey);
        shouldDispatch = false;
      }
    }

    bool handled = true;
    if (shouldDispatch) {
      // The following `handleRawKeyEvent` will call `_convertRawEventAndStore`
      // unless the event is not dispatched.
      handled = _rawKeyboard.handleRawKeyEvent(rawEvent);

      for (final KeyEvent event in _keyEventsSinceLastMessage) {
        handled = _hardwareKeyboard.handleKeyEvent(event) || handled;
      }
      if (_transitMode == KeyDataTransitMode.rawKeyData) {
        assert(
          setEquals(_rawKeyboard.physicalKeysPressed, _hardwareKeyboard.physicalKeysPressed),
          'RawKeyboard reported ${_rawKeyboard.physicalKeysPressed}, '
          'while HardwareKeyboard reported ${_hardwareKeyboard.physicalKeysPressed}',
        );
      }

      handled = _dispatchKeyMessage(_keyEventsSinceLastMessage, rawEvent) || handled;
      _keyEventsSinceLastMessage.clear();
    }

    return <String, dynamic>{'handled': handled};
  }

  ui.KeyEventDeviceType _convertDeviceType(RawKeyEvent rawEvent) {
    final RawKeyEventData data = rawEvent.data;
    // Device type is only available from Android.
    if (data is! RawKeyEventDataAndroid) {
      return ui.KeyEventDeviceType.keyboard;
    }

    switch (data.eventSource) {
      // https://developer.android.com/reference/android/view/InputDevice#SOURCE_KEYBOARD
      case 0x00000101:
        return ui.KeyEventDeviceType.keyboard;
      // https://developer.android.com/reference/android/view/InputDevice#SOURCE_DPAD
      case 0x00000201:
        return ui.KeyEventDeviceType.directionalPad;
      // https://developer.android.com/reference/android/view/InputDevice#SOURCE_GAMEPAD
      case 0x00000401:
        return ui.KeyEventDeviceType.gamepad;
      // https://developer.android.com/reference/android/view/InputDevice#SOURCE_JOYSTICK
      case 0x01000010:
        return ui.KeyEventDeviceType.joystick;
      // https://developer.android.com/reference/android/view/InputDevice#SOURCE_HDMI
      case 0x02000001:
        return ui.KeyEventDeviceType.hdmi;
    }
    return ui.KeyEventDeviceType.keyboard;
  }

  // Convert the raw event to key events, including synthesizing events for
  // modifiers, and store the key events in `_keyEventsSinceLastMessage`.
  //
  // This is only called when `_transitMode` is `rawKeyEvent` and if the raw
  // event should be dispatched.
  void _convertRawEventAndStore(RawKeyEvent rawEvent) {
    final PhysicalKeyboardKey physicalKey = rawEvent.physicalKey;
    final LogicalKeyboardKey logicalKey = rawEvent.logicalKey;
    final Set<PhysicalKeyboardKey> physicalKeysPressed = _hardwareKeyboard.physicalKeysPressed;
    final List<KeyEvent> eventAfterwards = <KeyEvent>[];
    final KeyEvent? mainEvent;
    final LogicalKeyboardKey? recordedLogicalMain = _hardwareKeyboard.lookUpLayout(physicalKey);
    final Duration timeStamp = ServicesBinding.instance.currentSystemFrameTimeStamp;
    final String? character = rawEvent.character == '' ? null : rawEvent.character;
    final ui.KeyEventDeviceType deviceType = _convertDeviceType(rawEvent);
    if (rawEvent is RawKeyDownEvent) {
      if (recordedLogicalMain == null) {
        mainEvent = KeyDownEvent(
          physicalKey: physicalKey,
          logicalKey: logicalKey,
          character: character,
          timeStamp: timeStamp,
          deviceType: deviceType,
        );
        physicalKeysPressed.add(physicalKey);
      } else {
        assert(physicalKeysPressed.contains(physicalKey));
        mainEvent = KeyRepeatEvent(
          physicalKey: physicalKey,
          logicalKey: recordedLogicalMain,
          character: character,
          timeStamp: timeStamp,
          deviceType: deviceType,
        );
      }
    } else {
      assert(
        rawEvent is RawKeyUpEvent,
        'Unexpected subclass of RawKeyEvent: ${rawEvent.runtimeType}',
      );
      if (recordedLogicalMain == null) {
        mainEvent = null;
      } else {
        mainEvent = KeyUpEvent(
          logicalKey: recordedLogicalMain,
          physicalKey: physicalKey,
          timeStamp: timeStamp,
          deviceType: deviceType,
        );
        physicalKeysPressed.remove(physicalKey);
      }
    }
    for (final PhysicalKeyboardKey key in physicalKeysPressed.difference(
      _rawKeyboard.physicalKeysPressed,
    )) {
      if (key == physicalKey) {
        // Somehow, a down event is dispatched but the key is absent from
        // keysPressed. Synthesize a up event for the key, but this event must
        // be added after the main key down event.
        eventAfterwards.add(
          KeyUpEvent(
            physicalKey: key,
            logicalKey: logicalKey,
            timeStamp: timeStamp,
            synthesized: true,
            deviceType: deviceType,
          ),
        );
      } else {
        _keyEventsSinceLastMessage.add(
          KeyUpEvent(
            physicalKey: key,
            logicalKey: _hardwareKeyboard.lookUpLayout(key)!,
            timeStamp: timeStamp,
            synthesized: true,
            deviceType: deviceType,
          ),
        );
      }
    }
    for (final PhysicalKeyboardKey key in _rawKeyboard.physicalKeysPressed.difference(
      physicalKeysPressed,
    )) {
      _keyEventsSinceLastMessage.add(
        KeyDownEvent(
          physicalKey: key,
          logicalKey: _rawKeyboard.lookUpLayout(key)!,
          timeStamp: timeStamp,
          synthesized: true,
          deviceType: deviceType,
        ),
      );
    }
    if (mainEvent != null) {
      _keyEventsSinceLastMessage.add(mainEvent);
    }
    _keyEventsSinceLastMessage.addAll(eventAfterwards);
  }

  /// Reset the inferred platform transit mode and related states.
  ///
  /// This method is only used in unit tests. In release mode, this is a no-op.
  @visibleForTesting
  void clearState() {
    assert(() {
      _transitMode = null;
      _rawKeyboard.removeListener(_convertRawEventAndStore);
      _keyEventsSinceLastMessage.clear();
      return true;
    }());
  }

  static KeyEvent _eventFromData(ui.KeyData keyData) {
    final PhysicalKeyboardKey physicalKey =
        PhysicalKeyboardKey.findKeyByCode(keyData.physical) ??
        PhysicalKeyboardKey(keyData.physical);
    final LogicalKeyboardKey logicalKey =
        LogicalKeyboardKey.findKeyByKeyId(keyData.logical) ?? LogicalKeyboardKey(keyData.logical);
    final Duration timeStamp = keyData.timeStamp;
    switch (keyData.type) {
      case ui.KeyEventType.down:
        return KeyDownEvent(
          physicalKey: physicalKey,
          logicalKey: logicalKey,
          timeStamp: timeStamp,
          character: keyData.character,
          synthesized: keyData.synthesized,
          deviceType: keyData.deviceType,
        );
      case ui.KeyEventType.up:
        assert(keyData.character == null);
        return KeyUpEvent(
          physicalKey: physicalKey,
          logicalKey: logicalKey,
          timeStamp: timeStamp,
          synthesized: keyData.synthesized,
          deviceType: keyData.deviceType,
        );
      case ui.KeyEventType.repeat:
        return KeyRepeatEvent(
          physicalKey: physicalKey,
          logicalKey: logicalKey,
          timeStamp: timeStamp,
          character: keyData.character,
          deviceType: keyData.deviceType,
        );
    }
  }
}
