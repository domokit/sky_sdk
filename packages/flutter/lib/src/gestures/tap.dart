// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'multitap.dart';
library;

import 'package:flutter/foundation.dart';

import 'arena.dart';
import 'constants.dart';
import 'events.dart';
import 'recognizer.dart';

export 'dart:ui' show Offset, PointerDeviceKind;

export 'package:flutter/foundation.dart' show DiagnosticPropertiesBuilder;
export 'package:vector_math/vector_math_64.dart' show Matrix4;

export 'arena.dart' show GestureDisposition;
export 'events.dart' show PointerCancelEvent, PointerDownEvent, PointerEvent, PointerUpEvent;

/// Details for [GestureTapDownCallback], such as position.
///
/// See also:
///
///  * [GestureDetector.onTapDown], which receives this information.
///  * [TapGestureRecognizer], which passes this information to one of its callbacks.
class TapDownDetails {
  /// Creates details for a [GestureTapDownCallback].
  TapDownDetails({this.globalPosition = Offset.zero, Offset? localPosition, this.kind})
    : localPosition = localPosition ?? globalPosition;

  /// The global position at which the pointer contacted the screen.
  final Offset globalPosition;

  /// The kind of the device that initiated the event.
  final PointerDeviceKind? kind;

  /// The local position at which the pointer contacted the screen.
  final Offset localPosition;
}

/// {@template flutter.gestures.tap.GestureTapDownCallback}
/// Signature for when a pointer that might cause a tap has contacted the
/// screen.
///
/// The position at which the pointer contacted the screen is available in the
/// `details`.
/// {@endtemplate}
///
/// See also:
///
///  * [GestureDetector.onTapDown], which matches this signature.
///  * [TapGestureRecognizer], which uses this signature in one of its callbacks.
typedef GestureTapDownCallback = void Function(TapDownDetails details);

/// Details for [GestureTapUpCallback], such as position.
///
/// See also:
///
///  * [GestureDetector.onTapUp], which receives this information.
///  * [TapGestureRecognizer], which passes this information to one of its callbacks.
class TapUpDetails {
  /// Creates a [TapUpDetails] data object.
  TapUpDetails({required this.kind, this.globalPosition = Offset.zero, Offset? localPosition})
    : localPosition = localPosition ?? globalPosition;

  /// The global position at which the pointer contacted the screen.
  final Offset globalPosition;

  /// The local position at which the pointer contacted the screen.
  final Offset localPosition;

  /// The kind of the device that initiated the event.
  final PointerDeviceKind kind;
}

/// {@template flutter.gestures.tap.GestureTapUpCallback}
/// Signature for when a pointer that will trigger a tap has stopped contacting
/// the screen.
///
/// The position at which the pointer stopped contacting the screen is available
/// in the `details`.
/// {@endtemplate}
///
/// See also:
///
///  * [GestureDetector.onTapUp], which matches this signature.
///  * [TapGestureRecognizer], which uses this signature in one of its callbacks.
typedef GestureTapUpCallback = void Function(TapUpDetails details);

/// Signature for when a tap has occurred.
///
/// See also:
///
///  * [GestureDetector.onTap], which matches this signature.
///  * [TapGestureRecognizer], which uses this signature in one of its callbacks.
typedef GestureTapCallback = void Function();

/// Signature for when the pointer that previously triggered a
/// [GestureTapDownCallback] will not end up causing a tap.
///
/// See also:
///
///  * [GestureDetector.onTapCancel], which matches this signature.
///  * [TapGestureRecognizer], which uses this signature in one of its callbacks.
typedef GestureTapCancelCallback = void Function();

/// A base class for gesture recognizers that recognize taps.
///
/// Gesture recognizers take part in gesture arenas to enable potential gestures
/// to be disambiguated from each other. This process is managed by a
/// [GestureArenaManager].
///
/// A tap is defined as a sequence of events that starts with a down, followed
/// by optional moves, then ends with an up. All move events must contain the
/// same `buttons` as the down event, and must not be too far from the initial
/// position. The gesture is rejected on any violation, a cancel event, or
/// if any other recognizers wins the arena. It is accepted only when it is the
/// last member of the arena.
///
/// The [BaseTapGestureRecognizer] considers all the pointers involved in the
/// pointer event sequence as contributing to one gesture. For this reason,
/// extra pointer interactions during a tap sequence are not recognized as
/// additional taps. For example, down-1, down-2, up-1, up-2 produces only one
/// tap on up-1.
///
/// The [BaseTapGestureRecognizer] can not be directly used, since it does not
/// define which buttons to accept, or what to do when a tap happens. If you
/// want to build a custom tap recognizer, extend this class by overriding
/// [isPointerAllowed] and the handler methods.
///
/// See also:
///
///  * [TapGestureRecognizer], a ready-to-use tap recognizer that recognizes
///    taps of the primary button and taps of the secondary button.
///  * [ModalBarrier], a widget that uses a custom tap recognizer that accepts
///    any buttons.
abstract class BaseTapGestureRecognizer extends PrimaryPointerGestureRecognizer {
  /// Creates a tap gesture recognizer.
  ///
  /// {@macro flutter.gestures.GestureRecognizer.supportedDevices}
  BaseTapGestureRecognizer({super.debugOwner, super.supportedDevices, super.allowedButtonsFilter})
    : super(deadline: kPressTimeout);

  bool _sentTapDown = false;
  bool _wonArenaForPrimaryPointer = false;

  PointerDownEvent? _down;
  PointerUpEvent? _up;

  /// A pointer has contacted the screen, which might be the start of a tap.
  ///
  /// This triggers after the down event, once a short timeout ([deadline]) has
  /// elapsed, or once the gesture has won the arena, whichever comes first.
  ///
  /// The parameter `down` is the down event of the primary pointer that started
  /// the tap sequence.
  ///
  /// If this recognizer doesn't win the arena, [handleTapCancel] is called next.
  /// Otherwise, [handleTapUp] is called next.
  @protected
  void handleTapDown({required PointerDownEvent down});

  /// A pointer has stopped contacting the screen, which is recognized as a tap.
  ///
  /// This triggers on the up event if the recognizer wins the arena with it
  /// or has previously won.
  ///
  /// The parameter `down` is the down event of the primary pointer that started
  /// the tap sequence, and `up` is the up event that ended the tap sequence.
  ///
  /// If this recognizer doesn't win the arena, [handleTapCancel] is called
  /// instead.
  @protected
  void handleTapUp({required PointerDownEvent down, required PointerUpEvent up});

  /// A pointer that previously triggered [handleTapDown] will not end up
  /// causing a tap.
  ///
  /// This triggers once the gesture loses the arena if [handleTapDown] has
  /// been previously triggered.
  ///
  /// The parameter `down` is the down event of the primary pointer that started
  /// the tap sequence; `cancel` is the cancel event, which might be null;
  /// `reason` is a short description of the cause if `cancel` is null, which
  /// can be "forced" if other gestures won the arena, or "spontaneous"
  /// otherwise.
  ///
  /// If this recognizer wins the arena, [handleTapUp] is called instead.
  @protected
  void handleTapCancel({
    required PointerDownEvent down,
    PointerCancelEvent? cancel,
    required String reason,
  });

  @override
  void addAllowedPointer(PointerDownEvent event) {
    if (state == GestureRecognizerState.ready) {
      // If there is no result in the previous gesture arena,
      // we ignore them and prepare to accept a new pointer.
      if (_down != null && _up != null) {
        assert(_down!.pointer == _up!.pointer);
        _reset();
      }

      assert(_down == null && _up == null);
      // `_down` must be assigned in this method instead of `handlePrimaryPointer`,
      // because `acceptGesture` might be called before `handlePrimaryPointer`,
      // which relies on `_down` to call `handleTapDown`.
      _down = event;
    }
    if (_down != null) {
      // This happens when this tap gesture has been rejected while the pointer
      // is down (i.e. due to movement), when another allowed pointer is added,
      // in which case all pointers are ignored. The `_down` being null
      // means that _reset() has been called, since it is always set at the
      // first allowed down event and will not be cleared except for reset(),
      super.addAllowedPointer(event);
    }
  }

  @override
  @protected
  void startTrackingPointer(int pointer, [Matrix4? transform]) {
    // The recognizer should never track any pointers when `_down` is null,
    // because calling `_checkDown` in this state will throw exception.
    assert(_down != null);
    super.startTrackingPointer(pointer, transform);
  }

  @override
  void handlePrimaryPointer(PointerEvent event) {
    if (event is PointerUpEvent) {
      _up = event;
      _checkUp();
    } else if (event is PointerCancelEvent) {
      resolve(GestureDisposition.rejected);
      if (_sentTapDown) {
        _checkCancel(event, '');
      }
      _reset();
    } else if (event.buttons != _down!.buttons) {
      resolve(GestureDisposition.rejected);
      stopTrackingPointer(primaryPointer!);
    }
  }

  @override
  void resolve(GestureDisposition disposition) {
    if (_wonArenaForPrimaryPointer && disposition == GestureDisposition.rejected) {
      // This can happen if the gesture has been canceled. For example, when
      // the pointer has exceeded the touch slop, the buttons have been changed,
      // or if the recognizer is disposed.
      assert(_sentTapDown);
      _checkCancel(null, 'spontaneous');
      _reset();
    }
    super.resolve(disposition);
  }

  @override
  void didExceedDeadline() {
    _checkDown();
  }

  @override
  void acceptGesture(int pointer) {
    super.acceptGesture(pointer);
    if (pointer == primaryPointer) {
      _checkDown();
      _wonArenaForPrimaryPointer = true;
      _checkUp();
    }
  }

  @override
  void rejectGesture(int pointer) {
    super.rejectGesture(pointer);
    if (pointer == primaryPointer) {
      // Another gesture won the arena.
      assert(state != GestureRecognizerState.possible);
      if (_sentTapDown) {
        _checkCancel(null, 'forced');
      }
      _reset();
    }
  }

  void _checkDown() {
    if (_sentTapDown) {
      return;
    }
    handleTapDown(down: _down!);
    _sentTapDown = true;
  }

  void _checkUp() {
    if (!_wonArenaForPrimaryPointer || _up == null) {
      return;
    }
    assert(_up!.pointer == _down!.pointer);
    handleTapUp(down: _down!, up: _up!);
    _reset();
  }

  void _checkCancel(PointerCancelEvent? event, String note) {
    handleTapCancel(down: _down!, cancel: event, reason: note);
  }

  void _reset() {
    _sentTapDown = false;
    _wonArenaForPrimaryPointer = false;
    _up = null;
    _down = null;
  }

  @override
  String get debugDescription => 'base tap';

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      FlagProperty(
        'wonArenaForPrimaryPointer',
        value: _wonArenaForPrimaryPointer,
        ifTrue: 'won arena',
      ),
    );
    properties.add(DiagnosticsProperty<Offset>('finalPosition', _up?.position, defaultValue: null));
    properties.add(
      DiagnosticsProperty<Offset>(
        'finalLocalPosition',
        _up?.localPosition,
        defaultValue: _up?.position,
      ),
    );
    properties.add(DiagnosticsProperty<int>('button', _down?.buttons, defaultValue: null));
    properties.add(FlagProperty('sentTapDown', value: _sentTapDown, ifTrue: 'sent tap down'));
  }
}

/// Recognizes taps.
///
/// Gesture recognizers take part in gesture arenas to enable potential gestures
/// to be disambiguated from each other. This process is managed by a
/// [GestureArenaManager].
///
/// [TapGestureRecognizer] considers all the pointers involved in the pointer
/// event sequence as contributing to one gesture. For this reason, extra
/// pointer interactions during a tap sequence are not recognized as additional
/// taps. For example, down-1, down-2, up-1, up-2 produces only one tap on up-1.
///
/// [TapGestureRecognizer] competes on pointer events of [kPrimaryButton] only
/// when it has at least one non-null `onTap*` callback, on events of
/// [kSecondaryButton] only when it has at least one non-null `onSecondaryTap*`
/// callback, and on events of [kTertiaryButton] only when it has at least
/// one non-null `onTertiaryTap*` callback. If it has no callbacks, it is a
/// no-op.
///
/// {@template flutter.gestures.tap.TapGestureRecognizer.allowedButtonsFilter}
/// The [allowedButtonsFilter] argument only gives this recognizer the
/// ability to limit the buttons it accepts. It does not provide the
/// ability to recognize any buttons beyond the ones it already accepts:
/// kPrimaryButton, kSecondaryButton or kTertiaryButton. Therefore, a
/// combined value of `kPrimaryButton & kSecondaryButton` would be ignored,
/// but `kPrimaryButton | kSecondaryButton` would be allowed, as long as
/// only one of them is selected at a time.
/// {@endtemplate}
///
/// See also:
///
///  * [GestureDetector.onTap], which uses this recognizer.
///  * [MultiTapGestureRecognizer]
class TapGestureRecognizer extends BaseTapGestureRecognizer {
  /// Creates a tap gesture recognizer.
  ///
  /// {@macro flutter.gestures.GestureRecognizer.supportedDevices}
  TapGestureRecognizer({super.debugOwner, super.supportedDevices, super.allowedButtonsFilter});

  /// {@template flutter.gestures.tap.TapGestureRecognizer.onTapDown}
  /// A pointer has contacted the screen at a particular location with a primary
  /// button, which might be the start of a tap.
  /// {@endtemplate}
  ///
  /// This triggers after the down event, once a short timeout ([deadline]) has
  /// elapsed, or once the gestures has won the arena, whichever comes first.
  ///
  /// If this recognizer doesn't win the arena, [onTapCancel] is called next.
  /// Otherwise, [onTapUp] is called next.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  ///  * [onSecondaryTapDown], a similar callback but for a secondary button.
  ///  * [onTertiaryTapDown], a similar callback but for a tertiary button.
  ///  * [TapDownDetails], which is passed as an argument to this callback.
  ///  * [GestureDetector.onTapDown], which exposes this callback.
  GestureTapDownCallback? onTapDown;

  /// {@template flutter.gestures.tap.TapGestureRecognizer.onTapUp}
  /// A pointer has stopped contacting the screen at a particular location,
  /// which is recognized as a tap of a primary button.
  /// {@endtemplate}
  ///
  /// This triggers on the up event, if the recognizer wins the arena with it
  /// or has previously won, immediately followed by [onTap].
  ///
  /// If this recognizer doesn't win the arena, [onTapCancel] is called instead.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  ///  * [onSecondaryTapUp], a similar callback but for a secondary button.
  ///  * [onTertiaryTapUp], a similar callback but for a tertiary button.
  ///  * [TapUpDetails], which is passed as an argument to this callback.
  ///  * [GestureDetector.onTapUp], which exposes this callback.
  GestureTapUpCallback? onTapUp;

  /// A pointer has stopped contacting the screen, which is recognized as a tap
  /// of a primary button.
  ///
  /// This triggers on the up event, if the recognizer wins the arena with it
  /// or has previously won, immediately following [onTapUp].
  ///
  /// If this recognizer doesn't win the arena, [onTapCancel] is called instead.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  ///  * [onSecondaryTap], a similar callback but for a secondary button.
  ///  * [onTapUp], which has the same timing but with details.
  ///  * [GestureDetector.onTap], which exposes this callback.
  GestureTapCallback? onTap;

  /// {@template flutter.gestures.tap.TapGestureRecognizer.onTapCancel}
  /// A pointer that previously triggered [onTapDown] will not end up causing
  /// a tap.
  /// {@endtemplate}
  ///
  /// This triggers once the gesture loses the arena if [onTapDown] has
  /// previously been triggered.
  ///
  /// If this recognizer wins the arena, [onTapUp] and [onTap] are called
  /// instead.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  ///  * [onSecondaryTapCancel], a similar callback but for a secondary button.
  ///  * [onTertiaryTapCancel], a similar callback but for a tertiary button.
  ///  * [GestureDetector.onTapCancel], which exposes this callback.
  GestureTapCancelCallback? onTapCancel;

  /// {@template flutter.gestures.tap.TapGestureRecognizer.onSecondaryTap}
  /// A pointer has stopped contacting the screen, which is recognized as a tap
  /// of a secondary button.
  /// {@endtemplate}
  ///
  /// This triggers on the up event, if the recognizer wins the arena with it or
  /// has previously won, immediately following [onSecondaryTapUp].
  ///
  /// If this recognizer doesn't win the arena, [onSecondaryTapCancel] is called
  /// instead.
  ///
  /// See also:
  ///
  ///  * [kSecondaryButton], the button this callback responds to.
  ///  * [onSecondaryTapUp], which has the same timing but with details.
  ///  * [GestureDetector.onSecondaryTap], which exposes this callback.
  GestureTapCallback? onSecondaryTap;

  /// {@template flutter.gestures.tap.TapGestureRecognizer.onSecondaryTapDown}
  /// A pointer has contacted the screen at a particular location with a
  /// secondary button, which might be the start of a secondary tap.
  /// {@endtemplate}
  ///
  /// This triggers after the down event, once a short timeout ([deadline]) has
  /// elapsed, or once the gestures has won the arena, whichever comes first.
  ///
  /// If this recognizer doesn't win the arena, [onSecondaryTapCancel] is called
  /// next. Otherwise, [onSecondaryTapUp] is called next.
  ///
  /// See also:
  ///
  ///  * [kSecondaryButton], the button this callback responds to.
  ///  * [onTapDown], a similar callback but for a primary button.
  ///  * [onTertiaryTapDown], a similar callback but for a tertiary button.
  ///  * [TapDownDetails], which is passed as an argument to this callback.
  ///  * [GestureDetector.onSecondaryTapDown], which exposes this callback.
  GestureTapDownCallback? onSecondaryTapDown;

  /// {@template flutter.gestures.tap.TapGestureRecognizer.onSecondaryTapUp}
  /// A pointer has stopped contacting the screen at a particular location,
  /// which is recognized as a tap of a secondary button.
  /// {@endtemplate}
  ///
  /// This triggers on the up event if the recognizer wins the arena with it
  /// or has previously won.
  ///
  /// If this recognizer doesn't win the arena, [onSecondaryTapCancel] is called
  /// instead.
  ///
  /// See also:
  ///
  ///  * [onSecondaryTap], a handler triggered right after this one that doesn't
  ///    pass any details about the tap.
  ///  * [kSecondaryButton], the button this callback responds to.
  ///  * [onTapUp], a similar callback but for a primary button.
  ///  * [onTertiaryTapUp], a similar callback but for a tertiary button.
  ///  * [TapUpDetails], which is passed as an argument to this callback.
  ///  * [GestureDetector.onSecondaryTapUp], which exposes this callback.
  GestureTapUpCallback? onSecondaryTapUp;

  /// {@template flutter.gestures.tap.TapGestureRecognizer.onSecondaryTapCancel}
  /// A pointer that previously triggered [onSecondaryTapDown] will not end up
  /// causing a tap.
  /// {@endtemplate}
  ///
  /// This triggers once the gesture loses the arena if [onSecondaryTapDown]
  /// has previously been triggered.
  ///
  /// If this recognizer wins the arena, [onSecondaryTapUp] is called instead.
  ///
  /// See also:
  ///
  ///  * [kSecondaryButton], the button this callback responds to.
  ///  * [onTapCancel], a similar callback but for a primary button.
  ///  * [onTertiaryTapCancel], a similar callback but for a tertiary button.
  ///  * [GestureDetector.onSecondaryTapCancel], which exposes this callback.
  GestureTapCancelCallback? onSecondaryTapCancel;

  /// A pointer has contacted the screen at a particular location with a
  /// tertiary button, which might be the start of a tertiary tap.
  ///
  /// This triggers after the down event, once a short timeout ([deadline]) has
  /// elapsed, or once the gestures has won the arena, whichever comes first.
  ///
  /// If this recognizer doesn't win the arena, [onTertiaryTapCancel] is called
  /// next. Otherwise, [onTertiaryTapUp] is called next.
  ///
  /// See also:
  ///
  ///  * [kTertiaryButton], the button this callback responds to.
  ///  * [onTapDown], a similar callback but for a primary button.
  ///  * [onSecondaryTapDown], a similar callback but for a secondary button.
  ///  * [TapDownDetails], which is passed as an argument to this callback.
  ///  * [GestureDetector.onTertiaryTapDown], which exposes this callback.
  GestureTapDownCallback? onTertiaryTapDown;

  /// A pointer has stopped contacting the screen at a particular location,
  /// which is recognized as a tap of a tertiary button.
  ///
  /// This triggers on the up event if the recognizer wins the arena with it
  /// or has previously won.
  ///
  /// If this recognizer doesn't win the arena, [onTertiaryTapCancel] is called
  /// instead.
  ///
  /// See also:
  ///
  ///  * [kTertiaryButton], the button this callback responds to.
  ///  * [onTapUp], a similar callback but for a primary button.
  ///  * [onSecondaryTapUp], a similar callback but for a secondary button.
  ///  * [TapUpDetails], which is passed as an argument to this callback.
  ///  * [GestureDetector.onTertiaryTapUp], which exposes this callback.
  GestureTapUpCallback? onTertiaryTapUp;

  /// A pointer that previously triggered [onTertiaryTapDown] will not end up
  /// causing a tap.
  ///
  /// This triggers once the gesture loses the arena if [onTertiaryTapDown]
  /// has previously been triggered.
  ///
  /// If this recognizer wins the arena, [onTertiaryTapUp] is called instead.
  ///
  /// See also:
  ///
  ///  * [kSecondaryButton], the button this callback responds to.
  ///  * [onTapCancel], a similar callback but for a primary button.
  ///  * [onSecondaryTapCancel], a similar callback but for a secondary button.
  ///  * [GestureDetector.onTertiaryTapCancel], which exposes this callback.
  GestureTapCancelCallback? onTertiaryTapCancel;

  @override
  bool isPointerAllowed(PointerDownEvent event) {
    switch (event.buttons) {
      case kPrimaryButton:
        if (onTapDown == null && onTap == null && onTapUp == null && onTapCancel == null) {
          return false;
        }
      case kSecondaryButton:
        if (onSecondaryTap == null &&
            onSecondaryTapDown == null &&
            onSecondaryTapUp == null &&
            onSecondaryTapCancel == null) {
          return false;
        }
      case kTertiaryButton:
        if (onTertiaryTapDown == null && onTertiaryTapUp == null && onTertiaryTapCancel == null) {
          return false;
        }
      default:
        return false;
    }
    return super.isPointerAllowed(event);
  }

  @protected
  @override
  void handleTapDown({required PointerDownEvent down}) {
    final TapDownDetails details = TapDownDetails(
      globalPosition: down.position,
      localPosition: down.localPosition,
      kind: getKindForPointer(down.pointer),
    );
    switch (down.buttons) {
      case kPrimaryButton:
        if (onTapDown != null) {
          invokeCallback<void>('onTapDown', () => onTapDown!(details));
        }
      case kSecondaryButton:
        if (onSecondaryTapDown != null) {
          invokeCallback<void>('onSecondaryTapDown', () => onSecondaryTapDown!(details));
        }
      case kTertiaryButton:
        if (onTertiaryTapDown != null) {
          invokeCallback<void>('onTertiaryTapDown', () => onTertiaryTapDown!(details));
        }
      default:
    }
  }

  @protected
  @override
  void handleTapUp({required PointerDownEvent down, required PointerUpEvent up}) {
    final TapUpDetails details = TapUpDetails(
      kind: up.kind,
      globalPosition: up.position,
      localPosition: up.localPosition,
    );
    switch (down.buttons) {
      case kPrimaryButton:
        if (onTapUp != null) {
          invokeCallback<void>('onTapUp', () => onTapUp!(details));
        }
        if (onTap != null) {
          invokeCallback<void>('onTap', onTap!);
        }
      case kSecondaryButton:
        if (onSecondaryTapUp != null) {
          invokeCallback<void>('onSecondaryTapUp', () => onSecondaryTapUp!(details));
        }
        if (onSecondaryTap != null) {
          invokeCallback<void>('onSecondaryTap', () => onSecondaryTap!());
        }
      case kTertiaryButton:
        if (onTertiaryTapUp != null) {
          invokeCallback<void>('onTertiaryTapUp', () => onTertiaryTapUp!(details));
        }
      default:
    }
  }

  @protected
  @override
  void handleTapCancel({
    required PointerDownEvent down,
    PointerCancelEvent? cancel,
    required String reason,
  }) {
    final String note = reason == '' ? reason : '$reason ';
    switch (down.buttons) {
      case kPrimaryButton:
        if (onTapCancel != null) {
          invokeCallback<void>('${note}onTapCancel', onTapCancel!);
        }
      case kSecondaryButton:
        if (onSecondaryTapCancel != null) {
          invokeCallback<void>('${note}onSecondaryTapCancel', onSecondaryTapCancel!);
        }
      case kTertiaryButton:
        if (onTertiaryTapCancel != null) {
          invokeCallback<void>('${note}onTertiaryTapCancel', onTertiaryTapCancel!);
        }
      default:
    }
  }

  @override
  String get debugDescription => 'tap';
}
