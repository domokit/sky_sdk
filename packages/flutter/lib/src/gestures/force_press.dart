// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart' show clampDouble;

import 'events.dart';
import 'recognizer.dart';

export 'dart:ui' show Offset, PointerDeviceKind;

export 'events.dart' show PointerDownEvent, PointerEvent;

enum _ForceState {
  // No pointer has touched down and the detector is ready for a pointer down to occur.
  ready,

  // A pointer has touched down, but a force press gesture has not yet been detected.
  possible,

  // A pointer is down and a force press gesture has been detected. However, if
  // the ForcePressGestureRecognizer is the only recognizer in the arena, thus
  // accepted as soon as the gesture state is possible, the gesture will not
  // yet have started.
  accepted,

  // A pointer is down and the gesture has started, ie. the pressure of the pointer
  // has just become greater than the ForcePressGestureRecognizer.startPressure.
  started,

  // A pointer is down and the pressure of the pointer has just become greater
  // than the ForcePressGestureRecognizer.peakPressure. Even after a pointer
  // crosses this threshold, onUpdate callbacks will still be sent.
  peaked,
}

/// Details object for callbacks that use [GestureForcePressStartCallback],
/// [GestureForcePressPeakCallback], [GestureForcePressEndCallback] or
/// [GestureForcePressUpdateCallback].
///
/// See also:
///
///  * [ForcePressGestureRecognizer.onStart], [ForcePressGestureRecognizer.onPeak],
///    [ForcePressGestureRecognizer.onEnd], and [ForcePressGestureRecognizer.onUpdate]
///    which use [ForcePressDetails].
class ForcePressDetails {
  /// Creates details for a [GestureForcePressStartCallback],
  /// [GestureForcePressPeakCallback] or [GestureForcePressEndCallback].
  ForcePressDetails({required this.globalPosition, Offset? localPosition, required this.pressure})
    : localPosition = localPosition ?? globalPosition;

  /// The global position at which the function was called.
  final Offset globalPosition;

  /// The local position at which the function was called.
  final Offset localPosition;

  /// The pressure of the pointer on the screen.
  final double pressure;
}

/// Signature used by a [ForcePressGestureRecognizer] for when a pointer has
/// pressed with at least [ForcePressGestureRecognizer.startPressure].
typedef GestureForcePressStartCallback = void Function(ForcePressDetails details);

/// Signature used by [ForcePressGestureRecognizer] for when a pointer that has
/// pressed with at least [ForcePressGestureRecognizer.peakPressure].
typedef GestureForcePressPeakCallback = void Function(ForcePressDetails details);

/// Signature used by [ForcePressGestureRecognizer] during the frames
/// after the triggering of a [ForcePressGestureRecognizer.onStart] callback.
typedef GestureForcePressUpdateCallback = void Function(ForcePressDetails details);

/// Signature for when the pointer that previously triggered a
/// [ForcePressGestureRecognizer.onStart] callback is no longer in contact
/// with the screen.
typedef GestureForcePressEndCallback = void Function(ForcePressDetails details);

/// Signature used by [ForcePressGestureRecognizer] for interpolating the raw
/// device pressure to a value in the range `[0, 1]` given the device's pressure
/// min and pressure max.
typedef GestureForceInterpolation =
    double Function(double pressureMin, double pressureMax, double pressure);

/// Recognizes a force press on devices that have force sensors.
///
/// Only the force from a single pointer is used to invoke events. A tap
/// recognizer will win against this recognizer on pointer up as long as the
/// pointer has not pressed with a force greater than
/// [ForcePressGestureRecognizer.startPressure]. A long press recognizer will
/// win when the press down time exceeds the threshold time as long as the
/// pointer's pressure was never greater than
/// [ForcePressGestureRecognizer.startPressure] in that duration.
///
/// As of November, 2018 iPhone devices of generation 6S and higher have
/// force touch functionality, with the exception of the iPhone XR. In addition,
/// a small handful of Android devices have this functionality as well.
///
/// Devices with faux screen pressure sensors like the Pixel 2 and 3 will not
/// send any force press related callbacks.
///
/// Reported pressure will always be in the range 0.0 to 1.0, where 1.0 is
/// maximum pressure and 0.0 is minimum pressure. If using a custom
/// [interpolation] callback, the pressure reported will correspond to that
/// custom curve.
class ForcePressGestureRecognizer extends OneSequenceGestureRecognizer {
  /// Creates a force press gesture recognizer.
  ///
  /// The [startPressure] defaults to 0.4, and [peakPressure] defaults to 0.85
  /// where a value of 0.0 is no pressure and a value of 1.0 is maximum pressure.
  ///
  /// The [startPressure], [peakPressure] and [interpolation] arguments must not
  /// be null. The [peakPressure] argument must be greater than [startPressure].
  /// The [interpolation] callback must always return a value in the range 0.0
  /// to 1.0 for values of `pressure` that are between `pressureMin` and
  /// `pressureMax`.
  ///
  /// {@macro flutter.gestures.GestureRecognizer.supportedDevices}
  ForcePressGestureRecognizer({
    this.startPressure = 0.4,
    this.peakPressure = 0.85,
    this.interpolation = _inverseLerp,
    super.debugOwner,
    super.supportedDevices,
    super.allowedButtonsFilter,
  }) : assert(peakPressure > startPressure);

  /// A pointer is in contact with the screen and has just pressed with a force
  /// exceeding the [startPressure]. Consequently, if there were other gesture
  /// detectors, only the force press gesture will be detected and all others
  /// will be rejected.
  ///
  /// The position of the pointer is provided in the callback's `details`
  /// argument, which is a [ForcePressDetails] object.
  GestureForcePressStartCallback? onStart;

  /// A pointer is in contact with the screen and is either moving on the plane
  /// of the screen, pressing the screen with varying forces or both
  /// simultaneously.
  ///
  /// This callback will be invoked for every pointer event after the invocation
  /// of [onStart] and/or [onPeak] and before the invocation of [onEnd], no
  /// matter what the pressure is during this time period. The position and
  /// pressure of the pointer is provided in the callback's `details` argument,
  /// which is a [ForcePressDetails] object.
  GestureForcePressUpdateCallback? onUpdate;

  /// A pointer is in contact with the screen and has just pressed with a force
  /// exceeding the [peakPressure]. This is an arbitrary second level action
  /// threshold and isn't necessarily the maximum possible device pressure
  /// (which is 1.0).
  ///
  /// The position of the pointer is provided in the callback's `details`
  /// argument, which is a [ForcePressDetails] object.
  GestureForcePressPeakCallback? onPeak;

  /// A pointer is no longer in contact with the screen.
  ///
  /// The position of the pointer is provided in the callback's `details`
  /// argument, which is a [ForcePressDetails] object.
  GestureForcePressEndCallback? onEnd;

  /// The pressure of the press required to initiate a force press.
  ///
  /// A value of 0.0 is no pressure, and 1.0 is maximum pressure.
  final double startPressure;

  /// The pressure of the press required to peak a force press.
  ///
  /// A value of 0.0 is no pressure, and 1.0 is maximum pressure. This value
  /// must be greater than [startPressure].
  final double peakPressure;

  /// The function used to convert the raw device pressure values into a value
  /// in the range 0.0 to 1.0.
  ///
  /// The function takes in the device's minimum, maximum and raw touch pressure
  /// and returns a value in the range 0.0 to 1.0 denoting the interpolated
  /// touch pressure.
  ///
  /// This function must always return values in the range 0.0 to 1.0 given a
  /// pressure that is between the minimum and maximum pressures. It may return
  /// `double.NaN` for values that it does not want to support.
  ///
  /// By default, the function is a linear interpolation; however, changing the
  /// function could be useful to accommodate variations in the way different
  /// devices respond to pressure, or to change how animations from pressure
  /// feedback are rendered.
  ///
  /// For example, an ease-in curve can be used to determine the interpolated
  /// value:
  ///
  /// ```dart
  /// double interpolateWithEasing(double min, double max, double t) {
  ///    final double lerp = (t - min) / (max - min);
  ///    return Curves.easeIn.transform(lerp);
  /// }
  /// ```
  final GestureForceInterpolation interpolation;

  late OffsetPair _lastPosition;
  late double _lastPressure;
  _ForceState _state = _ForceState.ready;

  @override
  void addAllowedPointer(PointerDownEvent event) {
    // If the device has a maximum pressure of less than or equal to 1, it
    // doesn't have touch pressure sensing capabilities. Do not participate
    // in the gesture arena.
    if (event.pressureMax <= 1.0) {
      resolve(GestureDisposition.rejected);
    } else {
      super.addAllowedPointer(event);
      if (_state == _ForceState.ready) {
        _state = _ForceState.possible;
        _lastPosition = OffsetPair.fromEventPosition(event);
      }
    }
  }

  @override
  void handleEvent(PointerEvent event) {
    assert(_state != _ForceState.ready);
    // A static pointer with changes in pressure creates PointerMoveEvent events.
    if (event is PointerMoveEvent || event is PointerDownEvent) {
      final double pressure = interpolation(event.pressureMin, event.pressureMax, event.pressure);
      assert(
        (pressure >= 0.0 &&
                pressure <= 1.0) || // Interpolated pressure must be between 1.0 and 0.0...
            pressure
                .isNaN, // and interpolation may return NaN for values it doesn't want to support...
      );

      _lastPosition = OffsetPair.fromEventPosition(event);
      _lastPressure = pressure;

      if (_state == _ForceState.possible) {
        if (pressure > startPressure) {
          _state = _ForceState.started;
          resolve(GestureDisposition.accepted);
        } else if (event.delta.distanceSquared > computeHitSlop(event.kind, gestureSettings)) {
          resolve(GestureDisposition.rejected);
        }
      }
      // In case this is the only gesture detector we still don't want to start
      // the gesture until the pressure is greater than the startPressure.
      if (pressure > startPressure && _state == _ForceState.accepted) {
        _state = _ForceState.started;
        if (onStart != null) {
          invokeCallback<void>(
            'onStart',
            () => onStart!(
              ForcePressDetails(
                pressure: pressure,
                globalPosition: _lastPosition.global,
                localPosition: _lastPosition.local,
              ),
            ),
          );
        }
      }
      if (onPeak != null && pressure > peakPressure && (_state == _ForceState.started)) {
        _state = _ForceState.peaked;
        if (onPeak != null) {
          invokeCallback<void>(
            'onPeak',
            () => onPeak!(
              ForcePressDetails(
                pressure: pressure,
                globalPosition: event.position,
                localPosition: event.localPosition,
              ),
            ),
          );
        }
      }
      if (onUpdate != null &&
          !pressure.isNaN &&
          (_state == _ForceState.started || _state == _ForceState.peaked)) {
        if (onUpdate != null) {
          invokeCallback<void>(
            'onUpdate',
            () => onUpdate!(
              ForcePressDetails(
                pressure: pressure,
                globalPosition: event.position,
                localPosition: event.localPosition,
              ),
            ),
          );
        }
      }
    }
    stopTrackingIfPointerNoLongerDown(event);
  }

  @override
  void acceptGesture(int pointer) {
    if (_state == _ForceState.possible) {
      _state = _ForceState.accepted;
    }

    if (onStart != null && _state == _ForceState.started) {
      invokeCallback<void>(
        'onStart',
        () => onStart!(
          ForcePressDetails(
            pressure: _lastPressure,
            globalPosition: _lastPosition.global,
            localPosition: _lastPosition.local,
          ),
        ),
      );
    }
  }

  @override
  void didStopTrackingLastPointer(int pointer) {
    final bool wasAccepted = _state == _ForceState.started || _state == _ForceState.peaked;
    if (_state == _ForceState.possible) {
      resolve(GestureDisposition.rejected);
      return;
    }
    if (wasAccepted && onEnd != null) {
      if (onEnd != null) {
        invokeCallback<void>(
          'onEnd',
          () => onEnd!(
            ForcePressDetails(
              pressure: 0.0,
              globalPosition: _lastPosition.global,
              localPosition: _lastPosition.local,
            ),
          ),
        );
      }
    }
    _state = _ForceState.ready;
  }

  @override
  void rejectGesture(int pointer) {
    stopTrackingPointer(pointer);
    didStopTrackingLastPointer(pointer);
  }

  static double _inverseLerp(double min, double max, double t) {
    assert(min <= max);
    double value = (t - min) / (max - min);

    // If the device incorrectly reports a pressure outside of pressureMin
    // and pressureMax, we still want this recognizer to respond normally.
    if (!value.isNaN) {
      value = clampDouble(value, 0.0, 1.0);
    }
    return value;
  }

  @override
  String get debugDescription => 'force press';
}
