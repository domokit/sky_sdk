// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'arena.dart';
library;

import 'dart:async';

import 'package:flutter/foundation.dart';

import 'constants.dart';
import 'events.dart';
import 'monodrag.dart';
import 'recognizer.dart';
import 'scale.dart';
import 'tap.dart';

// Examples can assume:
// void setState(VoidCallback fn) { }
// late String _last;

double _getGlobalDistance(PointerEvent event, OffsetPair? originPosition) {
  assert(originPosition != null);
  final Offset offset = event.position - originPosition!.global;
  return offset.distance;
}

// The possible states of a [BaseTapAndDragGestureRecognizer].
//
// The recognizer advances from [ready] to [possible] when it starts tracking
// a pointer in [BaseTapAndDragGestureRecognizer.addAllowedPointer]. Where it advances
// from there depends on the sequence of pointer events that is tracked by the
// recognizer, following the initial [PointerDownEvent]:
//
// * If a [PointerUpEvent] has not been tracked, the recognizer stays in the [possible]
//   state as long as it continues to track a pointer.
// * If a [PointerMoveEvent] is tracked that has moved a sufficient global distance
//   from the initial [PointerDownEvent] and it came before a [PointerUpEvent], then
//   this recognizer moves from the [possible] state to [accepted].
// * If a [PointerUpEvent] is tracked before the pointer has moved a sufficient global
//   distance to be considered a drag, then this recognizer moves from the [possible]
//   state to [ready].
// * If a [PointerCancelEvent] is tracked then this recognizer moves from its current
//   state to [ready].
//
// Once the recognizer has stopped tracking any remaining pointers, the recognizer
// returns to the [ready] state.
enum _DragState {
  // The recognizer is ready to start recognizing a drag.
  ready,

  // The sequence of pointer events seen thus far is consistent with a drag but
  // it has not been accepted definitively.
  possible,

  // The sequence of pointer events has been accepted definitively as a drag.
  accepted,
}

/// {@macro flutter.gestures.tap.GestureTapDownCallback}
///
/// The consecutive tap count at the time the pointer contacted the
/// screen is given by [TapDragDownDetails.consecutiveTapCount].
///
/// Used by [BaseTapAndDragGestureRecognizer.onTapDown].
typedef GestureTapDragDownCallback = void Function(TapDragDownDetails details);

/// Details for [GestureTapDragDownCallback], such as the number of
/// consecutive taps.
///
/// See also:
///
///  * [BaseTapAndDragGestureRecognizer], which passes this information to its
///    [BaseTapAndDragGestureRecognizer.onTapDown] callback.
///  * [TapDragUpDetails], the details for [GestureTapDragUpCallback].
///  * [TapDragStartDetails], the details for [GestureTapDragStartCallback].
///  * [TapDragUpdateDetails], the details for [GestureTapDragUpdateCallback].
///  * [TapDragEndDetails], the details for [GestureTapDragEndCallback].
class TapDragDownDetails with Diagnosticable {
  /// Creates details for a [GestureTapDragDownCallback].
  TapDragDownDetails({
    required this.globalPosition,
    required this.localPosition,
    this.kind,
    required this.consecutiveTapCount,
  });

  /// The global position at which the pointer contacted the screen.
  final Offset globalPosition;

  /// The local position at which the pointer contacted the screen.
  final Offset localPosition;

  /// The kind of the device that initiated the event.
  final PointerDeviceKind? kind;

  /// If this tap is in a series of taps, then this value represents
  /// the number in the series this tap is.
  final int consecutiveTapCount;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<Offset>('globalPosition', globalPosition));
    properties.add(DiagnosticsProperty<Offset>('localPosition', localPosition));
    properties.add(DiagnosticsProperty<PointerDeviceKind?>('kind', kind));
    properties.add(DiagnosticsProperty<int>('consecutiveTapCount', consecutiveTapCount));
  }
}

/// {@macro flutter.gestures.tap.GestureTapUpCallback}
///
/// The consecutive tap count at the time the pointer contacted the
/// screen is given by [TapDragUpDetails.consecutiveTapCount].
///
/// Used by [BaseTapAndDragGestureRecognizer.onTapUp].
typedef GestureTapDragUpCallback = void Function(TapDragUpDetails details);

/// Details for [GestureTapDragUpCallback], such as the number of
/// consecutive taps.
///
/// See also:
///
///  * [BaseTapAndDragGestureRecognizer], which passes this information to its
///    [BaseTapAndDragGestureRecognizer.onTapUp] callback.
///  * [TapDragDownDetails], the details for [GestureTapDragDownCallback].
///  * [TapDragStartDetails], the details for [GestureTapDragStartCallback].
///  * [TapDragUpdateDetails], the details for [GestureTapDragUpdateCallback].
///  * [TapDragEndDetails], the details for [GestureTapDragEndCallback].
class TapDragUpDetails with Diagnosticable {
  /// Creates details for a [GestureTapDragUpCallback].
  TapDragUpDetails({
    required this.kind,
    required this.globalPosition,
    required this.localPosition,
    required this.consecutiveTapCount,
  });

  /// The global position at which the pointer contacted the screen.
  final Offset globalPosition;

  /// The local position at which the pointer contacted the screen.
  final Offset localPosition;

  /// The kind of the device that initiated the event.
  final PointerDeviceKind kind;

  /// If this tap is in a series of taps, then this value represents
  /// the number in the series this tap is.
  final int consecutiveTapCount;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<Offset>('globalPosition', globalPosition));
    properties.add(DiagnosticsProperty<Offset>('localPosition', localPosition));
    properties.add(DiagnosticsProperty<PointerDeviceKind?>('kind', kind));
    properties.add(DiagnosticsProperty<int>('consecutiveTapCount', consecutiveTapCount));
  }
}

/// {@macro flutter.gestures.dragdetails.GestureDragStartCallback}
///
/// The consecutive tap count at the time the pointer contacted the
/// screen is given by [TapDragStartDetails.consecutiveTapCount].
///
/// Used by [BaseTapAndDragGestureRecognizer.onDragStart].
typedef GestureTapDragStartCallback = void Function(TapDragStartDetails details);

/// Details for [GestureTapDragStartCallback], such as the number of
/// consecutive taps.
///
/// See also:
///
///  * [BaseTapAndDragGestureRecognizer], which passes this information to its
///    [BaseTapAndDragGestureRecognizer.onDragStart] callback.
///  * [TapDragDownDetails], the details for [GestureTapDragDownCallback].
///  * [TapDragUpDetails], the details for [GestureTapDragUpCallback].
///  * [TapDragUpdateDetails], the details for [GestureTapDragUpdateCallback].
///  * [TapDragEndDetails], the details for [GestureTapDragEndCallback].
class TapDragStartDetails with Diagnosticable {
  /// Creates details for a [GestureTapDragStartCallback].
  TapDragStartDetails({
    this.sourceTimeStamp,
    required this.globalPosition,
    required this.localPosition,
    this.kind,
    required this.consecutiveTapCount,
  });

  /// Recorded timestamp of the source pointer event that triggered the drag
  /// event.
  ///
  /// Could be null if triggered from proxied events such as accessibility.
  final Duration? sourceTimeStamp;

  /// The global position at which the pointer contacted the screen.
  ///
  /// See also:
  ///
  ///  * [localPosition], which is the [globalPosition] transformed to the
  ///    coordinate space of the event receiver.
  final Offset globalPosition;

  /// The local position in the coordinate system of the event receiver at
  /// which the pointer contacted the screen.
  final Offset localPosition;

  /// The kind of the device that initiated the event.
  final PointerDeviceKind? kind;

  /// If this tap is in a series of taps, then this value represents
  /// the number in the series this tap is.
  final int consecutiveTapCount;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<Duration?>('sourceTimeStamp', sourceTimeStamp));
    properties.add(DiagnosticsProperty<Offset>('globalPosition', globalPosition));
    properties.add(DiagnosticsProperty<Offset>('localPosition', localPosition));
    properties.add(DiagnosticsProperty<PointerDeviceKind?>('kind', kind));
    properties.add(DiagnosticsProperty<int>('consecutiveTapCount', consecutiveTapCount));
  }
}

/// {@macro flutter.gestures.dragdetails.GestureDragUpdateCallback}
///
/// The consecutive tap count at the time the pointer contacted the
/// screen is given by [TapDragUpdateDetails.consecutiveTapCount].
///
/// Used by [BaseTapAndDragGestureRecognizer.onDragUpdate].
typedef GestureTapDragUpdateCallback = void Function(TapDragUpdateDetails details);

/// Details for [GestureTapDragUpdateCallback], such as the number of
/// consecutive taps.
///
/// See also:
///
///  * [BaseTapAndDragGestureRecognizer], which passes this information to its
///    [BaseTapAndDragGestureRecognizer.onDragUpdate] callback.
///  * [TapDragDownDetails], the details for [GestureTapDragDownCallback].
///  * [TapDragUpDetails], the details for [GestureTapDragUpCallback].
///  * [TapDragStartDetails], the details for [GestureTapDragStartCallback].
///  * [TapDragEndDetails], the details for [GestureTapDragEndCallback].
class TapDragUpdateDetails with Diagnosticable {
  /// Creates details for a [GestureTapDragUpdateCallback].
  ///
  /// If [primaryDelta] is non-null, then its value must match one of the
  /// coordinates of [delta] and the other coordinate must be zero.
  TapDragUpdateDetails({
    this.sourceTimeStamp,
    this.delta = Offset.zero,
    this.primaryDelta,
    required this.globalPosition,
    this.kind,
    required this.localPosition,
    required this.offsetFromOrigin,
    required this.localOffsetFromOrigin,
    required this.consecutiveTapCount,
  }) : assert(
         primaryDelta == null ||
             (primaryDelta == delta.dx && delta.dy == 0.0) ||
             (primaryDelta == delta.dy && delta.dx == 0.0),
       );

  /// Recorded timestamp of the source pointer event that triggered the drag
  /// event.
  ///
  /// Could be null if triggered from proxied events such as accessibility.
  final Duration? sourceTimeStamp;

  /// The amount the pointer has moved in the coordinate space of the event
  /// receiver since the previous update.
  ///
  /// If the [GestureTapDragUpdateCallback] is for a one-dimensional drag (e.g.,
  /// a horizontal or vertical drag), then this offset contains only the delta
  /// in that direction (i.e., the coordinate in the other direction is zero).
  ///
  /// Defaults to zero if not specified in the constructor.
  final Offset delta;

  /// The amount the pointer has moved along the primary axis in the coordinate
  /// space of the event receiver since the previous
  /// update.
  ///
  /// If the [GestureTapDragUpdateCallback] is for a one-dimensional drag (e.g.,
  /// a horizontal or vertical drag), then this value contains the component of
  /// [delta] along the primary axis (e.g., horizontal or vertical,
  /// respectively). Otherwise, if the [GestureTapDragUpdateCallback] is for a
  /// two-dimensional drag (e.g., a pan), then this value is null.
  ///
  /// Defaults to null if not specified in the constructor.
  final double? primaryDelta;

  /// The pointer's global position when it triggered this update.
  ///
  /// See also:
  ///
  ///  * [localPosition], which is the [globalPosition] transformed to the
  ///    coordinate space of the event receiver.
  final Offset globalPosition;

  /// The local position in the coordinate system of the event receiver at
  /// which the pointer contacted the screen.
  ///
  /// Defaults to [globalPosition] if not specified in the constructor.
  final Offset localPosition;

  /// The kind of the device that initiated the event.
  final PointerDeviceKind? kind;

  /// A delta offset from the point where the drag initially contacted
  /// the screen to the point where the pointer is currently located in global
  /// coordinates (the present [globalPosition]) when this callback is triggered.
  ///
  /// When considering a [GestureRecognizer] that tracks the number of consecutive taps,
  /// this offset is associated with the most recent [PointerDownEvent] that occurred.
  final Offset offsetFromOrigin;

  /// A local delta offset from the point where the drag initially contacted
  /// the screen to the point where the pointer is currently located in local
  /// coordinates (the present [localPosition]) when this callback is triggered.
  ///
  /// When considering a [GestureRecognizer] that tracks the number of consecutive taps,
  /// this offset is associated with the most recent [PointerDownEvent] that occurred.
  final Offset localOffsetFromOrigin;

  /// If this tap is in a series of taps, then this value represents
  /// the number in the series this tap is.
  final int consecutiveTapCount;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<Duration?>('sourceTimeStamp', sourceTimeStamp));
    properties.add(DiagnosticsProperty<Offset>('delta', delta));
    properties.add(DiagnosticsProperty<double?>('primaryDelta', primaryDelta));
    properties.add(DiagnosticsProperty<Offset>('globalPosition', globalPosition));
    properties.add(DiagnosticsProperty<Offset>('localPosition', localPosition));
    properties.add(DiagnosticsProperty<PointerDeviceKind?>('kind', kind));
    properties.add(DiagnosticsProperty<Offset>('offsetFromOrigin', offsetFromOrigin));
    properties.add(DiagnosticsProperty<Offset>('localOffsetFromOrigin', localOffsetFromOrigin));
    properties.add(DiagnosticsProperty<int>('consecutiveTapCount', consecutiveTapCount));
  }
}

/// {@macro flutter.gestures.monodrag.GestureDragEndCallback}
///
/// The consecutive tap count at the time the pointer contacted the
/// screen is given by [TapDragEndDetails.consecutiveTapCount].
///
/// Used by [BaseTapAndDragGestureRecognizer.onDragEnd].
typedef GestureTapDragEndCallback = void Function(TapDragEndDetails endDetails);

/// Details for [GestureTapDragEndCallback], such as the number of
/// consecutive taps.
///
/// See also:
///
///  * [BaseTapAndDragGestureRecognizer], which passes this information to its
///    [BaseTapAndDragGestureRecognizer.onDragEnd] callback.
///  * [TapDragDownDetails], the details for [GestureTapDragDownCallback].
///  * [TapDragUpDetails], the details for [GestureTapDragUpCallback].
///  * [TapDragStartDetails], the details for [GestureTapDragStartCallback].
///  * [TapDragUpdateDetails], the details for [GestureTapDragUpdateCallback].
class TapDragEndDetails with Diagnosticable {
  /// Creates details for a [GestureTapDragEndCallback].
  TapDragEndDetails({
    this.velocity = Velocity.zero,
    this.primaryVelocity,
    required this.consecutiveTapCount,
  }) : assert(
         primaryVelocity == null ||
             primaryVelocity == velocity.pixelsPerSecond.dx ||
             primaryVelocity == velocity.pixelsPerSecond.dy,
       );

  /// The velocity the pointer was moving when it stopped contacting the screen.
  ///
  /// Defaults to zero if not specified in the constructor.
  final Velocity velocity;

  /// The velocity the pointer was moving along the primary axis when it stopped
  /// contacting the screen, in logical pixels per second.
  ///
  /// If the [GestureTapDragEndCallback] is for a one-dimensional drag (e.g., a
  /// horizontal or vertical drag), then this value contains the component of
  /// [velocity] along the primary axis (e.g., horizontal or vertical,
  /// respectively). Otherwise, if the [GestureTapDragEndCallback] is for a
  /// two-dimensional drag (e.g., a pan), then this value is null.
  ///
  /// Defaults to null if not specified in the constructor.
  final double? primaryVelocity;

  /// If this tap is in a series of taps, then this value represents
  /// the number in the series this tap is.
  final int consecutiveTapCount;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<Velocity>('velocity', velocity));
    properties.add(DiagnosticsProperty<double?>('primaryVelocity', primaryVelocity));
    properties.add(DiagnosticsProperty<int>('consecutiveTapCount', consecutiveTapCount));
  }
}

/// Signature for when the pointer that previously triggered a
/// [GestureTapDragDownCallback] did not complete.
///
/// Used by [BaseTapAndDragGestureRecognizer.onCancel].
typedef GestureCancelCallback = void Function();

// A mixin for [OneSequenceGestureRecognizer] that tracks the number of taps
// that occur in a series of [PointerEvent]s and the most recent set of
// [LogicalKeyboardKey]s pressed on the most recent tap down.
//
// A tap is tracked as part of a series of taps if:
//
// 1. The elapsed time between when a [PointerUpEvent] and the subsequent
// [PointerDownEvent] does not exceed [kDoubleTapTimeout].
// 2. The delta between the position tapped in the global coordinate system
// and the position that was tapped previously must be less than or equal
// to [kDoubleTapSlop].
//
// This mixin's state, i.e. the series of taps being tracked is reset when
// a tap is tracked that does not meet any of the specifications stated above.
mixin _TapStatusTrackerMixin on OneSequenceGestureRecognizer {
  // Public state available to [OneSequenceGestureRecognizer].

  // The [PointerDownEvent] that was most recently tracked in [addAllowedPointer].
  //
  // This value will be null if a [PointerDownEvent] has not been tracked yet in
  // [addAllowedPointer] or the timer between two taps has elapsed.
  //
  // This value is only reset when the timer between a [PointerUpEvent] and the
  // [PointerDownEvent] times out or when a new [PointerDownEvent] is tracked in
  // [addAllowedPointer].
  PointerDownEvent? get currentDown => _down;

  // The [PointerUpEvent] that was most recently tracked in [handleEvent].
  //
  // This value will be null if a [PointerUpEvent] has not been tracked yet in
  // [handleEvent] or the timer between two taps has elapsed.
  //
  // This value is only reset when the timer between a [PointerUpEvent] and the
  // [PointerDownEvent] times out or when a new [PointerDownEvent] is tracked in
  // [addAllowedPointer].
  PointerUpEvent? get currentUp => _up;

  // The number of consecutive taps that the most recently tracked [PointerDownEvent]
  // in [currentDown] represents.
  //
  // This value defaults to zero, meaning a tap series is not currently being tracked.
  //
  // When this value is greater than zero it means [addAllowedPointer] has run
  // and at least one [PointerDownEvent] belongs to the current series of taps
  // being tracked.
  //
  // [addAllowedPointer] will either increment this value by `1` or set the value to `1`
  // depending if the new [PointerDownEvent] is determined to be in the same series as the
  // tap that preceded it. If too much time has elapsed between two taps, the recognizer has lost
  // in the arena, the gesture has been cancelled, or the recognizer is being disposed then
  // this value will be set to `0`, and a new series will begin.
  int get consecutiveTapCount => _consecutiveTapCount;

  // The upper limit for the [consecutiveTapCount]. When this limit is reached
  // all tap related state is reset and a new tap series is tracked.
  //
  // If this value is null, [consecutiveTapCount] can grow infinitely large.
  int? get maxConsecutiveTap;

  // Private tap state tracked.
  PointerDownEvent? _down;
  PointerUpEvent? _up;
  int _consecutiveTapCount = 0;

  OffsetPair? _originPosition;
  int? _previousButtons;

  // For timing taps.
  Timer? _consecutiveTapTimer;
  Offset? _lastTapOffset;

  /// {@macro flutter.gestures.selectionrecognizers.TextSelectionGestureDetector.onTapTrackStart}
  VoidCallback? onTapTrackStart;

  /// {@macro flutter.gestures.selectionrecognizers.TextSelectionGestureDetector.onTapTrackReset}
  VoidCallback? onTapTrackReset;

  // When tracking a tap, the [consecutiveTapCount] is incremented if the given tap
  // falls under the tolerance specifications and reset to 1 if not.
  @override
  void addAllowedPointer(PointerDownEvent event) {
    super.addAllowedPointer(event);
    if (_consecutiveTapTimer != null && !_consecutiveTapTimer!.isActive) {
      _tapTrackerReset();
    }
    if (maxConsecutiveTap == _consecutiveTapCount) {
      _tapTrackerReset();
    }
    _up = null;
    if (_down != null && !_representsSameSeries(event)) {
      // The given tap does not match the specifications of the series of taps being tracked,
      // reset the tap count and related state.
      _consecutiveTapCount = 1;
    } else {
      _consecutiveTapCount += 1;
    }
    _consecutiveTapTimerStop();
    // `_down` must be assigned in this method instead of [handleEvent],
    // because [acceptGesture] might be called before [handleEvent],
    // which may rely on `_down` to initiate a callback.
    _trackTap(event);
  }

  @override
  void handleEvent(PointerEvent event) {
    if (event is PointerMoveEvent) {
      final double computedSlop = computeHitSlop(event.kind, gestureSettings);
      final bool isSlopPastTolerance = _getGlobalDistance(event, _originPosition) > computedSlop;

      if (isSlopPastTolerance) {
        _consecutiveTapTimerStop();
        _previousButtons = null;
        _lastTapOffset = null;
      }
    } else if (event is PointerUpEvent) {
      _up = event;
      if (_down != null) {
        _consecutiveTapTimerStop();
        _consecutiveTapTimerStart();
      }
    } else if (event is PointerCancelEvent) {
      _tapTrackerReset();
    }
  }

  @override
  void rejectGesture(int pointer) {
    _tapTrackerReset();
  }

  @override
  void dispose() {
    _tapTrackerReset();
    super.dispose();
  }

  void _trackTap(PointerDownEvent event) {
    _down = event;
    _previousButtons = event.buttons;
    _lastTapOffset = event.position;
    _originPosition = OffsetPair(local: event.localPosition, global: event.position);
    onTapTrackStart?.call();
  }

  bool _hasSameButton(int buttons) {
    assert(_previousButtons != null);
    if (buttons == _previousButtons!) {
      return true;
    } else {
      return false;
    }
  }

  bool _isWithinConsecutiveTapTolerance(Offset secondTapOffset) {
    if (_lastTapOffset == null) {
      return false;
    }

    final Offset difference = secondTapOffset - _lastTapOffset!;
    return difference.distance <= kDoubleTapSlop;
  }

  bool _representsSameSeries(PointerDownEvent event) {
    return _consecutiveTapTimer != null &&
        _isWithinConsecutiveTapTolerance(event.position) &&
        _hasSameButton(event.buttons);
  }

  void _consecutiveTapTimerStart() {
    _consecutiveTapTimer ??= Timer(kDoubleTapTimeout, _consecutiveTapTimerTimeout);
  }

  void _consecutiveTapTimerStop() {
    if (_consecutiveTapTimer != null) {
      _consecutiveTapTimer!.cancel();
      _consecutiveTapTimer = null;
    }
  }

  void _consecutiveTapTimerTimeout() {
    // The consecutive tap timer may time out before a tap down/tap up event is
    // fired. In this case we should not reset the tap tracker state immediately.
    // Instead we should reset the tap tracker on the next call to [addAllowedPointer],
    // if the timer is no longer active.
  }

  void _tapTrackerReset() {
    // The timer has timed out, i.e. the time between a [PointerUpEvent] and the subsequent
    // [PointerDownEvent] exceeded the duration of [kDoubleTapTimeout], so the tap belonging
    // to the [PointerDownEvent] cannot be considered part of the same tap series as the
    // previous [PointerUpEvent].
    _consecutiveTapTimerStop();
    _previousButtons = null;
    _originPosition = null;
    _lastTapOffset = null;
    _consecutiveTapCount = 0;
    _down = null;
    _up = null;
    onTapTrackReset?.call();
  }
}

/// A base class for gesture recognizers that recognize taps and movements.
///
/// Takes on the responsibilities of [TapGestureRecognizer] and
/// [DragGestureRecognizer] in one [GestureRecognizer].
///
/// ### Gesture arena behavior
///
/// [BaseTapAndDragGestureRecognizer] competes on the pointer events of
/// [kPrimaryButton] only when it has at least one non-null `onTap*`
/// or `onDrag*` callback.
///
/// It will declare defeat if it determines that a gesture is not a
/// tap (e.g. if the pointer is dragged too far while it's contacting the
/// screen) or a drag (e.g. if the pointer was not dragged far enough to
/// be considered a drag.
///
/// This recognizer will not immediately declare victory for every tap that it
/// recognizes, but it declares victory for every drag.
///
/// The recognizer will declare victory when all other recognizer's in
/// the arena have lost, if the timer of [kPressTimeout] elapses and a tap
/// series greater than 1 is being tracked, or until the pointer has moved
/// a sufficient global distance from the origin to be considered a drag.
///
/// If this recognizer loses the arena (either by declaring defeat or by
/// another recognizer declaring victory) while the pointer is contacting the
/// screen, it will fire [onCancel] instead of [onTapUp] or [onDragEnd].
///
/// ### When competing with `TapGestureRecognizer` and `DragGestureRecognizer`
///
/// Similar to [TapGestureRecognizer] and [DragGestureRecognizer],
/// [BaseTapAndDragGestureRecognizer] will not aggressively declare victory when
/// it detects a tap, so when it is competing with those gesture recognizers and
/// others it has a chance of losing. Similarly, when `eagerVictoryOnDrag` is set
/// to `false`, this recognizer will not aggressively declare victory when it
/// detects a drag. By default, `eagerVictoryOnDrag` is set to `true`, so this
/// recognizer will aggressively declare victory when it detects a drag.
///
/// When competing against [TapGestureRecognizer], if the pointer does not move past the tap
/// tolerance, then the recognizer that entered the arena first will win. In this case the
/// gesture detected is a tap. If the pointer does travel past the tap tolerance then this
/// recognizer will be declared winner by default. The gesture detected in this case is a drag.
///
/// When competing against [DragGestureRecognizer], if the pointer does not move a sufficient
/// global distance to be considered a drag, the recognizers will tie in the arena. If the
/// pointer does travel enough distance then the recognizer that entered the arena
/// first will win. The gesture detected in this case is a drag.
///
/// {@tool dartpad}
/// This example shows how to use the [TapAndPanGestureRecognizer] along with a
/// [RawGestureDetector] to scale a Widget.
///
/// ** See code in examples/api/lib/gestures/tap_and_drag/tap_and_drag.0.dart **
/// {@end-tool}
///
/// {@tool snippet}
///
/// This example shows how to hook up [TapAndPanGestureRecognizer]s' to nested
/// [RawGestureDetector]s'. It assumes that the code is being used inside a [State]
/// object with a `_last` field that is then displayed as the child of the gesture detector.
///
/// In this example, if the pointer has moved past the drag threshold, then the
/// the first [TapAndPanGestureRecognizer] instance to receive the [PointerEvent]
/// will win the arena because the recognizer will immediately declare victory.
///
/// The first one to receive the event in the example will depend on where on both
/// containers the pointer lands first. If your pointer begins in the overlapping
/// area of both containers, then the inner-most widget will receive the event first.
/// If your pointer begins in the yellow container then it will be the first to
/// receive the event.
///
/// If the pointer has not moved past the drag threshold, then the first recognizer
/// to enter the arena will win (i.e. they both tie and the gesture arena will call
/// [GestureArenaManager.sweep] so the first member of the arena will win).
///
/// ```dart
/// RawGestureDetector(
///   gestures: <Type, GestureRecognizerFactory>{
///     TapAndPanGestureRecognizer: GestureRecognizerFactoryWithHandlers<TapAndPanGestureRecognizer>(
///       () => TapAndPanGestureRecognizer(),
///       (TapAndPanGestureRecognizer instance) {
///         instance
///           ..onTapDown = (TapDragDownDetails details) { setState(() { _last = 'down_a'; }); }
///           ..onDragStart = (TapDragStartDetails details) { setState(() { _last = 'drag_start_a'; }); }
///           ..onDragUpdate = (TapDragUpdateDetails details) { setState(() { _last = 'drag_update_a'; }); }
///           ..onDragEnd = (TapDragEndDetails details) { setState(() { _last = 'drag_end_a'; }); }
///           ..onTapUp = (TapDragUpDetails details) { setState(() { _last = 'up_a'; }); }
///           ..onCancel = () { setState(() { _last = 'cancel_a'; }); };
///       },
///     ),
///   },
///   child: Container(
///     width: 300.0,
///     height: 300.0,
///     color: Colors.yellow,
///     alignment: Alignment.center,
///     child: RawGestureDetector(
///       gestures: <Type, GestureRecognizerFactory>{
///         TapAndPanGestureRecognizer: GestureRecognizerFactoryWithHandlers<TapAndPanGestureRecognizer>(
///           () => TapAndPanGestureRecognizer(),
///           (TapAndPanGestureRecognizer instance) {
///             instance
///               ..onTapDown = (TapDragDownDetails details) { setState(() { _last = 'down_b'; }); }
///               ..onDragStart = (TapDragStartDetails details) { setState(() { _last = 'drag_start_b'; }); }
///               ..onDragUpdate = (TapDragUpdateDetails details) { setState(() { _last = 'drag_update_b'; }); }
///               ..onDragEnd = (TapDragEndDetails details) { setState(() { _last = 'drag_end_b'; }); }
///               ..onTapUp = (TapDragUpDetails details) { setState(() { _last = 'up_b'; }); }
///               ..onCancel = () { setState(() { _last = 'cancel_b'; }); };
///           },
///         ),
///       },
///       child: Container(
///         width: 150.0,
///         height: 150.0,
///         color: Colors.blue,
///         child: Text(_last),
///       ),
///     ),
///   ),
/// )
/// ```
/// {@end-tool}
sealed class BaseTapAndDragGestureRecognizer extends OneSequenceGestureRecognizer
    with _TapStatusTrackerMixin {
  /// Creates a tap and drag gesture recognizer.
  ///
  /// {@macro flutter.gestures.GestureRecognizer.supportedDevices}
  BaseTapAndDragGestureRecognizer({
    super.debugOwner,
    super.supportedDevices,
    super.allowedButtonsFilter,
    this.eagerVictoryOnDrag = true,
  }) : _deadline = kPressTimeout,
       dragStartBehavior = DragStartBehavior.start;

  /// Configure the behavior of offsets passed to [onDragStart].
  ///
  /// If set to [DragStartBehavior.start], the [onDragStart] callback will be called
  /// with the position of the pointer at the time this gesture recognizer won
  /// the arena. If [DragStartBehavior.down], [onDragStart] will be called with
  /// the position of the first detected down event for the pointer. When there
  /// are no other gestures competing with this gesture in the arena, there's
  /// no difference in behavior between the two settings.
  ///
  /// For more information about the gesture arena:
  /// https://flutter.dev/to/gesture-disambiguation
  ///
  /// By default, the drag start behavior is [DragStartBehavior.start].
  ///
  /// See also:
  ///
  ///  * [DragGestureRecognizer.dragStartBehavior], which includes more details and an example.
  DragStartBehavior dragStartBehavior;

  /// The frequency at which the [onDragUpdate] callback is called.
  ///
  /// The value defaults to null, meaning there is no delay for [onDragUpdate] callback.
  Duration? dragUpdateThrottleFrequency;

  /// An upper bound for the amount of taps that can belong to one tap series.
  ///
  /// When this limit is reached the series of taps being tracked by this
  /// recognizer will be reset.
  @override
  int? maxConsecutiveTap;

  /// Whether this recognizer eagerly declares victory when it has detected
  /// a drag.
  ///
  /// When this value is `false`, this recognizer will wait until it is the last
  /// recognizer in the gesture arena before declaring victory on a drag.
  ///
  /// Defaults to `true`.
  bool eagerVictoryOnDrag;

  /// {@macro flutter.gestures.tap.TapGestureRecognizer.onTapDown}
  ///
  /// This triggers after the down event, once a short timeout ([kPressTimeout]) has
  /// elapsed, or once the gestures has won the arena, whichever comes first.
  ///
  /// The position of the pointer is provided in the callback's `details`
  /// argument, which is a [TapDragDownDetails] object.
  ///
  /// {@template flutter.gestures.selectionrecognizers.BaseTapAndDragGestureRecognizer.tapStatusTrackerData}
  /// The number of consecutive taps, and the keys that were pressed on tap down
  /// are also provided in the callback's `details` argument.
  /// {@endtemplate}
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  ///  * [TapDragDownDetails], which is passed as an argument to this callback.
  GestureTapDragDownCallback? onTapDown;

  /// {@macro flutter.gestures.tap.TapGestureRecognizer.onTapUp}
  ///
  /// This triggers on the up event, if the recognizer wins the arena with it
  /// or has previously won.
  ///
  /// The position of the pointer is provided in the callback's `details`
  /// argument, which is a [TapDragUpDetails] object.
  ///
  /// {@macro flutter.gestures.selectionrecognizers.BaseTapAndDragGestureRecognizer.tapStatusTrackerData}
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  ///  * [TapDragUpDetails], which is passed as an argument to this callback.
  GestureTapDragUpCallback? onTapUp;

  /// {@macro flutter.gestures.monodrag.DragGestureRecognizer.onStart}
  ///
  /// The position of the pointer is provided in the callback's `details`
  /// argument, which is a [TapDragStartDetails] object. The [dragStartBehavior]
  /// determines this position.
  ///
  /// {@macro flutter.gestures.selectionrecognizers.BaseTapAndDragGestureRecognizer.tapStatusTrackerData}
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  ///  * [TapDragStartDetails], which is passed as an argument to this callback.
  GestureTapDragStartCallback? onDragStart;

  /// {@macro flutter.gestures.monodrag.DragGestureRecognizer.onUpdate}
  ///
  /// The distance traveled by the pointer since the last update is provided in
  /// the callback's `details` argument, which is a [TapDragUpdateDetails] object.
  ///
  /// {@macro flutter.gestures.selectionrecognizers.BaseTapAndDragGestureRecognizer.tapStatusTrackerData}
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  ///  * [TapDragUpdateDetails], which is passed as an argument to this callback.
  GestureTapDragUpdateCallback? onDragUpdate;

  /// {@macro flutter.gestures.monodrag.DragGestureRecognizer.onEnd}
  ///
  /// The velocity is provided in the callback's `details` argument, which is a
  /// [TapDragEndDetails] object.
  ///
  /// {@macro flutter.gestures.selectionrecognizers.BaseTapAndDragGestureRecognizer.tapStatusTrackerData}
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  ///  * [TapDragEndDetails], which is passed as an argument to this callback.
  GestureTapDragEndCallback? onDragEnd;

  /// The pointer that previously triggered [onTapDown] did not complete.
  ///
  /// This is called when a [PointerCancelEvent] is tracked when the [onTapDown] callback
  /// was previously called.
  ///
  /// It may also be called if a [PointerUpEvent] is tracked after the pointer has moved
  /// past the tap tolerance but not past the drag tolerance, and the recognizer has not
  /// yet won the arena.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  GestureCancelCallback? onCancel;

  // Tap related state.
  bool _pastSlopTolerance = false;
  bool _sentTapDown = false;
  bool _wonArenaForPrimaryPointer = false;

  // Primary pointer being tracked by this recognizer.
  int? _primaryPointer;
  Timer? _deadlineTimer;
  // The recognizer will call [onTapDown] after this amount of time has elapsed
  // since starting to track the primary pointer.
  //
  // [onTapDown] will not be called if the primary pointer is
  // accepted, rejected, or all pointers are up or canceled before [_deadline].
  final Duration _deadline;

  // Drag related state.
  _DragState _dragState = _DragState.ready;
  PointerEvent? _start;
  late OffsetPair _initialPosition;
  late double _globalDistanceMoved;
  late double _globalDistanceMovedAllAxes;
  OffsetPair? _correctedPosition;

  // For drag update throttle.
  TapDragUpdateDetails? _lastDragUpdateDetails;
  Timer? _dragUpdateThrottleTimer;

  final Set<int> _acceptedActivePointers = <int>{};

  Offset _getDeltaForDetails(Offset delta);
  double? _getPrimaryValueFromOffset(Offset value);
  bool _hasSufficientGlobalDistanceToAccept(PointerDeviceKind pointerDeviceKind);

  // Drag updates may require throttling to avoid excessive updating, such as for text layouts in text
  // fields. The frequency of invocations is controlled by the [dragUpdateThrottleFrequency].
  //
  // Once the drag gesture ends, any pending drag update will be fired
  // immediately. See [_checkDragEnd].
  void _handleDragUpdateThrottled() {
    assert(_lastDragUpdateDetails != null);
    if (onDragUpdate != null) {
      invokeCallback<void>('onDragUpdate', () => onDragUpdate!(_lastDragUpdateDetails!));
    }
    _dragUpdateThrottleTimer = null;
    _lastDragUpdateDetails = null;
  }

  @override
  bool isPointerAllowed(PointerEvent event) {
    if (_primaryPointer == null) {
      switch (event.buttons) {
        case kPrimaryButton:
          if (onTapDown == null &&
              onDragStart == null &&
              onDragUpdate == null &&
              onDragEnd == null &&
              onTapUp == null &&
              onCancel == null) {
            return false;
          }
        default:
          return false;
      }
    } else {
      if (event.pointer != _primaryPointer) {
        return false;
      }
    }

    return super.isPointerAllowed(event as PointerDownEvent);
  }

  @override
  void addAllowedPointer(PointerDownEvent event) {
    if (_dragState == _DragState.ready) {
      super.addAllowedPointer(event);
      _primaryPointer = event.pointer;
      _globalDistanceMoved = 0.0;
      _globalDistanceMovedAllAxes = 0.0;
      _dragState = _DragState.possible;
      _initialPosition = OffsetPair(global: event.position, local: event.localPosition);
      _deadlineTimer = Timer(_deadline, () => _didExceedDeadlineWithEvent(event));
    }
  }

  @override
  void handleNonAllowedPointer(PointerDownEvent event) {
    // There can be multiple drags simultaneously. Their effects are combined.
    if (event.buttons != kPrimaryButton) {
      if (!_wonArenaForPrimaryPointer) {
        super.handleNonAllowedPointer(event);
      }
    }
  }

  @override
  void acceptGesture(int pointer) {
    if (pointer != _primaryPointer) {
      return;
    }

    _stopDeadlineTimer();

    assert(!_acceptedActivePointers.contains(pointer));
    _acceptedActivePointers.add(pointer);

    // Called when this recognizer is accepted by the [GestureArena].
    if (currentDown != null) {
      _checkTapDown(currentDown!);
    }

    _wonArenaForPrimaryPointer = true;

    // resolve(GestureDisposition.accepted) will be called when the [PointerMoveEvent]
    // has moved a sufficient global distance to be considered a drag and
    // `eagerVictoryOnDrag` is set to `true`.
    if (_start != null && eagerVictoryOnDrag) {
      assert(_dragState == _DragState.accepted);
      assert(currentUp == null);
      _acceptDrag(_start!);
    }

    // This recognizer will wait until it is the last one in the gesture arena
    // before accepting a drag when `eagerVictoryOnDrag` is set to `false`.
    if (_start != null && !eagerVictoryOnDrag) {
      assert(_dragState == _DragState.possible);
      assert(currentUp == null);
      _dragState = _DragState.accepted;
      _acceptDrag(_start!);
    }

    if (currentUp != null) {
      _checkTapUp(currentUp!);
    }
  }

  @override
  void didStopTrackingLastPointer(int pointer) {
    switch (_dragState) {
      case _DragState.ready:
        _checkCancel();
        resolve(GestureDisposition.rejected);

      case _DragState.possible:
        if (_pastSlopTolerance) {
          // This means the pointer was not accepted as a tap.
          if (_wonArenaForPrimaryPointer) {
            // If the recognizer has already won the arena for the primary pointer being tracked
            // but the pointer has exceeded the tap tolerance, then the pointer is accepted as a
            // drag gesture.
            if (currentDown != null) {
              if (!_acceptedActivePointers.remove(pointer)) {
                resolvePointer(pointer, GestureDisposition.rejected);
              }
              _dragState = _DragState.accepted;
              _acceptDrag(currentDown!);
              _checkDragEnd();
            }
          } else {
            _checkCancel();
            resolve(GestureDisposition.rejected);
          }
        } else {
          // The pointer is accepted as a tap.
          if (currentUp != null) {
            _checkTapUp(currentUp!);
          }
        }

      case _DragState.accepted:
        // For the case when the pointer has been accepted as a drag.
        // Meaning [_checkTapDown] and [_checkDragStart] have already ran.
        _checkDragEnd();
    }

    _stopDeadlineTimer();
    _start = null;
    _dragState = _DragState.ready;
    _pastSlopTolerance = false;
  }

  @override
  void handleEvent(PointerEvent event) {
    if (event.pointer != _primaryPointer) {
      return;
    }
    super.handleEvent(event);
    if (event is PointerMoveEvent) {
      // Receiving a [PointerMoveEvent], does not automatically mean the pointer
      // being tracked is doing a drag gesture. There is some drift that can happen
      // between the initial [PointerDownEvent] and subsequent [PointerMoveEvent]s.
      // Accessing [_pastSlopTolerance] lets us know if our tap has moved past the
      // acceptable tolerance. If the pointer does not move past this tolerance than
      // it is not considered a drag.
      //
      // To be recognized as a drag, the [PointerMoveEvent] must also have moved
      // a sufficient global distance from the initial [PointerDownEvent] to be
      // accepted as a drag. This logic is handled in [_hasSufficientGlobalDistanceToAccept].
      //
      // The recognizer will also detect the gesture as a drag when the pointer
      // has been accepted and it has moved past the [slopTolerance] but has not moved
      // a sufficient global distance from the initial position to be considered a drag.
      // In this case since the gesture cannot be a tap, it defaults to a drag.
      final double computedSlop = computeHitSlop(event.kind, gestureSettings);
      _pastSlopTolerance =
          _pastSlopTolerance || _getGlobalDistance(event, _initialPosition) > computedSlop;

      if (_dragState == _DragState.accepted) {
        _checkDragUpdate(event);
      } else if (_dragState == _DragState.possible) {
        if (_start == null) {
          // Only check for a drag if the start of a drag was not already identified.
          _checkDrag(event);
        }

        // This can occur when the recognizer is accepted before a [PointerMoveEvent] has been
        // received that moves the pointer a sufficient global distance to be considered a drag.
        if (_start != null && _wonArenaForPrimaryPointer) {
          _dragState = _DragState.accepted;
          _acceptDrag(_start!);
        }
      }
    } else if (event is PointerUpEvent) {
      if (_dragState == _DragState.possible) {
        // The drag has not been accepted before a [PointerUpEvent], therefore the recognizer
        // attempts to recognize a tap.
        stopTrackingIfPointerNoLongerDown(event);
      } else if (_dragState == _DragState.accepted) {
        _giveUpPointer(event.pointer);
      }
    } else if (event is PointerCancelEvent) {
      _dragState = _DragState.ready;
      _giveUpPointer(event.pointer);
    }
  }

  @override
  void rejectGesture(int pointer) {
    if (pointer != _primaryPointer) {
      return;
    }
    super.rejectGesture(pointer);

    _stopDeadlineTimer();
    _giveUpPointer(pointer);
    _resetTaps();
    _resetDragUpdateThrottle();
  }

  @override
  void dispose() {
    _stopDeadlineTimer();
    _resetDragUpdateThrottle();
    super.dispose();
  }

  @override
  String get debugDescription => 'tap_and_drag';

  void _acceptDrag(PointerEvent event) {
    if (!_wonArenaForPrimaryPointer) {
      return;
    }
    if (dragStartBehavior == DragStartBehavior.start) {
      _initialPosition =
          _initialPosition + OffsetPair(global: event.delta, local: event.localDelta);
    }
    _checkDragStart(event);
    if (event.localDelta != Offset.zero) {
      final Matrix4? localToGlobal =
          event.transform != null ? Matrix4.tryInvert(event.transform!) : null;
      final Offset correctedLocalPosition = _initialPosition.local + event.localDelta;
      final Offset globalUpdateDelta = PointerEvent.transformDeltaViaPositions(
        untransformedEndPosition: correctedLocalPosition,
        untransformedDelta: event.localDelta,
        transform: localToGlobal,
      );
      final OffsetPair updateDelta = OffsetPair(local: event.localDelta, global: globalUpdateDelta);
      _correctedPosition = _initialPosition + updateDelta; // Only adds delta for down behaviour
      _checkDragUpdate(event);
      _correctedPosition = null;
    }
  }

  void _checkDrag(PointerMoveEvent event) {
    final Matrix4? localToGlobalTransform =
        event.transform == null ? null : Matrix4.tryInvert(event.transform!);
    final Offset movedLocally = _getDeltaForDetails(event.localDelta);
    _globalDistanceMoved +=
        PointerEvent.transformDeltaViaPositions(
          transform: localToGlobalTransform,
          untransformedDelta: movedLocally,
          untransformedEndPosition: event.localPosition,
        ).distance *
        (_getPrimaryValueFromOffset(movedLocally) ?? 1).sign;
    _globalDistanceMovedAllAxes +=
        PointerEvent.transformDeltaViaPositions(
          transform: localToGlobalTransform,
          untransformedDelta: event.localDelta,
          untransformedEndPosition: event.localPosition,
        ).distance *
        1.sign;
    if (_hasSufficientGlobalDistanceToAccept(event.kind) ||
        (_wonArenaForPrimaryPointer &&
            _globalDistanceMovedAllAxes.abs() > computePanSlop(event.kind, gestureSettings))) {
      _start = event;
      if (eagerVictoryOnDrag) {
        _dragState = _DragState.accepted;
        if (!_wonArenaForPrimaryPointer) {
          resolve(GestureDisposition.accepted);
        }
      }
    }
  }

  void _checkTapDown(PointerDownEvent event) {
    if (_sentTapDown) {
      return;
    }

    final TapDragDownDetails details = TapDragDownDetails(
      globalPosition: event.position,
      localPosition: event.localPosition,
      kind: getKindForPointer(event.pointer),
      consecutiveTapCount: consecutiveTapCount,
    );

    if (onTapDown != null) {
      invokeCallback('onTapDown', () => onTapDown!(details));
    }

    _sentTapDown = true;
  }

  void _checkTapUp(PointerUpEvent event) {
    if (!_wonArenaForPrimaryPointer) {
      return;
    }

    final TapDragUpDetails upDetails = TapDragUpDetails(
      kind: event.kind,
      globalPosition: event.position,
      localPosition: event.localPosition,
      consecutiveTapCount: consecutiveTapCount,
    );

    if (onTapUp != null) {
      invokeCallback('onTapUp', () => onTapUp!(upDetails));
    }

    _resetTaps();
    if (!_acceptedActivePointers.remove(event.pointer)) {
      resolvePointer(event.pointer, GestureDisposition.rejected);
    }
  }

  void _checkDragStart(PointerEvent event) {
    if (onDragStart != null) {
      final TapDragStartDetails details = TapDragStartDetails(
        sourceTimeStamp: event.timeStamp,
        globalPosition: _initialPosition.global,
        localPosition: _initialPosition.local,
        kind: getKindForPointer(event.pointer),
        consecutiveTapCount: consecutiveTapCount,
      );

      invokeCallback<void>('onDragStart', () => onDragStart!(details));
    }

    _start = null;
  }

  void _checkDragUpdate(PointerEvent event) {
    final Offset globalPosition =
        _correctedPosition != null ? _correctedPosition!.global : event.position;
    final Offset localPosition =
        _correctedPosition != null ? _correctedPosition!.local : event.localPosition;

    final TapDragUpdateDetails details = TapDragUpdateDetails(
      sourceTimeStamp: event.timeStamp,
      delta: event.localDelta,
      globalPosition: globalPosition,
      kind: getKindForPointer(event.pointer),
      localPosition: localPosition,
      offsetFromOrigin: globalPosition - _initialPosition.global,
      localOffsetFromOrigin: localPosition - _initialPosition.local,
      consecutiveTapCount: consecutiveTapCount,
    );

    if (dragUpdateThrottleFrequency != null) {
      _lastDragUpdateDetails = details;
      // Only schedule a new timer if there's not one pending.
      _dragUpdateThrottleTimer ??= Timer(dragUpdateThrottleFrequency!, _handleDragUpdateThrottled);
    } else {
      if (onDragUpdate != null) {
        invokeCallback<void>('onDragUpdate', () => onDragUpdate!(details));
      }
    }
  }

  void _checkDragEnd() {
    if (_dragUpdateThrottleTimer != null) {
      // If there's already an update scheduled, trigger it immediately and
      // cancel the timer.
      _dragUpdateThrottleTimer!.cancel();
      _handleDragUpdateThrottled();
    }

    final TapDragEndDetails endDetails = TapDragEndDetails(
      primaryVelocity: 0.0,
      consecutiveTapCount: consecutiveTapCount,
    );

    if (onDragEnd != null) {
      invokeCallback<void>('onDragEnd', () => onDragEnd!(endDetails));
    }

    _resetTaps();
    _resetDragUpdateThrottle();
  }

  void _checkCancel() {
    if (!_sentTapDown) {
      // Do not fire tap cancel if [onTapDown] was never called.
      return;
    }
    if (onCancel != null) {
      invokeCallback('onCancel', onCancel!);
    }
    _resetDragUpdateThrottle();
    _resetTaps();
  }

  void _didExceedDeadlineWithEvent(PointerDownEvent event) {
    _didExceedDeadline();
  }

  void _didExceedDeadline() {
    if (currentDown != null) {
      _checkTapDown(currentDown!);

      if (consecutiveTapCount > 1) {
        // If our consecutive tap count is greater than 1, i.e. is a double tap or greater,
        // then this recognizer declares victory to prevent the [LongPressGestureRecognizer]
        // from declaring itself the winner if a double tap is held for too long.
        resolve(GestureDisposition.accepted);
      }
    }
  }

  void _giveUpPointer(int pointer) {
    stopTrackingPointer(pointer);
    // If the pointer was never accepted, then it is rejected since this recognizer is no longer
    // interested in winning the gesture arena for it.
    if (!_acceptedActivePointers.remove(pointer)) {
      resolvePointer(pointer, GestureDisposition.rejected);
    }
  }

  void _resetTaps() {
    _sentTapDown = false;
    _wonArenaForPrimaryPointer = false;
    _primaryPointer = null;
  }

  void _resetDragUpdateThrottle() {
    if (dragUpdateThrottleFrequency == null) {
      return;
    }
    _lastDragUpdateDetails = null;
    if (_dragUpdateThrottleTimer != null) {
      _dragUpdateThrottleTimer!.cancel();
      _dragUpdateThrottleTimer = null;
    }
  }

  void _stopDeadlineTimer() {
    if (_deadlineTimer != null) {
      _deadlineTimer!.cancel();
      _deadlineTimer = null;
    }
  }
}

/// Recognizes taps along with movement in the horizontal direction.
///
/// Before this recognizer has won the arena for the primary pointer being tracked,
/// it will only accept a drag on the horizontal axis. If a drag is detected after
/// this recognizer has won the arena then it will accept a drag on any axis.
///
/// See also:
///
///  * [BaseTapAndDragGestureRecognizer], for the class that provides the main
///  implementation details of this recognizer.
///  * [TapAndPanGestureRecognizer], for a similar recognizer that accepts a drag
///  on any axis regardless if the recognizer has won the arena for the primary
///  pointer being tracked.
///  * [HorizontalDragGestureRecognizer], for a similar recognizer that only recognizes
///  horizontal movement.
class TapAndHorizontalDragGestureRecognizer extends BaseTapAndDragGestureRecognizer {
  /// Create a gesture recognizer for interactions in the horizontal axis.
  ///
  /// {@macro flutter.gestures.GestureRecognizer.supportedDevices}
  TapAndHorizontalDragGestureRecognizer({super.debugOwner, super.supportedDevices});

  @override
  bool _hasSufficientGlobalDistanceToAccept(PointerDeviceKind pointerDeviceKind) {
    return _globalDistanceMoved.abs() > computeHitSlop(pointerDeviceKind, gestureSettings);
  }

  @override
  Offset _getDeltaForDetails(Offset delta) => Offset(delta.dx, 0.0);

  @override
  double _getPrimaryValueFromOffset(Offset value) => value.dx;

  @override
  String get debugDescription => 'tap and horizontal drag';
}

/// {@template flutter.gestures.selectionrecognizers.TapAndPanGestureRecognizer}
/// Recognizes taps along with both horizontal and vertical movement.
///
/// This recognizer will accept a drag on any axis, regardless if it has won the
/// arena for the primary pointer being tracked.
///
/// See also:
///
///  * [BaseTapAndDragGestureRecognizer], for the class that provides the main
///  implementation details of this recognizer.
///  * [TapAndHorizontalDragGestureRecognizer], for a similar recognizer that
///  only accepts horizontal drags before it has won the arena for the primary
///  pointer being tracked.
///  * [PanGestureRecognizer], for a similar recognizer that only recognizes
///  movement.
/// {@endtemplate}
class TapAndPanGestureRecognizer extends BaseTapAndDragGestureRecognizer {
  /// Create a gesture recognizer for interactions on a plane.
  TapAndPanGestureRecognizer({super.debugOwner, super.supportedDevices});

  @override
  bool _hasSufficientGlobalDistanceToAccept(PointerDeviceKind pointerDeviceKind) {
    return _globalDistanceMoved.abs() > computePanSlop(pointerDeviceKind, gestureSettings);
  }

  @override
  Offset _getDeltaForDetails(Offset delta) => delta;

  @override
  double? _getPrimaryValueFromOffset(Offset value) => null;

  @override
  String get debugDescription => 'tap and pan';
}

@Deprecated(
  'Use TapAndPanGestureRecognizer instead. '
  'TapAndPanGestureRecognizer works exactly the same but has a more disambiguated name from BaseTapAndDragGestureRecognizer. '
  'This feature was deprecated after v3.9.0-19.0.pre.',
)
/// {@macro flutter.gestures.selectionrecognizers.TapAndPanGestureRecognizer}
class TapAndDragGestureRecognizer
    extends BaseTapAndDragGestureRecognizer {
  /// Create a gesture recognizer for interactions on a plane.
  @Deprecated(
    'Use TapAndPanGestureRecognizer instead. '
    'TapAndPanGestureRecognizer works exactly the same but has a more disambiguated name from BaseTapAndDragGestureRecognizer. '
    'This feature was deprecated after v3.9.0-19.0.pre.',
  )
  TapAndDragGestureRecognizer({super.debugOwner, super.supportedDevices});

  @override
  bool _hasSufficientGlobalDistanceToAccept(PointerDeviceKind pointerDeviceKind) {
    return _globalDistanceMoved.abs() > computePanSlop(pointerDeviceKind, gestureSettings);
  }

  @override
  Offset _getDeltaForDetails(Offset delta) => delta;

  @override
  double? _getPrimaryValueFromOffset(Offset value) => null;

  @override
  String get debugDescription => 'tap and pan';
}
