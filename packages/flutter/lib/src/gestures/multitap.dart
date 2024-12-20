// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
library;

import 'dart:async';

import 'arena.dart';
import 'binding.dart';
import 'constants.dart';
import 'events.dart';
import 'pointer_router.dart';
import 'recognizer.dart';
import 'tap.dart';

export 'dart:ui' show Offset, PointerDeviceKind;

export 'events.dart' show PointerDownEvent;
export 'tap.dart'
    show GestureTapCancelCallback, GestureTapDownCallback, TapDownDetails, TapUpDetails;

/// Signature for callback when the user has tapped the screen at the same
/// location twice in quick succession.
///
/// See also:
///
///  * [GestureDetector.onDoubleTap], which matches this signature.
typedef GestureDoubleTapCallback = void Function();

/// Signature used by [MultiTapGestureRecognizer] for when a pointer that might
/// cause a tap has contacted the screen at a particular location.
typedef GestureMultiTapDownCallback = void Function(int pointer, TapDownDetails details);

/// Signature used by [MultiTapGestureRecognizer] for when a pointer that will
/// trigger a tap has stopped contacting the screen at a particular location.
typedef GestureMultiTapUpCallback = void Function(int pointer, TapUpDetails details);

/// Signature used by [MultiTapGestureRecognizer] for when a tap has occurred.
typedef GestureMultiTapCallback = void Function(int pointer);

/// Signature for when the pointer that previously triggered a
/// [GestureMultiTapDownCallback] will not end up causing a tap.
typedef GestureMultiTapCancelCallback = void Function(int pointer);

/// CountdownZoned tracks whether the specified duration has elapsed since
/// creation, honoring [Zone].
class _CountdownZoned {
  _CountdownZoned({required Duration duration}) {
    Timer(duration, _onTimeout);
  }

  bool _timeout = false;

  bool get timeout => _timeout;

  void _onTimeout() {
    _timeout = true;
  }
}

/// TapTracker helps track individual tap sequences as part of a
/// larger gesture.
class _TapTracker {
  _TapTracker({
    required PointerDownEvent event,
    required this.entry,
    required Duration doubleTapMinTime,
    required this.gestureSettings,
  }) : pointer = event.pointer,
       _initialGlobalPosition = event.position,
       initialButtons = event.buttons,
       _doubleTapMinTimeCountdown = _CountdownZoned(duration: doubleTapMinTime);

  final DeviceGestureSettings? gestureSettings;
  final int pointer;
  final GestureArenaEntry entry;
  final Offset _initialGlobalPosition;
  final int initialButtons;
  final _CountdownZoned _doubleTapMinTimeCountdown;

  bool _isTrackingPointer = false;

  void startTrackingPointer(PointerRoute route, Matrix4? transform) {
    if (!_isTrackingPointer) {
      _isTrackingPointer = true;
      GestureBinding.instance.pointerRouter.addRoute(pointer, route, transform);
    }
  }

  void stopTrackingPointer(PointerRoute route) {
    if (_isTrackingPointer) {
      _isTrackingPointer = false;
      GestureBinding.instance.pointerRouter.removeRoute(pointer, route);
    }
  }

  bool isWithinGlobalTolerance(PointerEvent event, double tolerance) {
    final Offset offset = event.position - _initialGlobalPosition;
    return offset.distance <= tolerance;
  }

  bool hasElapsedMinTime() {
    return _doubleTapMinTimeCountdown.timeout;
  }

  bool hasSameButton(PointerDownEvent event) {
    return event.buttons == initialButtons;
  }
}

/// Recognizes when the user has tapped the screen at the same location twice in
/// quick succession.
///
/// [DoubleTapGestureRecognizer] competes on pointer events when it
/// has a non-null callback. If it has no callbacks, it is a no-op.
///
class DoubleTapGestureRecognizer extends GestureRecognizer {
  /// Create a gesture recognizer for double taps.
  ///
  /// {@macro flutter.gestures.GestureRecognizer.supportedDevices}
  DoubleTapGestureRecognizer({
    super.debugOwner,
    super.supportedDevices,
    super.allowedButtonsFilter = _defaultButtonAcceptBehavior,
  });

  // The default value for [allowedButtonsFilter].
  // Accept the input if, and only if, [kPrimaryButton] is pressed.
  static bool _defaultButtonAcceptBehavior(int buttons) => buttons == kPrimaryButton;

  // Implementation notes:
  //
  // The double tap recognizer can be in one of four states. There's no
  // explicit enum for the states, because they are already captured by
  // the state of existing fields. Specifically:
  //
  // 1. Waiting on first tap: In this state, the _trackers list is empty, and
  //    _firstTap is null.
  // 2. First tap in progress: In this state, the _trackers list contains all
  //    the states for taps that have begun but not completed. This list can
  //    have more than one entry if two pointers begin to tap.
  // 3. Waiting on second tap: In this state, one of the in-progress taps has
  //    completed successfully. The _trackers list is again empty, and
  //    _firstTap records the successful tap.
  // 4. Second tap in progress: Much like the "first tap in progress" state, but
  //    _firstTap is non-null. If a tap completes successfully while in this
  //    state, the callback is called and the state is reset.
  //
  // There are various other scenarios that cause the state to reset:
  //
  // - All in-progress taps are rejected (by time, distance, pointercancel, etc)
  // - The long timer between taps expires
  // - The gesture arena decides we have been rejected wholesale

  /// A pointer has contacted the screen with a primary button at the same
  /// location twice in quick succession, which might be the start of a double
  /// tap.
  ///
  /// This triggers immediately after the down event of the second tap.
  ///
  /// If this recognizer doesn't win the arena, [onDoubleTapCancel] is called
  /// next. Otherwise, [onDoubleTap] is called next.
  ///
  /// See also:
  ///
  ///  * [allowedButtonsFilter], which decides which button will be allowed.
  ///  * [TapDownDetails], which is passed as an argument to this callback.
  ///  * [GestureDetector.onDoubleTapDown], which exposes this callback.
  GestureTapDownCallback? onDoubleTapDown;

  /// Called when the user has tapped the screen with a primary button at the
  /// same location twice in quick succession.
  ///
  /// This triggers when the pointer stops contacting the device after the
  /// second tap.
  ///
  /// See also:
  ///
  ///  * [allowedButtonsFilter], which decides which button will be allowed.
  ///  * [GestureDetector.onDoubleTap], which exposes this callback.
  GestureDoubleTapCallback? onDoubleTap;

  /// A pointer that previously triggered [onDoubleTapDown] will not end up
  /// causing a double tap.
  ///
  /// This triggers once the gesture loses the arena if [onDoubleTapDown] has
  /// previously been triggered.
  ///
  /// If this recognizer wins the arena, [onDoubleTap] is called instead.
  ///
  /// See also:
  ///
  ///  * [allowedButtonsFilter], which decides which button will be allowed.
  ///  * [GestureDetector.onDoubleTapCancel], which exposes this callback.
  GestureTapCancelCallback? onDoubleTapCancel;

  Timer? _doubleTapTimer;
  _TapTracker? _firstTap;
  final Map<int, _TapTracker> _trackers = <int, _TapTracker>{};

  @override
  bool isPointerAllowed(PointerDownEvent event) {
    if (_firstTap == null) {
      if (onDoubleTapDown == null && onDoubleTap == null && onDoubleTapCancel == null) {
        return false;
      }
    }

    // If second tap is not allowed, reset the state.
    final bool isPointerAllowed = super.isPointerAllowed(event);
    if (!isPointerAllowed) {
      _reset();
    }
    return isPointerAllowed;
  }

  @override
  void addAllowedPointer(PointerDownEvent event) {
    if (_firstTap != null) {
      if (!_firstTap!.isWithinGlobalTolerance(event, kDoubleTapSlop)) {
        // Ignore out-of-bounds second taps.
        return;
      } else if (!_firstTap!.hasElapsedMinTime() || !_firstTap!.hasSameButton(event)) {
        // Restart when the second tap is too close to the first (touch screens
        // often detect touches intermittently), or when buttons mismatch.
        _reset();
        return _trackTap(event);
      } else if (onDoubleTapDown != null) {
        final TapDownDetails details = TapDownDetails(
          globalPosition: event.position,
          localPosition: event.localPosition,
          kind: getKindForPointer(event.pointer),
        );
        invokeCallback<void>('onDoubleTapDown', () => onDoubleTapDown!(details));
      }
    }
    _trackTap(event);
  }

  void _trackTap(PointerDownEvent event) {
    _stopDoubleTapTimer();
    final _TapTracker tracker = _TapTracker(
      event: event,
      entry: GestureBinding.instance.gestureArena.add(event.pointer, this),
      doubleTapMinTime: kDoubleTapMinTime,
      gestureSettings: gestureSettings,
    );
    _trackers[event.pointer] = tracker;
    tracker.startTrackingPointer(_handleEvent, event.transform);
  }

  void _handleEvent(PointerEvent event) {
    final _TapTracker tracker = _trackers[event.pointer]!;
    if (event is PointerUpEvent) {
      if (_firstTap == null) {
        _registerFirstTap(tracker);
      } else {
        _registerSecondTap(tracker);
      }
    } else if (event is PointerMoveEvent) {
      if (!tracker.isWithinGlobalTolerance(event, kDoubleTapTouchSlop)) {
        _reject(tracker);
      }
    } else if (event is PointerCancelEvent) {
      _reject(tracker);
    }
  }

  @override
  void acceptGesture(int pointer) {}

  @override
  void rejectGesture(int pointer) {
    _TapTracker? tracker = _trackers[pointer];
    // If tracker isn't in the list, check if this is the first tap tracker
    if (tracker == null && _firstTap != null && _firstTap!.pointer == pointer) {
      tracker = _firstTap;
    }
    // If tracker is still null, we rejected ourselves already
    if (tracker != null) {
      _reject(tracker);
    }
  }

  void _reject(_TapTracker tracker) {
    _trackers.remove(tracker.pointer);
    tracker.entry.resolve(GestureDisposition.rejected);
    _freezeTracker(tracker);
    if (_firstTap != null) {
      if (tracker == _firstTap) {
        _reset();
      } else {
        _checkCancel();
        if (_trackers.isEmpty) {
          _reset();
        }
      }
    }
  }

  @override
  void dispose() {
    _reset();
    super.dispose();
  }

  void _reset() {
    _stopDoubleTapTimer();
    if (_firstTap != null) {
      if (_trackers.isNotEmpty) {
        _checkCancel();
      }
      // Note, order is important below in order for the resolve -> reject logic
      // to work properly.
      final _TapTracker tracker = _firstTap!;
      _firstTap = null;
      _reject(tracker);
      GestureBinding.instance.gestureArena.release(tracker.pointer);
    }
    _clearTrackers();
  }

  void _registerFirstTap(_TapTracker tracker) {
    _startDoubleTapTimer();
    GestureBinding.instance.gestureArena.hold(tracker.pointer);
    // Note, order is important below in order for the clear -> reject logic to
    // work properly.
    _freezeTracker(tracker);
    _trackers.remove(tracker.pointer);
    _clearTrackers();
    _firstTap = tracker;
  }

  void _registerSecondTap(_TapTracker tracker) {
    _firstTap!.entry.resolve(GestureDisposition.accepted);
    tracker.entry.resolve(GestureDisposition.accepted);
    _freezeTracker(tracker);
    _trackers.remove(tracker.pointer);
    _checkUp(tracker.initialButtons);
    _reset();
  }

  void _clearTrackers() {
    _trackers.values.toList().forEach(_reject);
    assert(_trackers.isEmpty);
  }

  void _freezeTracker(_TapTracker tracker) {
    tracker.stopTrackingPointer(_handleEvent);
  }

  void _startDoubleTapTimer() {
    _doubleTapTimer ??= Timer(kDoubleTapTimeout, _reset);
  }

  void _stopDoubleTapTimer() {
    if (_doubleTapTimer != null) {
      _doubleTapTimer!.cancel();
      _doubleTapTimer = null;
    }
  }

  void _checkUp(int buttons) {
    if (onDoubleTap != null) {
      invokeCallback<void>('onDoubleTap', onDoubleTap!);
    }
  }

  void _checkCancel() {
    if (onDoubleTapCancel != null) {
      invokeCallback<void>('onDoubleTapCancel', onDoubleTapCancel!);
    }
  }

  @override
  String get debugDescription => 'double tap';
}

/// TapGesture represents a full gesture resulting from a single tap sequence,
/// as part of a [MultiTapGestureRecognizer]. Tap gestures are passive, meaning
/// that they will not preempt any other arena member in play.
class _TapGesture extends _TapTracker {
  _TapGesture({
    required this.gestureRecognizer,
    required PointerEvent event,
    required Duration longTapDelay,
    required super.gestureSettings,
  }) : _lastPosition = OffsetPair.fromEventPosition(event),
       super(
         event: event as PointerDownEvent,
         entry: GestureBinding.instance.gestureArena.add(event.pointer, gestureRecognizer),
         doubleTapMinTime: kDoubleTapMinTime,
       ) {
    startTrackingPointer(handleEvent, event.transform);
    if (longTapDelay > Duration.zero) {
      _timer = Timer(longTapDelay, () {
        _timer = null;
        gestureRecognizer._dispatchLongTap(event.pointer, _lastPosition);
      });
    }
  }

  final MultiTapGestureRecognizer gestureRecognizer;

  bool _wonArena = false;
  Timer? _timer;

  OffsetPair _lastPosition;
  OffsetPair? _finalPosition;

  void handleEvent(PointerEvent event) {
    assert(event.pointer == pointer);
    if (event is PointerMoveEvent) {
      if (!isWithinGlobalTolerance(event, computeHitSlop(event.kind, gestureSettings))) {
        cancel();
      } else {
        _lastPosition = OffsetPair.fromEventPosition(event);
      }
    } else if (event is PointerCancelEvent) {
      cancel();
    } else if (event is PointerUpEvent) {
      stopTrackingPointer(handleEvent);
      _finalPosition = OffsetPair.fromEventPosition(event);
      _check();
    }
  }

  @override
  void stopTrackingPointer(PointerRoute route) {
    _timer?.cancel();
    _timer = null;
    super.stopTrackingPointer(route);
  }

  void accept() {
    _wonArena = true;
    _check();
  }

  void reject() {
    stopTrackingPointer(handleEvent);
    gestureRecognizer._dispatchCancel(pointer);
  }

  void cancel() {
    // If we won the arena already, then entry is resolved, so resolving
    // again is a no-op. But we still need to clean up our own state.
    if (_wonArena) {
      reject();
    } else {
      entry.resolve(GestureDisposition.rejected); // eventually calls reject()
    }
  }

  void _check() {
    if (_wonArena && _finalPosition != null) {
      gestureRecognizer._dispatchTap(pointer, _finalPosition!);
    }
  }
}

/// Recognizes taps on a per-pointer basis.
///
/// [MultiTapGestureRecognizer] considers each sequence of pointer events that
/// could constitute a tap independently of other pointers: For example, down-1,
/// down-2, up-1, up-2 produces two taps, on up-1 and up-2.
///
/// See also:
///
///  * [TapGestureRecognizer]
class MultiTapGestureRecognizer extends GestureRecognizer {
  /// Creates a multi-tap gesture recognizer.
  ///
  /// The [longTapDelay] defaults to [Duration.zero], which means
  /// [onLongTapDown] is called immediately after [onTapDown].
  ///
  /// {@macro flutter.gestures.GestureRecognizer.supportedDevices}
  MultiTapGestureRecognizer({
    this.longTapDelay = Duration.zero,
    super.debugOwner,
    super.supportedDevices,
    super.allowedButtonsFilter,
  });

  /// A pointer that might cause a tap has contacted the screen at a particular
  /// location.
  GestureMultiTapDownCallback? onTapDown;

  /// A pointer that will trigger a tap has stopped contacting the screen at a
  /// particular location.
  GestureMultiTapUpCallback? onTapUp;

  /// A tap has occurred.
  GestureMultiTapCallback? onTap;

  /// The pointer that previously triggered [onTapDown] will not end up causing
  /// a tap.
  GestureMultiTapCancelCallback? onTapCancel;

  /// The amount of time between [onTapDown] and [onLongTapDown].
  Duration longTapDelay;

  /// A pointer that might cause a tap is still in contact with the screen at a
  /// particular location after [longTapDelay].
  GestureMultiTapDownCallback? onLongTapDown;

  final Map<int, _TapGesture> _gestureMap = <int, _TapGesture>{};

  @override
  void addAllowedPointer(PointerDownEvent event) {
    assert(!_gestureMap.containsKey(event.pointer));
    _gestureMap[event.pointer] = _TapGesture(
      gestureRecognizer: this,
      event: event,
      longTapDelay: longTapDelay,
      gestureSettings: gestureSettings,
    );
    if (onTapDown != null) {
      invokeCallback<void>('onTapDown', () {
        onTapDown!(
          event.pointer,
          TapDownDetails(
            globalPosition: event.position,
            localPosition: event.localPosition,
            kind: event.kind,
          ),
        );
      });
    }
  }

  @override
  void acceptGesture(int pointer) {
    assert(_gestureMap.containsKey(pointer));
    _gestureMap[pointer]!.accept();
  }

  @override
  void rejectGesture(int pointer) {
    assert(_gestureMap.containsKey(pointer));
    _gestureMap[pointer]!.reject();
    assert(!_gestureMap.containsKey(pointer));
  }

  void _dispatchCancel(int pointer) {
    assert(_gestureMap.containsKey(pointer));
    _gestureMap.remove(pointer);
    if (onTapCancel != null) {
      invokeCallback<void>('onTapCancel', () => onTapCancel!(pointer));
    }
  }

  void _dispatchTap(int pointer, OffsetPair position) {
    assert(_gestureMap.containsKey(pointer));
    _gestureMap.remove(pointer);
    if (onTapUp != null) {
      invokeCallback<void>('onTapUp', () {
        onTapUp!(
          pointer,
          TapUpDetails(
            kind: getKindForPointer(pointer),
            localPosition: position.local,
            globalPosition: position.global,
          ),
        );
      });
    }
    if (onTap != null) {
      invokeCallback<void>('onTap', () => onTap!(pointer));
    }
  }

  void _dispatchLongTap(int pointer, OffsetPair lastPosition) {
    assert(_gestureMap.containsKey(pointer));
    if (onLongTapDown != null) {
      invokeCallback<void>('onLongTapDown', () {
        onLongTapDown!(
          pointer,
          TapDownDetails(
            globalPosition: lastPosition.global,
            localPosition: lastPosition.local,
            kind: getKindForPointer(pointer),
          ),
        );
      });
    }
  }

  @override
  void dispose() {
    final List<_TapGesture> localGestures = List<_TapGesture>.of(_gestureMap.values);
    for (final _TapGesture gesture in localGestures) {
      gesture.cancel();
    }
    // Rejection of each gesture should cause it to be removed from our map
    assert(_gestureMap.isEmpty);
    super.dispose();
  }

  @override
  String get debugDescription => 'multitap';
}

/// Signature used by [SerialTapGestureRecognizer.onSerialTapDown] for when a
/// pointer that might cause a serial tap has contacted the screen at a
/// particular location.
typedef GestureSerialTapDownCallback = void Function(SerialTapDownDetails details);

/// Details for [GestureSerialTapDownCallback], such as the tap count within
/// the series.
///
/// See also:
///
///  * [SerialTapGestureRecognizer], which passes this information to its
///    [SerialTapGestureRecognizer.onSerialTapDown] callback.
class SerialTapDownDetails {
  /// Creates details for a [GestureSerialTapDownCallback].
  ///
  /// The `count` argument must be greater than zero.
  SerialTapDownDetails({
    this.globalPosition = Offset.zero,
    Offset? localPosition,
    required this.kind,
    this.buttons = 0,
    this.count = 1,
  }) : assert(count > 0),
       localPosition = localPosition ?? globalPosition;

  /// The global position at which the pointer contacted the screen.
  final Offset globalPosition;

  /// The local position at which the pointer contacted the screen.
  final Offset localPosition;

  /// The kind of the device that initiated the event.
  final PointerDeviceKind kind;

  /// Which buttons were pressed when the pointer contacted the screen.
  ///
  /// See also:
  ///
  ///  * [PointerEvent.buttons], which this field reflects.
  final int buttons;

  /// The number of consecutive taps that this "tap down" represents.
  ///
  /// This value will always be greater than zero. When the first pointer in a
  /// possible series contacts the screen, this value will be `1`, the second
  /// tap in a double-tap will be `2`, and so on.
  ///
  /// If a tap is determined to not be in the same series as the tap that
  /// preceded it (e.g. because too much time elapsed between the two taps or
  /// the two taps had too much distance between them), then this count will
  /// reset back to `1`, and a new series will have begun.
  final int count;
}

/// Signature used by [SerialTapGestureRecognizer.onSerialTapCancel] for when a
/// pointer that previously triggered a [GestureSerialTapDownCallback] will not
/// end up completing the serial tap.
typedef GestureSerialTapCancelCallback = void Function(SerialTapCancelDetails details);

/// Details for [GestureSerialTapCancelCallback], such as the tap count within
/// the series.
///
/// See also:
///
///  * [SerialTapGestureRecognizer], which passes this information to its
///    [SerialTapGestureRecognizer.onSerialTapCancel] callback.
class SerialTapCancelDetails {
  /// Creates details for a [GestureSerialTapCancelCallback].
  ///
  /// The `count` argument must be greater than zero.
  SerialTapCancelDetails({this.count = 1}) : assert(count > 0);

  /// The number of consecutive taps that were in progress when the gesture was
  /// interrupted.
  ///
  /// This number will match the corresponding count that was specified in
  /// [SerialTapDownDetails.count] for the tap that is being canceled. See
  /// that field for more information on how this count is reported.
  final int count;
}

/// Signature used by [SerialTapGestureRecognizer.onSerialTapUp] for when a
/// pointer that will trigger a serial tap has stopped contacting the screen.
typedef GestureSerialTapUpCallback = void Function(SerialTapUpDetails details);

/// Details for [GestureSerialTapUpCallback], such as the tap count within
/// the series.
///
/// See also:
///
///  * [SerialTapGestureRecognizer], which passes this information to its
///    [SerialTapGestureRecognizer.onSerialTapUp] callback.
class SerialTapUpDetails {
  /// Creates details for a [GestureSerialTapUpCallback].
  ///
  /// The `count` argument must be greater than zero.
  SerialTapUpDetails({
    this.globalPosition = Offset.zero,
    Offset? localPosition,
    this.kind,
    this.count = 1,
  }) : assert(count > 0),
       localPosition = localPosition ?? globalPosition;

  /// The global position at which the pointer contacted the screen.
  final Offset globalPosition;

  /// The local position at which the pointer contacted the screen.
  final Offset localPosition;

  /// The kind of the device that initiated the event.
  final PointerDeviceKind? kind;

  /// The number of consecutive taps that this tap represents.
  ///
  /// This value will always be greater than zero. When the first pointer in a
  /// possible series completes its tap, this value will be `1`, the second
  /// tap in a double-tap will be `2`, and so on.
  ///
  /// If a tap is determined to not be in the same series as the tap that
  /// preceded it (e.g. because too much time elapsed between the two taps or
  /// the two taps had too much distance between them), then this count will
  /// reset back to `1`, and a new series will have begun.
  final int count;
}

/// Recognizes serial taps (taps in a series).
///
/// A collection of taps are considered to be _in a series_ if they occur in
/// rapid succession in the same location (within a tolerance). The number of
/// taps in the series is its count. A double-tap, for instance, is a special
/// case of a tap series with a count of two.
///
/// ### Gesture arena behavior
///
/// [SerialTapGestureRecognizer] competes on all pointer events (regardless of
/// button). It will declare defeat if it determines that a gesture is not a
/// tap (e.g. if the pointer is dragged too far while it's contacting the
/// screen). It will immediately declare victory for every tap that it
/// recognizes.
///
/// Each time a pointer contacts the screen, this recognizer will enter that
/// gesture into the arena. This means that this recognizer will yield multiple
/// winning entries in the arena for a single tap series as the series
/// progresses.
///
/// If this recognizer loses the arena (either by declaring defeat or by
/// another recognizer declaring victory) while the pointer is contacting the
/// screen, it will fire [onSerialTapCancel], and [onSerialTapUp] will not
/// be fired.
///
/// ### Button behavior
///
/// A tap series is defined to have the same buttons across all taps. If a tap
/// with a different combination of buttons is delivered in the middle of a
/// series, it will "steal" the series and begin a new series, starting the
/// count over.
///
/// ### Interleaving tap behavior
///
/// A tap must be _completed_ in order for a subsequent tap to be considered
/// "in the same series" as that tap. Thus, if tap A is in-progress (the down
/// event has been received, but the corresponding up event has not yet been
/// received), and tap B begins (another pointer contacts the screen), tap A
/// will fire [onSerialTapCancel], and tap B will begin a new series (tap B's
/// [SerialTapDownDetails.count] will be 1).
///
/// ### Relation to `TapGestureRecognizer` and `DoubleTapGestureRecognizer`
///
/// [SerialTapGestureRecognizer] fires [onSerialTapDown] and [onSerialTapUp]
/// for every tap that it recognizes (passing the count in the details),
/// regardless of whether that tap is a single-tap, double-tap, etc. This
/// makes it especially useful when you want to respond to every tap in a
/// series. Contrast this with [DoubleTapGestureRecognizer], which only fires
/// if the user completes a double-tap, and [TapGestureRecognizer], which
/// _doesn't_ fire if the recognizer is competing with a
/// `DoubleTapGestureRecognizer`, and the user double-taps.
///
/// For example, consider a list item that should be _selected_ on the first
/// tap and _cause an edit dialog to open_ on a double-tap. If you use both
/// [TapGestureRecognizer] and [DoubleTapGestureRecognizer], there are a few
/// problems:
///
///   1. If the user single-taps the list item, it will not select
///      the list item until after enough time has passed to rule out a
///      double-tap.
///   2. If the user double-taps the list item, it will not select the list
///      item at all.
///
/// The solution is to use [SerialTapGestureRecognizer] and use the tap count
/// to either select the list item or open the edit dialog.
///
/// ### When competing with `TapGestureRecognizer` and `DoubleTapGestureRecognizer`
///
/// Unlike [TapGestureRecognizer] and [DoubleTapGestureRecognizer],
/// [SerialTapGestureRecognizer] aggressively declares victory when it detects
/// a tap, so when it is competing with those gesture recognizers, it will beat
/// them in the arena, regardless of which recognizer entered the arena first.
class SerialTapGestureRecognizer extends GestureRecognizer {
  /// Creates a serial tap gesture recognizer.
  SerialTapGestureRecognizer({
    super.debugOwner,
    super.supportedDevices,
    super.allowedButtonsFilter,
  });

  /// A pointer has contacted the screen at a particular location, which might
  /// be the start of a serial tap.
  ///
  /// If this recognizer loses the arena before the serial tap is completed
  /// (either because the gesture does not end up being a tap or because another
  /// recognizer wins the arena), [onSerialTapCancel] is called next. Otherwise,
  /// [onSerialTapUp] is called next.
  ///
  /// The [SerialTapDownDetails.count] that is passed to this callback
  /// specifies the series tap count.
  GestureSerialTapDownCallback? onSerialTapDown;

  /// A pointer that previously triggered [onSerialTapDown] will not end up
  /// triggering the corresponding [onSerialTapUp].
  ///
  /// If the user completes the serial tap, [onSerialTapUp] is called instead.
  ///
  /// The [SerialTapCancelDetails.count] that is passed to this callback will
  /// match the [SerialTapDownDetails.count] that was passed to the
  /// [onSerialTapDown] callback.
  GestureSerialTapCancelCallback? onSerialTapCancel;

  /// A pointer has stopped contacting the screen at a particular location,
  /// representing a serial tap.
  ///
  /// If the user didn't complete the tap, or if another recognizer won the
  /// arena, then [onSerialTapCancel] is called instead.
  ///
  /// The [SerialTapUpDetails.count] that is passed to this callback specifies
  /// the series tap count and will match the [SerialTapDownDetails.count] that
  /// was passed to the [onSerialTapDown] callback.
  GestureSerialTapUpCallback? onSerialTapUp;

  Timer? _serialTapTimer;
  final List<_TapTracker> _completedTaps = <_TapTracker>[];
  final Map<int, GestureDisposition> _gestureResolutions = <int, GestureDisposition>{};
  _TapTracker? _pendingTap;

  /// Indicates whether this recognizer is currently tracking a pointer that's
  /// in contact with the screen.
  ///
  /// If this is true, it implies that [onSerialTapDown] has fired, but neither
  /// [onSerialTapCancel] nor [onSerialTapUp] have yet fired.
  bool get isTrackingPointer => _pendingTap != null;

  @override
  bool isPointerAllowed(PointerDownEvent event) {
    if (onSerialTapDown == null && onSerialTapCancel == null && onSerialTapUp == null) {
      return false;
    }
    return super.isPointerAllowed(event);
  }

  @override
  void addAllowedPointer(PointerDownEvent event) {
    if ((_completedTaps.isNotEmpty && !_representsSameSeries(_completedTaps.last, event)) ||
        _pendingTap != null) {
      _reset();
    }
    _trackTap(event);
  }

  bool _representsSameSeries(_TapTracker tap, PointerDownEvent event) {
    return tap
            .hasElapsedMinTime() // touch screens often detect touches intermittently
            &&
        tap.hasSameButton(event) &&
        tap.isWithinGlobalTolerance(event, kDoubleTapSlop);
  }

  void _trackTap(PointerDownEvent event) {
    _stopSerialTapTimer();
    if (onSerialTapDown != null) {
      final SerialTapDownDetails details = SerialTapDownDetails(
        globalPosition: event.position,
        localPosition: event.localPosition,
        kind: getKindForPointer(event.pointer),
        buttons: event.buttons,
        count: _completedTaps.length + 1,
      );
      invokeCallback<void>('onSerialTapDown', () => onSerialTapDown!(details));
    }
    final _TapTracker tracker = _TapTracker(
      gestureSettings: gestureSettings,
      event: event,
      entry: GestureBinding.instance.gestureArena.add(event.pointer, this),
      doubleTapMinTime: kDoubleTapMinTime,
    );
    assert(_pendingTap == null);
    _pendingTap = tracker;
    tracker.startTrackingPointer(_handleEvent, event.transform);
  }

  void _handleEvent(PointerEvent event) {
    assert(_pendingTap != null);
    assert(_pendingTap!.pointer == event.pointer);
    final _TapTracker tracker = _pendingTap!;
    if (event is PointerUpEvent) {
      _registerTap(event, tracker);
    } else if (event is PointerMoveEvent) {
      if (!tracker.isWithinGlobalTolerance(event, kDoubleTapTouchSlop)) {
        _reset();
      }
    } else if (event is PointerCancelEvent) {
      _reset();
    }
  }

  @override
  void acceptGesture(int pointer) {
    assert(_pendingTap != null);
    assert(_pendingTap!.pointer == pointer);
    _gestureResolutions[pointer] = GestureDisposition.accepted;
  }

  @override
  void rejectGesture(int pointer) {
    _gestureResolutions[pointer] = GestureDisposition.rejected;
    _reset();
  }

  void _rejectPendingTap() {
    assert(_pendingTap != null);
    final _TapTracker tracker = _pendingTap!;
    _pendingTap = null;
    // Order is important here; the `resolve` call can yield a re-entrant
    // `reset()`, so we need to check cancel here while we can trust the
    // length of our _completedTaps list.
    _checkCancel(_completedTaps.length + 1);
    if (!_gestureResolutions.containsKey(tracker.pointer)) {
      tracker.entry.resolve(GestureDisposition.rejected);
    }
    _stopTrackingPointer(tracker);
  }

  @override
  void dispose() {
    _reset();
    super.dispose();
  }

  void _reset() {
    if (_pendingTap != null) {
      _rejectPendingTap();
    }
    _pendingTap = null;
    _completedTaps.clear();
    _gestureResolutions.clear();
    _stopSerialTapTimer();
  }

  void _registerTap(PointerUpEvent event, _TapTracker tracker) {
    assert(tracker == _pendingTap);
    assert(tracker.pointer == event.pointer);
    _startSerialTapTimer();
    assert(_gestureResolutions[event.pointer] != GestureDisposition.rejected);
    if (!_gestureResolutions.containsKey(event.pointer)) {
      tracker.entry.resolve(GestureDisposition.accepted);
    }
    assert(_gestureResolutions[event.pointer] == GestureDisposition.accepted);
    _stopTrackingPointer(tracker);
    // Note, order is important below in order for the clear -> reject logic to
    // work properly.
    _pendingTap = null;
    _checkUp(event, tracker);
    _completedTaps.add(tracker);
  }

  void _stopTrackingPointer(_TapTracker tracker) {
    tracker.stopTrackingPointer(_handleEvent);
  }

  void _startSerialTapTimer() {
    _serialTapTimer ??= Timer(kDoubleTapTimeout, _reset);
  }

  void _stopSerialTapTimer() {
    if (_serialTapTimer != null) {
      _serialTapTimer!.cancel();
      _serialTapTimer = null;
    }
  }

  void _checkUp(PointerUpEvent event, _TapTracker tracker) {
    if (onSerialTapUp != null) {
      final SerialTapUpDetails details = SerialTapUpDetails(
        globalPosition: event.position,
        localPosition: event.localPosition,
        kind: getKindForPointer(tracker.pointer),
        count: _completedTaps.length + 1,
      );
      invokeCallback<void>('onSerialTapUp', () => onSerialTapUp!(details));
    }
  }

  void _checkCancel(int count) {
    if (onSerialTapCancel != null) {
      final SerialTapCancelDetails details = SerialTapCancelDetails(count: count);
      invokeCallback<void>('onSerialTapCancel', () => onSerialTapCancel!(details));
    }
  }

  @override
  String get debugDescription => 'serial tap';
}
