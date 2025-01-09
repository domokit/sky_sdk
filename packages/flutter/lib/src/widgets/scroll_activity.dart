// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/rendering.dart';
///
/// @docImport 'scroll_controller.dart';
/// @docImport 'scroll_physics.dart';
/// @docImport 'scroll_position.dart';
/// @docImport 'scroll_position_with_single_context.dart';
/// @docImport 'scrollable.dart';
library;

import 'dart:async';
import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';

import 'basic.dart';
import 'framework.dart';
import 'scroll_metrics.dart';
import 'scroll_notification.dart';

/// A backend for a [ScrollActivity].
///
/// Used by subclasses of [ScrollActivity] to manipulate the scroll view that
/// they are acting upon.
///
/// See also:
///
///  * [ScrollActivity], which uses this class as its delegate.
///  * [ScrollPositionWithSingleContext], the main implementation of this interface.
abstract class ScrollActivityDelegate {
  /// The direction in which the scroll view scrolls.
  AxisDirection get axisDirection;

  /// Update the scroll position to the given pixel value.
  ///
  /// Returns the overscroll, if any. See [ScrollPosition.setPixels] for more
  /// information.
  double setPixels(double pixels);

  /// Updates the scroll position by the given amount.
  ///
  /// Appropriate for when the user is directly manipulating the scroll
  /// position, for example by dragging the scroll view. Typically applies
  /// [ScrollPhysics.applyPhysicsToUserOffset] and other transformations that
  /// are appropriate for user-driving scrolling.
  void applyUserOffset(double delta);

  /// Terminate the current activity and start an idle activity.
  void goIdle();

  /// Terminate the current activity and start a ballistic activity with the
  /// given velocity.
  void goBallistic(double velocity);
}

/// Base class for scrolling activities like dragging and flinging.
///
/// See also:
///
///  * [ScrollPosition], which uses [ScrollActivity] objects to manage the
///    [ScrollPosition] of a [Scrollable].
abstract class ScrollActivity {
  /// Initializes [delegate] for subclasses.
  ScrollActivity(this._delegate) {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: 'package:flutter/widgets.dart',
        className: '$ScrollActivity',
        object: this,
      );
    }
  }

  /// The delegate that this activity will use to actuate the scroll view.
  ScrollActivityDelegate get delegate => _delegate;
  ScrollActivityDelegate _delegate;

  bool _isDisposed = false;

  /// Updates the activity's link to the [ScrollActivityDelegate].
  ///
  /// This should only be called when an activity is being moved from a defunct
  /// (or about-to-be defunct) [ScrollActivityDelegate] object to a new one.
  void updateDelegate(ScrollActivityDelegate value) {
    assert(_delegate != value);
    _delegate = value;
  }

  /// Called by the [ScrollActivityDelegate] when it has changed type (for
  /// example, when changing from an Android-style scroll position to an
  /// iOS-style scroll position). If this activity can differ between the two
  /// modes, then it should tell the position to restart that activity
  /// appropriately.
  ///
  /// For example, [BallisticScrollActivity]'s implementation calls
  /// [ScrollActivityDelegate.goBallistic].
  void resetActivity() {}

  /// Dispatch a [ScrollStartNotification] with the given metrics.
  void dispatchScrollStartNotification(ScrollMetrics metrics, BuildContext? context) {
    ScrollStartNotification(metrics: metrics, context: context).dispatch(context);
  }

  /// Dispatch a [ScrollUpdateNotification] with the given metrics and scroll delta.
  void dispatchScrollUpdateNotification(
    ScrollMetrics metrics,
    BuildContext context,
    double scrollDelta,
  ) {
    ScrollUpdateNotification(
      metrics: metrics,
      context: context,
      scrollDelta: scrollDelta,
    ).dispatch(context);
  }

  /// Dispatch an [OverscrollNotification] with the given metrics and overscroll.
  void dispatchOverscrollNotification(
    ScrollMetrics metrics,
    BuildContext context,
    double overscroll,
  ) {
    OverscrollNotification(
      metrics: metrics,
      context: context,
      overscroll: overscroll,
    ).dispatch(context);
  }

  /// Dispatch a [ScrollEndNotification] with the given metrics and overscroll.
  void dispatchScrollEndNotification(ScrollMetrics metrics, BuildContext context) {
    ScrollEndNotification(metrics: metrics, context: context).dispatch(context);
  }

  /// Called when the scroll view that is performing this activity changes its metrics.
  void applyNewDimensions() {}

  /// Whether the scroll view should ignore pointer events while performing this
  /// activity.
  ///
  /// See also:
  ///
  ///  * [isScrolling], which describes whether the activity is considered
  ///    to represent user interaction or not.
  bool get shouldIgnorePointer;

  /// Whether performing this activity constitutes scrolling.
  ///
  /// Used, for example, to determine whether the user scroll
  /// direction (see [ScrollPosition.userScrollDirection]) is
  /// [ScrollDirection.idle].
  ///
  /// See also:
  ///
  ///  * [shouldIgnorePointer], which controls whether pointer events
  ///    are allowed while the activity is live.
  ///  * [UserScrollNotification], which exposes this status.
  bool get isScrolling;

  /// If applicable, the velocity at which the scroll offset is currently
  /// independently changing (i.e. without external stimuli such as a dragging
  /// gestures) in logical pixels per second for this activity.
  double get velocity;

  /// Called when the scroll view stops performing this activity.
  @mustCallSuper
  void dispose() {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }

    _isDisposed = true;
  }

  @override
  String toString() => describeIdentity(this);
}

/// A scroll activity that does nothing.
///
/// When a scroll view is not scrolling, it is performing the idle activity.
///
/// If the [Scrollable] changes dimensions, this activity triggers a ballistic
/// activity to restore the view.
class IdleScrollActivity extends ScrollActivity {
  /// Creates a scroll activity that does nothing.
  IdleScrollActivity(super.delegate);

  @override
  void applyNewDimensions() {
    delegate.goBallistic(0.0);
  }

  @override
  bool get shouldIgnorePointer => false;

  @override
  bool get isScrolling => false;

  @override
  double get velocity => 0.0;
}

/// Interface for holding a [Scrollable] stationary.
///
/// An object that implements this interface is returned by
/// [ScrollPosition.hold]. It holds the scrollable stationary until an activity
/// is started or the [cancel] method is called.
abstract class ScrollHoldController {
  /// Release the [Scrollable], potentially letting it go ballistic if
  /// necessary.
  void cancel();
}

/// A scroll activity that does nothing but can be released to resume
/// normal idle behavior.
///
/// This is used while the user is touching the [Scrollable] but before the
/// touch has become a [Drag].
///
/// For the purposes of [ScrollNotification]s, this activity does not constitute
/// scrolling, and does not prevent the user from interacting with the contents
/// of the [Scrollable] (unlike when a drag has begun or there is a scroll
/// animation underway).
class HoldScrollActivity extends ScrollActivity implements ScrollHoldController {
  /// Creates a scroll activity that does nothing.
  HoldScrollActivity({required ScrollActivityDelegate delegate, this.onHoldCanceled})
    : super(delegate);

  /// Called when [dispose] is called.
  final VoidCallback? onHoldCanceled;

  @override
  bool get shouldIgnorePointer => false;

  @override
  bool get isScrolling => false;

  @override
  double get velocity => 0.0;

  @override
  void cancel() {
    delegate.goBallistic(0.0);
  }

  @override
  void dispose() {
    onHoldCanceled?.call();
    super.dispose();
  }
}

/// Scrolls a scroll view as the user drags their finger across the screen.
///
/// See also:
///
///  * [DragScrollActivity], which is the activity the scroll view performs
///    while a drag is underway.
class ScrollDragController implements Drag {
  /// Creates an object that scrolls a scroll view as the user drags their
  /// finger across the screen.
  ScrollDragController({
    required ScrollActivityDelegate delegate,
    required DragStartDetails details,
    this.onDragCanceled,
    this.carriedVelocity,
    this.motionStartDistanceThreshold,
  }) : assert(
         motionStartDistanceThreshold == null || motionStartDistanceThreshold > 0.0,
         'motionStartDistanceThreshold must be a positive number or null',
       ),
       _delegate = delegate,
       _lastDetails = details,
       _retainMomentum = carriedVelocity != null && carriedVelocity != 0.0,
       _lastNonStationaryTimestamp = details.sourceTimeStamp,
       _kind = details.kind,
       _offsetSinceLastStop = motionStartDistanceThreshold == null ? null : 0.0 {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: 'package:flutter/widgets.dart',
        className: '$ScrollDragController',
        object: this,
      );
    }
  }

  /// The object that will actuate the scroll view as the user drags.
  ScrollActivityDelegate get delegate => _delegate;
  ScrollActivityDelegate _delegate;

  /// Called when [dispose] is called.
  final VoidCallback? onDragCanceled;

  /// Velocity that was present from a previous [ScrollActivity] when this drag
  /// began.
  final double? carriedVelocity;

  /// Amount of pixels in either direction the drag has to move by to start
  /// scroll movement again after each time scrolling came to a stop.
  final double? motionStartDistanceThreshold;

  Duration? _lastNonStationaryTimestamp;
  bool _retainMomentum;

  /// Null if already in motion or has no [motionStartDistanceThreshold].
  double? _offsetSinceLastStop;

  /// Maximum amount of time interval the drag can have consecutive stationary
  /// pointer update events before losing the momentum carried from a previous
  /// scroll activity.
  static const Duration momentumRetainStationaryDurationThreshold = Duration(milliseconds: 20);

  /// The minimum amount of velocity needed to apply the [carriedVelocity] at
  /// the end of a drag. Expressed as a factor. For example with a
  /// [carriedVelocity] of 2000, we will need a velocity of at least 1000 to
  /// apply the [carriedVelocity] as well. If the velocity does not meet the
  /// threshold, the [carriedVelocity] is lost. Decided by fair eyeballing
  /// with the scroll_overlay platform test.
  static const double momentumRetainVelocityThresholdFactor = 0.5;

  /// Maximum amount of time interval the drag can have consecutive stationary
  /// pointer update events before needing to break the
  /// [motionStartDistanceThreshold] to start motion again.
  static const Duration motionStoppedDurationThreshold = Duration(milliseconds: 50);

  /// The drag distance past which, a [motionStartDistanceThreshold] breaking
  /// drag is considered a deliberate fling.
  static const double _bigThresholdBreakDistance = 24.0;

  bool get _reversed => axisDirectionIsReversed(delegate.axisDirection);

  /// Updates the controller's link to the [ScrollActivityDelegate].
  ///
  /// This should only be called when a controller is being moved from a defunct
  /// (or about-to-be defunct) [ScrollActivityDelegate] object to a new one.
  void updateDelegate(ScrollActivityDelegate value) {
    assert(_delegate != value);
    _delegate = value;
  }

  /// Determines whether to lose the existing incoming velocity when starting
  /// the drag.
  void _maybeLoseMomentum(double offset, Duration? timestamp) {
    if (_retainMomentum &&
        offset == 0.0 &&
        (timestamp == null || // If drag event has no timestamp, we lose momentum.
            timestamp - _lastNonStationaryTimestamp! > momentumRetainStationaryDurationThreshold)) {
      // If pointer is stationary for too long, we lose momentum.
      _retainMomentum = false;
    }
  }

  /// If a motion start threshold exists, determine whether the threshold needs
  /// to be broken to scroll. Also possibly apply an offset adjustment when
  /// threshold is first broken.
  ///
  /// Returns `0.0` when stationary or within threshold. Returns `offset`
  /// transparently when already in motion.
  double _adjustForScrollStartThreshold(double offset, Duration? timestamp) {
    if (timestamp == null) {
      // If we can't track time, we can't apply thresholds.
      // May be null for proxied drags like via accessibility.
      return offset;
    }
    if (offset == 0.0) {
      if (motionStartDistanceThreshold != null &&
          _offsetSinceLastStop == null &&
          timestamp - _lastNonStationaryTimestamp! > motionStoppedDurationThreshold) {
        // Enforce a new threshold.
        _offsetSinceLastStop = 0.0;
      }
      // Not moving can't break threshold.
      return 0.0;
    } else {
      if (_offsetSinceLastStop == null) {
        // Already in motion or no threshold behavior configured such as for
        // Android. Allow transparent offset transmission.
        return offset;
      } else {
        _offsetSinceLastStop = _offsetSinceLastStop! + offset;
        if (_offsetSinceLastStop!.abs() > motionStartDistanceThreshold!) {
          // Threshold broken.
          _offsetSinceLastStop = null;
          if (offset.abs() > _bigThresholdBreakDistance) {
            // This is heuristically a very deliberate fling. Leave the motion
            // unaffected.
            return offset;
          } else {
            // This is a normal speed threshold break.
            return math.min(
                  // Ease into the motion when the threshold is initially broken
                  // to avoid a visible jump.
                  motionStartDistanceThreshold! / 3.0,
                  offset.abs(),
                ) *
                offset.sign;
          }
        } else {
          return 0.0;
        }
      }
    }
  }

  @override
  void update(DragUpdateDetails details) {
    assert(details.primaryDelta != null);
    _lastDetails = details;
    double offset = details.primaryDelta!;
    if (offset != 0.0) {
      _lastNonStationaryTimestamp = details.sourceTimeStamp;
    }
    // By default, iOS platforms carries momentum and has a start threshold
    // (configured in [BouncingScrollPhysics]). The 2 operations below are
    // no-ops on Android.
    _maybeLoseMomentum(offset, details.sourceTimeStamp);
    offset = _adjustForScrollStartThreshold(offset, details.sourceTimeStamp);
    if (offset == 0.0) {
      return;
    }
    if (_reversed) {
      offset = -offset;
    }
    delegate.applyUserOffset(offset);
  }

  @override
  void end(DragEndDetails details) {
    assert(details.primaryVelocity != null);
    // We negate the velocity here because if the touch is moving downwards,
    // the scroll has to move upwards. It's the same reason that update()
    // above negates the delta before applying it to the scroll offset.
    double velocity = -details.primaryVelocity!;
    if (_reversed) {
      velocity = -velocity;
    }
    _lastDetails = details;

    if (_retainMomentum) {
      // Build momentum only if dragging in the same direction.
      final bool isFlingingInSameDirection = velocity.sign == carriedVelocity!.sign;
      // Build momentum only if the velocity of the last drag was not
      // substantially lower than the carried momentum.
      final bool isVelocityNotSubstantiallyLessThanCarriedMomentum =
          velocity.abs() > carriedVelocity!.abs() * momentumRetainVelocityThresholdFactor;
      if (isFlingingInSameDirection && isVelocityNotSubstantiallyLessThanCarriedMomentum) {
        velocity += carriedVelocity!;
      }
    }
    delegate.goBallistic(velocity);
  }

  @override
  void cancel() {
    delegate.goBallistic(0.0);
  }

  /// Called by the delegate when it is no longer sending events to this object.
  @mustCallSuper
  void dispose() {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
    _lastDetails = null;
    onDragCanceled?.call();
  }

  /// The type of input device driving the drag.
  final PointerDeviceKind? _kind;

  /// The most recently observed [DragStartDetails], [DragUpdateDetails], or
  /// [DragEndDetails] object.
  dynamic get lastDetails => _lastDetails;
  dynamic _lastDetails;

  @override
  String toString() => describeIdentity(this);
}

/// The activity a scroll view performs when the user drags their finger
/// across the screen.
///
/// See also:
///
///  * [ScrollDragController], which listens to the [Drag] and actually scrolls
///    the scroll view.
class DragScrollActivity extends ScrollActivity {
  /// Creates an activity for when the user drags their finger across the
  /// screen.
  DragScrollActivity(super.delegate, ScrollDragController controller) : _controller = controller;

  ScrollDragController? _controller;

  @override
  void dispatchScrollStartNotification(ScrollMetrics metrics, BuildContext? context) {
    final dynamic lastDetails = _controller!.lastDetails;
    assert(lastDetails is DragStartDetails);
    ScrollStartNotification(
      metrics: metrics,
      context: context,
      dragDetails: lastDetails as DragStartDetails,
    ).dispatch(context);
  }

  @override
  void dispatchScrollUpdateNotification(
    ScrollMetrics metrics,
    BuildContext context,
    double scrollDelta,
  ) {
    final dynamic lastDetails = _controller!.lastDetails;
    assert(lastDetails is DragUpdateDetails);
    ScrollUpdateNotification(
      metrics: metrics,
      context: context,
      scrollDelta: scrollDelta,
      dragDetails: lastDetails as DragUpdateDetails,
    ).dispatch(context);
  }

  @override
  void dispatchOverscrollNotification(
    ScrollMetrics metrics,
    BuildContext context,
    double overscroll,
  ) {
    final dynamic lastDetails = _controller!.lastDetails;
    assert(lastDetails is DragUpdateDetails);
    OverscrollNotification(
      metrics: metrics,
      context: context,
      overscroll: overscroll,
      dragDetails: lastDetails as DragUpdateDetails,
    ).dispatch(context);
  }

  @override
  void dispatchScrollEndNotification(ScrollMetrics metrics, BuildContext context) {
    // We might not have DragEndDetails yet if we're being called from beginActivity.
    final dynamic lastDetails = _controller!.lastDetails;
    ScrollEndNotification(
      metrics: metrics,
      context: context,
      dragDetails: lastDetails is DragEndDetails ? lastDetails : null,
    ).dispatch(context);
  }

  @override
  bool get shouldIgnorePointer => _controller?._kind != PointerDeviceKind.trackpad;

  @override
  bool get isScrolling => true;

  // DragScrollActivity is not independently changing velocity yet
  // until the drag is ended.
  @override
  double get velocity => 0.0;

  @override
  void dispose() {
    _controller = null;
    super.dispose();
  }

  @override
  String toString() {
    return '${describeIdentity(this)}($_controller)';
  }
}

/// An activity that animates a scroll view based on a physics [Simulation].
///
/// A [BallisticScrollActivity] is typically used when the user lifts their
/// finger off the screen to continue the scrolling gesture with the current velocity.
///
/// [BallisticScrollActivity] is also used to restore a scroll view to a valid
/// scroll offset when the geometry of the scroll view changes. In these
/// situations, the [Simulation] typically starts with a zero velocity.
///
/// See also:
///
///  * [DrivenScrollActivity], which animates a scroll view based on a set of
///    animation parameters.
class BallisticScrollActivity extends ScrollActivity {
  /// Creates an activity that animates a scroll view based on a [simulation].
  BallisticScrollActivity(
    super.delegate,
    Simulation simulation,
    TickerProvider vsync,
    this.shouldIgnorePointer,
  ) {
    _controller =
        AnimationController.unbounded(
            debugLabel: kDebugMode ? objectRuntimeType(this, 'BallisticScrollActivity') : null,
            vsync: vsync,
          )
          ..addListener(_tick)
          ..animateWith(
            simulation,
          ).whenComplete(_end); // won't trigger if we dispose _controller before it completes.
  }

  late AnimationController _controller;

  @override
  void resetActivity() {
    delegate.goBallistic(velocity);
  }

  @override
  void applyNewDimensions() {
    delegate.goBallistic(velocity);
  }

  void _tick() {
    if (!applyMoveTo(_controller.value)) {
      delegate.goIdle();
    }
  }

  /// Move the position to the given location.
  ///
  /// If the new position was fully applied, returns true. If there was any
  /// overflow, returns false.
  ///
  /// The default implementation calls [ScrollActivityDelegate.setPixels]
  /// and returns true if the overflow was zero.
  @protected
  bool applyMoveTo(double value) {
    return delegate.setPixels(value).abs() < precisionErrorTolerance;
  }

  void _end() {
    // Check if the activity was disposed before going ballistic because _end might be called
    // if _controller is disposed just after completion.
    if (!_isDisposed) {
      delegate.goBallistic(0.0);
    }
  }

  @override
  void dispatchOverscrollNotification(
    ScrollMetrics metrics,
    BuildContext context,
    double overscroll,
  ) {
    OverscrollNotification(
      metrics: metrics,
      context: context,
      overscroll: overscroll,
      velocity: velocity,
    ).dispatch(context);
  }

  @override
  final bool shouldIgnorePointer;

  @override
  bool get isScrolling => true;

  @override
  double get velocity => _controller.velocity;

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  String toString() {
    return '${describeIdentity(this)}($_controller)';
  }
}

/// An activity that animates a scroll view based on animation parameters.
///
/// For example, a [DrivenScrollActivity] is used to implement
/// [ScrollController.animateTo].
///
/// See also:
///
///  * [BallisticScrollActivity], which animates a scroll view based on a
///    physics [Simulation].
class DrivenScrollActivity extends ScrollActivity {
  /// Creates an activity that animates a scroll view based on animation
  /// parameters.
  DrivenScrollActivity(
    super.delegate, {
    required double from,
    required double to,
    required Duration duration,
    required Curve curve,
    required TickerProvider vsync,
  }) : assert(duration > Duration.zero) {
    _completer = Completer<void>();
    _controller =
        AnimationController.unbounded(
            value: from,
            debugLabel: objectRuntimeType(this, 'DrivenScrollActivity'),
            vsync: vsync,
          )
          ..addListener(_tick)
          ..animateTo(
            to,
            duration: duration,
            curve: curve,
          ).whenComplete(_end); // won't trigger if we dispose _controller before it completes.
  }

  late final Completer<void> _completer;
  late final AnimationController _controller;

  /// A [Future] that completes when the activity stops.
  ///
  /// For example, this [Future] will complete if the animation reaches the end
  /// or if the user interacts with the scroll view in way that causes the
  /// animation to stop before it reaches the end.
  Future<void> get done => _completer.future;

  void _tick() {
    if (delegate.setPixels(_controller.value) != 0.0) {
      delegate.goIdle();
    }
  }

  void _end() {
    // Check if the activity was disposed before going ballistic because _end might be called
    // if _controller is disposed just after completion.
    if (!_isDisposed) {
      delegate.goBallistic(velocity);
    }
  }

  @override
  void dispatchOverscrollNotification(
    ScrollMetrics metrics,
    BuildContext context,
    double overscroll,
  ) {
    OverscrollNotification(
      metrics: metrics,
      context: context,
      overscroll: overscroll,
      velocity: velocity,
    ).dispatch(context);
  }

  @override
  bool get shouldIgnorePointer => true;

  @override
  bool get isScrolling => true;

  @override
  double get velocity => _controller.velocity;

  @override
  void dispose() {
    _completer.complete();
    _controller.dispose();
    super.dispose();
  }

  @override
  String toString() {
    return '${describeIdentity(this)}($_controller)';
  }
}
