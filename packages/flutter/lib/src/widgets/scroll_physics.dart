// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'scroll_activity.dart';
/// @docImport 'scroll_configuration.dart';
/// @docImport 'scroll_position.dart';
/// @docImport 'scroll_position_with_single_context.dart';
/// @docImport 'scroll_view.dart';
/// @docImport 'scrollable.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/painting.dart' show AxisDirection;
import 'package:flutter/physics.dart';

import 'binding.dart' show WidgetsBinding;
import 'framework.dart';
import 'overscroll_indicator.dart';
import 'scroll_metrics.dart';
import 'scroll_simulation.dart';
import 'view.dart';

export 'package:flutter/physics.dart' show ScrollSpringSimulation, Simulation, Tolerance;

/// The rate at which scroll momentum will be decelerated.
enum ScrollDecelerationRate {
  /// Standard deceleration, aligned with mobile software expectations.
  normal,

  /// Increased deceleration, aligned with desktop software expectations.
  ///
  /// Appropriate for use with input devices more precise than touch screens,
  /// such as trackpads or mouse wheels.
  fast,
}

// Examples can assume:
// class FooScrollPhysics extends ScrollPhysics {
//   const FooScrollPhysics({ super.parent });
//   @override
//   FooScrollPhysics applyTo(ScrollPhysics? ancestor) {
//     return FooScrollPhysics(parent: buildParent(ancestor));
//   }
// }
// class BarScrollPhysics extends ScrollPhysics {
//   const BarScrollPhysics({ super.parent });
// }

/// Determines the physics of a [Scrollable] widget.
///
/// For example, determines how the [Scrollable] will behave when the user
/// reaches the maximum scroll extent or when the user stops scrolling.
///
/// When starting a physics [Simulation], the current scroll position and
/// velocity are used as the initial conditions for the particle in the
/// simulation. The movement of the particle in the simulation is then used to
/// determine the scroll position for the widget.
///
/// Instead of creating your own subclasses, [parent] can be used to combine
/// [ScrollPhysics] objects of different types to get the desired scroll physics.
/// For example:
///
/// ```dart
/// const BouncingScrollPhysics(parent: AlwaysScrollableScrollPhysics())
/// ```
///
/// You can also use `applyTo`, which is useful when you already have
/// an instance of [ScrollPhysics]:
///
/// ```dart
/// ScrollPhysics physics = const BouncingScrollPhysics();
/// // ...
/// final ScrollPhysics mergedPhysics = physics.applyTo(const AlwaysScrollableScrollPhysics());
/// ```
///
/// When implementing a subclass, you must override [applyTo] so that it returns
/// an appropriate instance of your subclass.  Otherwise, classes like
/// [Scrollable] that inform a [ScrollPosition] will combine them with
/// the default [ScrollPhysics] object instead of your custom subclass.
@immutable
class ScrollPhysics {
  /// Creates an object with the default scroll physics.
  const ScrollPhysics({this.parent});

  /// If non-null, determines the default behavior for each method.
  ///
  /// If a subclass of [ScrollPhysics] does not override a method, that subclass
  /// will inherit an implementation from this base class that defers to
  /// [parent]. This mechanism lets you assemble novel combinations of
  /// [ScrollPhysics] subclasses at runtime. For example:
  ///
  /// ```dart
  /// const BouncingScrollPhysics(parent: AlwaysScrollableScrollPhysics())
  /// ```
  ///
  /// will result in a [ScrollPhysics] that has the combined behavior
  /// of [BouncingScrollPhysics] and [AlwaysScrollableScrollPhysics]:
  /// behaviors that are not specified in [BouncingScrollPhysics]
  /// (e.g. [shouldAcceptUserOffset]) will defer to [AlwaysScrollableScrollPhysics].
  final ScrollPhysics? parent;

  /// If [parent] is null then return ancestor, otherwise recursively build a
  /// ScrollPhysics that has [ancestor] as its parent.
  ///
  /// This method is typically used to define [applyTo] methods like:
  ///
  /// ```dart
  /// class MyScrollPhysics extends ScrollPhysics {
  ///   const MyScrollPhysics({ super.parent });
  ///
  ///   @override
  ///   MyScrollPhysics applyTo(ScrollPhysics? ancestor) {
  ///     return MyScrollPhysics(parent: buildParent(ancestor));
  ///   }
  ///
  ///   // ...
  /// }
  /// ```
  @protected
  ScrollPhysics? buildParent(ScrollPhysics? ancestor) => parent?.applyTo(ancestor) ?? ancestor;

  /// Combines this [ScrollPhysics] instance with the given physics.
  ///
  /// The returned object uses this instance's physics when it has an
  /// opinion, and defers to the given `ancestor` object's physics
  /// when it does not.
  ///
  /// If [parent] is null then this returns a [ScrollPhysics] with the
  /// same [runtimeType], but where the [parent] has been replaced
  /// with the [ancestor].
  ///
  /// If this scroll physics object already has a parent, then this
  /// method is applied recursively and ancestor will appear at the
  /// end of the existing chain of parents.
  ///
  /// Calling this method with a null argument will copy the current
  /// object. This is inefficient.
  ///
  /// {@tool snippet}
  ///
  /// In the following example, the [applyTo] method is used to combine the
  /// scroll physics of two [ScrollPhysics] objects. The resulting [ScrollPhysics]
  /// `x` has the same behavior as `y`.
  ///
  /// ```dart
  /// final FooScrollPhysics x = const FooScrollPhysics().applyTo(const BarScrollPhysics());
  /// const FooScrollPhysics y = FooScrollPhysics(parent: BarScrollPhysics());
  /// ```
  /// {@end-tool}
  ///
  /// ## Implementing [applyTo]
  ///
  /// When creating a custom [ScrollPhysics] subclass, this method
  /// must be implemented. If the physics class has no constructor
  /// arguments, then implementing this method is merely a matter of
  /// calling the constructor with a [parent] constructed using
  /// [buildParent], as follows:
  ///
  /// ```dart
  /// class MyScrollPhysics extends ScrollPhysics {
  ///   const MyScrollPhysics({ super.parent });
  ///
  ///   @override
  ///   MyScrollPhysics applyTo(ScrollPhysics? ancestor) {
  ///     return MyScrollPhysics(parent: buildParent(ancestor));
  ///   }
  ///
  ///   // ...
  /// }
  /// ```
  ///
  /// If the physics class has constructor arguments, they must be passed to
  /// the constructor here as well, so as to create a clone.
  ///
  /// See also:
  ///
  ///  * [buildParent], a utility method that's often used to define [applyTo]
  ///    methods for [ScrollPhysics] subclasses.
  ScrollPhysics applyTo(ScrollPhysics? ancestor) {
    return ScrollPhysics(parent: buildParent(ancestor));
  }

  /// Used by [DragScrollActivity] and other user-driven activities to convert
  /// an offset in logical pixels as provided by the [DragUpdateDetails] into a
  /// delta to apply (subtract from the current position) using
  /// [ScrollActivityDelegate.setPixels].
  ///
  /// This is used by some [ScrollPosition] subclasses to apply friction during
  /// overscroll situations.
  ///
  /// This method must not adjust parts of the offset that are entirely within
  /// the bounds described by the given `position`.
  ///
  /// The given `position` is only valid during this method call. Do not keep a
  /// reference to it to use later, as the values may update, may not update, or
  /// may update to reflect an entirely unrelated scrollable.
  double applyPhysicsToUserOffset(ScrollMetrics position, double offset) {
    return parent?.applyPhysicsToUserOffset(position, offset) ?? offset;
  }

  /// Whether the scrollable should let the user adjust the scroll offset, for
  /// example by dragging. If [allowUserScrolling] is false, the scrollable
  /// will never allow user input to change the scroll position.
  ///
  /// By default, the user can manipulate the scroll offset if, and only if,
  /// there is actually content outside the viewport to reveal.
  ///
  /// The given `position` is only valid during this method call. Do not keep a
  /// reference to it to use later, as the values may update, may not update, or
  /// may update to reflect an entirely unrelated scrollable.
  bool shouldAcceptUserOffset(ScrollMetrics position) {
    if (!allowUserScrolling) {
      return false;
    }

    if (parent == null) {
      return position.pixels != 0.0 || position.minScrollExtent != position.maxScrollExtent;
    }
    return parent!.shouldAcceptUserOffset(position);
  }

  /// Provides a heuristic to determine if expensive frame-bound tasks should be
  /// deferred.
  ///
  /// The `velocity` parameter may be positive, negative, or zero.
  ///
  /// The `context` parameter normally refers to the [BuildContext] of the widget
  /// making the call, such as an [Image] widget in a [ListView].
  ///
  /// This can be used to determine whether decoding or fetching complex data
  /// for the currently visible part of the viewport should be delayed
  /// to avoid doing work that will not have a chance to appear before a new
  /// frame is rendered.
  ///
  /// For example, a list of images could use this logic to delay decoding
  /// images until scrolling is slow enough to actually render the decoded
  /// image to the screen.
  ///
  /// The default implementation is a heuristic that compares the current
  /// scroll velocity in local logical pixels to the longest side of the window
  /// in physical pixels. Implementers can change this heuristic by overriding
  /// this method and providing their custom physics to the scrollable widget.
  /// For example, an application that changes the local coordinate system with
  /// a large perspective transform could provide a more or less aggressive
  /// heuristic depending on whether the transform was increasing or decreasing
  /// the overall scale between the global screen and local scrollable
  /// coordinate systems.
  ///
  /// The default implementation is stateless, and provides a point-in-time
  /// decision about how fast the scrollable is scrolling. It would always
  /// return true for a scrollable that is animating back and forth at high
  /// velocity in a loop. It is assumed that callers will handle such
  /// a case, or that a custom stateful implementation would be written that
  /// tracks the sign of the velocity on successive calls.
  ///
  /// Returning true from this method indicates that the current scroll velocity
  /// is great enough that expensive operations impacting the UI should be
  /// deferred.
  bool recommendDeferredLoading(double velocity, ScrollMetrics metrics, BuildContext context) {
    if (parent == null) {
      final double maxPhysicalPixels = View.of(context).physicalSize.longestSide;
      return velocity.abs() > maxPhysicalPixels;
    }
    return parent!.recommendDeferredLoading(velocity, metrics, context);
  }

  /// Determines the overscroll by applying the boundary conditions.
  ///
  /// Called by [ScrollPosition.applyBoundaryConditions], which is called by
  /// [ScrollPosition.setPixels] just before the [ScrollPosition.pixels] value
  /// is updated, to determine how much of the offset is to be clamped off and
  /// sent to [ScrollPosition.didOverscrollBy].
  ///
  /// The `value` argument is guaranteed to not equal the [ScrollMetrics.pixels]
  /// of the `position` argument when this is called.
  ///
  /// It is possible for this method to be called when the `position` describes
  /// an already-out-of-bounds position. In that case, the boundary conditions
  /// should usually only prevent a further increase in the extent to which the
  /// position is out of bounds, allowing a decrease to be applied successfully,
  /// so that (for instance) an animation can smoothly snap an out of bounds
  /// position to the bounds. See [BallisticScrollActivity].
  ///
  /// This method must not clamp parts of the offset that are entirely within
  /// the bounds described by the given `position`.
  ///
  /// The given `position` is only valid during this method call. Do not keep a
  /// reference to it to use later, as the values may update, may not update, or
  /// may update to reflect an entirely unrelated scrollable.
  ///
  /// ## Examples
  ///
  /// [BouncingScrollPhysics] returns zero. In other words, it allows scrolling
  /// past the boundary unhindered.
  ///
  /// [ClampingScrollPhysics] returns the amount by which the value is beyond
  /// the position or the boundary, whichever is furthest from the content. In
  /// other words, it disallows scrolling past the boundary, but allows
  /// scrolling back from being overscrolled, if for some reason the position
  /// ends up overscrolled.
  double applyBoundaryConditions(ScrollMetrics position, double value) {
    return parent?.applyBoundaryConditions(position, value) ?? 0.0;
  }

  /// Describes what the scroll position should be given new viewport dimensions.
  ///
  /// This is called by [ScrollPosition.correctForNewDimensions].
  ///
  /// The arguments consist of the scroll metrics as they stood in the previous
  /// frame and the scroll metrics as they now stand after the last layout,
  /// including the position and minimum and maximum scroll extents; a flag
  /// indicating if the current [ScrollActivity] considers that the user is
  /// actively scrolling (see [ScrollActivity.isScrolling]); and the current
  /// velocity of the scroll position, if it is being driven by the scroll
  /// activity (this is 0.0 during a user gesture) (see
  /// [ScrollActivity.velocity]).
  ///
  /// The scroll metrics will be identical except for the
  /// [ScrollMetrics.minScrollExtent] and [ScrollMetrics.maxScrollExtent]. They
  /// are referred to as the `oldPosition` and `newPosition` (even though they
  /// both technically have the same "position", in the form of
  /// [ScrollMetrics.pixels]) because they are generated from the
  /// [ScrollPosition] before and after updating the scroll extents.
  ///
  /// If the returned value does not exactly match the scroll offset given by
  /// the `newPosition` argument (see [ScrollMetrics.pixels]), then the
  /// [ScrollPosition] will call [ScrollPosition.correctPixels] to update the
  /// new scroll position to the returned value, and layout will be re-run. This
  /// is expensive. The new value is subject to further manipulation by
  /// [applyBoundaryConditions].
  ///
  /// If the returned value _does_ match the `newPosition.pixels` scroll offset
  /// exactly, then [ScrollPosition.applyNewDimensions] will be called next. In
  /// that case, [applyBoundaryConditions] is not applied to the return value.
  ///
  /// The given [ScrollMetrics] are only valid during this method call. Do not
  /// keep references to them to use later, as the values may update, may not
  /// update, or may update to reflect an entirely unrelated scrollable.
  ///
  /// The default implementation returns the [ScrollMetrics.pixels] of the
  /// `newPosition`, which indicates that the current scroll offset is
  /// acceptable.
  ///
  /// See also:
  ///
  ///  * [RangeMaintainingScrollPhysics], which is enabled by default, and
  ///    which prevents unexpected changes to the content dimensions from
  ///    causing the scroll position to get any further out of bounds.
  double adjustPositionForNewDimensions({
    required ScrollMetrics oldPosition,
    required ScrollMetrics newPosition,
    required bool isScrolling,
    required double velocity,
  }) {
    if (parent == null) {
      return newPosition.pixels;
    }
    return parent!.adjustPositionForNewDimensions(
      oldPosition: oldPosition,
      newPosition: newPosition,
      isScrolling: isScrolling,
      velocity: velocity,
    );
  }

  /// Returns a simulation for ballistic scrolling starting from the given
  /// position with the given velocity.
  ///
  /// This is used by [ScrollPositionWithSingleContext] in the
  /// [ScrollPositionWithSingleContext.goBallistic] method. If the result
  /// is non-null, [ScrollPositionWithSingleContext] will begin a
  /// [BallisticScrollActivity] with the returned value. Otherwise, it will
  /// begin an idle activity instead.
  ///
  /// The given `position` is only valid during this method call. Do not keep a
  /// reference to it to use later, as the values may update, may not update, or
  /// may update to reflect an entirely unrelated scrollable.
  ///
  /// This method can potentially be called in every frame, even in the middle
  /// of what the user perceives as a single ballistic scroll.  For example, in
  /// a [ListView] when previously off-screen items come into view and are laid
  /// out, this method may be called with a new [ScrollMetrics.maxScrollExtent].
  /// The method implementation should ensure that when the same ballistic
  /// scroll motion is still intended, these calls have no side effects on the
  /// physics beyond continuing that motion.
  ///
  /// Generally this is ensured by having the [Simulation] conform to a physical
  /// metaphor of a particle in ballistic flight, where the forces on the
  /// particle depend only on its position, velocity, and environment, and not
  /// on the current time or any internal state.  This means that the
  /// time-derivative of [Simulation.dx] should be possible to write
  /// mathematically as a function purely of the values of [Simulation.x],
  /// [Simulation.dx], and the parameters used to construct the [Simulation],
  /// independent of the time.
  // TODO(gnprice): Some scroll physics in the framework violate that invariant; fix them.
  //   An audit found three cases violating the invariant:
  //     https://github.com/flutter/flutter/issues/120338
  //     https://github.com/flutter/flutter/issues/120340
  //     https://github.com/flutter/flutter/issues/109675
  Simulation? createBallisticSimulation(ScrollMetrics position, double velocity) {
    return parent?.createBallisticSimulation(position, velocity);
  }

  static final SpringDescription _kDefaultSpring = SpringDescription.withDampingRatio(
    mass: 0.5,
    stiffness: 100.0,
    ratio: 1.1,
  );

  /// The spring to use for ballistic simulations.
  SpringDescription get spring => parent?.spring ?? _kDefaultSpring;

  /// Deprecated. Call [toleranceFor] instead.
  @Deprecated(
    'Call toleranceFor instead. '
    'This feature was deprecated after v3.7.0-13.0.pre.',
  )
  Tolerance get tolerance {
    return toleranceFor(
      FixedScrollMetrics(
        minScrollExtent: null,
        maxScrollExtent: null,
        pixels: null,
        viewportDimension: null,
        axisDirection: AxisDirection.down,
        devicePixelRatio: WidgetsBinding.instance.window.devicePixelRatio,
      ),
    );
  }

  /// The tolerance to use for ballistic simulations.
  Tolerance toleranceFor(ScrollMetrics metrics) {
    return parent?.toleranceFor(metrics) ??
        Tolerance(
          velocity: 1.0 / (0.050 * metrics.devicePixelRatio), // logical pixels per second
          distance: 1.0 / metrics.devicePixelRatio, // logical pixels
        );
  }

  /// The minimum distance an input pointer drag must have moved to be
  /// considered a scroll fling gesture.
  ///
  /// This value is typically compared with the distance traveled along the
  /// scrolling axis.
  ///
  /// See also:
  ///
  ///  * [VelocityTracker.getVelocityEstimate], which computes the velocity
  ///    of a press-drag-release gesture.
  double get minFlingDistance => parent?.minFlingDistance ?? kTouchSlop;

  /// The minimum velocity for an input pointer drag to be considered a
  /// scroll fling.
  ///
  /// This value is typically compared with the magnitude of fling gesture's
  /// velocity along the scrolling axis.
  ///
  /// See also:
  ///
  ///  * [VelocityTracker.getVelocityEstimate], which computes the velocity
  ///    of a press-drag-release gesture.
  double get minFlingVelocity => parent?.minFlingVelocity ?? kMinFlingVelocity;

  /// Scroll fling velocity magnitudes will be clamped to this value.
  double get maxFlingVelocity => parent?.maxFlingVelocity ?? kMaxFlingVelocity;

  /// Returns the velocity carried on repeated flings.
  ///
  /// The function is applied to the existing scroll velocity when another
  /// scroll drag is applied in the same direction.
  ///
  /// By default, physics for platforms other than iOS doesn't carry momentum.
  double carriedMomentum(double existingVelocity) {
    return parent?.carriedMomentum(existingVelocity) ?? 0.0;
  }

  /// The minimum amount of pixel distance drags must move by to start motion
  /// the first time or after each time the drag motion stopped.
  ///
  /// If null, no minimum threshold is enforced.
  double? get dragStartDistanceMotionThreshold => parent?.dragStartDistanceMotionThreshold;

  /// Whether a viewport is allowed to change its scroll position implicitly in
  /// response to a call to [RenderObject.showOnScreen].
  ///
  /// [RenderObject.showOnScreen] is for example used to bring a text field
  /// fully on screen after it has received focus. This property controls
  /// whether the viewport associated with this object is allowed to change the
  /// scroll position to fulfill such a request.
  bool get allowImplicitScrolling => true;

  /// Whether a viewport is allowed to change the scroll position as the result of user input.
  bool get allowUserScrolling => true;

  @override
  String toString() {
    if (parent == null) {
      return objectRuntimeType(this, 'ScrollPhysics');
    }
    return '${objectRuntimeType(this, 'ScrollPhysics')} -> $parent';
  }
}

/// Scroll physics that attempt to keep the scroll position in range when the
/// contents change dimensions suddenly.
///
/// This attempts to maintain the amount of overscroll or underscroll already present,
/// if the scroll position is already out of range _and_ the extents
/// have decreased, meaning that some content was removed. The reason for this
/// condition is that when new content is added, keeping the same overscroll
/// would mean that instead of showing it to the user, all of it is
/// being skipped by jumping right to the max extent.
///
/// If the scroll activity is animating the scroll position, sudden changes to
/// the scroll dimensions are allowed to happen (so as to prevent animations
/// from jumping back and forth between in-range and out-of-range values).
///
/// These physics should be combined with other scroll physics, e.g.
/// [BouncingScrollPhysics] or [ClampingScrollPhysics], to obtain a complete
/// description of typical scroll physics. See [applyTo].
///
/// ## Implementation details
///
/// Specifically, these physics perform two adjustments.
///
/// The first is to maintain overscroll when the position is out of range.
///
/// The second is to enforce the boundary when the position is in range.
///
/// If the current velocity is non-zero, neither adjustment is made. The
/// assumption is that there is an ongoing animation and therefore
/// further changing the scroll position would disrupt the experience.
///
/// If the extents haven't changed, then the overscroll adjustment is
/// not made. The assumption is that if the position is overscrolled,
/// it is intentional, otherwise the position could not have reached
/// that position. (Consider [ClampingScrollPhysics] vs
/// [BouncingScrollPhysics] for example.)
///
/// If the position itself changed since the last animation frame,
/// then the overscroll is not maintained. The assumption is similar
/// to the previous case: the position would not have been placed out
/// of range unless it was intentional.
///
/// In addition, if the position changed and the boundaries were and
/// still are finite, then the boundary isn't enforced either, for
/// the same reason. However, if any of the boundaries were or are
/// now infinite, the boundary _is_ enforced, on the assumption that
/// infinite boundaries indicate a lazy-loading scroll view, which
/// cannot enforce boundaries while the full list has not loaded.
///
/// If the range was out of range, then the boundary is not enforced
/// even if the range is not maintained. If the range is maintained,
/// then the distance between the old position and the old boundary is
/// applied to the new boundary to obtain the new position.
///
/// If the range was in range, and the boundary is to be enforced,
/// then the new position is obtained by deferring to the other physics,
/// if any, and then clamped to the new range.
class RangeMaintainingScrollPhysics extends ScrollPhysics {
  /// Creates scroll physics that maintain the scroll position in range.
  const RangeMaintainingScrollPhysics({super.parent});

  @override
  RangeMaintainingScrollPhysics applyTo(ScrollPhysics? ancestor) {
    return RangeMaintainingScrollPhysics(parent: buildParent(ancestor));
  }

  @override
  double adjustPositionForNewDimensions({
    required ScrollMetrics oldPosition,
    required ScrollMetrics newPosition,
    required bool isScrolling,
    required double velocity,
  }) {
    bool maintainOverscroll = true;
    bool enforceBoundary = true;
    if (velocity != 0.0) {
      // Don't try to adjust an animating position, the jumping around
      // would be distracting.
      maintainOverscroll = false;
      enforceBoundary = false;
    }
    if ((oldPosition.minScrollExtent == newPosition.minScrollExtent) &&
        (oldPosition.maxScrollExtent == newPosition.maxScrollExtent)) {
      // If the extents haven't changed then ignore overscroll.
      maintainOverscroll = false;
    }
    if (oldPosition.pixels != newPosition.pixels) {
      // If the position has been changed already, then it might have
      // been adjusted to expect new overscroll, so don't try to
      // maintain the relative overscroll.
      maintainOverscroll = false;
      if (oldPosition.minScrollExtent.isFinite &&
          oldPosition.maxScrollExtent.isFinite &&
          newPosition.minScrollExtent.isFinite &&
          newPosition.maxScrollExtent.isFinite) {
        // In addition, if the position changed then we don't enforce the new
        // boundary if both the new and previous boundaries are entirely finite.
        // A common case where the position changes while one
        // of the extents is infinite is a lazily-loaded list. (If the
        // boundaries were finite, and the position changed, then we
        // assume it was intentional.)
        enforceBoundary = false;
      }
    }
    if ((oldPosition.pixels < oldPosition.minScrollExtent) ||
        (oldPosition.pixels > oldPosition.maxScrollExtent)) {
      // If the old position was out of range, then we should
      // not try to keep the new position in range.
      enforceBoundary = false;
    }
    if (maintainOverscroll) {
      // Force the new position to be no more out of range than it was before, if:
      //  * it was overscrolled, and
      //  * the extents have decreased, meaning that some content was removed. The
      //    reason for this condition is that when new content is added, keeping
      //    the same overscroll would mean that instead of showing it to the user,
      //    all of it is being skipped by jumping right to the max extent.
      if (oldPosition.pixels < oldPosition.minScrollExtent &&
          newPosition.minScrollExtent > oldPosition.minScrollExtent) {
        final double oldDelta = oldPosition.minScrollExtent - oldPosition.pixels;
        return newPosition.minScrollExtent - oldDelta;
      }
      if (oldPosition.pixels > oldPosition.maxScrollExtent &&
          newPosition.maxScrollExtent < oldPosition.maxScrollExtent) {
        final double oldDelta = oldPosition.pixels - oldPosition.maxScrollExtent;
        return newPosition.maxScrollExtent + oldDelta;
      }
    }
    // If we're not forcing the overscroll, defer to other physics.
    double result = super.adjustPositionForNewDimensions(
      oldPosition: oldPosition,
      newPosition: newPosition,
      isScrolling: isScrolling,
      velocity: velocity,
    );
    if (enforceBoundary) {
      // ...but if they put us out of range then reinforce the boundary.
      result = clampDouble(result, newPosition.minScrollExtent, newPosition.maxScrollExtent);
    }
    return result;
  }
}

/// Scroll physics for environments that allow the scroll offset to go beyond
/// the bounds of the content, but then bounce the content back to the edge of
/// those bounds.
///
/// This is the behavior typically seen on iOS.
///
/// [BouncingScrollPhysics] by itself will not create an overscroll effect if
/// the contents of the scroll view do not extend beyond the size of the
/// viewport. To create the overscroll and bounce effect regardless of the
/// length of your scroll view, combine with [AlwaysScrollableScrollPhysics].
///
/// {@tool snippet}
/// ```dart
/// const BouncingScrollPhysics(parent: AlwaysScrollableScrollPhysics())
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [ScrollConfiguration], which uses this to provide the default
///    scroll behavior on iOS.
///  * [ClampingScrollPhysics], which is the analogous physics for Android's
///    clamping behavior.
///  * [ScrollPhysics], for more examples of combining [ScrollPhysics] objects
///    of different types to get the desired scroll physics.
class BouncingScrollPhysics extends ScrollPhysics {
  /// Creates scroll physics that bounce back from the edge.
  const BouncingScrollPhysics({
    this.decelerationRate = ScrollDecelerationRate.normal,
    super.parent,
  });

  /// Used to determine parameters for friction simulations.
  final ScrollDecelerationRate decelerationRate;

  @override
  BouncingScrollPhysics applyTo(ScrollPhysics? ancestor) {
    return BouncingScrollPhysics(parent: buildParent(ancestor), decelerationRate: decelerationRate);
  }

  /// The multiple applied to overscroll to make it appear that scrolling past
  /// the edge of the scrollable contents is harder than scrolling the list.
  /// This is done by reducing the ratio of the scroll effect output vs the
  /// scroll gesture input.
  ///
  /// This factor starts at 0.52 and progressively becomes harder to overscroll
  /// as more of the area past the edge is dragged in (represented by an increasing
  /// `overscrollFraction` which starts at 0 when there is no overscroll).
  double frictionFactor(double overscrollFraction) {
    return math.pow(1 - overscrollFraction, 2) *
        switch (decelerationRate) {
          ScrollDecelerationRate.fast => 0.26,
          ScrollDecelerationRate.normal => 0.52,
        };
  }

  @override
  double applyPhysicsToUserOffset(ScrollMetrics position, double offset) {
    assert(offset != 0.0);
    assert(position.minScrollExtent <= position.maxScrollExtent);

    if (!position.outOfRange) {
      return offset;
    }

    final double overscrollPastStart = math.max(position.minScrollExtent - position.pixels, 0.0);
    final double overscrollPastEnd = math.max(position.pixels - position.maxScrollExtent, 0.0);
    final double overscrollPast = math.max(overscrollPastStart, overscrollPastEnd);
    final bool easing =
        (overscrollPastStart > 0.0 && offset < 0.0) || (overscrollPastEnd > 0.0 && offset > 0.0);

    final double friction =
        easing
            // Apply less resistance when easing the overscroll vs tensioning.
            ? frictionFactor((overscrollPast - offset.abs()) / position.viewportDimension)
            : frictionFactor(overscrollPast / position.viewportDimension);
    final double direction = offset.sign;

    if (easing && decelerationRate == ScrollDecelerationRate.fast) {
      return direction * offset.abs();
    }
    return direction * _applyFriction(overscrollPast, offset.abs(), friction);
  }

  static double _applyFriction(double extentOutside, double absDelta, double gamma) {
    assert(absDelta > 0);
    double total = 0.0;
    if (extentOutside > 0) {
      final double deltaToLimit = extentOutside / gamma;
      if (absDelta < deltaToLimit) {
        return absDelta * gamma;
      }
      total += extentOutside;
      absDelta -= deltaToLimit;
    }
    return total + absDelta;
  }

  @override
  double applyBoundaryConditions(ScrollMetrics position, double value) => 0.0;

  @override
  Simulation? createBallisticSimulation(ScrollMetrics position, double velocity) {
    final Tolerance tolerance = toleranceFor(position);
    if (velocity.abs() >= tolerance.velocity || position.outOfRange) {
      return BouncingScrollSimulation(
        spring: spring,
        position: position.pixels,
        velocity: velocity,
        leadingExtent: position.minScrollExtent,
        trailingExtent: position.maxScrollExtent,
        tolerance: tolerance,
        constantDeceleration: switch (decelerationRate) {
          ScrollDecelerationRate.fast => 1400,
          ScrollDecelerationRate.normal => 0,
        },
      );
    }
    return null;
  }

  // The ballistic simulation here decelerates more slowly than the one for
  // ClampingScrollPhysics so we require a more deliberate input gesture
  // to trigger a fling.
  @override
  double get minFlingVelocity => kMinFlingVelocity * 2.0;

  // Methodology:
  // 1- Use https://github.com/flutter/platform_tests/tree/master/scroll_overlay to test with
  //    Flutter and platform scroll views superimposed.
  // 3- If the scrollables stopped overlapping at any moment, adjust the desired
  //    output value of this function at that input speed.
  // 4- Feed new input/output set into a power curve fitter. Change function
  //    and repeat from 2.
  // 5- Repeat from 2 with medium and slow flings.
  /// Momentum build-up function that mimics iOS's scroll speed increase with repeated flings.
  ///
  /// The velocity of the last fling is not an important factor. Existing speed
  /// and (related) time since last fling are factors for the velocity transfer
  /// calculations.
  @override
  double carriedMomentum(double existingVelocity) {
    return existingVelocity.sign *
        math.min(0.000816 * math.pow(existingVelocity.abs(), 1.967).toDouble(), 40000.0);
  }

  // Eyeballed from observation to counter the effect of an unintended scroll
  // from the natural motion of lifting the finger after a scroll.
  @override
  double get dragStartDistanceMotionThreshold => 3.5;

  @override
  double get maxFlingVelocity => switch (decelerationRate) {
    ScrollDecelerationRate.fast => kMaxFlingVelocity * 8.0,
    ScrollDecelerationRate.normal => super.maxFlingVelocity,
  };

  @override
  SpringDescription get spring {
    switch (decelerationRate) {
      case ScrollDecelerationRate.fast:
        return SpringDescription.withDampingRatio(mass: 0.3, stiffness: 75.0, ratio: 1.3);
      case ScrollDecelerationRate.normal:
        return super.spring;
    }
  }
}

/// Scroll physics for environments that prevent the scroll offset from reaching
/// beyond the bounds of the content.
///
/// This is the behavior typically seen on Android.
///
/// See also:
///
///  * [ScrollConfiguration], which uses this to provide the default
///    scroll behavior on Android.
///  * [BouncingScrollPhysics], which is the analogous physics for iOS' bouncing
///    behavior.
///  * [GlowingOverscrollIndicator], which is used by [ScrollConfiguration] to
///    provide the glowing effect that is usually found with this clamping effect
///    on Android. When using a [MaterialApp], the [GlowingOverscrollIndicator]'s
///    glow color is specified to use the overall theme's
///    [ColorScheme.secondary] color.
class ClampingScrollPhysics extends ScrollPhysics {
  /// Creates scroll physics that prevent the scroll offset from exceeding the
  /// bounds of the content.
  const ClampingScrollPhysics({super.parent});

  @override
  ClampingScrollPhysics applyTo(ScrollPhysics? ancestor) {
    return ClampingScrollPhysics(parent: buildParent(ancestor));
  }

  @override
  double applyBoundaryConditions(ScrollMetrics position, double value) {
    assert(() {
      if (value == position.pixels) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('$runtimeType.applyBoundaryConditions() was called redundantly.'),
          ErrorDescription(
            'The proposed new position, $value, is exactly equal to the current position of the '
            'given ${position.runtimeType}, ${position.pixels}.\n'
            'The applyBoundaryConditions method should only be called when the value is '
            'going to actually change the pixels, otherwise it is redundant.',
          ),
          DiagnosticsProperty<ScrollPhysics>(
            'The physics object in question was',
            this,
            style: DiagnosticsTreeStyle.errorProperty,
          ),
          DiagnosticsProperty<ScrollMetrics>(
            'The position object in question was',
            position,
            style: DiagnosticsTreeStyle.errorProperty,
          ),
        ]);
      }
      return true;
    }());
    if (value < position.pixels && position.pixels <= position.minScrollExtent) {
      // Underscroll.
      return value - position.pixels;
    }
    if (position.maxScrollExtent <= position.pixels && position.pixels < value) {
      // Overscroll.
      return value - position.pixels;
    }
    if (value < position.minScrollExtent && position.minScrollExtent < position.pixels) {
      // Hit top edge.
      return value - position.minScrollExtent;
    }
    if (position.pixels < position.maxScrollExtent && position.maxScrollExtent < value) {
      // Hit bottom edge.
      return value - position.maxScrollExtent;
    }
    return 0.0;
  }

  @override
  Simulation? createBallisticSimulation(ScrollMetrics position, double velocity) {
    final Tolerance tolerance = toleranceFor(position);
    if (position.outOfRange) {
      double? end;
      if (position.pixels > position.maxScrollExtent) {
        end = position.maxScrollExtent;
      }
      if (position.pixels < position.minScrollExtent) {
        end = position.minScrollExtent;
      }
      assert(end != null);
      return ScrollSpringSimulation(
        spring,
        position.pixels,
        end!,
        math.min(0.0, velocity),
        tolerance: tolerance,
      );
    }
    if (velocity.abs() < tolerance.velocity) {
      return null;
    }
    if (velocity > 0.0 && position.pixels >= position.maxScrollExtent) {
      return null;
    }
    if (velocity < 0.0 && position.pixels <= position.minScrollExtent) {
      return null;
    }
    return ClampingScrollSimulation(
      position: position.pixels,
      velocity: velocity,
      tolerance: tolerance,
    );
  }
}

/// Scroll physics that always lets the user scroll.
///
/// This overrides the default behavior which is to disable scrolling
/// when there is no content to scroll. It does not override the
/// handling of overscrolling.
///
/// On Android, overscrolls will be clamped by default and result in an
/// overscroll glow. On iOS, overscrolls will load a spring that will return the
/// scroll view to its normal range when released.
///
/// See also:
///
///  * [ScrollPhysics], which can be used instead of this class when the default
///    behavior is desired instead.
///  * [BouncingScrollPhysics], which provides the bouncing overscroll behavior
///    found on iOS.
///  * [ClampingScrollPhysics], which provides the clamping overscroll behavior
///    found on Android.
class AlwaysScrollableScrollPhysics extends ScrollPhysics {
  /// Creates scroll physics that always lets the user scroll.
  const AlwaysScrollableScrollPhysics({super.parent});

  @override
  AlwaysScrollableScrollPhysics applyTo(ScrollPhysics? ancestor) {
    return AlwaysScrollableScrollPhysics(parent: buildParent(ancestor));
  }

  @override
  bool shouldAcceptUserOffset(ScrollMetrics position) => true;
}

/// Scroll physics that does not allow the user to scroll.
///
/// See also:
///
///  * [ScrollPhysics], which can be used instead of this class when the default
///    behavior is desired instead.
///  * [BouncingScrollPhysics], which provides the bouncing overscroll behavior
///    found on iOS.
///  * [ClampingScrollPhysics], which provides the clamping overscroll behavior
///    found on Android.
class NeverScrollableScrollPhysics extends ScrollPhysics {
  /// Creates scroll physics that does not let the user scroll.
  const NeverScrollableScrollPhysics({super.parent});

  @override
  NeverScrollableScrollPhysics applyTo(ScrollPhysics? ancestor) {
    return NeverScrollableScrollPhysics(parent: buildParent(ancestor));
  }

  @override
  bool get allowUserScrolling => false;

  @override
  bool get allowImplicitScrolling => false;
}
