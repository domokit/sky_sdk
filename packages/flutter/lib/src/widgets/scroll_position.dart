// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'page_view.dart';
/// @docImport 'scroll_controller.dart';
/// @docImport 'scroll_notification_observer.dart';
/// @docImport 'scroll_position_with_single_context.dart';
/// @docImport 'scroll_view.dart';
/// @docImport 'scrollable.dart';
/// @docImport 'viewport.dart';
library;

import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/physics.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';

import 'basic.dart';
import 'framework.dart';
import 'notification_listener.dart';
import 'page_storage.dart';
import 'scroll_activity.dart';
import 'scroll_context.dart';
import 'scroll_metrics.dart';
import 'scroll_notification.dart';
import 'scroll_physics.dart';

export 'scroll_activity.dart' show ScrollHoldController;

/// The policy to use when applying the `alignment` parameter of
/// [ScrollPosition.ensureVisible].
enum ScrollPositionAlignmentPolicy {
  /// Use the `alignment` property of [ScrollPosition.ensureVisible] to decide
  /// where to align the visible object.
  explicit,

  /// Find the bottom edge of the scroll container, and scroll the container, if
  /// necessary, to show the bottom of the object.
  ///
  /// For example, find the bottom edge of the scroll container. If the bottom
  /// edge of the item is below the bottom edge of the scroll container, scroll
  /// the item so that the bottom of the item is just visible. If the entire
  /// item is already visible, then do nothing.
  keepVisibleAtEnd,

  /// Find the top edge of the scroll container, and scroll the container if
  /// necessary to show the top of the object.
  ///
  /// For example, find the top edge of the scroll container. If the top edge of
  /// the item is above the top edge of the scroll container, scroll the item so
  /// that the top of the item is just visible. If the entire item is already
  /// visible, then do nothing.
  keepVisibleAtStart,
}

/// Determines which portion of the content is visible in a scroll view.
///
/// The [pixels] value determines the scroll offset that the scroll view uses to
/// select which part of its content to display. As the user scrolls the
/// viewport, this value changes, which changes the content that is displayed.
///
/// The [ScrollPosition] applies [physics] to scrolling, and stores the
/// [minScrollExtent] and [maxScrollExtent].
///
/// Scrolling is controlled by the current [activity], which is set by
/// [beginActivity]. [ScrollPosition] itself does not start any activities.
/// Instead, concrete subclasses, such as [ScrollPositionWithSingleContext],
/// typically start activities in response to user input or instructions from a
/// [ScrollController].
///
/// This object is a [Listenable] that notifies its listeners when [pixels]
/// changes.
///
/// {@template flutter.widgets.scrollPosition.listening}
/// ### Accessing Scrolling Information
///
/// There are several ways to acquire information about scrolling and
/// scrollable widgets, but each provides different types of information about
/// the scrolling activity, the position, and the dimensions of the [Viewport].
///
/// A [ScrollController] is a [Listenable]. It notifies its listeners whenever
/// any of the attached [ScrollPosition]s notify _their_ listeners, such as when
/// scrolling occurs. This is very similar to using a [NotificationListener] of
/// type [ScrollNotification] to listen to changes in the scroll position, with
/// the difference being that a notification listener will provide information
/// about the scrolling activity. A notification listener can further listen to
/// specific subclasses of [ScrollNotification], like [UserScrollNotification].
///
/// {@tool dartpad}
/// This sample shows the difference between using a [ScrollController] or a
/// [NotificationListener] of type [ScrollNotification] to listen to scrolling
/// activities. Toggling the [Radio] button switches between the two.
/// Using a [ScrollNotification] will provide details about the scrolling
/// activity, along with the metrics of the [ScrollPosition], but not the scroll
/// position object itself. By listening with a [ScrollController], the position
/// object is directly accessible.
/// Both of these types of notifications are only triggered by scrolling.
///
/// ** See code in examples/api/lib/widgets/scroll_position/scroll_controller_notification.0.dart **
/// {@end-tool}
///
/// [ScrollController] does not notify its listeners when the list of
/// [ScrollPosition]s attached to the scroll controller changes. To listen to
/// the attaching and detaching of scroll positions to the controller, use the
/// [ScrollController.onAttach] and [ScrollController.onDetach] methods. This is
/// also useful for adding a listener to the
/// [ScrollPosition.isScrollingNotifier] when the position is created during the
/// build method of the [Scrollable].
///
/// At the time that a scroll position is attached, the [ScrollMetrics], such as
/// the [ScrollMetrics.maxScrollExtent], are not yet available. These are not
/// determined until the [Scrollable] has finished laying out its contents and
/// computing things like the full extent of that content.
/// [ScrollPosition.hasContentDimensions] can be used to know when the
/// metrics are available, or a [ScrollMetricsNotification] can be used,
/// discussed further below.
///
/// {@tool dartpad}
/// This sample shows how to apply a listener to the
/// [ScrollPosition.isScrollingNotifier] using [ScrollController.onAttach].
/// This is used to change the [AppBar]'s color when scrolling is occurring.
///
/// ** See code in examples/api/lib/widgets/scroll_position/scroll_controller_on_attach.0.dart **
/// {@end-tool}
///
/// #### From a different context
///
/// When needing to access scrolling information from a context that is within
/// the scrolling widget itself, use [Scrollable.of] to access the
/// [ScrollableState] and the [ScrollableState.position]. This would be the same
/// [ScrollPosition] attached to a [ScrollController].
///
/// When needing to access scrolling information from a context that is not an
/// ancestor of the scrolling widget, use [ScrollNotificationObserver]. This is
/// used by [AppBar] to create the scrolled under effect. Since [Scaffold.appBar]
/// is a separate subtree from the [Scaffold.body], scroll notifications would
/// not bubble up to the app bar. Use
/// [ScrollNotificationObserverState.addListener] to listen to scroll
/// notifications happening outside of the current context.
///
/// #### Dimension changes
///
/// Lastly, listening to a [ScrollController] or a [ScrollPosition] will
/// _not_ notify when the [ScrollMetrics] of a given scroll position changes,
/// such as when the window is resized, changing the dimensions of the
/// [Viewport] and the previously mentioned extents of the scrollable. In order
/// to listen to changes in scroll metrics, use a [NotificationListener] of type
/// [ScrollMetricsNotification]. This type of notification differs from
/// [ScrollNotification], as it is not associated with the activity of
/// scrolling, but rather the dimensions of the scrollable area, such as the
/// window size.
///
/// {@tool dartpad}
/// This sample shows how a [ScrollMetricsNotification] is dispatched when
/// the `windowSize` is changed. Press the floating action button to increase
/// the scrollable window's size.
///
/// ** See code in examples/api/lib/widgets/scroll_position/scroll_metrics_notification.0.dart **
/// {@end-tool}
/// {@endtemplate}
///
/// ## Subclassing ScrollPosition
///
/// Over time, a [Scrollable] might have many different [ScrollPosition]
/// objects. For example, if [Scrollable.physics] changes type, [Scrollable]
/// creates a new [ScrollPosition] with the new physics. To transfer state from
/// the old instance to the new instance, subclasses implement [absorb]. See
/// [absorb] for more details.
///
/// Subclasses also need to call [didUpdateScrollDirection] whenever
/// [userScrollDirection] changes values.
///
/// See also:
///
///  * [Scrollable], which uses a [ScrollPosition] to determine which portion of
///    its content to display.
///  * [ScrollController], which can be used with [ListView], [GridView] and
///    other scrollable widgets to control a [ScrollPosition].
///  * [ScrollPositionWithSingleContext], which is the most commonly used
///    concrete subclass of [ScrollPosition].
///  * [ScrollNotification] and [NotificationListener], which can be used to watch
///    the scroll position without using a [ScrollController].
abstract class ScrollPosition extends ViewportOffset with ScrollMetrics {
  /// Creates an object that determines which portion of the content is visible
  /// in a scroll view.
  ScrollPosition({
    required this.physics,
    required this.context,
    this.keepScrollOffset = true,
    ScrollPosition? oldPosition,
    this.debugLabel,
  }) {
    if (oldPosition != null) {
      absorb(oldPosition);
    }
    if (keepScrollOffset) {
      restoreScrollOffset();
    }
  }

  /// How the scroll position should respond to user input.
  ///
  /// For example, determines how the widget continues to animate after the
  /// user stops dragging the scroll view.
  final ScrollPhysics physics;

  /// Where the scrolling is taking place.
  ///
  /// Typically implemented by [ScrollableState].
  final ScrollContext context;

  /// Save the current scroll offset with [PageStorage] and restore it if
  /// this scroll position's scrollable is recreated.
  ///
  /// See also:
  ///
  ///  * [ScrollController.keepScrollOffset] and [PageController.keepPage], which
  ///    create scroll positions and initialize this property.
  // TODO(goderbauer): Deprecate this when state restoration supports all features of PageStorage.
  final bool keepScrollOffset;

  /// A label that is used in the [toString] output.
  ///
  /// Intended to aid with identifying animation controller instances in debug
  /// output.
  final String? debugLabel;

  @override
  double get minScrollExtent => _minScrollExtent!;
  double? _minScrollExtent;

  @override
  double get maxScrollExtent => _maxScrollExtent!;
  double? _maxScrollExtent;

  @override
  bool get hasContentDimensions => _minScrollExtent != null && _maxScrollExtent != null;

  /// The additional velocity added for a [forcePixels] change in a single
  /// frame.
  ///
  /// This value is used by [recommendDeferredLoading] in addition to the
  /// [activity]'s [ScrollActivity.velocity] to ask the [physics] whether or
  /// not to defer loading. It accounts for the fact that a [forcePixels] call
  /// may involve a [ScrollActivity] with 0 velocity, but the scrollable is
  /// still instantaneously moving from its current position to a potentially
  /// very far position, and which is of interest to callers of
  /// [recommendDeferredLoading].
  ///
  /// For example, if a scrollable is currently at 5000 pixels, and we [jumpTo]
  /// 0 to get back to the top of the list, we would have an implied velocity of
  /// -5000 and an `activity.velocity` of 0. The jump may be going past a
  /// number of resource intensive widgets which should avoid doing work if the
  /// position jumps past them.
  double _impliedVelocity = 0;

  @override
  double get pixels => _pixels!;
  double? _pixels;

  @override
  bool get hasPixels => _pixels != null;

  @override
  double get viewportDimension => _viewportDimension!;
  double? _viewportDimension;

  @override
  bool get hasViewportDimension => _viewportDimension != null;

  /// Whether [viewportDimension], [minScrollExtent], [maxScrollExtent],
  /// [outOfRange], and [atEdge] are available.
  ///
  /// Set to true just before the first time [applyNewDimensions] is called.
  bool get haveDimensions => _haveDimensions;
  bool _haveDimensions = false;

  /// Whether scrollables should absorb pointer events at this position.
  ///
  /// This is value relates to the current [ScrollActivity], which determines
  /// if additional touch input should be received by the scroll view or its children.
  /// If the position is overscrolled, as is allowed by [BouncingScrollPhysics],
  /// children of the scroll view will receive pointer events as the scroll view
  /// settles back from the overscrolled state.
  bool get shouldIgnorePointer => !outOfRange && (activity?.shouldIgnorePointer ?? true);

  /// Take any current applicable state from the given [ScrollPosition].
  ///
  /// This method is called by the constructor if it is given an `oldPosition`.
  /// The `other` argument might not have the same [runtimeType] as this object.
  ///
  /// This method can be destructive to the other [ScrollPosition]. The other
  /// object must be disposed immediately after this call (in the same call
  /// stack, before microtask resolution, by whomever called this object's
  /// constructor).
  ///
  /// If the old [ScrollPosition] object is a different [runtimeType] than this
  /// one, the [ScrollActivity.resetActivity] method is invoked on the newly
  /// adopted [ScrollActivity].
  ///
  /// ## Overriding
  ///
  /// Overrides of this method must call `super.absorb` after setting any
  /// metrics-related or activity-related state, since this method may restart
  /// the activity and scroll activities tend to use those metrics when being
  /// restarted.
  ///
  /// Overrides of this method might need to start an [IdleScrollActivity] if
  /// they are unable to absorb the activity from the other [ScrollPosition].
  ///
  /// Overrides of this method might also need to update the delegates of
  /// absorbed scroll activities if they use themselves as a
  /// [ScrollActivityDelegate].
  @protected
  @mustCallSuper
  void absorb(ScrollPosition other) {
    assert(other.context == context);
    assert(_pixels == null);
    if (other.hasContentDimensions) {
      _minScrollExtent = other.minScrollExtent;
      _maxScrollExtent = other.maxScrollExtent;
    }
    if (other.hasPixels) {
      _pixels = other.pixels;
    }
    if (other.hasViewportDimension) {
      _viewportDimension = other.viewportDimension;
    }

    assert(activity == null);
    assert(other.activity != null);
    _activity = other.activity;
    other._activity = null;
    if (other.runtimeType != runtimeType) {
      activity!.resetActivity();
    }
    context.setIgnorePointer(activity!.shouldIgnorePointer);
    isScrollingNotifier.value = activity!.isScrolling;
  }

  @override
  double get devicePixelRatio => context.devicePixelRatio;

  /// Update the scroll position ([pixels]) to a given pixel value.
  ///
  /// This should only be called by the current [ScrollActivity], either during
  /// the transient callback phase or in response to user input.
  ///
  /// Returns the overscroll, if any. If the return value is 0.0, that means
  /// that [pixels] now returns the given `value`. If the return value is
  /// positive, then [pixels] is less than the requested `value` by the given
  /// amount (overscroll past the max extent), and if it is negative, it is
  /// greater than the requested `value` by the given amount (underscroll past
  /// the min extent).
  ///
  /// The amount of overscroll is computed by [applyBoundaryConditions].
  ///
  /// The amount of the change that is applied is reported using [didUpdateScrollPositionBy].
  /// If there is any overscroll, it is reported using [didOverscrollBy].
  double setPixels(double newPixels) {
    assert(hasPixels);
    assert(
      SchedulerBinding.instance.schedulerPhase != SchedulerPhase.persistentCallbacks,
      "A scrollable's position should not change during the build, layout, and paint phases, otherwise the rendering will be confused.",
    );
    if (newPixels != pixels) {
      final double overscroll = applyBoundaryConditions(newPixels);
      assert(() {
        final double delta = newPixels - pixels;
        if (overscroll.abs() > delta.abs()) {
          throw FlutterError(
            '$runtimeType.applyBoundaryConditions returned invalid overscroll value.\n'
            'setPixels() was called to change the scroll offset from $pixels to $newPixels.\n'
            'That is a delta of $delta units.\n'
            '$runtimeType.applyBoundaryConditions reported an overscroll of $overscroll units.',
          );
        }
        return true;
      }());
      final double oldPixels = pixels;
      _pixels = newPixels - overscroll;
      if (_pixels != oldPixels) {
        if (outOfRange) {
          context.setIgnorePointer(false);
        }
        notifyListeners();
        didUpdateScrollPositionBy(pixels - oldPixels);
      }
      if (overscroll.abs() > precisionErrorTolerance) {
        didOverscrollBy(overscroll);
        return overscroll;
      }
    }
    return 0.0;
  }

  /// Change the value of [pixels] to the new value, without notifying any
  /// customers.
  ///
  /// This is used to adjust the position while doing layout. In particular,
  /// this is typically called as a response to [applyViewportDimension] or
  /// [applyContentDimensions] (in both cases, if this method is called, those
  /// methods should then return false to indicate that the position has been
  /// adjusted).
  ///
  /// Calling this is rarely correct in other contexts. It will not immediately
  /// cause the rendering to change, since it does not notify the widgets or
  /// render objects that might be listening to this object: they will only
  /// change when they next read the value, which could be arbitrarily later. It
  /// is generally only appropriate in the very specific case of the value being
  /// corrected during layout (since then the value is immediately read), in the
  /// specific case of a [ScrollPosition] with a single viewport customer.
  ///
  /// To cause the position to jump or animate to a new value, consider [jumpTo]
  /// or [animateTo], which will honor the normal conventions for changing the
  /// scroll offset.
  ///
  /// To force the [pixels] to a particular value without honoring the normal
  /// conventions for changing the scroll offset, consider [forcePixels]. (But
  /// see the discussion there for why that might still be a bad idea.)
  ///
  /// See also:
  ///
  ///  * [correctBy], which is a method of [ViewportOffset] used
  ///    by viewport render objects to correct the offset during layout
  ///    without notifying its listeners.
  ///  * [jumpTo], for making changes to position while not in the
  ///    middle of layout and applying the new position immediately.
  ///  * [animateTo], which is like [jumpTo] but animating to the
  ///    destination offset.
  // ignore: use_setters_to_change_properties, (API is intended to discourage setting value)
  void correctPixels(double value) {
    _pixels = value;
  }

  /// Apply a layout-time correction to the scroll offset.
  ///
  /// This method should change the [pixels] value by `correction`, but without
  /// calling [notifyListeners]. It is called during layout by the
  /// [RenderViewport], before [applyContentDimensions]. After this method is
  /// called, the layout will be recomputed and that may result in this method
  /// being called again, though this should be very rare.
  ///
  /// See also:
  ///
  ///  * [jumpTo], for also changing the scroll position when not in layout.
  ///    [jumpTo] applies the change immediately and notifies its listeners.
  ///  * [correctPixels], which is used by the [ScrollPosition] itself to
  ///    set the offset initially during construction or after
  ///    [applyViewportDimension] or [applyContentDimensions] is called.
  @override
  void correctBy(double correction) {
    assert(
      hasPixels,
      'An initial pixels value must exist by calling correctPixels on the ScrollPosition',
    );
    _pixels = _pixels! + correction;
    _didChangeViewportDimensionOrReceiveCorrection = true;
  }

  /// Change the value of [pixels] to the new value, and notify any customers,
  /// but without honoring normal conventions for changing the scroll offset.
  ///
  /// This is used to implement [jumpTo]. It can also be used adjust the
  /// position when the dimensions of the viewport change. It should only be
  /// used when manually implementing the logic for honoring the relevant
  /// conventions of the class. For example, [ScrollPositionWithSingleContext]
  /// introduces [ScrollActivity] objects and uses [forcePixels] in conjunction
  /// with adjusting the activity, e.g. by calling
  /// [ScrollPositionWithSingleContext.goIdle], so that the activity does
  /// not immediately set the value back. (Consider, for instance, a case where
  /// one is using a [DrivenScrollActivity]. That object will ignore any calls
  /// to [forcePixels], which would result in the rendering stuttering: changing
  /// in response to [forcePixels], and then changing back to the next value
  /// derived from the animation.)
  ///
  /// To cause the position to jump or animate to a new value, consider [jumpTo]
  /// or [animateTo].
  ///
  /// This should not be called during layout (e.g. when setting the initial
  /// scroll offset). Consider [correctPixels] if you find you need to adjust
  /// the position during layout.
  @protected
  void forcePixels(double value) {
    assert(hasPixels);
    _impliedVelocity = value - pixels;
    _pixels = value;
    notifyListeners();
    SchedulerBinding.instance.addPostFrameCallback((Duration timeStamp) {
      _impliedVelocity = 0;
    }, debugLabel: 'ScrollPosition.resetVelocity');
  }

  /// Called whenever scrolling ends, to store the current scroll offset in a
  /// storage mechanism with a lifetime that matches the app's lifetime.
  ///
  /// The stored value will be used by [restoreScrollOffset] when the
  /// [ScrollPosition] is recreated, in the case of the [Scrollable] being
  /// disposed then recreated in the same session. This might happen, for
  /// instance, if a [ListView] is on one of the pages inside a [TabBarView],
  /// and that page is displayed, then hidden, then displayed again.
  ///
  /// The default implementation writes the [pixels] using the nearest
  /// [PageStorage] found from the [context]'s [ScrollContext.storageContext]
  /// property.
  // TODO(goderbauer): Deprecate this when state restoration supports all features of PageStorage.
  @protected
  void saveScrollOffset() {
    PageStorage.maybeOf(context.storageContext)?.writeState(context.storageContext, pixels);
  }

  /// Called whenever the [ScrollPosition] is created, to restore the scroll
  /// offset if possible.
  ///
  /// The value is stored by [saveScrollOffset] when the scroll position
  /// changes, so that it can be restored in the case of the [Scrollable] being
  /// disposed then recreated in the same session. This might happen, for
  /// instance, if a [ListView] is on one of the pages inside a [TabBarView],
  /// and that page is displayed, then hidden, then displayed again.
  ///
  /// The default implementation reads the value from the nearest [PageStorage]
  /// found from the [context]'s [ScrollContext.storageContext] property, and
  /// sets it using [correctPixels], if [pixels] is still null.
  ///
  /// This method is called from the constructor, so layout has not yet
  /// occurred, and the viewport dimensions aren't yet known when it is called.
  // TODO(goderbauer): Deprecate this when state restoration supports all features of PageStorage.
  @protected
  void restoreScrollOffset() {
    if (!hasPixels) {
      final double? value =
          PageStorage.maybeOf(context.storageContext)?.readState(context.storageContext) as double?;
      if (value != null) {
        correctPixels(value);
      }
    }
  }

  /// Called by [context] to restore the scroll offset to the provided value.
  ///
  /// The provided value has previously been provided to the [context] by
  /// calling [ScrollContext.saveOffset], e.g. from [saveOffset].
  ///
  /// This method may be called right after the scroll position is created
  /// before layout has occurred. In that case, `initialRestore` is set to true
  /// and the viewport dimensions will not be known yet. If the [context]
  /// doesn't have any information to restore the scroll offset this method is
  /// not called.
  ///
  /// The method may be called multiple times in the lifecycle of a
  /// [ScrollPosition] to restore it to different scroll offsets.
  void restoreOffset(double offset, {bool initialRestore = false}) {
    if (initialRestore) {
      correctPixels(offset);
    } else {
      jumpTo(offset);
    }
  }

  /// Called whenever scrolling ends, to persist the current scroll offset for
  /// state restoration purposes.
  ///
  /// The default implementation stores the current value of [pixels] on the
  /// [context] by calling [ScrollContext.saveOffset]. At a later point in time
  /// or after the application restarts, the [context] may restore the scroll
  /// position to the persisted offset by calling [restoreOffset].
  @protected
  void saveOffset() {
    assert(hasPixels);
    context.saveOffset(pixels);
  }

  /// Returns the overscroll by applying the boundary conditions.
  ///
  /// If the given value is in bounds, returns 0.0. Otherwise, returns the
  /// amount of value that cannot be applied to [pixels] as a result of the
  /// boundary conditions. If the [physics] allow out-of-bounds scrolling, this
  /// method always returns 0.0.
  ///
  /// The default implementation defers to the [physics] object's
  /// [ScrollPhysics.applyBoundaryConditions].
  @protected
  double applyBoundaryConditions(double value) {
    final double result = physics.applyBoundaryConditions(this, value);
    assert(() {
      final double delta = value - pixels;
      if (result.abs() > delta.abs()) {
        throw FlutterError(
          '${physics.runtimeType}.applyBoundaryConditions returned invalid overscroll value.\n'
          'The method was called to consider a change from $pixels to $value, which is a '
          'delta of ${delta.toStringAsFixed(1)} units. However, it returned an overscroll of '
          '${result.toStringAsFixed(1)} units, which has a greater magnitude than the delta. '
          'The applyBoundaryConditions method is only supposed to reduce the possible range '
          'of movement, not increase it.\n'
          'The scroll extents are $minScrollExtent .. $maxScrollExtent, and the '
          'viewport dimension is $viewportDimension.',
        );
      }
      return true;
    }());
    return result;
  }

  bool _didChangeViewportDimensionOrReceiveCorrection = true;

  @override
  bool applyViewportDimension(double viewportDimension) {
    if (_viewportDimension != viewportDimension) {
      _viewportDimension = viewportDimension;
      _didChangeViewportDimensionOrReceiveCorrection = true;
      // If this is called, you can rely on applyContentDimensions being called
      // soon afterwards in the same layout phase. So we put all the logic that
      // relies on both values being computed into applyContentDimensions.
    }
    return true;
  }

  bool _pendingDimensions = false;
  ScrollMetrics? _lastMetrics;
  // True indicates that there is a ScrollMetrics update notification pending.
  bool _haveScheduledUpdateNotification = false;
  Axis? _lastAxis;

  bool _isMetricsChanged() {
    assert(haveDimensions);
    final ScrollMetrics currentMetrics = copyWith();

    return _lastMetrics == null ||
        !(currentMetrics.extentBefore == _lastMetrics!.extentBefore &&
            currentMetrics.extentInside == _lastMetrics!.extentInside &&
            currentMetrics.extentAfter == _lastMetrics!.extentAfter &&
            currentMetrics.axisDirection == _lastMetrics!.axisDirection);
  }

  @override
  bool applyContentDimensions(double minScrollExtent, double maxScrollExtent) {
    assert(haveDimensions == (_lastMetrics != null));
    if (!nearEqual(_minScrollExtent, minScrollExtent, Tolerance.defaultTolerance.distance) ||
        !nearEqual(_maxScrollExtent, maxScrollExtent, Tolerance.defaultTolerance.distance) ||
        _didChangeViewportDimensionOrReceiveCorrection ||
        _lastAxis != axis) {
      assert(minScrollExtent <= maxScrollExtent);
      _minScrollExtent = minScrollExtent;
      _maxScrollExtent = maxScrollExtent;
      _lastAxis = axis;
      final ScrollMetrics? currentMetrics = haveDimensions ? copyWith() : null;
      _didChangeViewportDimensionOrReceiveCorrection = false;
      _pendingDimensions = true;
      if (haveDimensions && !correctForNewDimensions(_lastMetrics!, currentMetrics!)) {
        return false;
      }
      _haveDimensions = true;
    }
    assert(haveDimensions);
    if (_pendingDimensions) {
      applyNewDimensions();
      _pendingDimensions = false;
    }
    assert(
      !_didChangeViewportDimensionOrReceiveCorrection,
      'Use correctForNewDimensions() (and return true) to change the scroll offset during applyContentDimensions().',
    );

    if (_isMetricsChanged()) {
      // It is too late to send useful notifications, because the potential
      // listeners have, by definition, already been built this frame. To make
      // sure the notification is sent at all, we delay it until after the frame
      // is complete.
      if (!_haveScheduledUpdateNotification) {
        scheduleMicrotask(didUpdateScrollMetrics);
        _haveScheduledUpdateNotification = true;
      }
      _lastMetrics = copyWith();
    }
    return true;
  }

  /// Verifies that the new content and viewport dimensions are acceptable.
  ///
  /// Called by [applyContentDimensions] to determine its return value.
  ///
  /// Should return true if the current scroll offset is correct given
  /// the new content and viewport dimensions.
  ///
  /// Otherwise, should call [correctPixels] to correct the scroll
  /// offset given the new dimensions, and then return false.
  ///
  /// This is only called when [haveDimensions] is true.
  ///
  /// The default implementation defers to [ScrollPhysics.adjustPositionForNewDimensions].
  @protected
  bool correctForNewDimensions(ScrollMetrics oldPosition, ScrollMetrics newPosition) {
    final double newPixels = physics.adjustPositionForNewDimensions(
      oldPosition: oldPosition,
      newPosition: newPosition,
      isScrolling: activity!.isScrolling,
      velocity: activity!.velocity,
    );
    if (newPixels != pixels) {
      correctPixels(newPixels);
      return false;
    }
    return true;
  }

  /// Notifies the activity that the dimensions of the underlying viewport or
  /// contents have changed.
  ///
  /// Called after [applyViewportDimension] or [applyContentDimensions] have
  /// changed the [minScrollExtent], the [maxScrollExtent], or the
  /// [viewportDimension]. When this method is called, it should be called
  /// _after_ any corrections are applied to [pixels] using [correctPixels], not
  /// before.
  ///
  /// The default implementation informs the [activity] of the new dimensions by
  /// calling its [ScrollActivity.applyNewDimensions] method.
  ///
  /// See also:
  ///
  ///  * [applyViewportDimension], which is called when new
  ///    viewport dimensions are established.
  ///  * [applyContentDimensions], which is called after new
  ///    viewport dimensions are established, and also if new content dimensions
  ///    are established, and which calls [ScrollPosition.applyNewDimensions].
  @protected
  @mustCallSuper
  void applyNewDimensions() {
    assert(hasPixels);
    assert(_pendingDimensions);
    activity!.applyNewDimensions();
    _updateSemanticActions(); // will potentially request a semantics update.
  }

  Set<SemanticsAction>? _semanticActions;

  /// Called whenever the scroll position or the dimensions of the scroll view
  /// change to schedule an update of the available semantics actions. The
  /// actual update will be performed in the next frame. If non is pending
  /// a frame will be scheduled.
  ///
  /// For example: If the scroll view has been scrolled all the way to the top,
  /// the action to scroll further up needs to be removed as the scroll view
  /// cannot be scrolled in that direction anymore.
  ///
  /// This method is potentially called twice per frame (if scroll position and
  /// scroll view dimensions both change) and therefore shouldn't do anything
  /// expensive.
  void _updateSemanticActions() {
    final (SemanticsAction forward, SemanticsAction backward) = switch (axisDirection) {
      AxisDirection.up => (SemanticsAction.scrollDown, SemanticsAction.scrollUp),
      AxisDirection.down => (SemanticsAction.scrollUp, SemanticsAction.scrollDown),
      AxisDirection.left => (SemanticsAction.scrollRight, SemanticsAction.scrollLeft),
      AxisDirection.right => (SemanticsAction.scrollLeft, SemanticsAction.scrollRight),
    };

    final Set<SemanticsAction> actions = <SemanticsAction>{
      if (pixels > minScrollExtent) backward,
      if (pixels < maxScrollExtent) forward,
    };

    if (setEquals<SemanticsAction>(actions, _semanticActions)) {
      return;
    }

    _semanticActions = actions;
    context.setSemanticsActions(_semanticActions!);
  }

  ScrollPositionAlignmentPolicy _maybeFlipAlignment(ScrollPositionAlignmentPolicy alignmentPolicy) {
    return switch (alignmentPolicy) {
      // Don't flip when explicit.
      ScrollPositionAlignmentPolicy.explicit => alignmentPolicy,
      ScrollPositionAlignmentPolicy.keepVisibleAtEnd =>
        ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      ScrollPositionAlignmentPolicy.keepVisibleAtStart =>
        ScrollPositionAlignmentPolicy.keepVisibleAtEnd,
    };
  }

  ScrollPositionAlignmentPolicy _applyAxisDirectionToAlignmentPolicy(
    ScrollPositionAlignmentPolicy alignmentPolicy,
  ) {
    return switch (axisDirection) {
      // Start and end alignments must account for axis direction.
      // When focus is requested for example, it knows the directionality of the
      // keyboard keys initiating traversal, but not the direction of the
      // Scrollable.
      AxisDirection.up || AxisDirection.left => _maybeFlipAlignment(alignmentPolicy),
      AxisDirection.down || AxisDirection.right => alignmentPolicy,
    };
  }

  /// Animates the position such that the given object is as visible as possible
  /// by just scrolling this position.
  ///
  /// The optional `targetRenderObject` parameter is used to determine which area
  /// of that object should be as visible as possible. If `targetRenderObject`
  /// is null, the entire [RenderObject] (as defined by its
  /// [RenderObject.paintBounds]) will be as visible as possible. If
  /// `targetRenderObject` is provided, it must be a descendant of the object.
  ///
  /// See also:
  ///
  ///  * [ScrollPositionAlignmentPolicy] for the way in which `alignment` is
  ///    applied, and the way the given `object` is aligned.
  Future<void> ensureVisible(
    RenderObject object, {
    double alignment = 0.0,
    Duration duration = Duration.zero,
    Curve curve = Curves.ease,
    ScrollPositionAlignmentPolicy alignmentPolicy = ScrollPositionAlignmentPolicy.explicit,
    RenderObject? targetRenderObject,
  }) async {
    assert(object.attached);
    final RenderAbstractViewport? viewport = RenderAbstractViewport.maybeOf(object);
    // If no viewport is found, return.
    if (viewport == null) {
      return;
    }

    Rect? targetRect;
    if (targetRenderObject != null && targetRenderObject != object) {
      targetRect = MatrixUtils.transformRect(
        targetRenderObject.getTransformTo(object),
        object.paintBounds.intersect(targetRenderObject.paintBounds),
      );
    }

    double target;
    switch (_applyAxisDirectionToAlignmentPolicy(alignmentPolicy)) {
      case ScrollPositionAlignmentPolicy.explicit:
        target = viewport.getOffsetToReveal(object, alignment, rect: targetRect, axis: axis).offset;
        target = clampDouble(target, minScrollExtent, maxScrollExtent);
      case ScrollPositionAlignmentPolicy.keepVisibleAtEnd:
        target =
            viewport
                .getOffsetToReveal(
                  object,
                  1.0, // Aligns to end
                  rect: targetRect,
                  axis: axis,
                )
                .offset;
        target = clampDouble(target, minScrollExtent, maxScrollExtent);
        if (target < pixels) {
          target = pixels;
        }
      case ScrollPositionAlignmentPolicy.keepVisibleAtStart:
        target =
            viewport
                .getOffsetToReveal(
                  object,
                  0.0, // Aligns to start
                  rect: targetRect,
                  axis: axis,
                )
                .offset;
        target = clampDouble(target, minScrollExtent, maxScrollExtent);
        if (target > pixels) {
          target = pixels;
        }
    }

    if (target == pixels) {
      return;
    }

    if (duration == Duration.zero) {
      jumpTo(target);
      return;
    }

    return animateTo(target, duration: duration, curve: curve);
  }

  /// This notifier's value is true if a scroll is underway and false if the scroll
  /// position is idle.
  ///
  /// Listeners added by stateful widgets should be removed in the widget's
  /// [State.dispose] method.
  ///
  /// {@tool dartpad}
  /// This sample shows how you can trigger an auto-scroll, which aligns the last
  /// partially visible fixed-height list item, by listening to this
  /// notifier's value. This sort of thing can also be done by listening for
  /// [ScrollEndNotification]s with a [NotificationListener]. An alternative
  /// example is provided with [ScrollEndNotification].
  ///
  /// ** See code in examples/api/lib/widgets/scroll_position/is_scrolling_listener.0.dart **
  /// {@end-tool}
  final ValueNotifier<bool> isScrollingNotifier = ValueNotifier<bool>(false);

  /// Animates the position from its current value to the given value.
  ///
  /// Any active animation is canceled. If the user is currently scrolling, that
  /// action is canceled.
  ///
  /// The returned [Future] will complete when the animation ends, whether it
  /// completed successfully or whether it was interrupted prematurely.
  ///
  /// An animation will be interrupted whenever the user attempts to scroll
  /// manually, or whenever another activity is started, or whenever the
  /// animation reaches the edge of the viewport and attempts to overscroll. (If
  /// the [ScrollPosition] does not overscroll but instead allows scrolling
  /// beyond the extents, then going beyond the extents will not interrupt the
  /// animation.)
  ///
  /// The animation is indifferent to changes to the viewport or content
  /// dimensions.
  ///
  /// Once the animation has completed, the scroll position will attempt to
  /// begin a ballistic activity in case its value is not stable (for example,
  /// if it is scrolled beyond the extents and in that situation the scroll
  /// position would normally bounce back).
  ///
  /// The duration must not be zero. To jump to a particular value without an
  /// animation, use [jumpTo].
  ///
  /// The animation is typically handled by an [DrivenScrollActivity].
  @override
  Future<void> animateTo(double to, {required Duration duration, required Curve curve});

  /// Jumps the scroll position from its current value to the given value,
  /// without animation, and without checking if the new value is in range.
  ///
  /// Any active animation is canceled. If the user is currently scrolling, that
  /// action is canceled.
  ///
  /// If this method changes the scroll position, a sequence of start/update/end
  /// scroll notifications will be dispatched. No overscroll notifications can
  /// be generated by this method.
  @override
  void jumpTo(double value);

  /// Changes the scrolling position based on a pointer signal from current
  /// value to delta without animation and without checking if new value is in
  /// range, taking min/max scroll extent into account.
  ///
  /// Any active animation is canceled. If the user is currently scrolling, that
  /// action is canceled.
  ///
  /// This method dispatches the start/update/end sequence of scrolling
  /// notifications.
  ///
  /// This method is very similar to [jumpTo], but [pointerScroll] will
  /// update the [ScrollDirection].
  void pointerScroll(double delta);

  /// Calls [jumpTo] if duration is null or [Duration.zero], otherwise
  /// [animateTo] is called.
  ///
  /// If [clamp] is true (the default) then [to] is adjusted to prevent over or
  /// underscroll.
  ///
  /// If [animateTo] is called then [curve] defaults to [Curves.ease].
  @override
  Future<void> moveTo(double to, {Duration? duration, Curve? curve, bool? clamp = true}) {
    assert(clamp != null);

    if (clamp!) {
      to = clampDouble(to, minScrollExtent, maxScrollExtent);
    }

    return super.moveTo(to, duration: duration, curve: curve);
  }

  @override
  bool get allowImplicitScrolling => physics.allowImplicitScrolling;

  /// Deprecated. Use [jumpTo] or a custom [ScrollPosition] instead.
  // flutter_ignore: deprecation_syntax, https://github.com/flutter/flutter/issues/44609
  @Deprecated('This will lead to bugs.')
  void jumpToWithoutSettling(double value);

  /// Stop the current activity and start a [HoldScrollActivity].
  ScrollHoldController hold(VoidCallback holdCancelCallback);

  /// Start a drag activity corresponding to the given [DragStartDetails].
  ///
  /// The `onDragCanceled` argument will be invoked if the drag is ended
  /// prematurely (e.g. from another activity taking over). See
  /// [ScrollDragController.onDragCanceled] for details.
  Drag drag(DragStartDetails details, VoidCallback dragCancelCallback);

  /// The currently operative [ScrollActivity].
  ///
  /// If the scroll position is not performing any more specific activity, the
  /// activity will be an [IdleScrollActivity]. To determine whether the scroll
  /// position is idle, check the [isScrollingNotifier].
  ///
  /// Call [beginActivity] to change the current activity.
  @protected
  @visibleForTesting
  ScrollActivity? get activity => _activity;
  ScrollActivity? _activity;

  /// Change the current [activity], disposing of the old one and
  /// sending scroll notifications as necessary.
  ///
  /// If the argument is null, this method has no effect. This is convenient for
  /// cases where the new activity is obtained from another method, and that
  /// method might return null, since it means the caller does not have to
  /// explicitly null-check the argument.
  void beginActivity(ScrollActivity? newActivity) {
    if (newActivity == null) {
      return;
    }
    bool wasScrolling, oldIgnorePointer;
    if (_activity != null) {
      oldIgnorePointer = _activity!.shouldIgnorePointer;
      wasScrolling = _activity!.isScrolling;
      if (wasScrolling && !newActivity.isScrolling) {
        // Notifies and then saves the scroll offset.
        didEndScroll();
      }
      _activity!.dispose();
    } else {
      oldIgnorePointer = false;
      wasScrolling = false;
    }
    _activity = newActivity;
    if (oldIgnorePointer != activity!.shouldIgnorePointer) {
      context.setIgnorePointer(activity!.shouldIgnorePointer);
    }
    isScrollingNotifier.value = activity!.isScrolling;
    if (!wasScrolling && _activity!.isScrolling) {
      didStartScroll();
    }
  }

  // NOTIFICATION DISPATCH

  /// Called by [beginActivity] to report when an activity has started.
  void didStartScroll() {
    activity!.dispatchScrollStartNotification(copyWith(), context.notificationContext);
  }

  /// Called by [setPixels] to report a change to the [pixels] position.
  void didUpdateScrollPositionBy(double delta) {
    activity!.dispatchScrollUpdateNotification(copyWith(), context.notificationContext!, delta);
  }

  /// Called by [beginActivity] to report when an activity has ended.
  ///
  /// This also saves the scroll offset using [saveScrollOffset].
  void didEndScroll() {
    activity!.dispatchScrollEndNotification(copyWith(), context.notificationContext!);
    saveOffset();
    if (keepScrollOffset) {
      saveScrollOffset();
    }
  }

  /// Called by [setPixels] to report overscroll when an attempt is made to
  /// change the [pixels] position. Overscroll is the amount of change that was
  /// not applied to the [pixels] value.
  void didOverscrollBy(double value) {
    assert(activity!.isScrolling);
    activity!.dispatchOverscrollNotification(copyWith(), context.notificationContext!, value);
  }

  /// Dispatches a notification that the [userScrollDirection] has changed.
  ///
  /// Subclasses should call this function when they change [userScrollDirection].
  void didUpdateScrollDirection(ScrollDirection direction) {
    UserScrollNotification(
      metrics: copyWith(),
      context: context.notificationContext!,
      direction: direction,
    ).dispatch(context.notificationContext);
  }

  /// Dispatches a notification that the [ScrollMetrics] have changed.
  void didUpdateScrollMetrics() {
    assert(SchedulerBinding.instance.schedulerPhase != SchedulerPhase.persistentCallbacks);
    assert(_haveScheduledUpdateNotification);
    _haveScheduledUpdateNotification = false;
    if (context.notificationContext != null) {
      ScrollMetricsNotification(
        metrics: copyWith(),
        context: context.notificationContext!,
      ).dispatch(context.notificationContext);
    }
  }

  /// Provides a heuristic to determine if expensive frame-bound tasks should be
  /// deferred.
  ///
  /// The actual work of this is delegated to the [physics] via
  /// [ScrollPhysics.recommendDeferredLoading] called with the current
  /// [activity]'s [ScrollActivity.velocity].
  ///
  /// Returning true from this method indicates that the [ScrollPhysics]
  /// evaluate the current scroll velocity to be great enough that expensive
  /// operations impacting the UI should be deferred.
  bool recommendDeferredLoading(BuildContext context) {
    assert(activity != null);
    return physics.recommendDeferredLoading(
      activity!.velocity + _impliedVelocity,
      copyWith(),
      context,
    );
  }

  @override
  void dispose() {
    activity?.dispose(); // it will be null if it got absorbed by another ScrollPosition
    _activity = null;
    isScrollingNotifier.dispose();
    super.dispose();
  }

  @override
  void notifyListeners() {
    _updateSemanticActions(); // will potentially request a semantics update.
    super.notifyListeners();
  }

  @override
  void debugFillDescription(List<String> description) {
    if (debugLabel != null) {
      description.add(debugLabel!);
    }
    super.debugFillDescription(description);
    description.add(
      'range: ${_minScrollExtent?.toStringAsFixed(1)}..${_maxScrollExtent?.toStringAsFixed(1)}',
    );
    description.add('viewport: ${_viewportDimension?.toStringAsFixed(1)}');
  }
}

/// A notification that a scrollable widget's [ScrollMetrics] have changed.
///
/// For example, when the content of a scrollable is altered, making it larger
/// or smaller, this notification will be dispatched. Similarly, if the size
/// of the window or parent changes, the scrollable can notify of these
/// changes in dimensions.
///
/// The above behaviors usually do not trigger [ScrollNotification] events,
/// so this is useful for listening to [ScrollMetrics] changes that are not
/// caused by the user scrolling.
///
/// {@tool dartpad}
/// This sample shows how a [ScrollMetricsNotification] is dispatched when
/// the `windowSize` is changed. Press the floating action button to increase
/// the scrollable window's size.
///
/// ** See code in examples/api/lib/widgets/scroll_position/scroll_metrics_notification.0.dart **
/// {@end-tool}
class ScrollMetricsNotification extends Notification with ViewportNotificationMixin {
  /// Creates a notification that the scrollable widget's [ScrollMetrics] have
  /// changed.
  ScrollMetricsNotification({required this.metrics, required this.context});

  /// Description of a scrollable widget's [ScrollMetrics].
  final ScrollMetrics metrics;

  /// The build context of the widget that fired this notification.
  ///
  /// This can be used to find the scrollable widget's render objects to
  /// determine the size of the viewport, for instance.
  final BuildContext context;

  /// Convert this notification to a [ScrollNotification].
  ///
  /// This allows it to be used with [ScrollNotificationPredicate]s.
  ScrollUpdateNotification asScrollUpdate() {
    return ScrollUpdateNotification(metrics: metrics, context: context, depth: depth);
  }

  @override
  void debugFillDescription(List<String> description) {
    super.debugFillDescription(description);
    description.add('$metrics');
  }
}
