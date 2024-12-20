// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'sliver.dart';
/// @docImport 'sliver_persistent_header.dart';
/// @docImport 'viewport.dart';
library;

import 'package:flutter/animation.dart';
import 'package:flutter/foundation.dart';

/// The direction of a scroll, relative to the positive scroll offset axis given
/// by an [AxisDirection] and a [GrowthDirection].
///
/// This is similar to [GrowthDirection], but contrasts in that it has a third
/// value, [idle], for the case where no scroll is occurring.
///
/// This is used by [RenderSliverFloatingPersistentHeader] to only expand when
/// the user is scrolling in the same direction as the detected scroll offset
/// change.
///
/// {@template flutter.rendering.ScrollDirection.sample}
/// {@tool dartpad}
/// This sample shows a [CustomScrollView], with [Radio] buttons in the
/// [AppBar.bottom] that change the [AxisDirection] to illustrate different
/// configurations. With a [NotificationListener] to listen to
/// [UserScrollNotification]s, which occur when the [ScrollDirection] changes
/// or stops.
///
/// ** See code in examples/api/lib/rendering/scroll_direction/scroll_direction.0.dart **
/// {@end-tool}
/// {@endtemplate}
///
/// See also:
///
///  * [AxisDirection], which is a directional version of this enum (with values
///    like left and right, rather than just horizontal).
///  * [GrowthDirection], the direction in which slivers and their content are
///    ordered, relative to the scroll offset axis as specified by
///    [AxisDirection].
///  * [UserScrollNotification], which will notify listeners when the
///    [ScrollDirection] changes.
enum ScrollDirection {
  /// No scrolling is underway.
  idle,

  /// Scrolling is happening in the negative scroll offset direction.
  ///
  /// For example, for the [GrowthDirection.forward] part of a vertical
  /// [AxisDirection.down] list, which is the default directional configuration
  /// of all scroll views, this means the content is going down, exposing
  /// earlier content as it approaches the zero position.
  ///
  /// An anecdote for this most common case is 'forward is toward' the zero
  /// position.
  forward,

  /// Scrolling is happening in the positive scroll offset direction.
  ///
  /// For example, for the [GrowthDirection.forward] part of a vertical
  /// [AxisDirection.down] list, which is the default directional configuration
  /// of all scroll views, this means the content is moving up, exposing
  /// lower content.
  ///
  /// An anecdote for this most common case is reversing, or backing away, from
  /// the zero position.
  reverse,
}

/// Returns the opposite of the given [ScrollDirection].
///
/// Specifically, returns [ScrollDirection.reverse] for [ScrollDirection.forward]
/// (and vice versa) and returns [ScrollDirection.idle] for
/// [ScrollDirection.idle].
ScrollDirection flipScrollDirection(ScrollDirection direction) {
  return switch (direction) {
    ScrollDirection.idle => ScrollDirection.idle,
    ScrollDirection.forward => ScrollDirection.reverse,
    ScrollDirection.reverse => ScrollDirection.forward,
  };
}

/// Which part of the content inside the viewport should be visible.
///
/// The [pixels] value determines the scroll offset that the viewport uses to
/// select which part of its content to display. As the user scrolls the
/// viewport, this value changes, which changes the content that is displayed.
///
/// This object is a [Listenable] that notifies its listeners when [pixels]
/// changes.
///
/// See also:
///
///  * [ScrollPosition], which is a commonly used concrete subclass.
///  * [RenderViewportBase], which is a render object that uses viewport
///    offsets.
abstract class ViewportOffset extends ChangeNotifier {
  /// Default constructor.
  ///
  /// Allows subclasses to construct this object directly.
  ViewportOffset() {
    if (kFlutterMemoryAllocationsEnabled) {
      ChangeNotifier.maybeDispatchObjectCreation(this);
    }
  }

  /// Creates a viewport offset with the given [pixels] value.
  ///
  /// The [pixels] value does not change unless the viewport issues a
  /// correction.
  factory ViewportOffset.fixed(double value) = _FixedViewportOffset;

  /// Creates a viewport offset with a [pixels] value of 0.0.
  ///
  /// The [pixels] value does not change unless the viewport issues a
  /// correction.
  factory ViewportOffset.zero() = _FixedViewportOffset.zero;

  /// The number of pixels to offset the children in the opposite of the axis direction.
  ///
  /// For example, if the axis direction is down, then the pixel value
  /// represents the number of logical pixels to move the children _up_ the
  /// screen. Similarly, if the axis direction is left, then the pixels value
  /// represents the number of logical pixels to move the children to _right_.
  ///
  /// This object notifies its listeners when this value changes (except when
  /// the value changes due to [correctBy]).
  double get pixels;

  /// Whether the [pixels] property is available.
  bool get hasPixels;

  /// Called when the viewport's extents are established.
  ///
  /// The argument is the dimension of the [RenderViewport] in the main axis
  /// (e.g. the height, for a vertical viewport).
  ///
  /// This may be called redundantly, with the same value, each frame. This is
  /// called during layout for the [RenderViewport]. If the viewport is
  /// configured to shrink-wrap its contents, it may be called several times,
  /// since the layout is repeated each time the scroll offset is corrected.
  ///
  /// If this is called, it is called before [applyContentDimensions]. If this
  /// is called, [applyContentDimensions] will be called soon afterwards in the
  /// same layout phase. If the viewport is not configured to shrink-wrap its
  /// contents, then this will only be called when the viewport recomputes its
  /// size (i.e. when its parent lays out), and not during normal scrolling.
  ///
  /// If applying the viewport dimensions changes the scroll offset, return
  /// false. Otherwise, return true. If you return false, the [RenderViewport]
  /// will be laid out again with the new scroll offset. This is expensive. (The
  /// return value is answering the question "did you accept these viewport
  /// dimensions unconditionally?"; if the new dimensions change the
  /// [ViewportOffset]'s actual [pixels] value, then the viewport will need to
  /// be laid out again.)
  bool applyViewportDimension(double viewportDimension);

  /// Called when the viewport's content extents are established.
  ///
  /// The arguments are the minimum and maximum scroll extents respectively. The
  /// minimum will be equal to or less than the maximum. In the case of slivers,
  /// the minimum will be equal to or less than zero, the maximum will be equal
  /// to or greater than zero.
  ///
  /// The maximum scroll extent has the viewport dimension subtracted from it.
  /// For instance, if there is 100.0 pixels of scrollable content, and the
  /// viewport is 80.0 pixels high, then the minimum scroll extent will
  /// typically be 0.0 and the maximum scroll extent will typically be 20.0,
  /// because there's only 20.0 pixels of actual scroll slack.
  ///
  /// If applying the content dimensions changes the scroll offset, return
  /// false. Otherwise, return true. If you return false, the [RenderViewport]
  /// will be laid out again with the new scroll offset. This is expensive. (The
  /// return value is answering the question "did you accept these content
  /// dimensions unconditionally?"; if the new dimensions change the
  /// [ViewportOffset]'s actual [pixels] value, then the viewport will need to
  /// be laid out again.)
  ///
  /// This is called at least once each time the [RenderViewport] is laid out,
  /// even if the values have not changed. It may be called many times if the
  /// scroll offset is corrected (if this returns false). This is always called
  /// after [applyViewportDimension], if that method is called.
  bool applyContentDimensions(double minScrollExtent, double maxScrollExtent);

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
  void correctBy(double correction);

  /// Jumps [pixels] from its current value to the given value,
  /// without animation, and without checking if the new value is in range.
  ///
  /// See also:
  ///
  ///  * [correctBy], for changing the current offset in the middle of layout
  ///    and that defers the notification of its listeners until after layout.
  void jumpTo(double pixels);

  /// Animates [pixels] from its current value to the given value.
  ///
  /// The returned [Future] will complete when the animation ends, whether it
  /// completed successfully or whether it was interrupted prematurely.
  ///
  /// The duration must not be zero. To jump to a particular value without an
  /// animation, use [jumpTo].
  Future<void> animateTo(double to, {required Duration duration, required Curve curve});

  /// Calls [jumpTo] if duration is null or [Duration.zero], otherwise
  /// [animateTo] is called.
  ///
  /// If [animateTo] is called then [curve] defaults to [Curves.ease]. The
  /// [clamp] parameter is ignored by this stub implementation but subclasses
  /// like [ScrollPosition] handle it by adjusting [to] to prevent over or
  /// underscroll.
  Future<void> moveTo(double to, {Duration? duration, Curve? curve, bool? clamp}) {
    if (duration == null || duration == Duration.zero) {
      jumpTo(to);
      return Future<void>.value();
    } else {
      return animateTo(to, duration: duration, curve: curve ?? Curves.ease);
    }
  }

  /// The direction in which the user is trying to change [pixels], relative to
  /// the viewport's [RenderViewportBase.axisDirection].
  ///
  /// If the _user_ is not scrolling, this will return [ScrollDirection.idle]
  /// even if there is (for example) a [ScrollActivity] currently animating the
  /// position.
  ///
  /// This is exposed in [SliverConstraints.userScrollDirection], which is used
  /// by some slivers to determine how to react to a change in scroll offset.
  /// For example, [RenderSliverFloatingPersistentHeader] will only expand a
  /// floating app bar when the [userScrollDirection] is in the positive scroll
  /// offset direction.
  ///
  /// {@macro flutter.rendering.ScrollDirection.sample}
  ScrollDirection get userScrollDirection;

  /// Whether a viewport is allowed to change [pixels] implicitly to respond to
  /// a call to [RenderObject.showOnScreen].
  ///
  /// [RenderObject.showOnScreen] is, for example, used to bring a text field
  /// fully on screen after it has received focus. This property controls
  /// whether the viewport associated with this offset is allowed to change the
  /// offset's [pixels] value to fulfill such a request.
  bool get allowImplicitScrolling;

  @override
  String toString() {
    final List<String> description = <String>[];
    debugFillDescription(description);
    return '${describeIdentity(this)}(${description.join(", ")})';
  }

  /// Add additional information to the given description for use by [toString].
  ///
  /// This method makes it easier for subclasses to coordinate to provide a
  /// high-quality [toString] implementation. The [toString] implementation on
  /// the [State] base class calls [debugFillDescription] to collect useful
  /// information from subclasses to incorporate into its return value.
  ///
  /// Implementations of this method should start with a call to the inherited
  /// method, as in `super.debugFillDescription(description)`.
  @mustCallSuper
  void debugFillDescription(List<String> description) {
    if (hasPixels) {
      description.add('offset: ${pixels.toStringAsFixed(1)}');
    }
  }
}

class _FixedViewportOffset extends ViewportOffset {
  _FixedViewportOffset(this._pixels);
  _FixedViewportOffset.zero() : _pixels = 0.0;

  double _pixels;

  @override
  double get pixels => _pixels;

  @override
  bool get hasPixels => true;

  @override
  bool applyViewportDimension(double viewportDimension) => true;

  @override
  bool applyContentDimensions(double minScrollExtent, double maxScrollExtent) => true;

  @override
  void correctBy(double correction) {
    _pixels += correction;
  }

  @override
  void jumpTo(double pixels) {
    // Do nothing, viewport is fixed.
  }

  @override
  Future<void> animateTo(double to, {required Duration duration, required Curve curve}) async {}

  @override
  ScrollDirection get userScrollDirection => ScrollDirection.idle;

  @override
  bool get allowImplicitScrolling => false;
}
