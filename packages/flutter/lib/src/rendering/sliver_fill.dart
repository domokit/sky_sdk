// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'sliver_list.dart';
library;

import 'dart:math' as math;

import 'box.dart';
import 'object.dart';
import 'sliver.dart';
import 'sliver_fixed_extent_list.dart';

/// A sliver that contains multiple box children that each fill the viewport.
///
/// [RenderSliverFillViewport] places its children in a linear array along the
/// main axis. Each child is sized to fill the viewport, both in the main and
/// cross axis. A [viewportFraction] factor can be provided to size the children
/// to a multiple of the viewport's main axis dimension (typically a fraction
/// less than 1.0).
///
/// See also:
///
///  * [RenderSliverFillRemaining], which sizes the children based on the
///    remaining space rather than the viewport itself.
///  * [RenderSliverFixedExtentList], which has a configurable [itemExtent].
///  * [RenderSliverList], which does not require its children to have the same
///    extent in the main axis.
class RenderSliverFillViewport extends RenderSliverFixedExtentBoxAdaptor {
  /// Creates a sliver that contains multiple box children that each fill the
  /// viewport.
  RenderSliverFillViewport({required super.childManager, double viewportFraction = 1.0})
    : assert(viewportFraction > 0.0),
      _viewportFraction = viewportFraction;

  @override
  double get itemExtent => constraints.viewportMainAxisExtent * viewportFraction;

  /// The fraction of the viewport that each child should fill in the main axis.
  ///
  /// If this fraction is less than 1.0, more than one child will be visible at
  /// once. If this fraction is greater than 1.0, each child will be larger than
  /// the viewport in the main axis.
  double get viewportFraction => _viewportFraction;
  double _viewportFraction;
  set viewportFraction(double value) {
    if (_viewportFraction == value) {
      return;
    }
    _viewportFraction = value;
    markNeedsLayout();
  }
}

/// A sliver that contains a single box child that contains a scrollable and
/// fills the viewport.
///
/// [RenderSliverFillRemainingWithScrollable] sizes its child to fill the
/// viewport in the cross axis and to fill the remaining space in the viewport
/// in the main axis.
///
/// Typically this will be the last sliver in a viewport, since (by definition)
/// there is never any room for anything beyond this sliver.
///
/// See also:
///
///  * [NestedScrollView], which uses this sliver for the inner scrollable.
///  * [RenderSliverFillRemaining], which lays out its
///    non-scrollable child slightly different than this widget.
///  * [RenderSliverFillRemainingAndOverscroll], which incorporates the
///    overscroll into the remaining space to fill.
///  * [RenderSliverFillViewport], which sizes its children based on the
///    size of the viewport, regardless of what else is in the scroll view.
///  * [RenderSliverList], which shows a list of variable-sized children in a
///    viewport.
class RenderSliverFillRemainingWithScrollable extends RenderSliverSingleBoxAdapter {
  /// Creates a [RenderSliver] that wraps a scrollable [RenderBox] which is
  /// sized to fit the remaining space in the viewport.
  RenderSliverFillRemainingWithScrollable({super.child});

  @override
  void performLayout() {
    final SliverConstraints constraints = this.constraints;
    final double extent = constraints.remainingPaintExtent - math.min(constraints.overlap, 0.0);

    final double cacheExtent = calculateCacheOffset(
      constraints,
      from: 0.0,
      to: constraints.viewportMainAxisExtent,
    );
    if (child != null) {
      double maxExtent = extent;

      // If sliver has no extent, but is within viewport's cacheExtent, use the
      // sliver's cacheExtent as the maxExtent so that it does not get dropped
      // from the semantic tree.
      if (extent == 0 && cacheExtent > 0) {
        maxExtent = cacheExtent;
      }
      child!.layout(constraints.asBoxConstraints(minExtent: extent, maxExtent: maxExtent));
    }

    final double paintedChildSize = calculatePaintOffset(constraints, from: 0.0, to: extent);
    assert(paintedChildSize.isFinite);
    assert(paintedChildSize >= 0.0);

    geometry = SliverGeometry(
      scrollExtent: constraints.viewportMainAxisExtent,
      paintExtent: paintedChildSize,
      maxPaintExtent: paintedChildSize,
      hasVisualOverflow:
          extent > constraints.remainingPaintExtent || constraints.scrollOffset > 0.0,
      cacheExtent: cacheExtent,
    );
    if (child != null) {
      setChildParentData(child!, constraints, geometry!);
    }
  }
}

/// A sliver that contains a single box child that is non-scrollable and fills
/// the remaining space in the viewport.
///
/// [RenderSliverFillRemaining] sizes its child to fill the
/// viewport in the cross axis and to fill the remaining space in the viewport
/// in the main axis.
///
/// Typically this will be the last sliver in a viewport, since (by definition)
/// there is never any room for anything beyond this sliver.
///
/// See also:
///
///  * [RenderSliverFillRemainingWithScrollable], which lays out its scrollable
///    child slightly different than this widget.
///  * [RenderSliverFillRemainingAndOverscroll], which incorporates the
///    overscroll into the remaining space to fill.
///  * [RenderSliverFillViewport], which sizes its children based on the
///    size of the viewport, regardless of what else is in the scroll view.
///  * [RenderSliverList], which shows a list of variable-sized children in a
///    viewport.
class RenderSliverFillRemaining extends RenderSliverSingleBoxAdapter {
  /// Creates a [RenderSliver] that wraps a non-scrollable [RenderBox] which is
  /// sized to fit the remaining space in the viewport.
  RenderSliverFillRemaining({super.child});

  @override
  void performLayout() {
    final SliverConstraints constraints = this.constraints;
    // The remaining space in the viewportMainAxisExtent. Can be <= 0 if we have
    // scrolled beyond the extent of the screen.
    double extent = constraints.viewportMainAxisExtent - constraints.precedingScrollExtent;

    if (child != null) {
      final double childExtent = switch (constraints.axis) {
        Axis.horizontal => child!.getMaxIntrinsicWidth(constraints.crossAxisExtent),
        Axis.vertical => child!.getMaxIntrinsicHeight(constraints.crossAxisExtent),
      };

      // If the childExtent is greater than the computed extent, we want to use
      // that instead of potentially cutting off the child. This allows us to
      // safely specify a maxExtent.
      extent = math.max(extent, childExtent);
      child!.layout(constraints.asBoxConstraints(minExtent: extent, maxExtent: extent));
    }

    assert(
      extent.isFinite,
      'The calculated extent for the child of SliverFillRemaining is not finite. '
      'This can happen if the child is a scrollable, in which case, the '
      'hasScrollBody property of SliverFillRemaining should not be set to '
      'false.',
    );
    final double paintedChildSize = calculatePaintOffset(constraints, from: 0.0, to: extent);
    assert(paintedChildSize.isFinite);
    assert(paintedChildSize >= 0.0);

    final double cacheExtent = calculateCacheOffset(constraints, from: 0.0, to: extent);
    geometry = SliverGeometry(
      scrollExtent: extent,
      paintExtent: paintedChildSize,
      maxPaintExtent: paintedChildSize,
      hasVisualOverflow:
          extent > constraints.remainingPaintExtent || constraints.scrollOffset > 0.0,
      cacheExtent: cacheExtent,
    );
    if (child != null) {
      setChildParentData(child!, constraints, geometry!);
    }
  }
}

/// A sliver that contains a single box child that is non-scrollable and fills
/// the remaining space in the viewport including any overscrolled area.
///
/// [RenderSliverFillRemainingAndOverscroll] sizes its child to fill the
/// viewport in the cross axis and to fill the remaining space in the viewport
/// in the main axis with the overscroll area included.
///
/// Typically this will be the last sliver in a viewport, since (by definition)
/// there is never any room for anything beyond this sliver.
///
/// See also:
///
///  * [RenderSliverFillRemainingWithScrollable], which lays out its scrollable
///    child without overscroll.
///  * [RenderSliverFillRemaining], which lays out its
///    non-scrollable child without overscroll.
///  * [RenderSliverFillViewport], which sizes its children based on the
///    size of the viewport, regardless of what else is in the scroll view.
///  * [RenderSliverList], which shows a list of variable-sized children in a
///    viewport.
class RenderSliverFillRemainingAndOverscroll extends RenderSliverSingleBoxAdapter {
  /// Creates a [RenderSliver] that wraps a non-scrollable [RenderBox] which is
  /// sized to fit the remaining space plus any overscroll in the viewport.
  RenderSliverFillRemainingAndOverscroll({super.child});

  @override
  void performLayout() {
    final SliverConstraints constraints = this.constraints;
    // The remaining space in the viewportMainAxisExtent. Can be <= 0 if we have
    // scrolled beyond the extent of the screen.
    double extent = constraints.viewportMainAxisExtent - constraints.precedingScrollExtent;
    // The maxExtent includes any overscrolled area. Can be < 0 if we have
    // overscroll in the opposite direction, away from the end of the list.
    double maxExtent = constraints.remainingPaintExtent - math.min(constraints.overlap, 0.0);

    if (child != null) {
      final double childExtent = switch (constraints.axis) {
        Axis.horizontal => child!.getMaxIntrinsicWidth(constraints.crossAxisExtent),
        Axis.vertical => child!.getMaxIntrinsicHeight(constraints.crossAxisExtent),
      };

      // If the childExtent is greater than the computed extent, we want to use
      // that instead of potentially cutting off the child. This allows us to
      // safely specify a maxExtent.
      extent = math.max(extent, childExtent);
      // The extent could be larger than the maxExtent due to a larger child
      // size or overscrolling at the top of the scrollable (rather than at the
      // end where this sliver is).
      maxExtent = math.max(extent, maxExtent);
      child!.layout(constraints.asBoxConstraints(minExtent: extent, maxExtent: maxExtent));
    }

    assert(
      extent.isFinite,
      'The calculated extent for the child of SliverFillRemaining is not finite. '
      'This can happen if the child is a scrollable, in which case, the '
      'hasScrollBody property of SliverFillRemaining should not be set to '
      'false.',
    );
    final double paintedChildSize = calculatePaintOffset(constraints, from: 0.0, to: extent);
    assert(paintedChildSize.isFinite);
    assert(paintedChildSize >= 0.0);

    final double cacheExtent = calculateCacheOffset(constraints, from: 0.0, to: extent);
    geometry = SliverGeometry(
      scrollExtent: extent,
      paintExtent: math.min(maxExtent, constraints.remainingPaintExtent),
      maxPaintExtent: maxExtent,
      hasVisualOverflow:
          extent > constraints.remainingPaintExtent || constraints.scrollOffset > 0.0,
      cacheExtent: cacheExtent,
    );
    if (child != null) {
      setChildParentData(child!, constraints, geometry!);
    }
  }
}
