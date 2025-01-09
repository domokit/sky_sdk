// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'sliver_fixed_extent_list.dart';
/// @docImport 'sliver_list.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart';

import 'box.dart';
import 'object.dart';
import 'sliver.dart';
import 'sliver_multi_box_adaptor.dart';

/// Describes the placement of a child in a [RenderSliverGrid].
///
/// This class is similar to [Rect], in that it gives a two-dimensional position
/// and a two-dimensional dimension, but is direction-agnostic.
///
/// {@tool dartpad}
/// This example shows how a custom [SliverGridLayout] uses [SliverGridGeometry]
/// to lay out the children.
///
/// ** See code in examples/api/lib/widgets/scroll_view/grid_view.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [SliverGridLayout], which represents the geometry of all the tiles in a
///    grid.
///  * [SliverGridLayout.getGeometryForChildIndex], which returns this object
///    to describe the child's placement.
///  * [RenderSliverGrid], which uses this class during its
///    [RenderSliverGrid.performLayout] method.
@immutable
class SliverGridGeometry {
  /// Creates an object that describes the placement of a child in a [RenderSliverGrid].
  const SliverGridGeometry({
    required this.scrollOffset,
    required this.crossAxisOffset,
    required this.mainAxisExtent,
    required this.crossAxisExtent,
  });

  /// The scroll offset of the leading edge of the child relative to the leading
  /// edge of the parent.
  final double scrollOffset;

  /// The offset of the child in the non-scrolling axis.
  ///
  /// If the scroll axis is vertical, this offset is from the left-most edge of
  /// the parent to the left-most edge of the child. If the scroll axis is
  /// horizontal, this offset is from the top-most edge of the parent to the
  /// top-most edge of the child.
  final double crossAxisOffset;

  /// The extent of the child in the scrolling axis.
  ///
  /// If the scroll axis is vertical, this extent is the child's height. If the
  /// scroll axis is horizontal, this extent is the child's width.
  final double mainAxisExtent;

  /// The extent of the child in the non-scrolling axis.
  ///
  /// If the scroll axis is vertical, this extent is the child's width. If the
  /// scroll axis is horizontal, this extent is the child's height.
  final double crossAxisExtent;

  /// The scroll offset of the trailing edge of the child relative to the
  /// leading edge of the parent.
  double get trailingScrollOffset => scrollOffset + mainAxisExtent;

  /// Returns a tight [BoxConstraints] that forces the child to have the
  /// required size, given a [SliverConstraints].
  BoxConstraints getBoxConstraints(SliverConstraints constraints) {
    return constraints.asBoxConstraints(
      minExtent: mainAxisExtent,
      maxExtent: mainAxisExtent,
      crossAxisExtent: crossAxisExtent,
    );
  }

  @override
  String toString() {
    final List<String> properties = <String>[
      'scrollOffset: $scrollOffset',
      'crossAxisOffset: $crossAxisOffset',
      'mainAxisExtent: $mainAxisExtent',
      'crossAxisExtent: $crossAxisExtent',
    ];
    return 'SliverGridGeometry(${properties.join(', ')})';
  }
}

/// The size and position of all the tiles in a [RenderSliverGrid].
///
/// Rather that providing a grid with a [SliverGridLayout] directly, the grid is
/// provided a [SliverGridDelegate], which computes a [SliverGridLayout] given a
/// set of [SliverConstraints]. This allows the algorithm to dynamically respond
/// to changes in the environment (e.g. the user rotating the device).
///
/// The tiles can be placed arbitrarily, but it is more efficient to place tiles
/// roughly in order by scroll offset because grids reify a contiguous sequence
/// of children.
///
/// {@tool dartpad}
/// This example shows how to construct a custom [SliverGridLayout] to lay tiles
/// in a grid form with some cells stretched to fit the entire width of the
/// grid (sometimes called "hero tiles").
///
/// ** See code in examples/api/lib/widgets/scroll_view/grid_view.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [SliverGridRegularTileLayout], which represents a layout that uses
///    equally sized and spaced tiles.
///  * [SliverGridGeometry], which represents the size and position of a single
///    tile in a grid.
///  * [SliverGridDelegate.getLayout], which returns this object to describe the
///    delegate's layout.
///  * [RenderSliverGrid], which uses this class during its
///    [RenderSliverGrid.performLayout] method.
@immutable
abstract class SliverGridLayout {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const SliverGridLayout();

  /// The minimum child index that intersects with (or is after) this scroll offset.
  int getMinChildIndexForScrollOffset(double scrollOffset);

  /// The maximum child index that intersects with (or is before) this scroll offset.
  int getMaxChildIndexForScrollOffset(double scrollOffset);

  /// The size and position of the child with the given index.
  SliverGridGeometry getGeometryForChildIndex(int index);

  /// The scroll extent needed to fully display all the tiles if there are
  /// `childCount` children in total.
  ///
  /// The child count will never be null.
  double computeMaxScrollOffset(int childCount);
}

/// A [SliverGridLayout] that uses equally sized and spaced tiles.
///
/// Rather that providing a grid with a [SliverGridLayout] directly, you instead
/// provide the grid a [SliverGridDelegate], which can compute a
/// [SliverGridLayout] given the current [SliverConstraints].
///
/// This layout is used by [SliverGridDelegateWithFixedCrossAxisCount] and
/// [SliverGridDelegateWithMaxCrossAxisExtent].
///
/// See also:
///
///  * [SliverGridDelegateWithFixedCrossAxisCount], which uses this layout.
///  * [SliverGridDelegateWithMaxCrossAxisExtent], which uses this layout.
///  * [SliverGridLayout], which represents an arbitrary tile layout.
///  * [SliverGridGeometry], which represents the size and position of a single
///    tile in a grid.
///  * [SliverGridDelegate.getLayout], which returns this object to describe the
///    delegate's layout.
///  * [RenderSliverGrid], which uses this class during its
///    [RenderSliverGrid.performLayout] method.
class SliverGridRegularTileLayout extends SliverGridLayout {
  /// Creates a layout that uses equally sized and spaced tiles.
  ///
  /// All of the arguments must not be negative. The `crossAxisCount` argument
  /// must be greater than zero.
  const SliverGridRegularTileLayout({
    required this.crossAxisCount,
    required this.mainAxisStride,
    required this.crossAxisStride,
    required this.childMainAxisExtent,
    required this.childCrossAxisExtent,
    required this.reverseCrossAxis,
  }) : assert(crossAxisCount > 0),
       assert(mainAxisStride >= 0),
       assert(crossAxisStride >= 0),
       assert(childMainAxisExtent >= 0),
       assert(childCrossAxisExtent >= 0);

  /// The number of children in the cross axis.
  final int crossAxisCount;

  /// The number of pixels from the leading edge of one tile to the leading edge
  /// of the next tile in the main axis.
  final double mainAxisStride;

  /// The number of pixels from the leading edge of one tile to the leading edge
  /// of the next tile in the cross axis.
  final double crossAxisStride;

  /// The number of pixels from the leading edge of one tile to the trailing
  /// edge of the same tile in the main axis.
  final double childMainAxisExtent;

  /// The number of pixels from the leading edge of one tile to the trailing
  /// edge of the same tile in the cross axis.
  final double childCrossAxisExtent;

  /// Whether the children should be placed in the opposite order of increasing
  /// coordinates in the cross axis.
  ///
  /// For example, if the cross axis is horizontal, the children are placed from
  /// left to right when [reverseCrossAxis] is false and from right to left when
  /// [reverseCrossAxis] is true.
  ///
  /// Typically set to the return value of [axisDirectionIsReversed] applied to
  /// the [SliverConstraints.crossAxisDirection].
  final bool reverseCrossAxis;

  @override
  int getMinChildIndexForScrollOffset(double scrollOffset) {
    return mainAxisStride > precisionErrorTolerance
        ? crossAxisCount * (scrollOffset ~/ mainAxisStride)
        : 0;
  }

  @override
  int getMaxChildIndexForScrollOffset(double scrollOffset) {
    if (mainAxisStride > 0.0) {
      final int mainAxisCount = (scrollOffset / mainAxisStride).ceil();
      return math.max(0, crossAxisCount * mainAxisCount - 1);
    }
    return 0;
  }

  double _getOffsetFromStartInCrossAxis(double crossAxisStart) {
    if (reverseCrossAxis) {
      return crossAxisCount * crossAxisStride -
          crossAxisStart -
          childCrossAxisExtent -
          (crossAxisStride - childCrossAxisExtent);
    }
    return crossAxisStart;
  }

  @override
  SliverGridGeometry getGeometryForChildIndex(int index) {
    final double crossAxisStart = (index % crossAxisCount) * crossAxisStride;
    return SliverGridGeometry(
      scrollOffset: (index ~/ crossAxisCount) * mainAxisStride,
      crossAxisOffset: _getOffsetFromStartInCrossAxis(crossAxisStart),
      mainAxisExtent: childMainAxisExtent,
      crossAxisExtent: childCrossAxisExtent,
    );
  }

  @override
  double computeMaxScrollOffset(int childCount) {
    if (childCount == 0) {
      // There are no children in the grid. The max scroll offset should be
      // zero.
      return 0.0;
    }
    final int mainAxisCount = ((childCount - 1) ~/ crossAxisCount) + 1;
    final double mainAxisSpacing = mainAxisStride - childMainAxisExtent;
    return mainAxisStride * mainAxisCount - mainAxisSpacing;
  }
}

/// Controls the layout of tiles in a grid.
///
/// Given the current constraints on the grid, a [SliverGridDelegate] computes
/// the layout for the tiles in the grid. The tiles can be placed arbitrarily,
/// but it is more efficient to place tiles roughly in order by scroll offset
/// because grids reify a contiguous sequence of children.
///
/// {@tool dartpad}
/// This example shows how a [SliverGridDelegate] returns a [SliverGridLayout]
/// configured based on the provided [SliverConstraints] in [getLayout].
///
/// ** See code in examples/api/lib/widgets/scroll_view/grid_view.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [SliverGridDelegateWithFixedCrossAxisCount], which creates a layout with
///    a fixed number of tiles in the cross axis.
///  * [SliverGridDelegateWithMaxCrossAxisExtent], which creates a layout with
///    tiles that have a maximum cross-axis extent.
///  * [GridView], which uses this delegate to control the layout of its tiles.
///  * [SliverGrid], which uses this delegate to control the layout of its
///    tiles.
///  * [RenderSliverGrid], which uses this delegate to control the layout of its
///    tiles.
abstract class SliverGridDelegate {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const SliverGridDelegate();

  /// Returns information about the size and position of the tiles in the grid.
  SliverGridLayout getLayout(SliverConstraints constraints);

  /// Override this method to return true when the children need to be
  /// laid out.
  ///
  /// This should compare the fields of the current delegate and the given
  /// `oldDelegate` and return true if the fields are such that the layout would
  /// be different.
  bool shouldRelayout(covariant SliverGridDelegate oldDelegate);
}

/// Creates grid layouts with a fixed number of tiles in the cross axis.
///
/// For example, if the grid is vertical, this delegate will create a layout
/// with a fixed number of columns. If the grid is horizontal, this delegate
/// will create a layout with a fixed number of rows.
///
/// This delegate creates grids with equally sized and spaced tiles.
///
/// {@tool dartpad}
/// Here is an example using the [childAspectRatio] property. On a device with a
/// screen width of 800.0, it creates a GridView with each tile with a width of
/// 200.0 and a height of 100.0.
///
/// ** See code in examples/api/lib/rendering/sliver_grid/sliver_grid_delegate_with_fixed_cross_axis_count.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// Here is an example using the [mainAxisExtent] property. On a device with a
/// screen width of 800.0, it creates a GridView with each tile with a width of
/// 200.0 and a height of 150.0.
///
/// ** See code in examples/api/lib/rendering/sliver_grid/sliver_grid_delegate_with_fixed_cross_axis_count.1.dart **
/// {@end-tool}
///
/// See also:
///
///  * [SliverGridDelegateWithMaxCrossAxisExtent], which creates a layout with
///    tiles that have a maximum cross-axis extent.
///  * [SliverGridDelegate], which creates arbitrary layouts.
///  * [GridView], which can use this delegate to control the layout of its
///    tiles.
///  * [SliverGrid], which can use this delegate to control the layout of its
///    tiles.
///  * [RenderSliverGrid], which can use this delegate to control the layout of
///    its tiles.
class SliverGridDelegateWithFixedCrossAxisCount extends SliverGridDelegate {
  /// Creates a delegate that makes grid layouts with a fixed number of tiles in
  /// the cross axis.
  ///
  /// The `mainAxisSpacing`, `mainAxisExtent` and `crossAxisSpacing` arguments
  /// must not be negative. The `crossAxisCount` and `childAspectRatio`
  /// arguments must be greater than zero.
  const SliverGridDelegateWithFixedCrossAxisCount({
    required this.crossAxisCount,
    this.mainAxisSpacing = 0.0,
    this.crossAxisSpacing = 0.0,
    this.childAspectRatio = 1.0,
    this.mainAxisExtent,
  }) : assert(crossAxisCount > 0),
       assert(mainAxisSpacing >= 0),
       assert(crossAxisSpacing >= 0),
       assert(childAspectRatio > 0),
       assert(mainAxisExtent == null || mainAxisExtent >= 0);

  /// The number of children in the cross axis.
  final int crossAxisCount;

  /// The number of logical pixels between each child along the main axis.
  final double mainAxisSpacing;

  /// The number of logical pixels between each child along the cross axis.
  final double crossAxisSpacing;

  /// The ratio of the cross-axis to the main-axis extent of each child.
  final double childAspectRatio;

  /// The extent of each tile in the main axis. If provided it would define the
  /// logical pixels taken by each tile in the main-axis.
  ///
  /// If null, [childAspectRatio] is used instead.
  final double? mainAxisExtent;

  bool _debugAssertIsValid() {
    assert(crossAxisCount > 0);
    assert(mainAxisSpacing >= 0.0);
    assert(crossAxisSpacing >= 0.0);
    assert(childAspectRatio > 0.0);
    return true;
  }

  @override
  SliverGridLayout getLayout(SliverConstraints constraints) {
    assert(_debugAssertIsValid());
    final double usableCrossAxisExtent = math.max(
      0.0,
      constraints.crossAxisExtent - crossAxisSpacing * (crossAxisCount - 1),
    );
    final double childCrossAxisExtent = usableCrossAxisExtent / crossAxisCount;
    final double childMainAxisExtent = mainAxisExtent ?? childCrossAxisExtent / childAspectRatio;
    return SliverGridRegularTileLayout(
      crossAxisCount: crossAxisCount,
      mainAxisStride: childMainAxisExtent + mainAxisSpacing,
      crossAxisStride: childCrossAxisExtent + crossAxisSpacing,
      childMainAxisExtent: childMainAxisExtent,
      childCrossAxisExtent: childCrossAxisExtent,
      reverseCrossAxis: axisDirectionIsReversed(constraints.crossAxisDirection),
    );
  }

  @override
  bool shouldRelayout(SliverGridDelegateWithFixedCrossAxisCount oldDelegate) {
    return oldDelegate.crossAxisCount != crossAxisCount ||
        oldDelegate.mainAxisSpacing != mainAxisSpacing ||
        oldDelegate.crossAxisSpacing != crossAxisSpacing ||
        oldDelegate.childAspectRatio != childAspectRatio ||
        oldDelegate.mainAxisExtent != mainAxisExtent;
  }
}

/// Creates grid layouts with tiles that each have a maximum cross-axis extent.
///
/// This delegate will select a cross-axis extent for the tiles that is as
/// large as possible subject to the following conditions:
///
///  - The extent evenly divides the cross-axis extent of the grid.
///  - The extent is at most [maxCrossAxisExtent].
///
/// For example, if the grid is vertical, the grid is 500.0 pixels wide, and
/// [maxCrossAxisExtent] is 150.0, this delegate will create a grid with 4
/// columns that are 125.0 pixels wide.
///
/// This delegate creates grids with equally sized and spaced tiles.
///
/// See also:
///
///  * [SliverGridDelegateWithFixedCrossAxisCount], which creates a layout with
///    a fixed number of tiles in the cross axis.
///  * [SliverGridDelegate], which creates arbitrary layouts.
///  * [GridView], which can use this delegate to control the layout of its
///    tiles.
///  * [SliverGrid], which can use this delegate to control the layout of its
///    tiles.
///  * [RenderSliverGrid], which can use this delegate to control the layout of
///    its tiles.
class SliverGridDelegateWithMaxCrossAxisExtent extends SliverGridDelegate {
  /// Creates a delegate that makes grid layouts with tiles that have a maximum
  /// cross-axis extent.
  ///
  /// The [maxCrossAxisExtent], [mainAxisExtent], [mainAxisSpacing],
  /// and [crossAxisSpacing] arguments must not be negative.
  /// The [childAspectRatio] argument must be greater than zero.
  const SliverGridDelegateWithMaxCrossAxisExtent({
    required this.maxCrossAxisExtent,
    this.mainAxisSpacing = 0.0,
    this.crossAxisSpacing = 0.0,
    this.childAspectRatio = 1.0,
    this.mainAxisExtent,
  }) : assert(maxCrossAxisExtent > 0),
       assert(mainAxisSpacing >= 0),
       assert(crossAxisSpacing >= 0),
       assert(childAspectRatio > 0),
       assert(mainAxisExtent == null || mainAxisExtent >= 0);

  /// The maximum extent of tiles in the cross axis.
  ///
  /// This delegate will select a cross-axis extent for the tiles that is as
  /// large as possible subject to the following conditions:
  ///
  ///  - The extent evenly divides the cross-axis extent of the grid.
  ///  - The extent is at most [maxCrossAxisExtent].
  ///
  /// For example, if the grid is vertical, the grid is 500.0 pixels wide, and
  /// [maxCrossAxisExtent] is 150.0, this delegate will create a grid with 4
  /// columns that are 125.0 pixels wide.
  final double maxCrossAxisExtent;

  /// The number of logical pixels between each child along the main axis.
  final double mainAxisSpacing;

  /// The number of logical pixels between each child along the cross axis.
  final double crossAxisSpacing;

  /// The ratio of the cross-axis to the main-axis extent of each child.
  final double childAspectRatio;

  /// The extent of each tile in the main axis. If provided it would define the
  /// logical pixels taken by each tile in the main-axis.
  ///
  /// If null, [childAspectRatio] is used instead.
  final double? mainAxisExtent;

  bool _debugAssertIsValid(double crossAxisExtent) {
    assert(crossAxisExtent > 0.0);
    assert(maxCrossAxisExtent > 0.0);
    assert(mainAxisSpacing >= 0.0);
    assert(crossAxisSpacing >= 0.0);
    assert(childAspectRatio > 0.0);
    return true;
  }

  @override
  SliverGridLayout getLayout(SliverConstraints constraints) {
    assert(_debugAssertIsValid(constraints.crossAxisExtent));
    int crossAxisCount =
        (constraints.crossAxisExtent / (maxCrossAxisExtent + crossAxisSpacing)).ceil();
    // Ensure a minimum count of 1, can be zero and result in an infinite extent
    // below when the window size is 0.
    crossAxisCount = math.max(1, crossAxisCount);
    final double usableCrossAxisExtent = math.max(
      0.0,
      constraints.crossAxisExtent - crossAxisSpacing * (crossAxisCount - 1),
    );
    final double childCrossAxisExtent = usableCrossAxisExtent / crossAxisCount;
    final double childMainAxisExtent = mainAxisExtent ?? childCrossAxisExtent / childAspectRatio;
    return SliverGridRegularTileLayout(
      crossAxisCount: crossAxisCount,
      mainAxisStride: childMainAxisExtent + mainAxisSpacing,
      crossAxisStride: childCrossAxisExtent + crossAxisSpacing,
      childMainAxisExtent: childMainAxisExtent,
      childCrossAxisExtent: childCrossAxisExtent,
      reverseCrossAxis: axisDirectionIsReversed(constraints.crossAxisDirection),
    );
  }

  @override
  bool shouldRelayout(SliverGridDelegateWithMaxCrossAxisExtent oldDelegate) {
    return oldDelegate.maxCrossAxisExtent != maxCrossAxisExtent ||
        oldDelegate.mainAxisSpacing != mainAxisSpacing ||
        oldDelegate.crossAxisSpacing != crossAxisSpacing ||
        oldDelegate.childAspectRatio != childAspectRatio ||
        oldDelegate.mainAxisExtent != mainAxisExtent;
  }
}

/// Parent data structure used by [RenderSliverGrid].
class SliverGridParentData extends SliverMultiBoxAdaptorParentData {
  /// The offset of the child in the non-scrolling axis.
  ///
  /// If the scroll axis is vertical, this offset is from the left-most edge of
  /// the parent to the left-most edge of the child. If the scroll axis is
  /// horizontal, this offset is from the top-most edge of the parent to the
  /// top-most edge of the child.
  double? crossAxisOffset;

  @override
  String toString() => 'crossAxisOffset=$crossAxisOffset; ${super.toString()}';
}

/// A sliver that places multiple box children in a two dimensional arrangement.
///
/// [RenderSliverGrid] places its children in arbitrary positions determined by
/// [gridDelegate]. Each child is forced to have the size specified by the
/// [gridDelegate].
///
/// See also:
///
///  * [RenderSliverList], which places its children in a linear
///    array.
///  * [RenderSliverFixedExtentList], which places its children in a linear
///    array with a fixed extent in the main axis.
class RenderSliverGrid extends RenderSliverMultiBoxAdaptor {
  /// Creates a sliver that contains multiple box children that whose size and
  /// position are determined by a delegate.
  RenderSliverGrid({required super.childManager, required SliverGridDelegate gridDelegate})
    : _gridDelegate = gridDelegate;

  @override
  void setupParentData(RenderObject child) {
    if (child.parentData is! SliverGridParentData) {
      child.parentData = SliverGridParentData();
    }
  }

  /// The delegate that controls the size and position of the children.
  SliverGridDelegate get gridDelegate => _gridDelegate;
  SliverGridDelegate _gridDelegate;
  set gridDelegate(SliverGridDelegate value) {
    if (_gridDelegate == value) {
      return;
    }
    if (value.runtimeType != _gridDelegate.runtimeType || value.shouldRelayout(_gridDelegate)) {
      markNeedsLayout();
    }
    _gridDelegate = value;
  }

  @override
  double childCrossAxisPosition(RenderBox child) {
    final SliverGridParentData childParentData = child.parentData! as SliverGridParentData;
    return childParentData.crossAxisOffset!;
  }

  @override
  void performLayout() {
    final SliverConstraints constraints = this.constraints;
    childManager.didStartLayout();
    childManager.setDidUnderflow(false);

    final double scrollOffset = constraints.scrollOffset + constraints.cacheOrigin;
    assert(scrollOffset >= 0.0);
    final double remainingExtent = constraints.remainingCacheExtent;
    assert(remainingExtent >= 0.0);
    final double targetEndScrollOffset = scrollOffset + remainingExtent;

    final SliverGridLayout layout = _gridDelegate.getLayout(constraints);

    final int firstIndex = layout.getMinChildIndexForScrollOffset(scrollOffset);
    final int? targetLastIndex =
        targetEndScrollOffset.isFinite
            ? layout.getMaxChildIndexForScrollOffset(targetEndScrollOffset)
            : null;
    if (firstChild != null) {
      final int leadingGarbage = calculateLeadingGarbage(firstIndex: firstIndex);
      final int trailingGarbage =
          targetLastIndex != null ? calculateTrailingGarbage(lastIndex: targetLastIndex) : 0;
      collectGarbage(leadingGarbage, trailingGarbage);
    } else {
      collectGarbage(0, 0);
    }

    final SliverGridGeometry firstChildGridGeometry = layout.getGeometryForChildIndex(firstIndex);

    if (firstChild == null) {
      if (!addInitialChild(index: firstIndex, layoutOffset: firstChildGridGeometry.scrollOffset)) {
        // There are either no children, or we are past the end of all our children.
        final double max = layout.computeMaxScrollOffset(childManager.childCount);
        geometry = SliverGeometry(scrollExtent: max, maxPaintExtent: max);
        childManager.didFinishLayout();
        return;
      }
    }

    final double leadingScrollOffset = firstChildGridGeometry.scrollOffset;
    double trailingScrollOffset = firstChildGridGeometry.trailingScrollOffset;
    RenderBox? trailingChildWithLayout;
    bool reachedEnd = false;

    for (int index = indexOf(firstChild!) - 1; index >= firstIndex; --index) {
      final SliverGridGeometry gridGeometry = layout.getGeometryForChildIndex(index);
      final RenderBox child =
          insertAndLayoutLeadingChild(gridGeometry.getBoxConstraints(constraints))!;
      final SliverGridParentData childParentData = child.parentData! as SliverGridParentData;
      childParentData.layoutOffset = gridGeometry.scrollOffset;
      childParentData.crossAxisOffset = gridGeometry.crossAxisOffset;
      assert(childParentData.index == index);
      trailingChildWithLayout ??= child;
      trailingScrollOffset = math.max(trailingScrollOffset, gridGeometry.trailingScrollOffset);
    }

    if (trailingChildWithLayout == null) {
      firstChild!.layout(firstChildGridGeometry.getBoxConstraints(constraints));
      final SliverGridParentData childParentData = firstChild!.parentData! as SliverGridParentData;
      childParentData.layoutOffset = firstChildGridGeometry.scrollOffset;
      childParentData.crossAxisOffset = firstChildGridGeometry.crossAxisOffset;
      trailingChildWithLayout = firstChild;
    }

    for (
      int index = indexOf(trailingChildWithLayout!) + 1;
      targetLastIndex == null || index <= targetLastIndex;
      ++index
    ) {
      final SliverGridGeometry gridGeometry = layout.getGeometryForChildIndex(index);
      final BoxConstraints childConstraints = gridGeometry.getBoxConstraints(constraints);
      RenderBox? child = childAfter(trailingChildWithLayout!);
      if (child == null || indexOf(child) != index) {
        child = insertAndLayoutChild(childConstraints, after: trailingChildWithLayout);
        if (child == null) {
          reachedEnd = true;
          // We have run out of children.
          break;
        }
      } else {
        child.layout(childConstraints);
      }
      trailingChildWithLayout = child;
      final SliverGridParentData childParentData = child.parentData! as SliverGridParentData;
      childParentData.layoutOffset = gridGeometry.scrollOffset;
      childParentData.crossAxisOffset = gridGeometry.crossAxisOffset;
      assert(childParentData.index == index);
      trailingScrollOffset = math.max(trailingScrollOffset, gridGeometry.trailingScrollOffset);
    }

    final int lastIndex = indexOf(lastChild!);

    assert(debugAssertChildListIsNonEmptyAndContiguous());
    assert(indexOf(firstChild!) == firstIndex);
    assert(targetLastIndex == null || lastIndex <= targetLastIndex);

    final double estimatedTotalExtent =
        reachedEnd
            ? trailingScrollOffset
            : childManager.estimateMaxScrollOffset(
              constraints,
              firstIndex: firstIndex,
              lastIndex: lastIndex,
              leadingScrollOffset: leadingScrollOffset,
              trailingScrollOffset: trailingScrollOffset,
            );
    final double paintExtent = calculatePaintOffset(
      constraints,
      from: math.min(constraints.scrollOffset, leadingScrollOffset),
      to: trailingScrollOffset,
    );
    final double cacheExtent = calculateCacheOffset(
      constraints,
      from: leadingScrollOffset,
      to: trailingScrollOffset,
    );

    geometry = SliverGeometry(
      scrollExtent: estimatedTotalExtent,
      paintExtent: paintExtent,
      maxPaintExtent: estimatedTotalExtent,
      cacheExtent: cacheExtent,
      hasVisualOverflow:
          estimatedTotalExtent > paintExtent ||
          constraints.scrollOffset > 0.0 ||
          constraints.overlap != 0.0,
    );

    // We may have started the layout while scrolled to the end, which
    // would not expose a new child.
    if (estimatedTotalExtent == trailingScrollOffset) {
      childManager.setDidUnderflow(true);
    }
    childManager.didFinishLayout();
  }
}
