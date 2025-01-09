// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart';

import 'box.dart';
import 'layer.dart';
import 'layout_helper.dart';
import 'object.dart';

typedef _NextChild = RenderBox? Function(RenderBox child);
typedef _PositionChild = void Function(Offset offset, RenderBox child);
typedef _GetChildSize = Size Function(RenderBox child);
// A 2D vector that uses a [RenderWrap]'s main axis and cross axis as its first and second coordinate axes.
// It represents the same vector as (double mainAxisExtent, double crossAxisExtent).
extension type const _AxisSize._(Size _size) {
  _AxisSize({required double mainAxisExtent, required double crossAxisExtent})
    : this._(Size(mainAxisExtent, crossAxisExtent));
  _AxisSize.fromSize({required Size size, required Axis direction})
    : this._(_convert(size, direction));

  static const _AxisSize empty = _AxisSize._(Size.zero);

  static Size _convert(Size size, Axis direction) {
    return switch (direction) {
      Axis.horizontal => size,
      Axis.vertical => size.flipped,
    };
  }

  double get mainAxisExtent => _size.width;
  double get crossAxisExtent => _size.height;

  Size toSize(Axis direction) => _convert(_size, direction);

  _AxisSize applyConstraints(BoxConstraints constraints, Axis direction) {
    final BoxConstraints effectiveConstraints = switch (direction) {
      Axis.horizontal => constraints,
      Axis.vertical => constraints.flipped,
    };
    return _AxisSize._(effectiveConstraints.constrain(_size));
  }

  _AxisSize get flipped => _AxisSize._(_size.flipped);
  _AxisSize operator +(_AxisSize other) => _AxisSize._(
    Size(_size.width + other._size.width, math.max(_size.height, other._size.height)),
  );
  _AxisSize operator -(_AxisSize other) =>
      _AxisSize._(Size(_size.width - other._size.width, _size.height - other._size.height));
}

/// How [Wrap] should align objects.
///
/// Used both to align children within a run in the main axis as well as to
/// align the runs themselves in the cross axis.
enum WrapAlignment {
  /// Place the objects as close to the start of the axis as possible.
  ///
  /// If this value is used in a horizontal direction, a [TextDirection] must be
  /// available to determine if the start is the left or the right.
  ///
  /// If this value is used in a vertical direction, a [VerticalDirection] must be
  /// available to determine if the start is the top or the bottom.
  start,

  /// Place the objects as close to the end of the axis as possible.
  ///
  /// If this value is used in a horizontal direction, a [TextDirection] must be
  /// available to determine if the end is the left or the right.
  ///
  /// If this value is used in a vertical direction, a [VerticalDirection] must be
  /// available to determine if the end is the top or the bottom.
  end,

  /// Place the objects as close to the middle of the axis as possible.
  center,

  /// Place the free space evenly between the objects.
  spaceBetween,

  /// Place the free space evenly between the objects as well as half of that
  /// space before and after the first and last objects.
  spaceAround,

  /// Place the free space evenly between the objects as well as before and
  /// after the first and last objects.
  spaceEvenly;

  (double leadingSpace, double betweenSpace) _distributeSpace(
    double freeSpace,
    double itemSpacing,
    int itemCount,
    bool flipped,
  ) {
    assert(itemCount > 0);
    return switch (this) {
      WrapAlignment.start => (flipped ? freeSpace : 0.0, itemSpacing),

      WrapAlignment.end => WrapAlignment.start._distributeSpace(
        freeSpace,
        itemSpacing,
        itemCount,
        !flipped,
      ),
      WrapAlignment.spaceBetween when itemCount < 2 => WrapAlignment.start._distributeSpace(
        freeSpace,
        itemSpacing,
        itemCount,
        flipped,
      ),

      WrapAlignment.center => (freeSpace / 2.0, itemSpacing),
      WrapAlignment.spaceBetween => (0, freeSpace / (itemCount - 1) + itemSpacing),
      WrapAlignment.spaceAround => (freeSpace / itemCount / 2, freeSpace / itemCount + itemSpacing),
      WrapAlignment.spaceEvenly => (
        freeSpace / (itemCount + 1),
        freeSpace / (itemCount + 1) + itemSpacing,
      ),
    };
  }
}

/// Who [Wrap] should align children within a run in the cross axis.
enum WrapCrossAlignment {
  /// Place the children as close to the start of the run in the cross axis as
  /// possible.
  ///
  /// If this value is used in a horizontal direction, a [TextDirection] must be
  /// available to determine if the start is the left or the right.
  ///
  /// If this value is used in a vertical direction, a [VerticalDirection] must be
  /// available to determine if the start is the top or the bottom.
  start,

  /// Place the children as close to the end of the run in the cross axis as
  /// possible.
  ///
  /// If this value is used in a horizontal direction, a [TextDirection] must be
  /// available to determine if the end is the left or the right.
  ///
  /// If this value is used in a vertical direction, a [VerticalDirection] must be
  /// available to determine if the end is the top or the bottom.
  end,

  /// Place the children as close to the middle of the run in the cross axis as
  /// possible.
  center;

  // TODO(ianh): baseline.

  WrapCrossAlignment get _flipped => switch (this) {
    WrapCrossAlignment.start => WrapCrossAlignment.end,
    WrapCrossAlignment.end => WrapCrossAlignment.start,
    WrapCrossAlignment.center => WrapCrossAlignment.center,
  };

  double get _alignment => switch (this) {
    WrapCrossAlignment.start => 0,
    WrapCrossAlignment.end => 1,
    WrapCrossAlignment.center => 0.5,
  };
}

class _RunMetrics {
  _RunMetrics(this.leadingChild, this.axisSize);

  _AxisSize axisSize;
  int childCount = 1;
  RenderBox leadingChild;

  // Look ahead, creates a new run if incorporating the child would exceed the allowed line width.
  _RunMetrics? tryAddingNewChild(
    RenderBox child,
    _AxisSize childSize,
    bool flipMainAxis,
    double spacing,
    double maxMainExtent,
  ) {
    final bool needsNewRun =
        axisSize.mainAxisExtent + childSize.mainAxisExtent + spacing - maxMainExtent >
        precisionErrorTolerance;
    if (needsNewRun) {
      return _RunMetrics(child, childSize);
    } else {
      axisSize += childSize + _AxisSize(mainAxisExtent: spacing, crossAxisExtent: 0.0);
      childCount += 1;
      if (flipMainAxis) {
        leadingChild = child;
      }
      return null;
    }
  }
}

/// Parent data for use with [RenderWrap].
class WrapParentData extends ContainerBoxParentData<RenderBox> {}

/// Displays its children in multiple horizontal or vertical runs.
///
/// A [RenderWrap] lays out each child and attempts to place the child adjacent
/// to the previous child in the main axis, given by [direction], leaving
/// [spacing] space in between. If there is not enough space to fit the child,
/// [RenderWrap] creates a new _run_ adjacent to the existing children in the
/// cross axis.
///
/// After all the children have been allocated to runs, the children within the
/// runs are positioned according to the [alignment] in the main axis and
/// according to the [crossAxisAlignment] in the cross axis.
///
/// The runs themselves are then positioned in the cross axis according to the
/// [runSpacing] and [runAlignment].
class RenderWrap extends RenderBox
    with
        ContainerRenderObjectMixin<RenderBox, WrapParentData>,
        RenderBoxContainerDefaultsMixin<RenderBox, WrapParentData> {
  /// Creates a wrap render object.
  ///
  /// By default, the wrap layout is horizontal and both the children and the
  /// runs are aligned to the start.
  RenderWrap({
    List<RenderBox>? children,
    Axis direction = Axis.horizontal,
    WrapAlignment alignment = WrapAlignment.start,
    double spacing = 0.0,
    WrapAlignment runAlignment = WrapAlignment.start,
    double runSpacing = 0.0,
    WrapCrossAlignment crossAxisAlignment = WrapCrossAlignment.start,
    TextDirection? textDirection,
    VerticalDirection verticalDirection = VerticalDirection.down,
    Clip clipBehavior = Clip.none,
  }) : _direction = direction,
       _alignment = alignment,
       _spacing = spacing,
       _runAlignment = runAlignment,
       _runSpacing = runSpacing,
       _crossAxisAlignment = crossAxisAlignment,
       _textDirection = textDirection,
       _verticalDirection = verticalDirection,
       _clipBehavior = clipBehavior {
    addAll(children);
  }

  /// The direction to use as the main axis.
  ///
  /// For example, if [direction] is [Axis.horizontal], the default, the
  /// children are placed adjacent to one another in a horizontal run until the
  /// available horizontal space is consumed, at which point a subsequent
  /// children are placed in a new run vertically adjacent to the previous run.
  Axis get direction => _direction;
  Axis _direction;
  set direction(Axis value) {
    if (_direction == value) {
      return;
    }
    _direction = value;
    markNeedsLayout();
  }

  /// How the children within a run should be placed in the main axis.
  ///
  /// For example, if [alignment] is [WrapAlignment.center], the children in
  /// each run are grouped together in the center of their run in the main axis.
  ///
  /// Defaults to [WrapAlignment.start].
  ///
  /// See also:
  ///
  ///  * [runAlignment], which controls how the runs are placed relative to each
  ///    other in the cross axis.
  ///  * [crossAxisAlignment], which controls how the children within each run
  ///    are placed relative to each other in the cross axis.
  WrapAlignment get alignment => _alignment;
  WrapAlignment _alignment;
  set alignment(WrapAlignment value) {
    if (_alignment == value) {
      return;
    }
    _alignment = value;
    markNeedsLayout();
  }

  /// How much space to place between children in a run in the main axis.
  ///
  /// For example, if [spacing] is 10.0, the children will be spaced at least
  /// 10.0 logical pixels apart in the main axis.
  ///
  /// If there is additional free space in a run (e.g., because the wrap has a
  /// minimum size that is not filled or because some runs are longer than
  /// others), the additional free space will be allocated according to the
  /// [alignment].
  ///
  /// Defaults to 0.0.
  double get spacing => _spacing;
  double _spacing;
  set spacing(double value) {
    if (_spacing == value) {
      return;
    }
    _spacing = value;
    markNeedsLayout();
  }

  /// How the runs themselves should be placed in the cross axis.
  ///
  /// For example, if [runAlignment] is [WrapAlignment.center], the runs are
  /// grouped together in the center of the overall [RenderWrap] in the cross
  /// axis.
  ///
  /// Defaults to [WrapAlignment.start].
  ///
  /// See also:
  ///
  ///  * [alignment], which controls how the children within each run are placed
  ///    relative to each other in the main axis.
  ///  * [crossAxisAlignment], which controls how the children within each run
  ///    are placed relative to each other in the cross axis.
  WrapAlignment get runAlignment => _runAlignment;
  WrapAlignment _runAlignment;
  set runAlignment(WrapAlignment value) {
    if (_runAlignment == value) {
      return;
    }
    _runAlignment = value;
    markNeedsLayout();
  }

  /// How much space to place between the runs themselves in the cross axis.
  ///
  /// For example, if [runSpacing] is 10.0, the runs will be spaced at least
  /// 10.0 logical pixels apart in the cross axis.
  ///
  /// If there is additional free space in the overall [RenderWrap] (e.g.,
  /// because the wrap has a minimum size that is not filled), the additional
  /// free space will be allocated according to the [runAlignment].
  ///
  /// Defaults to 0.0.
  double get runSpacing => _runSpacing;
  double _runSpacing;
  set runSpacing(double value) {
    if (_runSpacing == value) {
      return;
    }
    _runSpacing = value;
    markNeedsLayout();
  }

  /// How the children within a run should be aligned relative to each other in
  /// the cross axis.
  ///
  /// For example, if this is set to [WrapCrossAlignment.end], and the
  /// [direction] is [Axis.horizontal], then the children within each
  /// run will have their bottom edges aligned to the bottom edge of the run.
  ///
  /// Defaults to [WrapCrossAlignment.start].
  ///
  /// See also:
  ///
  ///  * [alignment], which controls how the children within each run are placed
  ///    relative to each other in the main axis.
  ///  * [runAlignment], which controls how the runs are placed relative to each
  ///    other in the cross axis.
  WrapCrossAlignment get crossAxisAlignment => _crossAxisAlignment;
  WrapCrossAlignment _crossAxisAlignment;
  set crossAxisAlignment(WrapCrossAlignment value) {
    if (_crossAxisAlignment == value) {
      return;
    }
    _crossAxisAlignment = value;
    markNeedsLayout();
  }

  /// Determines the order to lay children out horizontally and how to interpret
  /// `start` and `end` in the horizontal direction.
  ///
  /// If the [direction] is [Axis.horizontal], this controls the order in which
  /// children are positioned (left-to-right or right-to-left), and the meaning
  /// of the [alignment] property's [WrapAlignment.start] and
  /// [WrapAlignment.end] values.
  ///
  /// If the [direction] is [Axis.horizontal], and either the
  /// [alignment] is either [WrapAlignment.start] or [WrapAlignment.end], or
  /// there's more than one child, then the [textDirection] must not be null.
  ///
  /// If the [direction] is [Axis.vertical], this controls the order in
  /// which runs are positioned, the meaning of the [runAlignment] property's
  /// [WrapAlignment.start] and [WrapAlignment.end] values, as well as the
  /// [crossAxisAlignment] property's [WrapCrossAlignment.start] and
  /// [WrapCrossAlignment.end] values.
  ///
  /// If the [direction] is [Axis.vertical], and either the
  /// [runAlignment] is either [WrapAlignment.start] or [WrapAlignment.end], the
  /// [crossAxisAlignment] is either [WrapCrossAlignment.start] or
  /// [WrapCrossAlignment.end], or there's more than one child, then the
  /// [textDirection] must not be null.
  TextDirection? get textDirection => _textDirection;
  TextDirection? _textDirection;
  set textDirection(TextDirection? value) {
    if (_textDirection != value) {
      _textDirection = value;
      markNeedsLayout();
    }
  }

  /// Determines the order to lay children out vertically and how to interpret
  /// `start` and `end` in the vertical direction.
  ///
  /// If the [direction] is [Axis.vertical], this controls which order children
  /// are painted in (down or up), the meaning of the [alignment] property's
  /// [WrapAlignment.start] and [WrapAlignment.end] values.
  ///
  /// If the [direction] is [Axis.vertical], and either the [alignment]
  /// is either [WrapAlignment.start] or [WrapAlignment.end], or there's
  /// more than one child, then the [verticalDirection] must not be null.
  ///
  /// If the [direction] is [Axis.horizontal], this controls the order in which
  /// runs are positioned, the meaning of the [runAlignment] property's
  /// [WrapAlignment.start] and [WrapAlignment.end] values, as well as the
  /// [crossAxisAlignment] property's [WrapCrossAlignment.start] and
  /// [WrapCrossAlignment.end] values.
  ///
  /// If the [direction] is [Axis.horizontal], and either the
  /// [runAlignment] is either [WrapAlignment.start] or [WrapAlignment.end], the
  /// [crossAxisAlignment] is either [WrapCrossAlignment.start] or
  /// [WrapCrossAlignment.end], or there's more than one child, then the
  /// [verticalDirection] must not be null.
  VerticalDirection get verticalDirection => _verticalDirection;
  VerticalDirection _verticalDirection;
  set verticalDirection(VerticalDirection value) {
    if (_verticalDirection != value) {
      _verticalDirection = value;
      markNeedsLayout();
    }
  }

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.none].
  Clip get clipBehavior => _clipBehavior;
  Clip _clipBehavior = Clip.none;
  set clipBehavior(Clip value) {
    if (value != _clipBehavior) {
      _clipBehavior = value;
      markNeedsPaint();
      markNeedsSemanticsUpdate();
    }
  }

  bool get _debugHasNecessaryDirections {
    if (firstChild != null && lastChild != firstChild) {
      // i.e. there's more than one child
      switch (direction) {
        case Axis.horizontal:
          assert(
            textDirection != null,
            'Horizontal $runtimeType with multiple children has a null textDirection, so the layout order is undefined.',
          );
        case Axis.vertical:
          break;
      }
    }
    if (alignment == WrapAlignment.start || alignment == WrapAlignment.end) {
      switch (direction) {
        case Axis.horizontal:
          assert(
            textDirection != null,
            'Horizontal $runtimeType with alignment $alignment has a null textDirection, so the alignment cannot be resolved.',
          );
        case Axis.vertical:
          break;
      }
    }
    if (runAlignment == WrapAlignment.start || runAlignment == WrapAlignment.end) {
      switch (direction) {
        case Axis.horizontal:
          break;
        case Axis.vertical:
          assert(
            textDirection != null,
            'Vertical $runtimeType with runAlignment $runAlignment has a null textDirection, so the alignment cannot be resolved.',
          );
      }
    }
    if (crossAxisAlignment == WrapCrossAlignment.start ||
        crossAxisAlignment == WrapCrossAlignment.end) {
      switch (direction) {
        case Axis.horizontal:
          break;
        case Axis.vertical:
          assert(
            textDirection != null,
            'Vertical $runtimeType with crossAxisAlignment $crossAxisAlignment has a null textDirection, so the alignment cannot be resolved.',
          );
      }
    }
    return true;
  }

  @override
  void setupParentData(RenderBox child) {
    if (child.parentData is! WrapParentData) {
      child.parentData = WrapParentData();
    }
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    switch (direction) {
      case Axis.horizontal:
        double width = 0.0;
        RenderBox? child = firstChild;
        while (child != null) {
          width = math.max(width, child.getMinIntrinsicWidth(double.infinity));
          child = childAfter(child);
        }
        return width;
      case Axis.vertical:
        return getDryLayout(BoxConstraints(maxHeight: height)).width;
    }
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    switch (direction) {
      case Axis.horizontal:
        double width = 0.0;
        RenderBox? child = firstChild;
        while (child != null) {
          width += child.getMaxIntrinsicWidth(double.infinity);
          child = childAfter(child);
        }
        return width;
      case Axis.vertical:
        return getDryLayout(BoxConstraints(maxHeight: height)).width;
    }
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    switch (direction) {
      case Axis.horizontal:
        return getDryLayout(BoxConstraints(maxWidth: width)).height;
      case Axis.vertical:
        double height = 0.0;
        RenderBox? child = firstChild;
        while (child != null) {
          height = math.max(height, child.getMinIntrinsicHeight(double.infinity));
          child = childAfter(child);
        }
        return height;
    }
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    switch (direction) {
      case Axis.horizontal:
        return getDryLayout(BoxConstraints(maxWidth: width)).height;
      case Axis.vertical:
        double height = 0.0;
        RenderBox? child = firstChild;
        while (child != null) {
          height += child.getMaxIntrinsicHeight(double.infinity);
          child = childAfter(child);
        }
        return height;
    }
  }

  @override
  double? computeDistanceToActualBaseline(TextBaseline baseline) {
    return defaultComputeDistanceToHighestActualBaseline(baseline);
  }

  double _getMainAxisExtent(Size childSize) {
    return switch (direction) {
      Axis.horizontal => childSize.width,
      Axis.vertical => childSize.height,
    };
  }

  double _getCrossAxisExtent(Size childSize) {
    return switch (direction) {
      Axis.horizontal => childSize.height,
      Axis.vertical => childSize.width,
    };
  }

  Offset _getOffset(double mainAxisOffset, double crossAxisOffset) {
    return switch (direction) {
      Axis.horizontal => Offset(mainAxisOffset, crossAxisOffset),
      Axis.vertical => Offset(crossAxisOffset, mainAxisOffset),
    };
  }

  (bool flipHorizontal, bool flipVertical) get _areAxesFlipped {
    final bool flipHorizontal = switch (textDirection ?? TextDirection.ltr) {
      TextDirection.ltr => false,
      TextDirection.rtl => true,
    };
    final bool flipVertical = switch (verticalDirection) {
      VerticalDirection.down => false,
      VerticalDirection.up => true,
    };
    return switch (direction) {
      Axis.horizontal => (flipHorizontal, flipVertical),
      Axis.vertical => (flipVertical, flipHorizontal),
    };
  }

  @override
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    if (firstChild == null) {
      return null;
    }
    final BoxConstraints childConstraints = switch (direction) {
      Axis.horizontal => BoxConstraints(maxWidth: constraints.maxWidth),
      Axis.vertical => BoxConstraints(maxHeight: constraints.maxHeight),
    };

    final (_AxisSize childrenAxisSize, List<_RunMetrics> runMetrics) = _computeRuns(
      constraints,
      ChildLayoutHelper.dryLayoutChild,
    );
    final _AxisSize containerAxisSize = childrenAxisSize.applyConstraints(constraints, direction);

    BaselineOffset baselineOffset = BaselineOffset.noBaseline;
    void findHighestBaseline(Offset offset, RenderBox child) {
      baselineOffset = baselineOffset.minOf(
        BaselineOffset(child.getDryBaseline(childConstraints, baseline)) + offset.dy,
      );
    }

    Size getChildSize(RenderBox child) => child.getDryLayout(childConstraints);
    _positionChildren(
      runMetrics,
      childrenAxisSize,
      containerAxisSize,
      findHighestBaseline,
      getChildSize,
    );
    return baselineOffset.offset;
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    return _computeDryLayout(constraints);
  }

  Size _computeDryLayout(
    BoxConstraints constraints, [
    ChildLayouter layoutChild = ChildLayoutHelper.dryLayoutChild,
  ]) {
    final (BoxConstraints childConstraints, double mainAxisLimit) = switch (direction) {
      Axis.horizontal => (BoxConstraints(maxWidth: constraints.maxWidth), constraints.maxWidth),
      Axis.vertical => (BoxConstraints(maxHeight: constraints.maxHeight), constraints.maxHeight),
    };

    double mainAxisExtent = 0.0;
    double crossAxisExtent = 0.0;
    double runMainAxisExtent = 0.0;
    double runCrossAxisExtent = 0.0;
    int childCount = 0;
    RenderBox? child = firstChild;
    while (child != null) {
      final Size childSize = layoutChild(child, childConstraints);
      final double childMainAxisExtent = _getMainAxisExtent(childSize);
      final double childCrossAxisExtent = _getCrossAxisExtent(childSize);
      // There must be at least one child before we move on to the next run.
      if (childCount > 0 && runMainAxisExtent + childMainAxisExtent + spacing > mainAxisLimit) {
        mainAxisExtent = math.max(mainAxisExtent, runMainAxisExtent);
        crossAxisExtent += runCrossAxisExtent + runSpacing;
        runMainAxisExtent = 0.0;
        runCrossAxisExtent = 0.0;
        childCount = 0;
      }
      runMainAxisExtent += childMainAxisExtent;
      runCrossAxisExtent = math.max(runCrossAxisExtent, childCrossAxisExtent);
      if (childCount > 0) {
        runMainAxisExtent += spacing;
      }
      childCount += 1;
      child = childAfter(child);
    }
    crossAxisExtent += runCrossAxisExtent;
    mainAxisExtent = math.max(mainAxisExtent, runMainAxisExtent);

    return constraints.constrain(switch (direction) {
      Axis.horizontal => Size(mainAxisExtent, crossAxisExtent),
      Axis.vertical => Size(crossAxisExtent, mainAxisExtent),
    });
  }

  static Size _getChildSize(RenderBox child) => child.size;
  static void _setChildPosition(Offset offset, RenderBox child) {
    (child.parentData! as WrapParentData).offset = offset;
  }

  bool _hasVisualOverflow = false;

  @override
  void performLayout() {
    final BoxConstraints constraints = this.constraints;
    assert(_debugHasNecessaryDirections);
    if (firstChild == null) {
      size = constraints.smallest;
      _hasVisualOverflow = false;
      return;
    }

    final (_AxisSize childrenAxisSize, List<_RunMetrics> runMetrics) = _computeRuns(
      constraints,
      ChildLayoutHelper.layoutChild,
    );
    final _AxisSize containerAxisSize = childrenAxisSize.applyConstraints(constraints, direction);
    size = containerAxisSize.toSize(direction);
    final _AxisSize freeAxisSize = containerAxisSize - childrenAxisSize;
    _hasVisualOverflow = freeAxisSize.mainAxisExtent < 0.0 || freeAxisSize.crossAxisExtent < 0.0;
    _positionChildren(
      runMetrics,
      freeAxisSize,
      containerAxisSize,
      _setChildPosition,
      _getChildSize,
    );
  }

  (_AxisSize childrenSize, List<_RunMetrics> runMetrics) _computeRuns(
    BoxConstraints constraints,
    ChildLayouter layoutChild,
  ) {
    assert(firstChild != null);
    final (BoxConstraints childConstraints, double mainAxisLimit) = switch (direction) {
      Axis.horizontal => (BoxConstraints(maxWidth: constraints.maxWidth), constraints.maxWidth),
      Axis.vertical => (BoxConstraints(maxHeight: constraints.maxHeight), constraints.maxHeight),
    };

    final (bool flipMainAxis, _) = _areAxesFlipped;
    final double spacing = this.spacing;
    final List<_RunMetrics> runMetrics = <_RunMetrics>[];

    _RunMetrics? currentRun;
    _AxisSize childrenAxisSize = _AxisSize.empty;
    for (RenderBox? child = firstChild; child != null; child = childAfter(child)) {
      final _AxisSize childSize = _AxisSize.fromSize(
        size: layoutChild(child, childConstraints),
        direction: direction,
      );
      final _RunMetrics? newRun =
          currentRun == null
              ? _RunMetrics(child, childSize)
              : currentRun.tryAddingNewChild(
                child,
                childSize,
                flipMainAxis,
                spacing,
                mainAxisLimit,
              );
      if (newRun != null) {
        runMetrics.add(newRun);
        childrenAxisSize += currentRun?.axisSize.flipped ?? _AxisSize.empty;
        currentRun = newRun;
      }
    }
    assert(runMetrics.isNotEmpty);
    final double totalRunSpacing = runSpacing * (runMetrics.length - 1);
    childrenAxisSize +=
        _AxisSize(mainAxisExtent: totalRunSpacing, crossAxisExtent: 0.0) +
        currentRun!.axisSize.flipped;
    return (childrenAxisSize.flipped, runMetrics);
  }

  void _positionChildren(
    List<_RunMetrics> runMetrics,
    _AxisSize freeAxisSize,
    _AxisSize containerAxisSize,
    _PositionChild positionChild,
    _GetChildSize getChildSize,
  ) {
    assert(runMetrics.isNotEmpty);

    final double spacing = this.spacing;

    final double crossAxisFreeSpace = math.max(0.0, freeAxisSize.crossAxisExtent);

    final (bool flipMainAxis, bool flipCrossAxis) = _areAxesFlipped;
    final WrapCrossAlignment effectiveCrossAlignment =
        flipCrossAxis ? crossAxisAlignment._flipped : crossAxisAlignment;
    final (double runLeadingSpace, double runBetweenSpace) = runAlignment._distributeSpace(
      crossAxisFreeSpace,
      runSpacing,
      runMetrics.length,
      flipCrossAxis,
    );
    final _NextChild nextChild = flipMainAxis ? childBefore : childAfter;

    double runCrossAxisOffset = runLeadingSpace;
    final Iterable<_RunMetrics> runs = flipCrossAxis ? runMetrics.reversed : runMetrics;
    for (final _RunMetrics run in runs) {
      final double runCrossAxisExtent = run.axisSize.crossAxisExtent;
      final int childCount = run.childCount;

      final double mainAxisFreeSpace = math.max(
        0.0,
        containerAxisSize.mainAxisExtent - run.axisSize.mainAxisExtent,
      );
      final (double childLeadingSpace, double childBetweenSpace) = alignment._distributeSpace(
        mainAxisFreeSpace,
        spacing,
        childCount,
        flipMainAxis,
      );

      double childMainAxisOffset = childLeadingSpace;

      int remainingChildCount = run.childCount;
      for (
        RenderBox? child = run.leadingChild;
        child != null && remainingChildCount > 0;
        child = nextChild(child), remainingChildCount -= 1
      ) {
        final _AxisSize(
          mainAxisExtent: double childMainAxisExtent,
          crossAxisExtent: double childCrossAxisExtent,
        ) = _AxisSize.fromSize(size: getChildSize(child), direction: direction);
        final double childCrossAxisOffset =
            effectiveCrossAlignment._alignment * (runCrossAxisExtent - childCrossAxisExtent);
        positionChild(
          _getOffset(childMainAxisOffset, runCrossAxisOffset + childCrossAxisOffset),
          child,
        );
        childMainAxisOffset += childMainAxisExtent + childBetweenSpace;
      }
      runCrossAxisOffset += runCrossAxisExtent + runBetweenSpace;
    }
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    return defaultHitTestChildren(result, position: position);
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    // TODO(ianh): move the debug flex overflow paint logic somewhere common so
    // it can be reused here
    if (_hasVisualOverflow && clipBehavior != Clip.none) {
      _clipRectLayer.layer = context.pushClipRect(
        needsCompositing,
        offset,
        Offset.zero & size,
        defaultPaint,
        clipBehavior: clipBehavior,
        oldLayer: _clipRectLayer.layer,
      );
    } else {
      _clipRectLayer.layer = null;
      defaultPaint(context, offset);
    }
  }

  final LayerHandle<ClipRectLayer> _clipRectLayer = LayerHandle<ClipRectLayer>();

  @override
  void dispose() {
    _clipRectLayer.layer = null;
    super.dispose();
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(EnumProperty<Axis>('direction', direction));
    properties.add(EnumProperty<WrapAlignment>('alignment', alignment));
    properties.add(DoubleProperty('spacing', spacing));
    properties.add(EnumProperty<WrapAlignment>('runAlignment', runAlignment));
    properties.add(DoubleProperty('runSpacing', runSpacing));
    properties.add(DoubleProperty('crossAxisAlignment', runSpacing));
    properties.add(EnumProperty<TextDirection>('textDirection', textDirection, defaultValue: null));
    properties.add(
      EnumProperty<VerticalDirection>(
        'verticalDirection',
        verticalDirection,
        defaultValue: VerticalDirection.down,
      ),
    );
  }
}
