// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'single_child_scroll_view.dart';
library;

import 'dart:math' as math;

import 'package:flutter/rendering.dart';

import 'basic.dart';
import 'framework.dart';

/// Defines the horizontal alignment of [OverflowBar] children
/// when they're laid out in an overflow column.
///
/// This value must be interpreted relative to the ambient
/// [TextDirection].
enum OverflowBarAlignment {
  /// Each child is left-aligned for [TextDirection.ltr],
  /// right-aligned for [TextDirection.rtl].
  start,

  /// Each child is right-aligned for [TextDirection.ltr],
  /// left-aligned for [TextDirection.rtl].
  end,

  /// Each child is horizontally centered.
  center,
}

/// A widget that lays out its [children] in a row unless they
/// "overflow" the available horizontal space, in which case it lays
/// them out in a column instead.
///
/// This widget's width will expand to contain its children and the
/// specified [spacing] until it overflows. The overflow column will
/// consume all of the available width. The [overflowAlignment]
/// defines how each child will be aligned within the overflow column
/// and the [overflowSpacing] defines the gap between each child.
///
/// The order that the children appear in the horizontal layout
/// is defined by the [textDirection], just like the [Row] widget.
/// If the layout overflows, then children's order within their
/// column is specified by [overflowDirection] instead.
///
/// {@tool dartpad}
/// This example defines a simple approximation of a dialog
/// layout, where the layout of the dialog's action buttons are
/// defined by an [OverflowBar]. The content is wrapped in a
/// [SingleChildScrollView], so that if overflow occurs, the
/// action buttons will still be accessible by scrolling,
/// no matter how much vertical space is available.
///
/// ** See code in examples/api/lib/widgets/overflow_bar/overflow_bar.0.dart **
/// {@end-tool}
class OverflowBar extends MultiChildRenderObjectWidget {
  /// Constructs an OverflowBar.
  const OverflowBar({
    super.key,
    this.spacing = 0.0,
    this.alignment,
    this.overflowSpacing = 0.0,
    this.overflowAlignment = OverflowBarAlignment.start,
    this.overflowDirection = VerticalDirection.down,
    this.textDirection,
    super.children,
  });

  /// The width of the gap between [children] for the default
  /// horizontal layout.
  ///
  /// If the horizontal layout overflows, then [overflowSpacing] is
  /// used instead.
  ///
  /// Defaults to 0.0.
  final double spacing;

  /// Defines the [children]'s horizontal layout according to the same
  /// rules as for [Row.mainAxisAlignment].
  ///
  /// If this property is non-null, and the [children], separated by
  /// [spacing], fit within the available width, then the overflow
  /// bar will be as wide as possible. If the children do not fit
  /// within the available width, then this property is ignored and
  /// [overflowAlignment] applies instead.
  ///
  /// If this property is null (the default) then the overflow bar
  /// will be no wider than needed to layout the [children] separated
  /// by [spacing], modulo the incoming constraints.
  ///
  /// If [alignment] is one of [MainAxisAlignment.spaceAround],
  /// [MainAxisAlignment.spaceBetween], or
  /// [MainAxisAlignment.spaceEvenly], then the [spacing] parameter is
  /// only used to see if the horizontal layout will overflow.
  ///
  /// Defaults to null.
  ///
  /// See also:
  ///
  ///  * [overflowAlignment], the horizontal alignment of the [children] within
  ///    the vertical "overflow" layout.
  ///
  final MainAxisAlignment? alignment;

  /// The height of the gap between [children] in the vertical
  /// "overflow" layout.
  ///
  /// This parameter is only used if the horizontal layout overflows, i.e.
  /// if there isn't enough horizontal room for the [children] and [spacing].
  ///
  /// Defaults to 0.0.
  ///
  /// See also:
  ///
  ///  * [spacing], The width of the gap between each pair of children
  ///    for the default horizontal layout.
  final double overflowSpacing;

  /// The horizontal alignment of the [children] within the vertical
  /// "overflow" layout.
  ///
  /// This parameter is only used if the horizontal layout overflows, i.e.
  /// if there isn't enough horizontal room for the [children] and [spacing].
  /// In that case the overflow bar will expand to fill the available
  /// width and it will layout its [children] in a column. The
  /// horizontal alignment of each child within that column is
  /// defined by this parameter and the [textDirection]. If the
  /// [textDirection] is [TextDirection.ltr] then each child will be
  /// aligned with the left edge of the available space for
  /// [OverflowBarAlignment.start], with the right edge of the
  /// available space for [OverflowBarAlignment.end]. Similarly, if the
  /// [textDirection] is [TextDirection.rtl] then each child will
  /// be aligned with the right edge of the available space for
  /// [OverflowBarAlignment.start], and with the left edge of the
  /// available space for [OverflowBarAlignment.end]. For
  /// [OverflowBarAlignment.center] each child is horizontally
  /// centered within the available space.
  ///
  /// Defaults to [OverflowBarAlignment.start].
  ///
  /// See also:
  ///
  ///  * [alignment], which defines the [children]'s horizontal layout
  ///    (according to the same rules as for [Row.mainAxisAlignment]) when
  ///    the children, separated by [spacing], fit within the available space.
  ///  * [overflowDirection], which defines the order that the
  ///    [OverflowBar]'s children appear in, if the horizontal layout
  ///    overflows.
  final OverflowBarAlignment overflowAlignment;

  /// Defines the order that the [children] appear in, if
  /// the horizontal layout overflows.
  ///
  /// This parameter is only used if the horizontal layout overflows, i.e.
  /// if there isn't enough horizontal room for the [children] and [spacing].
  ///
  /// If the children do not fit into a single row, then they
  /// are arranged in a column. The first child is at the top of the
  /// column if this property is set to [VerticalDirection.down], since it
  /// "starts" at the top and "ends" at the bottom. On the other hand,
  /// the first child will be at the bottom of the column if this
  /// property is set to [VerticalDirection.up], since it "starts" at the
  /// bottom and "ends" at the top.
  ///
  /// Defaults to [VerticalDirection.down].
  ///
  /// See also:
  ///
  ///  * [overflowAlignment], which defines the horizontal alignment
  ///    of the children within the vertical "overflow" layout.
  final VerticalDirection overflowDirection;

  /// Determines the order that the [children] appear in for the default
  /// horizontal layout, and the interpretation of
  /// [OverflowBarAlignment.start] and [OverflowBarAlignment.end] for
  /// the vertical overflow layout.
  ///
  /// For the default horizontal layout, if [textDirection] is
  /// [TextDirection.rtl] then the last child is laid out first. If
  /// [textDirection] is [TextDirection.ltr] then the first child is
  /// laid out first.
  ///
  /// If this parameter is null, then the value of
  /// `Directionality.of(context)` is used.
  ///
  /// See also:
  ///
  ///  * [overflowDirection], which defines the order that the
  ///    [OverflowBar]'s children appear in, if the horizontal layout
  ///    overflows.
  ///  * [Directionality], which defines the ambient directionality of
  ///    text and text-direction-sensitive render objects.
  final TextDirection? textDirection;

  @override
  RenderObject createRenderObject(BuildContext context) {
    return _RenderOverflowBar(
      spacing: spacing,
      alignment: alignment,
      overflowSpacing: overflowSpacing,
      overflowAlignment: overflowAlignment,
      overflowDirection: overflowDirection,
      textDirection: textDirection ?? Directionality.of(context),
    );
  }

  @override
  void updateRenderObject(BuildContext context, RenderObject renderObject) {
    (renderObject as _RenderOverflowBar)
      ..spacing = spacing
      ..alignment = alignment
      ..overflowSpacing = overflowSpacing
      ..overflowAlignment = overflowAlignment
      ..overflowDirection = overflowDirection
      ..textDirection = textDirection ?? Directionality.of(context);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('spacing', spacing, defaultValue: 0));
    properties.add(EnumProperty<MainAxisAlignment>('alignment', alignment, defaultValue: null));
    properties.add(DoubleProperty('overflowSpacing', overflowSpacing, defaultValue: 0));
    properties.add(
      EnumProperty<OverflowBarAlignment>(
        'overflowAlignment',
        overflowAlignment,
        defaultValue: OverflowBarAlignment.start,
      ),
    );
    properties.add(
      EnumProperty<VerticalDirection>(
        'overflowDirection',
        overflowDirection,
        defaultValue: VerticalDirection.down,
      ),
    );
    properties.add(EnumProperty<TextDirection>('textDirection', textDirection, defaultValue: null));
  }
}

class _OverflowBarParentData extends ContainerBoxParentData<RenderBox> {}

class _RenderOverflowBar extends RenderBox
    with
        ContainerRenderObjectMixin<RenderBox, _OverflowBarParentData>,
        RenderBoxContainerDefaultsMixin<RenderBox, _OverflowBarParentData> {
  _RenderOverflowBar({
    List<RenderBox>? children,
    double spacing = 0.0,
    MainAxisAlignment? alignment,
    double overflowSpacing = 0.0,
    OverflowBarAlignment overflowAlignment = OverflowBarAlignment.start,
    VerticalDirection overflowDirection = VerticalDirection.down,
    required TextDirection textDirection,
  }) : _spacing = spacing,
       _alignment = alignment,
       _overflowSpacing = overflowSpacing,
       _overflowAlignment = overflowAlignment,
       _overflowDirection = overflowDirection,
       _textDirection = textDirection {
    addAll(children);
  }

  double get spacing => _spacing;
  double _spacing;
  set spacing(double value) {
    if (_spacing == value) {
      return;
    }
    _spacing = value;
    markNeedsLayout();
  }

  MainAxisAlignment? get alignment => _alignment;
  MainAxisAlignment? _alignment;
  set alignment(MainAxisAlignment? value) {
    if (_alignment == value) {
      return;
    }
    _alignment = value;
    markNeedsLayout();
  }

  double get overflowSpacing => _overflowSpacing;
  double _overflowSpacing;
  set overflowSpacing(double value) {
    if (_overflowSpacing == value) {
      return;
    }
    _overflowSpacing = value;
    markNeedsLayout();
  }

  OverflowBarAlignment get overflowAlignment => _overflowAlignment;
  OverflowBarAlignment _overflowAlignment;
  set overflowAlignment(OverflowBarAlignment value) {
    if (_overflowAlignment == value) {
      return;
    }
    _overflowAlignment = value;
    markNeedsLayout();
  }

  VerticalDirection get overflowDirection => _overflowDirection;
  VerticalDirection _overflowDirection;
  set overflowDirection(VerticalDirection value) {
    if (_overflowDirection == value) {
      return;
    }
    _overflowDirection = value;
    markNeedsLayout();
  }

  TextDirection get textDirection => _textDirection;
  TextDirection _textDirection;
  set textDirection(TextDirection value) {
    if (_textDirection == value) {
      return;
    }
    _textDirection = value;
    markNeedsLayout();
  }

  @override
  void setupParentData(RenderBox child) {
    if (child.parentData is! _OverflowBarParentData) {
      child.parentData = _OverflowBarParentData();
    }
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    RenderBox? child = firstChild;
    if (child == null) {
      return 0;
    }
    double barWidth = 0.0;
    while (child != null) {
      barWidth += child.getMinIntrinsicWidth(double.infinity);
      child = childAfter(child);
    }
    barWidth += spacing * (childCount - 1);

    double height = 0.0;
    if (barWidth > width) {
      child = firstChild;
      while (child != null) {
        height += child.getMinIntrinsicHeight(width);
        child = childAfter(child);
      }
      return height + overflowSpacing * (childCount - 1);
    } else {
      child = firstChild;
      while (child != null) {
        height = math.max(height, child.getMinIntrinsicHeight(width));
        child = childAfter(child);
      }
      return height;
    }
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    RenderBox? child = firstChild;
    if (child == null) {
      return 0;
    }
    double barWidth = 0.0;
    while (child != null) {
      barWidth += child.getMinIntrinsicWidth(double.infinity);
      child = childAfter(child);
    }
    barWidth += spacing * (childCount - 1);

    double height = 0.0;
    if (barWidth > width) {
      child = firstChild;
      while (child != null) {
        height += child.getMaxIntrinsicHeight(width);
        child = childAfter(child);
      }
      return height + overflowSpacing * (childCount - 1);
    } else {
      child = firstChild;
      while (child != null) {
        height = math.max(height, child.getMaxIntrinsicHeight(width));
        child = childAfter(child);
      }
      return height;
    }
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    RenderBox? child = firstChild;
    if (child == null) {
      return 0;
    }
    double width = 0.0;
    while (child != null) {
      width += child.getMinIntrinsicWidth(double.infinity);
      child = childAfter(child);
    }
    return width + spacing * (childCount - 1);
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    RenderBox? child = firstChild;
    if (child == null) {
      return 0;
    }
    double width = 0.0;
    while (child != null) {
      width += child.getMaxIntrinsicWidth(double.infinity);
      child = childAfter(child);
    }
    return width + spacing * (childCount - 1);
  }

  @override
  double? computeDistanceToActualBaseline(TextBaseline baseline) {
    return defaultComputeDistanceToHighestActualBaseline(baseline);
  }

  @override
  double? computeDryBaseline(BoxConstraints constraints, TextBaseline baseline) {
    final BoxConstraints childConstraints = constraints.loosen();

    final (
      RenderBox? Function(RenderBox) next,
      RenderBox? startChild,
    ) = switch (overflowDirection) {
      VerticalDirection.down => (childAfter, firstChild),
      VerticalDirection.up => (childBefore, lastChild),
    };

    double maxChildHeight = 0.0;
    double y = 0.0;
    double childrenWidth = 0.0;
    BaselineOffset minHorizontalBaseline = BaselineOffset.noBaseline;
    BaselineOffset verticalBaseline = BaselineOffset.noBaseline;

    for (RenderBox? child = startChild; child != null; child = next(child)) {
      final Size childSize = child.getDryLayout(childConstraints);
      final double heightDiff = childSize.height - maxChildHeight;
      if (heightDiff > 0) {
        minHorizontalBaseline += heightDiff / 2;
        maxChildHeight = childSize.height;
      }

      final BaselineOffset baselineOffset = BaselineOffset(
        child.getDryBaseline(childConstraints, baseline),
      );
      if (baselineOffset != null) {
        verticalBaseline ??= baselineOffset + y;
        minHorizontalBaseline = minHorizontalBaseline.minOf(
          baselineOffset + (maxChildHeight - childSize.height),
        );
      }
      y += childSize.height + overflowSpacing;
      childrenWidth += childSize.width;
    }

    assert((verticalBaseline == null) == (minHorizontalBaseline == null));
    return childrenWidth + spacing * (childCount - 1) > constraints.maxWidth
        ? verticalBaseline.offset
        : minHorizontalBaseline.offset;
  }

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    RenderBox? child = firstChild;
    if (child == null) {
      return constraints.smallest;
    }
    final BoxConstraints childConstraints = constraints.loosen();
    double childrenWidth = 0.0;
    double maxChildHeight = 0.0;
    double y = 0.0;
    while (child != null) {
      final Size childSize = child.getDryLayout(childConstraints);
      childrenWidth += childSize.width;
      maxChildHeight = math.max(maxChildHeight, childSize.height);
      y += childSize.height + overflowSpacing;
      child = childAfter(child);
    }
    final double actualWidth = childrenWidth + spacing * (childCount - 1);
    if (actualWidth > constraints.maxWidth) {
      return constraints.constrain(Size(constraints.maxWidth, y - overflowSpacing));
    } else {
      final double overallWidth = alignment == null ? actualWidth : constraints.maxWidth;
      return constraints.constrain(Size(overallWidth, maxChildHeight));
    }
  }

  @override
  void performLayout() {
    RenderBox? child = firstChild;
    if (child == null) {
      size = constraints.smallest;
      return;
    }

    final BoxConstraints childConstraints = constraints.loosen();
    double childrenWidth = 0;
    double maxChildHeight = 0;
    double maxChildWidth = 0;

    while (child != null) {
      child.layout(childConstraints, parentUsesSize: true);
      childrenWidth += child.size.width;
      maxChildHeight = math.max(maxChildHeight, child.size.height);
      maxChildWidth = math.max(maxChildWidth, child.size.width);
      child = childAfter(child);
    }

    final bool rtl = textDirection == TextDirection.rtl;
    final double actualWidth = childrenWidth + spacing * (childCount - 1);

    if (actualWidth > constraints.maxWidth) {
      // Overflow vertical layout
      child = overflowDirection == VerticalDirection.down ? firstChild : lastChild;
      RenderBox? nextChild() =>
          overflowDirection == VerticalDirection.down ? childAfter(child!) : childBefore(child!);
      double y = 0;
      while (child != null) {
        final _OverflowBarParentData childParentData = child.parentData! as _OverflowBarParentData;
        final double x = switch (overflowAlignment) {
          OverflowBarAlignment.center => (constraints.maxWidth - child.size.width) / 2,
          OverflowBarAlignment.start => rtl ? constraints.maxWidth - child.size.width : 0,
          OverflowBarAlignment.end => rtl ? 0 : constraints.maxWidth - child.size.width,
        };
        childParentData.offset = Offset(x, y);
        y += child.size.height + overflowSpacing;
        child = nextChild();
      }
      size = constraints.constrain(Size(constraints.maxWidth, y - overflowSpacing));
    } else {
      // Default horizontal layout
      child = firstChild;
      final double firstChildWidth = child!.size.width;
      final double overallWidth = alignment == null ? actualWidth : constraints.maxWidth;
      size = constraints.constrain(Size(overallWidth, maxChildHeight));

      late double x; // initial value: origin of the first child
      double layoutSpacing = spacing; // space between children
      switch (alignment) {
        case null:
          x = rtl ? size.width - firstChildWidth : 0;
        case MainAxisAlignment.start:
          x = rtl ? size.width - firstChildWidth : 0;
        case MainAxisAlignment.center:
          final double halfRemainingWidth = (size.width - actualWidth) / 2;
          x = rtl ? size.width - halfRemainingWidth - firstChildWidth : halfRemainingWidth;
        case MainAxisAlignment.end:
          x = rtl ? actualWidth - firstChildWidth : size.width - actualWidth;
        case MainAxisAlignment.spaceBetween:
          layoutSpacing = (size.width - childrenWidth) / (childCount - 1);
          x = rtl ? size.width - firstChildWidth : 0;
        case MainAxisAlignment.spaceAround:
          layoutSpacing = childCount > 0 ? (size.width - childrenWidth) / childCount : 0;
          x = rtl ? size.width - layoutSpacing / 2 - firstChildWidth : layoutSpacing / 2;
        case MainAxisAlignment.spaceEvenly:
          layoutSpacing = (size.width - childrenWidth) / (childCount + 1);
          x = rtl ? size.width - layoutSpacing - firstChildWidth : layoutSpacing;
      }

      while (child != null) {
        final _OverflowBarParentData childParentData = child.parentData! as _OverflowBarParentData;
        childParentData.offset = Offset(x, (maxChildHeight - child.size.height) / 2);
        // x is the horizontal origin of child. To advance x to the next child's
        // origin for LTR: add the width of the current child. To advance x to
        // the origin of the next child for RTL: subtract the width of the next
        // child (if there is one).
        if (!rtl) {
          x += child.size.width + layoutSpacing;
        }
        child = childAfter(child);
        if (rtl && child != null) {
          x -= child.size.width + layoutSpacing;
        }
      }
    }
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    return defaultHitTestChildren(result, position: position);
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    defaultPaint(context, offset);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('spacing', spacing, defaultValue: 0));
    properties.add(DoubleProperty('overflowSpacing', overflowSpacing, defaultValue: 0));
    properties.add(
      EnumProperty<OverflowBarAlignment>(
        'overflowAlignment',
        overflowAlignment,
        defaultValue: OverflowBarAlignment.start,
      ),
    );
    properties.add(
      EnumProperty<VerticalDirection>(
        'overflowDirection',
        overflowDirection,
        defaultValue: VerticalDirection.down,
      ),
    );
    properties.add(EnumProperty<TextDirection>('textDirection', textDirection, defaultValue: null));
  }
}
