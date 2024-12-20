// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'flow.dart';
/// @docImport 'proxy_box.dart';
library;

import 'dart:math' as math;
import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';

import 'box.dart';
import 'layer.dart';
import 'layout_helper.dart';
import 'object.dart';

/// An immutable 2D, axis-aligned, floating-point rectangle whose coordinates
/// are given relative to another rectangle's edges, known as the container.
/// Since the dimensions of the rectangle are relative to those of the
/// container, this class has no width and height members. To determine the
/// width or height of the rectangle, convert it to a [Rect] using [toRect()]
/// (passing the container's own Rect), and then examine that object.
@immutable
class RelativeRect {
  /// Creates a RelativeRect with the given values.
  const RelativeRect.fromLTRB(this.left, this.top, this.right, this.bottom);

  /// Creates a RelativeRect from a Rect and a Size. The Rect (first argument)
  /// and the RelativeRect (the output) are in the coordinate space of the
  /// rectangle described by the Size, with 0,0 being at the top left.
  RelativeRect.fromSize(Rect rect, Size container)
    : left = rect.left,
      top = rect.top,
      right = container.width - rect.right,
      bottom = container.height - rect.bottom;

  /// Creates a RelativeRect from two Rects. The second Rect provides the
  /// container, the first provides the rectangle, in the same coordinate space,
  /// that is to be converted to a RelativeRect. The output will be in the
  /// container's coordinate space.
  ///
  /// For example, if the top left of the rect is at 0,0, and the top left of
  /// the container is at 100,100, then the top left of the output will be at
  /// -100,-100.
  ///
  /// If the first rect is actually in the container's coordinate space, then
  /// use [RelativeRect.fromSize] and pass the container's size as the second
  /// argument instead.
  RelativeRect.fromRect(Rect rect, Rect container)
    : left = rect.left - container.left,
      top = rect.top - container.top,
      right = container.right - rect.right,
      bottom = container.bottom - rect.bottom;

  /// Creates a RelativeRect from horizontal position using `start` and `end`
  /// rather than `left` and `right`.
  ///
  /// If `textDirection` is [TextDirection.rtl], then the `start` argument is
  /// used for the [right] property and the `end` argument is used for the
  /// [left] property. Otherwise, if `textDirection` is [TextDirection.ltr],
  /// then the `start` argument is used for the [left] property and the `end`
  /// argument is used for the [right] property.
  factory RelativeRect.fromDirectional({
    required TextDirection textDirection,
    required double start,
    required double top,
    required double end,
    required double bottom,
  }) {
    final (double left, double right) = switch (textDirection) {
      TextDirection.rtl => (end, start),
      TextDirection.ltr => (start, end),
    };
    return RelativeRect.fromLTRB(left, top, right, bottom);
  }

  /// A rect that covers the entire container.
  static const RelativeRect fill = RelativeRect.fromLTRB(0.0, 0.0, 0.0, 0.0);

  /// Distance from the left side of the container to the left side of this rectangle.
  ///
  /// May be negative if the left side of the rectangle is outside of the container.
  final double left;

  /// Distance from the top side of the container to the top side of this rectangle.
  ///
  /// May be negative if the top side of the rectangle is outside of the container.
  final double top;

  /// Distance from the right side of the container to the right side of this rectangle.
  ///
  /// May be positive if the right side of the rectangle is outside of the container.
  final double right;

  /// Distance from the bottom side of the container to the bottom side of this rectangle.
  ///
  /// May be positive if the bottom side of the rectangle is outside of the container.
  final double bottom;

  /// Returns whether any of the values are greater than zero.
  ///
  /// This corresponds to one of the sides ([left], [top], [right], or [bottom]) having
  /// some positive inset towards the center.
  bool get hasInsets => left > 0.0 || top > 0.0 || right > 0.0 || bottom > 0.0;

  /// Returns a new rectangle object translated by the given offset.
  RelativeRect shift(Offset offset) {
    return RelativeRect.fromLTRB(
      left + offset.dx,
      top + offset.dy,
      right - offset.dx,
      bottom - offset.dy,
    );
  }

  /// Returns a new rectangle with edges moved outwards by the given delta.
  RelativeRect inflate(double delta) {
    return RelativeRect.fromLTRB(left - delta, top - delta, right - delta, bottom - delta);
  }

  /// Returns a new rectangle with edges moved inwards by the given delta.
  RelativeRect deflate(double delta) {
    return inflate(-delta);
  }

  /// Returns a new rectangle that is the intersection of the given rectangle and this rectangle.
  RelativeRect intersect(RelativeRect other) {
    return RelativeRect.fromLTRB(
      math.max(left, other.left),
      math.max(top, other.top),
      math.max(right, other.right),
      math.max(bottom, other.bottom),
    );
  }

  /// Convert this [RelativeRect] to a [Rect], in the coordinate space of the container.
  ///
  /// See also:
  ///
  ///  * [toSize], which returns the size part of the rect, based on the size of
  ///    the container.
  Rect toRect(Rect container) {
    return Rect.fromLTRB(left, top, container.width - right, container.height - bottom);
  }

  /// Convert this [RelativeRect] to a [Size], assuming a container with the given size.
  ///
  /// See also:
  ///
  ///  * [toRect], which also computes the position relative to the container.
  Size toSize(Size container) {
    return Size(container.width - left - right, container.height - top - bottom);
  }

  /// Linearly interpolate between two RelativeRects.
  ///
  /// If either rect is null, this function interpolates from [RelativeRect.fill].
  ///
  /// {@macro dart.ui.shadow.lerp}
  static RelativeRect? lerp(RelativeRect? a, RelativeRect? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    if (a == null) {
      return RelativeRect.fromLTRB(b!.left * t, b.top * t, b.right * t, b.bottom * t);
    }
    if (b == null) {
      final double k = 1.0 - t;
      return RelativeRect.fromLTRB(b!.left * k, b.top * k, b.right * k, b.bottom * k);
    }
    return RelativeRect.fromLTRB(
      lerpDouble(a.left, b.left, t)!,
      lerpDouble(a.top, b.top, t)!,
      lerpDouble(a.right, b.right, t)!,
      lerpDouble(a.bottom, b.bottom, t)!,
    );
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    return other is RelativeRect &&
        other.left == left &&
        other.top == top &&
        other.right == right &&
        other.bottom == bottom;
  }

  @override
  int get hashCode => Object.hash(left, top, right, bottom);

  @override
  String toString() =>
      'RelativeRect.fromLTRB(${left.toStringAsFixed(1)}, ${top.toStringAsFixed(1)}, ${right.toStringAsFixed(1)}, ${bottom.toStringAsFixed(1)})';
}

/// Parent data for use with [RenderStack].
class StackParentData extends ContainerBoxParentData<RenderBox> {
  /// The distance by which the child's top edge is inset from the top of the stack.
  double? top;

  /// The distance by which the child's right edge is inset from the right of the stack.
  double? right;

  /// The distance by which the child's bottom edge is inset from the bottom of the stack.
  double? bottom;

  /// The distance by which the child's left edge is inset from the left of the stack.
  double? left;

  /// The child's width.
  ///
  /// Ignored if both left and right are non-null.
  double? width;

  /// The child's height.
  ///
  /// Ignored if both top and bottom are non-null.
  double? height;

  /// Get or set the current values in terms of a RelativeRect object.
  RelativeRect get rect => RelativeRect.fromLTRB(left!, top!, right!, bottom!);
  set rect(RelativeRect value) {
    top = value.top;
    right = value.right;
    bottom = value.bottom;
    left = value.left;
  }

  /// Whether this child is considered positioned.
  ///
  /// A child is positioned if any of the top, right, bottom, or left properties
  /// are non-null. Positioned children do not factor into determining the size
  /// of the stack but are instead placed relative to the non-positioned
  /// children in the stack.
  bool get isPositioned =>
      top != null ||
      right != null ||
      bottom != null ||
      left != null ||
      width != null ||
      height != null;

  /// Computes the [BoxConstraints] the stack layout algorithm would give to
  /// this child, given the [Size] of the stack.
  ///
  /// This method should only be called when [isPositioned] is true for the child.
  BoxConstraints positionedChildConstraints(Size stackSize) {
    assert(isPositioned);
    final double? width = switch ((left, right)) {
      (final double left?, final double right?) => stackSize.width - right - left,
      (_, _) => this.width,
    };

    final double? height = switch ((top, bottom)) {
      (final double top?, final double bottom?) => stackSize.height - bottom - top,
      (_, _) => this.height,
    };
    assert(height == null || !height.isNaN);
    assert(width == null || !width.isNaN);
    return BoxConstraints.tightFor(
      width: width == null ? null : math.max(0.0, width),
      height: height == null ? null : math.max(0.0, height),
    );
  }

  @override
  String toString() {
    final List<String> values = <String>[
      if (top != null) 'top=${debugFormatDouble(top)}',
      if (right != null) 'right=${debugFormatDouble(right)}',
      if (bottom != null) 'bottom=${debugFormatDouble(bottom)}',
      if (left != null) 'left=${debugFormatDouble(left)}',
      if (width != null) 'width=${debugFormatDouble(width)}',
      if (height != null) 'height=${debugFormatDouble(height)}',
    ];
    if (values.isEmpty) {
      values.add('not positioned');
    }
    values.add(super.toString());
    return values.join('; ');
  }
}

/// How to size the non-positioned children of a [Stack].
///
/// This enum is used with [Stack.fit] and [RenderStack.fit] to control
/// how the [BoxConstraints] passed from the stack's parent to the stack's child
/// are adjusted.
///
/// See also:
///
///  * [Stack], the widget that uses this.
///  * [RenderStack], the render object that implements the stack algorithm.
enum StackFit {
  /// The constraints passed to the stack from its parent are loosened.
  ///
  /// For example, if the stack has constraints that force it to 350x600, then
  /// this would allow the non-positioned children of the stack to have any
  /// width from zero to 350 and any height from zero to 600.
  ///
  /// See also:
  ///
  ///  * [Center], which loosens the constraints passed to its child and then
  ///    centers the child in itself.
  ///  * [BoxConstraints.loosen], which implements the loosening of box
  ///    constraints.
  loose,

  /// The constraints passed to the stack from its parent are tightened to the
  /// biggest size allowed.
  ///
  /// For example, if the stack has loose constraints with a width in the range
  /// 10 to 100 and a height in the range 0 to 600, then the non-positioned
  /// children of the stack would all be sized as 100 pixels wide and 600 high.
  expand,

  /// The constraints passed to the stack from its parent are passed unmodified
  /// to the non-positioned children.
  ///
  /// For example, if a [Stack] is an [Expanded] child of a [Row], the
  /// horizontal constraints will be tight and the vertical constraints will be
  /// loose.
  passthrough,
}

/// Implements the stack layout algorithm.
///
/// In a stack layout, the children are positioned on top of each other in the
/// order in which they appear in the child list. First, the non-positioned
/// children (those with null values for top, right, bottom, and left) are
/// laid out and initially placed in the upper-left corner of the stack. The
/// stack is then sized to enclose all of the non-positioned children. If there
/// are no non-positioned children, the stack becomes as large as possible.
///
/// The final location of non-positioned children is determined by the alignment
/// parameter. The left of each non-positioned child becomes the
/// difference between the child's width and the stack's width scaled by
/// alignment.x. The top of each non-positioned child is computed
/// similarly and scaled by alignment.y. So if the alignment x and y properties
/// are 0.0 (the default) then the non-positioned children remain in the
/// upper-left corner. If the alignment x and y properties are 0.5 then the
/// non-positioned children are centered within the stack.
///
/// Next, the positioned children are laid out. If a child has top and bottom
/// values that are both non-null, the child is given a fixed height determined
/// by subtracting the sum of the top and bottom values from the height of the stack.
/// Similarly, if the child has right and left values that are both non-null,
/// the child is given a fixed width derived from the stack's width.
/// Otherwise, the child is given unbounded constraints in the non-fixed dimensions.
///
/// Once the child is laid out, the stack positions the child
/// according to the top, right, bottom, and left properties of their
/// [StackParentData]. For example, if the bottom value is 10.0, the
/// bottom edge of the child will be inset 10.0 pixels from the bottom
/// edge of the stack. If the child extends beyond the bounds of the
/// stack, the stack will clip the child's painting to the bounds of
/// the stack.
///
/// See also:
///
///  * [RenderFlow]
class RenderStack extends RenderBox
    with
        ContainerRenderObjectMixin<RenderBox, StackParentData>,
        RenderBoxContainerDefaultsMixin<RenderBox, StackParentData> {
  /// Creates a stack render object.
  ///
  /// By default, the non-positioned children of the stack are aligned by their
  /// top left corners.
  RenderStack({
    List<RenderBox>? children,
    AlignmentGeometry alignment = AlignmentDirectional.topStart,
    TextDirection? textDirection,
    StackFit fit = StackFit.loose,
    Clip clipBehavior = Clip.hardEdge,
  }) : _alignment = alignment,
       _textDirection = textDirection,
       _fit = fit,
       _clipBehavior = clipBehavior {
    addAll(children);
  }

  bool _hasVisualOverflow = false;

  @override
  void setupParentData(RenderBox child) {
    if (child.parentData is! StackParentData) {
      child.parentData = StackParentData();
    }
  }

  Alignment get _resolvedAlignment => _resolvedAlignmentCache ??= alignment.resolve(textDirection);
  Alignment? _resolvedAlignmentCache;

  void _markNeedResolution() {
    _resolvedAlignmentCache = null;
    markNeedsLayout();
  }

  /// How to align the non-positioned or partially-positioned children in the
  /// stack.
  ///
  /// The non-positioned children are placed relative to each other such that
  /// the points determined by [alignment] are co-located. For example, if the
  /// [alignment] is [Alignment.topLeft], then the top left corner of
  /// each non-positioned child will be located at the same global coordinate.
  ///
  /// Partially-positioned children, those that do not specify an alignment in a
  /// particular axis (e.g. that have neither `top` nor `bottom` set), use the
  /// alignment to determine how they should be positioned in that
  /// under-specified axis.
  ///
  /// If this is set to an [AlignmentDirectional] object, then [textDirection]
  /// must not be null.
  AlignmentGeometry get alignment => _alignment;
  AlignmentGeometry _alignment;
  set alignment(AlignmentGeometry value) {
    if (_alignment == value) {
      return;
    }
    _alignment = value;
    _markNeedResolution();
  }

  /// The text direction with which to resolve [alignment].
  ///
  /// This may be changed to null, but only after the [alignment] has been changed
  /// to a value that does not depend on the direction.
  TextDirection? get textDirection => _textDirection;
  TextDirection? _textDirection;
  set textDirection(TextDirection? value) {
    if (_textDirection == value) {
      return;
    }
    _textDirection = value;
    _markNeedResolution();
  }

  /// How to size the non-positioned children in the stack.
  ///
  /// The constraints passed into the [RenderStack] from its parent are either
  /// loosened ([StackFit.loose]) or tightened to their biggest size
  /// ([StackFit.expand]).
  StackFit get fit => _fit;
  StackFit _fit;
  set fit(StackFit value) {
    if (_fit != value) {
      _fit = value;
      markNeedsLayout();
    }
  }

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Stacks only clip children whose geometry overflow the stack. A child that
  /// paints outside its bounds (e.g. a box with a shadow) will not be clipped,
  /// regardless of the value of this property. Similarly, a child that itself
  /// has a descendant that overflows the stack will not be clipped, as only the
  /// geometry of the stack's direct children are considered.
  ///
  /// To clip children whose geometry does not overflow the stack, consider
  /// using a [RenderClipRect] render object.
  ///
  /// Defaults to [Clip.hardEdge].
  Clip get clipBehavior => _clipBehavior;
  Clip _clipBehavior = Clip.hardEdge;
  set clipBehavior(Clip value) {
    if (value != _clipBehavior) {
      _clipBehavior = value;
      markNeedsPaint();
      markNeedsSemanticsUpdate();
    }
  }

  /// Helper function for calculating the intrinsics metrics of a Stack.
  static double getIntrinsicDimension(
    RenderBox? firstChild,
    double Function(RenderBox child) mainChildSizeGetter,
  ) {
    double extent = 0.0;
    RenderBox? child = firstChild;
    while (child != null) {
      final StackParentData childParentData = child.parentData! as StackParentData;
      if (!childParentData.isPositioned) {
        extent = math.max(extent, mainChildSizeGetter(child));
      }
      assert(child.parentData == childParentData);
      child = childParentData.nextSibling;
    }
    return extent;
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    return getIntrinsicDimension(
      firstChild,
      (RenderBox child) => child.getMinIntrinsicWidth(height),
    );
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    return getIntrinsicDimension(
      firstChild,
      (RenderBox child) => child.getMaxIntrinsicWidth(height),
    );
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    return getIntrinsicDimension(
      firstChild,
      (RenderBox child) => child.getMinIntrinsicHeight(width),
    );
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    return getIntrinsicDimension(
      firstChild,
      (RenderBox child) => child.getMaxIntrinsicHeight(width),
    );
  }

  @override
  double? computeDistanceToActualBaseline(TextBaseline baseline) {
    return defaultComputeDistanceToHighestActualBaseline(baseline);
  }

  /// Lays out the positioned `child` according to `alignment` within a Stack of `size`.
  ///
  /// Returns true when the child has visual overflow.
  static bool layoutPositionedChild(
    RenderBox child,
    StackParentData childParentData,
    Size size,
    Alignment alignment,
  ) {
    assert(childParentData.isPositioned);
    assert(child.parentData == childParentData);
    final BoxConstraints childConstraints = childParentData.positionedChildConstraints(size);
    child.layout(childConstraints, parentUsesSize: true);

    final double x = switch (childParentData) {
      StackParentData(:final double left?) => left,
      StackParentData(:final double right?) => size.width - right - child.size.width,
      StackParentData() => alignment.alongOffset(size - child.size as Offset).dx,
    };

    final double y = switch (childParentData) {
      StackParentData(:final double top?) => top,
      StackParentData(:final double bottom?) => size.height - bottom - child.size.height,
      StackParentData() => alignment.alongOffset(size - child.size as Offset).dy,
    };

    childParentData.offset = Offset(x, y);
    return x < 0.0 ||
        x + child.size.width > size.width ||
        y < 0.0 ||
        y + child.size.height > size.height;
  }

  static double? _baselineForChild(
    RenderBox child,
    Size stackSize,
    BoxConstraints nonPositionedChildConstraints,
    Alignment alignment,
    TextBaseline baseline,
  ) {
    final StackParentData childParentData = child.parentData! as StackParentData;
    final BoxConstraints childConstraints =
        childParentData.isPositioned
            ? childParentData.positionedChildConstraints(stackSize)
            : nonPositionedChildConstraints;
    final double? baselineOffset = child.getDryBaseline(childConstraints, baseline);
    if (baselineOffset == null) {
      return null;
    }
    final double y = switch (childParentData) {
      StackParentData(:final double top?) => top,
      StackParentData(:final double bottom?) =>
        stackSize.height - bottom - child.getDryLayout(childConstraints).height,
      StackParentData() =>
        alignment.alongOffset(stackSize - child.getDryLayout(childConstraints) as Offset).dy,
    };
    return baselineOffset + y;
  }

  @override
  double? computeDryBaseline(BoxConstraints constraints, TextBaseline baseline) {
    final BoxConstraints nonPositionedChildConstraints = switch (fit) {
      StackFit.loose => constraints.loosen(),
      StackFit.expand => BoxConstraints.tight(constraints.biggest),
      StackFit.passthrough => constraints,
    };

    final Alignment alignment = _resolvedAlignment;
    final Size size = getDryLayout(constraints);

    BaselineOffset baselineOffset = BaselineOffset.noBaseline;
    for (RenderBox? child = firstChild; child != null; child = childAfter(child)) {
      baselineOffset = baselineOffset.minOf(
        BaselineOffset(
          _baselineForChild(child, size, nonPositionedChildConstraints, alignment, baseline),
        ),
      );
    }
    return baselineOffset.offset;
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    return _computeSize(constraints: constraints, layoutChild: ChildLayoutHelper.dryLayoutChild);
  }

  Size _computeSize({required BoxConstraints constraints, required ChildLayouter layoutChild}) {
    bool hasNonPositionedChildren = false;
    if (childCount == 0) {
      return constraints.biggest.isFinite ? constraints.biggest : constraints.smallest;
    }

    double width = constraints.minWidth;
    double height = constraints.minHeight;

    final BoxConstraints nonPositionedConstraints = switch (fit) {
      StackFit.loose => constraints.loosen(),
      StackFit.expand => BoxConstraints.tight(constraints.biggest),
      StackFit.passthrough => constraints,
    };

    RenderBox? child = firstChild;
    while (child != null) {
      final StackParentData childParentData = child.parentData! as StackParentData;

      if (!childParentData.isPositioned) {
        hasNonPositionedChildren = true;

        final Size childSize = layoutChild(child, nonPositionedConstraints);

        width = math.max(width, childSize.width);
        height = math.max(height, childSize.height);
      }

      child = childParentData.nextSibling;
    }

    final Size size;
    if (hasNonPositionedChildren) {
      size = Size(width, height);
      assert(size.width == constraints.constrainWidth(width));
      assert(size.height == constraints.constrainHeight(height));
    } else {
      size = constraints.biggest;
    }

    assert(size.isFinite);
    return size;
  }

  @override
  void performLayout() {
    final BoxConstraints constraints = this.constraints;
    _hasVisualOverflow = false;

    size = _computeSize(constraints: constraints, layoutChild: ChildLayoutHelper.layoutChild);

    final Alignment resolvedAlignment = _resolvedAlignment;
    RenderBox? child = firstChild;
    while (child != null) {
      final StackParentData childParentData = child.parentData! as StackParentData;

      if (!childParentData.isPositioned) {
        childParentData.offset = resolvedAlignment.alongOffset(size - child.size as Offset);
      } else {
        _hasVisualOverflow =
            layoutPositionedChild(child, childParentData, size, resolvedAlignment) ||
            _hasVisualOverflow;
      }

      assert(child.parentData == childParentData);
      child = childParentData.nextSibling;
    }
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    return defaultHitTestChildren(result, position: position);
  }

  /// Override in subclasses to customize how the stack paints.
  ///
  /// By default, the stack uses [defaultPaint]. This function is called by
  /// [paint] after potentially applying a clip to contain visual overflow.
  @protected
  void paintStack(PaintingContext context, Offset offset) {
    defaultPaint(context, offset);
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (clipBehavior != Clip.none && _hasVisualOverflow) {
      _clipRectLayer.layer = context.pushClipRect(
        needsCompositing,
        offset,
        Offset.zero & size,
        paintStack,
        clipBehavior: clipBehavior,
        oldLayer: _clipRectLayer.layer,
      );
    } else {
      _clipRectLayer.layer = null;
      paintStack(context, offset);
    }
  }

  final LayerHandle<ClipRectLayer> _clipRectLayer = LayerHandle<ClipRectLayer>();

  @override
  void dispose() {
    _clipRectLayer.layer = null;
    super.dispose();
  }

  @override
  Rect? describeApproximatePaintClip(RenderObject child) {
    switch (clipBehavior) {
      case Clip.none:
        return null;
      case Clip.hardEdge:
      case Clip.antiAlias:
      case Clip.antiAliasWithSaveLayer:
        return _hasVisualOverflow ? Offset.zero & size : null;
    }
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<AlignmentGeometry>('alignment', alignment));
    properties.add(EnumProperty<TextDirection>('textDirection', textDirection));
    properties.add(EnumProperty<StackFit>('fit', fit));
    properties.add(EnumProperty<Clip>('clipBehavior', clipBehavior, defaultValue: Clip.hardEdge));
  }
}

/// Implements the same layout algorithm as RenderStack but only paints the child
/// specified by index.
///
/// Although only one child is displayed, the cost of the layout algorithm is
/// still O(N), like an ordinary stack.
class RenderIndexedStack extends RenderStack {
  /// Creates a stack render object that paints a single child.
  ///
  /// If the [index] parameter is null, nothing is displayed.
  RenderIndexedStack({
    super.children,
    super.alignment,
    super.textDirection,
    super.fit,
    super.clipBehavior,
    int? index = 0,
  }) : _index = index;

  @override
  void visitChildrenForSemantics(RenderObjectVisitor visitor) {
    final RenderBox? displayedChild = _childAtIndex();
    if (displayedChild != null) {
      visitor(displayedChild);
    }
  }

  /// The index of the child to show, null if nothing is to be displayed.
  int? get index => _index;
  int? _index;
  set index(int? value) {
    if (_index != value) {
      _index = value;
      markNeedsLayout();
    }
  }

  RenderBox? _childAtIndex() {
    final int? index = this.index;
    if (index == null) {
      return null;
    }
    RenderBox? child = firstChild;
    for (int i = 0; i < index && child != null; i += 1) {
      child = childAfter(child);
    }
    assert(firstChild == null || child != null);
    return child;
  }

  @override
  double? computeDistanceToActualBaseline(TextBaseline baseline) {
    final RenderBox? displayedChild = _childAtIndex();
    if (displayedChild == null) {
      return null;
    }
    final StackParentData childParentData = displayedChild.parentData! as StackParentData;
    final BaselineOffset offset =
        BaselineOffset(displayedChild.getDistanceToActualBaseline(baseline)) +
        childParentData.offset.dy;
    return offset.offset;
  }

  @override
  double? computeDryBaseline(BoxConstraints constraints, TextBaseline baseline) {
    final RenderBox? displayedChild = _childAtIndex();
    if (displayedChild == null) {
      return null;
    }
    final BoxConstraints nonPositionedChildConstraints = switch (fit) {
      StackFit.loose => constraints.loosen(),
      StackFit.expand => BoxConstraints.tight(constraints.biggest),
      StackFit.passthrough => constraints,
    };

    final Alignment alignment = _resolvedAlignment;
    final Size size = getDryLayout(constraints);

    return RenderStack._baselineForChild(
      displayedChild,
      size,
      nonPositionedChildConstraints,
      alignment,
      baseline,
    );
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    final RenderBox? displayedChild = _childAtIndex();
    if (displayedChild == null) {
      return false;
    }
    final StackParentData childParentData = displayedChild.parentData! as StackParentData;
    return result.addWithPaintOffset(
      offset: childParentData.offset,
      position: position,
      hitTest: (BoxHitTestResult result, Offset transformed) {
        assert(transformed == position - childParentData.offset);
        return displayedChild.hitTest(result, position: transformed);
      },
    );
  }

  @override
  void paintStack(PaintingContext context, Offset offset) {
    final RenderBox? displayedChild = _childAtIndex();
    if (displayedChild == null) {
      return;
    }
    final StackParentData childParentData = displayedChild.parentData! as StackParentData;
    context.paintChild(displayedChild, childParentData.offset + offset);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(IntProperty('index', index));
  }

  @override
  List<DiagnosticsNode> debugDescribeChildren() {
    final List<DiagnosticsNode> children = <DiagnosticsNode>[];
    int i = 0;
    RenderObject? child = firstChild;
    while (child != null) {
      children.add(
        child.toDiagnosticsNode(
          name: 'child ${i + 1}',
          style: i != index ? DiagnosticsTreeStyle.offstage : null,
        ),
      );
      child = (child.parentData! as StackParentData).nextSibling;
      i += 1;
    }
    return children;
  }
}
