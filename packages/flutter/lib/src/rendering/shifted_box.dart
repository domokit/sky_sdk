// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'proxy_box.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart';

import 'box.dart';
import 'debug.dart';
import 'debug_overflow_indicator.dart';
import 'layer.dart';
import 'layout_helper.dart';
import 'object.dart';
import 'stack.dart' show RelativeRect;

/// Signature for a function that transforms a [BoxConstraints] to another
/// [BoxConstraints].
///
/// Used by [RenderConstraintsTransformBox] and [ConstraintsTransformBox].
/// Typically the caller requires the returned [BoxConstraints] to be
/// [BoxConstraints.isNormalized].
typedef BoxConstraintsTransform = BoxConstraints Function(BoxConstraints constraints);

/// Abstract class for one-child-layout render boxes that provide control over
/// the child's position.
abstract class RenderShiftedBox extends RenderBox with RenderObjectWithChildMixin<RenderBox> {
  /// Initializes the [child] property for subclasses.
  RenderShiftedBox(RenderBox? child) {
    this.child = child;
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    return child?.getMinIntrinsicWidth(height) ?? 0.0;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    return child?.getMaxIntrinsicWidth(height) ?? 0.0;
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    return child?.getMinIntrinsicHeight(width) ?? 0.0;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    return child?.getMaxIntrinsicHeight(width) ?? 0.0;
  }

  @override
  double? computeDistanceToActualBaseline(TextBaseline baseline) {
    double? result;
    final RenderBox? child = this.child;
    assert(!debugNeedsLayout);
    if (child != null) {
      assert(!child.debugNeedsLayout);
      result = child.getDistanceToActualBaseline(baseline);
      final BoxParentData childParentData = child.parentData! as BoxParentData;
      if (result != null) {
        result += childParentData.offset.dy;
      }
    } else {
      result = super.computeDistanceToActualBaseline(baseline);
    }
    return result;
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    final RenderBox? child = this.child;
    if (child != null) {
      final BoxParentData childParentData = child.parentData! as BoxParentData;
      context.paintChild(child, childParentData.offset + offset);
    }
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    final RenderBox? child = this.child;
    if (child != null) {
      final BoxParentData childParentData = child.parentData! as BoxParentData;
      return result.addWithPaintOffset(
        offset: childParentData.offset,
        position: position,
        hitTest: (BoxHitTestResult result, Offset transformed) {
          assert(transformed == position - childParentData.offset);
          return child.hitTest(result, position: transformed);
        },
      );
    }
    return false;
  }
}

/// Insets its child by the given padding.
///
/// When passing layout constraints to its child, padding shrinks the
/// constraints by the given padding, causing the child to layout at a smaller
/// size. Padding then sizes itself to its child's size, inflated by the
/// padding, effectively creating empty space around the child.
class RenderPadding extends RenderShiftedBox {
  /// Creates a render object that insets its child.
  ///
  /// The [padding] argument must have non-negative insets.
  RenderPadding({
    required EdgeInsetsGeometry padding,
    TextDirection? textDirection,
    RenderBox? child,
  }) : assert(padding.isNonNegative),
       _textDirection = textDirection,
       _padding = padding,
       super(child);

  EdgeInsets? _resolvedPaddingCache;
  EdgeInsets get _resolvedPadding {
    final EdgeInsets returnValue = _resolvedPaddingCache ??= padding.resolve(textDirection);
    assert(returnValue.isNonNegative);
    return returnValue;
  }

  void _markNeedResolution() {
    _resolvedPaddingCache = null;
    markNeedsLayout();
  }

  /// The amount to pad the child in each dimension.
  ///
  /// If this is set to an [EdgeInsetsDirectional] object, then [textDirection]
  /// must not be null.
  EdgeInsetsGeometry get padding => _padding;
  EdgeInsetsGeometry _padding;
  set padding(EdgeInsetsGeometry value) {
    assert(value.isNonNegative);
    if (_padding == value) {
      return;
    }
    _padding = value;
    _markNeedResolution();
  }

  /// The text direction with which to resolve [padding].
  ///
  /// This may be changed to null, but only after the [padding] has been changed
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

  @override
  double computeMinIntrinsicWidth(double height) {
    final EdgeInsets padding = _resolvedPadding;
    if (child != null) {
      // Relies on double.infinity absorption.
      return child!.getMinIntrinsicWidth(math.max(0.0, height - padding.vertical)) +
          padding.horizontal;
    }
    return padding.horizontal;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    final EdgeInsets padding = _resolvedPadding;
    if (child != null) {
      // Relies on double.infinity absorption.
      return child!.getMaxIntrinsicWidth(math.max(0.0, height - padding.vertical)) +
          padding.horizontal;
    }
    return padding.horizontal;
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    final EdgeInsets padding = _resolvedPadding;
    if (child != null) {
      // Relies on double.infinity absorption.
      return child!.getMinIntrinsicHeight(math.max(0.0, width - padding.horizontal)) +
          padding.vertical;
    }
    return padding.vertical;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    final EdgeInsets padding = _resolvedPadding;
    if (child != null) {
      // Relies on double.infinity absorption.
      return child!.getMaxIntrinsicHeight(math.max(0.0, width - padding.horizontal)) +
          padding.vertical;
    }
    return padding.vertical;
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    final EdgeInsets padding = _resolvedPadding;
    if (child == null) {
      return constraints.constrain(Size(padding.horizontal, padding.vertical));
    }
    final BoxConstraints innerConstraints = constraints.deflate(padding);
    final Size childSize = child!.getDryLayout(innerConstraints);
    return constraints.constrain(
      Size(padding.horizontal + childSize.width, padding.vertical + childSize.height),
    );
  }

  @override
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    final RenderBox? child = this.child;
    if (child == null) {
      return null;
    }
    final EdgeInsets padding = _resolvedPadding;
    final BoxConstraints innerConstraints = constraints.deflate(padding);
    final BaselineOffset result =
        BaselineOffset(child.getDryBaseline(innerConstraints, baseline)) + padding.top;
    return result.offset;
  }

  @override
  void performLayout() {
    final BoxConstraints constraints = this.constraints;
    final EdgeInsets padding = _resolvedPadding;
    if (child == null) {
      size = constraints.constrain(Size(padding.horizontal, padding.vertical));
      return;
    }
    final BoxConstraints innerConstraints = constraints.deflate(padding);
    child!.layout(innerConstraints, parentUsesSize: true);
    final BoxParentData childParentData = child!.parentData! as BoxParentData;
    childParentData.offset = Offset(padding.left, padding.top);
    size = constraints.constrain(
      Size(padding.horizontal + child!.size.width, padding.vertical + child!.size.height),
    );
  }

  @override
  void debugPaintSize(PaintingContext context, Offset offset) {
    super.debugPaintSize(context, offset);
    assert(() {
      final Rect outerRect = offset & size;
      debugPaintPadding(
        context.canvas,
        outerRect,
        child != null ? _resolvedPaddingCache!.deflateRect(outerRect) : null,
      );
      return true;
    }());
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<EdgeInsetsGeometry>('padding', padding));
    properties.add(EnumProperty<TextDirection>('textDirection', textDirection, defaultValue: null));
  }
}

/// Abstract class for one-child-layout render boxes that use a
/// [AlignmentGeometry] to align their children.
abstract class RenderAligningShiftedBox extends RenderShiftedBox {
  /// Initializes member variables for subclasses.
  ///
  /// The [textDirection] must be non-null if the [alignment] is
  /// direction-sensitive.
  RenderAligningShiftedBox({
    AlignmentGeometry alignment = Alignment.center,
    required TextDirection? textDirection,
    RenderBox? child,
  }) : _alignment = alignment,
       _textDirection = textDirection,
       super(child);

  /// The [Alignment] to use for aligning the child.
  ///
  /// This is the [alignment] resolved against [textDirection]. Subclasses should
  /// use [resolvedAlignment] instead of [alignment] directly, for computing the
  /// child's offset.
  ///
  /// The [performLayout] method will be called when the value changes.
  @protected
  Alignment get resolvedAlignment => _resolvedAlignment ??= alignment.resolve(textDirection);
  Alignment? _resolvedAlignment;

  void _markNeedResolution() {
    _resolvedAlignment = null;
    markNeedsLayout();
  }

  /// How to align the child.
  ///
  /// The x and y values of the alignment control the horizontal and vertical
  /// alignment, respectively. An x value of -1.0 means that the left edge of
  /// the child is aligned with the left edge of the parent whereas an x value
  /// of 1.0 means that the right edge of the child is aligned with the right
  /// edge of the parent. Other values interpolate (and extrapolate) linearly.
  /// For example, a value of 0.0 means that the center of the child is aligned
  /// with the center of the parent.
  ///
  /// If this is set to an [AlignmentDirectional] object, then
  /// [textDirection] must not be null.
  AlignmentGeometry get alignment => _alignment;
  AlignmentGeometry _alignment;

  /// Sets the alignment to a new value, and triggers a layout update.
  set alignment(AlignmentGeometry value) {
    if (_alignment == value) {
      return;
    }
    _alignment = value;
    _markNeedResolution();
  }

  /// The text direction with which to resolve [alignment].
  ///
  /// This may be changed to null, but only after [alignment] has been changed
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

  /// Apply the current [alignment] to the [child].
  ///
  /// Subclasses should call this method if they have a child, to have
  /// this class perform the actual alignment. If there is no child,
  /// do not call this method.
  ///
  /// This method must be called after the child has been laid out and
  /// this object's own size has been set.
  @protected
  void alignChild() {
    assert(child != null);
    assert(!child!.debugNeedsLayout);
    assert(child!.hasSize);
    assert(hasSize);
    final BoxParentData childParentData = child!.parentData! as BoxParentData;
    childParentData.offset = resolvedAlignment.alongOffset(size - child!.size as Offset);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<AlignmentGeometry>('alignment', alignment));
    properties.add(EnumProperty<TextDirection>('textDirection', textDirection, defaultValue: null));
  }
}

/// Positions its child using an [AlignmentGeometry].
///
/// For example, to align a box at the bottom right, you would pass this box a
/// tight constraint that is bigger than the child's natural size,
/// with an alignment of [Alignment.bottomRight].
///
/// By default, sizes to be as big as possible in both axes. If either axis is
/// unconstrained, then in that direction it will be sized to fit the child's
/// dimensions. Using widthFactor and heightFactor you can force this latter
/// behavior in all cases.
class RenderPositionedBox extends RenderAligningShiftedBox {
  /// Creates a render object that positions its child.
  RenderPositionedBox({
    super.child,
    double? widthFactor,
    double? heightFactor,
    super.alignment,
    super.textDirection,
  }) : assert(widthFactor == null || widthFactor >= 0.0),
       assert(heightFactor == null || heightFactor >= 0.0),
       _widthFactor = widthFactor,
       _heightFactor = heightFactor;

  /// If non-null, sets its width to the child's width multiplied by this factor.
  ///
  /// Can be both greater and less than 1.0 but must be positive.
  double? get widthFactor => _widthFactor;
  double? _widthFactor;
  set widthFactor(double? value) {
    assert(value == null || value >= 0.0);
    if (_widthFactor == value) {
      return;
    }
    _widthFactor = value;
    markNeedsLayout();
  }

  /// If non-null, sets its height to the child's height multiplied by this factor.
  ///
  /// Can be both greater and less than 1.0 but must be positive.
  double? get heightFactor => _heightFactor;
  double? _heightFactor;
  set heightFactor(double? value) {
    assert(value == null || value >= 0.0);
    if (_heightFactor == value) {
      return;
    }
    _heightFactor = value;
    markNeedsLayout();
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    return super.computeMinIntrinsicWidth(height) * (_widthFactor ?? 1);
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    return super.computeMaxIntrinsicWidth(height) * (_widthFactor ?? 1);
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    return super.computeMinIntrinsicHeight(width) * (_heightFactor ?? 1);
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    return super.computeMaxIntrinsicHeight(width) * (_heightFactor ?? 1);
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    final bool shrinkWrapWidth = _widthFactor != null || constraints.maxWidth == double.infinity;
    final bool shrinkWrapHeight = _heightFactor != null || constraints.maxHeight == double.infinity;
    if (child != null) {
      final Size childSize = child!.getDryLayout(constraints.loosen());
      return constraints.constrain(
        Size(
          shrinkWrapWidth ? childSize.width * (_widthFactor ?? 1.0) : double.infinity,
          shrinkWrapHeight ? childSize.height * (_heightFactor ?? 1.0) : double.infinity,
        ),
      );
    }
    return constraints.constrain(
      Size(shrinkWrapWidth ? 0.0 : double.infinity, shrinkWrapHeight ? 0.0 : double.infinity),
    );
  }

  @override
  void performLayout() {
    final BoxConstraints constraints = this.constraints;
    final bool shrinkWrapWidth = _widthFactor != null || constraints.maxWidth == double.infinity;
    final bool shrinkWrapHeight = _heightFactor != null || constraints.maxHeight == double.infinity;

    if (child != null) {
      child!.layout(constraints.loosen(), parentUsesSize: true);
      size = constraints.constrain(
        Size(
          shrinkWrapWidth ? child!.size.width * (_widthFactor ?? 1.0) : double.infinity,
          shrinkWrapHeight ? child!.size.height * (_heightFactor ?? 1.0) : double.infinity,
        ),
      );
      alignChild();
    } else {
      size = constraints.constrain(
        Size(shrinkWrapWidth ? 0.0 : double.infinity, shrinkWrapHeight ? 0.0 : double.infinity),
      );
    }
  }

  @override
  void debugPaintSize(PaintingContext context, Offset offset) {
    super.debugPaintSize(context, offset);
    assert(() {
      final Paint paint;
      if (child != null && !child!.size.isEmpty) {
        final Path path;
        paint =
            Paint()
              ..style = PaintingStyle.stroke
              ..strokeWidth = 1.0
              ..color = const Color(0xFFFFFF00);
        path = Path();
        final BoxParentData childParentData = child!.parentData! as BoxParentData;
        if (childParentData.offset.dy > 0.0) {
          // vertical alignment arrows
          final double headSize = math.min(childParentData.offset.dy * 0.2, 10.0);
          path
            ..moveTo(offset.dx + size.width / 2.0, offset.dy)
            ..relativeLineTo(0.0, childParentData.offset.dy - headSize)
            ..relativeLineTo(headSize, 0.0)
            ..relativeLineTo(-headSize, headSize)
            ..relativeLineTo(-headSize, -headSize)
            ..relativeLineTo(headSize, 0.0)
            ..moveTo(offset.dx + size.width / 2.0, offset.dy + size.height)
            ..relativeLineTo(0.0, -childParentData.offset.dy + headSize)
            ..relativeLineTo(headSize, 0.0)
            ..relativeLineTo(-headSize, -headSize)
            ..relativeLineTo(-headSize, headSize)
            ..relativeLineTo(headSize, 0.0);
          context.canvas.drawPath(path, paint);
        }
        if (childParentData.offset.dx > 0.0) {
          // horizontal alignment arrows
          final double headSize = math.min(childParentData.offset.dx * 0.2, 10.0);
          path
            ..moveTo(offset.dx, offset.dy + size.height / 2.0)
            ..relativeLineTo(childParentData.offset.dx - headSize, 0.0)
            ..relativeLineTo(0.0, headSize)
            ..relativeLineTo(headSize, -headSize)
            ..relativeLineTo(-headSize, -headSize)
            ..relativeLineTo(0.0, headSize)
            ..moveTo(offset.dx + size.width, offset.dy + size.height / 2.0)
            ..relativeLineTo(-childParentData.offset.dx + headSize, 0.0)
            ..relativeLineTo(0.0, headSize)
            ..relativeLineTo(-headSize, -headSize)
            ..relativeLineTo(headSize, -headSize)
            ..relativeLineTo(0.0, headSize);
          context.canvas.drawPath(path, paint);
        }
      } else {
        paint = Paint()..color = const Color(0x90909090);
        context.canvas.drawRect(offset & size, paint);
      }
      return true;
    }());
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('widthFactor', _widthFactor, ifNull: 'expand'));
    properties.add(DoubleProperty('heightFactor', _heightFactor, ifNull: 'expand'));
  }
}

/// How much space should be occupied by the [OverflowBox] if there is no
/// overflow.
enum OverflowBoxFit {
  /// The widget will size itself to be as large as the parent allows.
  max,

  /// The widget will follow the child's size.
  ///
  /// More specifically, the render object will size itself to match the size of
  /// its child within the constraints of its parent, or as small as the
  /// parent allows if no child is set.
  deferToChild,
}

/// A render object that imposes different constraints on its child than it gets
/// from its parent, possibly allowing the child to overflow the parent.
///
/// A render overflow box proxies most functions in the render box protocol to
/// its child, except that when laying out its child, it passes constraints
/// based on the minWidth, maxWidth, minHeight, and maxHeight fields instead of
/// just passing the parent's constraints in. Specifically, it overrides any of
/// the equivalent fields on the constraints given by the parent with the
/// constraints given by these fields for each such field that is not null. It
/// then sizes itself based on the parent's constraints' maxWidth and maxHeight,
/// ignoring the child's dimensions.
///
/// For example, if you wanted a box to always render 50 pixels high, regardless
/// of where it was rendered, you would wrap it in a
/// RenderConstrainedOverflowBox with minHeight and maxHeight set to 50.0.
/// Generally speaking, to avoid confusing behavior around hit testing, a
/// RenderConstrainedOverflowBox should usually be wrapped in a RenderClipRect.
///
/// The child is positioned according to [alignment]. To position a smaller
/// child inside a larger parent, use [RenderPositionedBox] and
/// [RenderConstrainedBox] rather than RenderConstrainedOverflowBox.
///
/// See also:
///
///  * [RenderConstraintsTransformBox] for a render object that applies an
///    arbitrary transform to its constraints before sizing its child using
///    the new constraints, treating any overflow as error.
///  * [RenderSizedOverflowBox], a render object that is a specific size but
///    passes its original constraints through to its child, which it allows to
///    overflow.
class RenderConstrainedOverflowBox extends RenderAligningShiftedBox {
  /// Creates a render object that lets its child overflow itself.
  RenderConstrainedOverflowBox({
    super.child,
    double? minWidth,
    double? maxWidth,
    double? minHeight,
    double? maxHeight,
    OverflowBoxFit fit = OverflowBoxFit.max,
    super.alignment,
    super.textDirection,
  }) : _minWidth = minWidth,
       _maxWidth = maxWidth,
       _minHeight = minHeight,
       _maxHeight = maxHeight,
       _fit = fit;

  /// The minimum width constraint to give the child. Set this to null (the
  /// default) to use the constraint from the parent instead.
  double? get minWidth => _minWidth;
  double? _minWidth;
  set minWidth(double? value) {
    if (_minWidth == value) {
      return;
    }
    _minWidth = value;
    markNeedsLayout();
  }

  /// The maximum width constraint to give the child. Set this to null (the
  /// default) to use the constraint from the parent instead.
  double? get maxWidth => _maxWidth;
  double? _maxWidth;
  set maxWidth(double? value) {
    if (_maxWidth == value) {
      return;
    }
    _maxWidth = value;
    markNeedsLayout();
  }

  /// The minimum height constraint to give the child. Set this to null (the
  /// default) to use the constraint from the parent instead.
  double? get minHeight => _minHeight;
  double? _minHeight;
  set minHeight(double? value) {
    if (_minHeight == value) {
      return;
    }
    _minHeight = value;
    markNeedsLayout();
  }

  /// The maximum height constraint to give the child. Set this to null (the
  /// default) to use the constraint from the parent instead.
  double? get maxHeight => _maxHeight;
  double? _maxHeight;
  set maxHeight(double? value) {
    if (_maxHeight == value) {
      return;
    }
    _maxHeight = value;
    markNeedsLayout();
  }

  /// The way to size the render object.
  ///
  /// This only affects scenario when the child does not indeed overflow.
  /// If set to [OverflowBoxFit.deferToChild], the render object will size
  /// itself to match the size of its child within the constraints of its
  /// parent, or as small as the parent allows if no child is set.
  /// If set to [OverflowBoxFit.max] (the default), the
  /// render object will size itself to be as large as the parent allows.
  OverflowBoxFit get fit => _fit;
  OverflowBoxFit _fit;
  set fit(OverflowBoxFit value) {
    if (_fit == value) {
      return;
    }
    _fit = value;
    markNeedsLayoutForSizedByParentChange();
  }

  BoxConstraints _getInnerConstraints(BoxConstraints constraints) {
    return BoxConstraints(
      minWidth: _minWidth ?? constraints.minWidth,
      maxWidth: _maxWidth ?? constraints.maxWidth,
      minHeight: _minHeight ?? constraints.minHeight,
      maxHeight: _maxHeight ?? constraints.maxHeight,
    );
  }

  @override
  bool get sizedByParent => switch (fit) {
    OverflowBoxFit.max => true,
    // If deferToChild, the size will be as small as its child when non-overflowing,
    // thus it cannot be sizedByParent.
    OverflowBoxFit.deferToChild => false,
  };

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    return switch (fit) {
      OverflowBoxFit.max => constraints.biggest,
      OverflowBoxFit.deferToChild => child?.getDryLayout(constraints) ?? constraints.smallest,
    };
  }

  @override
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    final RenderBox? child = this.child;
    if (child == null) {
      return null;
    }
    final BoxConstraints childConstraints = _getInnerConstraints(constraints);
    final double? result = child.getDryBaseline(childConstraints, baseline);
    if (result == null) {
      return null;
    }
    final Size childSize = child.getDryLayout(childConstraints);
    final Size size = getDryLayout(constraints);
    return result + resolvedAlignment.alongOffset(size - childSize as Offset).dy;
  }

  @override
  void performLayout() {
    if (child != null) {
      child!.layout(_getInnerConstraints(constraints), parentUsesSize: true);
      switch (fit) {
        case OverflowBoxFit.max:
          assert(sizedByParent);
        case OverflowBoxFit.deferToChild:
          size = constraints.constrain(child!.size);
      }
      alignChild();
    } else {
      switch (fit) {
        case OverflowBoxFit.max:
          assert(sizedByParent);
        case OverflowBoxFit.deferToChild:
          size = constraints.smallest;
      }
    }
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('minWidth', minWidth, ifNull: 'use parent minWidth constraint'));
    properties.add(DoubleProperty('maxWidth', maxWidth, ifNull: 'use parent maxWidth constraint'));
    properties.add(
      DoubleProperty('minHeight', minHeight, ifNull: 'use parent minHeight constraint'),
    );
    properties.add(
      DoubleProperty('maxHeight', maxHeight, ifNull: 'use parent maxHeight constraint'),
    );
    properties.add(EnumProperty<OverflowBoxFit>('fit', fit));
  }
}

/// A [RenderBox] that applies an arbitrary transform to its constraints,
/// and sizes its child using the resulting [BoxConstraints], optionally
/// clipping, or treating the overflow as an error.
///
/// This [RenderBox] sizes its child using a [BoxConstraints] created by
/// applying [constraintsTransform] to this [RenderBox]'s own [constraints].
/// This box will then attempt to adopt the same size, within the limits of its
/// own constraints. If it ends up with a different size, it will align the
/// child based on [alignment]. If the box cannot expand enough to accommodate
/// the entire child, the child will be clipped if [clipBehavior] is not
/// [Clip.none].
///
/// In debug mode, if [clipBehavior] is [Clip.none] and the child overflows the
/// container, a warning will be printed on the console, and black and yellow
/// striped areas will appear where the overflow occurs.
///
/// When [child] is null, this [RenderBox] takes the smallest possible size and
/// never overflows.
///
/// This [RenderBox] can be used to ensure some of [child]'s natural dimensions
/// are honored, and get an early warning during development otherwise. For
/// instance, if [child] requires a minimum height to fully display its content,
/// [constraintsTransform] can be set to a function that removes the `maxHeight`
/// constraint from the incoming [BoxConstraints], so that if the parent
/// [RenderObject] fails to provide enough vertical space, a warning will be
/// displayed in debug mode, while still allowing [child] to grow vertically.
///
/// See also:
///
///  * [ConstraintsTransformBox], the widget that makes use of this
///    [RenderObject] and exposes the same functionality.
///  * [RenderConstrainedBox], which renders a box which imposes constraints
///    on its child.
///  * [RenderConstrainedOverflowBox], which renders a box that imposes different
///    constraints on its child than it gets from its parent, possibly allowing
///    the child to overflow the parent.
///  * [RenderConstraintsTransformBox] for a render object that applies an
///    arbitrary transform to its constraints before sizing its child using
///    the new constraints, treating any overflow as error.
class RenderConstraintsTransformBox extends RenderAligningShiftedBox
    with DebugOverflowIndicatorMixin {
  /// Creates a [RenderBox] that sizes itself to the child and modifies the
  /// [constraints] before passing it down to that child.
  RenderConstraintsTransformBox({
    required super.alignment,
    required super.textDirection,
    required BoxConstraintsTransform constraintsTransform,
    super.child,
    Clip clipBehavior = Clip.none,
  }) : _constraintsTransform = constraintsTransform,
       _clipBehavior = clipBehavior;

  /// {@macro flutter.widgets.constraintsTransform}
  BoxConstraintsTransform get constraintsTransform => _constraintsTransform;
  BoxConstraintsTransform _constraintsTransform;
  set constraintsTransform(BoxConstraintsTransform value) {
    if (_constraintsTransform == value) {
      return;
    }
    _constraintsTransform = value;
    // The RenderObject only needs layout if the new transform maps the current
    // `constraints` to a different value, or the render object has never been
    // laid out before.
    final bool needsLayout = _childConstraints == null || _childConstraints != value(constraints);
    if (needsLayout) {
      markNeedsLayout();
    }
  }

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// {@macro flutter.widgets.ConstraintsTransformBox.clipBehavior}
  ///
  /// Defaults to [Clip.none].
  Clip get clipBehavior => _clipBehavior;
  Clip _clipBehavior;
  set clipBehavior(Clip value) {
    if (value != _clipBehavior) {
      _clipBehavior = value;
      markNeedsPaint();
      markNeedsSemanticsUpdate();
    }
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    return super.computeMinIntrinsicHeight(
      constraintsTransform(BoxConstraints(maxWidth: width)).maxWidth,
    );
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    return super.computeMaxIntrinsicHeight(
      constraintsTransform(BoxConstraints(maxWidth: width)).maxWidth,
    );
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    return super.computeMinIntrinsicWidth(
      constraintsTransform(BoxConstraints(maxHeight: height)).maxHeight,
    );
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    return super.computeMaxIntrinsicWidth(
      constraintsTransform(BoxConstraints(maxHeight: height)).maxHeight,
    );
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    final Size? childSize = child?.getDryLayout(constraintsTransform(constraints));
    return childSize == null ? constraints.smallest : constraints.constrain(childSize);
  }

  @override
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    final RenderBox? child = this.child;
    if (child == null) {
      return null;
    }
    final BoxConstraints childConstraints = constraintsTransform(constraints);
    final double? result = child.getDryBaseline(childConstraints, baseline);
    if (result == null) {
      return null;
    }
    final Size childSize = child.getDryLayout(childConstraints);
    final Size size = constraints.constrain(childSize);
    return result + resolvedAlignment.alongOffset(size - childSize as Offset).dy;
  }

  Rect _overflowContainerRect = Rect.zero;
  Rect _overflowChildRect = Rect.zero;
  bool _isOverflowing = false;

  BoxConstraints? _childConstraints;

  @override
  void performLayout() {
    final BoxConstraints constraints = this.constraints;
    final RenderBox? child = this.child;
    if (child != null) {
      final BoxConstraints childConstraints = constraintsTransform(constraints);
      assert(childConstraints.isNormalized, '$childConstraints is not normalized');
      _childConstraints = childConstraints;
      child.layout(childConstraints, parentUsesSize: true);
      size = constraints.constrain(child.size);
      alignChild();
      final BoxParentData childParentData = child.parentData! as BoxParentData;
      _overflowContainerRect = Offset.zero & size;
      _overflowChildRect = childParentData.offset & child.size;
    } else {
      size = constraints.smallest;
      _overflowContainerRect = Rect.zero;
      _overflowChildRect = Rect.zero;
    }
    _isOverflowing = RelativeRect.fromRect(_overflowContainerRect, _overflowChildRect).hasInsets;
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (child == null) {
      return;
    }

    if (!_isOverflowing) {
      super.paint(context, offset);
      return;
    }

    // We have overflow and the clipBehavior isn't none. Clip it.
    _clipRectLayer.layer = context.pushClipRect(
      needsCompositing,
      offset,
      Offset.zero & size,
      super.paint,
      clipBehavior: clipBehavior,
      oldLayer: _clipRectLayer.layer,
    );

    // Display the overflow indicator if clipBehavior is Clip.none.
    assert(() {
      if (size.isEmpty) {
        return true;
      }
      switch (clipBehavior) {
        case Clip.none:
          paintOverflowIndicator(context, offset, _overflowContainerRect, _overflowChildRect);
        case Clip.hardEdge:
        case Clip.antiAlias:
        case Clip.antiAliasWithSaveLayer:
          break;
      }
      return true;
    }());
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
        return _isOverflowing ? Offset.zero & size : null;
    }
  }

  @override
  String toStringShort() {
    String header = super.toStringShort();
    if (!kReleaseMode) {
      if (_isOverflowing) {
        header += ' OVERFLOWING';
      }
    }
    return header;
  }
}

/// A render object that is a specific size but passes its original constraints
/// through to its child, which it allows to overflow.
///
/// If the child's resulting size differs from this render object's size, then
/// the child is aligned according to the [alignment] property.
///
/// See also:
///
///  * [RenderConstraintsTransformBox] for a render object that applies an
///    arbitrary transform to its constraints before sizing its child using
///    the new constraints, treating any overflow as error.
///  * [RenderConstrainedOverflowBox] for a render object that imposes
///    different constraints on its child than it gets from its parent,
///    possibly allowing the child to overflow the parent.
class RenderSizedOverflowBox extends RenderAligningShiftedBox {
  /// Creates a render box of a given size that lets its child overflow.
  ///
  /// The [textDirection] argument must not be null if the [alignment] is
  /// direction-sensitive.
  RenderSizedOverflowBox({
    super.child,
    required Size requestedSize,
    super.alignment,
    super.textDirection,
  }) : _requestedSize = requestedSize;

  /// The size this render box should attempt to be.
  Size get requestedSize => _requestedSize;
  Size _requestedSize;
  set requestedSize(Size value) {
    if (_requestedSize == value) {
      return;
    }
    _requestedSize = value;
    markNeedsLayout();
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    return _requestedSize.width;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    return _requestedSize.width;
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    return _requestedSize.height;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    return _requestedSize.height;
  }

  @override
  double? computeDistanceToActualBaseline(TextBaseline baseline) {
    return child?.getDistanceToActualBaseline(baseline) ??
        super.computeDistanceToActualBaseline(baseline);
  }

  @override
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    final RenderBox? child = this.child;
    if (child == null) {
      return null;
    }
    final double? result = child.getDryBaseline(constraints, baseline);
    if (result == null) {
      return null;
    }
    final Size childSize = child.getDryLayout(constraints);
    final Size size = getDryLayout(constraints);
    return result + resolvedAlignment.alongOffset(size - childSize as Offset).dy;
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    return constraints.constrain(_requestedSize);
  }

  @override
  void performLayout() {
    size = constraints.constrain(_requestedSize);
    if (child != null) {
      child!.layout(constraints, parentUsesSize: true);
      alignChild();
    }
  }
}

/// Sizes its child to a fraction of the total available space.
///
/// For both its width and height, this render object imposes a tight
/// constraint on its child that is a multiple (typically less than 1.0) of the
/// maximum constraint it received from its parent on that axis. If the factor
/// for a given axis is null, then the constraints from the parent are just
/// passed through instead.
///
/// It then tries to size itself to the size of its child. Where this is not
/// possible (e.g. if the constraints from the parent are themselves tight), the
/// child is aligned according to [alignment].
class RenderFractionallySizedOverflowBox extends RenderAligningShiftedBox {
  /// Creates a render box that sizes its child to a fraction of the total available space.
  ///
  /// If non-null, the [widthFactor] and [heightFactor] arguments must be
  /// non-negative.
  ///
  /// The [textDirection] must be non-null if the [alignment] is
  /// direction-sensitive.
  RenderFractionallySizedOverflowBox({
    super.child,
    double? widthFactor,
    double? heightFactor,
    super.alignment,
    super.textDirection,
  }) : _widthFactor = widthFactor,
       _heightFactor = heightFactor {
    assert(_widthFactor == null || _widthFactor! >= 0.0);
    assert(_heightFactor == null || _heightFactor! >= 0.0);
  }

  /// If non-null, the factor of the incoming width to use.
  ///
  /// If non-null, the child is given a tight width constraint that is the max
  /// incoming width constraint multiplied by this factor. If null, the child is
  /// given the incoming width constraints.
  double? get widthFactor => _widthFactor;
  double? _widthFactor;
  set widthFactor(double? value) {
    assert(value == null || value >= 0.0);
    if (_widthFactor == value) {
      return;
    }
    _widthFactor = value;
    markNeedsLayout();
  }

  /// If non-null, the factor of the incoming height to use.
  ///
  /// If non-null, the child is given a tight height constraint that is the max
  /// incoming width constraint multiplied by this factor. If null, the child is
  /// given the incoming width constraints.
  double? get heightFactor => _heightFactor;
  double? _heightFactor;
  set heightFactor(double? value) {
    assert(value == null || value >= 0.0);
    if (_heightFactor == value) {
      return;
    }
    _heightFactor = value;
    markNeedsLayout();
  }

  BoxConstraints _getInnerConstraints(BoxConstraints constraints) {
    double minWidth = constraints.minWidth;
    double maxWidth = constraints.maxWidth;
    if (_widthFactor != null) {
      final double width = maxWidth * _widthFactor!;
      minWidth = width;
      maxWidth = width;
    }
    double minHeight = constraints.minHeight;
    double maxHeight = constraints.maxHeight;
    if (_heightFactor != null) {
      final double height = maxHeight * _heightFactor!;
      minHeight = height;
      maxHeight = height;
    }
    return BoxConstraints(
      minWidth: minWidth,
      maxWidth: maxWidth,
      minHeight: minHeight,
      maxHeight: maxHeight,
    );
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    final double result;
    if (child == null) {
      result = super.computeMinIntrinsicWidth(height);
    } else {
      // the following line relies on double.infinity absorption
      result = child!.getMinIntrinsicWidth(height * (_heightFactor ?? 1.0));
    }
    assert(result.isFinite);
    return result / (_widthFactor ?? 1.0);
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    final double result;
    if (child == null) {
      result = super.computeMaxIntrinsicWidth(height);
    } else {
      // the following line relies on double.infinity absorption
      result = child!.getMaxIntrinsicWidth(height * (_heightFactor ?? 1.0));
    }
    assert(result.isFinite);
    return result / (_widthFactor ?? 1.0);
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    final double result;
    if (child == null) {
      result = super.computeMinIntrinsicHeight(width);
    } else {
      // the following line relies on double.infinity absorption
      result = child!.getMinIntrinsicHeight(width * (_widthFactor ?? 1.0));
    }
    assert(result.isFinite);
    return result / (_heightFactor ?? 1.0);
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    final double result;
    if (child == null) {
      result = super.computeMaxIntrinsicHeight(width);
    } else {
      // the following line relies on double.infinity absorption
      result = child!.getMaxIntrinsicHeight(width * (_widthFactor ?? 1.0));
    }
    assert(result.isFinite);
    return result / (_heightFactor ?? 1.0);
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    if (child != null) {
      final Size childSize = child!.getDryLayout(_getInnerConstraints(constraints));
      return constraints.constrain(childSize);
    }
    return constraints.constrain(_getInnerConstraints(constraints).constrain(Size.zero));
  }

  @override
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    final RenderBox? child = this.child;
    if (child == null) {
      return null;
    }
    final BoxConstraints childConstraints = _getInnerConstraints(constraints);
    final double? result = child.getDryBaseline(childConstraints, baseline);
    if (result == null) {
      return null;
    }
    final Size childSize = child.getDryLayout(childConstraints);
    final Size size = getDryLayout(constraints);
    return result + resolvedAlignment.alongOffset(size - childSize as Offset).dy;
  }

  @override
  void performLayout() {
    if (child != null) {
      child!.layout(_getInnerConstraints(constraints), parentUsesSize: true);
      size = constraints.constrain(child!.size);
      alignChild();
    } else {
      size = constraints.constrain(_getInnerConstraints(constraints).constrain(Size.zero));
    }
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('widthFactor', _widthFactor, ifNull: 'pass-through'));
    properties.add(DoubleProperty('heightFactor', _heightFactor, ifNull: 'pass-through'));
  }
}

/// A delegate for computing the layout of a render object with a single child.
///
/// Used by [CustomSingleChildLayout] (in the widgets library) and
/// [RenderCustomSingleChildLayoutBox] (in the rendering library).
///
/// When asked to layout, [CustomSingleChildLayout] first calls [getSize] with
/// its incoming constraints to determine its size. It then calls
/// [getConstraintsForChild] to determine the constraints to apply to the child.
/// After the child completes its layout, [RenderCustomSingleChildLayoutBox]
/// calls [getPositionForChild] to determine the child's position.
///
/// The [shouldRelayout] method is called when a new instance of the class
/// is provided, to check if the new instance actually represents different
/// information.
///
/// The most efficient way to trigger a relayout is to supply a `relayout`
/// argument to the constructor of the [SingleChildLayoutDelegate]. The custom
/// layout will listen to this value and relayout whenever the Listenable
/// notifies its listeners, such as when an [Animation] ticks. This allows
/// the custom layout to avoid the build phase of the pipeline.
///
/// See also:
///
///  * [CustomSingleChildLayout], the widget that uses this delegate.
///  * [RenderCustomSingleChildLayoutBox], render object that uses this
///    delegate.
abstract class SingleChildLayoutDelegate {
  /// Creates a layout delegate.
  ///
  /// The layout will update whenever [relayout] notifies its listeners.
  const SingleChildLayoutDelegate({Listenable? relayout}) : _relayout = relayout;

  final Listenable? _relayout;

  /// The size of this object given the incoming constraints.
  ///
  /// Defaults to the biggest size that satisfies the given constraints.
  Size getSize(BoxConstraints constraints) => constraints.biggest;

  /// The constraints for the child given the incoming constraints.
  ///
  /// During layout, the child is given the layout constraints returned by this
  /// function. The child is required to pick a size for itself that satisfies
  /// these constraints.
  ///
  /// Defaults to the given constraints.
  BoxConstraints getConstraintsForChild(BoxConstraints constraints) => constraints;

  /// The position where the child should be placed.
  ///
  /// The `size` argument is the size of the parent, which might be different
  /// from the value returned by [getSize] if that size doesn't satisfy the
  /// constraints passed to [getSize]. The `childSize` argument is the size of
  /// the child, which will satisfy the constraints returned by
  /// [getConstraintsForChild].
  ///
  /// Defaults to positioning the child in the upper left corner of the parent.
  Offset getPositionForChild(Size size, Size childSize) => Offset.zero;

  /// Called whenever a new instance of the custom layout delegate class is
  /// provided to the [RenderCustomSingleChildLayoutBox] object, or any time
  /// that a new [CustomSingleChildLayout] object is created with a new instance
  /// of the custom layout delegate class (which amounts to the same thing,
  /// because the latter is implemented in terms of the former).
  ///
  /// If the new instance represents different information than the old
  /// instance, then the method should return true, otherwise it should return
  /// false.
  ///
  /// If the method returns false, then the [getSize],
  /// [getConstraintsForChild], and [getPositionForChild] calls might be
  /// optimized away.
  ///
  /// It's possible that the layout methods will get called even if
  /// [shouldRelayout] returns false (e.g. if an ancestor changed its layout).
  /// It's also possible that the layout method will get called
  /// without [shouldRelayout] being called at all (e.g. if the parent changes
  /// size).
  bool shouldRelayout(covariant SingleChildLayoutDelegate oldDelegate);
}

/// Defers the layout of its single child to a delegate.
///
/// The delegate can determine the layout constraints for the child and can
/// decide where to position the child. The delegate can also determine the size
/// of the parent, but the size of the parent cannot depend on the size of the
/// child.
class RenderCustomSingleChildLayoutBox extends RenderShiftedBox {
  /// Creates a render box that defers its layout to a delegate.
  ///
  /// The [delegate] argument must not be null.
  RenderCustomSingleChildLayoutBox({RenderBox? child, required SingleChildLayoutDelegate delegate})
    : _delegate = delegate,
      super(child);

  /// A delegate that controls this object's layout.
  SingleChildLayoutDelegate get delegate => _delegate;
  SingleChildLayoutDelegate _delegate;
  set delegate(SingleChildLayoutDelegate newDelegate) {
    if (_delegate == newDelegate) {
      return;
    }
    final SingleChildLayoutDelegate oldDelegate = _delegate;
    if (newDelegate.runtimeType != oldDelegate.runtimeType ||
        newDelegate.shouldRelayout(oldDelegate)) {
      markNeedsLayout();
    }
    _delegate = newDelegate;
    if (attached) {
      oldDelegate._relayout?.removeListener(markNeedsLayout);
      newDelegate._relayout?.addListener(markNeedsLayout);
    }
  }

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    _delegate._relayout?.addListener(markNeedsLayout);
  }

  @override
  void detach() {
    _delegate._relayout?.removeListener(markNeedsLayout);
    super.detach();
  }

  Size _getSize(BoxConstraints constraints) {
    return constraints.constrain(_delegate.getSize(constraints));
  }

  // TODO(ianh): It's a bit dubious to be using the getSize function from the delegate to
  // figure out the intrinsic dimensions. We really should either not support intrinsics,
  // or we should expose intrinsic delegate callbacks and throw if they're not implemented.

  @override
  double computeMinIntrinsicWidth(double height) {
    final double width = _getSize(BoxConstraints.tightForFinite(height: height)).width;
    if (width.isFinite) {
      return width;
    }
    return 0.0;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    final double width = _getSize(BoxConstraints.tightForFinite(height: height)).width;
    if (width.isFinite) {
      return width;
    }
    return 0.0;
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    final double height = _getSize(BoxConstraints.tightForFinite(width: width)).height;
    if (height.isFinite) {
      return height;
    }
    return 0.0;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    final double height = _getSize(BoxConstraints.tightForFinite(width: width)).height;
    if (height.isFinite) {
      return height;
    }
    return 0.0;
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    return _getSize(constraints);
  }

  @override
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    final RenderBox? child = this.child;
    if (child == null) {
      return null;
    }
    final BoxConstraints childConstraints = delegate.getConstraintsForChild(constraints);
    final double? result = child.getDryBaseline(childConstraints, baseline);
    if (result == null) {
      return null;
    }
    return result +
        delegate
            .getPositionForChild(
              _getSize(constraints),
              childConstraints.isTight
                  ? childConstraints.smallest
                  : child.getDryLayout(childConstraints),
            )
            .dy;
  }

  @override
  void performLayout() {
    size = _getSize(constraints);
    if (child != null) {
      final BoxConstraints childConstraints = delegate.getConstraintsForChild(constraints);
      assert(childConstraints.debugAssertIsValid(isAppliedConstraint: true));
      child!.layout(childConstraints, parentUsesSize: !childConstraints.isTight);
      final BoxParentData childParentData = child!.parentData! as BoxParentData;
      childParentData.offset = delegate.getPositionForChild(
        size,
        childConstraints.isTight ? childConstraints.smallest : child!.size,
      );
    }
  }
}

/// Shifts the child down such that the child's baseline (or the
/// bottom of the child, if the child has no baseline) is [baseline]
/// logical pixels below the top of this box, then sizes this box to
/// contain the child.
///
/// If [baseline] is less than the distance from the top of the child
/// to the baseline of the child, then the child will overflow the top
/// of the box. This is typically not desirable, in particular, that
/// part of the child will not be found when doing hit tests, so the
/// user cannot interact with that part of the child.
///
/// This box will be sized so that its bottom is coincident with the
/// bottom of the child. This means if this box shifts the child down,
/// there will be space between the top of this box and the top of the
/// child, but there is never space between the bottom of the child
/// and the bottom of the box.
class RenderBaseline extends RenderShiftedBox {
  /// Creates a [RenderBaseline] object.
  RenderBaseline({RenderBox? child, required double baseline, required TextBaseline baselineType})
    : _baseline = baseline,
      _baselineType = baselineType,
      super(child);

  /// The number of logical pixels from the top of this box at which to position
  /// the child's baseline.
  double get baseline => _baseline;
  double _baseline;
  set baseline(double value) {
    if (_baseline == value) {
      return;
    }
    _baseline = value;
    markNeedsLayout();
  }

  /// The type of baseline to use for positioning the child.
  TextBaseline get baselineType => _baselineType;
  TextBaseline _baselineType;
  set baselineType(TextBaseline value) {
    if (_baselineType == value) {
      return;
    }
    _baselineType = value;
    markNeedsLayout();
  }

  ({Size size, double top}) _computeSizes(
    covariant BoxConstraints constraints,
    ChildLayouter layoutChild,
    ChildBaselineGetter getBaseline,
  ) {
    final RenderBox? child = this.child;
    if (child == null) {
      return (size: constraints.smallest, top: 0);
    }
    final BoxConstraints childConstraints = constraints.loosen();
    final Size childSize = layoutChild(child, childConstraints);
    final double childBaseline =
        getBaseline(child, childConstraints, baselineType) ?? childSize.height;
    final double top = baseline - childBaseline;
    return (size: constraints.constrain(Size(childSize.width, top + childSize.height)), top: top);
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    return _computeSizes(
      constraints,
      ChildLayoutHelper.dryLayoutChild,
      ChildLayoutHelper.getDryBaseline,
    ).size;
  }

  @override
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    final RenderBox? child = this.child;
    final double? result1 = child?.getDryBaseline(constraints.loosen(), baseline);
    final double? result2 = child?.getDryBaseline(constraints.loosen(), baselineType);
    if (result1 == null || result2 == null) {
      return null;
    }
    return this.baseline + result1 - result2;
  }

  @override
  void performLayout() {
    final (:Size size, :double top) = _computeSizes(
      constraints,
      ChildLayoutHelper.layoutChild,
      ChildLayoutHelper.getBaseline,
    );
    this.size = size;
    (child?.parentData as BoxParentData?)?.offset = Offset(0.0, top);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('baseline', baseline));
    properties.add(EnumProperty<TextBaseline>('baselineType', baselineType));
  }
}
