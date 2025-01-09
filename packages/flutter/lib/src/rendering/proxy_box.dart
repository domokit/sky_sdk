// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'proxy_sliver.dart';
/// @docImport 'sliver.dart';
library;

import 'dart:ui' as ui show Color, Gradient, Image, ImageFilter;

import 'package:flutter/animation.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/semantics.dart';
import 'package:flutter/services.dart';

import 'box.dart';
import 'layer.dart';
import 'layout_helper.dart';
import 'object.dart';

export 'package:flutter/gestures.dart'
    show PointerCancelEvent, PointerDownEvent, PointerEvent, PointerMoveEvent, PointerUpEvent;

/// A base class for render boxes that resemble their children.
///
/// A proxy box has a single child and mimics all the properties of that
/// child by calling through to the child for each function in the render box
/// protocol. For example, a proxy box determines its size by asking its child
/// to layout with the same constraints and then matching the size.
///
/// A proxy box isn't useful on its own because you might as well just replace
/// the proxy box with its child. However, RenderProxyBox is a useful base class
/// for render objects that wish to mimic most, but not all, of the properties
/// of their child.
///
/// See also:
///
///  * [RenderProxySliver], a base class for render slivers that resemble their
///    children.
class RenderProxyBox extends RenderBox
    with RenderObjectWithChildMixin<RenderBox>, RenderProxyBoxMixin<RenderBox> {
  /// Creates a proxy render box.
  ///
  /// Proxy render boxes are rarely created directly because they proxy
  /// the render box protocol to [child]. Instead, consider using one of the
  /// subclasses.
  RenderProxyBox([RenderBox? child]) {
    this.child = child;
  }
}

/// Implementation of [RenderProxyBox].
///
/// Use this mixin in situations where the proxying behavior
/// of [RenderProxyBox] is desired but inheriting from [RenderProxyBox] is
/// impractical (e.g. because you want to inherit from a different class).
///
/// If a class already inherits from [RenderProxyBox] and also uses this mixin,
/// you can safely delete the use of the mixin.
@optionalTypeArgs
mixin RenderProxyBoxMixin<T extends RenderBox> on RenderBox, RenderObjectWithChildMixin<T> {
  @override
  void setupParentData(RenderObject child) {
    // We don't actually use the offset argument in BoxParentData, so let's
    // avoid allocating it at all.
    if (child.parentData is! ParentData) {
      child.parentData = ParentData();
    }
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
    return child?.getDistanceToActualBaseline(baseline) ??
        super.computeDistanceToActualBaseline(baseline);
  }

  @override
  @protected
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    return child?.getDryBaseline(constraints, baseline);
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    return child?.getDryLayout(constraints) ?? computeSizeForNoChild(constraints);
  }

  @override
  void performLayout() {
    size =
        (child?..layout(constraints, parentUsesSize: true))?.size ??
        computeSizeForNoChild(constraints);
    return;
  }

  /// Calculate the size the [RenderProxyBox] would have under the given
  /// [BoxConstraints] for the case where it does not have a child.
  Size computeSizeForNoChild(BoxConstraints constraints) {
    return constraints.smallest;
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    return child?.hitTest(result, position: position) ?? false;
  }

  @override
  void applyPaintTransform(RenderObject child, Matrix4 transform) {}

  @override
  void paint(PaintingContext context, Offset offset) {
    final RenderBox? child = this.child;
    if (child == null) {
      return;
    }
    context.paintChild(child, offset);
  }
}

/// How to behave during hit tests.
enum HitTestBehavior {
  /// Targets that defer to their children receive events within their bounds
  /// only if one of their children is hit by the hit test.
  deferToChild,

  /// Opaque targets can be hit by hit tests, causing them to both receive
  /// events within their bounds and prevent targets visually behind them from
  /// also receiving events.
  opaque,

  /// Translucent targets both receive events within their bounds and permit
  /// targets visually behind them to also receive events.
  translucent,
}

/// A RenderProxyBox subclass that allows you to customize the
/// hit-testing behavior.
abstract class RenderProxyBoxWithHitTestBehavior extends RenderProxyBox {
  /// Initializes member variables for subclasses.
  ///
  /// By default, the [behavior] is [HitTestBehavior.deferToChild].
  RenderProxyBoxWithHitTestBehavior({
    this.behavior = HitTestBehavior.deferToChild,
    RenderBox? child,
  }) : super(child);

  /// How to behave during hit testing when deciding how the hit test propagates
  /// to children and whether to consider targets behind this one.
  ///
  /// Defaults to [HitTestBehavior.deferToChild].
  ///
  /// See [HitTestBehavior] for the allowed values and their meanings.
  HitTestBehavior behavior;

  @override
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    bool hitTarget = false;
    if (size.contains(position)) {
      hitTarget = hitTestChildren(result, position: position) || hitTestSelf(position);
      if (hitTarget || behavior == HitTestBehavior.translucent) {
        result.add(BoxHitTestEntry(this, position));
      }
    }
    return hitTarget;
  }

  @override
  bool hitTestSelf(Offset position) => behavior == HitTestBehavior.opaque;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(EnumProperty<HitTestBehavior>('behavior', behavior, defaultValue: null));
  }
}

/// Imposes additional constraints on its child.
///
/// A render constrained box proxies most functions in the render box protocol
/// to its child, except that when laying out its child, it tightens the
/// constraints provided by its parent by enforcing the [additionalConstraints]
/// as well.
///
/// For example, if you wanted [child] to have a minimum height of 50.0 logical
/// pixels, you could use `const BoxConstraints(minHeight: 50.0)` as the
/// [additionalConstraints].
class RenderConstrainedBox extends RenderProxyBox {
  /// Creates a render box that constrains its child.
  ///
  /// The [additionalConstraints] argument must be valid.
  RenderConstrainedBox({RenderBox? child, required BoxConstraints additionalConstraints})
    : assert(additionalConstraints.debugAssertIsValid()),
      _additionalConstraints = additionalConstraints,
      super(child);

  /// Additional constraints to apply to [child] during layout.
  BoxConstraints get additionalConstraints => _additionalConstraints;
  BoxConstraints _additionalConstraints;
  set additionalConstraints(BoxConstraints value) {
    assert(value.debugAssertIsValid());
    if (_additionalConstraints == value) {
      return;
    }
    _additionalConstraints = value;
    markNeedsLayout();
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    if (_additionalConstraints.hasBoundedWidth && _additionalConstraints.hasTightWidth) {
      return _additionalConstraints.minWidth;
    }
    final double width = super.computeMinIntrinsicWidth(height);
    assert(width.isFinite);
    if (!_additionalConstraints.hasInfiniteWidth) {
      return _additionalConstraints.constrainWidth(width);
    }
    return width;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    if (_additionalConstraints.hasBoundedWidth && _additionalConstraints.hasTightWidth) {
      return _additionalConstraints.minWidth;
    }
    final double width = super.computeMaxIntrinsicWidth(height);
    assert(width.isFinite);
    if (!_additionalConstraints.hasInfiniteWidth) {
      return _additionalConstraints.constrainWidth(width);
    }
    return width;
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    if (_additionalConstraints.hasBoundedHeight && _additionalConstraints.hasTightHeight) {
      return _additionalConstraints.minHeight;
    }
    final double height = super.computeMinIntrinsicHeight(width);
    assert(height.isFinite);
    if (!_additionalConstraints.hasInfiniteHeight) {
      return _additionalConstraints.constrainHeight(height);
    }
    return height;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    if (_additionalConstraints.hasBoundedHeight && _additionalConstraints.hasTightHeight) {
      return _additionalConstraints.minHeight;
    }
    final double height = super.computeMaxIntrinsicHeight(width);
    assert(height.isFinite);
    if (!_additionalConstraints.hasInfiniteHeight) {
      return _additionalConstraints.constrainHeight(height);
    }
    return height;
  }

  @override
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    return child?.getDryBaseline(_additionalConstraints.enforce(constraints), baseline);
  }

  @override
  void performLayout() {
    final BoxConstraints constraints = this.constraints;
    if (child != null) {
      child!.layout(_additionalConstraints.enforce(constraints), parentUsesSize: true);
      size = child!.size;
    } else {
      size = _additionalConstraints.enforce(constraints).constrain(Size.zero);
    }
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    return child?.getDryLayout(_additionalConstraints.enforce(constraints)) ??
        _additionalConstraints.enforce(constraints).constrain(Size.zero);
  }

  @override
  void debugPaintSize(PaintingContext context, Offset offset) {
    super.debugPaintSize(context, offset);
    assert(() {
      final Paint paint;
      if (child == null || child!.size.isEmpty) {
        paint = Paint()..color = const Color(0x90909090);
        context.canvas.drawRect(offset & size, paint);
      }
      return true;
    }());
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      DiagnosticsProperty<BoxConstraints>('additionalConstraints', additionalConstraints),
    );
  }
}

/// Constrains the child's [BoxConstraints.maxWidth] and
/// [BoxConstraints.maxHeight] if they're otherwise unconstrained.
///
/// This has the effect of giving the child a natural dimension in unbounded
/// environments. For example, by providing a [maxHeight] to a widget that
/// normally tries to be as big as possible, the widget will normally size
/// itself to fit its parent, but when placed in a vertical list, it will take
/// on the given height.
///
/// This is useful when composing widgets that normally try to match their
/// parents' size, so that they behave reasonably in lists (which are
/// unbounded).
class RenderLimitedBox extends RenderProxyBox {
  /// Creates a render box that imposes a maximum width or maximum height on its
  /// child if the child is otherwise unconstrained.
  ///
  /// The [maxWidth] and [maxHeight] arguments not be null and must be
  /// non-negative.
  RenderLimitedBox({
    RenderBox? child,
    double maxWidth = double.infinity,
    double maxHeight = double.infinity,
  }) : assert(maxWidth >= 0.0),
       assert(maxHeight >= 0.0),
       _maxWidth = maxWidth,
       _maxHeight = maxHeight,
       super(child);

  /// The value to use for maxWidth if the incoming maxWidth constraint is infinite.
  double get maxWidth => _maxWidth;
  double _maxWidth;
  set maxWidth(double value) {
    assert(value >= 0.0);
    if (_maxWidth == value) {
      return;
    }
    _maxWidth = value;
    markNeedsLayout();
  }

  /// The value to use for maxHeight if the incoming maxHeight constraint is infinite.
  double get maxHeight => _maxHeight;
  double _maxHeight;
  set maxHeight(double value) {
    assert(value >= 0.0);
    if (_maxHeight == value) {
      return;
    }
    _maxHeight = value;
    markNeedsLayout();
  }

  BoxConstraints _limitConstraints(BoxConstraints constraints) {
    return BoxConstraints(
      minWidth: constraints.minWidth,
      maxWidth:
          constraints.hasBoundedWidth ? constraints.maxWidth : constraints.constrainWidth(maxWidth),
      minHeight: constraints.minHeight,
      maxHeight:
          constraints.hasBoundedHeight
              ? constraints.maxHeight
              : constraints.constrainHeight(maxHeight),
    );
  }

  Size _computeSize({required BoxConstraints constraints, required ChildLayouter layoutChild}) {
    if (child != null) {
      final Size childSize = layoutChild(child!, _limitConstraints(constraints));
      return constraints.constrain(childSize);
    }
    return _limitConstraints(constraints).constrain(Size.zero);
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    return _computeSize(constraints: constraints, layoutChild: ChildLayoutHelper.dryLayoutChild);
  }

  @override
  void performLayout() {
    size = _computeSize(constraints: constraints, layoutChild: ChildLayoutHelper.layoutChild);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('maxWidth', maxWidth, defaultValue: double.infinity));
    properties.add(DoubleProperty('maxHeight', maxHeight, defaultValue: double.infinity));
  }
}

/// Attempts to size the child to a specific aspect ratio.
///
/// The render object first tries the largest width permitted by the layout
/// constraints. The height of the render object is determined by applying the
/// given aspect ratio to the width, expressed as a ratio of width to height.
///
/// For example, a 16:9 width:height aspect ratio would have a value of
/// 16.0/9.0. If the maximum width is infinite, the initial width is determined
/// by applying the aspect ratio to the maximum height.
///
/// Now consider a second example, this time with an aspect ratio of 2.0 and
/// layout constraints that require the width to be between 0.0 and 100.0 and
/// the height to be between 0.0 and 100.0. We'll select a width of 100.0 (the
/// biggest allowed) and a height of 50.0 (to match the aspect ratio).
///
/// In that same situation, if the aspect ratio is 0.5, we'll also select a
/// width of 100.0 (still the biggest allowed) and we'll attempt to use a height
/// of 200.0. Unfortunately, that violates the constraints because the child can
/// be at most 100.0 pixels tall. The render object will then take that value
/// and apply the aspect ratio again to obtain a width of 50.0. That width is
/// permitted by the constraints and the child receives a width of 50.0 and a
/// height of 100.0. If the width were not permitted, the render object would
/// continue iterating through the constraints. If the render object does not
/// find a feasible size after consulting each constraint, the render object
/// will eventually select a size for the child that meets the layout
/// constraints but fails to meet the aspect ratio constraints.
class RenderAspectRatio extends RenderProxyBox {
  /// Creates as render object with a specific aspect ratio.
  ///
  /// The [aspectRatio] argument must be a finite, positive value.
  RenderAspectRatio({RenderBox? child, required double aspectRatio})
    : assert(aspectRatio > 0.0),
      assert(aspectRatio.isFinite),
      _aspectRatio = aspectRatio,
      super(child);

  /// The aspect ratio to attempt to use.
  ///
  /// The aspect ratio is expressed as a ratio of width to height. For example,
  /// a 16:9 width:height aspect ratio would have a value of 16.0/9.0.
  double get aspectRatio => _aspectRatio;
  double _aspectRatio;
  set aspectRatio(double value) {
    assert(value > 0.0);
    assert(value.isFinite);
    if (_aspectRatio == value) {
      return;
    }
    _aspectRatio = value;
    markNeedsLayout();
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    if (height.isFinite) {
      return height * _aspectRatio;
    }
    return child?.getMinIntrinsicWidth(height) ?? 0.0;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    if (height.isFinite) {
      return height * _aspectRatio;
    }
    return child?.getMaxIntrinsicWidth(height) ?? 0.0;
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    if (width.isFinite) {
      return width / _aspectRatio;
    }
    return child?.getMinIntrinsicHeight(width) ?? 0.0;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    if (width.isFinite) {
      return width / _aspectRatio;
    }
    return child?.getMaxIntrinsicHeight(width) ?? 0.0;
  }

  Size _applyAspectRatio(BoxConstraints constraints) {
    assert(constraints.debugAssertIsValid());
    assert(() {
      if (!constraints.hasBoundedWidth && !constraints.hasBoundedHeight) {
        throw FlutterError(
          '$runtimeType has unbounded constraints.\n'
          'This $runtimeType was given an aspect ratio of $aspectRatio but was given '
          'both unbounded width and unbounded height constraints. Because both '
          "constraints were unbounded, this render object doesn't know how much "
          'size to consume.',
        );
      }
      return true;
    }());

    if (constraints.isTight) {
      return constraints.smallest;
    }

    double width = constraints.maxWidth;
    double height;

    // We default to picking the height based on the width, but if the width
    // would be infinite, that's not sensible so we try to infer the height
    // from the width.
    if (width.isFinite) {
      height = width / _aspectRatio;
    } else {
      height = constraints.maxHeight;
      width = height * _aspectRatio;
    }

    // Similar to RenderImage, we iteratively attempt to fit within the given
    // constraints while maintaining the given aspect ratio. The order of
    // applying the constraints is also biased towards inferring the height
    // from the width.

    if (width > constraints.maxWidth) {
      width = constraints.maxWidth;
      height = width / _aspectRatio;
    }

    if (height > constraints.maxHeight) {
      height = constraints.maxHeight;
      width = height * _aspectRatio;
    }

    if (width < constraints.minWidth) {
      width = constraints.minWidth;
      height = width / _aspectRatio;
    }

    if (height < constraints.minHeight) {
      height = constraints.minHeight;
      width = height * _aspectRatio;
    }

    return constraints.constrain(Size(width, height));
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    return _applyAspectRatio(constraints);
  }

  @override
  double? computeDryBaseline(BoxConstraints constraints, TextBaseline baseline) {
    return super.computeDryBaseline(BoxConstraints.tight(getDryLayout(constraints)), baseline);
  }

  @override
  void performLayout() {
    size = getDryLayout(constraints);
    child?.layout(BoxConstraints.tight(size));
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('aspectRatio', aspectRatio));
  }
}

/// Sizes its child to the child's maximum intrinsic width.
///
/// This class is useful, for example, when unlimited width is available and
/// you would like a child that would otherwise attempt to expand infinitely to
/// instead size itself to a more reasonable width.
///
/// The constraints that this object passes to its child will adhere to the
/// parent's constraints, so if the constraints are not large enough to satisfy
/// the child's maximum intrinsic width, then the child will get less width
/// than it otherwise would. Likewise, if the minimum width constraint is
/// larger than the child's maximum intrinsic width, the child will be given
/// more width than it otherwise would.
///
/// If [stepWidth] is non-null, the child's width will be snapped to a multiple
/// of the [stepWidth]. Similarly, if [stepHeight] is non-null, the child's
/// height will be snapped to a multiple of the [stepHeight].
///
/// This class is relatively expensive, because it adds a speculative layout
/// pass before the final layout phase. Avoid using it where possible. In the
/// worst case, this render object can result in a layout that is O(N²) in the
/// depth of the tree.
///
/// See also:
///
///  * [Align], a widget that aligns its child within itself. This can be used
///    to loosen the constraints passed to the [RenderIntrinsicWidth],
///    allowing the [RenderIntrinsicWidth]'s child to be smaller than that of
///    its parent.
///  * [Row], which when used with [CrossAxisAlignment.stretch] can be used
///    to loosen just the width constraints that are passed to the
///    [RenderIntrinsicWidth], allowing the [RenderIntrinsicWidth]'s child's
///    width to be smaller than that of its parent.
class RenderIntrinsicWidth extends RenderProxyBox {
  /// Creates a render object that sizes itself to its child's intrinsic width.
  ///
  /// If [stepWidth] is non-null it must be > 0.0. Similarly If [stepHeight] is
  /// non-null it must be > 0.0.
  RenderIntrinsicWidth({double? stepWidth, double? stepHeight, RenderBox? child})
    : assert(stepWidth == null || stepWidth > 0.0),
      assert(stepHeight == null || stepHeight > 0.0),
      _stepWidth = stepWidth,
      _stepHeight = stepHeight,
      super(child);

  /// If non-null, force the child's width to be a multiple of this value.
  ///
  /// This value must be null or > 0.0.
  double? get stepWidth => _stepWidth;
  double? _stepWidth;
  set stepWidth(double? value) {
    assert(value == null || value > 0.0);
    if (value == _stepWidth) {
      return;
    }
    _stepWidth = value;
    markNeedsLayout();
  }

  /// If non-null, force the child's height to be a multiple of this value.
  ///
  /// This value must be null or > 0.0.
  double? get stepHeight => _stepHeight;
  double? _stepHeight;
  set stepHeight(double? value) {
    assert(value == null || value > 0.0);
    if (value == _stepHeight) {
      return;
    }
    _stepHeight = value;
    markNeedsLayout();
  }

  static double _applyStep(double input, double? step) {
    assert(input.isFinite);
    if (step == null) {
      return input;
    }
    return (input / step).ceil() * step;
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    return getMaxIntrinsicWidth(height);
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    if (child == null) {
      return 0.0;
    }
    final double width = child!.getMaxIntrinsicWidth(height);
    return _applyStep(width, _stepWidth);
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    if (child == null) {
      return 0.0;
    }
    if (!width.isFinite) {
      width = getMaxIntrinsicWidth(double.infinity);
    }
    assert(width.isFinite);
    final double height = child!.getMinIntrinsicHeight(width);
    return _applyStep(height, _stepHeight);
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    if (child == null) {
      return 0.0;
    }
    if (!width.isFinite) {
      width = getMaxIntrinsicWidth(double.infinity);
    }
    assert(width.isFinite);
    final double height = child!.getMaxIntrinsicHeight(width);
    return _applyStep(height, _stepHeight);
  }

  BoxConstraints _childConstraints(RenderBox child, BoxConstraints constraints) {
    return constraints.tighten(
      width:
          constraints.hasTightWidth
              ? null
              : _applyStep(child.getMaxIntrinsicWidth(constraints.maxHeight), _stepWidth),
      height:
          stepHeight == null
              ? null
              : _applyStep(child.getMaxIntrinsicHeight(constraints.maxWidth), _stepHeight),
    );
  }

  Size _computeSize({required ChildLayouter layoutChild, required BoxConstraints constraints}) {
    final RenderBox? child = this.child;
    return child == null
        ? constraints.smallest
        : layoutChild(child, _childConstraints(child, constraints));
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    return _computeSize(layoutChild: ChildLayoutHelper.dryLayoutChild, constraints: constraints);
  }

  @override
  double? computeDryBaseline(BoxConstraints constraints, TextBaseline baseline) {
    final RenderBox? child = this.child;
    return child?.getDryBaseline(_childConstraints(child, constraints), baseline);
  }

  @override
  void performLayout() {
    size = _computeSize(layoutChild: ChildLayoutHelper.layoutChild, constraints: constraints);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('stepWidth', stepWidth));
    properties.add(DoubleProperty('stepHeight', stepHeight));
  }
}

/// Sizes its child to the child's intrinsic height.
///
/// This class is useful, for example, when unlimited height is available and
/// you would like a child that would otherwise attempt to expand infinitely to
/// instead size itself to a more reasonable height.
///
/// The constraints that this object passes to its child will adhere to the
/// parent's constraints, so if the constraints are not large enough to satisfy
/// the child's maximum intrinsic height, then the child will get less height
/// than it otherwise would. Likewise, if the minimum height constraint is
/// larger than the child's maximum intrinsic height, the child will be given
/// more height than it otherwise would.
///
/// This class is relatively expensive, because it adds a speculative layout
/// pass before the final layout phase. Avoid using it where possible. In the
/// worst case, this render object can result in a layout that is O(N²) in the
/// depth of the tree.
///
/// See also:
///
///  * [Align], a widget that aligns its child within itself. This can be used
///    to loosen the constraints passed to the [RenderIntrinsicHeight],
///    allowing the [RenderIntrinsicHeight]'s child to be smaller than that of
///    its parent.
///  * [Column], which when used with [CrossAxisAlignment.stretch] can be used
///    to loosen just the height constraints that are passed to the
///    [RenderIntrinsicHeight], allowing the [RenderIntrinsicHeight]'s child's
///    height to be smaller than that of its parent.
class RenderIntrinsicHeight extends RenderProxyBox {
  /// Creates a render object that sizes itself to its child's intrinsic height.
  RenderIntrinsicHeight({RenderBox? child}) : super(child);

  @override
  double computeMinIntrinsicWidth(double height) {
    if (child == null) {
      return 0.0;
    }
    if (!height.isFinite) {
      height = child!.getMaxIntrinsicHeight(double.infinity);
    }
    assert(height.isFinite);
    return child!.getMinIntrinsicWidth(height);
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    if (child == null) {
      return 0.0;
    }
    if (!height.isFinite) {
      height = child!.getMaxIntrinsicHeight(double.infinity);
    }
    assert(height.isFinite);
    return child!.getMaxIntrinsicWidth(height);
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    return getMaxIntrinsicHeight(width);
  }

  BoxConstraints _childConstraints(RenderBox child, BoxConstraints constraints) {
    return constraints.hasTightHeight
        ? constraints
        : constraints.tighten(height: child.getMaxIntrinsicHeight(constraints.maxWidth));
  }

  Size _computeSize({required ChildLayouter layoutChild, required BoxConstraints constraints}) {
    final RenderBox? child = this.child;
    return child == null
        ? constraints.smallest
        : layoutChild(child, _childConstraints(child, constraints));
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    return _computeSize(layoutChild: ChildLayoutHelper.dryLayoutChild, constraints: constraints);
  }

  @override
  double? computeDryBaseline(BoxConstraints constraints, TextBaseline baseline) {
    final RenderBox? child = this.child;
    return child?.getDryBaseline(_childConstraints(child, constraints), baseline);
  }

  @override
  void performLayout() {
    size = _computeSize(layoutChild: ChildLayoutHelper.layoutChild, constraints: constraints);
  }
}

/// Excludes the child from baseline computations in the parent.
class RenderIgnoreBaseline extends RenderProxyBox {
  /// Create a render object that causes the parent to ignore the child for baseline computations.
  RenderIgnoreBaseline({RenderBox? child}) : super(child);

  @override
  Null computeDistanceToActualBaseline(TextBaseline baseline) {
    return null;
  }

  @override
  Null computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    return null;
  }
}

/// Makes its child partially transparent.
///
/// This class paints its child into an intermediate buffer and then blends the
/// child back into the scene partially transparent.
///
/// For values of opacity other than 0.0 and 1.0, this class is relatively
/// expensive because it requires painting the child into an intermediate
/// buffer. For the value 0.0, the child is not painted at all. For the
/// value 1.0, the child is painted immediately without an intermediate buffer.
class RenderOpacity extends RenderProxyBox {
  /// Creates a partially transparent render object.
  ///
  /// The [opacity] argument must be between 0.0 and 1.0, inclusive.
  RenderOpacity({double opacity = 1.0, bool alwaysIncludeSemantics = false, RenderBox? child})
    : assert(opacity >= 0.0 && opacity <= 1.0),
      _opacity = opacity,
      _alwaysIncludeSemantics = alwaysIncludeSemantics,
      _alpha = ui.Color.getAlphaFromOpacity(opacity),
      super(child);

  @override
  bool get alwaysNeedsCompositing => child != null && _alpha > 0;

  @override
  bool get isRepaintBoundary => alwaysNeedsCompositing;

  int _alpha;

  /// The fraction to scale the child's alpha value.
  ///
  /// An opacity of 1.0 is fully opaque. An opacity of 0.0 is fully transparent
  /// (i.e., invisible).
  ///
  /// Values 1.0 and 0.0 are painted with a fast path. Other values
  /// require painting the child into an intermediate buffer, which is
  /// expensive.
  double get opacity => _opacity;
  double _opacity;
  set opacity(double value) {
    assert(value >= 0.0 && value <= 1.0);
    if (_opacity == value) {
      return;
    }
    final bool didNeedCompositing = alwaysNeedsCompositing;
    final bool wasVisible = _alpha != 0;
    _opacity = value;
    _alpha = ui.Color.getAlphaFromOpacity(_opacity);
    if (didNeedCompositing != alwaysNeedsCompositing) {
      markNeedsCompositingBitsUpdate();
    }
    markNeedsCompositedLayerUpdate();
    if (wasVisible != (_alpha != 0) && !alwaysIncludeSemantics) {
      markNeedsSemanticsUpdate();
    }
  }

  /// Whether child semantics are included regardless of the opacity.
  ///
  /// If false, semantics are excluded when [opacity] is 0.0.
  ///
  /// Defaults to false.
  bool get alwaysIncludeSemantics => _alwaysIncludeSemantics;
  bool _alwaysIncludeSemantics;
  set alwaysIncludeSemantics(bool value) {
    if (value == _alwaysIncludeSemantics) {
      return;
    }
    _alwaysIncludeSemantics = value;
    markNeedsSemanticsUpdate();
  }

  @override
  bool paintsChild(RenderBox child) {
    assert(child.parent == this);
    return _alpha > 0;
  }

  @override
  OffsetLayer updateCompositedLayer({required covariant OpacityLayer? oldLayer}) {
    final OpacityLayer layer = oldLayer ?? OpacityLayer();
    layer.alpha = _alpha;
    return layer;
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (child == null || _alpha == 0) {
      return;
    }
    super.paint(context, offset);
  }

  @override
  void visitChildrenForSemantics(RenderObjectVisitor visitor) {
    if (child != null && (_alpha != 0 || alwaysIncludeSemantics)) {
      visitor(child!);
    }
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('opacity', opacity));
    properties.add(
      FlagProperty(
        'alwaysIncludeSemantics',
        value: alwaysIncludeSemantics,
        ifTrue: 'alwaysIncludeSemantics',
      ),
    );
  }
}

/// Implementation of [RenderAnimatedOpacity] and [RenderSliverAnimatedOpacity].
///
/// This mixin allows the logic of animating opacity to be used with different
/// layout models, e.g. the way that [RenderAnimatedOpacity] uses it for [RenderBox]
/// and [RenderSliverAnimatedOpacity] uses it for [RenderSliver].
mixin RenderAnimatedOpacityMixin<T extends RenderObject> on RenderObjectWithChildMixin<T> {
  int? _alpha;

  @override
  bool get isRepaintBoundary => child != null && _currentlyIsRepaintBoundary!;
  bool? _currentlyIsRepaintBoundary;

  @override
  OffsetLayer updateCompositedLayer({required covariant OpacityLayer? oldLayer}) {
    final OpacityLayer updatedLayer = oldLayer ?? OpacityLayer();
    updatedLayer.alpha = _alpha;
    return updatedLayer;
  }

  /// The animation that drives this render object's opacity.
  ///
  /// An opacity of 1.0 is fully opaque. An opacity of 0.0 is fully transparent
  /// (i.e., invisible).
  ///
  /// To change the opacity of a child in a static manner, not animated,
  /// consider [RenderOpacity] instead.
  ///
  /// This getter cannot be read until the value has been set. It should be set
  /// by the constructor of the class in which this mixin is included.
  Animation<double> get opacity => _opacity!;
  Animation<double>? _opacity;
  set opacity(Animation<double> value) {
    if (_opacity == value) {
      return;
    }
    if (attached && _opacity != null) {
      opacity.removeListener(_updateOpacity);
    }
    _opacity = value;
    if (attached) {
      opacity.addListener(_updateOpacity);
    }
    _updateOpacity();
  }

  /// Whether child semantics are included regardless of the opacity.
  ///
  /// If false, semantics are excluded when [opacity] is 0.0.
  ///
  /// Defaults to false.
  ///
  /// This getter cannot be read until the value has been set. It should be set
  /// by the constructor of the class in which this mixin is included.
  bool get alwaysIncludeSemantics => _alwaysIncludeSemantics!;
  bool? _alwaysIncludeSemantics;
  set alwaysIncludeSemantics(bool value) {
    if (value == _alwaysIncludeSemantics) {
      return;
    }
    _alwaysIncludeSemantics = value;
    markNeedsSemanticsUpdate();
  }

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    opacity.addListener(_updateOpacity);
    _updateOpacity(); // in case it changed while we weren't listening
  }

  @override
  void detach() {
    opacity.removeListener(_updateOpacity);
    super.detach();
  }

  void _updateOpacity() {
    final int? oldAlpha = _alpha;
    _alpha = ui.Color.getAlphaFromOpacity(opacity.value);
    if (oldAlpha != _alpha) {
      final bool? wasRepaintBoundary = _currentlyIsRepaintBoundary;
      _currentlyIsRepaintBoundary = _alpha! > 0;
      if (child != null && wasRepaintBoundary != _currentlyIsRepaintBoundary) {
        markNeedsCompositingBitsUpdate();
      }
      markNeedsCompositedLayerUpdate();
      if (oldAlpha == 0 || _alpha == 0) {
        markNeedsSemanticsUpdate();
      }
    }
  }

  @override
  bool paintsChild(RenderObject child) {
    assert(child.parent == this);
    return opacity.value > 0;
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (_alpha == 0) {
      return;
    }
    super.paint(context, offset);
  }

  @override
  void visitChildrenForSemantics(RenderObjectVisitor visitor) {
    if (child != null && (_alpha != 0 || alwaysIncludeSemantics)) {
      visitor(child!);
    }
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<Animation<double>>('opacity', opacity));
    properties.add(
      FlagProperty(
        'alwaysIncludeSemantics',
        value: alwaysIncludeSemantics,
        ifTrue: 'alwaysIncludeSemantics',
      ),
    );
  }
}

/// Makes its child partially transparent, driven from an [Animation].
///
/// This is a variant of [RenderOpacity] that uses an [Animation<double>] rather
/// than a [double] to control the opacity.
class RenderAnimatedOpacity extends RenderProxyBox with RenderAnimatedOpacityMixin<RenderBox> {
  /// Creates a partially transparent render object.
  RenderAnimatedOpacity({
    required Animation<double> opacity,
    bool alwaysIncludeSemantics = false,
    RenderBox? child,
  }) : super(child) {
    this.opacity = opacity;
    this.alwaysIncludeSemantics = alwaysIncludeSemantics;
  }
}

/// Signature for a function that creates a [Shader] for a given [Rect].
///
/// Used by [RenderShaderMask] and the [ShaderMask] widget.
typedef ShaderCallback = Shader Function(Rect bounds);

/// Applies a mask generated by a [Shader] to its child.
///
/// For example, [RenderShaderMask] can be used to gradually fade out the edge
/// of a child by using a [ui.Gradient.linear] mask.
class RenderShaderMask extends RenderProxyBox {
  /// Creates a render object that applies a mask generated by a [Shader] to its child.
  RenderShaderMask({
    RenderBox? child,
    required ShaderCallback shaderCallback,
    BlendMode blendMode = BlendMode.modulate,
  }) : _shaderCallback = shaderCallback,
       _blendMode = blendMode,
       super(child);

  @override
  ShaderMaskLayer? get layer => super.layer as ShaderMaskLayer?;

  /// Called to creates the [Shader] that generates the mask.
  ///
  /// The shader callback is called with the current size of the child so that
  /// it can customize the shader to the size and location of the child.
  ///
  /// The rectangle will always be at the origin when called by
  /// [RenderShaderMask].
  // TODO(abarth): Use the delegate pattern here to avoid generating spurious
  // repaints when the ShaderCallback changes identity.
  ShaderCallback get shaderCallback => _shaderCallback;
  ShaderCallback _shaderCallback;
  set shaderCallback(ShaderCallback value) {
    if (_shaderCallback == value) {
      return;
    }
    _shaderCallback = value;
    markNeedsPaint();
  }

  /// The [BlendMode] to use when applying the shader to the child.
  ///
  /// The default, [BlendMode.modulate], is useful for applying an alpha blend
  /// to the child. Other blend modes can be used to create other effects.
  BlendMode get blendMode => _blendMode;
  BlendMode _blendMode;
  set blendMode(BlendMode value) {
    if (_blendMode == value) {
      return;
    }
    _blendMode = value;
    markNeedsPaint();
  }

  @override
  bool get alwaysNeedsCompositing => child != null;

  @override
  void paint(PaintingContext context, Offset offset) {
    if (child != null) {
      assert(needsCompositing);
      layer ??= ShaderMaskLayer();
      layer!
        ..shader = _shaderCallback(Offset.zero & size)
        ..maskRect = offset & size
        ..blendMode = _blendMode;
      context.pushLayer(layer!, super.paint, offset);
      assert(() {
        layer!.debugCreator = debugCreator;
        return true;
      }());
    } else {
      layer = null;
    }
  }
}

/// Applies a filter to the existing painted content and then paints [child].
///
/// This effect is relatively expensive, especially if the filter is non-local,
/// such as a blur.
class RenderBackdropFilter extends RenderProxyBox {
  /// Creates a backdrop filter.
  //
  /// The [blendMode] argument defaults to [BlendMode.srcOver].
  RenderBackdropFilter({
    RenderBox? child,
    required ui.ImageFilter filter,
    BlendMode blendMode = BlendMode.srcOver,
    bool enabled = true,
    BackdropKey? backdropKey,
  }) : _filter = filter,
       _enabled = enabled,
       _blendMode = blendMode,
       _backdropKey = backdropKey,
       super(child);

  @override
  BackdropFilterLayer? get layer => super.layer as BackdropFilterLayer?;

  /// Whether or not the backdrop filter operation will be applied to the child.
  bool get enabled => _enabled;
  bool _enabled;
  set enabled(bool value) {
    if (enabled == value) {
      return;
    }
    _enabled = value;
    markNeedsPaint();
  }

  /// The image filter to apply to the existing painted content before painting
  /// the child.
  ///
  /// For example, consider using [ui.ImageFilter.blur] to create a backdrop
  /// blur effect.
  ui.ImageFilter get filter => _filter;
  ui.ImageFilter _filter;
  set filter(ui.ImageFilter value) {
    if (_filter == value) {
      return;
    }
    _filter = value;
    markNeedsPaint();
  }

  /// The blend mode to use to apply the filtered background content onto the background
  /// surface.
  ///
  /// {@macro flutter.widgets.BackdropFilter.blendMode}
  BlendMode get blendMode => _blendMode;
  BlendMode _blendMode;
  set blendMode(BlendMode value) {
    if (_blendMode == value) {
      return;
    }
    _blendMode = value;
    markNeedsPaint();
  }

  /// The backdrop key that identifies the [BackdropGroup] this filter will
  /// read from.
  ///
  /// The default value for the backdrop key is `null`.
  BackdropKey? get backdropKey => _backdropKey;
  BackdropKey? _backdropKey;
  set backdropKey(BackdropKey? value) {
    if (value == _backdropKey) {
      return;
    }
    _backdropKey = value;
    markNeedsPaint();
  }

  @override
  bool get alwaysNeedsCompositing => child != null;

  @override
  void paint(PaintingContext context, Offset offset) {
    if (!_enabled) {
      super.paint(context, offset);
      return;
    }

    if (child != null) {
      assert(needsCompositing);
      layer ??= BackdropFilterLayer();
      layer!.filter = _filter;
      layer!.blendMode = _blendMode;
      layer!.backdropKey = _backdropKey;
      context.pushLayer(layer!, super.paint, offset);
      assert(() {
        layer!.debugCreator = debugCreator;
        return true;
      }());
    } else {
      layer = null;
    }
  }
}

/// An interface for providing custom clips.
///
/// This class is used by a number of clip widgets (e.g., [ClipRect] and
/// [ClipPath]).
///
/// The [getClip] method is called whenever the custom clip needs to be updated.
///
/// The [shouldReclip] method is called when a new instance of the class
/// is provided, to check if the new instance actually represents different
/// information.
///
/// The most efficient way to update the clip provided by this class is to
/// supply a `reclip` argument to the constructor of the [CustomClipper]. The
/// custom object will listen to this animation and update the clip whenever the
/// animation ticks, avoiding both the build and layout phases of the pipeline.
///
/// See also:
///
///  * [ClipRect], which can be customized with a [CustomClipper<Rect>].
///  * [ClipRRect], which can be customized with a [CustomClipper<RRect>].
///  * [ClipOval], which can be customized with a [CustomClipper<Rect>].
///  * [ClipPath], which can be customized with a [CustomClipper<Path>].
///  * [ShapeBorderClipper], for specifying a clip path using a [ShapeBorder].
abstract class CustomClipper<T> extends Listenable {
  /// Creates a custom clipper.
  ///
  /// The clipper will update its clip whenever [reclip] notifies its listeners.
  const CustomClipper({Listenable? reclip}) : _reclip = reclip;

  final Listenable? _reclip;

  /// Register a closure to be notified when it is time to reclip.
  ///
  /// The [CustomClipper] implementation merely forwards to the same method on
  /// the [Listenable] provided to the constructor in the `reclip` argument, if
  /// it was not null.
  @override
  void addListener(VoidCallback listener) => _reclip?.addListener(listener);

  /// Remove a previously registered closure from the list of closures that the
  /// object notifies when it is time to reclip.
  ///
  /// The [CustomClipper] implementation merely forwards to the same method on
  /// the [Listenable] provided to the constructor in the `reclip` argument, if
  /// it was not null.
  @override
  void removeListener(VoidCallback listener) => _reclip?.removeListener(listener);

  /// Returns a description of the clip given that the render object being
  /// clipped is of the given size.
  T getClip(Size size);

  /// Returns an approximation of the clip returned by [getClip], as
  /// an axis-aligned Rect. This is used by the semantics layer to
  /// determine whether widgets should be excluded.
  ///
  /// By default, this returns a rectangle that is the same size as
  /// the RenderObject. If getClip returns a shape that is roughly the
  /// same size as the RenderObject (e.g. it's a rounded rectangle
  /// with very small arcs in the corners), then this may be adequate.
  Rect getApproximateClipRect(Size size) => Offset.zero & size;

  /// Called whenever a new instance of the custom clipper delegate class is
  /// provided to the clip object, or any time that a new clip object is created
  /// with a new instance of the custom clipper delegate class (which amounts to
  /// the same thing, because the latter is implemented in terms of the former).
  ///
  /// If the new instance represents different information than the old
  /// instance, then the method should return true, otherwise it should return
  /// false.
  ///
  /// If the method returns false, then the [getClip] call might be optimized
  /// away.
  ///
  /// It's possible that the [getClip] method will get called even if
  /// [shouldReclip] returns false or if the [shouldReclip] method is never
  /// called at all (e.g. if the box changes size).
  bool shouldReclip(covariant CustomClipper<T> oldClipper);

  @override
  String toString() => objectRuntimeType(this, 'CustomClipper');
}

/// A [CustomClipper] that clips to the outer path of a [ShapeBorder].
class ShapeBorderClipper extends CustomClipper<Path> {
  /// Creates a [ShapeBorder] clipper.
  ///
  /// The [textDirection] argument must be provided non-null if [shape]
  /// has a text direction dependency (for example if it is expressed in terms
  /// of "start" and "end" instead of "left" and "right"). It may be null if
  /// the border will not need the text direction to paint itself.
  const ShapeBorderClipper({required this.shape, this.textDirection});

  /// The shape border whose outer path this clipper clips to.
  final ShapeBorder shape;

  /// The text direction to use for getting the outer path for [shape].
  ///
  /// [ShapeBorder]s can depend on the text direction (e.g having a "dent"
  /// towards the start of the shape).
  final TextDirection? textDirection;

  /// Returns the outer path of [shape] as the clip.
  @override
  Path getClip(Size size) {
    return shape.getOuterPath(Offset.zero & size, textDirection: textDirection);
  }

  @override
  bool shouldReclip(CustomClipper<Path> oldClipper) {
    if (oldClipper.runtimeType != ShapeBorderClipper) {
      return true;
    }
    final ShapeBorderClipper typedOldClipper = oldClipper as ShapeBorderClipper;
    return typedOldClipper.shape != shape || typedOldClipper.textDirection != textDirection;
  }
}

abstract class _RenderCustomClip<T> extends RenderProxyBox {
  _RenderCustomClip({
    RenderBox? child,
    CustomClipper<T>? clipper,
    Clip clipBehavior = Clip.antiAlias,
  }) : _clipper = clipper,
       _clipBehavior = clipBehavior,
       super(child);

  /// If non-null, determines which clip to use on the child.
  CustomClipper<T>? get clipper => _clipper;
  CustomClipper<T>? _clipper;
  set clipper(CustomClipper<T>? newClipper) {
    if (_clipper == newClipper) {
      return;
    }
    final CustomClipper<T>? oldClipper = _clipper;
    _clipper = newClipper;
    assert(newClipper != null || oldClipper != null);
    if (newClipper == null ||
        oldClipper == null ||
        newClipper.runtimeType != oldClipper.runtimeType ||
        newClipper.shouldReclip(oldClipper)) {
      _markNeedsClip();
    }
    if (attached) {
      oldClipper?.removeListener(_markNeedsClip);
      newClipper?.addListener(_markNeedsClip);
    }
  }

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    _clipper?.addListener(_markNeedsClip);
  }

  @override
  void detach() {
    _clipper?.removeListener(_markNeedsClip);
    super.detach();
  }

  void _markNeedsClip() {
    _clip = null;
    markNeedsPaint();
    markNeedsSemanticsUpdate();
  }

  T get _defaultClip;
  T? _clip;

  Clip get clipBehavior => _clipBehavior;
  set clipBehavior(Clip value) {
    if (value != _clipBehavior) {
      _clipBehavior = value;
      markNeedsPaint();
    }
  }

  Clip _clipBehavior;

  @override
  void performLayout() {
    final Size? oldSize = hasSize ? size : null;
    super.performLayout();
    if (oldSize != size) {
      _clip = null;
    }
  }

  void _updateClip() {
    _clip ??= _clipper?.getClip(size) ?? _defaultClip;
  }

  @override
  Rect? describeApproximatePaintClip(RenderObject child) {
    switch (clipBehavior) {
      case Clip.none:
        return null;
      case Clip.hardEdge:
      case Clip.antiAlias:
      case Clip.antiAliasWithSaveLayer:
        return _clipper?.getApproximateClipRect(size) ?? Offset.zero & size;
    }
  }

  Paint? _debugPaint;
  TextPainter? _debugText;
  @override
  void debugPaintSize(PaintingContext context, Offset offset) {
    assert(() {
      _debugPaint ??=
          Paint()
            ..shader = ui.Gradient.linear(
              Offset.zero,
              const Offset(10.0, 10.0),
              <Color>[
                const Color(0x00000000),
                const Color(0xFFFF00FF),
                const Color(0xFFFF00FF),
                const Color(0x00000000),
              ],
              <double>[0.25, 0.25, 0.75, 0.75],
              TileMode.repeated,
            )
            ..strokeWidth = 2.0
            ..style = PaintingStyle.stroke;
      _debugText ??= TextPainter(
        text: const TextSpan(text: '✂', style: TextStyle(color: Color(0xFFFF00FF), fontSize: 14.0)),
        textDirection: TextDirection.rtl, // doesn't matter, it's one character
      )..layout();
      return true;
    }());
  }

  @override
  void dispose() {
    _debugText?.dispose();
    _debugText = null;
    super.dispose();
  }
}

/// Clips its child using a rectangle.
///
/// By default, [RenderClipRect] prevents its child from painting outside its
/// bounds, but the size and location of the clip rect can be customized using a
/// custom [clipper].
class RenderClipRect extends _RenderCustomClip<Rect> {
  /// Creates a rectangular clip.
  ///
  /// If [clipper] is null, the clip will match the layout size and position of
  /// the child.
  ///
  /// If [clipBehavior] is [Clip.none], no clipping will be applied.
  RenderClipRect({super.child, super.clipper, super.clipBehavior});

  @override
  Rect get _defaultClip => Offset.zero & size;

  @override
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    if (_clipper != null) {
      _updateClip();
      assert(_clip != null);
      if (!_clip!.contains(position)) {
        return false;
      }
    }
    return super.hitTest(result, position: position);
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (child != null) {
      if (clipBehavior != Clip.none) {
        _updateClip();
        layer = context.pushClipRect(
          needsCompositing,
          offset,
          _clip!,
          super.paint,
          clipBehavior: clipBehavior,
          oldLayer: layer as ClipRectLayer?,
        );
      } else {
        context.paintChild(child!, offset);
        layer = null;
      }
    } else {
      layer = null;
    }
  }

  @override
  void debugPaintSize(PaintingContext context, Offset offset) {
    assert(() {
      if (child != null) {
        super.debugPaintSize(context, offset);
        if (clipBehavior != Clip.none) {
          context.canvas.drawRect(_clip!.shift(offset), _debugPaint!);
          _debugText!.paint(
            context.canvas,
            offset + Offset(_clip!.width / 8.0, -_debugText!.text!.style!.fontSize! * 1.1),
          );
        }
      }
      return true;
    }());
  }
}

/// Clips its child using a rounded rectangle.
///
/// By default, [RenderClipRRect] uses its own bounds as the base rectangle for
/// the clip, but the size and location of the clip can be customized using a
/// custom [clipper].
class RenderClipRRect extends _RenderCustomClip<RRect> {
  /// Creates a rounded-rectangular clip.
  ///
  /// The [borderRadius] defaults to [BorderRadius.zero], i.e. a rectangle with
  /// right-angled corners.
  ///
  /// If [clipper] is non-null, then [borderRadius] is ignored.
  ///
  /// If [clipBehavior] is [Clip.none], no clipping will be applied.
  RenderClipRRect({
    super.child,
    BorderRadiusGeometry borderRadius = BorderRadius.zero,
    super.clipper,
    super.clipBehavior,
    TextDirection? textDirection,
  }) : _borderRadius = borderRadius,
       _textDirection = textDirection;

  /// The border radius of the rounded corners.
  ///
  /// Values are clamped so that horizontal and vertical radii sums do not
  /// exceed width/height.
  ///
  /// This value is ignored if [clipper] is non-null.
  BorderRadiusGeometry get borderRadius => _borderRadius;
  BorderRadiusGeometry _borderRadius;
  set borderRadius(BorderRadiusGeometry value) {
    if (_borderRadius == value) {
      return;
    }
    _borderRadius = value;
    _markNeedsClip();
  }

  /// The text direction with which to resolve [borderRadius].
  TextDirection? get textDirection => _textDirection;
  TextDirection? _textDirection;
  set textDirection(TextDirection? value) {
    if (_textDirection == value) {
      return;
    }
    _textDirection = value;
    _markNeedsClip();
  }

  @override
  RRect get _defaultClip => _borderRadius.resolve(textDirection).toRRect(Offset.zero & size);

  @override
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    if (_clipper != null) {
      _updateClip();
      assert(_clip != null);
      if (!_clip!.contains(position)) {
        return false;
      }
    }
    return super.hitTest(result, position: position);
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (child != null) {
      if (clipBehavior != Clip.none) {
        _updateClip();
        layer = context.pushClipRRect(
          needsCompositing,
          offset,
          _clip!.outerRect,
          _clip!,
          super.paint,
          clipBehavior: clipBehavior,
          oldLayer: layer as ClipRRectLayer?,
        );
      } else {
        context.paintChild(child!, offset);
        layer = null;
      }
    } else {
      layer = null;
    }
  }

  @override
  void debugPaintSize(PaintingContext context, Offset offset) {
    assert(() {
      if (child != null) {
        super.debugPaintSize(context, offset);
        if (clipBehavior != Clip.none) {
          context.canvas.drawRRect(_clip!.shift(offset), _debugPaint!);
          _debugText!.paint(
            context.canvas,
            offset + Offset(_clip!.tlRadiusX, -_debugText!.text!.style!.fontSize! * 1.1),
          );
        }
      }
      return true;
    }());
  }
}

/// Clips its child using an oval.
///
/// By default, inscribes an axis-aligned oval into its layout dimensions and
/// prevents its child from painting outside that oval, but the size and
/// location of the clip oval can be customized using a custom [clipper].
class RenderClipOval extends _RenderCustomClip<Rect> {
  /// Creates an oval-shaped clip.
  ///
  /// If [clipper] is null, the oval will be inscribed into the layout size and
  /// position of the child.
  ///
  /// If [clipBehavior] is [Clip.none], no clipping will be applied.
  RenderClipOval({super.child, super.clipper, super.clipBehavior});

  Rect? _cachedRect;
  late Path _cachedPath;

  Path _getClipPath(Rect rect) {
    if (rect != _cachedRect) {
      _cachedRect = rect;
      _cachedPath = Path()..addOval(_cachedRect!);
    }
    return _cachedPath;
  }

  @override
  Rect get _defaultClip => Offset.zero & size;

  @override
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    _updateClip();
    assert(_clip != null);
    final Offset center = _clip!.center;
    // convert the position to an offset from the center of the unit circle
    final Offset offset = Offset(
      (position.dx - center.dx) / _clip!.width,
      (position.dy - center.dy) / _clip!.height,
    );
    // check if the point is outside the unit circle
    if (offset.distanceSquared > 0.25) {
      // x^2 + y^2 > r^2
      return false;
    }
    return super.hitTest(result, position: position);
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (child != null) {
      if (clipBehavior != Clip.none) {
        _updateClip();
        layer = context.pushClipPath(
          needsCompositing,
          offset,
          _clip!,
          _getClipPath(_clip!),
          super.paint,
          clipBehavior: clipBehavior,
          oldLayer: layer as ClipPathLayer?,
        );
      } else {
        context.paintChild(child!, offset);
        layer = null;
      }
    } else {
      layer = null;
    }
  }

  @override
  void debugPaintSize(PaintingContext context, Offset offset) {
    assert(() {
      if (child != null) {
        super.debugPaintSize(context, offset);
        if (clipBehavior != Clip.none) {
          context.canvas.drawPath(_getClipPath(_clip!).shift(offset), _debugPaint!);
          _debugText!.paint(
            context.canvas,
            offset +
                Offset(
                  (_clip!.width - _debugText!.width) / 2.0,
                  -_debugText!.text!.style!.fontSize! * 1.1,
                ),
          );
        }
      }
      return true;
    }());
  }
}

/// Clips its child using a path.
///
/// Takes a delegate whose primary method returns a path that should
/// be used to prevent the child from painting outside the path.
///
/// Clipping to a path is expensive. Certain shapes have more
/// optimized render objects:
///
///  * To clip to a rectangle, consider [RenderClipRect].
///  * To clip to an oval or circle, consider [RenderClipOval].
///  * To clip to a rounded rectangle, consider [RenderClipRRect].
class RenderClipPath extends _RenderCustomClip<Path> {
  /// Creates a path clip.
  ///
  /// If [clipper] is null, the clip will be a rectangle that matches the layout
  /// size and location of the child. However, rather than use this default,
  /// consider using a [RenderClipRect], which can achieve the same effect more
  /// efficiently.
  ///
  /// If [clipBehavior] is [Clip.none], no clipping will be applied.
  RenderClipPath({super.child, super.clipper, super.clipBehavior});

  @override
  Path get _defaultClip => Path()..addRect(Offset.zero & size);

  @override
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    if (_clipper != null) {
      _updateClip();
      assert(_clip != null);
      if (!_clip!.contains(position)) {
        return false;
      }
    }
    return super.hitTest(result, position: position);
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (child != null) {
      if (clipBehavior != Clip.none) {
        _updateClip();
        layer = context.pushClipPath(
          needsCompositing,
          offset,
          Offset.zero & size,
          _clip!,
          super.paint,
          clipBehavior: clipBehavior,
          oldLayer: layer as ClipPathLayer?,
        );
      } else {
        context.paintChild(child!, offset);
        layer = null;
      }
    } else {
      layer = null;
    }
  }

  @override
  void debugPaintSize(PaintingContext context, Offset offset) {
    assert(() {
      if (child != null) {
        super.debugPaintSize(context, offset);
        if (clipBehavior != Clip.none) {
          context.canvas.drawPath(_clip!.shift(offset), _debugPaint!);
          _debugText!.paint(context.canvas, offset);
        }
      }
      return true;
    }());
  }
}

/// A physical model layer casts a shadow based on its [elevation].
///
/// The concrete implementations [RenderPhysicalModel] and [RenderPhysicalShape]
/// determine the actual shape of the physical model.
abstract class _RenderPhysicalModelBase<T> extends _RenderCustomClip<T> {
  /// The [elevation] parameter must be non-negative.
  _RenderPhysicalModelBase({
    required super.child,
    required double elevation,
    required Color color,
    required Color shadowColor,
    super.clipBehavior = Clip.none,
    super.clipper,
  }) : assert(elevation >= 0.0),
       _elevation = elevation,
       _color = color,
       _shadowColor = shadowColor;

  /// The z-coordinate relative to the parent at which to place this material.
  ///
  /// The value is non-negative.
  ///
  /// If [debugDisableShadows] is set, this value is ignored and no shadow is
  /// drawn (an outline is rendered instead).
  double get elevation => _elevation;
  double _elevation;
  set elevation(double value) {
    assert(value >= 0.0);
    if (elevation == value) {
      return;
    }
    final bool didNeedCompositing = alwaysNeedsCompositing;
    _elevation = value;
    if (didNeedCompositing != alwaysNeedsCompositing) {
      markNeedsCompositingBitsUpdate();
    }
    markNeedsPaint();
  }

  /// The shadow color.
  Color get shadowColor => _shadowColor;
  Color _shadowColor;
  set shadowColor(Color value) {
    if (shadowColor == value) {
      return;
    }
    _shadowColor = value;
    markNeedsPaint();
  }

  /// The background color.
  Color get color => _color;
  Color _color;
  set color(Color value) {
    if (color == value) {
      return;
    }
    _color = value;
    markNeedsPaint();
  }

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    super.describeSemanticsConfiguration(config);
    config.elevation = elevation;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder description) {
    super.debugFillProperties(description);
    description.add(DoubleProperty('elevation', elevation));
    description.add(ColorProperty('color', color));
    description.add(ColorProperty('shadowColor', color));
  }
}

/// Creates a physical model layer that clips its child to a rounded
/// rectangle.
///
/// A physical model layer casts a shadow based on its [elevation].
class RenderPhysicalModel extends _RenderPhysicalModelBase<RRect> {
  /// Creates a rounded-rectangular clip.
  ///
  /// The [color] is required.
  ///
  /// The [elevation] parameter must be non-negative.
  RenderPhysicalModel({
    super.child,
    BoxShape shape = BoxShape.rectangle,
    super.clipBehavior,
    BorderRadius? borderRadius,
    super.elevation = 0.0,
    required super.color,
    super.shadowColor = const Color(0xFF000000),
  }) : assert(elevation >= 0.0),
       _shape = shape,
       _borderRadius = borderRadius;

  /// The shape of the layer.
  ///
  /// Defaults to [BoxShape.rectangle]. The [borderRadius] affects the corners
  /// of the rectangle.
  BoxShape get shape => _shape;
  BoxShape _shape;
  set shape(BoxShape value) {
    if (shape == value) {
      return;
    }
    _shape = value;
    _markNeedsClip();
  }

  /// The border radius of the rounded corners.
  ///
  /// Values are clamped so that horizontal and vertical radii sums do not
  /// exceed width/height.
  ///
  /// This property is ignored if the [shape] is not [BoxShape.rectangle].
  ///
  /// The value null is treated like [BorderRadius.zero].
  BorderRadius? get borderRadius => _borderRadius;
  BorderRadius? _borderRadius;
  set borderRadius(BorderRadius? value) {
    if (borderRadius == value) {
      return;
    }
    _borderRadius = value;
    _markNeedsClip();
  }

  @override
  RRect get _defaultClip {
    assert(hasSize);
    final Rect rect = Offset.zero & size;
    return switch (_shape) {
      BoxShape.rectangle => (borderRadius ?? BorderRadius.zero).toRRect(rect),
      BoxShape.circle => RRect.fromRectXY(rect, rect.width / 2, rect.height / 2),
    };
  }

  @override
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    if (_clipper != null) {
      _updateClip();
      assert(_clip != null);
      if (!_clip!.contains(position)) {
        return false;
      }
    }
    return super.hitTest(result, position: position);
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (child == null) {
      layer = null;
      return;
    }

    _updateClip();
    final RRect offsetRRect = _clip!.shift(offset);
    final Path offsetRRectAsPath = Path()..addRRect(offsetRRect);
    bool paintShadows = true;
    assert(() {
      if (debugDisableShadows) {
        if (elevation > 0.0) {
          context.canvas.drawRRect(
            offsetRRect,
            Paint()
              ..color = shadowColor
              ..style = PaintingStyle.stroke
              ..strokeWidth = elevation * 2.0,
          );
        }
        paintShadows = false;
      }
      return true;
    }());

    final Canvas canvas = context.canvas;
    if (elevation != 0.0 && paintShadows) {
      canvas.drawShadow(offsetRRectAsPath, shadowColor, elevation, color.alpha != 0xFF);
    }
    final bool usesSaveLayer = clipBehavior == Clip.antiAliasWithSaveLayer;
    if (!usesSaveLayer) {
      canvas.drawRRect(offsetRRect, Paint()..color = color);
    }
    layer = context.pushClipRRect(
      needsCompositing,
      offset,
      Offset.zero & size,
      _clip!,
      (PaintingContext context, Offset offset) {
        if (usesSaveLayer) {
          // If we want to avoid the bleeding edge artifact
          // (https://github.com/flutter/flutter/issues/18057#issue-328003931)
          // using saveLayer, we have to call drawPaint instead of drawPath as
          // anti-aliased drawPath will always have such artifacts.
          context.canvas.drawPaint(Paint()..color = color);
        }
        super.paint(context, offset);
      },
      oldLayer: layer as ClipRRectLayer?,
      clipBehavior: clipBehavior,
    );

    assert(() {
      layer?.debugCreator = debugCreator;
      return true;
    }());
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder description) {
    super.debugFillProperties(description);
    description.add(DiagnosticsProperty<BoxShape>('shape', shape));
    description.add(DiagnosticsProperty<BorderRadius>('borderRadius', borderRadius));
  }
}

/// Creates a physical shape layer that clips its child to a [Path].
///
/// A physical shape layer casts a shadow based on its [elevation].
///
/// See also:
///
///  * [RenderPhysicalModel], which is optimized for rounded rectangles and
///    circles.
class RenderPhysicalShape extends _RenderPhysicalModelBase<Path> {
  /// Creates an arbitrary shape clip.
  ///
  /// The [color] and [clipper] parameters are required.
  ///
  /// The [elevation] parameter must be non-negative.
  RenderPhysicalShape({
    super.child,
    required CustomClipper<Path> super.clipper,
    super.clipBehavior,
    super.elevation = 0.0,
    required super.color,
    super.shadowColor = const Color(0xFF000000),
  }) : assert(elevation >= 0.0);

  @override
  Path get _defaultClip => Path()..addRect(Offset.zero & size);

  @override
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    if (_clipper != null) {
      _updateClip();
      assert(_clip != null);
      if (!_clip!.contains(position)) {
        return false;
      }
    }
    return super.hitTest(result, position: position);
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (child == null) {
      layer = null;
      return;
    }

    _updateClip();
    final Path offsetPath = _clip!.shift(offset);
    bool paintShadows = true;
    assert(() {
      if (debugDisableShadows) {
        if (elevation > 0.0) {
          context.canvas.drawPath(
            offsetPath,
            Paint()
              ..color = shadowColor
              ..style = PaintingStyle.stroke
              ..strokeWidth = elevation * 2.0,
          );
        }
        paintShadows = false;
      }
      return true;
    }());

    final Canvas canvas = context.canvas;
    if (elevation != 0.0 && paintShadows) {
      canvas.drawShadow(offsetPath, shadowColor, elevation, color.alpha != 0xFF);
    }
    final bool usesSaveLayer = clipBehavior == Clip.antiAliasWithSaveLayer;
    if (!usesSaveLayer) {
      canvas.drawPath(offsetPath, Paint()..color = color);
    }
    layer = context.pushClipPath(
      needsCompositing,
      offset,
      Offset.zero & size,
      _clip!,
      (PaintingContext context, Offset offset) {
        if (usesSaveLayer) {
          // If we want to avoid the bleeding edge artifact
          // (https://github.com/flutter/flutter/issues/18057#issue-328003931)
          // using saveLayer, we have to call drawPaint instead of drawPath as
          // anti-aliased drawPath will always have such artifacts.
          context.canvas.drawPaint(Paint()..color = color);
        }
        super.paint(context, offset);
      },
      oldLayer: layer as ClipPathLayer?,
      clipBehavior: clipBehavior,
    );

    assert(() {
      layer?.debugCreator = debugCreator;
      return true;
    }());
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder description) {
    super.debugFillProperties(description);
    description.add(DiagnosticsProperty<CustomClipper<Path>>('clipper', clipper));
  }
}

/// Where to paint a box decoration.
enum DecorationPosition {
  /// Paint the box decoration behind the children.
  background,

  /// Paint the box decoration in front of the children.
  foreground,
}

/// Paints a [Decoration] either before or after its child paints.
class RenderDecoratedBox extends RenderProxyBox {
  /// Creates a decorated box.
  ///
  /// The [decoration], [position], and [configuration] arguments must not be
  /// null. By default the decoration paints behind the child.
  ///
  /// The [ImageConfiguration] will be passed to the decoration (with the size
  /// filled in) to let it resolve images.
  RenderDecoratedBox({
    required Decoration decoration,
    DecorationPosition position = DecorationPosition.background,
    ImageConfiguration configuration = ImageConfiguration.empty,
    RenderBox? child,
  }) : _decoration = decoration,
       _position = position,
       _configuration = configuration,
       super(child);

  BoxPainter? _painter;

  /// What decoration to paint.
  ///
  /// Commonly a [BoxDecoration].
  Decoration get decoration => _decoration;
  Decoration _decoration;
  set decoration(Decoration value) {
    if (value == _decoration) {
      return;
    }
    _painter?.dispose();
    _painter = null;
    _decoration = value;
    markNeedsPaint();
  }

  /// Whether to paint the box decoration behind or in front of the child.
  DecorationPosition get position => _position;
  DecorationPosition _position;
  set position(DecorationPosition value) {
    if (value == _position) {
      return;
    }
    _position = value;
    markNeedsPaint();
  }

  /// The settings to pass to the decoration when painting, so that it can
  /// resolve images appropriately. See [ImageProvider.resolve] and
  /// [BoxPainter.paint].
  ///
  /// The [ImageConfiguration.textDirection] field is also used by
  /// direction-sensitive [Decoration]s for painting and hit-testing.
  ImageConfiguration get configuration => _configuration;
  ImageConfiguration _configuration;
  set configuration(ImageConfiguration value) {
    if (value == _configuration) {
      return;
    }
    _configuration = value;
    markNeedsPaint();
  }

  @override
  void detach() {
    _painter?.dispose();
    _painter = null;
    super.detach();
    // Since we're disposing of our painter, we won't receive change
    // notifications. We mark ourselves as needing paint so that we will
    // resubscribe to change notifications. If we didn't do this, then, for
    // example, animated GIFs would stop animating when a DecoratedBox gets
    // moved around the tree due to GlobalKey reparenting.
    markNeedsPaint();
  }

  @override
  void dispose() {
    _painter?.dispose();
    super.dispose();
  }

  @override
  bool hitTestSelf(Offset position) {
    return _decoration.hitTest(size, position, textDirection: configuration.textDirection);
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    _painter ??= _decoration.createBoxPainter(markNeedsPaint);
    final ImageConfiguration filledConfiguration = configuration.copyWith(size: size);
    if (position == DecorationPosition.background) {
      int? debugSaveCount;
      assert(() {
        debugSaveCount = context.canvas.getSaveCount();
        return true;
      }());
      _painter!.paint(context.canvas, offset, filledConfiguration);
      assert(() {
        if (debugSaveCount != context.canvas.getSaveCount()) {
          throw FlutterError.fromParts(<DiagnosticsNode>[
            ErrorSummary(
              '${_decoration.runtimeType} painter had mismatching save and restore calls.',
            ),
            ErrorDescription(
              'Before painting the decoration, the canvas save count was $debugSaveCount. '
              'After painting it, the canvas save count was ${context.canvas.getSaveCount()}. '
              'Every call to save() or saveLayer() must be matched by a call to restore().',
            ),
            DiagnosticsProperty<Decoration>(
              'The decoration was',
              decoration,
              style: DiagnosticsTreeStyle.errorProperty,
            ),
            DiagnosticsProperty<BoxPainter>(
              'The painter was',
              _painter,
              style: DiagnosticsTreeStyle.errorProperty,
            ),
          ]);
        }
        return true;
      }());
      if (decoration.isComplex) {
        context.setIsComplexHint();
      }
    }
    super.paint(context, offset);
    if (position == DecorationPosition.foreground) {
      _painter!.paint(context.canvas, offset, filledConfiguration);
      if (decoration.isComplex) {
        context.setIsComplexHint();
      }
    }
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(_decoration.toDiagnosticsNode(name: 'decoration'));
    properties.add(DiagnosticsProperty<ImageConfiguration>('configuration', configuration));
  }
}

/// Applies a transformation before painting its child.
class RenderTransform extends RenderProxyBox {
  /// Creates a render object that transforms its child.
  RenderTransform({
    required Matrix4 transform,
    Offset? origin,
    AlignmentGeometry? alignment,
    TextDirection? textDirection,
    this.transformHitTests = true,
    FilterQuality? filterQuality,
    RenderBox? child,
  }) : super(child) {
    this.transform = transform;
    this.alignment = alignment;
    this.textDirection = textDirection;
    this.filterQuality = filterQuality;
    this.origin = origin;
  }

  /// The origin of the coordinate system (relative to the upper left corner of
  /// this render object) in which to apply the matrix.
  ///
  /// Setting an origin is equivalent to conjugating the transform matrix by a
  /// translation. This property is provided just for convenience.
  Offset? get origin => _origin;
  Offset? _origin;
  set origin(Offset? value) {
    if (_origin == value) {
      return;
    }
    _origin = value;
    markNeedsPaint();
    markNeedsSemanticsUpdate();
  }

  /// The alignment of the origin, relative to the size of the box.
  ///
  /// This is equivalent to setting an origin based on the size of the box.
  /// If it is specified at the same time as an offset, both are applied.
  ///
  /// An [AlignmentDirectional.centerStart] value is the same as an [Alignment]
  /// whose [Alignment.x] value is `-1.0` if [textDirection] is
  /// [TextDirection.ltr], and `1.0` if [textDirection] is [TextDirection.rtl].
  /// Similarly [AlignmentDirectional.centerEnd] is the same as an [Alignment]
  /// whose [Alignment.x] value is `1.0` if [textDirection] is
  /// [TextDirection.ltr], and `-1.0` if [textDirection] is [TextDirection.rtl].
  AlignmentGeometry? get alignment => _alignment;
  AlignmentGeometry? _alignment;
  set alignment(AlignmentGeometry? value) {
    if (_alignment == value) {
      return;
    }
    _alignment = value;
    markNeedsPaint();
    markNeedsSemanticsUpdate();
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
    markNeedsPaint();
    markNeedsSemanticsUpdate();
  }

  @override
  bool get alwaysNeedsCompositing => child != null && _filterQuality != null;

  /// When set to true, hit tests are performed based on the position of the
  /// child as it is painted. When set to false, hit tests are performed
  /// ignoring the transformation.
  ///
  /// [applyPaintTransform], and therefore [localToGlobal] and [globalToLocal],
  /// always honor the transformation, regardless of the value of this property.
  bool transformHitTests;

  Matrix4? _transform;

  /// The matrix to transform the child by during painting. The provided value
  /// is copied on assignment.
  ///
  /// There is no getter for [transform], because [Matrix4] is mutable, and
  /// mutations outside of the control of the render object could not reliably
  /// be reflected in the rendering.
  // ignore: avoid_setters_without_getters
  set transform(Matrix4 value) {
    if (_transform == value) {
      return;
    }
    _transform = Matrix4.copy(value);
    markNeedsPaint();
    markNeedsSemanticsUpdate();
  }

  /// The filter quality with which to apply the transform as a bitmap operation.
  ///
  /// {@macro flutter.widgets.Transform.optional.FilterQuality}
  FilterQuality? get filterQuality => _filterQuality;
  FilterQuality? _filterQuality;
  set filterQuality(FilterQuality? value) {
    if (_filterQuality == value) {
      return;
    }
    final bool didNeedCompositing = alwaysNeedsCompositing;
    _filterQuality = value;
    if (didNeedCompositing != alwaysNeedsCompositing) {
      markNeedsCompositingBitsUpdate();
    }
    markNeedsPaint();
  }

  /// Sets the transform to the identity matrix.
  void setIdentity() {
    _transform!.setIdentity();
    markNeedsPaint();
    markNeedsSemanticsUpdate();
  }

  /// Concatenates a rotation about the x axis into the transform.
  void rotateX(double radians) {
    _transform!.rotateX(radians);
    markNeedsPaint();
    markNeedsSemanticsUpdate();
  }

  /// Concatenates a rotation about the y axis into the transform.
  void rotateY(double radians) {
    _transform!.rotateY(radians);
    markNeedsPaint();
    markNeedsSemanticsUpdate();
  }

  /// Concatenates a rotation about the z axis into the transform.
  void rotateZ(double radians) {
    _transform!.rotateZ(radians);
    markNeedsPaint();
    markNeedsSemanticsUpdate();
  }

  /// Concatenates a translation by (x, y, z) into the transform.
  void translate(double x, [double y = 0.0, double z = 0.0]) {
    _transform!.translate(x, y, z);
    markNeedsPaint();
    markNeedsSemanticsUpdate();
  }

  /// Concatenates a scale into the transform.
  void scale(double x, [double? y, double? z]) {
    _transform!.scale(x, y, z);
    markNeedsPaint();
    markNeedsSemanticsUpdate();
  }

  Matrix4? get _effectiveTransform {
    final Alignment? resolvedAlignment = alignment?.resolve(textDirection);
    if (_origin == null && resolvedAlignment == null) {
      return _transform;
    }
    final Matrix4 result = Matrix4.identity();
    if (_origin != null) {
      result.translate(_origin!.dx, _origin!.dy);
    }
    Offset? translation;
    if (resolvedAlignment != null) {
      translation = resolvedAlignment.alongSize(size);
      result.translate(translation.dx, translation.dy);
    }
    result.multiply(_transform!);
    if (resolvedAlignment != null) {
      result.translate(-translation!.dx, -translation.dy);
    }
    if (_origin != null) {
      result.translate(-_origin!.dx, -_origin!.dy);
    }
    return result;
  }

  @override
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    // RenderTransform objects don't check if they are
    // themselves hit, because it's confusing to think about
    // how the untransformed size and the child's transformed
    // position interact.
    return hitTestChildren(result, position: position);
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    assert(!transformHitTests || _effectiveTransform != null);
    return result.addWithPaintTransform(
      transform: transformHitTests ? _effectiveTransform : null,
      position: position,
      hitTest: (BoxHitTestResult result, Offset position) {
        return super.hitTestChildren(result, position: position);
      },
    );
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (child != null) {
      final Matrix4 transform = _effectiveTransform!;
      if (filterQuality == null) {
        final Offset? childOffset = MatrixUtils.getAsTranslation(transform);
        if (childOffset == null) {
          // if the matrix is singular the children would be compressed to a line or
          // single point, instead short-circuit and paint nothing.
          final double det = transform.determinant();
          if (det == 0 || !det.isFinite) {
            layer = null;
            return;
          }
          layer = context.pushTransform(
            needsCompositing,
            offset,
            transform,
            super.paint,
            oldLayer: layer is TransformLayer ? layer as TransformLayer? : null,
          );
        } else {
          super.paint(context, offset + childOffset);
          layer = null;
        }
      } else {
        final Matrix4 effectiveTransform =
            Matrix4.translationValues(offset.dx, offset.dy, 0.0)
              ..multiply(transform)
              ..translate(-offset.dx, -offset.dy);
        final ui.ImageFilter filter = ui.ImageFilter.matrix(
          effectiveTransform.storage,
          filterQuality: filterQuality!,
        );
        if (layer case final ImageFilterLayer filterLayer) {
          filterLayer.imageFilter = filter;
        } else {
          layer = ImageFilterLayer(imageFilter: filter);
        }
        context.pushLayer(layer!, super.paint, offset);
        assert(() {
          layer!.debugCreator = debugCreator;
          return true;
        }());
      }
    }
  }

  @override
  void applyPaintTransform(RenderBox child, Matrix4 transform) {
    transform.multiply(_effectiveTransform!);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(TransformProperty('transform matrix', _transform));
    properties.add(DiagnosticsProperty<Offset>('origin', origin));
    properties.add(DiagnosticsProperty<AlignmentGeometry>('alignment', alignment));
    properties.add(EnumProperty<TextDirection>('textDirection', textDirection, defaultValue: null));
    properties.add(DiagnosticsProperty<bool>('transformHitTests', transformHitTests));
  }
}

/// Scales and positions its child within itself according to [fit].
class RenderFittedBox extends RenderProxyBox {
  /// Scales and positions its child within itself.
  RenderFittedBox({
    BoxFit fit = BoxFit.contain,
    AlignmentGeometry alignment = Alignment.center,
    TextDirection? textDirection,
    RenderBox? child,
    Clip clipBehavior = Clip.none,
  }) : _fit = fit,
       _alignment = alignment,
       _textDirection = textDirection,
       _clipBehavior = clipBehavior,
       super(child);

  Alignment _resolve() => _resolvedAlignment ??= alignment.resolve(textDirection);
  Alignment? _resolvedAlignment;

  void _markNeedResolution() {
    _resolvedAlignment = null;
    markNeedsPaint();
  }

  bool _fitAffectsLayout(BoxFit fit) {
    switch (fit) {
      case BoxFit.scaleDown:
        return true;
      case BoxFit.contain:
      case BoxFit.cover:
      case BoxFit.fill:
      case BoxFit.fitHeight:
      case BoxFit.fitWidth:
      case BoxFit.none:
        return false;
    }
  }

  /// How to inscribe the child into the space allocated during layout.
  BoxFit get fit => _fit;
  BoxFit _fit;
  set fit(BoxFit value) {
    if (_fit == value) {
      return;
    }
    final BoxFit lastFit = _fit;
    _fit = value;
    if (_fitAffectsLayout(lastFit) || _fitAffectsLayout(value)) {
      markNeedsLayout();
    } else {
      _clearPaintData();
      markNeedsPaint();
    }
  }

  /// How to align the child within its parent's bounds.
  ///
  /// An alignment of (0.0, 0.0) aligns the child to the top-left corner of its
  /// parent's bounds. An alignment of (1.0, 0.5) aligns the child to the middle
  /// of the right edge of its parent's bounds.
  ///
  /// If this is set to an [AlignmentDirectional] object, then
  /// [textDirection] must not be null.
  AlignmentGeometry get alignment => _alignment;
  AlignmentGeometry _alignment;
  set alignment(AlignmentGeometry value) {
    if (_alignment == value) {
      return;
    }
    _alignment = value;
    _clearPaintData();
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
    _clearPaintData();
    _markNeedResolution();
  }

  // TODO(ianh): The intrinsic dimensions of this box are wrong.

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    if (child != null) {
      final Size childSize = child!.getDryLayout(const BoxConstraints());

      switch (fit) {
        case BoxFit.scaleDown:
          final BoxConstraints sizeConstraints = constraints.loosen();
          final Size unconstrainedSize = sizeConstraints
              .constrainSizeAndAttemptToPreserveAspectRatio(childSize);
          return constraints.constrain(unconstrainedSize);
        case BoxFit.contain:
        case BoxFit.cover:
        case BoxFit.fill:
        case BoxFit.fitHeight:
        case BoxFit.fitWidth:
        case BoxFit.none:
          return constraints.constrainSizeAndAttemptToPreserveAspectRatio(childSize);
      }
    } else {
      return constraints.smallest;
    }
  }

  @override
  void performLayout() {
    if (child != null) {
      child!.layout(const BoxConstraints(), parentUsesSize: true);
      switch (fit) {
        case BoxFit.scaleDown:
          final BoxConstraints sizeConstraints = constraints.loosen();
          final Size unconstrainedSize = sizeConstraints
              .constrainSizeAndAttemptToPreserveAspectRatio(child!.size);
          size = constraints.constrain(unconstrainedSize);
        case BoxFit.contain:
        case BoxFit.cover:
        case BoxFit.fill:
        case BoxFit.fitHeight:
        case BoxFit.fitWidth:
        case BoxFit.none:
          size = constraints.constrainSizeAndAttemptToPreserveAspectRatio(child!.size);
      }
      _clearPaintData();
    } else {
      size = constraints.smallest;
    }
  }

  bool? _hasVisualOverflow;
  Matrix4? _transform;

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

  void _clearPaintData() {
    _hasVisualOverflow = null;
    _transform = null;
  }

  void _updatePaintData() {
    if (_transform != null) {
      return;
    }

    if (child == null) {
      _hasVisualOverflow = false;
      _transform = Matrix4.identity();
    } else {
      final Alignment resolvedAlignment = _resolve();
      final Size childSize = child!.size;
      final FittedSizes sizes = applyBoxFit(_fit, childSize, size);
      final double scaleX = sizes.destination.width / sizes.source.width;
      final double scaleY = sizes.destination.height / sizes.source.height;
      final Rect sourceRect = resolvedAlignment.inscribe(sizes.source, Offset.zero & childSize);
      final Rect destinationRect = resolvedAlignment.inscribe(
        sizes.destination,
        Offset.zero & size,
      );
      _hasVisualOverflow =
          sourceRect.width < childSize.width || sourceRect.height < childSize.height;
      assert(scaleX.isFinite && scaleY.isFinite);
      _transform =
          Matrix4.translationValues(destinationRect.left, destinationRect.top, 0.0)
            ..scale(scaleX, scaleY, 1.0)
            ..translate(-sourceRect.left, -sourceRect.top);
      assert(_transform!.storage.every((double value) => value.isFinite));
    }
  }

  TransformLayer? _paintChildWithTransform(PaintingContext context, Offset offset) {
    final Offset? childOffset = MatrixUtils.getAsTranslation(_transform!);
    if (childOffset == null) {
      return context.pushTransform(
        needsCompositing,
        offset,
        _transform!,
        super.paint,
        oldLayer: layer is TransformLayer ? layer! as TransformLayer : null,
      );
    } else {
      super.paint(context, offset + childOffset);
    }
    return null;
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (child == null || size.isEmpty || child!.size.isEmpty) {
      return;
    }
    _updatePaintData();
    assert(child != null);
    if (_hasVisualOverflow! && clipBehavior != Clip.none) {
      layer = context.pushClipRect(
        needsCompositing,
        offset,
        Offset.zero & size,
        _paintChildWithTransform,
        oldLayer: layer is ClipRectLayer ? layer! as ClipRectLayer : null,
        clipBehavior: clipBehavior,
      );
    } else {
      layer = _paintChildWithTransform(context, offset);
    }
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    if (size.isEmpty || (child?.size.isEmpty ?? false)) {
      return false;
    }
    _updatePaintData();
    return result.addWithPaintTransform(
      transform: _transform,
      position: position,
      hitTest: (BoxHitTestResult result, Offset position) {
        return super.hitTestChildren(result, position: position);
      },
    );
  }

  @override
  bool paintsChild(RenderBox child) {
    assert(child.parent == this);
    return !size.isEmpty && !child.size.isEmpty;
  }

  @override
  void applyPaintTransform(RenderBox child, Matrix4 transform) {
    if (!paintsChild(child)) {
      transform.setZero();
    } else {
      _updatePaintData();
      transform.multiply(_transform!);
    }
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(EnumProperty<BoxFit>('fit', fit));
    properties.add(DiagnosticsProperty<AlignmentGeometry>('alignment', alignment));
    properties.add(EnumProperty<TextDirection>('textDirection', textDirection, defaultValue: null));
  }
}

/// Applies a translation transformation before painting its child.
///
/// The translation is expressed as an [Offset] scaled to the child's size. For
/// example, an [Offset] with a `dx` of 0.25 will result in a horizontal
/// translation of one quarter the width of the child.
///
/// Hit tests will only be detected inside the bounds of the
/// [RenderFractionalTranslation], even if the contents are offset such that
/// they overflow.
class RenderFractionalTranslation extends RenderProxyBox {
  /// Creates a render object that translates its child's painting.
  RenderFractionalTranslation({
    required Offset translation,
    this.transformHitTests = true,
    RenderBox? child,
  }) : _translation = translation,
       super(child);

  /// The translation to apply to the child, scaled to the child's size.
  ///
  /// For example, an [Offset] with a `dx` of 0.25 will result in a horizontal
  /// translation of one quarter the width of the child.
  Offset get translation => _translation;
  Offset _translation;
  set translation(Offset value) {
    if (_translation == value) {
      return;
    }
    _translation = value;
    markNeedsPaint();
    markNeedsSemanticsUpdate();
  }

  @override
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    // RenderFractionalTranslation objects don't check if they are
    // themselves hit, because it's confusing to think about
    // how the untransformed size and the child's transformed
    // position interact.
    return hitTestChildren(result, position: position);
  }

  /// When set to true, hit tests are performed based on the position of the
  /// child as it is painted. When set to false, hit tests are performed
  /// ignoring the transformation.
  ///
  /// applyPaintTransform(), and therefore localToGlobal() and globalToLocal(),
  /// always honor the transformation, regardless of the value of this property.
  bool transformHitTests;

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    assert(!debugNeedsLayout);
    return result.addWithPaintOffset(
      offset:
          transformHitTests
              ? Offset(translation.dx * size.width, translation.dy * size.height)
              : null,
      position: position,
      hitTest: (BoxHitTestResult result, Offset position) {
        return super.hitTestChildren(result, position: position);
      },
    );
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    assert(!debugNeedsLayout);
    if (child != null) {
      super.paint(
        context,
        Offset(offset.dx + translation.dx * size.width, offset.dy + translation.dy * size.height),
      );
    }
  }

  @override
  void applyPaintTransform(RenderBox child, Matrix4 transform) {
    transform.translate(translation.dx * size.width, translation.dy * size.height);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<Offset>('translation', translation));
    properties.add(DiagnosticsProperty<bool>('transformHitTests', transformHitTests));
  }
}

/// Signature for listening to [PointerDownEvent] events.
///
/// Used by [Listener] and [RenderPointerListener].
typedef PointerDownEventListener = void Function(PointerDownEvent event);

/// Signature for listening to [PointerMoveEvent] events.
///
/// Used by [Listener] and [RenderPointerListener].
typedef PointerMoveEventListener = void Function(PointerMoveEvent event);

/// Signature for listening to [PointerUpEvent] events.
///
/// Used by [Listener] and [RenderPointerListener].
typedef PointerUpEventListener = void Function(PointerUpEvent event);

/// Signature for listening to [PointerCancelEvent] events.
///
/// Used by [Listener] and [RenderPointerListener].
typedef PointerCancelEventListener = void Function(PointerCancelEvent event);

/// Signature for listening to [PointerPanZoomStartEvent] events.
///
/// Used by [Listener] and [RenderPointerListener].
typedef PointerPanZoomStartEventListener = void Function(PointerPanZoomStartEvent event);

/// Signature for listening to [PointerPanZoomUpdateEvent] events.
///
/// Used by [Listener] and [RenderPointerListener].
typedef PointerPanZoomUpdateEventListener = void Function(PointerPanZoomUpdateEvent event);

/// Signature for listening to [PointerPanZoomEndEvent] events.
///
/// Used by [Listener] and [RenderPointerListener].
typedef PointerPanZoomEndEventListener = void Function(PointerPanZoomEndEvent event);

/// Signature for listening to [PointerSignalEvent] events.
///
/// Used by [Listener] and [RenderPointerListener].
typedef PointerSignalEventListener = void Function(PointerSignalEvent event);

/// Calls callbacks in response to common pointer events.
///
/// It responds to events that can construct gestures, such as when the
/// pointer is pointer is pressed and moved, and then released or canceled.
///
/// It does not respond to events that are exclusive to mouse, such as when the
/// mouse enters and exits a region without pressing any buttons. For
/// these events, use [RenderMouseRegion].
///
/// If it has a child, defers to the child for sizing behavior.
///
/// If it does not have a child, grows to fit the parent-provided constraints.
class RenderPointerListener extends RenderProxyBoxWithHitTestBehavior {
  /// Creates a render object that forwards pointer events to callbacks.
  ///
  /// The [behavior] argument defaults to [HitTestBehavior.deferToChild].
  RenderPointerListener({
    this.onPointerDown,
    this.onPointerMove,
    this.onPointerUp,
    this.onPointerHover,
    this.onPointerCancel,
    this.onPointerPanZoomStart,
    this.onPointerPanZoomUpdate,
    this.onPointerPanZoomEnd,
    this.onPointerSignal,
    super.behavior,
    super.child,
  });

  /// Called when a pointer comes into contact with the screen (for touch
  /// pointers), or has its button pressed (for mouse pointers) at this widget's
  /// location.
  PointerDownEventListener? onPointerDown;

  /// Called when a pointer that triggered an [onPointerDown] changes position.
  PointerMoveEventListener? onPointerMove;

  /// Called when a pointer that triggered an [onPointerDown] is no longer in
  /// contact with the screen.
  PointerUpEventListener? onPointerUp;

  /// Called when a pointer that has not an [onPointerDown] changes position.
  PointerHoverEventListener? onPointerHover;

  /// Called when the input from a pointer that triggered an [onPointerDown] is
  /// no longer directed towards this receiver.
  PointerCancelEventListener? onPointerCancel;

  /// Called when a pan/zoom begins such as from a trackpad gesture.
  PointerPanZoomStartEventListener? onPointerPanZoomStart;

  /// Called when a pan/zoom is updated.
  PointerPanZoomUpdateEventListener? onPointerPanZoomUpdate;

  /// Called when a pan/zoom finishes.
  PointerPanZoomEndEventListener? onPointerPanZoomEnd;

  /// Called when a pointer signal occurs over this object.
  PointerSignalEventListener? onPointerSignal;

  @override
  Size computeSizeForNoChild(BoxConstraints constraints) {
    return constraints.biggest;
  }

  @override
  void handleEvent(PointerEvent event, HitTestEntry entry) {
    assert(debugHandleEvent(event, entry));
    return switch (event) {
      PointerDownEvent() => onPointerDown?.call(event),
      PointerMoveEvent() => onPointerMove?.call(event),
      PointerUpEvent() => onPointerUp?.call(event),
      PointerHoverEvent() => onPointerHover?.call(event),
      PointerCancelEvent() => onPointerCancel?.call(event),
      PointerPanZoomStartEvent() => onPointerPanZoomStart?.call(event),
      PointerPanZoomUpdateEvent() => onPointerPanZoomUpdate?.call(event),
      PointerPanZoomEndEvent() => onPointerPanZoomEnd?.call(event),
      PointerSignalEvent() => onPointerSignal?.call(event),
      _ => null,
    };
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      FlagsSummary<Function?>('listeners', <String, Function?>{
        'down': onPointerDown,
        'move': onPointerMove,
        'up': onPointerUp,
        'hover': onPointerHover,
        'cancel': onPointerCancel,
        'panZoomStart': onPointerPanZoomStart,
        'panZoomUpdate': onPointerPanZoomUpdate,
        'panZoomEnd': onPointerPanZoomEnd,
        'signal': onPointerSignal,
      }, ifEmpty: '<none>'),
    );
  }
}

/// Calls callbacks in response to pointer events that are exclusive to mice.
///
/// It responds to events that are related to hovering, i.e. when the mouse
/// enters, exits (with or without pressing buttons), or moves over a region
/// without pressing buttons.
///
/// It does not respond to common events that construct gestures, such as when
/// the pointer is pressed, moved, then released or canceled. For these events,
/// use [RenderPointerListener].
///
/// If it has a child, it defers to the child for sizing behavior.
///
/// If it does not have a child, it grows to fit the parent-provided constraints.
///
/// See also:
///
///  * [MouseRegion], a widget that listens to hover events using
///    [RenderMouseRegion].
class RenderMouseRegion extends RenderProxyBoxWithHitTestBehavior
    implements MouseTrackerAnnotation {
  /// Creates a render object that forwards pointer events to callbacks.
  ///
  /// All parameters are optional. By default this method creates an opaque
  /// mouse region with no callbacks and cursor being [MouseCursor.defer].
  RenderMouseRegion({
    this.onEnter,
    this.onHover,
    this.onExit,
    MouseCursor cursor = MouseCursor.defer,
    bool validForMouseTracker = true,
    bool opaque = true,
    super.child,
    HitTestBehavior? hitTestBehavior = HitTestBehavior.opaque,
  }) : _cursor = cursor,
       _validForMouseTracker = validForMouseTracker,
       _opaque = opaque,
       super(behavior: hitTestBehavior ?? HitTestBehavior.opaque);

  @override
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    return super.hitTest(result, position: position) && _opaque;
  }

  @override
  void handleEvent(PointerEvent event, HitTestEntry entry) {
    assert(debugHandleEvent(event, entry));
    if (event is PointerHoverEvent) {
      onHover?.call(event);
    }
  }

  /// Whether this object should prevent [RenderMouseRegion]s visually behind it
  /// from detecting the pointer, thus affecting how their [onHover], [onEnter],
  /// and [onExit] behave.
  ///
  /// If [opaque] is true, this object will absorb the mouse pointer and
  /// prevent this object's siblings (or any other objects that are not
  /// ancestors or descendants of this object) from detecting the mouse
  /// pointer even when the pointer is within their areas.
  ///
  /// If [opaque] is false, this object will not affect how [RenderMouseRegion]s
  /// behind it behave, which will detect the mouse pointer as long as the
  /// pointer is within their areas.
  ///
  /// This defaults to true.
  bool get opaque => _opaque;
  bool _opaque;
  set opaque(bool value) {
    if (_opaque != value) {
      _opaque = value;
      // Trigger [MouseTracker]'s device update to recalculate mouse states.
      markNeedsPaint();
    }
  }

  /// How to behave during hit testing.
  ///
  /// This defaults to [HitTestBehavior.opaque] if null.
  HitTestBehavior? get hitTestBehavior => behavior;
  set hitTestBehavior(HitTestBehavior? value) {
    final HitTestBehavior newValue = value ?? HitTestBehavior.opaque;
    if (behavior != newValue) {
      behavior = newValue;
      // Trigger [MouseTracker]'s device update to recalculate mouse states.
      markNeedsPaint();
    }
  }

  @override
  PointerEnterEventListener? onEnter;

  /// Triggered when a pointer has moved onto or within the region without
  /// buttons pressed.
  ///
  /// This callback is not triggered by the movement of the object.
  PointerHoverEventListener? onHover;

  @override
  PointerExitEventListener? onExit;

  @override
  MouseCursor get cursor => _cursor;
  MouseCursor _cursor;
  set cursor(MouseCursor value) {
    if (_cursor != value) {
      _cursor = value;
      // A repaint is needed in order to trigger a device update of
      // [MouseTracker] so that this new value can be found.
      markNeedsPaint();
    }
  }

  @override
  bool get validForMouseTracker => _validForMouseTracker;
  bool _validForMouseTracker;

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    _validForMouseTracker = true;
  }

  @override
  void detach() {
    // It's possible that the renderObject be detached during mouse events
    // dispatching, set the [MouseTrackerAnnotation.validForMouseTracker] false to prevent
    // the callbacks from being called.
    _validForMouseTracker = false;
    super.detach();
  }

  @override
  Size computeSizeForNoChild(BoxConstraints constraints) {
    return constraints.biggest;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      FlagsSummary<Function?>('listeners', <String, Function?>{
        'enter': onEnter,
        'hover': onHover,
        'exit': onExit,
      }, ifEmpty: '<none>'),
    );
    properties.add(
      DiagnosticsProperty<MouseCursor>('cursor', cursor, defaultValue: MouseCursor.defer),
    );
    properties.add(DiagnosticsProperty<bool>('opaque', opaque, defaultValue: true));
    properties.add(
      FlagProperty(
        'validForMouseTracker',
        value: validForMouseTracker,
        defaultValue: true,
        ifFalse: 'invalid for MouseTracker',
      ),
    );
  }
}

/// Creates a separate display list for its child.
///
/// This render object creates a separate display list for its child, which
/// can improve performance if the subtree repaints at different times than
/// the surrounding parts of the tree. Specifically, when the child does not
/// repaint but its parent does, we can re-use the display list we recorded
/// previously. Similarly, when the child repaints but the surround tree does
/// not, we can re-record its display list without re-recording the display list
/// for the surround tree.
///
/// In some cases, it is necessary to place _two_ (or more) repaint boundaries
/// to get a useful effect. Consider, for example, an e-mail application that
/// shows an unread count and a list of e-mails. Whenever a new e-mail comes in,
/// the list would update, but so would the unread count. If only one of these
/// two parts of the application was behind a repaint boundary, the entire
/// application would repaint each time. On the other hand, if both were behind
/// a repaint boundary, a new e-mail would only change those two parts of the
/// application and the rest of the application would not repaint.
///
/// To tell if a particular RenderRepaintBoundary is useful, run your
/// application in debug mode, interacting with it in typical ways, and then
/// call [debugDumpRenderTree]. Each RenderRepaintBoundary will include the
/// ratio of cases where the repaint boundary was useful vs the cases where it
/// was not. These counts can also be inspected programmatically using
/// [debugAsymmetricPaintCount] and [debugSymmetricPaintCount] respectively.
class RenderRepaintBoundary extends RenderProxyBox {
  /// Creates a repaint boundary around [child].
  RenderRepaintBoundary({RenderBox? child}) : super(child);

  @override
  bool get isRepaintBoundary => true;

  /// Capture an image of the current state of this render object and its
  /// children.
  ///
  /// The returned [ui.Image] has uncompressed raw RGBA bytes in the dimensions
  /// of the render object, multiplied by the [pixelRatio].
  ///
  /// To use [toImage], the render object must have gone through the paint phase
  /// (i.e. [debugNeedsPaint] must be false).
  ///
  /// The [pixelRatio] describes the scale between the logical pixels and the
  /// size of the output image. It is independent of the
  /// [dart:ui.FlutterView.devicePixelRatio] for the device, so specifying 1.0
  /// (the default) will give you a 1:1 mapping between logical pixels and the
  /// output pixels in the image.
  ///
  /// {@tool snippet}
  ///
  /// The following is an example of how to go from a `GlobalKey` on a
  /// `RepaintBoundary` to a PNG:
  ///
  /// ```dart
  /// class PngHome extends StatefulWidget {
  ///   const PngHome({super.key});
  ///
  ///   @override
  ///   State<PngHome> createState() => _PngHomeState();
  /// }
  ///
  /// class _PngHomeState extends State<PngHome> {
  ///   GlobalKey globalKey = GlobalKey();
  ///
  ///   Future<void> _capturePng() async {
  ///     final RenderRepaintBoundary boundary = globalKey.currentContext!.findRenderObject()! as RenderRepaintBoundary;
  ///     final ui.Image image = await boundary.toImage();
  ///     final ByteData? byteData = await image.toByteData(format: ui.ImageByteFormat.png);
  ///     final Uint8List pngBytes = byteData!.buffer.asUint8List();
  ///     print(pngBytes);
  ///   }
  ///
  ///   @override
  ///   Widget build(BuildContext context) {
  ///     return RepaintBoundary(
  ///       key: globalKey,
  ///       child: Center(
  ///         child: TextButton(
  ///           onPressed: _capturePng,
  ///           child: const Text('Hello World', textDirection: TextDirection.ltr),
  ///         ),
  ///       ),
  ///     );
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [OffsetLayer.toImage] for a similar API at the layer level.
  ///  * [dart:ui.Scene.toImage] for more information about the image returned.
  Future<ui.Image> toImage({double pixelRatio = 1.0}) {
    assert(!debugNeedsPaint);
    final OffsetLayer offsetLayer = layer! as OffsetLayer;
    return offsetLayer.toImage(Offset.zero & size, pixelRatio: pixelRatio);
  }

  /// Capture an image of the current state of this render object and its
  /// children synchronously.
  ///
  /// The returned [ui.Image] has uncompressed raw RGBA bytes in the dimensions
  /// of the render object, multiplied by the [pixelRatio].
  ///
  /// To use [toImageSync], the render object must have gone through the paint phase
  /// (i.e. [debugNeedsPaint] must be false).
  ///
  /// The [pixelRatio] describes the scale between the logical pixels and the
  /// size of the output image. It is independent of the
  /// [dart:ui.FlutterView.devicePixelRatio] for the device, so specifying 1.0
  /// (the default) will give you a 1:1 mapping between logical pixels and the
  /// output pixels in the image.
  ///
  /// This API functions like [toImage], except that rasterization begins eagerly
  /// on the raster thread and the image is returned before this is completed.
  ///
  /// {@tool snippet}
  ///
  /// The following is an example of how to go from a `GlobalKey` on a
  /// `RepaintBoundary` to an image handle:
  ///
  /// ```dart
  /// class ImageCaptureHome extends StatefulWidget {
  ///   const ImageCaptureHome({super.key});
  ///
  ///   @override
  ///   State<ImageCaptureHome> createState() => _ImageCaptureHomeState();
  /// }
  ///
  /// class _ImageCaptureHomeState extends State<ImageCaptureHome> {
  ///   GlobalKey globalKey = GlobalKey();
  ///
  ///   void _captureImage() {
  ///     final RenderRepaintBoundary boundary = globalKey.currentContext!.findRenderObject()! as RenderRepaintBoundary;
  ///     final ui.Image image = boundary.toImageSync();
  ///     print('Image dimensions: ${image.width}x${image.height}');
  ///   }
  ///
  ///   @override
  ///   Widget build(BuildContext context) {
  ///     return RepaintBoundary(
  ///       key: globalKey,
  ///       child: Center(
  ///         child: TextButton(
  ///           onPressed: _captureImage,
  ///           child: const Text('Hello World', textDirection: TextDirection.ltr),
  ///         ),
  ///       ),
  ///     );
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [OffsetLayer.toImageSync] for a similar API at the layer level.
  ///  * [dart:ui.Scene.toImageSync] for more information about the image returned.
  ui.Image toImageSync({double pixelRatio = 1.0}) {
    assert(!debugNeedsPaint);
    final OffsetLayer offsetLayer = layer! as OffsetLayer;
    return offsetLayer.toImageSync(Offset.zero & size, pixelRatio: pixelRatio);
  }

  /// The number of times that this render object repainted at the same time as
  /// its parent. Repaint boundaries are only useful when the parent and child
  /// paint at different times. When both paint at the same time, the repaint
  /// boundary is redundant, and may be actually making performance worse.
  ///
  /// Only valid when asserts are enabled. In release builds, always returns
  /// zero.
  ///
  /// Can be reset using [debugResetMetrics]. See [debugAsymmetricPaintCount]
  /// for the corresponding count of times where only the parent or only the
  /// child painted.
  int get debugSymmetricPaintCount => _debugSymmetricPaintCount;
  int _debugSymmetricPaintCount = 0;

  /// The number of times that either this render object repainted without the
  /// parent being painted, or the parent repainted without this object being
  /// painted. When a repaint boundary is used at a seam in the render tree
  /// where the parent tends to repaint at entirely different times than the
  /// child, it can improve performance by reducing the number of paint
  /// operations that have to be recorded each frame.
  ///
  /// Only valid when asserts are enabled. In release builds, always returns
  /// zero.
  ///
  /// Can be reset using [debugResetMetrics]. See [debugSymmetricPaintCount] for
  /// the corresponding count of times where both the parent and the child
  /// painted together.
  int get debugAsymmetricPaintCount => _debugAsymmetricPaintCount;
  int _debugAsymmetricPaintCount = 0;

  /// Resets the [debugSymmetricPaintCount] and [debugAsymmetricPaintCount]
  /// counts to zero.
  ///
  /// Only valid when asserts are enabled. Does nothing in release builds.
  void debugResetMetrics() {
    assert(() {
      _debugSymmetricPaintCount = 0;
      _debugAsymmetricPaintCount = 0;
      return true;
    }());
  }

  @override
  void debugRegisterRepaintBoundaryPaint({bool includedParent = true, bool includedChild = false}) {
    assert(() {
      if (includedParent && includedChild) {
        _debugSymmetricPaintCount += 1;
      } else {
        _debugAsymmetricPaintCount += 1;
      }
      return true;
    }());
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    bool inReleaseMode = true;
    assert(() {
      inReleaseMode = false;
      final int totalPaints = debugSymmetricPaintCount + debugAsymmetricPaintCount;
      if (totalPaints == 0) {
        properties.add(
          MessageProperty('usefulness ratio', 'no metrics collected yet (never painted)'),
        );
      } else {
        final double fraction = debugAsymmetricPaintCount / totalPaints;
        final String diagnosis = switch (fraction) {
          _ when totalPaints < 5 =>
            'insufficient data to draw conclusion (less than five repaints)',
          > 0.9 => 'this is an outstandingly useful repaint boundary and should definitely be kept',
          > 0.5 => 'this is a useful repaint boundary and should be kept',
          > 0.3 =>
            'this repaint boundary is probably useful, but maybe it would be more useful in tandem with adding more repaint boundaries elsewhere',
          > 0.1 =>
            'this repaint boundary does sometimes show value, though currently not that often',
          _ when debugAsymmetricPaintCount > 0 =>
            'this repaint boundary is not very effective and should probably be removed',
          _ => 'this repaint boundary is astoundingly ineffectual and should be removed',
        };

        properties.add(
          PercentProperty(
            'metrics',
            fraction,
            unit: 'useful',
            tooltip: '$debugSymmetricPaintCount bad vs $debugAsymmetricPaintCount good',
          ),
        );
        properties.add(MessageProperty('diagnosis', diagnosis));
      }
      return true;
    }());
    if (inReleaseMode) {
      properties.add(
        DiagnosticsNode.message('(run in debug mode to collect repaint boundary statistics)'),
      );
    }
  }
}

/// A render object that is invisible during hit testing.
///
/// When [ignoring] is true, this render object (and its subtree) is invisible
/// to hit testing. It still consumes space during layout and paints its child
/// as usual. It just cannot be the target of located events, because its render
/// object returns false from [hitTest].
///
/// ## Semantics
///
/// Using this class may also affect how the semantics subtree underneath is
/// collected.
///
/// {@macro flutter.widgets.IgnorePointer.semantics}
///
/// {@macro flutter.widgets.IgnorePointer.ignoringSemantics}
///
/// See also:
///
///  * [RenderAbsorbPointer], which takes the pointer events but prevents any
///    nodes in the subtree from seeing them.
class RenderIgnorePointer extends RenderProxyBox {
  /// Creates a render object that is invisible to hit testing.
  RenderIgnorePointer({
    RenderBox? child,
    bool ignoring = true,
    @Deprecated(
      'Use ExcludeSemantics or create a custom ignore pointer widget instead. '
      'This feature was deprecated after v3.8.0-12.0.pre.',
    )
    bool? ignoringSemantics,
  }) : _ignoring = ignoring,
       _ignoringSemantics = ignoringSemantics,
       super(child);

  /// Whether this render object is ignored during hit testing.
  ///
  /// Regardless of whether this render object is ignored during hit testing, it
  /// will still consume space during layout and be visible during painting.
  ///
  /// {@macro flutter.widgets.IgnorePointer.semantics}
  bool get ignoring => _ignoring;
  bool _ignoring;
  set ignoring(bool value) {
    if (value == _ignoring) {
      return;
    }
    _ignoring = value;
    if (ignoringSemantics == null) {
      markNeedsSemanticsUpdate();
    }
  }

  /// Whether the semantics of this render object is ignored when compiling the semantics tree.
  ///
  /// {@macro flutter.widgets.IgnorePointer.ignoringSemantics}
  ///
  /// See [SemanticsNode] for additional information about the semantics tree.
  @Deprecated(
    'Use ExcludeSemantics or create a custom ignore pointer widget instead. '
    'This feature was deprecated after v3.8.0-12.0.pre.',
  )
  bool? get ignoringSemantics => _ignoringSemantics;
  bool? _ignoringSemantics;
  set ignoringSemantics(bool? value) {
    if (value == _ignoringSemantics) {
      return;
    }
    _ignoringSemantics = value;
    markNeedsSemanticsUpdate();
  }

  @override
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    return !ignoring && super.hitTest(result, position: position);
  }

  @override
  void visitChildrenForSemantics(RenderObjectVisitor visitor) {
    if (_ignoringSemantics ?? false) {
      return;
    }
    super.visitChildrenForSemantics(visitor);
  }

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    super.describeSemanticsConfiguration(config);
    // Do not block user interactions if _ignoringSemantics is false; otherwise,
    // delegate to _ignoring
    config.isBlockingUserActions = _ignoring && (_ignoringSemantics ?? true);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<bool>('ignoring', _ignoring));
    properties.add(
      DiagnosticsProperty<bool>(
        'ignoringSemantics',
        _ignoringSemantics,
        description: _ignoringSemantics == null ? null : 'implicitly $_ignoringSemantics',
      ),
    );
  }
}

/// Lays the child out as if it was in the tree, but without painting anything,
/// without making the child available for hit testing, and without taking any
/// room in the parent.
class RenderOffstage extends RenderProxyBox {
  /// Creates an offstage render object.
  RenderOffstage({bool offstage = true, RenderBox? child}) : _offstage = offstage, super(child);

  /// Whether the child is hidden from the rest of the tree.
  ///
  /// If true, the child is laid out as if it was in the tree, but without
  /// painting anything, without making the child available for hit testing, and
  /// without taking any room in the parent.
  ///
  /// If false, the child is included in the tree as normal.
  bool get offstage => _offstage;
  bool _offstage;
  set offstage(bool value) {
    if (value == _offstage) {
      return;
    }
    _offstage = value;
    markNeedsLayoutForSizedByParentChange();
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    if (offstage) {
      return 0.0;
    }
    return super.computeMinIntrinsicWidth(height);
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    if (offstage) {
      return 0.0;
    }
    return super.computeMaxIntrinsicWidth(height);
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    if (offstage) {
      return 0.0;
    }
    return super.computeMinIntrinsicHeight(width);
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    if (offstage) {
      return 0.0;
    }
    return super.computeMaxIntrinsicHeight(width);
  }

  @override
  double? computeDistanceToActualBaseline(TextBaseline baseline) {
    if (offstage) {
      return null;
    }
    return super.computeDistanceToActualBaseline(baseline);
  }

  @override
  bool get sizedByParent => offstage;

  @override
  double? computeDryBaseline(BoxConstraints constraints, TextBaseline baseline) {
    return offstage ? null : super.computeDryBaseline(constraints, baseline);
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    if (offstage) {
      return constraints.smallest;
    }
    return super.computeDryLayout(constraints);
  }

  @override
  void performResize() {
    assert(offstage);
    super.performResize();
  }

  @override
  void performLayout() {
    if (offstage) {
      child?.layout(constraints);
    } else {
      super.performLayout();
    }
  }

  @override
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    return !offstage && super.hitTest(result, position: position);
  }

  @override
  bool paintsChild(RenderBox child) {
    assert(child.parent == this);
    return !offstage;
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (offstage) {
      return;
    }
    super.paint(context, offset);
  }

  @override
  void visitChildrenForSemantics(RenderObjectVisitor visitor) {
    if (offstage) {
      return;
    }
    super.visitChildrenForSemantics(visitor);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<bool>('offstage', offstage));
  }

  @override
  List<DiagnosticsNode> debugDescribeChildren() {
    if (child == null) {
      return <DiagnosticsNode>[];
    }
    return <DiagnosticsNode>[
      child!.toDiagnosticsNode(
        name: 'child',
        style: offstage ? DiagnosticsTreeStyle.offstage : DiagnosticsTreeStyle.sparse,
      ),
    ];
  }
}

/// A render object that absorbs pointers during hit testing.
///
/// When [absorbing] is true, this render object prevents its subtree from
/// receiving pointer events by terminating hit testing at itself. It still
/// consumes space during layout and paints its child as usual. It just prevents
/// its children from being the target of located events, because its render
/// object returns true from [hitTest].
///
/// ## Semantics
///
/// Using this class may also affect how the semantics subtree underneath is
/// collected.
///
/// {@macro flutter.widgets.AbsorbPointer.semantics}
///
/// {@macro flutter.widgets.AbsorbPointer.ignoringSemantics}
///
/// See also:
///
///  * [RenderIgnorePointer], which has the opposite effect: removing the
///    subtree from considering entirely for the purposes of hit testing.
class RenderAbsorbPointer extends RenderProxyBox {
  /// Creates a render object that absorbs pointers during hit testing.
  RenderAbsorbPointer({
    RenderBox? child,
    bool absorbing = true,
    @Deprecated(
      'Use ExcludeSemantics or create a custom absorb pointer widget instead. '
      'This feature was deprecated after v3.8.0-12.0.pre.',
    )
    bool? ignoringSemantics,
  }) : _absorbing = absorbing,
       _ignoringSemantics = ignoringSemantics,
       super(child);

  /// Whether this render object absorbs pointers during hit testing.
  ///
  /// Regardless of whether this render object absorbs pointers during hit
  /// testing, it will still consume space during layout and be visible during
  /// painting.
  ///
  /// {@macro flutter.widgets.AbsorbPointer.semantics}
  bool get absorbing => _absorbing;
  bool _absorbing;
  set absorbing(bool value) {
    if (_absorbing == value) {
      return;
    }
    _absorbing = value;
    if (ignoringSemantics == null) {
      markNeedsSemanticsUpdate();
    }
  }

  /// Whether the semantics of this render object is ignored when compiling the
  /// semantics tree.
  ///
  /// {@macro flutter.widgets.AbsorbPointer.ignoringSemantics}
  ///
  /// See [SemanticsNode] for additional information about the semantics tree.
  @Deprecated(
    'Use ExcludeSemantics or create a custom absorb pointer widget instead. '
    'This feature was deprecated after v3.8.0-12.0.pre.',
  )
  bool? get ignoringSemantics => _ignoringSemantics;
  bool? _ignoringSemantics;
  set ignoringSemantics(bool? value) {
    if (value == _ignoringSemantics) {
      return;
    }
    _ignoringSemantics = value;
    markNeedsSemanticsUpdate();
  }

  @override
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    return absorbing ? size.contains(position) : super.hitTest(result, position: position);
  }

  @override
  void visitChildrenForSemantics(RenderObjectVisitor visitor) {
    if (_ignoringSemantics ?? false) {
      return;
    }
    super.visitChildrenForSemantics(visitor);
  }

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    super.describeSemanticsConfiguration(config);
    // Do not block user interactions if _ignoringSemantics is false; otherwise,
    // delegate to absorbing
    config.isBlockingUserActions = absorbing && (_ignoringSemantics ?? true);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<bool>('absorbing', absorbing));
    properties.add(
      DiagnosticsProperty<bool>(
        'ignoringSemantics',
        ignoringSemantics,
        description: ignoringSemantics == null ? null : 'implicitly $ignoringSemantics',
      ),
    );
  }
}

/// Holds opaque meta data in the render tree.
///
/// Useful for decorating the render tree with information that will be consumed
/// later. For example, you could store information in the render tree that will
/// be used when the user interacts with the render tree but has no visual
/// impact prior to the interaction.
class RenderMetaData extends RenderProxyBoxWithHitTestBehavior {
  /// Creates a render object that hold opaque meta data.
  ///
  /// The [behavior] argument defaults to [HitTestBehavior.deferToChild].
  RenderMetaData({this.metaData, super.behavior, super.child});

  /// Opaque meta data ignored by the render tree.
  dynamic metaData;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<dynamic>('metaData', metaData));
  }
}

/// Listens for the specified gestures from the semantics server (e.g.
/// an accessibility tool).
class RenderSemanticsGestureHandler extends RenderProxyBoxWithHitTestBehavior {
  /// Creates a render object that listens for specific semantic gestures.
  RenderSemanticsGestureHandler({
    super.child,
    GestureTapCallback? onTap,
    GestureLongPressCallback? onLongPress,
    GestureDragUpdateCallback? onHorizontalDragUpdate,
    GestureDragUpdateCallback? onVerticalDragUpdate,
    this.scrollFactor = 0.8,
    super.behavior,
  }) : _onTap = onTap,
       _onLongPress = onLongPress,
       _onHorizontalDragUpdate = onHorizontalDragUpdate,
       _onVerticalDragUpdate = onVerticalDragUpdate;

  /// If non-null, the set of actions to allow. Other actions will be omitted,
  /// even if their callback is provided.
  ///
  /// For example, if [onTap] is non-null but [validActions] does not contain
  /// [SemanticsAction.tap], then the semantic description of this node will
  /// not claim to support taps.
  ///
  /// This is normally used to filter the actions made available by
  /// [onHorizontalDragUpdate] and [onVerticalDragUpdate]. Normally, these make
  /// both the right and left, or up and down, actions available. For example,
  /// if [onHorizontalDragUpdate] is set but [validActions] only contains
  /// [SemanticsAction.scrollLeft], then the [SemanticsAction.scrollRight]
  /// action will be omitted.
  Set<SemanticsAction>? get validActions => _validActions;
  Set<SemanticsAction>? _validActions;
  set validActions(Set<SemanticsAction>? value) {
    if (setEquals<SemanticsAction>(value, _validActions)) {
      return;
    }
    _validActions = value;
    markNeedsSemanticsUpdate();
  }

  /// Called when the user taps on the render object.
  GestureTapCallback? get onTap => _onTap;
  GestureTapCallback? _onTap;
  set onTap(GestureTapCallback? value) {
    if (_onTap == value) {
      return;
    }
    final bool hadHandler = _onTap != null;
    _onTap = value;
    if ((value != null) != hadHandler) {
      markNeedsSemanticsUpdate();
    }
  }

  /// Called when the user presses on the render object for a long period of time.
  GestureLongPressCallback? get onLongPress => _onLongPress;
  GestureLongPressCallback? _onLongPress;
  set onLongPress(GestureLongPressCallback? value) {
    if (_onLongPress == value) {
      return;
    }
    final bool hadHandler = _onLongPress != null;
    _onLongPress = value;
    if ((value != null) != hadHandler) {
      markNeedsSemanticsUpdate();
    }
  }

  /// Called when the user scrolls to the left or to the right.
  GestureDragUpdateCallback? get onHorizontalDragUpdate => _onHorizontalDragUpdate;
  GestureDragUpdateCallback? _onHorizontalDragUpdate;
  set onHorizontalDragUpdate(GestureDragUpdateCallback? value) {
    if (_onHorizontalDragUpdate == value) {
      return;
    }
    final bool hadHandler = _onHorizontalDragUpdate != null;
    _onHorizontalDragUpdate = value;
    if ((value != null) != hadHandler) {
      markNeedsSemanticsUpdate();
    }
  }

  /// Called when the user scrolls up or down.
  GestureDragUpdateCallback? get onVerticalDragUpdate => _onVerticalDragUpdate;
  GestureDragUpdateCallback? _onVerticalDragUpdate;
  set onVerticalDragUpdate(GestureDragUpdateCallback? value) {
    if (_onVerticalDragUpdate == value) {
      return;
    }
    final bool hadHandler = _onVerticalDragUpdate != null;
    _onVerticalDragUpdate = value;
    if ((value != null) != hadHandler) {
      markNeedsSemanticsUpdate();
    }
  }

  /// The fraction of the dimension of this render box to use when
  /// scrolling. For example, if this is 0.8 and the box is 200 pixels
  /// wide, then when a left-scroll action is received from the
  /// accessibility system, it will translate into a 160 pixel
  /// leftwards drag.
  double scrollFactor;

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    super.describeSemanticsConfiguration(config);

    if (onTap != null && _isValidAction(SemanticsAction.tap)) {
      config.onTap = onTap;
    }
    if (onLongPress != null && _isValidAction(SemanticsAction.longPress)) {
      config.onLongPress = onLongPress;
    }
    if (onHorizontalDragUpdate != null) {
      if (_isValidAction(SemanticsAction.scrollRight)) {
        config.onScrollRight = _performSemanticScrollRight;
      }
      if (_isValidAction(SemanticsAction.scrollLeft)) {
        config.onScrollLeft = _performSemanticScrollLeft;
      }
    }
    if (onVerticalDragUpdate != null) {
      if (_isValidAction(SemanticsAction.scrollUp)) {
        config.onScrollUp = _performSemanticScrollUp;
      }
      if (_isValidAction(SemanticsAction.scrollDown)) {
        config.onScrollDown = _performSemanticScrollDown;
      }
    }
  }

  bool _isValidAction(SemanticsAction action) {
    return validActions == null || validActions!.contains(action);
  }

  void _performSemanticScrollLeft() {
    if (onHorizontalDragUpdate != null) {
      final double primaryDelta = size.width * -scrollFactor;
      onHorizontalDragUpdate!(
        DragUpdateDetails(
          delta: Offset(primaryDelta, 0.0),
          primaryDelta: primaryDelta,
          globalPosition: localToGlobal(size.center(Offset.zero)),
        ),
      );
    }
  }

  void _performSemanticScrollRight() {
    if (onHorizontalDragUpdate != null) {
      final double primaryDelta = size.width * scrollFactor;
      onHorizontalDragUpdate!(
        DragUpdateDetails(
          delta: Offset(primaryDelta, 0.0),
          primaryDelta: primaryDelta,
          globalPosition: localToGlobal(size.center(Offset.zero)),
        ),
      );
    }
  }

  void _performSemanticScrollUp() {
    if (onVerticalDragUpdate != null) {
      final double primaryDelta = size.height * -scrollFactor;
      onVerticalDragUpdate!(
        DragUpdateDetails(
          delta: Offset(0.0, primaryDelta),
          primaryDelta: primaryDelta,
          globalPosition: localToGlobal(size.center(Offset.zero)),
        ),
      );
    }
  }

  void _performSemanticScrollDown() {
    if (onVerticalDragUpdate != null) {
      final double primaryDelta = size.height * scrollFactor;
      onVerticalDragUpdate!(
        DragUpdateDetails(
          delta: Offset(0.0, primaryDelta),
          primaryDelta: primaryDelta,
          globalPosition: localToGlobal(size.center(Offset.zero)),
        ),
      );
    }
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    final List<String> gestures = <String>[
      if (onTap != null) 'tap',
      if (onLongPress != null) 'long press',
      if (onHorizontalDragUpdate != null) 'horizontal scroll',
      if (onVerticalDragUpdate != null) 'vertical scroll',
    ];
    if (gestures.isEmpty) {
      gestures.add('<none>');
    }
    properties.add(IterableProperty<String>('gestures', gestures));
  }
}

/// Add annotations to the [SemanticsNode] for this subtree.
class RenderSemanticsAnnotations extends RenderProxyBox {
  /// Creates a render object that attaches a semantic annotation.
  ///
  /// If the [SemanticsProperties.attributedLabel] is not null, the [textDirection] must also not be null.
  RenderSemanticsAnnotations({
    RenderBox? child,
    required SemanticsProperties properties,
    bool container = false,
    bool explicitChildNodes = false,
    bool excludeSemantics = false,
    bool blockUserActions = false,
    TextDirection? textDirection,
  }) : _container = container,
       _explicitChildNodes = explicitChildNodes,
       _excludeSemantics = excludeSemantics,
       _blockUserActions = blockUserActions,
       _textDirection = textDirection,
       _properties = properties,
       super(child) {
    _updateAttributedFields(_properties);
  }

  /// All of the [SemanticsProperties] for this [RenderSemanticsAnnotations].
  SemanticsProperties get properties => _properties;
  SemanticsProperties _properties;
  set properties(SemanticsProperties value) {
    if (_properties == value) {
      return;
    }
    _properties = value;
    _updateAttributedFields(_properties);
    markNeedsSemanticsUpdate();
  }

  /// If 'container' is true, this [RenderObject] will introduce a new
  /// node in the semantics tree. Otherwise, the semantics will be
  /// merged with the semantics of any ancestors.
  ///
  /// Whether descendants of this [RenderObject] can add their semantic information
  /// to the [SemanticsNode] introduced by this configuration is controlled by
  /// [explicitChildNodes].
  bool get container => _container;
  bool _container;
  set container(bool value) {
    if (container == value) {
      return;
    }
    _container = value;
    markNeedsSemanticsUpdate();
  }

  /// Whether descendants of this [RenderObject] are allowed to add semantic
  /// information to the [SemanticsNode] annotated by this widget.
  ///
  /// When set to false descendants are allowed to annotate [SemanticsNode]s of
  /// their parent with the semantic information they want to contribute to the
  /// semantic tree.
  /// When set to true the only way for descendants to contribute semantic
  /// information to the semantic tree is to introduce new explicit
  /// [SemanticsNode]s to the tree.
  ///
  /// This setting is often used in combination with
  /// [SemanticsConfiguration.isSemanticBoundary] to create semantic boundaries
  /// that are either writable or not for children.
  bool get explicitChildNodes => _explicitChildNodes;
  bool _explicitChildNodes;
  set explicitChildNodes(bool value) {
    if (_explicitChildNodes == value) {
      return;
    }
    _explicitChildNodes = value;
    markNeedsSemanticsUpdate();
  }

  /// Whether descendants of this [RenderObject] should have their semantic
  /// information ignored.
  ///
  /// When this flag is set to true, all child semantics nodes are ignored.
  /// This can be used as a convenience for cases where a child is wrapped in
  /// an [ExcludeSemantics] widget and then another [Semantics] widget.
  bool get excludeSemantics => _excludeSemantics;
  bool _excludeSemantics;
  set excludeSemantics(bool value) {
    if (_excludeSemantics == value) {
      return;
    }
    _excludeSemantics = value;
    markNeedsSemanticsUpdate();
  }

  /// Whether to block user interactions for the semantics subtree.
  ///
  /// Setting this true prevents user from activating pointer related
  /// [SemanticsAction]s, such as [SemanticsAction.tap] or
  /// [SemanticsAction.longPress].
  bool get blockUserActions => _blockUserActions;
  bool _blockUserActions;
  set blockUserActions(bool value) {
    if (_blockUserActions == value) {
      return;
    }
    _blockUserActions = value;
    markNeedsSemanticsUpdate();
  }

  void _updateAttributedFields(SemanticsProperties value) {
    _attributedLabel = _effectiveAttributedLabel(value);
    _attributedValue = _effectiveAttributedValue(value);
    _attributedIncreasedValue = _effectiveAttributedIncreasedValue(value);
    _attributedDecreasedValue = _effectiveAttributedDecreasedValue(value);
    _attributedHint = _effectiveAttributedHint(value);
  }

  AttributedString? _effectiveAttributedLabel(SemanticsProperties value) {
    return value.attributedLabel ?? (value.label == null ? null : AttributedString(value.label!));
  }

  AttributedString? _effectiveAttributedValue(SemanticsProperties value) {
    return value.attributedValue ?? (value.value == null ? null : AttributedString(value.value!));
  }

  AttributedString? _effectiveAttributedIncreasedValue(SemanticsProperties value) {
    return value.attributedIncreasedValue ??
        (value.increasedValue == null ? null : AttributedString(value.increasedValue!));
  }

  AttributedString? _effectiveAttributedDecreasedValue(SemanticsProperties value) {
    return properties.attributedDecreasedValue ??
        (value.decreasedValue == null ? null : AttributedString(value.decreasedValue!));
  }

  AttributedString? _effectiveAttributedHint(SemanticsProperties value) {
    return value.attributedHint ?? (value.hint == null ? null : AttributedString(value.hint!));
  }

  AttributedString? _attributedLabel;
  AttributedString? _attributedValue;
  AttributedString? _attributedIncreasedValue;
  AttributedString? _attributedDecreasedValue;
  AttributedString? _attributedHint;

  /// If non-null, sets the [SemanticsNode.textDirection] semantic to the given
  /// value.
  ///
  /// This must not be null if [SemanticsProperties.attributedLabel],
  /// [SemanticsProperties.attributedHint],
  /// [SemanticsProperties.attributedValue],
  /// [SemanticsProperties.attributedIncreasedValue], or
  /// [SemanticsProperties.attributedDecreasedValue] are not null.
  TextDirection? get textDirection => _textDirection;
  TextDirection? _textDirection;
  set textDirection(TextDirection? value) {
    if (textDirection == value) {
      return;
    }
    _textDirection = value;
    markNeedsSemanticsUpdate();
  }

  @override
  void visitChildrenForSemantics(RenderObjectVisitor visitor) {
    if (excludeSemantics) {
      return;
    }
    super.visitChildrenForSemantics(visitor);
  }

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    super.describeSemanticsConfiguration(config);
    config.isSemanticBoundary = container;
    config.explicitChildNodes = explicitChildNodes;
    config.isBlockingUserActions = blockUserActions;
    assert(
      ((_properties.scopesRoute ?? false) && explicitChildNodes) ||
          !(_properties.scopesRoute ?? false),
      'explicitChildNodes must be set to true if scopes route is true',
    );
    assert(
      !((_properties.toggled ?? false) && (_properties.checked ?? false)),
      'A semantics node cannot be toggled and checked at the same time',
    );

    if (_properties.enabled != null) {
      config.isEnabled = _properties.enabled;
    }
    if (_properties.checked != null) {
      config.isChecked = _properties.checked;
    }
    if (_properties.mixed != null) {
      config.isCheckStateMixed = _properties.mixed;
    }
    if (_properties.toggled != null) {
      config.isToggled = _properties.toggled;
    }
    if (_properties.selected != null) {
      config.isSelected = _properties.selected!;
    }
    if (_properties.button != null) {
      config.isButton = _properties.button!;
    }
    if (_properties.expanded != null) {
      config.isExpanded = _properties.expanded;
    }
    if (_properties.link != null) {
      config.isLink = _properties.link!;
    }
    if (_properties.linkUrl != null) {
      config.linkUrl = _properties.linkUrl;
    }
    if (_properties.slider != null) {
      config.isSlider = _properties.slider!;
    }
    if (_properties.keyboardKey != null) {
      config.isKeyboardKey = _properties.keyboardKey!;
    }
    if (_properties.header != null) {
      config.isHeader = _properties.header!;
    }
    if (_properties.headingLevel != null) {
      config.headingLevel = _properties.headingLevel!;
    }
    if (_properties.textField != null) {
      config.isTextField = _properties.textField!;
    }
    if (_properties.readOnly != null) {
      config.isReadOnly = _properties.readOnly!;
    }
    if (_properties.focusable != null) {
      config.isFocusable = _properties.focusable!;
    }
    if (_properties.focused != null) {
      config.isFocused = _properties.focused!;
    }
    if (_properties.inMutuallyExclusiveGroup != null) {
      config.isInMutuallyExclusiveGroup = _properties.inMutuallyExclusiveGroup!;
    }
    if (_properties.obscured != null) {
      config.isObscured = _properties.obscured!;
    }
    if (_properties.multiline != null) {
      config.isMultiline = _properties.multiline!;
    }
    if (_properties.hidden != null) {
      config.isHidden = _properties.hidden!;
    }
    if (_properties.image != null) {
      config.isImage = _properties.image!;
    }
    if (_properties.identifier != null) {
      config.identifier = _properties.identifier!;
    }
    if (_attributedLabel != null) {
      config.attributedLabel = _attributedLabel!;
    }
    if (_attributedValue != null) {
      config.attributedValue = _attributedValue!;
    }
    if (_attributedIncreasedValue != null) {
      config.attributedIncreasedValue = _attributedIncreasedValue!;
    }
    if (_attributedDecreasedValue != null) {
      config.attributedDecreasedValue = _attributedDecreasedValue!;
    }
    if (_attributedHint != null) {
      config.attributedHint = _attributedHint!;
    }
    if (_properties.tooltip != null) {
      config.tooltip = _properties.tooltip!;
    }
    if (_properties.hintOverrides != null && _properties.hintOverrides!.isNotEmpty) {
      config.hintOverrides = _properties.hintOverrides;
    }
    if (_properties.scopesRoute != null) {
      config.scopesRoute = _properties.scopesRoute!;
    }
    if (_properties.namesRoute != null) {
      config.namesRoute = _properties.namesRoute!;
    }
    if (_properties.liveRegion != null) {
      config.liveRegion = _properties.liveRegion!;
    }
    if (_properties.maxValueLength != null) {
      config.maxValueLength = _properties.maxValueLength;
    }
    if (_properties.currentValueLength != null) {
      config.currentValueLength = _properties.currentValueLength;
    }
    if (textDirection != null) {
      config.textDirection = textDirection;
    }
    if (_properties.sortKey != null) {
      config.sortKey = _properties.sortKey;
    }
    if (_properties.tagForChildren != null) {
      config.addTagForChildren(_properties.tagForChildren!);
    }
    // Registering _perform* as action handlers instead of the user provided
    // ones to ensure that changing a user provided handler from a non-null to
    // another non-null value doesn't require a semantics update.
    if (_properties.onTap != null) {
      config.onTap = _performTap;
    }
    if (_properties.onLongPress != null) {
      config.onLongPress = _performLongPress;
    }
    if (_properties.onDismiss != null) {
      config.onDismiss = _performDismiss;
    }
    if (_properties.onScrollLeft != null) {
      config.onScrollLeft = _performScrollLeft;
    }
    if (_properties.onScrollRight != null) {
      config.onScrollRight = _performScrollRight;
    }
    if (_properties.onScrollUp != null) {
      config.onScrollUp = _performScrollUp;
    }
    if (_properties.onScrollDown != null) {
      config.onScrollDown = _performScrollDown;
    }
    if (_properties.onIncrease != null) {
      config.onIncrease = _performIncrease;
    }
    if (_properties.onDecrease != null) {
      config.onDecrease = _performDecrease;
    }
    if (_properties.onCopy != null) {
      config.onCopy = _performCopy;
    }
    if (_properties.onCut != null) {
      config.onCut = _performCut;
    }
    if (_properties.onPaste != null) {
      config.onPaste = _performPaste;
    }
    if (_properties.onMoveCursorForwardByCharacter != null) {
      config.onMoveCursorForwardByCharacter = _performMoveCursorForwardByCharacter;
    }
    if (_properties.onMoveCursorBackwardByCharacter != null) {
      config.onMoveCursorBackwardByCharacter = _performMoveCursorBackwardByCharacter;
    }
    if (_properties.onMoveCursorForwardByWord != null) {
      config.onMoveCursorForwardByWord = _performMoveCursorForwardByWord;
    }
    if (_properties.onMoveCursorBackwardByWord != null) {
      config.onMoveCursorBackwardByWord = _performMoveCursorBackwardByWord;
    }
    if (_properties.onSetSelection != null) {
      config.onSetSelection = _performSetSelection;
    }
    if (_properties.onSetText != null) {
      config.onSetText = _performSetText;
    }
    if (_properties.onDidGainAccessibilityFocus != null) {
      config.onDidGainAccessibilityFocus = _performDidGainAccessibilityFocus;
    }
    if (_properties.onDidLoseAccessibilityFocus != null) {
      config.onDidLoseAccessibilityFocus = _performDidLoseAccessibilityFocus;
    }
    if (_properties.onFocus != null) {
      config.onFocus = _performFocus;
    }
    if (_properties.customSemanticsActions != null) {
      config.customSemanticsActions = _properties.customSemanticsActions!;
    }
  }

  void _performTap() {
    _properties.onTap?.call();
  }

  void _performLongPress() {
    _properties.onLongPress?.call();
  }

  void _performDismiss() {
    _properties.onDismiss?.call();
  }

  void _performScrollLeft() {
    _properties.onScrollLeft?.call();
  }

  void _performScrollRight() {
    _properties.onScrollRight?.call();
  }

  void _performScrollUp() {
    _properties.onScrollUp?.call();
  }

  void _performScrollDown() {
    _properties.onScrollDown?.call();
  }

  void _performIncrease() {
    _properties.onIncrease?.call();
  }

  void _performDecrease() {
    _properties.onDecrease?.call();
  }

  void _performCopy() {
    _properties.onCopy?.call();
  }

  void _performCut() {
    _properties.onCut?.call();
  }

  void _performPaste() {
    _properties.onPaste?.call();
  }

  void _performMoveCursorForwardByCharacter(bool extendSelection) {
    _properties.onMoveCursorForwardByCharacter?.call(extendSelection);
  }

  void _performMoveCursorBackwardByCharacter(bool extendSelection) {
    _properties.onMoveCursorBackwardByCharacter?.call(extendSelection);
  }

  void _performMoveCursorForwardByWord(bool extendSelection) {
    _properties.onMoveCursorForwardByWord?.call(extendSelection);
  }

  void _performMoveCursorBackwardByWord(bool extendSelection) {
    _properties.onMoveCursorBackwardByWord?.call(extendSelection);
  }

  void _performSetSelection(TextSelection selection) {
    _properties.onSetSelection?.call(selection);
  }

  void _performSetText(String text) {
    _properties.onSetText?.call(text);
  }

  void _performDidGainAccessibilityFocus() {
    _properties.onDidGainAccessibilityFocus?.call();
  }

  void _performDidLoseAccessibilityFocus() {
    _properties.onDidLoseAccessibilityFocus?.call();
  }

  void _performFocus() {
    _properties.onFocus?.call();
  }
}

/// Causes the semantics of all earlier render objects below the same semantic
/// boundary to be dropped.
///
/// This is useful in a stack where an opaque mask should prevent interactions
/// with the render objects painted below the mask.
class RenderBlockSemantics extends RenderProxyBox {
  /// Create a render object that blocks semantics for nodes below it in paint
  /// order.
  RenderBlockSemantics({RenderBox? child, bool blocking = true})
    : _blocking = blocking,
      super(child);

  /// Whether this render object is blocking semantics of previously painted
  /// [RenderObject]s below a common semantics boundary from the semantic tree.
  bool get blocking => _blocking;
  bool _blocking;
  set blocking(bool value) {
    if (value == _blocking) {
      return;
    }
    _blocking = value;
    markNeedsSemanticsUpdate();
  }

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    super.describeSemanticsConfiguration(config);
    config.isBlockingSemanticsOfPreviouslyPaintedNodes = blocking;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<bool>('blocking', blocking));
  }
}

/// Causes the semantics of all descendants to be merged into this
/// node such that the entire subtree becomes a single leaf in the
/// semantics tree.
///
/// Useful for combining the semantics of multiple render objects that
/// form part of a single conceptual widget, e.g. a checkbox, a label,
/// and the gesture detector that goes with them.
class RenderMergeSemantics extends RenderProxyBox {
  /// Creates a render object that merges the semantics from its descendants.
  RenderMergeSemantics({RenderBox? child}) : super(child);

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    super.describeSemanticsConfiguration(config);
    config
      ..isSemanticBoundary = true
      ..isMergingSemanticsOfDescendants = true;
  }
}

/// Excludes this subtree from the semantic tree.
///
/// When [excluding] is true, this render object (and its subtree) is excluded
/// from the semantic tree.
///
/// Useful e.g. for hiding text that is redundant with other text next
/// to it (e.g. text included only for the visual effect).
class RenderExcludeSemantics extends RenderProxyBox {
  /// Creates a render object that ignores the semantics of its subtree.
  RenderExcludeSemantics({RenderBox? child, bool excluding = true})
    : _excluding = excluding,
      super(child);

  /// Whether this render object is excluded from the semantic tree.
  bool get excluding => _excluding;
  bool _excluding;
  set excluding(bool value) {
    if (value == _excluding) {
      return;
    }
    _excluding = value;
    markNeedsSemanticsUpdate();
  }

  @override
  void visitChildrenForSemantics(RenderObjectVisitor visitor) {
    if (excluding) {
      return;
    }
    super.visitChildrenForSemantics(visitor);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<bool>('excluding', excluding));
  }
}

/// A render objects that annotates semantics with an index.
///
/// Certain widgets will automatically provide a child index for building
/// semantics. For example, the [ScrollView] uses the index of the first
/// visible child semantics node to determine the
/// [SemanticsConfiguration.scrollIndex].
///
/// See also:
///
///  * [CustomScrollView], for an explanation of scroll semantics.
class RenderIndexedSemantics extends RenderProxyBox {
  /// Creates a render object that annotates the child semantics with an index.
  RenderIndexedSemantics({RenderBox? child, required int index}) : _index = index, super(child);

  /// The index used to annotated child semantics.
  int get index => _index;
  int _index;
  set index(int value) {
    if (value == index) {
      return;
    }
    _index = value;
    markNeedsSemanticsUpdate();
  }

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    super.describeSemanticsConfiguration(config);
    config.indexInParent = index;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<int>('index', index));
  }
}

/// Provides an anchor for a [RenderFollowerLayer].
///
/// See also:
///
///  * [CompositedTransformTarget], the corresponding widget.
///  * [LeaderLayer], the layer that this render object creates.
class RenderLeaderLayer extends RenderProxyBox {
  /// Creates a render object that uses a [LeaderLayer].
  RenderLeaderLayer({required LayerLink link, RenderBox? child}) : _link = link, super(child);

  /// The link object that connects this [RenderLeaderLayer] with one or more
  /// [RenderFollowerLayer]s.
  ///
  /// The object must not be associated with another [RenderLeaderLayer] that is
  /// also being painted.
  LayerLink get link => _link;
  LayerLink _link;
  set link(LayerLink value) {
    if (_link == value) {
      return;
    }
    _link.leaderSize = null;
    _link = value;
    if (_previousLayoutSize != null) {
      _link.leaderSize = _previousLayoutSize;
    }
    markNeedsPaint();
  }

  @override
  bool get alwaysNeedsCompositing => true;

  // The latest size of this [RenderBox], computed during the previous layout
  // pass. It should always be equal to [size], but can be accessed even when
  // [debugDoingThisResize] and [debugDoingThisLayout] are false.
  Size? _previousLayoutSize;

  @override
  void performLayout() {
    super.performLayout();
    _previousLayoutSize = size;
    link.leaderSize = size;
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (layer == null) {
      layer = LeaderLayer(link: link, offset: offset);
    } else {
      final LeaderLayer leaderLayer = layer! as LeaderLayer;
      leaderLayer
        ..link = link
        ..offset = offset;
    }
    context.pushLayer(layer!, super.paint, Offset.zero);
    assert(() {
      layer!.debugCreator = debugCreator;
      return true;
    }());
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<LayerLink>('link', link));
  }
}

/// Transform the child so that its origin is [offset] from the origin of the
/// [RenderLeaderLayer] with the same [LayerLink].
///
/// The [RenderLeaderLayer] in question must be earlier in the paint order.
///
/// Hit testing on descendants of this render object will only work if the
/// target position is within the box that this render object's parent considers
/// to be hittable.
///
/// See also:
///
///  * [CompositedTransformFollower], the corresponding widget.
///  * [FollowerLayer], the layer that this render object creates.
class RenderFollowerLayer extends RenderProxyBox {
  /// Creates a render object that uses a [FollowerLayer].
  RenderFollowerLayer({
    required LayerLink link,
    bool showWhenUnlinked = true,
    Offset offset = Offset.zero,
    Alignment leaderAnchor = Alignment.topLeft,
    Alignment followerAnchor = Alignment.topLeft,
    RenderBox? child,
  }) : _link = link,
       _showWhenUnlinked = showWhenUnlinked,
       _offset = offset,
       _leaderAnchor = leaderAnchor,
       _followerAnchor = followerAnchor,
       super(child);

  /// The link object that connects this [RenderFollowerLayer] with a
  /// [RenderLeaderLayer] earlier in the paint order.
  LayerLink get link => _link;
  LayerLink _link;
  set link(LayerLink value) {
    if (_link == value) {
      return;
    }
    _link = value;
    markNeedsPaint();
  }

  /// Whether to show the render object's contents when there is no
  /// corresponding [RenderLeaderLayer] with the same [link].
  ///
  /// When the render object is linked, the child is positioned such that it has
  /// the same global position as the linked [RenderLeaderLayer].
  ///
  /// When the render object is not linked, then: if [showWhenUnlinked] is true,
  /// the child is visible and not repositioned; if it is false, then child is
  /// hidden, and its hit testing is also disabled.
  bool get showWhenUnlinked => _showWhenUnlinked;
  bool _showWhenUnlinked;
  set showWhenUnlinked(bool value) {
    if (_showWhenUnlinked == value) {
      return;
    }
    _showWhenUnlinked = value;
    markNeedsPaint();
  }

  /// The offset to apply to the origin of the linked [RenderLeaderLayer] to
  /// obtain this render object's origin.
  Offset get offset => _offset;
  Offset _offset;
  set offset(Offset value) {
    if (_offset == value) {
      return;
    }
    _offset = value;
    markNeedsPaint();
  }

  /// The anchor point on the linked [RenderLeaderLayer] that [followerAnchor]
  /// will line up with.
  ///
  /// {@template flutter.rendering.RenderFollowerLayer.leaderAnchor}
  /// For example, when [leaderAnchor] and [followerAnchor] are both
  /// [Alignment.topLeft], this [RenderFollowerLayer] will be top left aligned
  /// with the linked [RenderLeaderLayer]. When [leaderAnchor] is
  /// [Alignment.bottomLeft] and [followerAnchor] is [Alignment.topLeft], this
  /// [RenderFollowerLayer] will be left aligned with the linked
  /// [RenderLeaderLayer], and its top edge will line up with the
  /// [RenderLeaderLayer]'s bottom edge.
  /// {@endtemplate}
  ///
  /// Defaults to [Alignment.topLeft].
  Alignment get leaderAnchor => _leaderAnchor;
  Alignment _leaderAnchor;
  set leaderAnchor(Alignment value) {
    if (_leaderAnchor == value) {
      return;
    }
    _leaderAnchor = value;
    markNeedsPaint();
  }

  /// The anchor point on this [RenderFollowerLayer] that will line up with
  /// [followerAnchor] on the linked [RenderLeaderLayer].
  ///
  /// {@macro flutter.rendering.RenderFollowerLayer.leaderAnchor}
  ///
  /// Defaults to [Alignment.topLeft].
  Alignment get followerAnchor => _followerAnchor;
  Alignment _followerAnchor;
  set followerAnchor(Alignment value) {
    if (_followerAnchor == value) {
      return;
    }
    _followerAnchor = value;
    markNeedsPaint();
  }

  @override
  void detach() {
    layer = null;
    super.detach();
  }

  @override
  bool get alwaysNeedsCompositing => true;

  /// The layer we created when we were last painted.
  @override
  FollowerLayer? get layer => super.layer as FollowerLayer?;

  /// Return the transform that was used in the last composition phase, if any.
  ///
  /// If the [FollowerLayer] has not yet been created, was never composited, or
  /// was unable to determine the transform (see
  /// [FollowerLayer.getLastTransform]), this returns the identity matrix (see
  /// [Matrix4.identity].
  Matrix4 getCurrentTransform() {
    return layer?.getLastTransform() ?? Matrix4.identity();
  }

  @override
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    // Disables the hit testing if this render object is hidden.
    if (link.leader == null && !showWhenUnlinked) {
      return false;
    }
    // RenderFollowerLayer objects don't check if they are
    // themselves hit, because it's confusing to think about
    // how the untransformed size and the child's transformed
    // position interact.
    return hitTestChildren(result, position: position);
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    return result.addWithPaintTransform(
      transform: getCurrentTransform(),
      position: position,
      hitTest: (BoxHitTestResult result, Offset position) {
        return super.hitTestChildren(result, position: position);
      },
    );
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    final Size? leaderSize = link.leaderSize;
    assert(
      link.leaderSize != null || link.leader == null || leaderAnchor == Alignment.topLeft,
      '$link: layer is linked to ${link.leader} but a valid leaderSize is not set. '
      'leaderSize is required when leaderAnchor is not Alignment.topLeft '
      '(current value is $leaderAnchor).',
    );
    final Offset effectiveLinkedOffset =
        leaderSize == null
            ? this.offset
            : leaderAnchor.alongSize(leaderSize) - followerAnchor.alongSize(size) + this.offset;
    if (layer == null) {
      layer = FollowerLayer(
        link: link,
        showWhenUnlinked: showWhenUnlinked,
        linkedOffset: effectiveLinkedOffset,
        unlinkedOffset: offset,
      );
    } else {
      layer
        ?..link = link
        ..showWhenUnlinked = showWhenUnlinked
        ..linkedOffset = effectiveLinkedOffset
        ..unlinkedOffset = offset;
    }
    context.pushLayer(
      layer!,
      super.paint,
      Offset.zero,
      childPaintBounds: const Rect.fromLTRB(
        // We don't know where we'll end up, so we have no idea what our cull rect should be.
        double.negativeInfinity,
        double.negativeInfinity,
        double.infinity,
        double.infinity,
      ),
    );
    assert(() {
      layer!.debugCreator = debugCreator;
      return true;
    }());
  }

  @override
  void applyPaintTransform(RenderBox child, Matrix4 transform) {
    transform.multiply(getCurrentTransform());
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<LayerLink>('link', link));
    properties.add(DiagnosticsProperty<bool>('showWhenUnlinked', showWhenUnlinked));
    properties.add(DiagnosticsProperty<Offset>('offset', offset));
    properties.add(TransformProperty('current transform matrix', getCurrentTransform()));
  }
}

/// Render object which inserts an [AnnotatedRegionLayer] into the layer tree.
///
/// See also:
///
///  * [Layer.find], for an example of how this value is retrieved.
///  * [AnnotatedRegionLayer], the layer this render object creates.
class RenderAnnotatedRegion<T extends Object> extends RenderProxyBox {
  /// Creates a new [RenderAnnotatedRegion] to insert [value] into the
  /// layer tree.
  ///
  /// If [sized] is true, the layer is provided with the size of this render
  /// object to clip the results of [Layer.find].
  ///
  /// Neither [value] nor [sized] can be null.
  RenderAnnotatedRegion({required T value, required bool sized, RenderBox? child})
    : _value = value,
      _sized = sized,
      _layerHandle = LayerHandle<AnnotatedRegionLayer<T>>(),
      super(child);

  /// A value which can be retrieved using [Layer.find].
  T get value => _value;
  T _value;
  set value(T newValue) {
    if (_value == newValue) {
      return;
    }
    _value = newValue;
    markNeedsPaint();
  }

  /// Whether the render object will pass its [size] to the [AnnotatedRegionLayer].
  bool get sized => _sized;
  bool _sized;
  set sized(bool value) {
    if (_sized == value) {
      return;
    }
    _sized = value;
    markNeedsPaint();
  }

  final LayerHandle<AnnotatedRegionLayer<T>> _layerHandle;

  @override
  final bool alwaysNeedsCompositing = true;

  @override
  void paint(PaintingContext context, Offset offset) {
    // Annotated region layers are not retained because they do not create engine layers.
    final AnnotatedRegionLayer<T> layer = AnnotatedRegionLayer<T>(
      value,
      size: sized ? size : null,
      offset: sized ? offset : null,
    );
    _layerHandle.layer = layer;
    context.pushLayer(layer, super.paint, offset);
  }

  @override
  void dispose() {
    _layerHandle.layer = null;
    super.dispose();
  }
}
