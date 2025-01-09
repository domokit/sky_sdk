// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/rendering.dart';

const double kTwoPi = 2 * math.pi;

class SectorConstraints extends Constraints {
  const SectorConstraints({
    this.minDeltaRadius = 0.0,
    this.maxDeltaRadius = double.infinity,
    this.minDeltaTheta = 0.0,
    this.maxDeltaTheta = kTwoPi,
  }) : assert(maxDeltaRadius >= minDeltaRadius),
       assert(maxDeltaTheta >= minDeltaTheta);

  const SectorConstraints.tight({double deltaRadius = 0.0, double deltaTheta = 0.0})
    : minDeltaRadius = deltaRadius,
      maxDeltaRadius = deltaRadius,
      minDeltaTheta = deltaTheta,
      maxDeltaTheta = deltaTheta;

  final double minDeltaRadius;
  final double maxDeltaRadius;
  final double minDeltaTheta;
  final double maxDeltaTheta;

  double constrainDeltaRadius(double deltaRadius) {
    return deltaRadius.clamp(minDeltaRadius, maxDeltaRadius);
  }

  double constrainDeltaTheta(double deltaTheta) {
    return deltaTheta.clamp(minDeltaTheta, maxDeltaTheta);
  }

  @override
  bool get isTight => minDeltaTheta >= maxDeltaTheta && minDeltaTheta >= maxDeltaTheta;

  @override
  bool get isNormalized => minDeltaRadius <= maxDeltaRadius && minDeltaTheta <= maxDeltaTheta;

  @override
  bool debugAssertIsValid({
    bool isAppliedConstraint = false,
    InformationCollector? informationCollector,
  }) {
    assert(isNormalized);
    return isNormalized;
  }
}

class SectorDimensions {
  const SectorDimensions({this.deltaRadius = 0.0, this.deltaTheta = 0.0});

  factory SectorDimensions.withConstraints(
    SectorConstraints constraints, {
    double deltaRadius = 0.0,
    double deltaTheta = 0.0,
  }) {
    return SectorDimensions(
      deltaRadius: constraints.constrainDeltaRadius(deltaRadius),
      deltaTheta: constraints.constrainDeltaTheta(deltaTheta),
    );
  }

  final double deltaRadius;
  final double deltaTheta;
}

class SectorParentData extends ParentData {
  double radius = 0.0;
  double theta = 0.0;
}

/// Base class for [RenderObject]s that live in a polar coordinate space.
///
/// In a polar coordinate system each point on a plane is determined by a
/// distance from a reference point ("radius") and an angle from a reference
/// direction ("theta").
///
/// See also:
///
///  * <https://en.wikipedia.org/wiki/Polar_coordinate_system>, which defines
///    the polar coordinate space.
///  * [RenderBox], which is the base class for [RenderObject]s that live in a
///    Cartesian coordinate space.
abstract class RenderSector extends RenderObject {
  @override
  void setupParentData(RenderObject child) {
    if (child.parentData is! SectorParentData) {
      child.parentData = SectorParentData();
    }
  }

  // RenderSectors always use SectorParentData subclasses, as they need to be
  // able to read their position information for painting and hit testing.
  @override
  SectorParentData? get parentData => super.parentData as SectorParentData?;

  SectorDimensions getIntrinsicDimensions(SectorConstraints constraints, double radius) {
    return SectorDimensions.withConstraints(constraints);
  }

  @override
  SectorConstraints get constraints => super.constraints as SectorConstraints;

  @override
  void debugAssertDoesMeetConstraints() {
    assert(deltaRadius < double.infinity);
    assert(deltaTheta < double.infinity);
    assert(constraints.minDeltaRadius <= deltaRadius);
    assert(deltaRadius <= math.max(constraints.minDeltaRadius, constraints.maxDeltaRadius));
    assert(constraints.minDeltaTheta <= deltaTheta);
    assert(deltaTheta <= math.max(constraints.minDeltaTheta, constraints.maxDeltaTheta));
  }

  @override
  void performResize() {
    // default behavior for subclasses that have sizedByParent = true
    deltaRadius = constraints.constrainDeltaRadius(0.0);
    deltaTheta = constraints.constrainDeltaTheta(0.0);
  }

  @override
  void performLayout() {
    // descendants have to either override performLayout() to set both
    // the dimensions and lay out children, or, set sizedByParent to
    // true so that performResize()'s logic above does its thing.
    assert(sizedByParent);
  }

  @override
  Rect get paintBounds => Rect.fromLTWH(0.0, 0.0, 2.0 * deltaRadius, 2.0 * deltaRadius);

  @override
  Rect get semanticBounds =>
      Rect.fromLTWH(-deltaRadius, -deltaRadius, 2.0 * deltaRadius, 2.0 * deltaRadius);

  bool hitTest(SectorHitTestResult result, {required double radius, required double theta}) {
    if (radius < parentData!.radius ||
        radius >= parentData!.radius + deltaRadius ||
        theta < parentData!.theta ||
        theta >= parentData!.theta + deltaTheta) {
      return false;
    }
    hitTestChildren(result, radius: radius, theta: theta);
    result.add(SectorHitTestEntry(this, radius: radius, theta: theta));
    return true;
  }

  void hitTestChildren(
    SectorHitTestResult result, {
    required double radius,
    required double theta,
  }) {}

  late double deltaRadius;
  late double deltaTheta;
}

abstract class RenderDecoratedSector extends RenderSector {
  RenderDecoratedSector(BoxDecoration? decoration) : _decoration = decoration;

  BoxDecoration? _decoration;
  BoxDecoration? get decoration => _decoration;
  set decoration(BoxDecoration? value) {
    if (value == _decoration) {
      return;
    }
    _decoration = value;
    markNeedsPaint();
  }

  // offset must point to the center of the circle
  @override
  void paint(PaintingContext context, Offset offset) {
    assert(parentData is SectorParentData);

    if (_decoration == null) {
      return;
    }

    if (_decoration!.color != null) {
      final Canvas canvas = context.canvas;
      final Paint paint = Paint()..color = _decoration!.color!;
      final Path path = Path();
      final double outerRadius = parentData!.radius + deltaRadius;
      final Rect outerBounds = Rect.fromLTRB(
        offset.dx - outerRadius,
        offset.dy - outerRadius,
        offset.dx + outerRadius,
        offset.dy + outerRadius,
      );
      path.arcTo(outerBounds, parentData!.theta, deltaTheta, true);
      final double innerRadius = parentData!.radius;
      final Rect innerBounds = Rect.fromLTRB(
        offset.dx - innerRadius,
        offset.dy - innerRadius,
        offset.dx + innerRadius,
        offset.dy + innerRadius,
      );
      path.arcTo(innerBounds, parentData!.theta + deltaTheta, -deltaTheta, false);
      path.close();
      canvas.drawPath(path, paint);
    }
  }
}

class SectorChildListParentData extends SectorParentData
    with ContainerParentDataMixin<RenderSector> {}

class RenderSectorWithChildren extends RenderDecoratedSector
    with ContainerRenderObjectMixin<RenderSector, SectorChildListParentData> {
  RenderSectorWithChildren(super.decoration);

  @override
  void hitTestChildren(
    SectorHitTestResult result, {
    required double radius,
    required double theta,
  }) {
    RenderSector? child = lastChild;
    while (child != null) {
      if (child.hitTest(result, radius: radius, theta: theta)) {
        return;
      }
      final SectorChildListParentData childParentData =
          child.parentData! as SectorChildListParentData;
      child = childParentData.previousSibling;
    }
  }

  @override
  void visitChildren(RenderObjectVisitor visitor) {
    RenderSector? child = lastChild;
    while (child != null) {
      visitor(child);
      final SectorChildListParentData childParentData =
          child.parentData! as SectorChildListParentData;
      child = childParentData.previousSibling;
    }
  }
}

class RenderSectorRing extends RenderSectorWithChildren {
  // lays out RenderSector children in a ring

  RenderSectorRing({
    BoxDecoration? decoration,
    double deltaRadius = double.infinity,
    double padding = 0.0,
  }) : _padding = padding,
       assert(deltaRadius >= 0.0),
       _desiredDeltaRadius = deltaRadius,
       super(decoration);

  double _desiredDeltaRadius;
  double get desiredDeltaRadius => _desiredDeltaRadius;
  set desiredDeltaRadius(double value) {
    assert(value >= 0);
    if (_desiredDeltaRadius != value) {
      _desiredDeltaRadius = value;
      markNeedsLayout();
    }
  }

  double _padding;
  double get padding => _padding;
  set padding(double value) {
    // TODO(ianh): avoid code duplication
    if (_padding != value) {
      _padding = value;
      markNeedsLayout();
    }
  }

  @override
  void setupParentData(RenderObject child) {
    // TODO(ianh): avoid code duplication
    if (child.parentData is! SectorChildListParentData) {
      child.parentData = SectorChildListParentData();
    }
  }

  @override
  SectorDimensions getIntrinsicDimensions(SectorConstraints constraints, double radius) {
    final double outerDeltaRadius = constraints.constrainDeltaRadius(desiredDeltaRadius);
    final double innerDeltaRadius = math.max(0.0, outerDeltaRadius - padding * 2.0);
    final double childRadius = radius + padding;
    final double paddingTheta = math.atan(padding / (radius + outerDeltaRadius));
    double innerTheta = paddingTheta; // increments with each child
    double remainingDeltaTheta = math.max(
      0.0,
      constraints.maxDeltaTheta - (innerTheta + paddingTheta),
    );
    RenderSector? child = firstChild;
    while (child != null) {
      final SectorConstraints innerConstraints = SectorConstraints(
        maxDeltaRadius: innerDeltaRadius,
        maxDeltaTheta: remainingDeltaTheta,
      );
      final SectorDimensions childDimensions = child.getIntrinsicDimensions(
        innerConstraints,
        childRadius,
      );
      innerTheta += childDimensions.deltaTheta;
      remainingDeltaTheta -= childDimensions.deltaTheta;
      final SectorChildListParentData childParentData =
          child.parentData! as SectorChildListParentData;
      child = childParentData.nextSibling;
      if (child != null) {
        innerTheta += paddingTheta;
        remainingDeltaTheta -= paddingTheta;
      }
    }
    return SectorDimensions.withConstraints(
      constraints,
      deltaRadius: outerDeltaRadius,
      deltaTheta: innerTheta,
    );
  }

  @override
  void performLayout() {
    assert(parentData is SectorParentData);
    deltaRadius = constraints.constrainDeltaRadius(desiredDeltaRadius);
    assert(deltaRadius < double.infinity);
    final double innerDeltaRadius = deltaRadius - padding * 2.0;
    final double childRadius = parentData!.radius + padding;
    final double paddingTheta = math.atan(padding / (parentData!.radius + deltaRadius));
    double innerTheta = paddingTheta; // increments with each child
    double remainingDeltaTheta = constraints.maxDeltaTheta - (innerTheta + paddingTheta);
    RenderSector? child = firstChild;
    while (child != null) {
      final SectorConstraints innerConstraints = SectorConstraints(
        maxDeltaRadius: innerDeltaRadius,
        maxDeltaTheta: remainingDeltaTheta,
      );
      assert(child.parentData is SectorParentData);
      child.parentData!.theta = innerTheta;
      child.parentData!.radius = childRadius;
      child.layout(innerConstraints, parentUsesSize: true);
      innerTheta += child.deltaTheta;
      remainingDeltaTheta -= child.deltaTheta;
      final SectorChildListParentData childParentData =
          child.parentData! as SectorChildListParentData;
      child = childParentData.nextSibling;
      if (child != null) {
        innerTheta += paddingTheta;
        remainingDeltaTheta -= paddingTheta;
      }
    }
    deltaTheta = innerTheta;
  }

  // offset must point to the center of our circle
  // each sector then knows how to paint itself at its location
  @override
  void paint(PaintingContext context, Offset offset) {
    // TODO(ianh): avoid code duplication
    super.paint(context, offset);
    RenderSector? child = firstChild;
    while (child != null) {
      context.paintChild(child, offset);
      final SectorChildListParentData childParentData =
          child.parentData! as SectorChildListParentData;
      child = childParentData.nextSibling;
    }
  }
}

class RenderSectorSlice extends RenderSectorWithChildren {
  // lays out RenderSector children in a stack

  RenderSectorSlice({BoxDecoration? decoration, double deltaTheta = kTwoPi, double padding = 0.0})
    : _padding = padding,
      _desiredDeltaTheta = deltaTheta,
      super(decoration);

  double _desiredDeltaTheta;
  double get desiredDeltaTheta => _desiredDeltaTheta;
  set desiredDeltaTheta(double value) {
    if (_desiredDeltaTheta != value) {
      _desiredDeltaTheta = value;
      markNeedsLayout();
    }
  }

  double _padding;
  double get padding => _padding;
  set padding(double value) {
    // TODO(ianh): avoid code duplication
    if (_padding != value) {
      _padding = value;
      markNeedsLayout();
    }
  }

  @override
  void setupParentData(RenderObject child) {
    // TODO(ianh): avoid code duplication
    if (child.parentData is! SectorChildListParentData) {
      child.parentData = SectorChildListParentData();
    }
  }

  @override
  SectorDimensions getIntrinsicDimensions(SectorConstraints constraints, double radius) {
    assert(parentData is SectorParentData);
    final double paddingTheta = math.atan(padding / parentData!.radius);
    final double outerDeltaTheta = constraints.constrainDeltaTheta(desiredDeltaTheta);
    final double innerDeltaTheta = outerDeltaTheta - paddingTheta * 2.0;
    double childRadius = parentData!.radius + padding;
    double remainingDeltaRadius = constraints.maxDeltaRadius - (padding * 2.0);
    RenderSector? child = firstChild;
    while (child != null) {
      final SectorConstraints innerConstraints = SectorConstraints(
        maxDeltaRadius: remainingDeltaRadius,
        maxDeltaTheta: innerDeltaTheta,
      );
      final SectorDimensions childDimensions = child.getIntrinsicDimensions(
        innerConstraints,
        childRadius,
      );
      childRadius += childDimensions.deltaRadius;
      remainingDeltaRadius -= childDimensions.deltaRadius;
      final SectorChildListParentData childParentData =
          child.parentData! as SectorChildListParentData;
      child = childParentData.nextSibling;
      childRadius += padding;
      remainingDeltaRadius -= padding;
    }
    return SectorDimensions.withConstraints(
      constraints,
      deltaRadius: childRadius - parentData!.radius,
      deltaTheta: outerDeltaTheta,
    );
  }

  @override
  void performLayout() {
    assert(parentData is SectorParentData);
    deltaTheta = constraints.constrainDeltaTheta(desiredDeltaTheta);
    assert(deltaTheta <= kTwoPi);
    final double paddingTheta = math.atan(padding / parentData!.radius);
    final double innerTheta = parentData!.theta + paddingTheta;
    final double innerDeltaTheta = deltaTheta - paddingTheta * 2.0;
    double childRadius = parentData!.radius + padding;
    double remainingDeltaRadius = constraints.maxDeltaRadius - (padding * 2.0);
    RenderSector? child = firstChild;
    while (child != null) {
      final SectorConstraints innerConstraints = SectorConstraints(
        maxDeltaRadius: remainingDeltaRadius,
        maxDeltaTheta: innerDeltaTheta,
      );
      child.parentData!.theta = innerTheta;
      child.parentData!.radius = childRadius;
      child.layout(innerConstraints, parentUsesSize: true);
      childRadius += child.deltaRadius;
      remainingDeltaRadius -= child.deltaRadius;
      final SectorChildListParentData childParentData =
          child.parentData! as SectorChildListParentData;
      child = childParentData.nextSibling;
      childRadius += padding;
      remainingDeltaRadius -= padding;
    }
    deltaRadius = childRadius - parentData!.radius;
  }

  // offset must point to the center of our circle
  // each sector then knows how to paint itself at its location
  @override
  void paint(PaintingContext context, Offset offset) {
    // TODO(ianh): avoid code duplication
    super.paint(context, offset);
    RenderSector? child = firstChild;
    while (child != null) {
      assert(child.parentData is SectorChildListParentData);
      context.paintChild(child, offset);
      final SectorChildListParentData childParentData =
          child.parentData! as SectorChildListParentData;
      child = childParentData.nextSibling;
    }
  }
}

class RenderBoxToRenderSectorAdapter extends RenderBox
    with RenderObjectWithChildMixin<RenderSector> {
  RenderBoxToRenderSectorAdapter({double innerRadius = 0.0, RenderSector? child})
    : _innerRadius = innerRadius {
    this.child = child;
  }

  double _innerRadius;
  double get innerRadius => _innerRadius;
  set innerRadius(double value) {
    _innerRadius = value;
    markNeedsLayout();
  }

  @override
  void setupParentData(RenderObject child) {
    if (child.parentData is! SectorParentData) {
      child.parentData = SectorParentData();
    }
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    if (child == null) {
      return 0.0;
    }
    return getIntrinsicDimensions(height: height).width;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    if (child == null) {
      return 0.0;
    }
    return getIntrinsicDimensions(height: height).width;
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    if (child == null) {
      return 0.0;
    }
    return getIntrinsicDimensions(width: width).height;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    if (child == null) {
      return 0.0;
    }
    return getIntrinsicDimensions(width: width).height;
  }

  Size getIntrinsicDimensions({double width = double.infinity, double height = double.infinity}) {
    assert(child is RenderSector);
    assert(child!.parentData is SectorParentData);
    if (!width.isFinite && !height.isFinite) {
      return Size.zero;
    }
    final double maxChildDeltaRadius = math.max(0.0, math.min(width, height) / 2.0 - innerRadius);
    final SectorDimensions childDimensions = child!.getIntrinsicDimensions(
      SectorConstraints(maxDeltaRadius: maxChildDeltaRadius),
      innerRadius,
    );
    final double dimension = (innerRadius + childDimensions.deltaRadius) * 2.0;
    return Size.square(dimension);
  }

  @override
  void performLayout() {
    if (child == null || (!constraints.hasBoundedWidth && !constraints.hasBoundedHeight)) {
      size = constraints.constrain(Size.zero);
      child?.layout(SectorConstraints(maxDeltaRadius: innerRadius), parentUsesSize: true);
      return;
    }
    assert(child is RenderSector);
    assert(child!.parentData is SectorParentData);
    final double maxChildDeltaRadius =
        math.min(constraints.maxWidth, constraints.maxHeight) / 2.0 - innerRadius;
    child!.parentData!.radius = innerRadius;
    child!.parentData!.theta = 0.0;
    child!.layout(SectorConstraints(maxDeltaRadius: maxChildDeltaRadius), parentUsesSize: true);
    final double dimension = (innerRadius + child!.deltaRadius) * 2.0;
    size = constraints.constrain(Size(dimension, dimension));
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    super.paint(context, offset);
    if (child != null) {
      final Rect bounds = offset & size;
      // we move the offset to the center of the circle for the RenderSectors
      context.paintChild(child!, bounds.center);
    }
  }

  @override
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    if (child == null) {
      return false;
    }
    double x = position.dx;
    double y = position.dy;
    // translate to our origin
    x -= size.width / 2.0;
    y -= size.height / 2.0;
    // convert to radius/theta
    final double radius = math.sqrt(x * x + y * y);
    final double theta = (math.atan2(x, -y) - math.pi / 2.0) % kTwoPi;
    if (radius < innerRadius) {
      return false;
    }
    if (radius >= innerRadius + child!.deltaRadius) {
      return false;
    }
    if (theta > child!.deltaTheta) {
      return false;
    }
    child!.hitTest(SectorHitTestResult.wrap(result), radius: radius, theta: theta);
    result.add(BoxHitTestEntry(this, position));
    return true;
  }
}

class RenderSolidColor extends RenderDecoratedSector {
  RenderSolidColor(
    this.backgroundColor, {
    this.desiredDeltaRadius = double.infinity,
    this.desiredDeltaTheta = kTwoPi,
  }) : super(BoxDecoration(color: backgroundColor));

  double desiredDeltaRadius;
  double desiredDeltaTheta;
  final Color backgroundColor;

  @override
  SectorDimensions getIntrinsicDimensions(SectorConstraints constraints, double radius) {
    return SectorDimensions.withConstraints(constraints, deltaTheta: desiredDeltaTheta);
  }

  @override
  void performLayout() {
    deltaRadius = constraints.constrainDeltaRadius(desiredDeltaRadius);
    deltaTheta = constraints.constrainDeltaTheta(desiredDeltaTheta);
  }

  @override
  void handleEvent(PointerEvent event, HitTestEntry entry) {
    if (event is PointerDownEvent) {
      decoration = const BoxDecoration(color: Color(0xFFFF0000));
    } else if (event is PointerUpEvent) {
      decoration = BoxDecoration(color: backgroundColor);
    }
  }
}

/// The result of performing a hit test on [RenderSector]s.
class SectorHitTestResult extends HitTestResult {
  /// Creates an empty hit test result for hit testing on [RenderSector].
  SectorHitTestResult() : super();

  /// Wraps `result` to create a [HitTestResult] that implements the
  /// [SectorHitTestResult] protocol for hit testing on [RenderSector]s.
  ///
  /// This method is used by [RenderObject]s that adapt between the
  /// [RenderSector]-world and the non-[RenderSector]-world to convert a (subtype of)
  /// [HitTestResult] to a [SectorHitTestResult] for hit testing on [RenderSector]s.
  ///
  /// The [HitTestEntry]s added to the returned [SectorHitTestResult] are also
  /// added to the wrapped `result` (both share the same underlying data
  /// structure to store [HitTestEntry]s).
  ///
  /// See also:
  ///
  ///  * [HitTestResult.wrap], which turns a [SectorHitTestResult] back into a
  ///    generic [HitTestResult].
  SectorHitTestResult.wrap(super.result) : super.wrap();

  // TODO(goderbauer): Add convenience methods to transform hit test positions
  //    once we have RenderSector implementations that move the origin of their
  //    children (e.g. RenderSectorTransform analogs to RenderTransform).
}

/// A hit test entry used by [RenderSector].
class SectorHitTestEntry extends HitTestEntry {
  /// Creates a box hit test entry.
  SectorHitTestEntry(RenderSector super.target, {required this.radius, required this.theta});

  @override
  RenderSector get target => super.target as RenderSector;

  /// The radius component of the hit test position in the local coordinates of
  /// [target].
  final double radius;

  /// The theta component of the hit test position in the local coordinates of
  /// [target].
  final double theta;
}
