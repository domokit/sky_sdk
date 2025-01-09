// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'image.dart';
/// @docImport 'paragraph.dart';
/// @docImport 'proxy_box.dart';
/// @docImport 'shifted_box.dart';
/// @docImport 'sliver.dart';
/// @docImport 'viewport.dart';
library;

import 'dart:math' as math;
import 'dart:ui' as ui show ViewConstraints, lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';

import 'package:vector_math/vector_math_64.dart';

import 'debug.dart';
import 'object.dart';

// Examples can assume:
// abstract class RenderBar extends RenderBox { }
// late RenderBox firstChild;
// void markNeedsLayout() { }

// This class should only be used in debug builds.
class _DebugSize extends Size {
  _DebugSize(super.source, this._owner, this._canBeUsedByParent) : super.copy();
  final RenderBox _owner;
  final bool _canBeUsedByParent;
}

/// Immutable layout constraints for [RenderBox] layout.
///
/// A [Size] respects a [BoxConstraints] if, and only if, all of the following
/// relations hold:
///
/// * [minWidth] <= [Size.width] <= [maxWidth]
/// * [minHeight] <= [Size.height] <= [maxHeight]
///
/// The constraints themselves must satisfy these relations:
///
/// * 0.0 <= [minWidth] <= [maxWidth] <= [double.infinity]
/// * 0.0 <= [minHeight] <= [maxHeight] <= [double.infinity]
///
/// [double.infinity] is a legal value for each constraint.
///
/// ## The box layout model
///
/// Render objects in the Flutter framework are laid out by a one-pass layout
/// model which walks down the render tree passing constraints, then walks back
/// up the render tree passing concrete geometry.
///
/// For boxes, the constraints are [BoxConstraints], which, as described herein,
/// consist of four numbers: a minimum width [minWidth], a maximum width
/// [maxWidth], a minimum height [minHeight], and a maximum height [maxHeight].
///
/// The geometry for boxes consists of a [Size], which must satisfy the
/// constraints described above.
///
/// Each [RenderBox] (the objects that provide the layout models for box
/// widgets) receives [BoxConstraints] from its parent, then lays out each of
/// its children, then picks a [Size] that satisfies the [BoxConstraints].
///
/// Render objects position their children independently of laying them out.
/// Frequently, the parent will use the children's sizes to determine their
/// position. A child does not know its position and will not necessarily be
/// laid out again, or repainted, if its position changes.
///
/// ## Terminology
///
/// When the minimum constraints and the maximum constraint in an axis are the
/// same, that axis is _tightly_ constrained. See: [
/// BoxConstraints.tightFor], [BoxConstraints.tightForFinite], [tighten],
/// [hasTightWidth], [hasTightHeight], [isTight].
///
/// An axis with a minimum constraint of 0.0 is _loose_ (regardless of the
/// maximum constraint; if it is also 0.0, then the axis is simultaneously tight
/// and loose!). See: [BoxConstraints.loose], [loosen].
///
/// An axis whose maximum constraint is not infinite is _bounded_. See:
/// [hasBoundedWidth], [hasBoundedHeight].
///
/// An axis whose maximum constraint is infinite is _unbounded_. An axis is
/// _expanding_ if it is tightly infinite (its minimum and maximum constraints
/// are both infinite). See: [BoxConstraints.expand].
///
/// An axis whose _minimum_ constraint is infinite is just said to be _infinite_
/// (since by definition the maximum constraint must also be infinite in that
/// case). See: [hasInfiniteWidth], [hasInfiniteHeight].
///
/// A size is _constrained_ when it satisfies a [BoxConstraints] description.
/// See: [constrain], [constrainWidth], [constrainHeight],
/// [constrainDimensions], [constrainSizeAndAttemptToPreserveAspectRatio],
/// [isSatisfiedBy].
class BoxConstraints extends Constraints {
  /// Creates box constraints with the given constraints.
  const BoxConstraints({
    this.minWidth = 0.0,
    this.maxWidth = double.infinity,
    this.minHeight = 0.0,
    this.maxHeight = double.infinity,
  });

  /// Creates box constraints that is respected only by the given size.
  BoxConstraints.tight(Size size)
    : minWidth = size.width,
      maxWidth = size.width,
      minHeight = size.height,
      maxHeight = size.height;

  /// Creates box constraints that require the given width or height.
  ///
  /// See also:
  ///
  ///  * [BoxConstraints.tightForFinite], which is similar but instead of
  ///    being tight if the value is non-null, is tight if the value is not
  ///    infinite.
  const BoxConstraints.tightFor({double? width, double? height})
    : minWidth = width ?? 0.0,
      maxWidth = width ?? double.infinity,
      minHeight = height ?? 0.0,
      maxHeight = height ?? double.infinity;

  /// Creates box constraints that require the given width or height, except if
  /// they are infinite.
  ///
  /// See also:
  ///
  ///  * [BoxConstraints.tightFor], which is similar but instead of being
  ///    tight if the value is not infinite, is tight if the value is non-null.
  const BoxConstraints.tightForFinite({
    double width = double.infinity,
    double height = double.infinity,
  }) : minWidth = width != double.infinity ? width : 0.0,
       maxWidth = width != double.infinity ? width : double.infinity,
       minHeight = height != double.infinity ? height : 0.0,
       maxHeight = height != double.infinity ? height : double.infinity;

  /// Creates box constraints that forbid sizes larger than the given size.
  BoxConstraints.loose(Size size)
    : minWidth = 0.0,
      maxWidth = size.width,
      minHeight = 0.0,
      maxHeight = size.height;

  /// Creates box constraints that expand to fill another box constraints.
  ///
  /// If width or height is given, the constraints will require exactly the
  /// given value in the given dimension.
  const BoxConstraints.expand({double? width, double? height})
    : minWidth = width ?? double.infinity,
      maxWidth = width ?? double.infinity,
      minHeight = height ?? double.infinity,
      maxHeight = height ?? double.infinity;

  /// Creates box constraints that match the given view constraints.
  BoxConstraints.fromViewConstraints(ui.ViewConstraints constraints)
    : minWidth = constraints.minWidth,
      maxWidth = constraints.maxWidth,
      minHeight = constraints.minHeight,
      maxHeight = constraints.maxHeight;

  /// The minimum width that satisfies the constraints.
  final double minWidth;

  /// The maximum width that satisfies the constraints.
  ///
  /// Might be [double.infinity].
  final double maxWidth;

  /// The minimum height that satisfies the constraints.
  final double minHeight;

  /// The maximum height that satisfies the constraints.
  ///
  /// Might be [double.infinity].
  final double maxHeight;

  /// Creates a copy of this box constraints but with the given fields replaced with the new values.
  BoxConstraints copyWith({
    double? minWidth,
    double? maxWidth,
    double? minHeight,
    double? maxHeight,
  }) {
    return BoxConstraints(
      minWidth: minWidth ?? this.minWidth,
      maxWidth: maxWidth ?? this.maxWidth,
      minHeight: minHeight ?? this.minHeight,
      maxHeight: maxHeight ?? this.maxHeight,
    );
  }

  /// Returns new box constraints that are smaller by the given edge dimensions.
  BoxConstraints deflate(EdgeInsetsGeometry edges) {
    assert(debugAssertIsValid());
    final double horizontal = edges.horizontal;
    final double vertical = edges.vertical;
    final double deflatedMinWidth = math.max(0.0, minWidth - horizontal);
    final double deflatedMinHeight = math.max(0.0, minHeight - vertical);
    return BoxConstraints(
      minWidth: deflatedMinWidth,
      maxWidth: math.max(deflatedMinWidth, maxWidth - horizontal),
      minHeight: deflatedMinHeight,
      maxHeight: math.max(deflatedMinHeight, maxHeight - vertical),
    );
  }

  /// Returns new box constraints that remove the minimum width and height requirements.
  BoxConstraints loosen() {
    assert(debugAssertIsValid());
    return BoxConstraints(maxWidth: maxWidth, maxHeight: maxHeight);
  }

  /// Returns new box constraints that respect the given constraints while being
  /// as close as possible to the original constraints.
  BoxConstraints enforce(BoxConstraints constraints) {
    return BoxConstraints(
      minWidth: clampDouble(minWidth, constraints.minWidth, constraints.maxWidth),
      maxWidth: clampDouble(maxWidth, constraints.minWidth, constraints.maxWidth),
      minHeight: clampDouble(minHeight, constraints.minHeight, constraints.maxHeight),
      maxHeight: clampDouble(maxHeight, constraints.minHeight, constraints.maxHeight),
    );
  }

  /// Returns new box constraints with a tight width and/or height as close to
  /// the given width and height as possible while still respecting the original
  /// box constraints.
  BoxConstraints tighten({double? width, double? height}) {
    return BoxConstraints(
      minWidth: width == null ? minWidth : clampDouble(width, minWidth, maxWidth),
      maxWidth: width == null ? maxWidth : clampDouble(width, minWidth, maxWidth),
      minHeight: height == null ? minHeight : clampDouble(height, minHeight, maxHeight),
      maxHeight: height == null ? maxHeight : clampDouble(height, minHeight, maxHeight),
    );
  }

  /// A box constraints with the width and height constraints flipped.
  BoxConstraints get flipped {
    return BoxConstraints(
      minWidth: minHeight,
      maxWidth: maxHeight,
      minHeight: minWidth,
      maxHeight: maxWidth,
    );
  }

  /// Returns box constraints with the same width constraints but with
  /// unconstrained height.
  BoxConstraints widthConstraints() => BoxConstraints(minWidth: minWidth, maxWidth: maxWidth);

  /// Returns box constraints with the same height constraints but with
  /// unconstrained width.
  BoxConstraints heightConstraints() => BoxConstraints(minHeight: minHeight, maxHeight: maxHeight);

  /// Returns the width that both satisfies the constraints and is as close as
  /// possible to the given width.
  double constrainWidth([double width = double.infinity]) {
    assert(debugAssertIsValid());
    return clampDouble(width, minWidth, maxWidth);
  }

  /// Returns the height that both satisfies the constraints and is as close as
  /// possible to the given height.
  double constrainHeight([double height = double.infinity]) {
    assert(debugAssertIsValid());
    return clampDouble(height, minHeight, maxHeight);
  }

  Size _debugPropagateDebugSize(Size size, Size result) {
    assert(() {
      if (size is _DebugSize) {
        result = _DebugSize(result, size._owner, size._canBeUsedByParent);
      }
      return true;
    }());
    return result;
  }

  /// Returns the size that both satisfies the constraints and is as close as
  /// possible to the given size.
  ///
  /// See also:
  ///
  ///  * [constrainDimensions], which applies the same algorithm to
  ///    separately provided widths and heights.
  Size constrain(Size size) {
    Size result = Size(constrainWidth(size.width), constrainHeight(size.height));
    assert(() {
      result = _debugPropagateDebugSize(size, result);
      return true;
    }());
    return result;
  }

  /// Returns the size that both satisfies the constraints and is as close as
  /// possible to the given width and height.
  ///
  /// When you already have a [Size], prefer [constrain], which applies the same
  /// algorithm to a [Size] directly.
  Size constrainDimensions(double width, double height) {
    return Size(constrainWidth(width), constrainHeight(height));
  }

  /// Returns a size that attempts to meet the following conditions, in order:
  ///
  ///  * The size must satisfy these constraints.
  ///  * The aspect ratio of the returned size matches the aspect ratio of the
  ///    given size.
  ///  * The returned size is as big as possible while still being equal to or
  ///    smaller than the given size.
  Size constrainSizeAndAttemptToPreserveAspectRatio(Size size) {
    if (isTight) {
      Size result = smallest;
      assert(() {
        result = _debugPropagateDebugSize(size, result);
        return true;
      }());
      return result;
    }

    if (size.isEmpty) {
      return constrain(size);
    }

    double width = size.width;
    double height = size.height;
    final double aspectRatio = width / height;

    if (width > maxWidth) {
      width = maxWidth;
      height = width / aspectRatio;
    }

    if (height > maxHeight) {
      height = maxHeight;
      width = height * aspectRatio;
    }

    if (width < minWidth) {
      width = minWidth;
      height = width / aspectRatio;
    }

    if (height < minHeight) {
      height = minHeight;
      width = height * aspectRatio;
    }

    Size result = Size(constrainWidth(width), constrainHeight(height));
    assert(() {
      result = _debugPropagateDebugSize(size, result);
      return true;
    }());
    return result;
  }

  /// The biggest size that satisfies the constraints.
  Size get biggest => Size(constrainWidth(), constrainHeight());

  /// The smallest size that satisfies the constraints.
  Size get smallest => Size(constrainWidth(0.0), constrainHeight(0.0));

  /// Whether there is exactly one width value that satisfies the constraints.
  bool get hasTightWidth => minWidth >= maxWidth;

  /// Whether there is exactly one height value that satisfies the constraints.
  bool get hasTightHeight => minHeight >= maxHeight;

  /// Whether there is exactly one size that satisfies the constraints.
  @override
  bool get isTight => hasTightWidth && hasTightHeight;

  /// Whether there is an upper bound on the maximum width.
  ///
  /// See also:
  ///
  ///  * [hasBoundedHeight], the equivalent for the vertical axis.
  ///  * [hasInfiniteWidth], which describes whether the minimum width
  ///    constraint is infinite.
  bool get hasBoundedWidth => maxWidth < double.infinity;

  /// Whether there is an upper bound on the maximum height.
  ///
  /// See also:
  ///
  ///  * [hasBoundedWidth], the equivalent for the horizontal axis.
  ///  * [hasInfiniteHeight], which describes whether the minimum height
  ///    constraint is infinite.
  bool get hasBoundedHeight => maxHeight < double.infinity;

  /// Whether the width constraint is infinite.
  ///
  /// Such a constraint is used to indicate that a box should grow as large as
  /// some other constraint (in this case, horizontally). If constraints are
  /// infinite, then they must have other (non-infinite) constraints [enforce]d
  /// upon them, or must be [tighten]ed, before they can be used to derive a
  /// [Size] for a [RenderBox.size].
  ///
  /// See also:
  ///
  ///  * [hasInfiniteHeight], the equivalent for the vertical axis.
  ///  * [hasBoundedWidth], which describes whether the maximum width
  ///    constraint is finite.
  bool get hasInfiniteWidth => minWidth >= double.infinity;

  /// Whether the height constraint is infinite.
  ///
  /// Such a constraint is used to indicate that a box should grow as large as
  /// some other constraint (in this case, vertically). If constraints are
  /// infinite, then they must have other (non-infinite) constraints [enforce]d
  /// upon them, or must be [tighten]ed, before they can be used to derive a
  /// [Size] for a [RenderBox.size].
  ///
  /// See also:
  ///
  ///  * [hasInfiniteWidth], the equivalent for the horizontal axis.
  ///  * [hasBoundedHeight], which describes whether the maximum height
  ///    constraint is finite.
  bool get hasInfiniteHeight => minHeight >= double.infinity;

  /// Whether the given size satisfies the constraints.
  bool isSatisfiedBy(Size size) {
    assert(debugAssertIsValid());
    return (minWidth <= size.width) &&
        (size.width <= maxWidth) &&
        (minHeight <= size.height) &&
        (size.height <= maxHeight);
  }

  /// Scales each constraint parameter by the given factor.
  BoxConstraints operator *(double factor) {
    return BoxConstraints(
      minWidth: minWidth * factor,
      maxWidth: maxWidth * factor,
      minHeight: minHeight * factor,
      maxHeight: maxHeight * factor,
    );
  }

  /// Scales each constraint parameter by the inverse of the given factor.
  BoxConstraints operator /(double factor) {
    return BoxConstraints(
      minWidth: minWidth / factor,
      maxWidth: maxWidth / factor,
      minHeight: minHeight / factor,
      maxHeight: maxHeight / factor,
    );
  }

  /// Scales each constraint parameter by the inverse of the given factor, rounded to the nearest integer.
  BoxConstraints operator ~/(double factor) {
    return BoxConstraints(
      minWidth: (minWidth ~/ factor).toDouble(),
      maxWidth: (maxWidth ~/ factor).toDouble(),
      minHeight: (minHeight ~/ factor).toDouble(),
      maxHeight: (maxHeight ~/ factor).toDouble(),
    );
  }

  /// Computes the remainder of each constraint parameter by the given value.
  BoxConstraints operator %(double value) {
    return BoxConstraints(
      minWidth: minWidth % value,
      maxWidth: maxWidth % value,
      minHeight: minHeight % value,
      maxHeight: maxHeight % value,
    );
  }

  /// Linearly interpolate between two BoxConstraints.
  ///
  /// If either is null, this function interpolates from a [BoxConstraints]
  /// object whose fields are all set to 0.0.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static BoxConstraints? lerp(BoxConstraints? a, BoxConstraints? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    if (a == null) {
      return b! * t;
    }
    if (b == null) {
      return a * (1.0 - t);
    }
    assert(a.debugAssertIsValid());
    assert(b.debugAssertIsValid());
    assert(
      (a.minWidth.isFinite && b.minWidth.isFinite) ||
          (a.minWidth == double.infinity && b.minWidth == double.infinity),
      'Cannot interpolate between finite constraints and unbounded constraints.',
    );
    assert(
      (a.maxWidth.isFinite && b.maxWidth.isFinite) ||
          (a.maxWidth == double.infinity && b.maxWidth == double.infinity),
      'Cannot interpolate between finite constraints and unbounded constraints.',
    );
    assert(
      (a.minHeight.isFinite && b.minHeight.isFinite) ||
          (a.minHeight == double.infinity && b.minHeight == double.infinity),
      'Cannot interpolate between finite constraints and unbounded constraints.',
    );
    assert(
      (a.maxHeight.isFinite && b.maxHeight.isFinite) ||
          (a.maxHeight == double.infinity && b.maxHeight == double.infinity),
      'Cannot interpolate between finite constraints and unbounded constraints.',
    );
    return BoxConstraints(
      minWidth: a.minWidth.isFinite ? ui.lerpDouble(a.minWidth, b.minWidth, t)! : double.infinity,
      maxWidth: a.maxWidth.isFinite ? ui.lerpDouble(a.maxWidth, b.maxWidth, t)! : double.infinity,
      minHeight:
          a.minHeight.isFinite ? ui.lerpDouble(a.minHeight, b.minHeight, t)! : double.infinity,
      maxHeight:
          a.maxHeight.isFinite ? ui.lerpDouble(a.maxHeight, b.maxHeight, t)! : double.infinity,
    );
  }

  /// Returns whether the object's constraints are normalized.
  /// Constraints are normalized if the minimums are less than or
  /// equal to the corresponding maximums.
  ///
  /// For example, a BoxConstraints object with a minWidth of 100.0
  /// and a maxWidth of 90.0 is not normalized.
  ///
  /// Most of the APIs on BoxConstraints expect the constraints to be
  /// normalized and have undefined behavior when they are not. In
  /// debug mode, many of these APIs will assert if the constraints
  /// are not normalized.
  @override
  bool get isNormalized {
    return minWidth >= 0.0 && minWidth <= maxWidth && minHeight >= 0.0 && minHeight <= maxHeight;
  }

  @override
  bool debugAssertIsValid({
    bool isAppliedConstraint = false,
    InformationCollector? informationCollector,
  }) {
    assert(() {
      void throwError(DiagnosticsNode message) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          message,
          if (informationCollector != null) ...informationCollector(),
          DiagnosticsProperty<BoxConstraints>(
            'The offending constraints were',
            this,
            style: DiagnosticsTreeStyle.errorProperty,
          ),
        ]);
      }

      if (minWidth.isNaN || maxWidth.isNaN || minHeight.isNaN || maxHeight.isNaN) {
        final List<String> affectedFieldsList = <String>[
          if (minWidth.isNaN) 'minWidth',
          if (maxWidth.isNaN) 'maxWidth',
          if (minHeight.isNaN) 'minHeight',
          if (maxHeight.isNaN) 'maxHeight',
        ];
        assert(affectedFieldsList.isNotEmpty);
        if (affectedFieldsList.length > 1) {
          affectedFieldsList.add('and ${affectedFieldsList.removeLast()}');
        }
        final String whichFields = switch (affectedFieldsList.length) {
          1 => affectedFieldsList.single,
          2 => affectedFieldsList.join(' '),
          _ => affectedFieldsList.join(', '),
        };
        throwError(
          ErrorSummary(
            'BoxConstraints has ${affectedFieldsList.length == 1 ? 'a NaN value' : 'NaN values'} in $whichFields.',
          ),
        );
      }
      if (minWidth < 0.0 && minHeight < 0.0) {
        throwError(
          ErrorSummary(
            'BoxConstraints has both a negative minimum width and a negative minimum height.',
          ),
        );
      }
      if (minWidth < 0.0) {
        throwError(ErrorSummary('BoxConstraints has a negative minimum width.'));
      }
      if (minHeight < 0.0) {
        throwError(ErrorSummary('BoxConstraints has a negative minimum height.'));
      }
      if (maxWidth < minWidth && maxHeight < minHeight) {
        throwError(
          ErrorSummary('BoxConstraints has both width and height constraints non-normalized.'),
        );
      }
      if (maxWidth < minWidth) {
        throwError(ErrorSummary('BoxConstraints has non-normalized width constraints.'));
      }
      if (maxHeight < minHeight) {
        throwError(ErrorSummary('BoxConstraints has non-normalized height constraints.'));
      }
      if (isAppliedConstraint) {
        if (minWidth.isInfinite && minHeight.isInfinite) {
          throwError(ErrorSummary('BoxConstraints forces an infinite width and infinite height.'));
        }
        if (minWidth.isInfinite) {
          throwError(ErrorSummary('BoxConstraints forces an infinite width.'));
        }
        if (minHeight.isInfinite) {
          throwError(ErrorSummary('BoxConstraints forces an infinite height.'));
        }
      }
      assert(isNormalized);
      return true;
    }());
    return isNormalized;
  }

  /// Returns a box constraints that [isNormalized].
  ///
  /// The returned [maxWidth] is at least as large as the [minWidth]. Similarly,
  /// the returned [maxHeight] is at least as large as the [minHeight].
  BoxConstraints normalize() {
    if (isNormalized) {
      return this;
    }
    final double minWidth = this.minWidth >= 0.0 ? this.minWidth : 0.0;
    final double minHeight = this.minHeight >= 0.0 ? this.minHeight : 0.0;
    return BoxConstraints(
      minWidth: minWidth,
      maxWidth: minWidth > maxWidth ? minWidth : maxWidth,
      minHeight: minHeight,
      maxHeight: minHeight > maxHeight ? minHeight : maxHeight,
    );
  }

  @override
  bool operator ==(Object other) {
    assert(debugAssertIsValid());
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    assert(other is BoxConstraints && other.debugAssertIsValid());
    return other is BoxConstraints &&
        other.minWidth == minWidth &&
        other.maxWidth == maxWidth &&
        other.minHeight == minHeight &&
        other.maxHeight == maxHeight;
  }

  @override
  int get hashCode {
    assert(debugAssertIsValid());
    return Object.hash(minWidth, maxWidth, minHeight, maxHeight);
  }

  @override
  String toString() {
    final String annotation = isNormalized ? '' : '; NOT NORMALIZED';
    if (minWidth == double.infinity && minHeight == double.infinity) {
      return 'BoxConstraints(biggest$annotation)';
    }
    if (minWidth == 0 &&
        maxWidth == double.infinity &&
        minHeight == 0 &&
        maxHeight == double.infinity) {
      return 'BoxConstraints(unconstrained$annotation)';
    }
    String describe(double min, double max, String dim) {
      if (min == max) {
        return '$dim=${min.toStringAsFixed(1)}';
      }
      return '${min.toStringAsFixed(1)}<=$dim<=${max.toStringAsFixed(1)}';
    }

    final String width = describe(minWidth, maxWidth, 'w');
    final String height = describe(minHeight, maxHeight, 'h');
    return 'BoxConstraints($width, $height$annotation)';
  }
}

/// Method signature for hit testing a [RenderBox].
///
/// Used by [BoxHitTestResult.addWithPaintTransform] to hit test children
/// of a [RenderBox].
///
/// See also:
///
///  * [RenderBox.hitTest], which documents more details around hit testing
///    [RenderBox]es.
typedef BoxHitTest = bool Function(BoxHitTestResult result, Offset position);

/// Method signature for hit testing a [RenderBox] with a manually
/// managed position (one that is passed out-of-band).
///
/// Used by [RenderSliverSingleBoxAdapter.hitTestBoxChild] to hit test
/// [RenderBox] children of a [RenderSliver].
///
/// See also:
///
///  * [RenderBox.hitTest], which documents more details around hit testing
///    [RenderBox]es.
typedef BoxHitTestWithOutOfBandPosition = bool Function(BoxHitTestResult result);

/// The result of performing a hit test on [RenderBox]es.
///
/// An instance of this class is provided to [RenderBox.hitTest] to record the
/// result of the hit test.
class BoxHitTestResult extends HitTestResult {
  /// Creates an empty hit test result for hit testing on [RenderBox].
  BoxHitTestResult() : super();

  /// Wraps `result` to create a [HitTestResult] that implements the
  /// [BoxHitTestResult] protocol for hit testing on [RenderBox]es.
  ///
  /// This method is used by [RenderObject]s that adapt between the
  /// [RenderBox]-world and the non-[RenderBox]-world to convert a (subtype of)
  /// [HitTestResult] to a [BoxHitTestResult] for hit testing on [RenderBox]es.
  ///
  /// The [HitTestEntry] instances added to the returned [BoxHitTestResult] are
  /// also added to the wrapped `result` (both share the same underlying data
  /// structure to store [HitTestEntry] instances).
  ///
  /// See also:
  ///
  ///  * [HitTestResult.wrap], which turns a [BoxHitTestResult] back into a
  ///    generic [HitTestResult].
  ///  * [SliverHitTestResult.wrap], which turns a [BoxHitTestResult] into a
  ///    [SliverHitTestResult] for hit testing on [RenderSliver] children.
  BoxHitTestResult.wrap(super.result) : super.wrap();

  /// Transforms `position` to the local coordinate system of a child for
  /// hit-testing the child.
  ///
  /// The actual hit testing of the child needs to be implemented in the
  /// provided `hitTest` callback, which is invoked with the transformed
  /// `position` as argument.
  ///
  /// The provided paint `transform` (which describes the transform from the
  /// child to the parent in 3D) is processed by
  /// [PointerEvent.removePerspectiveTransform] to remove the
  /// perspective component and inverted before it is used to transform
  /// `position` from the coordinate system of the parent to the system of the
  /// child.
  ///
  /// If `transform` is null it will be treated as the identity transform and
  /// `position` is provided to the `hitTest` callback as-is. If `transform`
  /// cannot be inverted, the `hitTest` callback is not invoked and false is
  /// returned. Otherwise, the return value of the `hitTest` callback is
  /// returned.
  ///
  /// The `position` argument may be null, which will be forwarded to the
  /// `hitTest` callback as-is. Using null as the position can be useful if
  /// the child speaks a different hit test protocol than the parent and the
  /// position is not required to do the actual hit testing in that protocol.
  ///
  /// The function returns the return value of the `hitTest` callback.
  ///
  /// {@tool snippet}
  /// This method is used in [RenderBox.hitTestChildren] when the child and
  /// parent don't share the same origin.
  ///
  /// ```dart
  /// abstract class RenderFoo extends RenderBox {
  ///   final Matrix4 _effectiveTransform = Matrix4.rotationZ(50);
  ///
  ///   @override
  ///   void applyPaintTransform(RenderBox child, Matrix4 transform) {
  ///     transform.multiply(_effectiveTransform);
  ///   }
  ///
  ///   @override
  ///   bool hitTestChildren(BoxHitTestResult result, { required Offset position }) {
  ///     return result.addWithPaintTransform(
  ///       transform: _effectiveTransform,
  ///       position: position,
  ///       hitTest: (BoxHitTestResult result, Offset position) {
  ///         return super.hitTestChildren(result, position: position);
  ///       },
  ///     );
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [addWithPaintOffset], which can be used for `transform`s that are just
  ///    simple matrix translations by an [Offset].
  ///  * [addWithRawTransform], which takes a transform matrix that is directly
  ///    used to transform the position without any pre-processing.
  bool addWithPaintTransform({
    required Matrix4? transform,
    required Offset position,
    required BoxHitTest hitTest,
  }) {
    if (transform != null) {
      transform = Matrix4.tryInvert(PointerEvent.removePerspectiveTransform(transform));
      if (transform == null) {
        // Objects are not visible on screen and cannot be hit-tested.
        return false;
      }
    }
    return addWithRawTransform(transform: transform, position: position, hitTest: hitTest);
  }

  /// Convenience method for hit testing children, that are translated by
  /// an [Offset].
  ///
  /// The actual hit testing of the child needs to be implemented in the
  /// provided `hitTest` callback, which is invoked with the transformed
  /// `position` as argument.
  ///
  /// This method can be used as a convenience over [addWithPaintTransform] if
  /// a parent paints a child at an `offset`.
  ///
  /// A null value for `offset` is treated as if [Offset.zero] was provided.
  ///
  /// The function returns the return value of the `hitTest` callback.
  ///
  /// See also:
  ///
  ///  * [addWithPaintTransform], which takes a generic paint transform matrix and
  ///    documents the intended usage of this API in more detail.
  bool addWithPaintOffset({
    required Offset? offset,
    required Offset position,
    required BoxHitTest hitTest,
  }) {
    final Offset transformedPosition = offset == null ? position : position - offset;
    if (offset != null) {
      pushOffset(-offset);
    }
    final bool isHit = hitTest(this, transformedPosition);
    if (offset != null) {
      popTransform();
    }
    return isHit;
  }

  /// Transforms `position` to the local coordinate system of a child for
  /// hit-testing the child.
  ///
  /// The actual hit testing of the child needs to be implemented in the
  /// provided `hitTest` callback, which is invoked with the transformed
  /// `position` as argument.
  ///
  /// Unlike [addWithPaintTransform], the provided `transform` matrix is used
  /// directly to transform `position` without any pre-processing.
  ///
  /// If `transform` is null it will be treated as the identity transform ad
  /// `position` is provided to the `hitTest` callback as-is.
  ///
  /// The function returns the return value of the `hitTest` callback.
  ///
  /// See also:
  ///
  ///  * [addWithPaintTransform], which accomplishes the same thing, but takes a
  ///    _paint_ transform matrix.
  bool addWithRawTransform({
    required Matrix4? transform,
    required Offset position,
    required BoxHitTest hitTest,
  }) {
    final Offset transformedPosition =
        transform == null ? position : MatrixUtils.transformPoint(transform, position);
    if (transform != null) {
      pushTransform(transform);
    }
    final bool isHit = hitTest(this, transformedPosition);
    if (transform != null) {
      popTransform();
    }
    return isHit;
  }

  /// Pass-through method for adding a hit test while manually managing
  /// the position transformation logic.
  ///
  /// The actual hit testing of the child needs to be implemented in the
  /// provided `hitTest` callback. The position needs to be handled by
  /// the caller.
  ///
  /// The function returns the return value of the `hitTest` callback.
  ///
  /// A `paintOffset`, `paintTransform`, or `rawTransform` should be
  /// passed to the method to update the hit test stack.
  ///
  ///  * `paintOffset` has the semantics of the `offset` passed to
  ///    [addWithPaintOffset].
  ///
  ///  * `paintTransform` has the semantics of the `transform` passed to
  ///    [addWithPaintTransform], except that it must be invertible; it
  ///    is the responsibility of the caller to ensure this.
  ///
  ///  * `rawTransform` has the semantics of the `transform` passed to
  ///    [addWithRawTransform].
  ///
  /// Exactly one of these must be non-null.
  ///
  /// See also:
  ///
  ///  * [addWithPaintTransform], which takes a generic paint transform matrix and
  ///    documents the intended usage of this API in more detail.
  bool addWithOutOfBandPosition({
    Offset? paintOffset,
    Matrix4? paintTransform,
    Matrix4? rawTransform,
    required BoxHitTestWithOutOfBandPosition hitTest,
  }) {
    assert(
      (paintOffset == null && paintTransform == null && rawTransform != null) ||
          (paintOffset == null && paintTransform != null && rawTransform == null) ||
          (paintOffset != null && paintTransform == null && rawTransform == null),
      'Exactly one transform or offset argument must be provided.',
    );
    if (paintOffset != null) {
      pushOffset(-paintOffset);
    } else if (rawTransform != null) {
      pushTransform(rawTransform);
    } else {
      assert(paintTransform != null);
      paintTransform = Matrix4.tryInvert(PointerEvent.removePerspectiveTransform(paintTransform!));
      assert(paintTransform != null, 'paintTransform must be invertible.');
      pushTransform(paintTransform!);
    }
    final bool isHit = hitTest(this);
    popTransform();
    return isHit;
  }
}

/// A hit test entry used by [RenderBox].
class BoxHitTestEntry extends HitTestEntry<RenderBox> {
  /// Creates a box hit test entry.
  BoxHitTestEntry(super.target, this.localPosition);

  /// The position of the hit test in the local coordinates of [target].
  final Offset localPosition;

  @override
  String toString() => '${describeIdentity(target)}@$localPosition';
}

/// Parent data used by [RenderBox] and its subclasses.
///
/// {@tool dartpad}
/// Parent data is used to communicate to a render object about its
/// children. In this example, there are two render objects that perform
/// text layout. They use parent data to identify the kind of child they
/// are laying out, and space the children accordingly.
///
/// ** See code in examples/api/lib/rendering/box/parent_data.0.dart **
/// {@end-tool}
class BoxParentData extends ParentData {
  /// The offset at which to paint the child in the parent's coordinate system.
  Offset offset = Offset.zero;

  @override
  String toString() => 'offset=$offset';
}

/// Abstract [ParentData] subclass for [RenderBox] subclasses that want the
/// [ContainerRenderObjectMixin].
///
/// This is a convenience class that mixes in the relevant classes with
/// the relevant type arguments.
abstract class ContainerBoxParentData<ChildType extends RenderObject> extends BoxParentData
    with ContainerParentDataMixin<ChildType> {}

/// A wrapper that represents the baseline location of a `RenderBox`.
extension type const BaselineOffset(double? offset) {
  /// A value that indicates that the associated `RenderBox` does not have any
  /// baselines.
  ///
  /// [BaselineOffset.noBaseline] is an identity element in most binary
  /// operations involving two [BaselineOffset]s (such as [minOf]), for render
  /// objects with no baselines typically do not contribute to the baseline
  /// offset of their parents.
  static const BaselineOffset noBaseline = BaselineOffset(null);

  /// Returns a new baseline location that is `offset` pixels further away from
  /// the origin than `this`, or unchanged if `this` is [noBaseline].
  BaselineOffset operator +(double offset) {
    final double? value = this.offset;
    return BaselineOffset(value == null ? null : value + offset);
  }

  /// Compares this [BaselineOffset] and `other`, and returns whichever is closer
  /// to the origin.
  ///
  /// When both `this` and `other` are [noBaseline], this method returns
  /// [noBaseline]. When one of them is [noBaseline], this method returns the
  /// other operand that's not [noBaseline].
  BaselineOffset minOf(BaselineOffset other) {
    return switch ((this, other)) {
      (final double lhs?, final double rhs?) => lhs >= rhs ? other : this,
      (final double lhs?, null) => BaselineOffset(lhs),
      (null, final BaselineOffset rhs) => rhs,
    };
  }
}

/// An interface that represents a memoized layout computation run by a [RenderBox].
///
/// Each subclass is inhabited by a single object. Each object represents the
/// signature of a memoized layout computation run by [RenderBox]. For instance,
/// the [dryLayout] object of the [_DryLayout] subclass represents the signature
/// of the [RenderBox.computeDryLayout] method: it takes a [BoxConstraints] (the
/// subclass's `Input` type parameter) and returns a [Size] (the subclass's
/// `Output` type parameter).
///
/// Subclasses do not own their own cache storage. Rather, their [memoize]
/// implementation takes a `cacheStorage`. If a prior computation with the same
/// input values has already been memoized in `cacheStorage`, it returns the
/// memoized value without running `computer`. Otherwise the method runs the
/// `computer` to compute the return value, and caches the result to
/// `cacheStorage`.
///
/// The layout cache storage is typically cleared in `markNeedsLayout`, but is
/// usually kept across [RenderObject.layout] calls because the incoming
/// [BoxConstraints] is always an input of every layout computation.
abstract class _CachedLayoutCalculation<Input extends Object, Output> {
  static const _DryLayout dryLayout = _DryLayout();
  static const _Baseline baseline = _Baseline();

  Output memoize(_LayoutCacheStorage cacheStorage, Input input, Output Function(Input) computer);

  // Debug information that will be used to generate the Timeline event for this type of calculation.
  Map<String, String> debugFillTimelineArguments(
    Map<String, String> timelineArguments,
    Input input,
  );
  String eventLabel(RenderBox renderBox);
}

final class _DryLayout implements _CachedLayoutCalculation<BoxConstraints, Size> {
  const _DryLayout();

  @override
  Size memoize(
    _LayoutCacheStorage cacheStorage,
    BoxConstraints input,
    Size Function(BoxConstraints) computer,
  ) {
    return (cacheStorage._cachedDryLayoutSizes ??= <BoxConstraints, Size>{}).putIfAbsent(
      input,
      () => computer(input),
    );
  }

  @override
  Map<String, String> debugFillTimelineArguments(
    Map<String, String> timelineArguments,
    BoxConstraints input,
  ) {
    return timelineArguments..['getDryLayout constraints'] = '$input';
  }

  @override
  String eventLabel(RenderBox renderBox) => '${renderBox.runtimeType}.getDryLayout';
}

final class _Baseline
    implements _CachedLayoutCalculation<(BoxConstraints, TextBaseline), BaselineOffset> {
  const _Baseline();

  @override
  BaselineOffset memoize(
    _LayoutCacheStorage cacheStorage,
    (BoxConstraints, TextBaseline) input,
    BaselineOffset Function((BoxConstraints, TextBaseline)) computer,
  ) {
    final Map<BoxConstraints, BaselineOffset> cache = switch (input.$2) {
      TextBaseline.alphabetic =>
        cacheStorage._cachedAlphabeticBaseline ??= <BoxConstraints, BaselineOffset>{},
      TextBaseline.ideographic =>
        cacheStorage._cachedIdeoBaseline ??= <BoxConstraints, BaselineOffset>{},
    };
    BaselineOffset ifAbsent() => computer(input);
    return cache.putIfAbsent(input.$1, ifAbsent);
  }

  @override
  Map<String, String> debugFillTimelineArguments(
    Map<String, String> timelineArguments,
    (BoxConstraints, TextBaseline) input,
  ) {
    return timelineArguments
      ..['baseline type'] = '${input.$2}'
      ..['constraints'] = '${input.$1}';
  }

  @override
  String eventLabel(RenderBox renderBox) => '${renderBox.runtimeType}.getDryBaseline';
}

// Intrinsic dimension calculation that computes the intrinsic width given the
// max height, or the intrinsic height given the max width.
enum _IntrinsicDimension implements _CachedLayoutCalculation<double, double> {
  minWidth,
  maxWidth,
  minHeight,
  maxHeight;

  @override
  double memoize(_LayoutCacheStorage cacheStorage, double input, double Function(double) computer) {
    return (cacheStorage._cachedIntrinsicDimensions ??= <(_IntrinsicDimension, double), double>{})
        .putIfAbsent((this, input), () => computer(input));
  }

  @override
  Map<String, String> debugFillTimelineArguments(
    Map<String, String> timelineArguments,
    double input,
  ) {
    return timelineArguments
      ..['intrinsics dimension'] = name
      ..['intrinsics argument'] = '$input';
  }

  @override
  String eventLabel(RenderBox renderBox) => '${renderBox.runtimeType} intrinsics';
}

final class _LayoutCacheStorage {
  Map<(_IntrinsicDimension, double), double>? _cachedIntrinsicDimensions;
  Map<BoxConstraints, Size>? _cachedDryLayoutSizes;
  Map<BoxConstraints, BaselineOffset>? _cachedAlphabeticBaseline;
  Map<BoxConstraints, BaselineOffset>? _cachedIdeoBaseline;

  // Returns a boolean indicating whether the cache storage has cached
  // intrinsics / dry layout data in it.
  bool clear() {
    final bool hasCache =
        (_cachedDryLayoutSizes?.isNotEmpty ?? false) ||
        (_cachedIntrinsicDimensions?.isNotEmpty ?? false) ||
        (_cachedAlphabeticBaseline?.isNotEmpty ?? false) ||
        (_cachedIdeoBaseline?.isNotEmpty ?? false);

    if (hasCache) {
      _cachedDryLayoutSizes?.clear();
      _cachedIntrinsicDimensions?.clear();
      _cachedAlphabeticBaseline?.clear();
      _cachedIdeoBaseline?.clear();
    }
    return hasCache;
  }
}

/// A render object in a 2D Cartesian coordinate system.
///
/// The [size] of each box is expressed as a width and a height. Each box has
/// its own coordinate system in which its upper left corner is placed at (0,
/// 0). The lower right corner of the box is therefore at (width, height). The
/// box contains all the points including the upper left corner and extending
/// to, but not including, the lower right corner.
///
/// Box layout is performed by passing a [BoxConstraints] object down the tree.
/// The box constraints establish a min and max value for the child's width and
/// height. In determining its size, the child must respect the constraints
/// given to it by its parent.
///
/// This protocol is sufficient for expressing a number of common box layout
/// data flows. For example, to implement a width-in-height-out data flow, call
/// your child's [layout] function with a set of box constraints with a tight
/// width value (and pass true for parentUsesSize). After the child determines
/// its height, use the child's height to determine your size.
///
/// ## Writing a RenderBox subclass
///
/// One would implement a new [RenderBox] subclass to describe a new layout
/// model, new paint model, new hit-testing model, or new semantics model, while
/// remaining in the Cartesian space defined by the [RenderBox] protocol.
///
/// To create a new protocol, consider subclassing [RenderObject] instead.
///
/// ### Constructors and properties of a new RenderBox subclass
///
/// The constructor will typically take a named argument for each property of
/// the class. The value is then passed to a private field of the class and the
/// constructor asserts its correctness (e.g. if it should not be null, it
/// asserts it's not null).
///
/// Properties have the form of a getter/setter/field group like the following:
///
/// ```dart
/// AxisDirection get axis => _axis;
/// AxisDirection _axis = AxisDirection.down; // or initialized in constructor
/// set axis(AxisDirection value) {
///   if (value == _axis) {
///     return;
///   }
///   _axis = value;
///   markNeedsLayout();
/// }
/// ```
///
/// The setter will typically finish with either a call to [markNeedsLayout], if
/// the layout uses this property, or [markNeedsPaint], if only the painter
/// function does. (No need to call both, [markNeedsLayout] implies
/// [markNeedsPaint].)
///
/// Consider layout and paint to be expensive; be conservative about calling
/// [markNeedsLayout] or [markNeedsPaint]. They should only be called if the
/// layout (or paint, respectively) has actually changed.
///
/// ### Children
///
/// If a render object is a leaf, that is, it cannot have any children, then
/// ignore this section. (Examples of leaf render objects are [RenderImage] and
/// [RenderParagraph].)
///
/// For render objects with children, there are four possible scenarios:
///
/// * A single [RenderBox] child. In this scenario, consider inheriting from
///   [RenderProxyBox] (if the render object sizes itself to match the child) or
///   [RenderShiftedBox] (if the child will be smaller than the box and the box
///   will align the child inside itself).
///
/// * A single child, but it isn't a [RenderBox]. Use the
///   [RenderObjectWithChildMixin] mixin.
///
/// * A single list of children. Use the [ContainerRenderObjectMixin] mixin.
///
/// * A more complicated child model.
///
/// #### Using RenderProxyBox
///
/// By default, a [RenderProxyBox] render object sizes itself to fit its child, or
/// to be as small as possible if there is no child; it passes all hit testing
/// and painting on to the child, and intrinsic dimensions and baseline
/// measurements similarly are proxied to the child.
///
/// A subclass of [RenderProxyBox] just needs to override the parts of the
/// [RenderBox] protocol that matter. For example, [RenderOpacity] just
/// overrides the paint method (and [alwaysNeedsCompositing] to reflect what the
/// paint method does, and the [visitChildrenForSemantics] method so that the
/// child is hidden from accessibility tools when it's invisible), and adds an
/// [RenderOpacity.opacity] field.
///
/// [RenderProxyBox] assumes that the child is the size of the parent and
/// positioned at 0,0. If this is not true, then use [RenderShiftedBox] instead.
///
/// See
/// [proxy_box.dart](https://github.com/flutter/flutter/blob/main/packages/flutter/lib/src/rendering/proxy_box.dart)
/// for examples of inheriting from [RenderProxyBox].
///
/// #### Using RenderShiftedBox
///
/// By default, a [RenderShiftedBox] acts much like a [RenderProxyBox] but
/// without assuming that the child is positioned at 0,0 (the actual position
/// recorded in the child's [parentData] field is used), and without providing a
/// default layout algorithm.
///
/// See
/// [shifted_box.dart](https://github.com/flutter/flutter/blob/main/packages/flutter/lib/src/rendering/shifted_box.dart)
/// for examples of inheriting from [RenderShiftedBox].
///
/// #### Kinds of children and child-specific data
///
/// A [RenderBox] doesn't have to have [RenderBox] children. One can use another
/// subclass of [RenderObject] for a [RenderBox]'s children. See the discussion
/// at [RenderObject].
///
/// Children can have additional data owned by the parent but stored on the
/// child using the [parentData] field. The class used for that data must
/// inherit from [ParentData]. The [setupParentData] method is used to
/// initialize the [parentData] field of a child when the child is attached.
///
/// By convention, [RenderBox] objects that have [RenderBox] children use the
/// [BoxParentData] class, which has a [BoxParentData.offset] field to store the
/// position of the child relative to the parent. ([RenderProxyBox] does not
/// need this offset and therefore is an exception to this rule.)
///
/// #### Using RenderObjectWithChildMixin
///
/// If a render object has a single child but it isn't a [RenderBox], then the
/// [RenderObjectWithChildMixin] class, which is a mixin that will handle the
/// boilerplate of managing a child, will be useful.
///
/// It's a generic class with one type argument, the type of the child. For
/// example, if you are building a `RenderFoo` class which takes a single
/// `RenderBar` child, you would use the mixin as follows:
///
/// ```dart
/// class RenderFoo extends RenderBox
///   with RenderObjectWithChildMixin<RenderBar> {
///   // ...
/// }
/// ```
///
/// Since the `RenderFoo` class itself is still a [RenderBox] in this case, you
/// still have to implement the [RenderBox] layout algorithm, as well as
/// features like intrinsics and baselines, painting, and hit testing.
///
/// #### Using ContainerRenderObjectMixin
///
/// If a render box can have multiple children, then the
/// [ContainerRenderObjectMixin] mixin can be used to handle the boilerplate. It
/// uses a linked list to model the children in a manner that is easy to mutate
/// dynamically and that can be walked efficiently. Random access is not
/// efficient in this model; if you need random access to the children consider
/// the next section on more complicated child models.
///
/// The [ContainerRenderObjectMixin] class has two type arguments. The first is
/// the type of the child objects. The second is the type for their
/// [parentData]. The class used for [parentData] must itself have the
/// [ContainerParentDataMixin] class mixed into it; this is where
/// [ContainerRenderObjectMixin] stores the linked list. A [ParentData] class
/// can extend [ContainerBoxParentData]; this is essentially
/// [BoxParentData] mixed with [ContainerParentDataMixin]. For example, if a
/// `RenderFoo` class wanted to have a linked list of [RenderBox] children, one
/// might create a `FooParentData` class as follows:
///
/// ```dart
/// class FooParentData extends ContainerBoxParentData<RenderBox> {
///   // (any fields you might need for these children)
/// }
/// ```
///
/// When using [ContainerRenderObjectMixin] in a [RenderBox], consider mixing in
/// [RenderBoxContainerDefaultsMixin], which provides a collection of utility
/// methods that implement common parts of the [RenderBox] protocol (such as
/// painting the children).
///
/// The declaration of the `RenderFoo` class itself would thus look like this:
///
/// ```dart
/// // continuing from previous example...
/// class RenderFoo extends RenderBox with
///   ContainerRenderObjectMixin<RenderBox, FooParentData>,
///   RenderBoxContainerDefaultsMixin<RenderBox, FooParentData> {
///   // ...
/// }
/// ```
///
/// When walking the children (e.g. during layout), the following pattern is
/// commonly used (in this case assuming that the children are all [RenderBox]
/// objects and that this render object uses `FooParentData` objects for its
/// children's [parentData] fields):
///
/// ```dart
/// // continuing from previous example...
/// RenderBox? child = firstChild;
/// while (child != null) {
///   final FooParentData childParentData = child.parentData! as FooParentData;
///   // ...operate on child and childParentData...
///   assert(child.parentData == childParentData);
///   child = childParentData.nextSibling;
/// }
/// ```
///
/// #### More complicated child models
///
/// Render objects can have more complicated models, for example a map of
/// children keyed on an enum, or a 2D grid of efficiently randomly-accessible
/// children, or multiple lists of children, etc. If a render object has a model
/// that can't be handled by the mixins above, it must implement the
/// [RenderObject] child protocol, as follows:
///
/// * Any time a child is removed, call [dropChild] with the child.
///
/// * Any time a child is added, call [adoptChild] with the child.
///
/// * Implement the [attach] method such that it calls [attach] on each child.
///
/// * Implement the [detach] method such that it calls [detach] on each child.
///
/// * Implement the [redepthChildren] method such that it calls [redepthChild]
///   on each child.
///
/// * Implement the [visitChildren] method such that it calls its argument for
///   each child, typically in paint order (back-most to front-most).
///
/// * Implement [debugDescribeChildren] such that it outputs a [DiagnosticsNode]
///   for each child.
///
/// Implementing these seven bullet points is essentially all that the two
/// aforementioned mixins do.
///
/// ### Layout
///
/// [RenderBox] classes implement a layout algorithm. They have a set of
/// constraints provided to them, and they size themselves based on those
/// constraints and whatever other inputs they may have (for example, their
/// children or properties).
///
/// When implementing a [RenderBox] subclass, one must make a choice. Does it
/// size itself exclusively based on the constraints, or does it use any other
/// information in sizing itself? An example of sizing purely based on the
/// constraints would be growing to fit the parent.
///
/// Sizing purely based on the constraints allows the system to make some
/// significant optimizations. Classes that use this approach should override
/// [sizedByParent] to return true, and then override [computeDryLayout] to
/// compute the [Size] using nothing but the constraints, e.g.:
///
/// ```dart
/// @override
/// bool get sizedByParent => true;
///
/// @override
/// Size computeDryLayout(BoxConstraints constraints) {
///   return constraints.smallest;
/// }
/// ```
///
/// Otherwise, the size is set in the [performLayout] function.
///
/// The [performLayout] function is where render boxes decide, if they are not
/// [sizedByParent], what [size] they should be, and also where they decide
/// where their children should be.
///
/// #### Layout of RenderBox children
///
/// The [performLayout] function should call the [layout] function of each (box)
/// child, passing it a [BoxConstraints] object describing the constraints
/// within which the child can render. Passing tight constraints (see
/// [BoxConstraints.isTight]) to the child will allow the rendering library to
/// apply some optimizations, as it knows that if the constraints are tight, the
/// child's dimensions cannot change even if the layout of the child itself
/// changes.
///
/// If the [performLayout] function will use the child's size to affect other
/// aspects of the layout, for example if the render box sizes itself around the
/// child, or positions several children based on the size of those children,
/// then it must specify the `parentUsesSize` argument to the child's [layout]
/// function, setting it to true.
///
/// This flag turns off some optimizations; algorithms that do not rely on the
/// children's sizes will be more efficient. (In particular, relying on the
/// child's [size] means that if the child is marked dirty for layout, the
/// parent will probably also be marked dirty for layout, unless the
/// [constraints] given by the parent to the child were tight constraints.)
///
/// For [RenderBox] classes that do not inherit from [RenderProxyBox], once they
/// have laid out their children, they should also position them, by setting the
/// [BoxParentData.offset] field of each child's [parentData] object.
///
/// #### Layout of non-RenderBox children
///
/// The children of a [RenderBox] do not have to be [RenderBox]es themselves. If
/// they use another protocol (as discussed at [RenderObject]), then instead of
/// [BoxConstraints], the parent would pass in the appropriate [Constraints]
/// subclass, and instead of reading the child's size, the parent would read
/// whatever the output of [layout] is for that layout protocol. The
/// `parentUsesSize` flag is still used to indicate whether the parent is going
/// to read that output, and optimizations still kick in if the child has tight
/// constraints (as defined by [Constraints.isTight]).
///
/// ### Painting
///
/// To describe how a render box paints, implement the [paint] method. It is
/// given a [PaintingContext] object and an [Offset]. The painting context
/// provides methods to affect the layer tree as well as a
/// [PaintingContext.canvas] which can be used to add drawing commands. The
/// canvas object should not be cached across calls to the [PaintingContext]'s
/// methods; every time a method on [PaintingContext] is called, there is a
/// chance that the canvas will change identity. The offset specifies the
/// position of the top left corner of the box in the coordinate system of the
/// [PaintingContext.canvas].
///
/// To draw text on a canvas, use a [TextPainter].
///
/// To draw an image to a canvas, use the [paintImage] method.
///
/// A [RenderBox] that uses methods on [PaintingContext] that introduce new
/// layers should override the [alwaysNeedsCompositing] getter and set it to
/// true. If the object sometimes does and sometimes does not, it can have that
/// getter return true in some cases and false in others. In that case, whenever
/// the return value would change, call [markNeedsCompositingBitsUpdate]. (This
/// is done automatically when a child is added or removed, so you don't have to
/// call it explicitly if the [alwaysNeedsCompositing] getter only changes value
/// based on the presence or absence of children.)
///
/// Anytime anything changes on the object that would cause the [paint] method
/// to paint something different (but would not cause the layout to change),
/// the object should call [markNeedsPaint].
///
/// #### Painting children
///
/// The [paint] method's `context` argument has a [PaintingContext.paintChild]
/// method, which should be called for each child that is to be painted. It
/// should be given a reference to the child, and an [Offset] giving the
/// position of the child relative to the parent.
///
/// If the [paint] method applies a transform to the painting context before
/// painting children (or generally applies an additional offset beyond the
/// offset it was itself given as an argument), then the [applyPaintTransform]
/// method should also be overridden. That method must adjust the matrix that it
/// is given in the same manner as it transformed the painting context and
/// offset before painting the given child. This is used by the [globalToLocal]
/// and [localToGlobal] methods.
///
/// #### Hit Tests
///
/// Hit testing for render boxes is implemented by the [hitTest] method. The
/// default implementation of this method defers to [hitTestSelf] and
/// [hitTestChildren]. When implementing hit testing, you can either override
/// these latter two methods, or ignore them and just override [hitTest].
///
/// The [hitTest] method itself is given an [Offset], and must return true if the
/// object or one of its children has absorbed the hit (preventing objects below
/// this one from being hit), or false if the hit can continue to other objects
/// below this one.
///
/// For each child [RenderBox], the [hitTest] method on the child should be
/// called with the same [HitTestResult] argument and with the point transformed
/// into the child's coordinate space (in the same manner that the
/// [applyPaintTransform] method would). The default implementation defers to
/// [hitTestChildren] to call the children. [RenderBoxContainerDefaultsMixin]
/// provides a [RenderBoxContainerDefaultsMixin.defaultHitTestChildren] method
/// that does this assuming that the children are axis-aligned, not transformed,
/// and positioned according to the [BoxParentData.offset] field of the
/// [parentData]; more elaborate boxes can override [hitTestChildren]
/// accordingly.
///
/// If the object is hit, then it should also add itself to the [HitTestResult]
/// object that is given as an argument to the [hitTest] method, using
/// [HitTestResult.add]. The default implementation defers to [hitTestSelf] to
/// determine if the box is hit. If the object adds itself before the children
/// can add themselves, then it will be as if the object was above the children.
/// If it adds itself after the children, then it will be as if it was below the
/// children. Entries added to the [HitTestResult] object should use the
/// [BoxHitTestEntry] class. The entries are subsequently walked by the system
/// in the order they were added, and for each entry, the target's [handleEvent]
/// method is called, passing in the [HitTestEntry] object.
///
/// Hit testing cannot rely on painting having happened.
///
/// ### Semantics
///
/// For a render box to be accessible, implement the
/// [describeApproximatePaintClip], [visitChildrenForSemantics], and
/// [describeSemanticsConfiguration] methods. The default implementations are
/// sufficient for objects that only affect layout, but nodes that represent
/// interactive components or information (diagrams, text, images, etc) should
/// provide more complete implementations. For more information, see the
/// documentation for these members.
///
/// ### Intrinsics and Baselines
///
/// The layout, painting, hit testing, and semantics protocols are common to all
/// render objects. [RenderBox] objects must implement two additional protocols:
/// intrinsic sizing and baseline measurements.
///
/// There are four methods to implement for intrinsic sizing, to compute the
/// minimum and maximum intrinsic width and height of the box. The documentation
/// for these methods discusses the protocol in detail:
/// [computeMinIntrinsicWidth], [computeMaxIntrinsicWidth],
/// [computeMinIntrinsicHeight], [computeMaxIntrinsicHeight].
///
/// Be sure to set [debugCheckIntrinsicSizes] to true in your unit tests if you
/// do override any of these methods, which will add additional checks to
/// help validate your implementation.
///
/// In addition, if the box has any children, it must implement
/// [computeDistanceToActualBaseline]. [RenderProxyBox] provides a simple
/// implementation that forwards to the child; [RenderShiftedBox] provides an
/// implementation that offsets the child's baseline information by the position
/// of the child relative to the parent. If you do not inherited from either of
/// these classes, however, you must implement the algorithm yourself.
abstract class RenderBox extends RenderObject {
  @override
  void setupParentData(covariant RenderObject child) {
    if (child.parentData is! BoxParentData) {
      child.parentData = BoxParentData();
    }
  }

  final _LayoutCacheStorage _layoutCacheStorage = _LayoutCacheStorage();

  static int _debugIntrinsicsDepth = 0;
  Output _computeIntrinsics<Input extends Object, Output>(
    _CachedLayoutCalculation<Input, Output> type,
    Input input,
    Output Function(Input) computer,
  ) {
    assert(
      RenderObject.debugCheckingIntrinsics || !debugDoingThisResize,
    ); // performResize should not depend on anything except the incoming constraints
    bool shouldCache = true;
    assert(() {
      // we don't want the debug-mode intrinsic tests to affect
      // who gets marked dirty, etc.
      shouldCache = !RenderObject.debugCheckingIntrinsics;
      return true;
    }());
    return shouldCache ? _computeWithTimeline(type, input, computer) : computer(input);
  }

  Output _computeWithTimeline<Input extends Object, Output>(
    _CachedLayoutCalculation<Input, Output> type,
    Input input,
    Output Function(Input) computer,
  ) {
    Map<String, String>? debugTimelineArguments;
    assert(() {
      final Map<String, String> arguments =
          debugEnhanceLayoutTimelineArguments
              ? toDiagnosticsNode().toTimelineArguments()!
              : <String, String>{};
      debugTimelineArguments = type.debugFillTimelineArguments(arguments, input);
      return true;
    }());
    if (!kReleaseMode) {
      if (debugProfileLayoutsEnabled || _debugIntrinsicsDepth == 0) {
        FlutterTimeline.startSync(type.eventLabel(this), arguments: debugTimelineArguments);
      }
      _debugIntrinsicsDepth += 1;
    }
    final Output result = type.memoize(_layoutCacheStorage, input, computer);
    if (!kReleaseMode) {
      _debugIntrinsicsDepth -= 1;
      if (debugProfileLayoutsEnabled || _debugIntrinsicsDepth == 0) {
        FlutterTimeline.finishSync();
      }
    }
    return result;
  }

  /// Returns the minimum width that this box could be without failing to
  /// correctly paint its contents within itself, without clipping.
  ///
  /// The height argument may give a specific height to assume. The given height
  /// can be infinite, meaning that the intrinsic width in an unconstrained
  /// environment is being requested. The given height should never be negative
  /// or null.
  ///
  /// This function should only be called on one's children. Calling this
  /// function couples the child with the parent so that when the child's layout
  /// changes, the parent is notified (via [markNeedsLayout]).
  ///
  /// Calling this function is expensive as it can result in O(N^2) behavior.
  ///
  /// Do not override this method. Instead, implement [computeMinIntrinsicWidth].
  @mustCallSuper
  double getMinIntrinsicWidth(double height) {
    assert(() {
      if (height < 0.0) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('The height argument to getMinIntrinsicWidth was negative.'),
          ErrorDescription('The argument to getMinIntrinsicWidth must not be negative or null.'),
          ErrorHint(
            'If you perform computations on another height before passing it to '
            'getMinIntrinsicWidth, consider using math.max() or double.clamp() '
            'to force the value into the valid range.',
          ),
        ]);
      }
      return true;
    }());
    return _computeIntrinsics(_IntrinsicDimension.minWidth, height, computeMinIntrinsicWidth);
  }

  /// Computes the value returned by [getMinIntrinsicWidth]. Do not call this
  /// function directly, instead, call [getMinIntrinsicWidth].
  ///
  /// Override in subclasses that implement [performLayout]. This method should
  /// return the minimum width that this box could be without failing to
  /// correctly paint its contents within itself, without clipping.
  ///
  /// If the layout algorithm is independent of the context (e.g. it always
  /// tries to be a particular size), or if the layout algorithm is
  /// width-in-height-out, or if the layout algorithm uses both the incoming
  /// width and height constraints (e.g. it always sizes itself to
  /// [BoxConstraints.biggest]), then the `height` argument should be ignored.
  ///
  /// If the layout algorithm is strictly height-in-width-out, or is
  /// height-in-width-out when the width is unconstrained, then the height
  /// argument is the height to use.
  ///
  /// The `height` argument will never be negative or null. It may be infinite.
  ///
  /// If this algorithm depends on the intrinsic dimensions of a child, the
  /// intrinsic dimensions of that child should be obtained using the functions
  /// whose names start with `get`, not `compute`.
  ///
  /// This function should never return a negative or infinite value.
  ///
  /// Be sure to set [debugCheckIntrinsicSizes] to true in your unit tests if
  /// you do override this method, which will add additional checks to help
  /// validate your implementation.
  ///
  /// ## Examples
  ///
  /// ### Text
  ///
  /// English text is the canonical example of a width-in-height-out algorithm.
  /// The `height` argument is therefore ignored.
  ///
  /// Consider the string "Hello World". The _maximum_ intrinsic width (as
  /// returned from [computeMaxIntrinsicWidth]) would be the width of the string
  /// with no line breaks.
  ///
  /// The minimum intrinsic width would be the width of the widest word, "Hello"
  /// or "World". If the text is rendered in an even narrower width, however, it
  /// might still not overflow. For example, maybe the rendering would put a
  /// line-break half-way through the words, as in "Hel⁞lo⁞Wor⁞ld". However,
  /// this wouldn't be a _correct_ rendering, and [computeMinIntrinsicWidth] is
  /// defined as returning the minimum width that the box could be without
  /// failing to _correctly_ paint the contents within itself.
  ///
  /// The minimum intrinsic _height_ for a given width _smaller_ than the
  /// minimum intrinsic width could therefore be greater than the minimum
  /// intrinsic height for the minimum intrinsic width.
  ///
  /// ### Viewports (e.g. scrolling lists)
  ///
  /// Some render boxes are intended to clip their children. For example, the
  /// render box for a scrolling list might always size itself to its parents'
  /// size (or rather, to the maximum incoming constraints), regardless of the
  /// children's sizes, and then clip the children and position them based on
  /// the current scroll offset.
  ///
  /// The intrinsic dimensions in these cases still depend on the children, even
  /// though the layout algorithm sizes the box in a way independent of the
  /// children. It is the size that is needed to paint the box's contents (in
  /// this case, the children) _without clipping_ that matters.
  ///
  /// ### When the intrinsic dimensions cannot be known
  ///
  /// There are cases where render objects do not have an efficient way to
  /// compute their intrinsic dimensions. For example, it may be prohibitively
  /// expensive to reify and measure every child of a lazy viewport (viewports
  /// generally only instantiate the actually visible children), or the
  /// dimensions may be computed by a callback about which the render object
  /// cannot reason.
  ///
  /// In such cases, it may be impossible (or at least impractical) to actually
  /// return a valid answer. In such cases, the intrinsic functions should throw
  /// when [RenderObject.debugCheckingIntrinsics] is false and asserts are
  /// enabled, and return 0.0 otherwise.
  ///
  /// See the implementations of [LayoutBuilder] or [RenderViewportBase] for
  /// examples (in particular,
  /// [RenderViewportBase.debugThrowIfNotCheckingIntrinsics]).
  ///
  /// ### Aspect-ratio-driven boxes
  ///
  /// Some boxes always return a fixed size based on the constraints. For these
  /// boxes, the intrinsic functions should return the appropriate size when the
  /// incoming `height` or `width` argument is finite, treating that as a tight
  /// constraint in the respective direction and treating the other direction's
  /// constraints as unbounded. This is because the definitions of
  /// [computeMinIntrinsicWidth] and [computeMinIntrinsicHeight] are in terms of
  /// what the dimensions _could be_, and such boxes can only be one size in
  /// such cases.
  ///
  /// When the incoming argument is not finite, then they should return the
  /// actual intrinsic dimensions based on the contents, as any other box would.
  ///
  /// See also:
  ///
  ///  * [computeMaxIntrinsicWidth], which computes the smallest width beyond
  ///    which increasing the width never decreases the preferred height.
  @protected
  double computeMinIntrinsicWidth(double height) {
    return 0.0;
  }

  /// Returns the smallest width beyond which increasing the width never
  /// decreases the preferred height. The preferred height is the value that
  /// would be returned by [getMinIntrinsicHeight] for that width.
  ///
  /// The height argument may give a specific height to assume. The given height
  /// can be infinite, meaning that the intrinsic width in an unconstrained
  /// environment is being requested. The given height should never be negative
  /// or null.
  ///
  /// This function should only be called on one's children. Calling this
  /// function couples the child with the parent so that when the child's layout
  /// changes, the parent is notified (via [markNeedsLayout]).
  ///
  /// Calling this function is expensive as it can result in O(N^2) behavior.
  ///
  /// Do not override this method. Instead, implement
  /// [computeMaxIntrinsicWidth].
  @mustCallSuper
  double getMaxIntrinsicWidth(double height) {
    assert(() {
      if (height < 0.0) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('The height argument to getMaxIntrinsicWidth was negative.'),
          ErrorDescription('The argument to getMaxIntrinsicWidth must not be negative or null.'),
          ErrorHint(
            'If you perform computations on another height before passing it to '
            'getMaxIntrinsicWidth, consider using math.max() or double.clamp() '
            'to force the value into the valid range.',
          ),
        ]);
      }
      return true;
    }());
    return _computeIntrinsics(_IntrinsicDimension.maxWidth, height, computeMaxIntrinsicWidth);
  }

  /// Computes the value returned by [getMaxIntrinsicWidth]. Do not call this
  /// function directly, instead, call [getMaxIntrinsicWidth].
  ///
  /// Override in subclasses that implement [performLayout]. This should return
  /// the smallest width beyond which increasing the width never decreases the
  /// preferred height. The preferred height is the value that would be returned
  /// by [computeMinIntrinsicHeight] for that width.
  ///
  /// If the layout algorithm is strictly height-in-width-out, or is
  /// height-in-width-out when the width is unconstrained, then this should
  /// return the same value as [computeMinIntrinsicWidth] for the same height.
  ///
  /// Otherwise, the height argument should be ignored, and the returned value
  /// should be equal to or bigger than the value returned by
  /// [computeMinIntrinsicWidth].
  ///
  /// The `height` argument will never be negative or null. It may be infinite.
  ///
  /// The value returned by this method might not match the size that the object
  /// would actually take. For example, a [RenderBox] subclass that always
  /// exactly sizes itself using [BoxConstraints.biggest] might well size itself
  /// bigger than its max intrinsic size.
  ///
  /// If this algorithm depends on the intrinsic dimensions of a child, the
  /// intrinsic dimensions of that child should be obtained using the functions
  /// whose names start with `get`, not `compute`.
  ///
  /// This function should never return a negative or infinite value.
  ///
  /// Be sure to set [debugCheckIntrinsicSizes] to true in your unit tests if
  /// you do override this method, which will add additional checks to help
  /// validate your implementation.
  ///
  /// See also:
  ///
  ///  * [computeMinIntrinsicWidth], which has usage examples.
  @visibleForOverriding
  @protected
  double computeMaxIntrinsicWidth(double height) {
    return 0.0;
  }

  /// Returns the minimum height that this box could be without failing to
  /// correctly paint its contents within itself, without clipping.
  ///
  /// The width argument may give a specific width to assume. The given width
  /// can be infinite, meaning that the intrinsic height in an unconstrained
  /// environment is being requested. The given width should never be negative
  /// or null.
  ///
  /// This function should only be called on one's children. Calling this
  /// function couples the child with the parent so that when the child's layout
  /// changes, the parent is notified (via [markNeedsLayout]).
  ///
  /// Calling this function is expensive as it can result in O(N^2) behavior.
  ///
  /// Do not override this method. Instead, implement
  /// [computeMinIntrinsicHeight].
  @mustCallSuper
  double getMinIntrinsicHeight(double width) {
    assert(() {
      if (width < 0.0) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('The width argument to getMinIntrinsicHeight was negative.'),
          ErrorDescription('The argument to getMinIntrinsicHeight must not be negative or null.'),
          ErrorHint(
            'If you perform computations on another width before passing it to '
            'getMinIntrinsicHeight, consider using math.max() or double.clamp() '
            'to force the value into the valid range.',
          ),
        ]);
      }
      return true;
    }());
    return _computeIntrinsics(_IntrinsicDimension.minHeight, width, computeMinIntrinsicHeight);
  }

  /// Computes the value returned by [getMinIntrinsicHeight]. Do not call this
  /// function directly, instead, call [getMinIntrinsicHeight].
  ///
  /// Override in subclasses that implement [performLayout]. Should return the
  /// minimum height that this box could be without failing to correctly paint
  /// its contents within itself, without clipping.
  ///
  /// If the layout algorithm is independent of the context (e.g. it always
  /// tries to be a particular size), or if the layout algorithm is
  /// height-in-width-out, or if the layout algorithm uses both the incoming
  /// height and width constraints (e.g. it always sizes itself to
  /// [BoxConstraints.biggest]), then the `width` argument should be ignored.
  ///
  /// If the layout algorithm is strictly width-in-height-out, or is
  /// width-in-height-out when the height is unconstrained, then the width
  /// argument is the width to use.
  ///
  /// The `width` argument will never be negative or null. It may be infinite.
  ///
  /// If this algorithm depends on the intrinsic dimensions of a child, the
  /// intrinsic dimensions of that child should be obtained using the functions
  /// whose names start with `get`, not `compute`.
  ///
  /// This function should never return a negative or infinite value.
  ///
  /// Be sure to set [debugCheckIntrinsicSizes] to true in your unit tests if
  /// you do override this method, which will add additional checks to help
  /// validate your implementation.
  ///
  /// See also:
  ///
  ///  * [computeMinIntrinsicWidth], which has usage examples.
  ///  * [computeMaxIntrinsicHeight], which computes the smallest height beyond
  ///    which increasing the height never decreases the preferred width.
  @visibleForOverriding
  @protected
  double computeMinIntrinsicHeight(double width) {
    return 0.0;
  }

  /// Returns the smallest height beyond which increasing the height never
  /// decreases the preferred width. The preferred width is the value that
  /// would be returned by [getMinIntrinsicWidth] for that height.
  ///
  /// The width argument may give a specific width to assume. The given width
  /// can be infinite, meaning that the intrinsic height in an unconstrained
  /// environment is being requested. The given width should never be negative
  /// or null.
  ///
  /// This function should only be called on one's children. Calling this
  /// function couples the child with the parent so that when the child's layout
  /// changes, the parent is notified (via [markNeedsLayout]).
  ///
  /// Calling this function is expensive as it can result in O(N^2) behavior.
  ///
  /// Do not override this method. Instead, implement
  /// [computeMaxIntrinsicHeight].
  @mustCallSuper
  double getMaxIntrinsicHeight(double width) {
    assert(() {
      if (width < 0.0) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('The width argument to getMaxIntrinsicHeight was negative.'),
          ErrorDescription('The argument to getMaxIntrinsicHeight must not be negative or null.'),
          ErrorHint(
            'If you perform computations on another width before passing it to '
            'getMaxIntrinsicHeight, consider using math.max() or double.clamp() '
            'to force the value into the valid range.',
          ),
        ]);
      }
      return true;
    }());
    return _computeIntrinsics(_IntrinsicDimension.maxHeight, width, computeMaxIntrinsicHeight);
  }

  /// Computes the value returned by [getMaxIntrinsicHeight]. Do not call this
  /// function directly, instead, call [getMaxIntrinsicHeight].
  ///
  /// Override in subclasses that implement [performLayout]. Should return the
  /// smallest height beyond which increasing the height never decreases the
  /// preferred width. The preferred width is the value that would be returned
  /// by [computeMinIntrinsicWidth] for that height.
  ///
  /// If the layout algorithm is strictly width-in-height-out, or is
  /// width-in-height-out when the height is unconstrained, then this should
  /// return the same value as [computeMinIntrinsicHeight] for the same width.
  ///
  /// Otherwise, the width argument should be ignored, and the returned value
  /// should be equal to or bigger than the value returned by
  /// [computeMinIntrinsicHeight].
  ///
  /// The `width` argument will never be negative or null. It may be infinite.
  ///
  /// The value returned by this method might not match the size that the object
  /// would actually take. For example, a [RenderBox] subclass that always
  /// exactly sizes itself using [BoxConstraints.biggest] might well size itself
  /// bigger than its max intrinsic size.
  ///
  /// If this algorithm depends on the intrinsic dimensions of a child, the
  /// intrinsic dimensions of that child should be obtained using the functions
  /// whose names start with `get`, not `compute`.
  ///
  /// This function should never return a negative or infinite value.
  ///
  /// Be sure to set [debugCheckIntrinsicSizes] to true in your unit tests if
  /// you do override this method, which will add additional checks to help
  /// validate your implementation.
  ///
  /// See also:
  ///
  ///  * [computeMinIntrinsicWidth], which has usage examples.
  @visibleForOverriding
  @protected
  double computeMaxIntrinsicHeight(double width) {
    return 0.0;
  }

  /// Returns the [Size] that this [RenderBox] would like to be given the
  /// provided [BoxConstraints].
  ///
  /// The size returned by this method is guaranteed to be the same size that
  /// this [RenderBox] computes for itself during layout given the same
  /// constraints.
  ///
  /// This function should only be called on one's children. Calling this
  /// function couples the child with the parent so that when the child's layout
  /// changes, the parent is notified (via [markNeedsLayout]).
  ///
  /// This layout is called "dry" layout as opposed to the regular "wet" layout
  /// run performed by [performLayout] because it computes the desired size for
  /// the given constraints without changing any internal state.
  ///
  /// Calling this function is expensive as it can result in O(N^2) behavior.
  ///
  /// Do not override this method. Instead, implement [computeDryLayout].
  @mustCallSuper
  Size getDryLayout(covariant BoxConstraints constraints) {
    return _computeIntrinsics(_CachedLayoutCalculation.dryLayout, constraints, _computeDryLayout);
  }

  bool _computingThisDryLayout = false;
  Size _computeDryLayout(BoxConstraints constraints) {
    assert(() {
      assert(!_computingThisDryLayout);
      _computingThisDryLayout = true;
      return true;
    }());
    final Size result = computeDryLayout(constraints);
    assert(() {
      assert(_computingThisDryLayout);
      _computingThisDryLayout = false;
      return true;
    }());
    return result;
  }

  /// Computes the value returned by [getDryLayout]. Do not call this
  /// function directly, instead, call [getDryLayout].
  ///
  /// Override in subclasses that implement [performLayout] or [performResize]
  /// or when setting [sizedByParent] to true without overriding
  /// [performResize]. This method should return the [Size] that this
  /// [RenderBox] would like to be given the provided [BoxConstraints].
  ///
  /// The size returned by this method must match the [size] that the
  /// [RenderBox] will compute for itself in [performLayout] (or
  /// [performResize], if [sizedByParent] is true).
  ///
  /// If this algorithm depends on the size of a child, the size of that child
  /// should be obtained using its [getDryLayout] method.
  ///
  /// This layout is called "dry" layout as opposed to the regular "wet" layout
  /// run performed by [performLayout] because it computes the desired size for
  /// the given constraints without changing any internal state.
  ///
  /// ### When the size cannot be known
  ///
  /// There are cases where render objects do not have an efficient way to
  /// compute their size. For example, the size may computed by a callback about
  /// which the render object cannot reason.
  ///
  /// In such cases, it may be impossible (or at least impractical) to actually
  /// return a valid answer. In such cases, the function should call
  /// [debugCannotComputeDryLayout] from within an assert and return a dummy
  /// value of `const Size(0, 0)`.
  @visibleForOverriding
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    assert(
      debugCannotComputeDryLayout(
        error: FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary(
            'The ${objectRuntimeType(this, 'RenderBox')} class does not implement "computeDryLayout".',
          ),
          ErrorHint(
            'If you are not writing your own RenderBox subclass, then this is not\n'
            'your fault. Contact support: https://github.com/flutter/flutter/issues/new?template=2_bug.yml',
          ),
        ]),
      ),
    );
    return Size.zero;
  }

  /// Returns the distance from the top of the box to the first baseline of the
  /// box's contents for the given `constraints`, or `null` if this [RenderBox]
  /// does not have any baselines.
  ///
  /// This method calls [computeDryBaseline] under the hood and caches the result.
  /// [RenderBox] subclasses typically don't overridden [getDryBaseline]. Instead,
  /// consider overriding [computeDryBaseline] such that it returns a baseline
  /// location that is consistent with [getDistanceToActualBaseline]. See the
  /// documentation for the [computeDryBaseline] method for more details.
  ///
  /// This method is usually called by the [computeDryBaseline] or the
  /// [computeDryLayout] implementation of a parent [RenderBox] to get the
  /// baseline location of a [RenderBox] child. Unlike [getDistanceToBaseline],
  /// this method takes a [BoxConstraints] as an argument and computes the
  /// baseline location as if the [RenderBox] was laid out by the parent using
  /// that [BoxConstraints].
  ///
  /// The "dry" in the method name means this method, like [getDryLayout], has
  /// no observable side effects when called, as opposed to "wet" layout methods
  /// such as [performLayout] (which changes this [RenderBox]'s [size], and the
  /// offsets of its children if any). Since this method does not depend on the
  /// current layout, unlike [getDistanceToBaseline], it's ok to call this method
  /// when this [RenderBox]'s layout is outdated.
  ///
  /// Similar to the intrinsic width/height and [getDryLayout], calling this
  /// function in [performLayout] is expensive, as it can result in O(N^2) layout
  /// performance, where N is the number of render objects in the render subtree.
  /// Typically this method should be only called by the parent [RenderBox]'s
  /// [computeDryBaseline] or [computeDryLayout] implementation.
  double? getDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    final double? baselineOffset =
        _computeIntrinsics(_CachedLayoutCalculation.baseline, (
          constraints,
          baseline,
        ), _computeDryBaseline).offset;
    // This assert makes sure computeDryBaseline always gets called in debug mode,
    // in case the computeDryBaseline implementation invokes debugCannotComputeDryLayout.
    // This check should be skipped when debugCheckingIntrinsics is true to avoid
    // slowing down the app significantly.
    assert(
      RenderObject.debugCheckingIntrinsics ||
          baselineOffset == computeDryBaseline(constraints, baseline),
    );
    return baselineOffset;
  }

  bool _computingThisDryBaseline = false;
  BaselineOffset _computeDryBaseline((BoxConstraints, TextBaseline) pair) {
    assert(() {
      assert(!_computingThisDryBaseline);
      _computingThisDryBaseline = true;
      return true;
    }());
    final BaselineOffset result = BaselineOffset(computeDryBaseline(pair.$1, pair.$2));
    assert(() {
      assert(_computingThisDryBaseline);
      _computingThisDryBaseline = false;
      return true;
    }());
    return result;
  }

  /// Computes the value returned by [getDryBaseline].
  ///
  /// This method is for overriding only and shouldn't be called directly. To
  /// get this [RenderBox]'s speculative baseline location for the given
  /// `constraints`, call [getDryBaseline] instead.
  ///
  /// The "dry" in the method name means the implementation must not produce
  /// observable side effects when called. For example, it must not change the
  /// [size] of the [RenderBox], or its children's paint offsets, otherwise that
  /// would results in UI changes when [paint] is called, or hit-testing behavior
  /// changes when [hitTest] is called. Moreover, accessing the current layout
  /// of this [RenderBox] or child [RenderBox]es (including accessing [size], or
  /// `child.size`) usually indicates a bug in the implementation, as the current
  /// layout is typically calculated using a set of [BoxConstraints] that's
  /// different from the `constraints` given as the first parameter. To get the
  /// size of this [RenderBox] or a child [RenderBox] in this method's
  /// implementation, use the [getDryLayout] method instead.
  ///
  /// The implementation must return a value that represents the distance from
  /// the top of the box to the first baseline of the box's contents, for the
  /// given `constraints`, or `null` if the [RenderBox] has no baselines. It's
  /// the same exact value [RenderBox.computeDistanceToActualBaseline] would
  /// return, when this [RenderBox] was laid out at `constraints` in the same
  /// exact state.
  ///
  /// Not all [RenderBox]es support dry baseline computation. For example, to
  /// compute the dry baseline of a [LayoutBuilder], its `builder` may have to
  /// be called with different constraints, which may have side effects such as
  /// updating the widget tree, violating the "dry" contract. In such cases the
  /// [RenderBox] must call [debugCannotComputeDryLayout] in an assert, and
  /// return a dummy baseline offset value (such as `null`).
  @visibleForOverriding
  @protected
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    assert(
      debugCannotComputeDryLayout(
        error: FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary(
            'The ${objectRuntimeType(this, 'RenderBox')} class does not implement "computeDryBaseline".',
          ),
          ErrorHint(
            'If you are not writing your own RenderBox subclass, then this is not\n'
            'your fault. Contact support: https://github.com/flutter/flutter/issues/new?template=2_bug.yml',
          ),
        ]),
      ),
    );
    return null;
  }

  static bool _debugDryLayoutCalculationValid = true;

  /// Called from [computeDryLayout] or [computeDryBaseline] within an assert if
  /// the given [RenderBox] subclass does not support calculating a dry layout.
  ///
  /// When asserts are enabled and [debugCheckingIntrinsics] is not true, this
  /// method will either throw the provided [FlutterError] or it will create and
  /// throw a [FlutterError] with the provided `reason`. Otherwise, it will
  /// return true.
  ///
  /// One of the arguments has to be provided.
  ///
  /// See also:
  ///
  ///  * [computeDryLayout], which lists some reasons why it may not be feasible
  ///    to compute the dry layout.
  bool debugCannotComputeDryLayout({String? reason, FlutterError? error}) {
    assert((reason == null) != (error == null));
    assert(() {
      if (!RenderObject.debugCheckingIntrinsics) {
        if (reason != null) {
          assert(error == null);
          throw FlutterError.fromParts(<DiagnosticsNode>[
            ErrorSummary(
              'The ${objectRuntimeType(this, 'RenderBox')} class does not support dry layout.',
            ),
            if (reason.isNotEmpty) ErrorDescription(reason),
          ]);
        }
        assert(error != null);
        throw error!;
      }
      _debugDryLayoutCalculationValid = false;
      return true;
    }());
    return true;
  }

  /// Whether this render object has undergone layout and has a [size].
  bool get hasSize => _size != null;

  /// The size of this render box computed during layout.
  ///
  /// This value is stale whenever this object is marked as needing layout.
  /// During [performLayout], do not read the size of a child unless you pass
  /// true for parentUsesSize when calling the child's [layout] function.
  ///
  /// The size of a box should be set only during the box's [performLayout] or
  /// [performResize] functions. If you wish to change the size of a box outside
  /// of those functions, call [markNeedsLayout] instead to schedule a layout of
  /// the box.
  Size get size {
    assert(hasSize, 'RenderBox was not laid out: $this');
    assert(() {
      final Size? size = _size;
      if (size is _DebugSize) {
        assert(size._owner == this);
        final RenderObject? parent = this.parent;
        // Whether the size getter is accessed during layout (but not in a
        // layout callback).
        final bool doingRegularLayout =
            !(RenderObject.debugActiveLayout?.debugDoingThisLayoutWithCallback ?? true);
        final bool sizeAccessAllowed =
            !doingRegularLayout ||
            debugDoingThisResize ||
            debugDoingThisLayout ||
            _computingThisDryLayout ||
            RenderObject.debugActiveLayout == parent && size._canBeUsedByParent;
        assert(
          sizeAccessAllowed,
          'RenderBox.size accessed beyond the scope of resize, layout, or '
          'permitted parent access. RenderBox can always access its own size, '
          'otherwise, the only object that is allowed to read RenderBox.size '
          'is its parent, if they have said they will. It you hit this assert '
          'trying to access a child\'s size, pass "parentUsesSize: true" to '
          "that child's layout() in ${objectRuntimeType(this, 'RenderBox')}.performLayout.",
        );
        final RenderBox? renderBoxDoingDryBaseline =
            _computingThisDryBaseline
                ? this
                : (parent is RenderBox && parent._computingThisDryBaseline ? parent : null);
        assert(
          renderBoxDoingDryBaseline == null,
          'RenderBox.size accessed in '
          '${objectRuntimeType(renderBoxDoingDryBaseline, 'RenderBox')}.computeDryBaseline.'
          'The computeDryBaseline method must not access '
          '${renderBoxDoingDryBaseline == this ? "the RenderBox's own size" : "the size of its child"},'
          "because it's established in performLayout or performResize using different BoxConstraints.",
        );
        assert(size == _size);
      }
      return true;
    }());
    return _size ??
        (throw StateError('RenderBox was not laid out: $runtimeType#${shortHash(this)}'));
  }

  Size? _size;

  /// Setting the size, in debug mode, triggers some analysis of the render box,
  /// as implemented by [debugAssertDoesMeetConstraints], including calling the intrinsic
  /// sizing methods and checking that they meet certain invariants.
  @protected
  set size(Size value) {
    assert(!(debugDoingThisResize && debugDoingThisLayout));
    assert(sizedByParent || !debugDoingThisResize);
    assert(() {
      if ((sizedByParent && debugDoingThisResize) || (!sizedByParent && debugDoingThisLayout)) {
        return true;
      }
      assert(!debugDoingThisResize);
      final List<DiagnosticsNode> information = <DiagnosticsNode>[
        ErrorSummary('RenderBox size setter called incorrectly.'),
      ];
      if (debugDoingThisLayout) {
        assert(sizedByParent);
        information.add(
          ErrorDescription('It appears that the size setter was called from performLayout().'),
        );
      } else {
        information.add(
          ErrorDescription(
            'The size setter was called from outside layout (neither performResize() nor performLayout() were being run for this object).',
          ),
        );
        if (owner != null && owner!.debugDoingLayout) {
          information.add(
            ErrorDescription(
              'Only the object itself can set its size. It is a contract violation for other objects to set it.',
            ),
          );
        }
      }
      if (sizedByParent) {
        information.add(
          ErrorDescription(
            'Because this RenderBox has sizedByParent set to true, it must set its size in performResize().',
          ),
        );
      } else {
        information.add(
          ErrorDescription(
            'Because this RenderBox has sizedByParent set to false, it must set its size in performLayout().',
          ),
        );
      }
      throw FlutterError.fromParts(information);
    }());
    assert(() {
      value = debugAdoptSize(value);
      return true;
    }());
    _size = value;
    assert(() {
      debugAssertDoesMeetConstraints();
      return true;
    }());
  }

  /// Claims ownership of the given [Size].
  ///
  /// In debug mode, the [RenderBox] class verifies that [Size] objects obtained
  /// from other [RenderBox] objects are only used according to the semantics of
  /// the [RenderBox] protocol, namely that a [Size] from a [RenderBox] can only
  /// be used by its parent, and then only if `parentUsesSize` was set.
  ///
  /// Sometimes, a [Size] that can validly be used ends up no longer being valid
  /// over time. The common example is a [Size] taken from a child that is later
  /// removed from the parent. In such cases, this method can be called to first
  /// check whether the size can legitimately be used, and if so, to then create
  /// a new [Size] that can be used going forward, regardless of what happens to
  /// the original owner.
  Size debugAdoptSize(Size value) {
    Size result = value;
    assert(() {
      if (value is _DebugSize) {
        if (value._owner != this) {
          if (value._owner.parent != this) {
            throw FlutterError.fromParts(<DiagnosticsNode>[
              ErrorSummary('The size property was assigned a size inappropriately.'),
              describeForError('The following render object'),
              value._owner.describeForError('...was assigned a size obtained from'),
              ErrorDescription(
                'However, this second render object is not, or is no longer, a '
                'child of the first, and it is therefore a violation of the '
                'RenderBox layout protocol to use that size in the layout of the '
                'first render object.',
              ),
              ErrorHint(
                'If the size was obtained at a time where it was valid to read '
                'the size (because the second render object above was a child '
                'of the first at the time), then it should be adopted using '
                'debugAdoptSize at that time.',
              ),
              ErrorHint(
                'If the size comes from a grandchild or a render object from an '
                'entirely different part of the render tree, then there is no '
                'way to be notified when the size changes and therefore attempts '
                'to read that size are almost certainly a source of bugs. A different '
                'approach should be used.',
              ),
            ]);
          }
          if (!value._canBeUsedByParent) {
            throw FlutterError.fromParts(<DiagnosticsNode>[
              ErrorSummary("A child's size was used without setting parentUsesSize."),
              describeForError('The following render object'),
              value._owner.describeForError('...was assigned a size obtained from its child'),
              ErrorDescription(
                'However, when the child was laid out, the parentUsesSize argument '
                'was not set or set to false. Subsequently this transpired to be '
                'inaccurate: the size was nonetheless used by the parent.\n'
                'It is important to tell the framework if the size will be used or not '
                'as several important performance optimizations can be made if the '
                'size will not be used by the parent.',
              ),
            ]);
          }
        }
      }
      result = _DebugSize(value, this, debugCanParentUseSize);
      return true;
    }());
    return result;
  }

  @override
  Rect get semanticBounds => Offset.zero & size;

  @override
  void debugResetSize() {
    // updates the value of size._canBeUsedByParent if necessary
    size = size; // ignore: no_self_assignments
  }

  static bool _debugDoingBaseline = false;
  static bool _debugSetDoingBaseline(bool value) {
    _debugDoingBaseline = value;
    return true;
  }

  /// Returns the distance from the y-coordinate of the position of the box to
  /// the y-coordinate of the first given baseline in the box's contents.
  ///
  /// Used by certain layout models to align adjacent boxes on a common
  /// baseline, regardless of padding, font size differences, etc. If there is
  /// no baseline, this function returns the distance from the y-coordinate of
  /// the position of the box to the y-coordinate of the bottom of the box
  /// (i.e., the height of the box) unless the caller passes true
  /// for `onlyReal`, in which case the function returns null.
  ///
  /// Only call this function after calling [layout] on this box. You
  /// are only allowed to call this from the parent of this box during
  /// that parent's [performLayout] or [paint] functions.
  ///
  /// When implementing a [RenderBox] subclass, to override the baseline
  /// computation, override [computeDistanceToActualBaseline].
  ///
  /// See also:
  ///
  ///  * [getDryBaseline], which returns the baseline location of this
  ///    [RenderBox] at a certain [BoxConstraints].
  double? getDistanceToBaseline(TextBaseline baseline, {bool onlyReal = false}) {
    assert(
      !_debugDoingBaseline,
      'Please see the documentation for computeDistanceToActualBaseline for the required calling conventions of this method.',
    );
    assert(!debugNeedsLayout || RenderObject.debugCheckingIntrinsics);
    assert(
      RenderObject.debugCheckingIntrinsics ||
          switch (owner!) {
            PipelineOwner(debugDoingLayout: true) =>
              RenderObject.debugActiveLayout == parent && parent!.debugDoingThisLayout,
            PipelineOwner(debugDoingPaint: true) =>
              RenderObject.debugActivePaint == parent && parent!.debugDoingThisPaint ||
                  (RenderObject.debugActivePaint == this && debugDoingThisPaint),
            PipelineOwner() => false,
          },
    );
    assert(_debugSetDoingBaseline(true));
    final double? result;
    try {
      result = getDistanceToActualBaseline(baseline);
    } finally {
      assert(_debugSetDoingBaseline(false));
    }
    if (result == null && !onlyReal) {
      return size.height;
    }
    return result;
  }

  /// Calls [computeDistanceToActualBaseline] and caches the result.
  ///
  /// This function must only be called from [getDistanceToBaseline] and
  /// [computeDistanceToActualBaseline]. Do not call this function directly from
  /// outside those two methods.
  @protected
  @mustCallSuper
  double? getDistanceToActualBaseline(TextBaseline baseline) {
    assert(
      _debugDoingBaseline,
      'Please see the documentation for computeDistanceToActualBaseline for the required calling conventions of this method.',
    );
    return _computeIntrinsics(
      _CachedLayoutCalculation.baseline,
      (constraints, baseline),
      ((BoxConstraints, TextBaseline) pair) =>
          BaselineOffset(computeDistanceToActualBaseline(pair.$2)),
    ).offset;
  }

  /// Returns the distance from the y-coordinate of the position of the box to
  /// the y-coordinate of the first given baseline in the box's contents, if
  /// any, or null otherwise.
  ///
  /// Do not call this function directly. If you need to know the baseline of a
  /// child from an invocation of [performLayout] or [paint], call
  /// [getDistanceToBaseline].
  ///
  /// Subclasses should override this method to supply the distances to their
  /// baselines. When implementing this method, there are generally three
  /// strategies:
  ///
  ///  * For classes that use the [ContainerRenderObjectMixin] child model,
  ///    consider mixing in the [RenderBoxContainerDefaultsMixin] class and
  ///    using
  ///    [RenderBoxContainerDefaultsMixin.defaultComputeDistanceToFirstActualBaseline].
  ///
  ///  * For classes that define a particular baseline themselves, return that
  ///    value directly.
  ///
  ///  * For classes that have a child to which they wish to defer the
  ///    computation, call [getDistanceToActualBaseline] on the child (not
  ///    [computeDistanceToActualBaseline], the internal implementation, and not
  ///    [getDistanceToBaseline], the public entry point for this API).
  @visibleForOverriding
  @protected
  double? computeDistanceToActualBaseline(TextBaseline baseline) {
    assert(
      _debugDoingBaseline,
      'Please see the documentation for computeDistanceToActualBaseline for the required calling conventions of this method.',
    );
    return null;
  }

  /// The box constraints most recently received from the parent.
  @override
  BoxConstraints get constraints => super.constraints as BoxConstraints;

  @override
  void debugAssertDoesMeetConstraints() {
    assert(() {
      if (!hasSize) {
        final DiagnosticsNode contract;
        if (sizedByParent) {
          contract = ErrorDescription(
            'Because this RenderBox has sizedByParent set to true, it must set its size in performResize().',
          );
        } else {
          contract = ErrorDescription(
            'Because this RenderBox has sizedByParent set to false, it must set its size in performLayout().',
          );
        }
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('RenderBox did not set its size during layout.'),
          contract,
          ErrorDescription(
            'It appears that this did not happen; layout completed, but the size property is still null.',
          ),
          DiagnosticsProperty<RenderBox>(
            'The RenderBox in question is',
            this,
            style: DiagnosticsTreeStyle.errorProperty,
          ),
        ]);
      }
      // verify that the size is not infinite
      if (!_size!.isFinite) {
        final List<DiagnosticsNode> information = <DiagnosticsNode>[
          ErrorSummary('$runtimeType object was given an infinite size during layout.'),
          ErrorDescription(
            'This probably means that it is a render object that tries to be '
            'as big as possible, but it was put inside another render object '
            'that allows its children to pick their own size.',
          ),
        ];
        if (!constraints.hasBoundedWidth) {
          RenderBox node = this;
          while (!node.constraints.hasBoundedWidth && node.parent is RenderBox) {
            node = node.parent! as RenderBox;
          }

          information.add(
            node.describeForError(
              'The nearest ancestor providing an unbounded width constraint is',
            ),
          );
        }
        if (!constraints.hasBoundedHeight) {
          RenderBox node = this;
          while (!node.constraints.hasBoundedHeight && node.parent is RenderBox) {
            node = node.parent! as RenderBox;
          }

          information.add(
            node.describeForError(
              'The nearest ancestor providing an unbounded height constraint is',
            ),
          );
        }
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ...information,
          DiagnosticsProperty<BoxConstraints>(
            'The constraints that applied to the $runtimeType were',
            constraints,
            style: DiagnosticsTreeStyle.errorProperty,
          ),
          DiagnosticsProperty<Size>(
            'The exact size it was given was',
            _size,
            style: DiagnosticsTreeStyle.errorProperty,
          ),
          ErrorHint('See https://flutter.dev/to/unbounded-constraints for more information.'),
        ]);
      }
      // verify that the size is within the constraints
      if (!constraints.isSatisfiedBy(_size!)) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('$runtimeType does not meet its constraints.'),
          DiagnosticsProperty<BoxConstraints>(
            'Constraints',
            constraints,
            style: DiagnosticsTreeStyle.errorProperty,
          ),
          DiagnosticsProperty<Size>('Size', _size, style: DiagnosticsTreeStyle.errorProperty),
          ErrorHint(
            'If you are not writing your own RenderBox subclass, then this is not '
            'your fault. Contact support: https://github.com/flutter/flutter/issues/new?template=2_bug.yml',
          ),
        ]);
      }
      if (debugCheckIntrinsicSizes) {
        // verify that the intrinsics are sane
        assert(!RenderObject.debugCheckingIntrinsics);
        RenderObject.debugCheckingIntrinsics = true;
        final List<DiagnosticsNode> failures = <DiagnosticsNode>[];

        double testIntrinsic(
          double Function(double extent) function,
          String name,
          double constraint,
        ) {
          final double result = function(constraint);
          if (result < 0) {
            failures.add(
              ErrorDescription(' * $name($constraint) returned a negative value: $result'),
            );
          }
          if (!result.isFinite) {
            failures.add(
              ErrorDescription(' * $name($constraint) returned a non-finite value: $result'),
            );
          }
          return result;
        }

        void testIntrinsicsForValues(
          double Function(double extent) getMin,
          double Function(double extent) getMax,
          String name,
          double constraint,
        ) {
          final double min = testIntrinsic(getMin, 'getMinIntrinsic$name', constraint);
          final double max = testIntrinsic(getMax, 'getMaxIntrinsic$name', constraint);
          if (min > max) {
            failures.add(
              ErrorDescription(
                ' * getMinIntrinsic$name($constraint) returned a larger value ($min) than getMaxIntrinsic$name($constraint) ($max)',
              ),
            );
          }
        }

        try {
          testIntrinsicsForValues(
            getMinIntrinsicWidth,
            getMaxIntrinsicWidth,
            'Width',
            double.infinity,
          );
          testIntrinsicsForValues(
            getMinIntrinsicHeight,
            getMaxIntrinsicHeight,
            'Height',
            double.infinity,
          );
          if (constraints.hasBoundedWidth) {
            testIntrinsicsForValues(
              getMinIntrinsicWidth,
              getMaxIntrinsicWidth,
              'Width',
              constraints.maxHeight,
            );
          }
          if (constraints.hasBoundedHeight) {
            testIntrinsicsForValues(
              getMinIntrinsicHeight,
              getMaxIntrinsicHeight,
              'Height',
              constraints.maxWidth,
            );
          }
          // TODO(ianh): Test that values are internally consistent in more ways than the above.
        } finally {
          RenderObject.debugCheckingIntrinsics = false;
        }

        if (failures.isNotEmpty) {
          // TODO(jacobr): consider nesting the failures object so it is collapsible.
          throw FlutterError.fromParts(<DiagnosticsNode>[
            ErrorSummary(
              'The intrinsic dimension methods of the $runtimeType class returned values that violate the intrinsic protocol contract.',
            ),
            ErrorDescription(
              'The following ${failures.length > 1 ? "failures" : "failure"} was detected:',
            ), // should this be tagged as an error or not?
            ...failures,
            ErrorHint(
              'If you are not writing your own RenderBox subclass, then this is not\n'
              'your fault. Contact support: https://github.com/flutter/flutter/issues/new?template=2_bug.yml',
            ),
          ]);
        }

        // Checking that getDryLayout computes the same size.
        _debugDryLayoutCalculationValid = true;
        RenderObject.debugCheckingIntrinsics = true;
        final Size dryLayoutSize;
        try {
          dryLayoutSize = getDryLayout(constraints);
        } finally {
          RenderObject.debugCheckingIntrinsics = false;
        }
        if (_debugDryLayoutCalculationValid && dryLayoutSize != size) {
          throw FlutterError.fromParts(<DiagnosticsNode>[
            ErrorSummary(
              'The size given to the ${objectRuntimeType(this, 'RenderBox')} class differs from the size computed by computeDryLayout.',
            ),
            ErrorDescription(
              'The size computed in ${sizedByParent ? 'performResize' : 'performLayout'} '
              'is $size, which is different from $dryLayoutSize, which was computed by computeDryLayout.',
            ),
            ErrorDescription('The constraints used were $constraints.'),
            ErrorHint(
              'If you are not writing your own RenderBox subclass, then this is not\n'
              'your fault. Contact support: https://github.com/flutter/flutter/issues/new?template=2_bug.yml',
            ),
          ]);
        }
      }
      return true;
    }());
  }

  void _debugVerifyDryBaselines() {
    assert(() {
      final List<DiagnosticsNode> messages = <DiagnosticsNode>[
        ErrorDescription('The constraints used were $constraints.'),
        ErrorHint(
          'If you are not writing your own RenderBox subclass, then this is not\n'
          'your fault. Contact support: https://github.com/flutter/flutter/issues/new?template=2_bug.yml',
        ),
      ];

      for (final TextBaseline baseline in TextBaseline.values) {
        assert(!RenderObject.debugCheckingIntrinsics);
        RenderObject.debugCheckingIntrinsics = true;
        _debugDryLayoutCalculationValid = true;
        final double? dryBaseline;
        final double? realBaseline;
        try {
          dryBaseline = getDryBaseline(constraints, baseline);
          realBaseline = getDistanceToBaseline(baseline, onlyReal: true);
        } finally {
          RenderObject.debugCheckingIntrinsics = false;
        }
        assert(!RenderObject.debugCheckingIntrinsics);
        if (!_debugDryLayoutCalculationValid || dryBaseline == realBaseline) {
          continue;
        }
        if ((dryBaseline == null) != (realBaseline == null)) {
          final (String methodReturnedNull, String methodReturnedNonNull) =
              dryBaseline == null
                  ? ('computeDryBaseline', 'computeDistanceToActualBaseline')
                  : ('computeDistanceToActualBaseline', 'computeDryBaseline');
          throw FlutterError.fromParts(<DiagnosticsNode>[
            ErrorSummary(
              'The $baseline location returned by ${objectRuntimeType(this, 'RenderBox')}.computeDistanceToActualBaseline '
              'differs from the baseline location computed by computeDryBaseline.',
            ),
            ErrorDescription(
              'The $methodReturnedNull method returned null while the $methodReturnedNonNull returned a non-null $baseline of ${dryBaseline ?? realBaseline}. '
              'Did you forget to implement $methodReturnedNull for ${objectRuntimeType(this, 'RenderBox')}?',
            ),
            ...messages,
          ]);
        } else {
          throw FlutterError.fromParts(<DiagnosticsNode>[
            ErrorSummary(
              'The $baseline location returned by ${objectRuntimeType(this, 'RenderBox')}.computeDistanceToActualBaseline '
              'differs from the baseline location computed by computeDryBaseline.',
            ),
            DiagnosticsProperty<RenderObject>('The RenderBox was', this),
            ErrorDescription(
              'The computeDryBaseline method returned $dryBaseline,\n'
              'while the computeDistanceToActualBaseline method returned $realBaseline.\n'
              'Consider checking the implementations of the following methods on the ${objectRuntimeType(this, 'RenderBox')} class and make sure they are consistent:\n'
              ' * computeDistanceToActualBaseline\n'
              ' * computeDryBaseline\n'
              ' * performLayout\n',
            ),
            ...messages,
          ]);
        }
      }
      return true;
    }());
  }

  @override
  void markNeedsLayout() {
    // If `_layoutCacheStorage.clear` returns true, then this [RenderBox]'s layout
    // is used by the parent's layout algorithm (it's possible that the parent
    // only used the intrinsics for paint, but there's no good way to detect that
    // so we conservatively assume it's a layout dependency).
    //
    // A render object's performLayout implementation may depend on the baseline
    // location or the intrinsic dimensions of a descendant, even when there are
    // relayout boundaries between them. The `_layoutCacheStorage` being non-empty
    // indicates that the parent depended on this RenderBox's baseline location,
    // or intrinsic sizes, and thus may need relayout, regardless of relayout
    // boundaries.
    //
    // Some calculations may fail (dry baseline, for example). The layout
    // dependency is still established, but only from the RenderBox that failed
    // to compute the dry baseline to the ancestor that queried the dry baseline.
    if (_layoutCacheStorage.clear() && parent != null) {
      markParentNeedsLayout();
      return;
    }
    super.markNeedsLayout();
  }

  /// {@macro flutter.rendering.RenderObject.performResize}
  ///
  /// By default this method sets [size] to the result of [computeDryLayout]
  /// called with the current [constraints]. Instead of overriding this method,
  /// consider overriding [computeDryLayout].
  @override
  void performResize() {
    // default behavior for subclasses that have sizedByParent = true
    size = computeDryLayout(constraints);
    assert(size.isFinite);
  }

  @override
  void performLayout() {
    assert(() {
      if (!sizedByParent) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('$runtimeType did not implement performLayout().'),
          ErrorHint(
            'RenderBox subclasses need to either override performLayout() to '
            'set a size and lay out any children, or, set sizedByParent to true '
            'so that performResize() sizes the render object.',
          ),
        ]);
      }
      return true;
    }());
  }

  /// Determines the set of render objects located at the given position.
  ///
  /// Returns true, and adds any render objects that contain the point to the
  /// given hit test result, if this render object or one of its descendants
  /// absorbs the hit (preventing objects below this one from being hit).
  /// Returns false if the hit can continue to other objects below this one.
  ///
  /// The caller is responsible for transforming [position] from global
  /// coordinates to its location relative to the origin of this [RenderBox].
  /// This [RenderBox] is responsible for checking whether the given position is
  /// within its bounds.
  ///
  /// If transforming is necessary, [BoxHitTestResult.addWithPaintTransform],
  /// [BoxHitTestResult.addWithPaintOffset], or
  /// [BoxHitTestResult.addWithRawTransform] need to be invoked by the caller
  /// to record the required transform operations in the [HitTestResult]. These
  /// methods will also help with applying the transform to `position`.
  ///
  /// Hit testing requires layout to be up-to-date but does not require painting
  /// to be up-to-date. That means a render object can rely upon [performLayout]
  /// having been called in [hitTest] but cannot rely upon [paint] having been
  /// called. For example, a render object might be a child of a [RenderOpacity]
  /// object, which calls [hitTest] on its children when its opacity is zero
  /// even though it does not [paint] its children.
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    assert(() {
      if (!hasSize) {
        if (debugNeedsLayout) {
          throw FlutterError.fromParts(<DiagnosticsNode>[
            ErrorSummary('Cannot hit test a render box that has never been laid out.'),
            describeForError('The hitTest() method was called on this RenderBox'),
            ErrorDescription(
              "Unfortunately, this object's geometry is not known at this time, "
              'probably because it has never been laid out. '
              'This means it cannot be accurately hit-tested.',
            ),
            ErrorHint(
              'If you are trying '
              'to perform a hit test during the layout phase itself, make sure '
              "you only hit test nodes that have completed layout (e.g. the node's "
              'children, after their layout() method has been called).',
            ),
          ]);
        }
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('Cannot hit test a render box with no size.'),
          describeForError('The hitTest() method was called on this RenderBox'),
          ErrorDescription(
            'Although this node is not marked as needing layout, '
            'its size is not set.',
          ),
          ErrorHint(
            'A RenderBox object must have an '
            'explicit size before it can be hit-tested. Make sure '
            'that the RenderBox in question sets its size during layout.',
          ),
        ]);
      }
      return true;
    }());
    if (_size!.contains(position)) {
      if (hitTestChildren(result, position: position) || hitTestSelf(position)) {
        result.add(BoxHitTestEntry(this, position));
        return true;
      }
    }
    return false;
  }

  /// Override this method if this render object can be hit even if its
  /// children were not hit.
  ///
  /// Returns true if the specified `position` should be considered a hit
  /// on this render object.
  ///
  /// The caller is responsible for transforming [position] from global
  /// coordinates to its location relative to the origin of this [RenderBox].
  /// This [RenderBox] is responsible for checking whether the given position is
  /// within its bounds.
  ///
  /// Used by [hitTest]. If you override [hitTest] and do not call this
  /// function, then you don't need to implement this function.
  @protected
  bool hitTestSelf(Offset position) => false;

  /// Override this method to check whether any children are located at the
  /// given position.
  ///
  /// Subclasses should return true if at least one child reported a hit at the
  /// specified position.
  ///
  /// Typically children should be hit-tested in reverse paint order so that
  /// hit tests at locations where children overlap hit the child that is
  /// visually "on top" (i.e., paints later).
  ///
  /// The caller is responsible for transforming [position] from global
  /// coordinates to its location relative to the origin of this [RenderBox].
  /// Likewise, this [RenderBox] is responsible for transforming the position
  /// that it passes to its children when it calls [hitTest] on each child.
  ///
  /// If transforming is necessary, [BoxHitTestResult.addWithPaintTransform],
  /// [BoxHitTestResult.addWithPaintOffset], or
  /// [BoxHitTestResult.addWithRawTransform] need to be invoked by subclasses to
  /// record the required transform operations in the [BoxHitTestResult]. These
  /// methods will also help with applying the transform to `position`.
  ///
  /// Used by [hitTest]. If you override [hitTest] and do not call this
  /// function, then you don't need to implement this function.
  @protected
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) => false;

  /// Multiply the transform from the parent's coordinate system to this box's
  /// coordinate system into the given transform.
  ///
  /// This function is used to convert coordinate systems between boxes.
  /// Subclasses that apply transforms during painting should override this
  /// function to factor those transforms into the calculation.
  ///
  /// The [RenderBox] implementation takes care of adjusting the matrix for the
  /// position of the given child as determined during layout and stored on the
  /// child's [parentData] in the [BoxParentData.offset] field.
  @override
  void applyPaintTransform(RenderObject child, Matrix4 transform) {
    assert(child.parent == this);
    assert(() {
      if (child.parentData is! BoxParentData) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('$runtimeType does not implement applyPaintTransform.'),
          describeForError('The following $runtimeType object'),
          child.describeForError(
            '...did not use a BoxParentData class for the parentData field of the following child',
          ),
          ErrorDescription('The $runtimeType class inherits from RenderBox.'),
          ErrorHint(
            'The default applyPaintTransform implementation provided by RenderBox assumes that the '
            'children all use BoxParentData objects for their parentData field. '
            'Since $runtimeType does not in fact use that ParentData class for its children, it must '
            'provide an implementation of applyPaintTransform that supports the specific ParentData '
            'subclass used by its children (which apparently is ${child.parentData.runtimeType}).',
          ),
        ]);
      }
      return true;
    }());
    final BoxParentData childParentData = child.parentData! as BoxParentData;
    final Offset offset = childParentData.offset;
    transform.translate(offset.dx, offset.dy);
  }

  /// Convert the given point from the global coordinate system in logical pixels
  /// to the local coordinate system for this box.
  ///
  /// This method will un-project the point from the screen onto the widget,
  /// which makes it different from [MatrixUtils.transformPoint].
  ///
  /// If the transform from global coordinates to local coordinates is
  /// degenerate, this function returns [Offset.zero].
  ///
  /// If `ancestor` is non-null, this function converts the given point from the
  /// coordinate system of `ancestor` (which must be an ancestor of this render
  /// object) instead of from the global coordinate system.
  ///
  /// This method is implemented in terms of [getTransformTo].
  Offset globalToLocal(Offset point, {RenderObject? ancestor}) {
    // We want to find point (p) that corresponds to a given point on the
    // screen (s), but that also physically resides on the local render plane,
    // so that it is useful for visually accurate gesture processing in the
    // local space. For that, we can't simply transform 2D screen point to
    // the 3D local space since the screen space lacks the depth component |z|,
    // and so there are many 3D points that correspond to the screen point.
    // We must first unproject the screen point onto the render plane to find
    // the true 3D point that corresponds to the screen point.
    // We do orthogonal unprojection after undoing perspective, in local space.
    // The render plane is specified by renderBox offset (o) and Z axis (n).
    // Unprojection is done by finding the intersection of the view vector (d)
    // with the local X-Y plane: (o-s).dot(n) == (p-s).dot(n), (p-s) == |z|*d.
    final Matrix4 transform = getTransformTo(ancestor);
    final double det = transform.invert();
    if (det == 0.0) {
      return Offset.zero;
    }
    final Vector3 n = Vector3(0.0, 0.0, 1.0);
    final Vector3 i = transform.perspectiveTransform(Vector3(0.0, 0.0, 0.0));
    final Vector3 d = transform.perspectiveTransform(Vector3(0.0, 0.0, 1.0)) - i;
    final Vector3 s = transform.perspectiveTransform(Vector3(point.dx, point.dy, 0.0));
    final Vector3 p = s - d * (n.dot(s) / n.dot(d));
    return Offset(p.x, p.y);
  }

  /// Convert the given point from the local coordinate system for this box to
  /// the global coordinate system in logical pixels.
  ///
  /// If `ancestor` is non-null, this function converts the given point to the
  /// coordinate system of `ancestor` (which must be an ancestor of this render
  /// object) instead of to the global coordinate system.
  ///
  /// This method is implemented in terms of [getTransformTo]. If the transform
  /// matrix puts the given `point` on the line at infinity (for instance, when
  /// the transform matrix is the zero matrix), this method returns (NaN, NaN).
  Offset localToGlobal(Offset point, {RenderObject? ancestor}) {
    return MatrixUtils.transformPoint(getTransformTo(ancestor), point);
  }

  /// Returns a rectangle that contains all the pixels painted by this box.
  ///
  /// The paint bounds can be larger or smaller than [size], which is the amount
  /// of space this box takes up during layout. For example, if this box casts a
  /// shadow, that shadow might extend beyond the space allocated to this box
  /// during layout.
  ///
  /// The paint bounds are used to size the buffers into which this box paints.
  /// If the box attempts to paints outside its paint bounds, there might not be
  /// enough memory allocated to represent the box's visual appearance, which
  /// can lead to undefined behavior.
  ///
  /// The returned paint bounds are in the local coordinate system of this box.
  @override
  Rect get paintBounds => Offset.zero & size;

  /// Override this method to handle pointer events that hit this render object.
  ///
  /// For [RenderBox] objects, the `entry` argument is a [BoxHitTestEntry]. From this
  /// object you can determine the [PointerDownEvent]'s position in local coordinates.
  /// (This is useful because [PointerEvent.position] is in global coordinates.)
  ///
  /// Implementations of this method should call [debugHandleEvent] as follows,
  /// so that they support [debugPaintPointersEnabled]:
  ///
  /// ```dart
  /// class RenderFoo extends RenderBox {
  ///   // ...
  ///
  ///   @override
  ///   void handleEvent(PointerEvent event, HitTestEntry entry) {
  ///     assert(debugHandleEvent(event, entry));
  ///     // ... handle the event ...
  ///   }
  ///
  ///   // ...
  /// }
  /// ```
  @override
  void handleEvent(PointerEvent event, BoxHitTestEntry entry) {
    super.handleEvent(event, entry);
  }

  int _debugActivePointers = 0;

  /// Implements the [debugPaintPointersEnabled] debugging feature.
  ///
  /// [RenderBox] subclasses that implement [handleEvent] should call
  /// [debugHandleEvent] from their [handleEvent] method, as follows:
  ///
  /// ```dart
  /// class RenderFoo extends RenderBox {
  ///   // ...
  ///
  ///   @override
  ///   void handleEvent(PointerEvent event, HitTestEntry entry) {
  ///     assert(debugHandleEvent(event, entry));
  ///     // ... handle the event ...
  ///   }
  ///
  ///   // ...
  /// }
  /// ```
  ///
  /// If you call this for a [PointerDownEvent], make sure you also call it for
  /// the corresponding [PointerUpEvent] or [PointerCancelEvent].
  bool debugHandleEvent(PointerEvent event, HitTestEntry entry) {
    assert(() {
      if (debugPaintPointersEnabled) {
        if (event is PointerDownEvent) {
          _debugActivePointers += 1;
        } else if (event is PointerUpEvent || event is PointerCancelEvent) {
          _debugActivePointers -= 1;
        }
        markNeedsPaint();
      }
      return true;
    }());
    return true;
  }

  @override
  void debugPaint(PaintingContext context, Offset offset) {
    assert(() {
      // Only perform the baseline checks after `PipelineOwner.flushLayout` completes.
      // We can't run this check in the same places we run other intrinsics checks
      // (in the `RenderBox.size` setter, or after `performResize`), because
      // `getDistanceToBaseline` may depend on the layout of the child so it's
      // the safest to only call `getDistanceToBaseline` after the entire tree
      // finishes doing layout.
      //
      // Descendant `RenderObject`s typically call `debugPaint` before their
      // parents do. This means the baseline implementations are checked from
      // descendants to ancestors, allowing us to spot the `RenderBox` with an
      // inconsistent implementation, instead of its ancestors that only reported
      // inconsistent baseline values because one of its ancestors has an
      // inconsistent implementation.
      if (debugCheckIntrinsicSizes) {
        _debugVerifyDryBaselines();
      }
      if (debugPaintSizeEnabled) {
        debugPaintSize(context, offset);
      }
      if (debugPaintBaselinesEnabled) {
        debugPaintBaselines(context, offset);
      }
      if (debugPaintPointersEnabled) {
        debugPaintPointers(context, offset);
      }
      return true;
    }());
  }

  /// In debug mode, paints a border around this render box.
  ///
  /// Called for every [RenderBox] when [debugPaintSizeEnabled] is true.
  @protected
  @visibleForTesting
  void debugPaintSize(PaintingContext context, Offset offset) {
    assert(() {
      final Paint paint =
          Paint()
            ..style = PaintingStyle.stroke
            ..strokeWidth = 1.0
            ..color = const Color(0xFF00FFFF);
      context.canvas.drawRect((offset & size).deflate(0.5), paint);
      return true;
    }());
  }

  /// In debug mode, paints a line for each baseline.
  ///
  /// Called for every [RenderBox] when [debugPaintBaselinesEnabled] is true.
  @protected
  void debugPaintBaselines(PaintingContext context, Offset offset) {
    assert(() {
      final Paint paint =
          Paint()
            ..style = PaintingStyle.stroke
            ..strokeWidth = 0.25;
      Path path;
      // ideographic baseline
      final double? baselineI = getDistanceToBaseline(TextBaseline.ideographic, onlyReal: true);
      if (baselineI != null) {
        paint.color = const Color(0xFFFFD000);
        path = Path();
        path.moveTo(offset.dx, offset.dy + baselineI);
        path.lineTo(offset.dx + size.width, offset.dy + baselineI);
        context.canvas.drawPath(path, paint);
      }
      // alphabetic baseline
      final double? baselineA = getDistanceToBaseline(TextBaseline.alphabetic, onlyReal: true);
      if (baselineA != null) {
        paint.color = const Color(0xFF00FF00);
        path = Path();
        path.moveTo(offset.dx, offset.dy + baselineA);
        path.lineTo(offset.dx + size.width, offset.dy + baselineA);
        context.canvas.drawPath(path, paint);
      }
      return true;
    }());
  }

  /// In debug mode, paints a rectangle if this render box has counted more
  /// pointer downs than pointer up events.
  ///
  /// Called for every [RenderBox] when [debugPaintPointersEnabled] is true.
  ///
  /// By default, events are not counted. For details on how to ensure that
  /// events are counted for your class, see [debugHandleEvent].
  @protected
  void debugPaintPointers(PaintingContext context, Offset offset) {
    assert(() {
      if (_debugActivePointers > 0) {
        final Paint paint = Paint()..color = Color(0x00BBBB | ((0x04000000 * depth) & 0xFF000000));
        context.canvas.drawRect(offset & size, paint);
      }
      return true;
    }());
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<Size>('size', _size, missingIfNull: true));
  }
}

/// A mixin that provides useful default behaviors for boxes with children
/// managed by the [ContainerRenderObjectMixin] mixin.
///
/// By convention, this class doesn't override any members of the superclass.
/// Instead, it provides helpful functions that subclasses can call as
/// appropriate.
mixin RenderBoxContainerDefaultsMixin<
  ChildType extends RenderBox,
  ParentDataType extends ContainerBoxParentData<ChildType>
>
    implements ContainerRenderObjectMixin<ChildType, ParentDataType> {
  /// Returns the baseline of the first child with a baseline.
  ///
  /// Useful when the children are displayed vertically in the same order they
  /// appear in the child list.
  double? defaultComputeDistanceToFirstActualBaseline(TextBaseline baseline) {
    assert(!debugNeedsLayout);
    ChildType? child = firstChild;
    while (child != null) {
      final ParentDataType childParentData = child.parentData! as ParentDataType;
      final double? result = child.getDistanceToActualBaseline(baseline);
      if (result != null) {
        return result + childParentData.offset.dy;
      }
      child = childParentData.nextSibling;
    }
    return null;
  }

  /// Returns the minimum baseline value among every child.
  ///
  /// Useful when the vertical position of the children isn't determined by the
  /// order in the child list.
  double? defaultComputeDistanceToHighestActualBaseline(TextBaseline baseline) {
    assert(!debugNeedsLayout);
    BaselineOffset minBaseline = BaselineOffset.noBaseline;
    ChildType? child = firstChild;
    while (child != null) {
      final ParentDataType childParentData = child.parentData! as ParentDataType;
      final BaselineOffset candidate =
          BaselineOffset(child.getDistanceToActualBaseline(baseline)) + childParentData.offset.dy;
      minBaseline = minBaseline.minOf(candidate);
      child = childParentData.nextSibling;
    }
    return minBaseline.offset;
  }

  /// Performs a hit test on each child by walking the child list backwards.
  ///
  /// Stops walking once after the first child reports that it contains the
  /// given point. Returns whether any children contain the given point.
  ///
  /// See also:
  ///
  ///  * [defaultPaint], which paints the children appropriate for this
  ///    hit-testing strategy.
  bool defaultHitTestChildren(BoxHitTestResult result, {required Offset position}) {
    ChildType? child = lastChild;
    while (child != null) {
      // The x, y parameters have the top left of the node's box as the origin.
      final ParentDataType childParentData = child.parentData! as ParentDataType;
      final bool isHit = result.addWithPaintOffset(
        offset: childParentData.offset,
        position: position,
        hitTest: (BoxHitTestResult result, Offset transformed) {
          assert(transformed == position - childParentData.offset);
          return child!.hitTest(result, position: transformed);
        },
      );
      if (isHit) {
        return true;
      }
      child = childParentData.previousSibling;
    }
    return false;
  }

  /// Paints each child by walking the child list forwards.
  ///
  /// See also:
  ///
  ///  * [defaultHitTestChildren], which implements hit-testing of the children
  ///    in a manner appropriate for this painting strategy.
  void defaultPaint(PaintingContext context, Offset offset) {
    ChildType? child = firstChild;
    while (child != null) {
      final ParentDataType childParentData = child.parentData! as ParentDataType;
      context.paintChild(child, childParentData.offset + offset);
      child = childParentData.nextSibling;
    }
  }

  /// Returns a list containing the children of this render object.
  ///
  /// This function is useful when you need random-access to the children of
  /// this render object. If you're accessing the children in order, consider
  /// walking the child list directly.
  List<ChildType> getChildrenAsList() {
    final List<ChildType> result = <ChildType>[];
    RenderBox? child = firstChild;
    while (child != null) {
      final ParentDataType childParentData = child.parentData! as ParentDataType;
      result.add(child as ChildType);
      child = childParentData.nextSibling;
    }
    return result;
  }
}
