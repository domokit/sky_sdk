// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'box_border.dart';
library;

import 'dart:math' as math;
import 'dart:ui' as ui show lerpDouble;

import 'package:flutter/foundation.dart';

import 'basic_types.dart';
import 'edge_insets.dart';

/// The style of line to draw for a [BorderSide] in a [Border].
enum BorderStyle {
  /// Skip the border.
  none,

  /// Draw the border as a solid line.
  solid,

  // if you add more, think about how they will lerp
}

/// A side of a border of a box.
///
/// A [Border] consists of four [BorderSide] objects: [Border.top],
/// [Border.left], [Border.right], and [Border.bottom].
///
/// Setting [BorderSide.width] to 0.0 will result in hairline rendering; see
/// [BorderSide.width] for a more involved explanation.
///
/// {@tool snippet}
/// This sample shows how [BorderSide] objects can be used in a [Container], via
/// a [BoxDecoration] and a [Border], to decorate some [Text]. In this example,
/// the text has a thick bar above it that is light blue, and a thick bar below
/// it that is a darker shade of blue.
///
/// ```dart
/// Container(
///   padding: const EdgeInsets.all(8.0),
///   decoration: BoxDecoration(
///     border: Border(
///       top: BorderSide(width: 16.0, color: Colors.lightBlue.shade50),
///       bottom: BorderSide(width: 16.0, color: Colors.lightBlue.shade900),
///     ),
///   ),
///   child: const Text('Flutter in the sky', textAlign: TextAlign.center),
/// )
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [Border], which uses [BorderSide] objects to represent its sides.
///  * [BoxDecoration], which optionally takes a [Border] object.
///  * [TableBorder], which is similar to [Border] but has two more sides
///    ([TableBorder.horizontalInside] and [TableBorder.verticalInside]), both
///    of which are also [BorderSide] objects.
@immutable
class BorderSide with Diagnosticable {
  /// Creates the side of a border.
  ///
  /// By default, the border is 1.0 logical pixels wide and solid black.
  const BorderSide({
    this.color = const Color(0xFF000000),
    this.width = 1.0,
    this.style = BorderStyle.solid,
    this.strokeAlign = strokeAlignInside,
  }) : assert(width >= 0.0);

  /// Creates a [BorderSide] that represents the addition of the two given
  /// [BorderSide]s.
  ///
  /// It is only valid to call this if [canMerge] returns true for the two
  /// sides.
  ///
  /// If one of the sides is zero-width with [BorderStyle.none], then the other
  /// side is return as-is. If both of the sides are zero-width with
  /// [BorderStyle.none], then [BorderSide.none] is returned.
  static BorderSide merge(BorderSide a, BorderSide b) {
    assert(canMerge(a, b));
    final bool aIsNone = a.style == BorderStyle.none && a.width == 0.0;
    final bool bIsNone = b.style == BorderStyle.none && b.width == 0.0;
    if (aIsNone && bIsNone) {
      return BorderSide.none;
    }
    if (aIsNone) {
      return b;
    }
    if (bIsNone) {
      return a;
    }
    assert(a.color == b.color);
    assert(a.style == b.style);
    return BorderSide(
      color: a.color, // == b.color
      width: a.width + b.width,
      strokeAlign: math.max(a.strokeAlign, b.strokeAlign),
      style: a.style, // == b.style
    );
  }

  /// The color of this side of the border.
  final Color color;

  /// The width of this side of the border, in logical pixels.
  ///
  /// Setting width to 0.0 will result in a hairline border. This means that
  /// the border will have the width of one physical pixel. Hairline
  /// rendering takes shortcuts when the path overlaps a pixel more than once.
  /// This means that it will render faster than otherwise, but it might
  /// double-hit pixels, giving it a slightly darker/lighter result.
  ///
  /// To omit the border entirely, set the [style] to [BorderStyle.none].
  final double width;

  /// The style of this side of the border.
  ///
  /// To omit a side, set [style] to [BorderStyle.none]. This skips
  /// painting the border, but the border still has a [width].
  final BorderStyle style;

  /// A hairline black border that is not rendered.
  static const BorderSide none = BorderSide(width: 0.0, style: BorderStyle.none);

  /// The relative position of the stroke on a [BorderSide] in an
  /// [OutlinedBorder] or [Border].
  ///
  /// Values typically range from -1.0 ([strokeAlignInside], inside border,
  /// default) to 1.0 ([strokeAlignOutside], outside border), without any
  /// bound constraints (e.g., a value of -2.0 is not typical, but allowed).
  /// A value of 0 ([strokeAlignCenter]) will center the border on the edge
  /// of the widget.
  ///
  /// When set to [strokeAlignInside], the stroke is drawn completely inside
  /// the widget. For [strokeAlignCenter] and [strokeAlignOutside], a property
  /// such as [Container.clipBehavior] can be used in an outside widget to clip
  /// it. If [Container.decoration] has a border, the container may incorporate
  /// [width] as additional padding:
  /// - [strokeAlignInside] provides padding with full [width].
  /// - [strokeAlignCenter] provides padding with half [width].
  /// - [strokeAlignOutside] provides zero padding, as stroke is drawn entirely outside.
  ///
  /// This property is not honored by [toPaint] (because the [Paint] object
  /// cannot represent it); it is intended that classes that use [BorderSide]
  /// objects implement this property when painting borders by suitably
  /// inflating or deflating their regions.
  ///
  /// {@tool dartpad}
  /// This example shows an animation of how [strokeAlign] affects the drawing
  /// when applied to borders of various shapes.
  ///
  /// ** See code in examples/api/lib/painting/borders/border_side.stroke_align.0.dart **
  /// {@end-tool}
  final double strokeAlign;

  /// The border is drawn fully inside of the border path.
  ///
  /// This is a constant for use with [strokeAlign].
  ///
  /// This is the default value for [strokeAlign].
  static const double strokeAlignInside = -1.0;

  /// The border is drawn on the center of the border path, with half of the
  /// [BorderSide.width] on the inside, and the other half on the outside of
  /// the path.
  ///
  /// This is a constant for use with [strokeAlign].
  static const double strokeAlignCenter = 0.0;

  /// The border is drawn on the outside of the border path.
  ///
  /// This is a constant for use with [strokeAlign].
  static const double strokeAlignOutside = 1.0;

  /// Creates a copy of this border but with the given fields replaced with the new values.
  BorderSide copyWith({Color? color, double? width, BorderStyle? style, double? strokeAlign}) {
    return BorderSide(
      color: color ?? this.color,
      width: width ?? this.width,
      style: style ?? this.style,
      strokeAlign: strokeAlign ?? this.strokeAlign,
    );
  }

  /// Creates a copy of this border side description but with the width scaled
  /// by the factor `t`.
  ///
  /// The `t` argument represents the multiplicand, or the position on the
  /// timeline for an interpolation from nothing to `this`, with 0.0 meaning
  /// that the object returned should be the nil variant of this object, 1.0
  /// meaning that no change should be applied, returning `this` (or something
  /// equivalent to `this`), and other values meaning that the object should be
  /// multiplied by `t`. Negative values are treated like zero.
  ///
  /// Since a zero width is normally painted as a hairline width rather than no
  /// border at all, the zero factor is special-cased to instead change the
  /// style to [BorderStyle.none].
  ///
  /// Values for `t` are usually obtained from an [Animation<double>], such as
  /// an [AnimationController].
  BorderSide scale(double t) {
    return BorderSide(
      color: color,
      width: math.max(0.0, width * t),
      style: t <= 0.0 ? BorderStyle.none : style,
    );
  }

  /// Create a [Paint] object that, if used to stroke a line, will draw the line
  /// in this border's style.
  ///
  /// The [strokeAlign] property is not reflected in the [Paint]; consumers must
  /// implement that directly by inflating or deflating their region appropriately.
  ///
  /// Not all borders use this method to paint their border sides. For example,
  /// non-uniform rectangular [Border]s have beveled edges and so paint their
  /// border sides as filled shapes rather than using a stroke.
  Paint toPaint() {
    switch (style) {
      case BorderStyle.solid:
        return Paint()
          ..color = color
          ..strokeWidth = width
          ..style = PaintingStyle.stroke;
      case BorderStyle.none:
        return Paint()
          ..color = const Color(0x00000000)
          ..strokeWidth = 0.0
          ..style = PaintingStyle.stroke;
    }
  }

  /// Whether the two given [BorderSide]s can be merged using
  /// [BorderSide.merge].
  ///
  /// Two sides can be merged if one or both are zero-width with
  /// [BorderStyle.none], or if they both have the same color and style.
  static bool canMerge(BorderSide a, BorderSide b) {
    if ((a.style == BorderStyle.none && a.width == 0.0) ||
        (b.style == BorderStyle.none && b.width == 0.0)) {
      return true;
    }
    return a.style == b.style && a.color == b.color;
  }

  /// Linearly interpolate between two border sides.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static BorderSide lerp(BorderSide a, BorderSide b, double t) {
    if (identical(a, b)) {
      return a;
    }
    if (t == 0.0) {
      return a;
    }
    if (t == 1.0) {
      return b;
    }
    final double width = ui.lerpDouble(a.width, b.width, t)!;
    if (width < 0.0) {
      return BorderSide.none;
    }
    if (a.style == b.style && a.strokeAlign == b.strokeAlign) {
      return BorderSide(
        color: Color.lerp(a.color, b.color, t)!,
        width: width,
        style: a.style, // == b.style
        strokeAlign: a.strokeAlign, // == b.strokeAlign
      );
    }
    final Color colorA = switch (a.style) {
      BorderStyle.solid => a.color,
      BorderStyle.none => a.color.withAlpha(0x00),
    };
    final Color colorB = switch (b.style) {
      BorderStyle.solid => b.color,
      BorderStyle.none => b.color.withAlpha(0x00),
    };
    if (a.strokeAlign != b.strokeAlign) {
      return BorderSide(
        color: Color.lerp(colorA, colorB, t)!,
        width: width,
        strokeAlign: ui.lerpDouble(a.strokeAlign, b.strokeAlign, t)!,
      );
    }
    return BorderSide(
      color: Color.lerp(colorA, colorB, t)!,
      width: width,
      strokeAlign: a.strokeAlign, // == b.strokeAlign
    );
  }

  /// Get the amount of the stroke width that lies inside of the [BorderSide].
  ///
  /// For example, this will return the [width] for a [strokeAlign] of -1, half
  /// the [width] for a [strokeAlign] of 0, and 0 for a [strokeAlign] of 1.
  double get strokeInset => width * (1 - (1 + strokeAlign) / 2);

  /// Get the amount of the stroke width that lies outside of the [BorderSide].
  ///
  /// For example, this will return 0 for a [strokeAlign] of -1, half the
  /// [width] for a [strokeAlign] of 0, and the [width] for a [strokeAlign]
  /// of 1.
  double get strokeOutset => width * (1 + strokeAlign) / 2;

  /// The offset of the stroke, taking into account the stroke alignment.
  ///
  /// For example, this will return the negative [width] of the stroke
  /// for a [strokeAlign] of -1, 0 for a [strokeAlign] of 0, and the
  /// [width] for a [strokeAlign] of -1.
  double get strokeOffset => width * strokeAlign;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is BorderSide &&
        other.color == color &&
        other.width == width &&
        other.style == style &&
        other.strokeAlign == strokeAlign;
  }

  @override
  int get hashCode => Object.hash(color, width, style, strokeAlign);

  @override
  String toStringShort() => 'BorderSide';

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      DiagnosticsProperty<Color>('color', color, defaultValue: const Color(0xFF000000)),
    );
    properties.add(DoubleProperty('width', width, defaultValue: 1.0));
    properties.add(DoubleProperty('strokeAlign', strokeAlign, defaultValue: strokeAlignInside));
    properties.add(EnumProperty<BorderStyle>('style', style, defaultValue: BorderStyle.solid));
  }
}

/// Base class for shape outlines.
///
/// This class handles how to add multiple borders together. Subclasses define
/// various shapes, like circles ([CircleBorder]), rounded rectangles
/// ([RoundedRectangleBorder]), continuous rectangles
/// ([ContinuousRectangleBorder]), or beveled rectangles
/// ([BeveledRectangleBorder]).
///
/// See also:
///
///  * [ShapeDecoration], which can be used with [DecoratedBox] to show a shape.
///  * [Material] (and many other widgets in the Material library), which takes
///    a [ShapeBorder] to define its shape.
///  * [NotchedShape], which describes a shape with a hole in it.
@immutable
abstract class ShapeBorder {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const ShapeBorder();

  /// The widths of the sides of this border represented as an [EdgeInsets].
  ///
  /// Specifically, this is the amount by which a rectangle should be inset so
  /// as to avoid painting over any important part of the border. It is the
  /// amount by which additional borders will be inset before they are drawn.
  ///
  /// This can be used, for example, with a [Padding] widget to inset a box by
  /// the size of these borders.
  ///
  /// Shapes that have a fixed ratio regardless of the area on which they are
  /// painted, or that change their rendering based on the size they are given
  /// when painting (for instance [CircleBorder]), will not return valid
  /// [dimensions] information because they cannot know their eventual size when
  /// computing their [dimensions].
  EdgeInsetsGeometry get dimensions;

  /// Attempts to create a new object that represents the amalgamation of `this`
  /// border and the `other` border.
  ///
  /// If the type of the other border isn't known, or the given instance cannot
  /// be reasonably added to this instance, then this should return null.
  ///
  /// This method is used by the [operator +] implementation.
  ///
  /// The `reversed` argument is true if this object was the right operand of
  /// the `+` operator, and false if it was the left operand.
  @protected
  ShapeBorder? add(ShapeBorder other, {bool reversed = false}) => null;

  /// Creates a new border consisting of the two borders on either side of the
  /// operator.
  ///
  /// If the borders belong to classes that know how to add themselves, then
  /// this results in a new border that represents the intelligent addition of
  /// those two borders (see [add]). Otherwise, an object is returned that
  /// merely paints the two borders sequentially, with the left hand operand on
  /// the inside and the right hand operand on the outside.
  ShapeBorder operator +(ShapeBorder other) {
    return add(other) ??
        other.add(this, reversed: true) ??
        _CompoundBorder(<ShapeBorder>[other, this]);
  }

  /// Creates a copy of this border, scaled by the factor `t`.
  ///
  /// Typically this means scaling the width of the border's side, but it can
  /// also include scaling other artifacts of the border, e.g. the border radius
  /// of a [RoundedRectangleBorder].
  ///
  /// The `t` argument represents the multiplicand, or the position on the
  /// timeline for an interpolation from nothing to `this`, with 0.0 meaning
  /// that the object returned should be the nil variant of this object, 1.0
  /// meaning that no change should be applied, returning `this` (or something
  /// equivalent to `this`), and other values meaning that the object should be
  /// multiplied by `t`. Negative values are allowed but may be meaningless
  /// (they correspond to extrapolating the interpolation from this object to
  /// nothing, and going beyond nothing)
  ///
  /// Values for `t` are usually obtained from an [Animation<double>], such as
  /// an [AnimationController].
  ///
  /// See also:
  ///
  ///  * [BorderSide.scale], which most [ShapeBorder] subclasses defer to for
  ///    the actual computation.
  ShapeBorder scale(double t);

  /// Linearly interpolates from another [ShapeBorder] (possibly of another
  /// class) to `this`.
  ///
  /// When implementing this method in subclasses, return null if this class
  /// cannot interpolate from `a`. In that case, [lerp] will try `a`'s [lerpTo]
  /// method instead. If `a` is null, this must not return null.
  ///
  /// The base class implementation handles the case of `a` being null by
  /// deferring to [scale].
  ///
  /// The `t` argument represents position on the timeline, with 0.0 meaning
  /// that the interpolation has not started, returning `a` (or something
  /// equivalent to `a`), 1.0 meaning that the interpolation has finished,
  /// returning `this` (or something equivalent to `this`), and values in
  /// between meaning that the interpolation is at the relevant point on the
  /// timeline between `a` and `this`. The interpolation can be extrapolated
  /// beyond 0.0 and 1.0, so negative values and values greater than 1.0 are
  /// valid (and can easily be generated by curves such as
  /// [Curves.elasticInOut]).
  ///
  /// Values for `t` are usually obtained from an [Animation<double>], such as
  /// an [AnimationController].
  ///
  /// Instead of calling this directly, use [ShapeBorder.lerp].
  @protected
  ShapeBorder? lerpFrom(ShapeBorder? a, double t) {
    if (a == null) {
      return scale(t);
    }
    return null;
  }

  /// Linearly interpolates from `this` to another [ShapeBorder] (possibly of
  /// another class).
  ///
  /// This is called if `b`'s [lerpTo] did not know how to handle this class.
  ///
  /// When implementing this method in subclasses, return null if this class
  /// cannot interpolate from `b`. In that case, [lerp] will apply a default
  /// behavior instead. If `b` is null, this must not return null.
  ///
  /// The base class implementation handles the case of `b` being null by
  /// deferring to [scale].
  ///
  /// The `t` argument represents position on the timeline, with 0.0 meaning
  /// that the interpolation has not started, returning `this` (or something
  /// equivalent to `this`), 1.0 meaning that the interpolation has finished,
  /// returning `b` (or something equivalent to `b`), and values in between
  /// meaning that the interpolation is at the relevant point on the timeline
  /// between `this` and `b`. The interpolation can be extrapolated beyond 0.0
  /// and 1.0, so negative values and values greater than 1.0 are valid (and can
  /// easily be generated by curves such as [Curves.elasticInOut]).
  ///
  /// Values for `t` are usually obtained from an [Animation<double>], such as
  /// an [AnimationController].
  ///
  /// Instead of calling this directly, use [ShapeBorder.lerp].
  @protected
  ShapeBorder? lerpTo(ShapeBorder? b, double t) {
    if (b == null) {
      return scale(1.0 - t);
    }
    return null;
  }

  /// Linearly interpolates between two [ShapeBorder]s.
  ///
  /// This defers to `b`'s [lerpTo] function if `b` is not null. If `b` is
  /// null or if its [lerpTo] returns null, it uses `a`'s [lerpFrom]
  /// function instead. If both return null, it returns `a` before `t=0.5`
  /// and `b` after `t=0.5`.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static ShapeBorder? lerp(ShapeBorder? a, ShapeBorder? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    final ShapeBorder? result = b?.lerpFrom(a, t) ?? a?.lerpTo(b, t);
    return result ?? (t < 0.5 ? a : b);
  }

  /// Create a [Path] that describes the outer edge of the border.
  ///
  /// This path must not cross the path given by [getInnerPath] for the same
  /// [Rect].
  ///
  /// To obtain a [Path] that describes the area of the border itself, set the
  /// [Path.fillType] of the returned object to [PathFillType.evenOdd], and add
  /// to this object the path returned from [getInnerPath] (using
  /// [Path.addPath]).
  ///
  /// The `textDirection` argument must be provided non-null if the border
  /// has a text direction dependency (for example if it is expressed in terms
  /// of "start" and "end" instead of "left" and "right"). It may be null if
  /// the border will not need the text direction to paint itself.
  ///
  /// See also:
  ///
  ///  * [getInnerPath], which creates the path for the inner edge.
  ///  * [Path.contains], which can tell if an [Offset] is within a [Path].
  Path getOuterPath(Rect rect, {TextDirection? textDirection});

  /// Create a [Path] that describes the inner edge of the border.
  ///
  /// This path must not cross the path given by [getOuterPath] for the same
  /// [Rect].
  ///
  /// To obtain a [Path] that describes the area of the border itself, set the
  /// [Path.fillType] of the returned object to [PathFillType.evenOdd], and add
  /// to this object the path returned from [getOuterPath] (using
  /// [Path.addPath]).
  ///
  /// The `textDirection` argument must be provided and non-null if the border
  /// has a text direction dependency (for example if it is expressed in terms
  /// of "start" and "end" instead of "left" and "right"). It may be null if
  /// the border will not need the text direction to paint itself.
  ///
  /// See also:
  ///
  ///  * [getOuterPath], which creates the path for the outer edge.
  ///  * [Path.contains], which can tell if an [Offset] is within a [Path].
  Path getInnerPath(Rect rect, {TextDirection? textDirection});

  /// Paint a canvas with the appropriate shape.
  ///
  /// On [ShapeBorder] subclasses whose [preferPaintInterior] method returns
  /// true, this should be faster than using [Canvas.drawPath] with the path
  /// provided by [getOuterPath]. (If [preferPaintInterior] returns false,
  /// then this method asserts in debug mode and does nothing in release mode.)
  ///
  /// Subclasses are expected to implement this method when the [Canvas] API
  /// has a dedicated method to draw the relevant shape. For example,
  /// [CircleBorder] uses this to call [Canvas.drawCircle], and
  /// [RoundedRectangleBorder] uses this to call [Canvas.drawRRect].
  ///
  /// Subclasses that implement this must ensure that calling [paintInterior]
  /// is semantically equivalent to (i.e. renders the same pixels as) calling
  /// [Canvas.drawPath] with the same [Paint] and the [Path] returned from
  /// [getOuterPath], and must also override [preferPaintInterior] to
  /// return true.
  ///
  /// For example, a shape that draws a rectangle might implement
  /// [getOuterPath], [paintInterior], and [preferPaintInterior] as follows:
  ///
  /// ```dart
  /// class RectangleBorder extends OutlinedBorder {
  ///   // ...
  ///
  ///   @override
  ///   Path getOuterPath(Rect rect, { TextDirection? textDirection }) {
  ///    return Path()
  ///      ..addRect(rect);
  ///   }
  ///
  ///   @override
  ///   void paintInterior(Canvas canvas, Rect rect, Paint paint, {TextDirection? textDirection}) {
  ///    canvas.drawRect(rect, paint);
  ///   }
  ///
  ///   @override
  ///   bool get preferPaintInterior => true;
  ///
  ///   // ...
  /// }
  /// ```
  ///
  /// When a shape can only be drawn using path, [preferPaintInterior] must
  /// return false. In that case, classes such as [ShapeDecoration] will cache
  /// the path from [getOuterPath] and call [Canvas.drawPath] directly.
  void paintInterior(Canvas canvas, Rect rect, Paint paint, {TextDirection? textDirection}) {
    assert(
      !preferPaintInterior,
      '$runtimeType.preferPaintInterior returns true but $runtimeType.paintInterior is not implemented.',
    );
    assert(
      false,
      '$runtimeType.preferPaintInterior returns false, so it is an error to call its paintInterior method.',
    );
  }

  /// Reports whether [paintInterior] is implemented.
  ///
  /// Classes such as [ShapeDecoration] prefer to use [paintInterior] if this
  /// getter returns true. This is intended to enable faster painting; instead
  /// of computing a shape using [getOuterPath] and then drawing it using
  /// [Canvas.drawPath], the path can be drawn directly to the [Canvas] using
  /// dedicated methods such as [Canvas.drawRect] or [Canvas.drawCircle].
  ///
  /// By default, this getter returns false.
  ///
  /// Subclasses that implement [paintInterior] should override this to return
  /// true. Subclasses should only override [paintInterior] if doing so enables
  /// faster rendering than is possible with [Canvas.drawPath] (so, in
  /// particular, subclasses should not call [Canvas.drawPath] in
  /// [paintInterior]).
  ///
  /// See also:
  ///
  ///  * [paintInterior], whose API documentation has an example implementation.
  bool get preferPaintInterior => false;

  /// Paints the border within the given [Rect] on the given [Canvas].
  ///
  /// The `textDirection` argument must be provided and non-null if the border
  /// has a text direction dependency (for example if it is expressed in terms
  /// of "start" and "end" instead of "left" and "right"). It may be null if
  /// the border will not need the text direction to paint itself.
  void paint(Canvas canvas, Rect rect, {TextDirection? textDirection});

  @override
  String toString() {
    return '${objectRuntimeType(this, 'ShapeBorder')}()';
  }
}

/// A ShapeBorder that draws an outline with the width and color specified
/// by [side].
@immutable
abstract class OutlinedBorder extends ShapeBorder {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const OutlinedBorder({this.side = BorderSide.none});

  @override
  EdgeInsetsGeometry get dimensions => EdgeInsets.all(math.max(side.strokeInset, 0));

  /// The border outline's color and weight.
  ///
  /// If [side] is [BorderSide.none], which is the default, an outline is not drawn.
  /// Otherwise the outline is centered over the shape's boundary.
  final BorderSide side;

  /// Returns a copy of this OutlinedBorder that draws its outline with the
  /// specified [side], if [side] is non-null.
  OutlinedBorder copyWith({BorderSide? side});

  @override
  ShapeBorder scale(double t);

  @override
  ShapeBorder? lerpFrom(ShapeBorder? a, double t) {
    if (a == null) {
      return scale(t);
    }
    return null;
  }

  @override
  ShapeBorder? lerpTo(ShapeBorder? b, double t) {
    if (b == null) {
      return scale(1.0 - t);
    }
    return null;
  }

  /// Linearly interpolates between two [OutlinedBorder]s.
  ///
  /// This defers to `b`'s [lerpTo] function if `b` is not null. If `b` is
  /// null or if its [lerpTo] returns null, it uses `a`'s [lerpFrom]
  /// function instead. If both return null, it returns `a` before `t=0.5`
  /// and `b` after `t=0.5`.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static OutlinedBorder? lerp(OutlinedBorder? a, OutlinedBorder? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    final ShapeBorder? result = b?.lerpFrom(a, t) ?? a?.lerpTo(b, t);
    return result as OutlinedBorder? ?? (t < 0.5 ? a : b);
  }
}

/// Represents the addition of two otherwise-incompatible borders.
///
/// The borders are listed from the outside to the inside.
class _CompoundBorder extends ShapeBorder {
  _CompoundBorder(this.borders)
    : assert(borders.length >= 2),
      assert(!borders.any((ShapeBorder border) => border is _CompoundBorder));

  final List<ShapeBorder> borders;

  @override
  EdgeInsetsGeometry get dimensions {
    return borders.fold<EdgeInsetsGeometry>(EdgeInsets.zero, (
      EdgeInsetsGeometry previousValue,
      ShapeBorder border,
    ) {
      return previousValue.add(border.dimensions);
    });
  }

  @override
  ShapeBorder add(ShapeBorder other, {bool reversed = false}) {
    // This wraps the list of borders with "other", or, if "reversed" is true,
    // wraps "other" with the list of borders.
    // If "reversed" is false, "other" should end up being at the start of the
    // list, otherwise, if "reversed" is true, it should end up at the end.
    // First, see if we can merge the new adjacent borders.
    if (other is! _CompoundBorder) {
      // Here, "ours" is the border at the side where we're adding the new
      // border, and "merged" is the result of attempting to merge it with the
      // new border. If it's null, it couldn't be merged.
      final ShapeBorder ours = reversed ? borders.last : borders.first;
      final ShapeBorder? merged =
          ours.add(other, reversed: reversed) ?? other.add(ours, reversed: !reversed);
      if (merged != null) {
        final List<ShapeBorder> result = <ShapeBorder>[...borders];
        result[reversed ? result.length - 1 : 0] = merged;
        return _CompoundBorder(result);
      }
    }
    // We can't, so fall back to just adding the new border to the list.
    final List<ShapeBorder> mergedBorders = <ShapeBorder>[
      if (reversed) ...borders,
      if (other is _CompoundBorder) ...other.borders else other,
      if (!reversed) ...borders,
    ];
    return _CompoundBorder(mergedBorders);
  }

  @override
  ShapeBorder scale(double t) {
    return _CompoundBorder(
      borders.map<ShapeBorder>((ShapeBorder border) => border.scale(t)).toList(),
    );
  }

  @override
  ShapeBorder? lerpFrom(ShapeBorder? a, double t) {
    return _CompoundBorder.lerp(a, this, t);
  }

  @override
  ShapeBorder? lerpTo(ShapeBorder? b, double t) {
    return _CompoundBorder.lerp(this, b, t);
  }

  static _CompoundBorder lerp(ShapeBorder? a, ShapeBorder? b, double t) {
    assert(
      a is _CompoundBorder || b is _CompoundBorder,
    ); // Not really necessary, but all call sites currently intend this.
    final List<ShapeBorder?> aList = a is _CompoundBorder ? a.borders : <ShapeBorder?>[a];
    final List<ShapeBorder?> bList = b is _CompoundBorder ? b.borders : <ShapeBorder?>[b];
    final List<ShapeBorder> results = <ShapeBorder>[];
    final int length = math.max(aList.length, bList.length);
    for (int index = 0; index < length; index += 1) {
      final ShapeBorder? localA = index < aList.length ? aList[index] : null;
      final ShapeBorder? localB = index < bList.length ? bList[index] : null;
      if (localA != null && localB != null) {
        final ShapeBorder? localResult = localA.lerpTo(localB, t) ?? localB.lerpFrom(localA, t);
        if (localResult != null) {
          results.add(localResult);
          continue;
        }
      }
      // If we're changing from one shape to another, make sure the shape that is coming in
      // is inserted before the shape that is going away, so that the outer path changes to
      // the new border earlier rather than later. (This affects, among other things, where
      // the ShapeDecoration class puts its background.)
      if (localB != null) {
        results.add(localB.scale(t));
      }
      if (localA != null) {
        results.add(localA.scale(1.0 - t));
      }
    }
    return _CompoundBorder(results);
  }

  @override
  Path getInnerPath(Rect rect, {TextDirection? textDirection}) {
    for (int index = 0; index < borders.length - 1; index += 1) {
      rect = borders[index].dimensions.resolve(textDirection).deflateRect(rect);
    }
    return borders.last.getInnerPath(rect, textDirection: textDirection);
  }

  @override
  Path getOuterPath(Rect rect, {TextDirection? textDirection}) {
    return borders.first.getOuterPath(rect, textDirection: textDirection);
  }

  @override
  void paintInterior(Canvas canvas, Rect rect, Paint paint, {TextDirection? textDirection}) {
    borders.first.paintInterior(canvas, rect, paint, textDirection: textDirection);
  }

  @override
  bool get preferPaintInterior => borders.every((ShapeBorder border) => border.preferPaintInterior);

  @override
  void paint(Canvas canvas, Rect rect, {TextDirection? textDirection}) {
    for (final ShapeBorder border in borders) {
      border.paint(canvas, rect, textDirection: textDirection);
      rect = border.dimensions.resolve(textDirection).deflateRect(rect);
    }
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is _CompoundBorder && listEquals<ShapeBorder>(other.borders, borders);
  }

  @override
  int get hashCode => Object.hashAll(borders);

  @override
  String toString() {
    // We list them in reverse order because when adding two borders they end up
    // in the list in the opposite order of what the source looks like: a + b =>
    // [b, a]. We do this to make the painting code more optimal, and most of
    // the rest of the code doesn't care, except toString() (for debugging).
    return borders.reversed.map<String>((ShapeBorder border) => border.toString()).join(' + ');
  }
}

/// Paints a border around the given rectangle on the canvas.
///
/// The four sides can be independently specified. They are painted in the order
/// top, right, bottom, left. This is only notable if the widths of the borders
/// and the size of the given rectangle are such that the border sides will
/// overlap each other. No effort is made to optimize the rendering of uniform
/// borders (where all the borders have the same configuration); to render a
/// uniform border, consider using [Canvas.drawRect] directly.
///
/// See also:
///
///  * [paintImage], which paints an image in a rectangle on a canvas.
///  * [Border], which uses this function to paint its border when the border is
///    not uniform.
///  * [BoxDecoration], which describes its border using the [Border] class.
void paintBorder(
  Canvas canvas,
  Rect rect, {
  BorderSide top = BorderSide.none,
  BorderSide right = BorderSide.none,
  BorderSide bottom = BorderSide.none,
  BorderSide left = BorderSide.none,
}) {
  // We draw the borders as filled shapes, unless the borders are hairline
  // borders, in which case we use PaintingStyle.stroke, with the stroke width
  // specified here.
  final Paint paint = Paint()..strokeWidth = 0.0;

  final Path path = Path();

  switch (top.style) {
    case BorderStyle.solid:
      paint.color = top.color;
      path.reset();
      path.moveTo(rect.left, rect.top);
      path.lineTo(rect.right, rect.top);
      if (top.width == 0.0) {
        paint.style = PaintingStyle.stroke;
      } else {
        paint.style = PaintingStyle.fill;
        path.lineTo(rect.right - right.width, rect.top + top.width);
        path.lineTo(rect.left + left.width, rect.top + top.width);
      }
      canvas.drawPath(path, paint);
    case BorderStyle.none:
      break;
  }

  switch (right.style) {
    case BorderStyle.solid:
      paint.color = right.color;
      path.reset();
      path.moveTo(rect.right, rect.top);
      path.lineTo(rect.right, rect.bottom);
      if (right.width == 0.0) {
        paint.style = PaintingStyle.stroke;
      } else {
        paint.style = PaintingStyle.fill;
        path.lineTo(rect.right - right.width, rect.bottom - bottom.width);
        path.lineTo(rect.right - right.width, rect.top + top.width);
      }
      canvas.drawPath(path, paint);
    case BorderStyle.none:
      break;
  }

  switch (bottom.style) {
    case BorderStyle.solid:
      paint.color = bottom.color;
      path.reset();
      path.moveTo(rect.right, rect.bottom);
      path.lineTo(rect.left, rect.bottom);
      if (bottom.width == 0.0) {
        paint.style = PaintingStyle.stroke;
      } else {
        paint.style = PaintingStyle.fill;
        path.lineTo(rect.left + left.width, rect.bottom - bottom.width);
        path.lineTo(rect.right - right.width, rect.bottom - bottom.width);
      }
      canvas.drawPath(path, paint);
    case BorderStyle.none:
      break;
  }

  switch (left.style) {
    case BorderStyle.solid:
      paint.color = left.color;
      path.reset();
      path.moveTo(rect.left, rect.bottom);
      path.lineTo(rect.left, rect.top);
      if (left.width == 0.0) {
        paint.style = PaintingStyle.stroke;
      } else {
        paint.style = PaintingStyle.fill;
        path.lineTo(rect.left + left.width, rect.top + top.width);
        path.lineTo(rect.left + left.width, rect.bottom - bottom.width);
      }
      canvas.drawPath(path, paint);
    case BorderStyle.none:
      break;
  }
}
