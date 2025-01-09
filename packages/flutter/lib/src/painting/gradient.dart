// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'box_decoration.dart';
library;

import 'dart:collection';
import 'dart:math' as math;
import 'dart:ui' as ui show Gradient, lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:vector_math/vector_math_64.dart';

import 'alignment.dart';
import 'basic_types.dart';

class _ColorsAndStops {
  _ColorsAndStops(this.colors, this.stops);
  final List<Color> colors;
  final List<double> stops;
}

/// Calculate the color at position [t] of the gradient defined by [colors] and [stops].
Color _sample(List<Color> colors, List<double> stops, double t) {
  assert(colors.isNotEmpty);
  assert(stops.isNotEmpty);
  if (t <= stops.first) {
    return colors.first;
  }
  if (t >= stops.last) {
    return colors.last;
  }
  final int index = stops.lastIndexWhere((double s) => s <= t);
  assert(index != -1);
  return Color.lerp(
    colors[index],
    colors[index + 1],
    (t - stops[index]) / (stops[index + 1] - stops[index]),
  )!;
}

_ColorsAndStops _interpolateColorsAndStops(
  List<Color> aColors,
  List<double> aStops,
  List<Color> bColors,
  List<double> bStops,
  double t,
) {
  assert(aColors.length >= 2);
  assert(bColors.length >= 2);
  assert(aStops.length == aColors.length);
  assert(bStops.length == bColors.length);
  final SplayTreeSet<double> stops =
      SplayTreeSet<double>()
        ..addAll(aStops)
        ..addAll(bStops);
  final List<double> interpolatedStops = stops.toList(growable: false);
  final List<Color> interpolatedColors = interpolatedStops
      .map<Color>(
        (double stop) =>
            Color.lerp(_sample(aColors, aStops, stop), _sample(bColors, bStops, stop), t)!,
      )
      .toList(growable: false);
  return _ColorsAndStops(interpolatedColors, interpolatedStops);
}

/// Base class for transforming gradient shaders without applying the same
/// transform to the entire canvas.
///
/// For example, a [SweepGradient] normally starts its gradation at 3 o'clock
/// and draws clockwise. To have the sweep appear to start at 6 o'clock, supply
/// a [GradientRotation] of `pi/4` radians (i.e. 45 degrees).
@immutable
abstract class GradientTransform {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const GradientTransform();

  /// When a [Gradient] creates its [Shader], it will call this method to
  /// determine what transform to apply to the shader for the given [Rect] and
  /// [TextDirection].
  ///
  /// Implementers may return null from this method, which achieves the same
  /// final effect as returning [Matrix4.identity].
  Matrix4? transform(Rect bounds, {TextDirection? textDirection});
}

/// A [GradientTransform] that rotates the gradient around the center-point of
/// its bounding box.
///
/// {@tool snippet}
///
/// This sample would rotate a sweep gradient by a quarter turn clockwise:
///
/// ```dart
/// const SweepGradient gradient = SweepGradient(
///   colors: <Color>[Color(0xFFFFFFFF), Color(0xFF009900)],
///   transform: GradientRotation(math.pi/4),
/// );
/// ```
/// {@end-tool}
@immutable
class GradientRotation extends GradientTransform {
  /// Constructs a [GradientRotation] for the specified angle.
  ///
  /// The angle is in radians in the clockwise direction.
  const GradientRotation(this.radians);

  /// The angle of rotation in radians in the clockwise direction.
  final double radians;

  @override
  Matrix4 transform(Rect bounds, {TextDirection? textDirection}) {
    final double sinRadians = math.sin(radians);
    final double oneMinusCosRadians = 1 - math.cos(radians);
    final Offset center = bounds.center;
    final double originX = sinRadians * center.dy + oneMinusCosRadians * center.dx;
    final double originY = -sinRadians * center.dx + oneMinusCosRadians * center.dy;

    return Matrix4.identity()
      ..translate(originX, originY)
      ..rotateZ(radians);
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is GradientRotation && other.radians == radians;
  }

  @override
  int get hashCode => radians.hashCode;

  @override
  String toString() {
    return '${objectRuntimeType(this, 'GradientRotation')}(radians: ${debugFormatDouble(radians)})';
  }
}

/// A 2D gradient.
///
/// This is an interface that allows [LinearGradient], [RadialGradient], and
/// [SweepGradient] classes to be used interchangeably in [BoxDecoration]s.
///
/// See also:
///
///  * [Gradient](dart-ui/Gradient-class.html), the class in the [dart:ui] library.
///
@immutable
abstract class Gradient {
  /// Initialize the gradient's colors and stops.
  ///
  /// The [colors] argument must have at least two colors (the length is not
  /// verified until the [createShader] method is called).
  ///
  /// If specified, the [stops] argument must have the same number of entries as
  /// [colors] (this is also not verified until the [createShader] method is
  /// called).
  ///
  /// The [transform] argument can be applied to transform _only_ the gradient,
  /// without rotating the canvas itself or other geometry on the canvas. For
  /// example, a `GradientRotation(math.pi/4)` will result in a [SweepGradient]
  /// that starts from a position of 6 o'clock instead of 3 o'clock, assuming
  /// no other rotation or perspective transformations have been applied to the
  /// [Canvas]. If null, no transformation is applied.
  const Gradient({required this.colors, this.stops, this.transform});

  /// The colors the gradient should obtain at each of the stops.
  ///
  /// If [stops] is non-null, this list must have the same length as [stops].
  ///
  /// This list must have at least two colors in it (otherwise, it's not a
  /// gradient!).
  final List<Color> colors;

  /// A list of values from 0.0 to 1.0 that denote fractions along the gradient.
  ///
  /// If non-null, this list must have the same length as [colors].
  ///
  /// If the first value is not 0.0, then a stop with position 0.0 and a color
  /// equal to the first color in [colors] is implied.
  ///
  /// If the last value is not 1.0, then a stop with position 1.0 and a color
  /// equal to the last color in [colors] is implied.
  ///
  /// The values in the [stops] list must be in ascending order. If a value in
  /// the [stops] list is less than an earlier value in the list, then its value
  /// is assumed to equal the previous value.
  ///
  /// If stops is null, then a set of uniformly distributed stops is implied,
  /// with the first stop at 0.0 and the last stop at 1.0.
  final List<double>? stops;

  /// The transform, if any, to apply to the gradient.
  ///
  /// This transform is in addition to any other transformations applied to the
  /// canvas, but does not add any transformations to the canvas.
  final GradientTransform? transform;

  List<double> _impliedStops() {
    if (stops != null) {
      return stops!;
    }
    assert(colors.length >= 2, 'colors list must have at least two colors');
    final double separation = 1.0 / (colors.length - 1);
    return List<double>.generate(colors.length, (int index) => index * separation, growable: false);
  }

  /// Creates a [Shader] for this gradient to fill the given rect.
  ///
  /// If the gradient's configuration is text-direction-dependent, for example
  /// it uses [AlignmentDirectional] objects instead of [Alignment]
  /// objects, then the `textDirection` argument must not be null.
  ///
  /// The shader's transform will be resolved from the [transform] of this
  /// gradient.
  @factory
  Shader createShader(Rect rect, {TextDirection? textDirection});

  /// Returns a new gradient with its properties scaled by the given factor.
  ///
  /// A factor of 0.0 (or less) should result in a variant of the gradient that
  /// is invisible; any two factors epsilon apart should be unnoticeably
  /// different from each other at first glance. From this it follows that
  /// scaling a gradient with values from 1.0 to 0.0 over time should cause the
  /// gradient to smoothly disappear.
  ///
  /// Typically this is the same as interpolating from null (with [lerp]).
  Gradient scale(double factor);

  /// Returns a new [Gradient] with each color set to the given opacity.
  Gradient withOpacity(double opacity);

  /// Linearly interpolates from another [Gradient] to `this`.
  ///
  /// When implementing this method in subclasses, return null if this class
  /// cannot interpolate from `a`. In that case, [lerp] will try `a`'s [lerpTo]
  /// method instead.
  ///
  /// If `a` is null, this must not return null. The base class implements this
  /// by deferring to [scale].
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
  /// Instead of calling this directly, use [Gradient.lerp].
  @protected
  Gradient? lerpFrom(Gradient? a, double t) {
    if (a == null) {
      return scale(t);
    }
    return null;
  }

  /// Linearly interpolates from `this` to another [Gradient].
  ///
  /// This is called if `b`'s [lerpTo] did not know how to handle this class.
  ///
  /// When implementing this method in subclasses, return null if this class
  /// cannot interpolate from `b`. In that case, [lerp] will apply a default
  /// behavior instead.
  ///
  /// If `b` is null, this must not return null. The base class implements this
  /// by deferring to [scale].
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
  /// Instead of calling this directly, use [Gradient.lerp].
  @protected
  Gradient? lerpTo(Gradient? b, double t) {
    if (b == null) {
      return scale(1.0 - t);
    }
    return null;
  }

  /// Linearly interpolates between two [Gradient]s.
  ///
  /// This defers to `b`'s [lerpTo] function if `b` is not null. If `b` is
  /// null or if its [lerpTo] returns null, it uses `a`'s [lerpFrom]
  /// function instead. If both return null, it returns `a` before `t == 0.5`
  /// and `b` after `t == 0.5`.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static Gradient? lerp(Gradient? a, Gradient? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    Gradient? result;
    if (b != null) {
      result = b.lerpFrom(a, t); // if a is null, this must return non-null
    }
    if (result == null && a != null) {
      result = a.lerpTo(b, t); // if b is null, this must return non-null
    }
    if (result != null) {
      return result;
    }
    assert(a != null && b != null);
    return t < 0.5 ? a!.scale(1.0 - (t * 2.0)) : b!.scale((t - 0.5) * 2.0);
  }

  Float64List? _resolveTransform(Rect bounds, TextDirection? textDirection) {
    return transform?.transform(bounds, textDirection: textDirection)?.storage;
  }
}

/// A 2D linear gradient.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=gYNTcgZVcWw}
///
/// This class is used by [BoxDecoration] to represent linear gradients. This
/// abstracts out the arguments to the [ui.Gradient.linear] constructor from
/// the `dart:ui` library.
///
/// A gradient has two anchor points, [begin] and [end]. The [begin] point
/// corresponds to 0.0, and the [end] point corresponds to 1.0. These points are
/// expressed in fractions, so that the same gradient can be reused with varying
/// sized boxes without changing the parameters. (This contrasts with [
/// ui.Gradient.linear], whose arguments are expressed in logical pixels.)
///
/// The [colors] are described by a list of [Color] objects. There must be at
/// least two colors. The [stops] list, if specified, must have the same length
/// as [colors]. It specifies fractions of the vector from start to end, between
/// 0.0 and 1.0, for each color. If it is null, a uniform distribution is
/// assumed.
///
/// The region of the canvas before [begin] and after [end] is colored according
/// to [tileMode].
///
/// Typically this class is used with [BoxDecoration], which does the painting.
/// To use a [LinearGradient] to paint on a canvas directly, see [createShader].
///
/// {@tool dartpad}
/// This sample draws a picture with a gradient sweeping through different
/// colors, by having a [Container] display a [BoxDecoration] with a
/// [LinearGradient].
///
/// ** See code in examples/api/lib/painting/gradient/linear_gradient.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [RadialGradient], which displays a gradient in concentric circles, and
///    has an example which shows a different way to use [Gradient] objects.
///  * [SweepGradient], which displays a gradient in a sweeping arc around a
///    center point.
///  * [BoxDecoration], which can take a [LinearGradient] in its
///    [BoxDecoration.gradient] property.
class LinearGradient extends Gradient {
  /// Creates a linear gradient.
  ///
  /// If [stops] is non-null, it must have the same length as [colors].
  const LinearGradient({
    this.begin = Alignment.centerLeft,
    this.end = Alignment.centerRight,
    required super.colors,
    super.stops,
    this.tileMode = TileMode.clamp,
    super.transform,
  });

  /// The offset at which stop 0.0 of the gradient is placed.
  ///
  /// If this is an [Alignment], then it is expressed as a vector from
  /// coordinate (0.0, 0.0), in a coordinate space that maps the center of the
  /// paint box at (0.0, 0.0) and the bottom right at (1.0, 1.0).
  ///
  /// For example, a begin offset of (-1.0, 0.0) is half way down the
  /// left side of the box.
  ///
  /// It can also be an [AlignmentDirectional], where the start is the
  /// left in left-to-right contexts and the right in right-to-left contexts. If
  /// a text-direction-dependent value is provided here, then the [createShader]
  /// method will need to be given a [TextDirection].
  final AlignmentGeometry begin;

  /// The offset at which stop 1.0 of the gradient is placed.
  ///
  /// If this is an [Alignment], then it is expressed as a vector from
  /// coordinate (0.0, 0.0), in a coordinate space that maps the center of the
  /// paint box at (0.0, 0.0) and the bottom right at (1.0, 1.0).
  ///
  /// For example, a begin offset of (1.0, 0.0) is half way down the
  /// right side of the box.
  ///
  /// It can also be an [AlignmentDirectional], where the start is the left in
  /// left-to-right contexts and the right in right-to-left contexts. If a
  /// text-direction-dependent value is provided here, then the [createShader]
  /// method will need to be given a [TextDirection].
  final AlignmentGeometry end;

  /// How this gradient should tile the plane beyond in the region before
  /// [begin] and after [end].
  ///
  /// For details, see [TileMode].
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/dart-ui/tile_mode_clamp_linear.png)
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/dart-ui/tile_mode_decal_linear.png)
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/dart-ui/tile_mode_mirror_linear.png)
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/dart-ui/tile_mode_repeated_linear.png)
  final TileMode tileMode;

  @override
  Shader createShader(Rect rect, {TextDirection? textDirection}) {
    return ui.Gradient.linear(
      begin.resolve(textDirection).withinRect(rect),
      end.resolve(textDirection).withinRect(rect),
      colors,
      _impliedStops(),
      tileMode,
      _resolveTransform(rect, textDirection),
    );
  }

  /// Returns a new [LinearGradient] with its colors scaled by the given factor.
  ///
  /// Since the alpha component of the Color is what is scaled, a factor
  /// of 0.0 or less results in a gradient that is fully transparent.
  @override
  LinearGradient scale(double factor) {
    return LinearGradient(
      begin: begin,
      end: end,
      colors: colors.map<Color>((Color color) => Color.lerp(null, color, factor)!).toList(),
      stops: stops,
      tileMode: tileMode,
    );
  }

  @override
  Gradient? lerpFrom(Gradient? a, double t) {
    if (a is LinearGradient?) {
      return LinearGradient.lerp(a, this, t);
    }
    return super.lerpFrom(a, t);
  }

  @override
  Gradient? lerpTo(Gradient? b, double t) {
    if (b is LinearGradient?) {
      return LinearGradient.lerp(this, b, t);
    }
    return super.lerpTo(b, t);
  }

  /// Linearly interpolate between two [LinearGradient]s.
  ///
  /// If either gradient is null, this function linearly interpolates from
  /// a gradient that matches the other gradient in [begin], [end], [stops] and
  /// [tileMode] and with the same [colors] but transparent (using [scale]).
  ///
  /// If neither gradient is null, they must have the same number of [colors].
  ///
  /// The `t` argument represents a position on the timeline, with 0.0 meaning
  /// that the interpolation has not started, returning `a` (or something
  /// equivalent to `a`), 1.0 meaning that the interpolation has finished,
  /// returning `b` (or something equivalent to `b`), and values in between
  /// meaning that the interpolation is at the relevant point on the timeline
  /// between `a` and `b`. The interpolation can be extrapolated beyond 0.0 and
  /// 1.0, so negative values and values greater than 1.0 are valid (and can
  /// easily be generated by curves such as [Curves.elasticInOut]).
  ///
  /// Values for `t` are usually obtained from an [Animation<double>], such as
  /// an [AnimationController].
  static LinearGradient? lerp(LinearGradient? a, LinearGradient? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    if (a == null) {
      return b!.scale(t);
    }
    if (b == null) {
      return a.scale(1.0 - t);
    }
    final _ColorsAndStops interpolated = _interpolateColorsAndStops(
      a.colors,
      a._impliedStops(),
      b.colors,
      b._impliedStops(),
      t,
    );
    return LinearGradient(
      begin: AlignmentGeometry.lerp(a.begin, b.begin, t)!,
      end: AlignmentGeometry.lerp(a.end, b.end, t)!,
      colors: interpolated.colors,
      stops: interpolated.stops,
      tileMode: t < 0.5 ? a.tileMode : b.tileMode, // TODO(ianh): interpolate tile mode
      transform: t < 0.5 ? a.transform : b.transform,
    );
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is LinearGradient &&
        other.begin == begin &&
        other.end == end &&
        other.tileMode == tileMode &&
        other.transform == transform &&
        listEquals<Color>(other.colors, colors) &&
        listEquals<double>(other.stops, stops);
  }

  @override
  int get hashCode => Object.hash(
    begin,
    end,
    tileMode,
    transform,
    Object.hashAll(colors),
    stops == null ? null : Object.hashAll(stops!),
  );

  @override
  String toString() {
    final List<String> description = <String>[
      'begin: $begin',
      'end: $end',
      'colors: $colors',
      if (stops != null) 'stops: $stops',
      'tileMode: $tileMode',
      if (transform != null) 'transform: $transform',
    ];

    return '${objectRuntimeType(this, 'LinearGradient')}(${description.join(', ')})';
  }

  @override
  LinearGradient withOpacity(double opacity) {
    return LinearGradient(
      begin: begin,
      end: end,
      colors: <Color>[for (final Color color in colors) color.withOpacity(opacity)],
      stops: stops,
      tileMode: tileMode,
      transform: transform,
    );
  }
}

/// A 2D radial gradient.
///
/// This class is used by [BoxDecoration] to represent radial gradients. This
/// abstracts out the arguments to the [ui.Gradient.radial] constructor from
/// the `dart:ui` library.
///
/// A normal radial gradient has a [center] and a [radius]. The [center] point
/// corresponds to 0.0, and the ring at [radius] from the center corresponds
/// to 1.0. These lengths are expressed in fractions, so that the same gradient
/// can be reused with varying sized boxes without changing the parameters.
/// (This contrasts with [ui.Gradient.radial], whose arguments are expressed
/// in logical pixels.)
///
/// It is also possible to create a two-point (or focal pointed) radial gradient
/// (which is sometimes referred to as a two point conic gradient, but is not the
/// same as a CSS conic gradient which corresponds to a [SweepGradient]). A [focal]
/// point and [focalRadius] can be specified similarly to [center] and [radius],
/// which will make the rendered gradient appear to be pointed or directed in the
/// direction of the [focal] point. This is only important if [focal] and [center]
/// are not equal or [focalRadius] > 0.0 (as this case is visually identical to a
/// normal radial gradient). One important case to avoid is having [focal] and
/// [center] both resolve to [Offset.zero] when [focalRadius] > 0.0. In such a case,
/// a valid shader cannot be created by the framework.
///
/// The [colors] are described by a list of [Color] objects. There must be at
/// least two colors. The [stops] list, if specified, must have the same length
/// as [colors]. It specifies fractions of the radius between 0.0 and 1.0,
/// giving concentric rings for each color stop. If it is null, a uniform
/// distribution is assumed.
///
/// The region of the canvas beyond [radius] from the [center] is colored
/// according to [tileMode].
///
/// Typically this class is used with [BoxDecoration], which does the painting.
/// To use a [RadialGradient] to paint on a canvas directly, see [createShader].
///
/// {@tool snippet}
///
/// This function draws a gradient that looks like a sun in a blue sky.
///
/// ```dart
/// void paintSky(Canvas canvas, Rect rect) {
///   const RadialGradient gradient = RadialGradient(
///     center: Alignment(0.7, -0.6), // near the top right
///     radius: 0.2,
///     colors: <Color>[
///       Color(0xFFFFFF00), // yellow sun
///       Color(0xFF0099FF), // blue sky
///     ],
///     stops: <double>[0.4, 1.0],
///   );
///   // rect is the area we are painting over
///   final Paint paint = Paint()
///     ..shader = gradient.createShader(rect);
///   canvas.drawRect(rect, paint);
/// }
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [LinearGradient], which displays a gradient in parallel lines, and has an
///    example which shows a different way to use [Gradient] objects.
///  * [SweepGradient], which displays a gradient in a sweeping arc around a
///    center point.
///  * [BoxDecoration], which can take a [RadialGradient] in its
///    [BoxDecoration.gradient] property.
///  * [CustomPainter], which shows how to use the above sample code in a custom
///    painter.
class RadialGradient extends Gradient {
  /// Creates a radial gradient.
  ///
  /// If [stops] is non-null, it must have the same length as [colors].
  const RadialGradient({
    this.center = Alignment.center,
    this.radius = 0.5,
    required super.colors,
    super.stops,
    this.tileMode = TileMode.clamp,
    this.focal,
    this.focalRadius = 0.0,
    super.transform,
  });

  /// The center of the gradient, as an offset into the (-1.0, -1.0) x (1.0, 1.0)
  /// square describing the gradient which will be mapped onto the paint box.
  ///
  /// For example, an alignment of (0.0, 0.0) will place the radial
  /// gradient in the center of the box.
  ///
  /// If this is an [Alignment], then it is expressed as a vector from
  /// coordinate (0.0, 0.0), in a coordinate space that maps the center of the
  /// paint box at (0.0, 0.0) and the bottom right at (1.0, 1.0).
  ///
  /// It can also be an [AlignmentDirectional], where the start is the left in
  /// left-to-right contexts and the right in right-to-left contexts. If a
  /// text-direction-dependent value is provided here, then the [createShader]
  /// method will need to be given a [TextDirection].
  final AlignmentGeometry center;

  /// The radius of the gradient, as a fraction of the shortest side
  /// of the paint box.
  ///
  /// For example, if a radial gradient is painted on a box that is
  /// 100.0 pixels wide and 200.0 pixels tall, then a radius of 1.0
  /// will place the 1.0 stop at 100.0 pixels from the [center].
  final double radius;

  /// How this gradient should tile the plane beyond the outer ring at [radius]
  /// pixels from the [center].
  ///
  /// For details, see [TileMode].
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/dart-ui/tile_mode_clamp_radial.png)
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/dart-ui/tile_mode_decal_radial.png)
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/dart-ui/tile_mode_mirror_radial.png)
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/dart-ui/tile_mode_repeated_radial.png)
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/dart-ui/tile_mode_clamp_radialWithFocal.png)
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/dart-ui/tile_mode_decal_radialWithFocal.png)
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/dart-ui/tile_mode_mirror_radialWithFocal.png)
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/dart-ui/tile_mode_repeated_radialWithFocal.png)
  final TileMode tileMode;

  /// The focal point of the gradient. If specified, the gradient will appear
  /// to be focused along the vector from [center] to focal.
  ///
  /// See [center] for a description of how the coordinates are mapped.
  ///
  /// If this value is specified and [focalRadius] > 0.0, care should be taken
  /// to ensure that either this value or [center] will not both resolve to
  /// [Offset.zero], which would fail to create a valid gradient.
  final AlignmentGeometry? focal;

  /// The radius of the focal point of gradient, as a fraction of the shortest
  /// side of the paint box.
  ///
  /// For example, if a radial gradient is painted on a box that is
  /// 100.0 pixels wide and 200.0 pixels tall, then a radius of 1.0
  /// will place the 1.0 stop at 100.0 pixels from the [focal] point.
  ///
  /// If this value is specified and is greater than 0.0, either [focal] or
  /// [center] must not resolve to [Offset.zero], which would fail to create
  /// a valid gradient.
  final double focalRadius;

  @override
  Shader createShader(Rect rect, {TextDirection? textDirection}) {
    return ui.Gradient.radial(
      center.resolve(textDirection).withinRect(rect),
      radius * rect.shortestSide,
      colors,
      _impliedStops(),
      tileMode,
      _resolveTransform(rect, textDirection),
      focal?.resolve(textDirection).withinRect(rect),
      focalRadius * rect.shortestSide,
    );
  }

  /// Returns a new [RadialGradient] with its colors scaled by the given factor.
  ///
  /// Since the alpha component of the Color is what is scaled, a factor
  /// of 0.0 or less results in a gradient that is fully transparent.
  @override
  RadialGradient scale(double factor) {
    return RadialGradient(
      center: center,
      radius: radius,
      colors: colors.map<Color>((Color color) => Color.lerp(null, color, factor)!).toList(),
      stops: stops,
      tileMode: tileMode,
      focal: focal,
      focalRadius: focalRadius,
    );
  }

  @override
  Gradient? lerpFrom(Gradient? a, double t) {
    if (a is RadialGradient?) {
      return RadialGradient.lerp(a, this, t);
    }
    return super.lerpFrom(a, t);
  }

  @override
  Gradient? lerpTo(Gradient? b, double t) {
    if (b is RadialGradient?) {
      return RadialGradient.lerp(this, b, t);
    }
    return super.lerpTo(b, t);
  }

  /// Linearly interpolate between two [RadialGradient]s.
  ///
  /// If either gradient is null, this function linearly interpolates from
  /// a gradient that matches the other gradient in [center], [radius], [stops] and
  /// [tileMode] and with the same [colors] but transparent (using [scale]).
  ///
  /// If neither gradient is null, they must have the same number of [colors].
  ///
  /// The `t` argument represents a position on the timeline, with 0.0 meaning
  /// that the interpolation has not started, returning `a` (or something
  /// equivalent to `a`), 1.0 meaning that the interpolation has finished,
  /// returning `b` (or something equivalent to `b`), and values in between
  /// meaning that the interpolation is at the relevant point on the timeline
  /// between `a` and `b`. The interpolation can be extrapolated beyond 0.0 and
  /// 1.0, so negative values and values greater than 1.0 are valid (and can
  /// easily be generated by curves such as [Curves.elasticInOut]).
  ///
  /// Values for `t` are usually obtained from an [Animation<double>], such as
  /// an [AnimationController].
  static RadialGradient? lerp(RadialGradient? a, RadialGradient? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    if (a == null) {
      return b!.scale(t);
    }
    if (b == null) {
      return a.scale(1.0 - t);
    }
    final _ColorsAndStops interpolated = _interpolateColorsAndStops(
      a.colors,
      a._impliedStops(),
      b.colors,
      b._impliedStops(),
      t,
    );
    return RadialGradient(
      center: AlignmentGeometry.lerp(a.center, b.center, t)!,
      radius: math.max(0.0, ui.lerpDouble(a.radius, b.radius, t)!),
      colors: interpolated.colors,
      stops: interpolated.stops,
      tileMode: t < 0.5 ? a.tileMode : b.tileMode, // TODO(ianh): interpolate tile mode
      focal: AlignmentGeometry.lerp(a.focal, b.focal, t),
      focalRadius: math.max(0.0, ui.lerpDouble(a.focalRadius, b.focalRadius, t)!),
      transform: t < 0.5 ? a.transform : b.transform,
    );
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is RadialGradient &&
        other.center == center &&
        other.radius == radius &&
        other.tileMode == tileMode &&
        other.transform == transform &&
        listEquals<Color>(other.colors, colors) &&
        listEquals<double>(other.stops, stops) &&
        other.focal == focal &&
        other.focalRadius == focalRadius;
  }

  @override
  int get hashCode => Object.hash(
    center,
    radius,
    tileMode,
    transform,
    Object.hashAll(colors),
    stops == null ? null : Object.hashAll(stops!),
    focal,
    focalRadius,
  );

  @override
  String toString() {
    final List<String> description = <String>[
      'center: $center',
      'radius: ${debugFormatDouble(radius)}',
      'colors: $colors',
      if (stops != null) 'stops: $stops',
      'tileMode: $tileMode',
      if (focal != null) 'focal: $focal',
      'focalRadius: ${debugFormatDouble(focalRadius)}',
      if (transform != null) 'transform: $transform',
    ];

    return '${objectRuntimeType(this, 'RadialGradient')}(${description.join(', ')})';
  }

  @override
  RadialGradient withOpacity(double opacity) {
    return RadialGradient(
      center: center,
      radius: radius,
      colors: <Color>[for (final Color color in colors) color.withOpacity(opacity)],
      stops: stops,
      tileMode: tileMode,
      focal: focal,
      focalRadius: focalRadius,
      transform: transform,
    );
  }
}

/// A 2D sweep gradient.
///
/// This class is used by [BoxDecoration] to represent sweep gradients. This
/// abstracts out the arguments to the [ui.Gradient.sweep] constructor from
/// the `dart:ui` library.
///
/// A gradient has a [center], a [startAngle], and an [endAngle]. The [startAngle]
/// corresponds to 0.0, and the [endAngle] corresponds to 1.0. These angles are
/// expressed in radians.
///
/// The [colors] are described by a list of [Color] objects. There must be at
/// least two colors. The [stops] list, if specified, must have the same length
/// as [colors]. It specifies fractions of the vector from start to end, between
/// 0.0 and 1.0, for each color. If it is null, a uniform distribution is
/// assumed.
///
/// The region of the canvas before [startAngle] and after [endAngle] is colored
/// according to [tileMode].
///
/// Typically this class is used with [BoxDecoration], which does the painting.
/// To use a [SweepGradient] to paint on a canvas directly, see [createShader].
///
/// {@tool snippet}
///
/// This sample draws a different color in each quadrant.
///
/// ```dart
/// Container(
///   decoration: const BoxDecoration(
///     gradient: SweepGradient(
///       center: FractionalOffset.center,
///       colors: <Color>[
///         Color(0xFF4285F4), // blue
///         Color(0xFF34A853), // green
///         Color(0xFFFBBC05), // yellow
///         Color(0xFFEA4335), // red
///         Color(0xFF4285F4), // blue again to seamlessly transition to the start
///       ],
///       stops: <double>[0.0, 0.25, 0.5, 0.75, 1.0],
///     ),
///   )
/// )
/// ```
/// {@end-tool}
///
/// {@tool snippet}
///
/// This sample takes the above gradient and rotates it by `math.pi/4` radians,
/// i.e. 45 degrees.
///
/// ```dart
/// Container(
///   decoration: const BoxDecoration(
///     gradient: SweepGradient(
///       center: FractionalOffset.center,
///       colors: <Color>[
///         Color(0xFF4285F4), // blue
///         Color(0xFF34A853), // green
///         Color(0xFFFBBC05), // yellow
///         Color(0xFFEA4335), // red
///         Color(0xFF4285F4), // blue again to seamlessly transition to the start
///       ],
///       stops: <double>[0.0, 0.25, 0.5, 0.75, 1.0],
///       transform: GradientRotation(math.pi/4),
///     ),
///   ),
/// )
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [LinearGradient], which displays a gradient in parallel lines, and has an
///    example which shows a different way to use [Gradient] objects.
///  * [RadialGradient], which displays a gradient in concentric circles, and
///    has an example which shows a different way to use [Gradient] objects.
///  * [BoxDecoration], which can take a [SweepGradient] in its
///    [BoxDecoration.gradient] property.
class SweepGradient extends Gradient {
  /// Creates a sweep gradient.
  ///
  /// If [stops] is non-null, it must have the same length as [colors].
  const SweepGradient({
    this.center = Alignment.center,
    this.startAngle = 0.0,
    this.endAngle = math.pi * 2,
    required super.colors,
    super.stops,
    this.tileMode = TileMode.clamp,
    super.transform,
  });

  /// The center of the gradient, as an offset into the (-1.0, -1.0) x (1.0, 1.0)
  /// square describing the gradient which will be mapped onto the paint box.
  ///
  /// For example, an alignment of (0.0, 0.0) will place the sweep
  /// gradient in the center of the box.
  ///
  /// If this is an [Alignment], then it is expressed as a vector from
  /// coordinate (0.0, 0.0), in a coordinate space that maps the center of the
  /// paint box at (0.0, 0.0) and the bottom right at (1.0, 1.0).
  ///
  /// It can also be an [AlignmentDirectional], where the start is the left in
  /// left-to-right contexts and the right in right-to-left contexts. If a
  /// text-direction-dependent value is provided here, then the [createShader]
  /// method will need to be given a [TextDirection].
  final AlignmentGeometry center;

  /// The angle in radians at which stop 0.0 of the gradient is placed.
  ///
  /// Defaults to 0.0.
  final double startAngle;

  /// The angle in radians at which stop 1.0 of the gradient is placed.
  ///
  /// Defaults to math.pi * 2.
  final double endAngle;

  /// How this gradient should tile the plane beyond in the region before
  /// [startAngle] and after [endAngle].
  ///
  /// For details, see [TileMode].
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/dart-ui/tile_mode_clamp_sweep.png)
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/dart-ui/tile_mode_decal_sweep.png)
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/dart-ui/tile_mode_mirror_sweep.png)
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/dart-ui/tile_mode_repeated_sweep.png)
  final TileMode tileMode;

  @override
  Shader createShader(Rect rect, {TextDirection? textDirection}) {
    return ui.Gradient.sweep(
      center.resolve(textDirection).withinRect(rect),
      colors,
      _impliedStops(),
      tileMode,
      startAngle,
      endAngle,
      _resolveTransform(rect, textDirection),
    );
  }

  /// Returns a new [SweepGradient] with its colors scaled by the given factor.
  ///
  /// Since the alpha component of the Color is what is scaled, a factor
  /// of 0.0 or less results in a gradient that is fully transparent.
  @override
  SweepGradient scale(double factor) {
    return SweepGradient(
      center: center,
      startAngle: startAngle,
      endAngle: endAngle,
      colors: colors.map<Color>((Color color) => Color.lerp(null, color, factor)!).toList(),
      stops: stops,
      tileMode: tileMode,
    );
  }

  @override
  Gradient? lerpFrom(Gradient? a, double t) {
    if (a is SweepGradient?) {
      return SweepGradient.lerp(a, this, t);
    }
    return super.lerpFrom(a, t);
  }

  @override
  Gradient? lerpTo(Gradient? b, double t) {
    if (b is SweepGradient?) {
      return SweepGradient.lerp(this, b, t);
    }
    return super.lerpTo(b, t);
  }

  /// Linearly interpolate between two [SweepGradient]s.
  ///
  /// If either gradient is null, then the non-null gradient is returned with
  /// its color scaled in the same way as the [scale] function.
  ///
  /// If neither gradient is null, they must have the same number of [colors].
  ///
  /// The `t` argument represents a position on the timeline, with 0.0 meaning
  /// that the interpolation has not started, returning `a` (or something
  /// equivalent to `a`), 1.0 meaning that the interpolation has finished,
  /// returning `b` (or something equivalent to `b`), and values in between
  /// meaning that the interpolation is at the relevant point on the timeline
  /// between `a` and `b`. The interpolation can be extrapolated beyond 0.0 and
  /// 1.0, so negative values and values greater than 1.0 are valid (and can
  /// easily be generated by curves such as [Curves.elasticInOut]).
  ///
  /// Values for `t` are usually obtained from an [Animation<double>], such as
  /// an [AnimationController].
  static SweepGradient? lerp(SweepGradient? a, SweepGradient? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    if (a == null) {
      return b!.scale(t);
    }
    if (b == null) {
      return a.scale(1.0 - t);
    }
    final _ColorsAndStops interpolated = _interpolateColorsAndStops(
      a.colors,
      a._impliedStops(),
      b.colors,
      b._impliedStops(),
      t,
    );
    return SweepGradient(
      center: AlignmentGeometry.lerp(a.center, b.center, t)!,
      startAngle: math.max(0.0, ui.lerpDouble(a.startAngle, b.startAngle, t)!),
      endAngle: math.max(0.0, ui.lerpDouble(a.endAngle, b.endAngle, t)!),
      colors: interpolated.colors,
      stops: interpolated.stops,
      tileMode: t < 0.5 ? a.tileMode : b.tileMode, // TODO(ianh): interpolate tile mode
      transform: t < 0.5 ? a.transform : b.transform,
    );
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is SweepGradient &&
        other.center == center &&
        other.startAngle == startAngle &&
        other.endAngle == endAngle &&
        other.tileMode == tileMode &&
        other.transform == transform &&
        listEquals<Color>(other.colors, colors) &&
        listEquals<double>(other.stops, stops);
  }

  @override
  int get hashCode => Object.hash(
    center,
    startAngle,
    endAngle,
    tileMode,
    transform,
    Object.hashAll(colors),
    stops == null ? null : Object.hashAll(stops!),
  );

  @override
  String toString() {
    final List<String> description = <String>[
      'center: $center',
      'startAngle: ${debugFormatDouble(startAngle)}',
      'endAngle: ${debugFormatDouble(endAngle)}',
      'colors: $colors',
      if (stops != null) 'stops: $stops',
      'tileMode: $tileMode',
      if (transform != null) 'transform: $transform',
    ];

    return '${objectRuntimeType(this, 'SweepGradient')}(${description.join(', ')})';
  }

  @override
  SweepGradient withOpacity(double opacity) {
    return SweepGradient(
      center: center,
      startAngle: startAngle,
      endAngle: endAngle,
      colors: <Color>[for (final Color color in colors) color.withOpacity(opacity)],
      stops: stops,
      tileMode: tileMode,
      transform: transform,
    );
  }
}
