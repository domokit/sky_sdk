// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'dart:math' as math;
import 'dart:ui';

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';

export 'dart:ui' show Offset;

/// An abstract class providing an interface for evaluating a parametric curve.
///
/// A parametric curve transforms a parameter (hence the name) `t` along a curve
/// to the value of the curve at that value of `t`. The curve can be of
/// arbitrary dimension, but is typically a 1D, 2D, or 3D curve.
///
/// See also:
///
///  * [Curve], a 1D animation easing curve that starts at 0.0 and ends at 1.0.
///  * [Curve2D], a parametric curve that transforms the parameter to a 2D point.
abstract class ParametricCurve<T> {
  /// Abstract const constructor to enable subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const ParametricCurve();

  /// Returns the value of the curve at point `t`.
  ///
  /// This method asserts that t is between 0 and 1 before delegating to
  /// [transformInternal].
  ///
  /// It is recommended that subclasses override [transformInternal] instead of
  /// this function, as the above case is already handled in the default
  /// implementation of [transform], which delegates the remaining logic to
  /// [transformInternal].
  T transform(double t) {
    assert(t >= 0.0 && t <= 1.0, 'parametric value $t is outside of [0, 1] range.');
    return transformInternal(t);
  }

  /// Returns the value of the curve at point `t`.
  ///
  /// The given parametric value `t` will be between 0.0 and 1.0, inclusive.
  @protected
  T transformInternal(double t) {
    throw UnimplementedError();
  }

  @override
  String toString() => objectRuntimeType(this, 'ParametricCurve');
}

/// An parametric animation easing curve, i.e. a mapping of the unit interval to
/// the unit interval.
///
/// Easing curves are used to adjust the rate of change of an animation over
/// time, allowing them to speed up and slow down, rather than moving at a
/// constant rate.
///
/// A [Curve] must map t=0.0 to 0.0 and t=1.0 to 1.0.
///
/// See also:
///
///  * [Curves], a collection of common animation easing curves.
///  * [CurveTween], which can be used to apply a [Curve] to an [Animation].
///  * [Canvas.drawArc], which draws an arc, and has nothing to do with easing
///    curves.
///  * [Animatable], for a more flexible interface that maps fractions to
///    arbitrary values.
@immutable
abstract class Curve extends ParametricCurve<double> {
  /// Abstract const constructor to enable subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const Curve();

  /// Returns the value of the curve at point `t`.
  ///
  /// This function must ensure the following:
  /// - The value of `t` must be between 0.0 and 1.0
  /// - Values of `t`=0.0 and `t`=1.0 must be mapped to 0.0 and 1.0,
  /// respectively.
  ///
  /// It is recommended that subclasses override [transformInternal] instead of
  /// this function, as the above cases are already handled in the default
  /// implementation of [transform], which delegates the remaining logic to
  /// [transformInternal].
  @override
  double transform(double t) {
    if (t == 0.0 || t == 1.0) {
      return t;
    }
    return super.transform(t);
  }

  /// Returns a new curve that is the reversed inversion of this one.
  ///
  /// This is often useful with [CurvedAnimation.reverseCurve].
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_bounce_in.mp4}
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_flipped.mp4}
  ///
  /// See also:
  ///
  ///  * [FlippedCurve], the class that is used to implement this getter.
  ///  * [ReverseAnimation], which reverses an [Animation] rather than a [Curve].
  ///  * [CurvedAnimation], which can take a separate curve and reverse curve.
  Curve get flipped => FlippedCurve(this);
}

/// The identity map over the unit interval.
///
/// See [Curves.linear] for an instance of this class.
class _Linear extends Curve {
  const _Linear._();

  @override
  double transformInternal(double t) => t;
}

/// A sawtooth curve that repeats a given number of times over the unit interval.
///
/// The curve rises linearly from 0.0 to 1.0 and then falls discontinuously back
/// to 0.0 each iteration.
///
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_sawtooth.mp4}
class SawTooth extends Curve {
  /// Creates a sawtooth curve.
  const SawTooth(this.count);

  /// The number of repetitions of the sawtooth pattern in the unit interval.
  final int count;

  @override
  double transformInternal(double t) {
    t *= count;
    return t - t.truncateToDouble();
  }

  @override
  String toString() {
    return '${objectRuntimeType(this, 'SawTooth')}($count)';
  }
}

/// A curve that is 0.0 until [begin], then curved (according to [curve]) from
/// 0.0 at [begin] to 1.0 at [end], then remains 1.0 past [end].
///
/// An [Interval] can be used to delay an animation. For example, a six second
/// animation that uses an [Interval] with its [begin] set to 0.5 and its [end]
/// set to 1.0 will essentially become a three-second animation that starts
/// three seconds later.
///
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_interval.mp4}
class Interval extends Curve {
  /// Creates an interval curve.
  const Interval(this.begin, this.end, {this.curve = Curves.linear});

  /// The largest value for which this interval is 0.0.
  ///
  /// From t=0.0 to t=[begin], the interval's value is 0.0.
  final double begin;

  /// The smallest value for which this interval is 1.0.
  ///
  /// From t=[end] to t=1.0, the interval's value is 1.0.
  final double end;

  /// The curve to apply between [begin] and [end].
  final Curve curve;

  @override
  double transformInternal(double t) {
    assert(begin >= 0.0);
    assert(begin <= 1.0);
    assert(end >= 0.0);
    assert(end <= 1.0);
    assert(end >= begin);
    t = clampDouble((t - begin) / (end - begin), 0.0, 1.0);
    if (t == 0.0 || t == 1.0) {
      return t;
    }
    return curve.transform(t);
  }

  @override
  String toString() {
    if (curve is! _Linear) {
      return '${objectRuntimeType(this, 'Interval')}($begin\u22EF$end)\u27A9$curve';
    }
    return '${objectRuntimeType(this, 'Interval')}($begin\u22EF$end)';
  }
}

/// A curve that progresses according to [beginCurve] until [split], then
/// according to [endCurve].
///
/// Split curves are useful in situations where a widget must track the
/// user's finger (which requires a linear animation), but can also be flung
/// using a curve specified with the [endCurve] argument, after the finger is
/// released. In such a case, the value of [split] would be the progress
/// of the animation at the time when the finger was released.
///
/// For example, if [split] is set to 0.5, [beginCurve] is [Curves.linear],
/// and [endCurve] is [Curves.easeOutCubic], then the bottom-left quarter of the
/// curve will be a straight line, and the top-right quarter will contain the
/// entire [Curves.easeOutCubic] curve.
///
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_split.mp4}
class Split extends Curve {
  /// Creates a split curve.
  const Split(this.split, {this.beginCurve = Curves.linear, this.endCurve = Curves.easeOutCubic});

  /// The progress value separating [beginCurve] from [endCurve].
  ///
  /// The value before which the curve progresses according to [beginCurve] and
  /// after which the curve progresses according to [endCurve].
  ///
  /// When t is exactly `split`, the curve has the value `split`.
  ///
  /// Must be between 0 and 1.0, inclusively.
  final double split;

  /// The curve to use before [split] is reached.
  ///
  /// Defaults to [Curves.linear].
  final Curve beginCurve;

  /// The curve to use after [split] is reached.
  ///
  /// Defaults to [Curves.easeOutCubic].
  final Curve endCurve;

  @override
  double transform(double t) {
    assert(t >= 0.0 && t <= 1.0);
    assert(split >= 0.0 && split <= 1.0);

    if (t == 0.0 || t == 1.0) {
      return t;
    }

    if (t == split) {
      return split;
    }

    if (t < split) {
      final double curveProgress = t / split;
      final double transformed = beginCurve.transform(curveProgress);
      return lerpDouble(0, split, transformed)!;
    } else {
      final double curveProgress = (t - split) / (1 - split);
      final double transformed = endCurve.transform(curveProgress);
      return lerpDouble(split, 1, transformed)!;
    }
  }

  @override
  String toString() {
    return '${describeIdentity(this)}($split, $beginCurve, $endCurve)';
  }
}

/// A curve that is 0.0 until it hits the threshold, then it jumps to 1.0.
///
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_threshold.mp4}
class Threshold extends Curve {
  /// Creates a threshold curve.
  const Threshold(this.threshold);

  /// The value before which the curve is 0.0 and after which the curve is 1.0.
  ///
  /// When t is exactly [threshold], the curve has the value 1.0.
  final double threshold;

  @override
  double transformInternal(double t) {
    assert(threshold >= 0.0);
    assert(threshold <= 1.0);
    return t < threshold ? 0.0 : 1.0;
  }
}

/// A cubic polynomial mapping of the unit interval.
///
/// The [Curves] class contains some commonly used cubic curves:
///
///  * [Curves.fastLinearToSlowEaseIn]
///  * [Curves.ease]
///  * [Curves.easeIn]
///  * [Curves.easeInToLinear]
///  * [Curves.easeInSine]
///  * [Curves.easeInQuad]
///  * [Curves.easeInCubic]
///  * [Curves.easeInQuart]
///  * [Curves.easeInQuint]
///  * [Curves.easeInExpo]
///  * [Curves.easeInCirc]
///  * [Curves.easeInBack]
///  * [Curves.easeOut]
///  * [Curves.linearToEaseOut]
///  * [Curves.easeOutSine]
///  * [Curves.easeOutQuad]
///  * [Curves.easeOutCubic]
///  * [Curves.easeOutQuart]
///  * [Curves.easeOutQuint]
///  * [Curves.easeOutExpo]
///  * [Curves.easeOutCirc]
///  * [Curves.easeOutBack]
///  * [Curves.easeInOut]
///  * [Curves.easeInOutSine]
///  * [Curves.easeInOutQuad]
///  * [Curves.easeInOutCubic]
///  * [Curves.easeInOutQuart]
///  * [Curves.easeInOutQuint]
///  * [Curves.easeInOutExpo]
///  * [Curves.easeInOutCirc]
///  * [Curves.easeInOutBack]
///  * [Curves.fastOutSlowIn]
///  * [Curves.slowMiddle]
///
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_fast_linear_to_slow_ease_in.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_to_linear.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_sine.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_quad.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_cubic.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_quart.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_quint.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_expo.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_circ.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_back.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_linear_to_ease_out.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_sine.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_quad.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_cubic.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_quart.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_quint.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_expo.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_circ.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_back.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_sine.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_quad.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_cubic.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_quart.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_quint.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_expo.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_circ.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_back.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_fast_out_slow_in.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_slow_middle.mp4}
///
/// The [Cubic] class implements third-order Bézier curves.
///
/// See also:
///
///  * [Curves], where many more predefined curves are available.
///  * [CatmullRomCurve], a curve which passes through specific values.
class Cubic extends Curve {
  /// Creates a cubic curve.
  ///
  /// Rather than creating a new instance, consider using one of the common
  /// cubic curves in [Curves].
  const Cubic(this.a, this.b, this.c, this.d);

  /// The x coordinate of the first control point.
  ///
  /// The line through the point (0, 0) and the first control point is tangent
  /// to the curve at the point (0, 0).
  final double a;

  /// The y coordinate of the first control point.
  ///
  /// The line through the point (0, 0) and the first control point is tangent
  /// to the curve at the point (0, 0).
  final double b;

  /// The x coordinate of the second control point.
  ///
  /// The line through the point (1, 1) and the second control point is tangent
  /// to the curve at the point (1, 1).
  final double c;

  /// The y coordinate of the second control point.
  ///
  /// The line through the point (1, 1) and the second control point is tangent
  /// to the curve at the point (1, 1).
  final double d;

  static const double _cubicErrorBound = 0.001;

  double _evaluateCubic(double a, double b, double m) {
    return 3 * a * (1 - m) * (1 - m) * m + 3 * b * (1 - m) * m * m + m * m * m;
  }

  @override
  double transformInternal(double t) {
    double start = 0.0;
    double end = 1.0;
    while (true) {
      final double midpoint = (start + end) / 2;
      final double estimate = _evaluateCubic(a, c, midpoint);
      if ((t - estimate).abs() < _cubicErrorBound) {
        return _evaluateCubic(b, d, midpoint);
      }
      if (estimate < t) {
        start = midpoint;
      } else {
        end = midpoint;
      }
    }
  }

  @override
  String toString() {
    return '${objectRuntimeType(this, 'Cubic')}(${a.toStringAsFixed(2)}, ${b.toStringAsFixed(2)}, ${c.toStringAsFixed(2)}, ${d.toStringAsFixed(2)})';
  }
}

/// A cubic polynomial composed of two curves that share a common center point.
///
/// The curve runs through three points: (0,0), the [midpoint], and (1,1).
///
/// The [Curves] class contains a curve defined with this class:
/// [Curves.easeInOutCubicEmphasized].
///
/// The [ThreePointCubic] class implements third-order Bézier curves, where two
/// curves share an interior [midpoint] that the curve passes through. If the
/// control points surrounding the middle point ([b1], and [a2]) are not
/// colinear with the middle point, then the curve's derivative will have a
/// discontinuity (a cusp) at the shared middle point.
///
/// See also:
///
///  * [Curves], where many more predefined curves are available.
///  * [Cubic], which defines a single cubic polynomial.
///  * [CatmullRomCurve], a curve which passes through specific values.
class ThreePointCubic extends Curve {
  /// Creates two cubic curves that share a common control point.
  ///
  /// Rather than creating a new instance, consider using one of the common
  /// three-point cubic curves in [Curves].
  ///
  /// The arguments correspond to the control points for the two curves,
  /// including the [midpoint], but do not include the two implied end points at
  /// (0,0) and (1,1), which are fixed.
  const ThreePointCubic(this.a1, this.b1, this.midpoint, this.a2, this.b2);

  /// The coordinates of the first control point of the first curve.
  ///
  /// The line through the point (0, 0) and this control point is tangent to the
  /// curve at the point (0, 0).
  final Offset a1;

  /// The coordinates of the second control point of the first curve.
  ///
  /// The line through the [midpoint] and this control point is tangent to the
  /// curve approaching the [midpoint].
  final Offset b1;

  /// The coordinates of the middle shared point.
  ///
  /// The curve will go through this point. If the control points surrounding
  /// this middle point ([b1], and [a2]) are not colinear with this point, then
  /// the curve's derivative will have a discontinuity (a cusp) at this point.
  final Offset midpoint;

  /// The coordinates of the first control point of the second curve.
  ///
  /// The line through the [midpoint] and this control point is tangent to the
  /// curve approaching the [midpoint].
  final Offset a2;

  /// The coordinates of the second control point of the second curve.
  ///
  /// The line through the point (1, 1) and this control point is tangent to the
  /// curve at (1, 1).
  final Offset b2;

  @override
  double transformInternal(double t) {
    final bool firstCurve = t < midpoint.dx;
    final double scaleX = firstCurve ? midpoint.dx : 1.0 - midpoint.dx;
    final double scaleY = firstCurve ? midpoint.dy : 1.0 - midpoint.dy;
    final double scaledT = (t - (firstCurve ? 0.0 : midpoint.dx)) / scaleX;
    if (firstCurve) {
      return Cubic(
            a1.dx / scaleX,
            a1.dy / scaleY,
            b1.dx / scaleX,
            b1.dy / scaleY,
          ).transform(scaledT) *
          scaleY;
    } else {
      return Cubic(
                (a2.dx - midpoint.dx) / scaleX,
                (a2.dy - midpoint.dy) / scaleY,
                (b2.dx - midpoint.dx) / scaleX,
                (b2.dy - midpoint.dy) / scaleY,
              ).transform(scaledT) *
              scaleY +
          midpoint.dy;
    }
  }

  @override
  String toString() {
    return '${objectRuntimeType(this, 'ThreePointCubic($a1, $b1, $midpoint, $a2, $b2)')} ';
  }
}

/// Abstract class that defines an API for evaluating 2D parametric curves.
///
/// [Curve2D] differs from [Curve] in that the values interpolated are [Offset]
/// values instead of [double] values, hence the "2D" in the name. They both
/// take a single double `t` that has a range of 0.0 to 1.0, inclusive, as input
/// to the [transform] function . Unlike [Curve], [Curve2D] is not required to
/// map `t=0.0` and `t=1.0` to specific output values.
///
/// The interpolated `t` value given to [transform] represents the progression
/// along the curve, but it doesn't necessarily progress at a constant velocity, so
/// incrementing `t` by, say, 0.1 might move along the curve by quite a lot at one
/// part of the curve, or hardly at all in another part of the curve, depending
/// on the definition of the curve.
///
/// {@tool dartpad}
/// This example shows how to use a [Curve2D] to modify the position of a widget
/// so that it can follow an arbitrary path.
///
/// ** See code in examples/api/lib/animation/curves/curve2_d.0.dart **
/// {@end-tool}
///
abstract class Curve2D extends ParametricCurve<Offset> {
  /// Abstract const constructor to enable subclasses to provide const
  /// constructors so that they can be used in const expressions.
  const Curve2D();

  /// Generates a list of samples with a recursive subdivision until a tolerance
  /// of `tolerance` is reached.
  ///
  /// Samples are generated in order.
  ///
  /// Samples can be used to render a curve efficiently, since the samples
  /// constitute line segments which vary in size with the curvature of the
  /// curve. They can also be used to quickly approximate the value of the curve
  /// by searching for the desired range in X and linearly interpolating between
  /// samples to obtain an approximation of Y at the desired X value. The
  /// implementation of [CatmullRomCurve] uses samples for this purpose
  /// internally.
  ///
  /// The tolerance is computed as the area of a triangle formed by a new point
  /// and the preceding and following point.
  ///
  /// See also:
  ///
  ///  * Luiz Henrique de Figueire's Graphics Gem on [the algorithm](http://ariel.chronotext.org/dd/defigueiredo93adaptive.pdf).
  Iterable<Curve2DSample> generateSamples({
    double start = 0.0,
    double end = 1.0,
    double tolerance = 1e-10,
  }) {
    // The sampling algorithm is:
    // 1. Evaluate the area of the triangle (a proxy for the "flatness" of the
    //    curve) formed by two points and a test point.
    // 2. If the area of the triangle is small enough (below tolerance), then
    //    the two points form the final segment.
    // 3. If the area is still too large, divide the interval into two parts
    //    using a random subdivision point to avoid aliasing.
    // 4. Recursively sample the two parts.
    //
    // This algorithm concentrates samples in areas of high curvature.
    assert(end > start);
    // We want to pick a random seed that will keep the result stable if
    // evaluated again, so we use the first non-generated control point.
    final math.Random rand = math.Random(samplingSeed);
    bool isFlat(Offset p, Offset q, Offset r) {
      // Calculates the area of the triangle given by the three points.
      final Offset pr = p - r;
      final Offset qr = q - r;
      final double z = pr.dx * qr.dy - qr.dx * pr.dy;
      return (z * z) < tolerance;
    }

    final Curve2DSample first = Curve2DSample(start, transform(start));
    final Curve2DSample last = Curve2DSample(end, transform(end));
    final List<Curve2DSample> samples = <Curve2DSample>[first];
    void sample(Curve2DSample p, Curve2DSample q, {bool forceSubdivide = false}) {
      // Pick a random point somewhat near the center, which avoids aliasing
      // problems with periodic curves.
      final double t = p.t + (0.45 + 0.1 * rand.nextDouble()) * (q.t - p.t);
      final Curve2DSample r = Curve2DSample(t, transform(t));

      if (!forceSubdivide && isFlat(p.value, q.value, r.value)) {
        samples.add(q);
      } else {
        sample(p, r);
        sample(r, q);
      }
    }

    // If the curve starts and ends on the same point, then we force it to
    // subdivide at least once, because otherwise it will terminate immediately.
    sample(
      first,
      last,
      forceSubdivide:
          (first.value.dx - last.value.dx).abs() < tolerance &&
          (first.value.dy - last.value.dy).abs() < tolerance,
    );
    return samples;
  }

  /// Returns a seed value used by [generateSamples] to seed a random number
  /// generator to avoid sample aliasing.
  ///
  /// Subclasses should override this and provide a custom seed.
  ///
  /// The value returned should be the same each time it is called, unless the
  /// curve definition changes.
  @protected
  int get samplingSeed => 0;

  /// Returns the parameter `t` that corresponds to the given x value of the spline.
  ///
  /// This will only work properly for curves which are single-valued in x
  /// (where every value of `x` maps to only one value in 'y', i.e. the curve
  /// does not loop or curve back over itself). For curves that are not
  /// single-valued, it will return the parameter for only one of the values at
  /// the given `x` location.
  double findInverse(double x) {
    double start = 0.0;
    double end = 1.0;
    late double mid;
    double offsetToOrigin(double pos) => x - transform(pos).dx;
    // Use a binary search to find the inverse point within 1e-6, or 100
    // subdivisions, whichever comes first.
    const double errorLimit = 1e-6;
    int count = 100;
    final double startValue = offsetToOrigin(start);
    while ((end - start) / 2.0 > errorLimit && count > 0) {
      mid = (end + start) / 2.0;
      final double value = offsetToOrigin(mid);
      if (value.sign == startValue.sign) {
        start = mid;
      } else {
        end = mid;
      }
      count--;
    }
    return mid;
  }
}

/// A class that holds a sample of a 2D parametric curve, containing the [value]
/// (the X, Y coordinates) of the curve at the parametric value [t].
///
/// See also:
///
///  * [Curve2D.generateSamples], which generates samples of this type.
///  * [Curve2D], a parametric curve that maps a double parameter to a 2D location.
class Curve2DSample {
  /// Creates an object that holds a sample; used with [Curve2D] subclasses.
  const Curve2DSample(this.t, this.value);

  /// The parametric location of this sample point along the curve.
  final double t;

  /// The value (the X, Y coordinates) of the curve at parametric value [t].
  final Offset value;

  @override
  String toString() {
    return '[(${value.dx.toStringAsFixed(2)}, ${value.dy.toStringAsFixed(2)}), ${t.toStringAsFixed(2)}]';
  }
}

/// A 2D spline that passes smoothly through the given control points using a
/// centripetal Catmull-Rom spline.
///
/// When the curve is evaluated with [transform], the output values will move
/// smoothly from one control point to the next, passing through the control
/// points.
///
/// {@template flutter.animation.CatmullRomSpline}
/// Unlike most cubic splines, Catmull-Rom splines have the advantage that their
/// curves pass through the control points given to them. They are cubic
/// polynomial representations, and, in fact, Catmull-Rom splines can be
/// converted mathematically into cubic splines. This class implements a
/// "centripetal" Catmull-Rom spline. The term centripetal implies that it won't
/// form loops or self-intersections within a single segment.
/// {@endtemplate}
///
/// See also:
///  * [Centripetal Catmull–Rom splines](https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline)
///    on Wikipedia.
///  * [Parameterization and Applications of Catmull-Rom Curves](http://faculty.cs.tamu.edu/schaefer/research/cr_cad.pdf),
///    a paper on using Catmull-Rom splines.
///  * [CatmullRomCurve], an animation curve that uses a [CatmullRomSpline] as its
///    internal representation.
class CatmullRomSpline extends Curve2D {
  /// Constructs a centripetal Catmull-Rom spline curve.
  ///
  /// The `controlPoints` argument is a list of four or more points that
  /// describe the points that the curve must pass through.
  ///
  /// The optional `tension` argument controls how tightly the curve approaches
  /// the given `controlPoints`. It must be in the range 0.0 to 1.0, inclusive. It
  /// defaults to 0.0, which provides the smoothest curve. A value of 1.0
  /// produces a linear interpolation between points.
  ///
  /// The optional `endHandle` and `startHandle` points are the beginning and
  /// ending handle positions. If not specified, they are created automatically
  /// by extending the line formed by the first and/or last line segment in the
  /// `controlPoints`, respectively. The spline will not go through these handle
  /// points, but they will affect the slope of the line at the beginning and
  /// end of the spline. The spline will attempt to match the slope of the line
  /// formed by the start or end handle and the neighboring first or last
  /// control point. The default is chosen so that the slope of the line at the
  /// ends matches that of the first or last line segment in the control points.
  ///
  /// The `controlPoints` list must contain at least four control points to
  /// interpolate.
  ///
  /// The internal curve data structures are lazily computed the first time
  /// [transform] is called. If you would rather pre-compute the structures,
  /// use [CatmullRomSpline.precompute] instead.
  CatmullRomSpline(
    List<Offset> controlPoints, {
    double tension = 0.0,
    Offset? startHandle,
    Offset? endHandle,
  }) : assert(tension <= 1.0, 'tension $tension must not be greater than 1.0.'),
       assert(tension >= 0.0, 'tension $tension must not be negative.'),
       assert(
         controlPoints.length > 3,
         'There must be at least four control points to create a CatmullRomSpline.',
       ),
       _controlPoints = controlPoints,
       _startHandle = startHandle,
       _endHandle = endHandle,
       _tension = tension,
       _cubicSegments = <List<Offset>>[];

  /// Constructs a centripetal Catmull-Rom spline curve.
  ///
  /// The same as [CatmullRomSpline.new], except that the internal data
  /// structures are precomputed instead of being computed lazily.
  CatmullRomSpline.precompute(
    List<Offset> controlPoints, {
    double tension = 0.0,
    Offset? startHandle,
    Offset? endHandle,
  }) : assert(tension <= 1.0, 'tension $tension must not be greater than 1.0.'),
       assert(tension >= 0.0, 'tension $tension must not be negative.'),
       assert(
         controlPoints.length > 3,
         'There must be at least four control points to create a CatmullRomSpline.',
       ),
       _controlPoints = null,
       _startHandle = null,
       _endHandle = null,
       _tension = null,
       _cubicSegments = _computeSegments(
         controlPoints,
         tension,
         startHandle: startHandle,
         endHandle: endHandle,
       );

  static List<List<Offset>> _computeSegments(
    List<Offset> controlPoints,
    double tension, {
    Offset? startHandle,
    Offset? endHandle,
  }) {
    assert(
      startHandle == null || startHandle.isFinite,
      'The provided startHandle of CatmullRomSpline must be finite. The '
      'startHandle given was $startHandle.',
    );
    assert(
      endHandle == null || endHandle.isFinite,
      'The provided endHandle of CatmullRomSpline must be finite. The endHandle '
      'given was $endHandle.',
    );
    assert(() {
      for (int index = 0; index < controlPoints.length; index++) {
        if (!controlPoints[index].isFinite) {
          throw FlutterError(
            'The provided CatmullRomSpline control point at index $index is not '
            'finite. The control point given was ${controlPoints[index]}.',
          );
        }
      }
      return true;
    }());
    // If not specified, select the first and last control points (which are
    // handles: they are not intersected by the resulting curve) so that they
    // extend the first and last segments, respectively.
    startHandle ??= controlPoints[0] * 2.0 - controlPoints[1];
    endHandle ??= controlPoints.last * 2.0 - controlPoints[controlPoints.length - 2];
    final List<Offset> allPoints = <Offset>[startHandle, ...controlPoints, endHandle];

    // An alpha of 0.5 is what makes it a centripetal Catmull-Rom spline. A
    // value of 0.0 would make it a uniform Catmull-Rom spline, and a value of
    // 1.0 would make it a chordal Catmull-Rom spline. Non-centripetal values
    // for alpha can give self-intersecting behavior or looping within a
    // segment.
    const double alpha = 0.5;
    final double reverseTension = 1.0 - tension;
    final List<List<Offset>> result = <List<Offset>>[];
    for (int i = 0; i < allPoints.length - 3; ++i) {
      final List<Offset> curve = <Offset>[
        allPoints[i],
        allPoints[i + 1],
        allPoints[i + 2],
        allPoints[i + 3],
      ];
      final Offset diffCurve10 = curve[1] - curve[0];
      final Offset diffCurve21 = curve[2] - curve[1];
      final Offset diffCurve32 = curve[3] - curve[2];
      final double t01 = math.pow(diffCurve10.distance, alpha).toDouble();
      final double t12 = math.pow(diffCurve21.distance, alpha).toDouble();
      final double t23 = math.pow(diffCurve32.distance, alpha).toDouble();

      final Offset m1 =
          (diffCurve21 + (diffCurve10 / t01 - (curve[2] - curve[0]) / (t01 + t12)) * t12) *
          reverseTension;
      final Offset m2 =
          (diffCurve21 + (diffCurve32 / t23 - (curve[3] - curve[1]) / (t12 + t23)) * t12) *
          reverseTension;
      final Offset sumM12 = m1 + m2;

      final List<Offset> segment = <Offset>[
        diffCurve21 * -2.0 + sumM12,
        diffCurve21 * 3.0 - m1 - sumM12,
        m1,
        curve[1],
      ];
      result.add(segment);
    }
    return result;
  }

  // The list of control point lists for each cubic segment of the spline.
  final List<List<Offset>> _cubicSegments;

  // This is non-empty only if the _cubicSegments are being computed lazily.
  final List<Offset>? _controlPoints;
  final Offset? _startHandle;
  final Offset? _endHandle;
  final double? _tension;

  void _initializeIfNeeded() {
    if (_cubicSegments.isNotEmpty) {
      return;
    }
    _cubicSegments.addAll(
      _computeSegments(
        _controlPoints!,
        _tension!,
        startHandle: _startHandle,
        endHandle: _endHandle,
      ),
    );
  }

  @override
  @protected
  int get samplingSeed {
    _initializeIfNeeded();
    final Offset seedPoint = _cubicSegments[0][1];
    return ((seedPoint.dx + seedPoint.dy) * 10000).round();
  }

  @override
  Offset transformInternal(double t) {
    _initializeIfNeeded();
    final double length = _cubicSegments.length.toDouble();
    final double position;
    final double localT;
    final int index;
    if (t < 1.0) {
      position = t * length;
      localT = position % 1.0;
      index = position.floor();
    } else {
      position = length;
      localT = 1.0;
      index = _cubicSegments.length - 1;
    }
    final List<Offset> cubicControlPoints = _cubicSegments[index];
    final double localT2 = localT * localT;
    return cubicControlPoints[0] * localT2 * localT +
        cubicControlPoints[1] * localT2 +
        cubicControlPoints[2] * localT +
        cubicControlPoints[3];
  }
}

/// An animation easing curve that passes smoothly through the given control
/// points using a centripetal Catmull-Rom spline.
///
/// When this curve is evaluated with [transform], the values will interpolate
/// smoothly from one control point to the next, passing through (0.0, 0.0), the
/// given points, and then (1.0, 1.0).
///
/// {@macro flutter.animation.CatmullRomSpline}
///
/// This class uses a centripetal Catmull-Rom curve (a [CatmullRomSpline]) as
/// its internal representation. The term centripetal implies that it won't form
/// loops or self-intersections within a single segment, and corresponds to a
/// Catmull-Rom α (alpha) value of 0.5.
///
/// See also:
///
///  * [CatmullRomSpline], the 2D spline that this curve uses to generate its values.
///  * A Wikipedia article on [centripetal Catmull-Rom splines](https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline).
///  * [CatmullRomCurve.new] for a description of the constraints put on the
///    input control points.
///  * This [paper on using Catmull-Rom splines](http://faculty.cs.tamu.edu/schaefer/research/cr_cad.pdf).
class CatmullRomCurve extends Curve {
  /// Constructs a centripetal [CatmullRomCurve].
  ///
  /// It takes a list of two or more points that describe the points that the
  /// curve must pass through. See [controlPoints] for a description of the
  /// restrictions placed on control points. In addition to the given
  /// [controlPoints], the curve will begin with an implicit control point at
  /// (0.0, 0.0) and end with an implicit control point at (1.0, 1.0), so that
  /// the curve begins and ends at those points.
  ///
  /// The optional [tension] argument controls how tightly the curve approaches
  /// the given `controlPoints`. It must be in the range 0.0 to 1.0, inclusive. It
  /// defaults to 0.0, which provides the smoothest curve. A value of 1.0
  /// is equivalent to a linear interpolation between points.
  ///
  /// The internal curve data structures are lazily computed the first time
  /// [transform] is called. If you would rather pre-compute the curve, use
  /// [CatmullRomCurve.precompute] instead.
  ///
  /// See also:
  ///
  ///  * This [paper on using Catmull-Rom splines](http://faculty.cs.tamu.edu/schaefer/research/cr_cad.pdf).
  CatmullRomCurve(this.controlPoints, {this.tension = 0.0})
    : assert(
        () {
          return validateControlPoints(
            controlPoints,
            tension: tension,
            reasons: _debugAssertReasons..clear(),
          );
        }(),
        'control points $controlPoints could not be validated:\n  ${_debugAssertReasons.join('\n  ')}',
      ),
      // Pre-compute samples so that we don't have to evaluate the spline's inverse
      // all the time in transformInternal.
      _precomputedSamples = <Curve2DSample>[];

  /// Constructs a centripetal [CatmullRomCurve].
  ///
  /// Same as [CatmullRomCurve.new], but it precomputes the internal curve data
  /// structures for a more predictable computation load.
  CatmullRomCurve.precompute(this.controlPoints, {this.tension = 0.0})
    : assert(
        () {
          return validateControlPoints(
            controlPoints,
            tension: tension,
            reasons: _debugAssertReasons..clear(),
          );
        }(),
        'control points $controlPoints could not be validated:\n  ${_debugAssertReasons.join('\n  ')}',
      ),
      // Pre-compute samples so that we don't have to evaluate the spline's inverse
      // all the time in transformInternal.
      _precomputedSamples = _computeSamples(controlPoints, tension);

  static List<Curve2DSample> _computeSamples(List<Offset> controlPoints, double tension) {
    return CatmullRomSpline.precompute(
      // Force the first and last control points for the spline to be (0, 0)
      // and (1, 1), respectively.
      <Offset>[Offset.zero, ...controlPoints, const Offset(1.0, 1.0)],
      tension: tension,
    ).generateSamples(tolerance: 1e-12).toList();
  }

  /// A static accumulator for assertion failures. Not used in release mode.
  static final List<String> _debugAssertReasons = <String>[];

  // The precomputed approximation curve, so that evaluation of the curve is
  // efficient.
  //
  // If the curve is constructed lazily, then this will be empty, and will be filled
  // the first time transform is called.
  final List<Curve2DSample> _precomputedSamples;

  /// The control points used to create this curve.
  ///
  /// The `dx` value of each [Offset] in [controlPoints] represents the
  /// animation value at which the curve should pass through the `dy` value of
  /// the same control point.
  ///
  /// The [controlPoints] list must meet the following criteria:
  ///
  ///  * The list must contain at least two points.
  ///  * The X value of each point must be greater than 0.0 and less then 1.0.
  ///  * The X values of each point must be greater than the
  ///    previous point's X value (i.e. monotonically increasing). The Y values
  ///    are not constrained.
  ///  * The resulting spline must be single-valued in X. That is, for each X
  ///    value, there must be exactly one Y value. This means that the control
  ///    points must not generated a spline that loops or overlaps itself.
  ///
  /// The static function [validateControlPoints] can be used to check that
  /// these conditions are met, and will return true if they are. In debug mode,
  /// it will also optionally return a list of reasons in text form. In debug
  /// mode, the constructor will assert that these conditions are met and print
  /// the reasons if the assert fires.
  ///
  /// When the curve is evaluated with [transform], the values will interpolate
  /// smoothly from one control point to the next, passing through (0.0, 0.0), the
  /// given control points, and (1.0, 1.0).
  final List<Offset> controlPoints;

  /// The "tension" of the curve.
  ///
  /// The [tension] attribute controls how tightly the curve approaches the
  /// given [controlPoints]. It must be in the range 0.0 to 1.0, inclusive. It
  /// is optional, and defaults to 0.0, which provides the smoothest curve. A
  /// value of 1.0 is equivalent to a linear interpolation between control
  /// points.
  final double tension;

  /// Validates that a given set of control points for a [CatmullRomCurve] is
  /// well-formed and will not produce a spline that self-intersects.
  ///
  /// This method is also used in debug mode to validate a curve to make sure
  /// that it won't violate the contract for the [CatmullRomCurve.new]
  /// constructor.
  ///
  /// If in debug mode, and `reasons` is non-null, this function will fill in
  /// `reasons` with descriptions of the problems encountered. The `reasons`
  /// argument is ignored in release mode.
  ///
  /// In release mode, this function can be used to decide if a proposed
  /// modification to the curve will result in a valid curve.
  static bool validateControlPoints(
    List<Offset>? controlPoints, {
    double tension = 0.0,
    List<String>? reasons,
  }) {
    if (controlPoints == null) {
      assert(() {
        reasons?.add('Supplied control points cannot be null');
        return true;
      }());
      return false;
    }

    if (controlPoints.length < 2) {
      assert(() {
        reasons?.add('There must be at least two points supplied to create a valid curve.');
        return true;
      }());
      return false;
    }

    controlPoints = <Offset>[Offset.zero, ...controlPoints, const Offset(1.0, 1.0)];
    final Offset startHandle = controlPoints[0] * 2.0 - controlPoints[1];
    final Offset endHandle = controlPoints.last * 2.0 - controlPoints[controlPoints.length - 2];
    controlPoints = <Offset>[startHandle, ...controlPoints, endHandle];
    double lastX = -double.infinity;
    for (int i = 0; i < controlPoints.length; ++i) {
      if (i > 1 &&
          i < controlPoints.length - 2 &&
          (controlPoints[i].dx <= 0.0 || controlPoints[i].dx >= 1.0)) {
        assert(() {
          reasons?.add(
            'Control points must have X values between 0.0 and 1.0, exclusive. '
            'Point $i has an x value (${controlPoints![i].dx}) which is outside the range.',
          );
          return true;
        }());
        return false;
      }
      if (controlPoints[i].dx <= lastX) {
        assert(() {
          reasons?.add(
            'Each X coordinate must be greater than the preceding X coordinate '
            '(i.e. must be monotonically increasing in X). Point $i has an x value of '
            '${controlPoints![i].dx}, which is not greater than $lastX',
          );
          return true;
        }());
        return false;
      }
      lastX = controlPoints[i].dx;
    }

    bool success = true;

    // An empirical test to make sure things are single-valued in X.
    lastX = -double.infinity;
    const double tolerance = 1e-3;
    final CatmullRomSpline testSpline = CatmullRomSpline(controlPoints, tension: tension);
    final double start = testSpline.findInverse(0.0);
    final double end = testSpline.findInverse(1.0);
    final Iterable<Curve2DSample> samplePoints = testSpline.generateSamples(start: start, end: end);

    /// If the first and last points in the samples aren't at (0,0) or (1,1)
    /// respectively, then the curve is multi-valued at the ends.
    if (samplePoints.first.value.dy.abs() > tolerance ||
        (1.0 - samplePoints.last.value.dy).abs() > tolerance) {
      bool bail = true;
      success = false;
      assert(() {
        reasons?.add(
          'The curve has more than one Y value at X = ${samplePoints.first.value.dx}. '
          'Try moving some control points further away from this value of X, or increasing '
          'the tension.',
        );
        // No need to keep going if we're not giving reasons.
        bail = reasons == null;
        return true;
      }());
      if (bail) {
        // If we're not in debug mode, then we want to bail immediately
        // instead of checking everything else.
        return false;
      }
    }
    for (final Curve2DSample sample in samplePoints) {
      final Offset point = sample.value;
      final double t = sample.t;
      final double x = point.dx;
      if (t >= start && t <= end && (x < -1e-3 || x > 1.0 + 1e-3)) {
        bool bail = true;
        success = false;
        assert(() {
          reasons?.add(
            'The resulting curve has an X value ($x) which is outside '
            'the range [0.0, 1.0], inclusive.',
          );
          // No need to keep going if we're not giving reasons.
          bail = reasons == null;
          return true;
        }());
        if (bail) {
          // If we're not in debug mode, then we want to bail immediately
          // instead of checking all the segments.
          return false;
        }
      }
      if (x < lastX) {
        bool bail = true;
        success = false;
        assert(() {
          reasons?.add(
            'The curve has more than one Y value at x = $x. Try moving '
            'some control points further apart in X, or increasing the tension.',
          );
          // No need to keep going if we're not giving reasons.
          bail = reasons == null;
          return true;
        }());
        if (bail) {
          // If we're not in debug mode, then we want to bail immediately
          // instead of checking all the segments.
          return false;
        }
      }
      lastX = x;
    }
    return success;
  }

  @override
  double transformInternal(double t) {
    // Linearly interpolate between the two closest samples generated when the
    // curve was created.
    if (_precomputedSamples.isEmpty) {
      // Compute the samples now if we were constructed lazily.
      _precomputedSamples.addAll(_computeSamples(controlPoints, tension));
    }
    int start = 0;
    int end = _precomputedSamples.length - 1;
    int mid;
    Offset value;
    Offset startValue = _precomputedSamples[start].value;
    Offset endValue = _precomputedSamples[end].value;
    // Use a binary search to find the index of the sample point that is just
    // before t.
    while (end - start > 1) {
      mid = (end + start) ~/ 2;
      value = _precomputedSamples[mid].value;
      if (t >= value.dx) {
        start = mid;
        startValue = value;
      } else {
        end = mid;
        endValue = value;
      }
    }

    // Now interpolate between the found sample and the next one.
    final double t2 = (t - startValue.dx) / (endValue.dx - startValue.dx);
    return lerpDouble(startValue.dy, endValue.dy, t2)!;
  }
}

/// A curve that is the reversed inversion of its given curve.
///
/// This curve evaluates the given curve in reverse (i.e., from 1.0 to 0.0 as t
/// increases from 0.0 to 1.0) and returns the inverse of the given curve's
/// value (i.e., 1.0 minus the given curve's value).
///
/// This is the class used to implement the [flipped] getter on curves.
///
/// This is often useful with [CurvedAnimation.reverseCurve].
///
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_bounce_in.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_flipped.mp4}
///
/// See also:
///
///  * [Curve.flipped], which provides the [FlippedCurve] of a [Curve].
///  * [ReverseAnimation], which reverses an [Animation] rather than a [Curve].
///  * [CurvedAnimation], which can take a separate curve and reverse curve.
class FlippedCurve extends Curve {
  /// Creates a flipped curve.
  const FlippedCurve(this.curve);

  /// The curve that is being flipped.
  final Curve curve;

  @override
  double transformInternal(double t) => 1.0 - curve.transform(1.0 - t);

  @override
  String toString() {
    return '${objectRuntimeType(this, 'FlippedCurve')}($curve)';
  }
}

/// A curve where the rate of change starts out quickly and then decelerates; an
/// upside-down `f(t) = t²` parabola.
///
/// This is equivalent to the Android `DecelerateInterpolator` class with a unit
/// factor (the default factor).
///
/// See [Curves.decelerate] for an instance of this class.
class _DecelerateCurve extends Curve {
  const _DecelerateCurve._();

  @override
  double transformInternal(double t) {
    // Intended to match the behavior of:
    // https://android.googlesource.com/platform/frameworks/base/+/main/core/java/android/view/animation/DecelerateInterpolator.java
    // ...as of December 2016.
    t = 1.0 - t;
    return 1.0 - t * t;
  }
}

// BOUNCE CURVES

double _bounce(double t) {
  if (t < 1.0 / 2.75) {
    return 7.5625 * t * t;
  } else if (t < 2 / 2.75) {
    t -= 1.5 / 2.75;
    return 7.5625 * t * t + 0.75;
  } else if (t < 2.5 / 2.75) {
    t -= 2.25 / 2.75;
    return 7.5625 * t * t + 0.9375;
  }
  t -= 2.625 / 2.75;
  return 7.5625 * t * t + 0.984375;
}

/// An oscillating curve that grows in magnitude.
///
/// See [Curves.bounceIn] for an instance of this class.
class _BounceInCurve extends Curve {
  const _BounceInCurve._();

  @override
  double transformInternal(double t) {
    return 1.0 - _bounce(1.0 - t);
  }
}

/// An oscillating curve that shrink in magnitude.
///
/// See [Curves.bounceOut] for an instance of this class.
class _BounceOutCurve extends Curve {
  const _BounceOutCurve._();

  @override
  double transformInternal(double t) {
    return _bounce(t);
  }
}

/// An oscillating curve that first grows and then shrink in magnitude.
///
/// See [Curves.bounceInOut] for an instance of this class.
class _BounceInOutCurve extends Curve {
  const _BounceInOutCurve._();

  @override
  double transformInternal(double t) {
    if (t < 0.5) {
      return (1.0 - _bounce(1.0 - t * 2.0)) * 0.5;
    } else {
      return _bounce(t * 2.0 - 1.0) * 0.5 + 0.5;
    }
  }
}

// ELASTIC CURVES

/// An oscillating curve that grows in magnitude while overshooting its bounds.
///
/// An instance of this class using the default period of 0.4 is available as
/// [Curves.elasticIn].
///
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_elastic_in.mp4}
class ElasticInCurve extends Curve {
  /// Creates an elastic-in curve.
  ///
  /// Rather than creating a new instance, consider using [Curves.elasticIn].
  const ElasticInCurve([this.period = 0.4]);

  /// The duration of the oscillation.
  final double period;

  @override
  double transformInternal(double t) {
    final double s = period / 4.0;
    t = t - 1.0;
    return -math.pow(2.0, 10.0 * t) * math.sin((t - s) * (math.pi * 2.0) / period);
  }

  @override
  String toString() {
    return '${objectRuntimeType(this, 'ElasticInCurve')}($period)';
  }
}

/// An oscillating curve that shrinks in magnitude while overshooting its bounds.
///
/// An instance of this class using the default period of 0.4 is available as
/// [Curves.elasticOut].
///
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_elastic_out.mp4}
class ElasticOutCurve extends Curve {
  /// Creates an elastic-out curve.
  ///
  /// Rather than creating a new instance, consider using [Curves.elasticOut].
  const ElasticOutCurve([this.period = 0.4]);

  /// The duration of the oscillation.
  final double period;

  @override
  double transformInternal(double t) {
    final double s = period / 4.0;
    return math.pow(2.0, -10 * t) * math.sin((t - s) * (math.pi * 2.0) / period) + 1.0;
  }

  @override
  String toString() {
    return '${objectRuntimeType(this, 'ElasticOutCurve')}($period)';
  }
}

/// An oscillating curve that grows and then shrinks in magnitude while
/// overshooting its bounds.
///
/// An instance of this class using the default period of 0.4 is available as
/// [Curves.elasticInOut].
///
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_elastic_in_out.mp4}
class ElasticInOutCurve extends Curve {
  /// Creates an elastic-in-out curve.
  ///
  /// Rather than creating a new instance, consider using [Curves.elasticInOut].
  const ElasticInOutCurve([this.period = 0.4]);

  /// The duration of the oscillation.
  final double period;

  @override
  double transformInternal(double t) {
    final double s = period / 4.0;
    t = 2.0 * t - 1.0;
    if (t < 0.0) {
      return -0.5 * math.pow(2.0, 10.0 * t) * math.sin((t - s) * (math.pi * 2.0) / period);
    } else {
      return math.pow(2.0, -10.0 * t) * math.sin((t - s) * (math.pi * 2.0) / period) * 0.5 + 1.0;
    }
  }

  @override
  String toString() {
    return '${objectRuntimeType(this, 'ElasticInOutCurve')}($period)';
  }
}

// PREDEFINED CURVES

/// A collection of common animation curves.
///
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_bounce_in.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_bounce_in_out.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_bounce_out.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_decelerate.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_sine.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_quad.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_cubic.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_quart.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_quint.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_expo.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_circ.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_back.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_sine.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_quad.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_cubic.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_quart.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_quint.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_expo.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_circ.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_back.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_sine.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_quad.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_cubic.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_quart.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_quint.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_expo.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_circ.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_back.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_elastic_in.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_elastic_in_out.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_elastic_out.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_fast_out_slow_in.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_slow_middle.mp4}
/// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_linear.mp4}
///
/// See also:
///
///  * [Curve], the interface implemented by the constants available from the
///    [Curves] class.
///  * [Easing], for the Material animation curves.
abstract final class Curves {
  /// A linear animation curve.
  ///
  /// This is the identity map over the unit interval: its [Curve.transform]
  /// method returns its input unmodified. This is useful as a default curve for
  /// cases where a [Curve] is required but no actual curve is desired.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_linear.mp4}
  static const Curve linear = _Linear._();

  /// A curve where the rate of change starts out quickly and then decelerates; an
  /// upside-down `f(t) = t²` parabola.
  ///
  /// This is equivalent to the Android `DecelerateInterpolator` class with a unit
  /// factor (the default factor).
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_decelerate.mp4}
  static const Curve decelerate = _DecelerateCurve._();

  /// A curve that is very steep and linear at the beginning, but quickly flattens out
  /// and very slowly eases in.
  ///
  /// By default is the curve used to animate pages on iOS back to their original
  /// position if a swipe gesture is ended midway through a swipe.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_fast_linear_to_slow_ease_in.mp4}
  static const Cubic fastLinearToSlowEaseIn = Cubic(0.18, 1.0, 0.04, 1.0);

  /// A curve that starts slowly, speeds up very quickly, and then ends slowly.
  ///
  /// This curve is used by default to animate page transitions used by
  /// [CupertinoPageRoute].
  ///
  /// It has been derived from plots of native iOS 16.3
  /// animation frames on iPhone 14 Pro Max.
  /// Specifically, transition animation positions were measured
  /// every frame and plotted against time. Then, a cubic curve was
  /// strictly fit to the measured data points.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_fast_ease_in_to_slow_ease_out.mp4}
  static const ThreePointCubic fastEaseInToSlowEaseOut = ThreePointCubic(
    Offset(0.056, 0.024),
    Offset(0.108, 0.3085),
    Offset(0.198, 0.541),
    Offset(0.3655, 1.0),
    Offset(0.5465, 0.989),
  );

  /// A cubic animation curve that speeds up quickly and ends slowly.
  ///
  /// This is the same as the CSS easing function `ease`.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease.mp4}
  static const Cubic ease = Cubic(0.25, 0.1, 0.25, 1.0);

  /// A cubic animation curve that starts slowly and ends quickly.
  ///
  /// This is the same as the CSS easing function `ease-in`.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in.mp4}
  static const Cubic easeIn = Cubic(0.42, 0.0, 1.0, 1.0);

  /// A cubic animation curve that starts slowly and ends linearly.
  ///
  /// The symmetric animation to [linearToEaseOut].
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_to_linear.mp4}
  static const Cubic easeInToLinear = Cubic(0.67, 0.03, 0.65, 0.09);

  /// A cubic animation curve that starts slowly and ends quickly. This is
  /// similar to [Curves.easeIn], but with sinusoidal easing for a slightly less
  /// abrupt beginning and end. Nonetheless, the result is quite gentle and is
  /// hard to distinguish from [Curves.linear] at a glance.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_sine.mp4}
  static const Cubic easeInSine = Cubic(0.47, 0.0, 0.745, 0.715);

  /// A cubic animation curve that starts slowly and ends quickly. Based on a
  /// quadratic equation where `f(t) = t²`, this is effectively the inverse of
  /// [Curves.decelerate].
  ///
  /// Compared to [Curves.easeInSine], this curve is slightly steeper.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_quad.mp4}
  static const Cubic easeInQuad = Cubic(0.55, 0.085, 0.68, 0.53);

  /// A cubic animation curve that starts slowly and ends quickly. This curve is
  /// based on a cubic equation where `f(t) = t³`. The result is a safe sweet
  /// spot when choosing a curve for widgets animating off the viewport.
  ///
  /// Compared to [Curves.easeInQuad], this curve is slightly steeper.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_cubic.mp4}
  static const Cubic easeInCubic = Cubic(0.55, 0.055, 0.675, 0.19);

  /// A cubic animation curve that starts slowly and ends quickly. This curve is
  /// based on a quartic equation where `f(t) = t⁴`.
  ///
  /// Animations using this curve or steeper curves will benefit from a longer
  /// duration to avoid motion feeling unnatural.
  ///
  /// Compared to [Curves.easeInCubic], this curve is slightly steeper.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_quart.mp4}
  static const Cubic easeInQuart = Cubic(0.895, 0.03, 0.685, 0.22);

  /// A cubic animation curve that starts slowly and ends quickly. This curve is
  /// based on a quintic equation where `f(t) = t⁵`.
  ///
  /// Compared to [Curves.easeInQuart], this curve is slightly steeper.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_quint.mp4}
  static const Cubic easeInQuint = Cubic(0.755, 0.05, 0.855, 0.06);

  /// A cubic animation curve that starts slowly and ends quickly. This curve is
  /// based on an exponential equation where `f(t) = 2¹⁰⁽ᵗ⁻¹⁾`.
  ///
  /// Using this curve can give your animations extra flare, but a longer
  /// duration may need to be used to compensate for the steepness of the curve.
  ///
  /// Compared to [Curves.easeInQuint], this curve is slightly steeper.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_expo.mp4}
  static const Cubic easeInExpo = Cubic(0.95, 0.05, 0.795, 0.035);

  /// A cubic animation curve that starts slowly and ends quickly. This curve is
  /// effectively the bottom-right quarter of a circle.
  ///
  /// Like [Curves.easeInExpo], this curve is fairly dramatic and will reduce
  /// the clarity of an animation if not given a longer duration.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_circ.mp4}
  static const Cubic easeInCirc = Cubic(0.6, 0.04, 0.98, 0.335);

  /// A cubic animation curve that starts slowly and ends quickly. This curve
  /// is similar to [Curves.elasticIn] in that it overshoots its bounds before
  /// reaching its end. Instead of repeated swinging motions before ascending,
  /// though, this curve overshoots once, then continues to ascend.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_back.mp4}
  static const Cubic easeInBack = Cubic(0.6, -0.28, 0.735, 0.045);

  /// A cubic animation curve that starts quickly and ends slowly.
  ///
  /// This is the same as the CSS easing function `ease-out`.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out.mp4}
  static const Cubic easeOut = Cubic(0.0, 0.0, 0.58, 1.0);

  /// A cubic animation curve that starts linearly and ends slowly.
  ///
  /// A symmetric animation to [easeInToLinear].
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_linear_to_ease_out.mp4}
  static const Cubic linearToEaseOut = Cubic(0.35, 0.91, 0.33, 0.97);

  /// A cubic animation curve that starts quickly and ends slowly. This is
  /// similar to [Curves.easeOut], but with sinusoidal easing for a slightly
  /// less abrupt beginning and end. Nonetheless, the result is quite gentle and
  /// is hard to distinguish from [Curves.linear] at a glance.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_sine.mp4}
  static const Cubic easeOutSine = Cubic(0.39, 0.575, 0.565, 1.0);

  /// A cubic animation curve that starts quickly and ends slowly. This is
  /// effectively the same as [Curves.decelerate], only simulated using a cubic
  /// bezier function.
  ///
  /// Compared to [Curves.easeOutSine], this curve is slightly steeper.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_quad.mp4}
  static const Cubic easeOutQuad = Cubic(0.25, 0.46, 0.45, 0.94);

  /// A cubic animation curve that starts quickly and ends slowly. This curve is
  /// a flipped version of [Curves.easeInCubic].
  ///
  /// The result is a safe sweet spot when choosing a curve for animating a
  /// widget's position entering or already inside the viewport.
  ///
  /// Compared to [Curves.easeOutQuad], this curve is slightly steeper.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_cubic.mp4}
  static const Cubic easeOutCubic = Cubic(0.215, 0.61, 0.355, 1.0);

  /// A cubic animation curve that starts quickly and ends slowly. This curve is
  /// a flipped version of [Curves.easeInQuart].
  ///
  /// Animations using this curve or steeper curves will benefit from a longer
  /// duration to avoid motion feeling unnatural.
  ///
  /// Compared to [Curves.easeOutCubic], this curve is slightly steeper.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_quart.mp4}
  static const Cubic easeOutQuart = Cubic(0.165, 0.84, 0.44, 1.0);

  /// A cubic animation curve that starts quickly and ends slowly. This curve is
  /// a flipped version of [Curves.easeInQuint].
  ///
  /// Compared to [Curves.easeOutQuart], this curve is slightly steeper.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_quint.mp4}
  static const Cubic easeOutQuint = Cubic(0.23, 1.0, 0.32, 1.0);

  /// A cubic animation curve that starts quickly and ends slowly. This curve is
  /// a flipped version of [Curves.easeInExpo]. Using this curve can give your
  /// animations extra flare, but a longer duration may need to be used to
  /// compensate for the steepness of the curve.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_expo.mp4}
  static const Cubic easeOutExpo = Cubic(0.19, 1.0, 0.22, 1.0);

  /// A cubic animation curve that starts quickly and ends slowly. This curve is
  /// effectively the top-left quarter of a circle.
  ///
  /// Like [Curves.easeOutExpo], this curve is fairly dramatic and will reduce
  /// the clarity of an animation if not given a longer duration.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_circ.mp4}
  static const Cubic easeOutCirc = Cubic(0.075, 0.82, 0.165, 1.0);

  /// A cubic animation curve that starts quickly and ends slowly. This curve is
  /// similar to [Curves.elasticOut] in that it overshoots its bounds before
  /// reaching its end. Instead of repeated swinging motions after ascending,
  /// though, this curve only overshoots once.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_out_back.mp4}
  static const Cubic easeOutBack = Cubic(0.175, 0.885, 0.32, 1.275);

  /// A cubic animation curve that starts slowly, speeds up, and then ends
  /// slowly.
  ///
  /// This is the same as the CSS easing function `ease-in-out`.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out.mp4}
  static const Cubic easeInOut = Cubic(0.42, 0.0, 0.58, 1.0);

  /// A cubic animation curve that starts slowly, speeds up, and then ends
  /// slowly. This is similar to [Curves.easeInOut], but with sinusoidal easing
  /// for a slightly less abrupt beginning and end.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_sine.mp4}
  static const Cubic easeInOutSine = Cubic(0.445, 0.05, 0.55, 0.95);

  /// A cubic animation curve that starts slowly, speeds up, and then ends
  /// slowly. This curve can be imagined as [Curves.easeInQuad] as the first
  /// half, and [Curves.easeOutQuad] as the second.
  ///
  /// Compared to [Curves.easeInOutSine], this curve is slightly steeper.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_quad.mp4}
  static const Cubic easeInOutQuad = Cubic(0.455, 0.03, 0.515, 0.955);

  /// A cubic animation curve that starts slowly, speeds up, and then ends
  /// slowly. This curve can be imagined as [Curves.easeInCubic] as the first
  /// half, and [Curves.easeOutCubic] as the second.
  ///
  /// The result is a safe sweet spot when choosing a curve for a widget whose
  /// initial and final positions are both within the viewport.
  ///
  /// Compared to [Curves.easeInOutQuad], this curve is slightly steeper.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_cubic.mp4}
  static const Cubic easeInOutCubic = Cubic(0.645, 0.045, 0.355, 1.0);

  /// A cubic animation curve that starts slowly, speeds up shortly thereafter,
  /// and then ends slowly. This curve can be imagined as a steeper version of
  /// [easeInOutCubic].
  ///
  /// The result is a more emphasized eased curve when choosing a curve for a
  /// widget whose initial and final positions are both within the viewport.
  ///
  /// Compared to [Curves.easeInOutCubic], this curve is slightly steeper.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_cubic_emphasized.mp4}
  static const ThreePointCubic easeInOutCubicEmphasized = ThreePointCubic(
    Offset(0.05, 0),
    Offset(0.133333, 0.06),
    Offset(0.166666, 0.4),
    Offset(0.208333, 0.82),
    Offset(0.25, 1),
  );

  /// A cubic animation curve that starts slowly, speeds up, and then ends
  /// slowly. This curve can be imagined as [Curves.easeInQuart] as the first
  /// half, and [Curves.easeOutQuart] as the second.
  ///
  /// Animations using this curve or steeper curves will benefit from a longer
  /// duration to avoid motion feeling unnatural.
  ///
  /// Compared to [Curves.easeInOutCubic], this curve is slightly steeper.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_quart.mp4}
  static const Cubic easeInOutQuart = Cubic(0.77, 0.0, 0.175, 1.0);

  /// A cubic animation curve that starts slowly, speeds up, and then ends
  /// slowly. This curve can be imagined as [Curves.easeInQuint] as the first
  /// half, and [Curves.easeOutQuint] as the second.
  ///
  /// Compared to [Curves.easeInOutQuart], this curve is slightly steeper.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_quint.mp4}
  static const Cubic easeInOutQuint = Cubic(0.86, 0.0, 0.07, 1.0);

  /// A cubic animation curve that starts slowly, speeds up, and then ends
  /// slowly.
  ///
  /// Since this curve is arrived at with an exponential function, the midpoint
  /// is exceptionally steep. Extra consideration should be taken when designing
  /// an animation using this.
  ///
  /// Compared to [Curves.easeInOutQuint], this curve is slightly steeper.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_expo.mp4}
  static const Cubic easeInOutExpo = Cubic(1.0, 0.0, 0.0, 1.0);

  /// A cubic animation curve that starts slowly, speeds up, and then ends
  /// slowly. This curve can be imagined as [Curves.easeInCirc] as the first
  /// half, and [Curves.easeOutCirc] as the second.
  ///
  /// Like [Curves.easeInOutExpo], this curve is fairly dramatic and will reduce
  /// the clarity of an animation if not given a longer duration.
  ///
  /// Compared to [Curves.easeInOutExpo], this curve is slightly steeper.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_circ.mp4}
  static const Cubic easeInOutCirc = Cubic(0.785, 0.135, 0.15, 0.86);

  /// A cubic animation curve that starts slowly, speeds up, and then ends
  /// slowly. This curve can be imagined as [Curves.easeInBack] as the first
  /// half, and [Curves.easeOutBack] as the second.
  ///
  /// Since two curves are used as a basis for this curve, the resulting
  /// animation will overshoot its bounds twice before reaching its end - first
  /// by exceeding its lower bound, then exceeding its upper bound and finally
  /// descending to its final position.
  ///
  /// Derived from Robert Penner’s easing functions.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_ease_in_out_back.mp4}
  static const Cubic easeInOutBack = Cubic(0.68, -0.55, 0.265, 1.55);

  /// A curve that starts quickly and eases into its final position.
  ///
  /// Over the course of the animation, the object spends more time near its
  /// final destination. As a result, the user isn’t left waiting for the
  /// animation to finish, and the negative effects of motion are minimized.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_fast_out_slow_in.mp4}
  ///
  /// See also:
  ///
  ///  * [Easing.legacy], the name for this curve in the Material specification.
  static const Cubic fastOutSlowIn = Cubic(0.4, 0.0, 0.2, 1.0);

  /// A cubic animation curve that starts quickly, slows down, and then ends
  /// quickly.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_slow_middle.mp4}
  static const Cubic slowMiddle = Cubic(0.15, 0.85, 0.85, 0.15);

  /// An oscillating curve that grows in magnitude.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_bounce_in.mp4}
  static const Curve bounceIn = _BounceInCurve._();

  /// An oscillating curve that first grows and then shrink in magnitude.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_bounce_out.mp4}
  static const Curve bounceOut = _BounceOutCurve._();

  /// An oscillating curve that first grows and then shrink in magnitude.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_bounce_in_out.mp4}
  static const Curve bounceInOut = _BounceInOutCurve._();

  /// An oscillating curve that grows in magnitude while overshooting its bounds.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_elastic_in.mp4}
  static const ElasticInCurve elasticIn = ElasticInCurve();

  /// An oscillating curve that shrinks in magnitude while overshooting its bounds.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_elastic_out.mp4}
  static const ElasticOutCurve elasticOut = ElasticOutCurve();

  /// An oscillating curve that grows and then shrinks in magnitude while overshooting its bounds.
  ///
  /// {@animation 464 192 https://flutter.github.io/assets-for-api-docs/assets/animation/curve_elastic_in_out.mp4}
  static const ElasticInOutCurve elasticInOut = ElasticInOutCurve();
}
