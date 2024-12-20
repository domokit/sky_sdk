// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'dart:ui' show Color, Rect, Size;

import 'package:flutter/foundation.dart';

import 'animations.dart';

export 'dart:ui' show Color, Rect, Size;

export 'animation.dart' show Animation;
export 'curves.dart' show Curve;

// Examples can assume:
// late Animation<Offset> _animation;
// late AnimationController _controller;

/// A typedef used by [Animatable.fromCallback] to create an [Animatable]
/// from a callback.
typedef AnimatableCallback<T> = T Function(double value);

/// An object that can produce a value of type [T] given an [Animation<double>]
/// as input.
///
/// Typically, the values of the input animation are nominally in the range 0.0
/// to 1.0. In principle, however, any value could be provided.
///
/// The main subclass of [Animatable] is [Tween].
abstract class Animatable<T> {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const Animatable();

  /// Create a new [Animatable] from the provided [callback].
  ///
  /// See also:
  ///
  ///  * [Animation.drive], which provides an example for how this can be
  ///    used.
  const factory Animatable.fromCallback(AnimatableCallback<T> callback) = _CallbackAnimatable<T>;

  /// Returns the value of the object at point `t`.
  ///
  /// The value of `t` is nominally a fraction in the range 0.0 to 1.0, though
  /// in practice it may extend outside this range.
  ///
  /// See also:
  ///
  ///  * [evaluate], which is a shorthand for applying [transform] to the value
  ///    of an [Animation].
  ///  * [Curve.transform], a similar method for easing curves.
  T transform(double t);

  /// The current value of this object for the given [Animation].
  ///
  /// This function is implemented by deferring to [transform]. Subclasses that
  /// want to provide custom behavior should override [transform], not
  /// [evaluate].
  ///
  /// See also:
  ///
  ///  * [transform], which is similar but takes a `t` value directly instead of
  ///    an [Animation].
  ///  * [animate], which creates an [Animation] out of this object, continually
  ///    applying [evaluate].
  T evaluate(Animation<double> animation) => transform(animation.value);

  /// Returns a new [Animation] that is driven by the given animation but that
  /// takes on values determined by this object.
  ///
  /// Essentially this returns an [Animation] that automatically applies the
  /// [evaluate] method to the parent's value.
  ///
  /// See also:
  ///
  ///  * [AnimationController.drive], which does the same thing from the
  ///    opposite starting point.
  Animation<T> animate(Animation<double> parent) {
    return _AnimatedEvaluation<T>(parent, this);
  }

  /// Returns a new [Animatable] whose value is determined by first evaluating
  /// the given parent and then evaluating this object at the result.
  ///
  /// This method represents function composition on [transform]:
  /// the [transform] method of the returned [Animatable] is the result of
  /// composing this object's [transform] method with
  /// the given parent's [transform] method.
  ///
  /// This allows [Tween]s to be chained before obtaining an [Animation],
  /// without allocating an [Animation] for the intermediate result.
  Animatable<T> chain(Animatable<double> parent) {
    return _ChainedEvaluation<T>(parent, this);
  }
}

// A concrete subclass of `Animatable` used by `Animatable.fromCallback`.
class _CallbackAnimatable<T> extends Animatable<T> {
  const _CallbackAnimatable(this._callback);

  final AnimatableCallback<T> _callback;

  @override
  T transform(double t) {
    return _callback(t);
  }
}

class _AnimatedEvaluation<T> extends Animation<T> with AnimationWithParentMixin<double> {
  _AnimatedEvaluation(this.parent, this._evaluatable);

  @override
  final Animation<double> parent;

  final Animatable<T> _evaluatable;

  @override
  T get value => _evaluatable.evaluate(parent);

  @override
  String toString() {
    return '$parent\u27A9$_evaluatable\u27A9$value';
  }

  @override
  String toStringDetails() {
    return '${super.toStringDetails()} $_evaluatable';
  }
}

class _ChainedEvaluation<T> extends Animatable<T> {
  _ChainedEvaluation(this._parent, this._evaluatable);

  final Animatable<double> _parent;
  final Animatable<T> _evaluatable;

  @override
  T transform(double t) {
    return _evaluatable.transform(_parent.transform(t));
  }

  @override
  String toString() {
    return '$_parent\u27A9$_evaluatable';
  }
}

/// A linear interpolation between a beginning and ending value.
///
/// [Tween] is useful if you want to interpolate across a range.
///
/// To use a [Tween] object with an animation, call the [Tween] object's
/// [animate] method and pass it the [Animation] object that you want to
/// modify.
///
/// You can chain [Tween] objects together using the [chain] method,
/// producing the function composition of their [transform] methods.
/// Configuring a single [Animation] object by calling [animate] on the
/// resulting [Tween] produces the same result as calling the [animate] method
/// on each [Tween] separately in succession, but more efficiently because
/// it avoids creating [Animation] objects for the intermediate results.
///
/// {@tool snippet}
///
/// Suppose `_controller` is an [AnimationController], and we want to create an
/// [Animation<Offset>] that is controlled by that controller, and save it in
/// `_animation`. Here are two possible ways of expressing this:
///
/// ```dart
/// _animation = _controller.drive(
///   Tween<Offset>(
///     begin: const Offset(100.0, 50.0),
///     end: const Offset(200.0, 300.0),
///   ),
/// );
/// ```
/// {@end-tool}
/// {@tool snippet}
///
/// ```dart
/// _animation = Tween<Offset>(
///   begin: const Offset(100.0, 50.0),
///   end: const Offset(200.0, 300.0),
/// ).animate(_controller);
/// ```
/// {@end-tool}
///
/// In both cases, the `_animation` variable holds an object that, over the
/// lifetime of the `_controller`'s animation, returns a value
/// (`_animation.value`) that depicts a point along the line between the two
/// offsets above. If we used a [MaterialPointArcTween] instead of a
/// [Tween<Offset>] in the code above, the points would follow a pleasing curve
/// instead of a straight line, with no other changes necessary.
///
/// ## Performance optimizations
///
/// Tweens are mutable; specifically, their [begin] and [end] values can be
/// changed at runtime. An object created with [Animation.drive] using a [Tween]
/// will immediately honor changes to that underlying [Tween] (though the
/// listeners will only be triggered if the [Animation] is actively animating).
/// This can be used to change an animation on the fly without having to
/// recreate all the objects in the chain from the [AnimationController] to the
/// final [Tween].
///
/// If a [Tween]'s values are never changed, however, a further optimization can
/// be applied: the object can be stored in a `static final` variable, so that
/// the exact same instance is used whenever the [Tween] is needed. This is
/// preferable to creating an identical [Tween] afresh each time a [State.build]
/// method is called, for example.
///
/// ## Types with special considerations
///
/// Classes with [lerp] static methods typically have corresponding dedicated
/// [Tween] subclasses that call that method. For example, [ColorTween] uses
/// [Color.lerp] to implement the [ColorTween.lerp] method.
///
/// Types that define `+` and `-` operators to combine values (`T + T → T` and
/// `T - T → T`) and an `*` operator to scale by multiplying with a double (`T *
/// double → T`) can be directly used with `Tween<T>`.
///
/// This does not extend to any type with `+`, `-`, and `*` operators. In
/// particular, [int] does not satisfy this precise contract (`int * double`
/// actually returns [num], not [int]). There are therefore two specific classes
/// that can be used to interpolate integers:
///
///  * [IntTween], which is an approximation of a linear interpolation (using
///    [double.round]).
///  * [StepTween], which uses [double.floor] to ensure that the result is
///    never greater than it would be using if a `Tween<double>`.
///
/// The relevant operators on [Size] also don't fulfill this contract, so
/// [SizeTween] uses [Size.lerp].
///
/// In addition, some of the types that _do_ have suitable `+`, `-`, and `*`
/// operators still have dedicated [Tween] subclasses that perform the
/// interpolation in a more specialized manner. One such class is
/// [MaterialPointArcTween], which is mentioned above. The [AlignmentTween], and
/// [AlignmentGeometryTween], and [FractionalOffsetTween] are another group of
/// [Tween]s that use dedicated `lerp` methods instead of merely relying on the
/// operators (in particular, this allows them to handle null values in a more
/// useful manner).
///
/// ## Nullability
///
/// The [begin] and [end] fields are nullable; a [Tween] does not have to
/// have non-null values specified when it is created.
///
/// If `T` is nullable, then [lerp] and [transform] may return null.
/// This is typically seen in the case where [begin] is null and `t`
/// is 0.0, or [end] is null and `t` is 1.0, or both are null (at any
/// `t` value).
///
/// If `T` is not nullable, then [begin] and [end] must both be set to
/// non-null values before using [lerp] or [transform], otherwise they
/// will throw.
///
/// ## Implementing a Tween
///
/// To specialize this class for a new type, the subclass should implement
/// the [lerp] method (and a constructor). The other methods of this class
/// are all defined in terms of [lerp].
class Tween<T extends Object?> extends Animatable<T> {
  /// Creates a tween.
  ///
  /// The [begin] and [end] properties must be non-null before the tween is
  /// first used, but the arguments can be null if the values are going to be
  /// filled in later.
  Tween({this.begin, this.end});

  /// The value this variable has at the beginning of the animation.
  ///
  /// See the constructor for details about whether this property may be null
  /// (it varies from subclass to subclass).
  T? begin;

  /// The value this variable has at the end of the animation.
  ///
  /// See the constructor for details about whether this property may be null
  /// (it varies from subclass to subclass).
  T? end;

  /// Returns the value this variable has at the given animation clock value.
  ///
  /// The default implementation of this method uses the `+`, `-`, and `*`
  /// operators on `T`. The [begin] and [end] properties must therefore be
  /// non-null by the time this method is called.
  ///
  /// In general, however, it is possible for this to return null, especially
  /// when `t`=0.0 and [begin] is null, or `t`=1.0 and [end] is null.
  @protected
  T lerp(double t) {
    assert(begin != null);
    assert(end != null);
    assert(() {
      // Assertions that attempt to catch common cases of tweening types
      // that do not conform to the Tween requirements.
      dynamic result;
      try {
        // ignore: avoid_dynamic_calls
        result = (begin as dynamic) + ((end as dynamic) - (begin as dynamic)) * t;
        result as T;
        return true;
      } on NoSuchMethodError {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('Cannot lerp between "$begin" and "$end".'),
          ErrorDescription(
            'The type ${begin.runtimeType} might not fully implement `+`, `-`, and/or `*`. '
            'See "Types with special considerations" at https://api.flutter.dev/flutter/animation/Tween-class.html '
            'for more information.',
          ),
          if (begin is Color || end is Color)
            ErrorHint('To lerp colors, consider ColorTween instead.')
          else if (begin is Rect || end is Rect)
            ErrorHint('To lerp rects, consider RectTween instead.')
          else
            ErrorHint(
              'There may be a dedicated "${begin.runtimeType}Tween" for this type, '
              'or you may need to create one.',
            ),
        ]);
      } on TypeError {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('Cannot lerp between "$begin" and "$end".'),
          ErrorDescription(
            'The type ${begin.runtimeType} returned a ${result.runtimeType} after '
            'multiplication with a double value. '
            'See "Types with special considerations" at https://api.flutter.dev/flutter/animation/Tween-class.html '
            'for more information.',
          ),
          if (begin is int || end is int)
            ErrorHint('To lerp int values, consider IntTween or StepTween instead.')
          else
            ErrorHint(
              'There may be a dedicated "${begin.runtimeType}Tween" for this type, '
              'or you may need to create one.',
            ),
        ]);
      }
    }());
    // ignore: avoid_dynamic_calls
    return (begin as dynamic) + ((end as dynamic) - (begin as dynamic)) * t as T;
  }

  /// Returns the interpolated value for the current value of the given animation.
  ///
  /// This method returns `begin` and `end` when the animation values are 0.0 or
  /// 1.0, respectively.
  ///
  /// This function is implemented by deferring to [lerp]. Subclasses that want
  /// to provide custom behavior should override [lerp], not [transform] (nor
  /// [evaluate]).
  ///
  /// See the constructor for details about whether the [begin] and [end]
  /// properties may be null when this is called. It varies from subclass to
  /// subclass.
  @override
  T transform(double t) {
    if (t == 0.0) {
      return begin as T;
    }
    if (t == 1.0) {
      return end as T;
    }
    return lerp(t);
  }

  @override
  String toString() => '${objectRuntimeType(this, 'Animatable')}($begin \u2192 $end)';
}

/// A [Tween] that evaluates its [parent] in reverse.
class ReverseTween<T extends Object?> extends Tween<T> {
  /// Construct a [Tween] that evaluates its [parent] in reverse.
  ReverseTween(this.parent) : super(begin: parent.end, end: parent.begin);

  /// This tween's value is the same as the parent's value evaluated in reverse.
  ///
  /// This tween's [begin] is the parent's [end] and its [end] is the parent's
  /// [begin]. The [lerp] method returns `parent.lerp(1.0 - t)` and its
  /// [evaluate] method is similar.
  final Tween<T> parent;

  @override
  T lerp(double t) => parent.lerp(1.0 - t);
}

/// An interpolation between two colors.
///
/// This class specializes the interpolation of [Tween<Color>] to use
/// [Color.lerp].
///
/// The values can be null, representing no color (which is distinct to
/// transparent black, as represented by [Colors.transparent]).
///
/// See [Tween] for a discussion on how to use interpolation objects.
class ColorTween extends Tween<Color?> {
  /// Creates a [Color] tween.
  ///
  /// The [begin] and [end] properties may be null; the null value
  /// is treated as transparent.
  ///
  /// We recommend that you do not pass [Colors.transparent] as [begin]
  /// or [end] if you want the effect of fading in or out of transparent.
  /// Instead prefer null. [Colors.transparent] refers to black transparent and
  /// thus will fade out of or into black which is likely unwanted.
  ColorTween({super.begin, super.end});

  /// Returns the value this variable has at the given animation clock value.
  @override
  Color? lerp(double t) => Color.lerp(begin, end, t);
}

/// An interpolation between two sizes.
///
/// This class specializes the interpolation of [Tween<Size>] to use
/// [Size.lerp].
///
/// The values can be null, representing [Size.zero].
///
/// See [Tween] for a discussion on how to use interpolation objects.
class SizeTween extends Tween<Size?> {
  /// Creates a [Size] tween.
  ///
  /// The [begin] and [end] properties may be null; the null value
  /// is treated as an empty size.
  SizeTween({super.begin, super.end});

  /// Returns the value this variable has at the given animation clock value.
  @override
  Size? lerp(double t) => Size.lerp(begin, end, t);
}

/// An interpolation between two rectangles.
///
/// This class specializes the interpolation of [Tween<Rect>] to use
/// [Rect.lerp].
///
/// The values can be null, representing a zero-sized rectangle at the
/// origin ([Rect.zero]).
///
/// See [Tween] for a discussion on how to use interpolation objects.
class RectTween extends Tween<Rect?> {
  /// Creates a [Rect] tween.
  ///
  /// The [begin] and [end] properties may be null; the null value
  /// is treated as an empty rect at the top left corner.
  RectTween({super.begin, super.end});

  /// Returns the value this variable has at the given animation clock value.
  @override
  Rect? lerp(double t) => Rect.lerp(begin, end, t);
}

/// An interpolation between two integers that rounds.
///
/// This class specializes the interpolation of [Tween<int>] to be
/// appropriate for integers by interpolating between the given begin
/// and end values and then rounding the result to the nearest
/// integer.
///
/// This is the closest approximation to a linear tween that is possible with an
/// integer. Compare to [StepTween] and [Tween<double>].
///
/// The [begin] and [end] values must be set to non-null values before
/// calling [lerp] or [transform].
///
/// See [Tween] for a discussion on how to use interpolation objects.
class IntTween extends Tween<int> {
  /// Creates an int tween.
  ///
  /// The [begin] and [end] properties must be non-null before the tween is
  /// first used, but the arguments can be null if the values are going to be
  /// filled in later.
  IntTween({super.begin, super.end});

  // The inherited lerp() function doesn't work with ints because it multiplies
  // the begin and end types by a double, and int * double returns a double.
  @override
  int lerp(double t) => (begin! + (end! - begin!) * t).round();
}

/// An interpolation between two integers that floors.
///
/// This class specializes the interpolation of [Tween<int>] to be
/// appropriate for integers by interpolating between the given begin
/// and end values and then using [double.floor] to return the current
/// integer component, dropping the fractional component.
///
/// This results in a value that is never greater than the equivalent
/// value from a linear double interpolation. Compare to [IntTween].
///
/// The [begin] and [end] values must be set to non-null values before
/// calling [lerp] or [transform].
///
/// See [Tween] for a discussion on how to use interpolation objects.
class StepTween extends Tween<int> {
  /// Creates an [int] tween that floors.
  ///
  /// The [begin] and [end] properties must be non-null before the tween is
  /// first used, but the arguments can be null if the values are going to be
  /// filled in later.
  StepTween({super.begin, super.end});

  // The inherited lerp() function doesn't work with ints because it multiplies
  // the begin and end types by a double, and int * double returns a double.
  @override
  int lerp(double t) => (begin! + (end! - begin!) * t).floor();
}

/// A tween with a constant value.
class ConstantTween<T> extends Tween<T> {
  /// Create a tween whose [begin] and [end] values equal [value].
  ConstantTween(T value) : super(begin: value, end: value);

  /// This tween doesn't interpolate, it always returns the same value.
  @override
  T lerp(double t) => begin as T;

  @override
  String toString() => '${objectRuntimeType(this, 'ConstantTween')}(value: $begin)';
}

/// Transforms the value of the given animation by the given curve.
///
/// This class differs from [CurvedAnimation] in that [CurvedAnimation] applies
/// a curve to an existing [Animation] object whereas [CurveTween] can be
/// chained with another [Tween] prior to receiving the underlying [Animation].
/// ([CurvedAnimation] also has the additional ability of having different
/// curves when the animation is going forward vs when it is going backward,
/// which can be useful in some scenarios.)
///
/// {@tool snippet}
///
/// The following code snippet shows how you can apply a curve to a linear
/// animation produced by an [AnimationController] `controller`:
///
/// ```dart
/// final Animation<double> animation = _controller.drive(
///   CurveTween(curve: Curves.ease),
/// );
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [CurvedAnimation], for an alternative way of expressing the sample above.
///  * [AnimationController], for examples of creating and disposing of an
///    [AnimationController].
class CurveTween extends Animatable<double> {
  /// Creates a curve tween.
  CurveTween({required this.curve});

  /// The curve to use when transforming the value of the animation.
  Curve curve;

  @override
  double transform(double t) {
    if (t == 0.0 || t == 1.0) {
      assert(curve.transform(t).round() == t);
      return t;
    }
    return curve.transform(t);
  }

  @override
  String toString() => '${objectRuntimeType(this, 'CurveTween')}(curve: $curve)';
}
