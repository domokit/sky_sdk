// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'package:flutter/foundation.dart';

import 'curves.dart';
import 'tween.dart';

/// Used to override the default parameters of an animation.
///
/// Currently, this class is used by the following widgets:
/// - [ExpansionTile]
/// - [MaterialApp]
/// - [PopupMenuButton]
/// - [ScaffoldMessengerState.showSnackBar]
/// - [showBottomSheet]
/// - [showModalBottomSheet]
///
/// If [duration] and [reverseDuration] are set to [Duration.zero], the
/// corresponding animation will be disabled.
///
/// All of the parameters are optional. If no parameters are specified,
/// the default animation will be used.
@immutable
class AnimationStyle with Diagnosticable {
  /// Creates an instance of Animation Style class.
  AnimationStyle({this.curve, this.duration, this.reverseCurve, this.reverseDuration});

  /// Creates an instance of Animation Style class with no animation.
  static AnimationStyle noAnimation = AnimationStyle(
    duration: Duration.zero,
    reverseDuration: Duration.zero,
  );

  /// When specified, the animation will use this curve.
  final Curve? curve;

  /// When specified, the animation will use this duration.
  final Duration? duration;

  /// When specified, the reverse animation will use this curve.
  final Curve? reverseCurve;

  /// When specified, the reverse animation will use this duration.
  final Duration? reverseDuration;

  /// Creates a new [AnimationStyle] based on the current selection, with the
  /// provided parameters overridden.
  AnimationStyle copyWith({
    final Curve? curve,
    final Duration? duration,
    final Curve? reverseCurve,
    final Duration? reverseDuration,
  }) {
    return AnimationStyle(
      curve: curve ?? this.curve,
      duration: duration ?? this.duration,
      reverseCurve: reverseCurve ?? this.reverseCurve,
      reverseDuration: reverseDuration ?? this.reverseDuration,
    );
  }

  /// Returns a modified version of the [other] style, where its `null` properties
  /// are filled in with the non-null properties of this style, where applicable.
  ///
  /// If a `null` argument is passed, returns this text style.
  AnimationStyle merge(AnimationStyle? other) {
    if (other == null) {
      return this;
    }
    return copyWith(
      curve: other.curve,
      duration: other.duration,
      reverseCurve: other.reverseCurve,
      reverseDuration: other.reverseDuration,
    );
  }

  /// Linearly interpolate between two animation styles.
  static AnimationStyle? lerp(AnimationStyle? a, AnimationStyle? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    return AnimationStyle(
      curve: _lerp(a?.curve, b?.curve, t, _LerpedCurve.new),
      duration: _lerp(a?.duration, b?.duration, t, _lerpDuration),
      reverseCurve: _lerp(a?.reverseCurve, b?.reverseCurve, t, _LerpedCurve.new),
      reverseDuration: _lerp(a?.reverseDuration, b?.reverseDuration, t, _lerpDuration),
    );
  }

  @optionalTypeArgs
  static T? _lerp<T extends Object>(T? a, T? b, double t, T Function(T a, T b, double t) lerp) {
    if (identical(a, b)) {
      return a;
    }
    if (t == 0.0) {
      return a;
    }
    if (t == 1.0) {
      return b;
    }
    if (a == null || b == null) {
      return a ?? b;
    }
    return lerp(a, b, t);
  }

  static Duration _lerpDuration(Duration a, Duration b, double t) {
    return Duration(
      microseconds: (a.inMicroseconds * (1.0 - t) + b.inMicroseconds * t).round(),
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
    return other is AnimationStyle &&
        other.curve == curve &&
        other.duration == duration &&
        other.reverseCurve == reverseCurve &&
        other.reverseDuration == reverseDuration;
  }

  @override
  int get hashCode => Object.hash(curve, duration, reverseCurve, reverseDuration);

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<Curve>('curve', curve, defaultValue: null));
    properties.add(DiagnosticsProperty<Duration>('duration', duration, defaultValue: null));
    properties.add(DiagnosticsProperty<Curve>('reverseCurve', reverseCurve, defaultValue: null));
    properties.add(
      DiagnosticsProperty<Duration>('reverseDuration', reverseDuration, defaultValue: null),
    );
  }
}

class _LerpedCurve extends Curve {
  const _LerpedCurve(this.first, this.second, this._t);

  final Curve first;
  final Curve second;
  final double _t;

  @override
  double transform(double t) {
    final double a = first.transform(t);
    final double b = second.transform(t);

    return a * (1.0 - _t) + b * _t;
  }

  @override
  bool operator==(Object other) {
    return other is _LerpedCurve
        && other.first == first
        && other.second == second
        && other._t == _t;
  }

  @override
  int get hashCode => Object.hash(first, second, _t);
}
