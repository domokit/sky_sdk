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
  const AnimationStyle({this.curve, this.duration, this.reverseCurve, this.reverseDuration});

  /// Creates an instance of Animation Style class with no animation.
  static const AnimationStyle noAnimation = AnimationStyle(
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

  /// Linearly interpolate between two animation styles.
  static AnimationStyle? lerp(AnimationStyle? a, AnimationStyle? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    return AnimationStyle(
      curve: t < 0.5 ? a?.curve : b?.curve,
      duration: t < 0.5 ? a?.duration : b?.duration,
      reverseCurve: t < 0.5 ? a?.reverseCurve : b?.reverseCurve,
      reverseDuration: t < 0.5 ? a?.reverseDuration : b?.reverseDuration,
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
