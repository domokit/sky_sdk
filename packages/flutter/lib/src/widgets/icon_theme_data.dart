// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/cupertino.dart';
/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'icon.dart';
/// @docImport 'icon_theme.dart';
library;

import 'dart:ui' as ui show lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/painting.dart';

import 'framework.dart' show BuildContext;

/// Defines the size, font variations, color, opacity, and shadows of icons.
///
/// Used by [IconTheme] to control those properties in a widget subtree.
///
/// To obtain the current icon theme, use [IconTheme.of]. To convert an icon
/// theme to a version with all the fields filled in, use
/// [IconThemeData.fallback].
@immutable
class IconThemeData with Diagnosticable {
  /// Creates an icon theme data.
  ///
  /// The opacity applies to both explicit and default icon colors. The value
  /// is clamped between 0.0 and 1.0.
  const IconThemeData({
    this.size,
    this.fill,
    this.weight,
    this.grade,
    this.opticalSize,
    this.color,
    double? opacity,
    this.shadows,
    this.applyTextScaling,
  }) : _opacity = opacity,
       assert(fill == null || (0.0 <= fill && fill <= 1.0)),
       assert(weight == null || (0.0 < weight)),
       assert(opticalSize == null || (0.0 < opticalSize));

  /// Creates an icon theme with some reasonable default values.
  ///
  /// The [size] is 24.0, [fill] is 0.0, [weight] is 400.0, [grade] is 0.0,
  /// opticalSize is 48.0, [color] is black, and [opacity] is 1.0.
  const IconThemeData.fallback()
    : size = 24.0,
      fill = 0.0,
      weight = 400.0,
      grade = 0.0,
      opticalSize = 48.0,
      color = const Color(0xFF000000),
      _opacity = 1.0,
      shadows = null,
      applyTextScaling = false;

  /// Creates a copy of this icon theme but with the given fields replaced with
  /// the new values.
  IconThemeData copyWith({
    double? size,
    double? fill,
    double? weight,
    double? grade,
    double? opticalSize,
    Color? color,
    double? opacity,
    List<Shadow>? shadows,
    bool? applyTextScaling,
  }) {
    return IconThemeData(
      size: size ?? this.size,
      fill: fill ?? this.fill,
      weight: weight ?? this.weight,
      grade: grade ?? this.grade,
      opticalSize: opticalSize ?? this.opticalSize,
      color: color ?? this.color,
      opacity: opacity ?? this.opacity,
      shadows: shadows ?? this.shadows,
      applyTextScaling: applyTextScaling ?? this.applyTextScaling,
    );
  }

  /// Returns a new icon theme that matches this icon theme but with some values
  /// replaced by the non-null parameters of the given icon theme. If the given
  /// icon theme is null, returns this icon theme.
  IconThemeData merge(IconThemeData? other) {
    if (other == null) {
      return this;
    }
    return copyWith(
      size: other.size,
      fill: other.fill,
      weight: other.weight,
      grade: other.grade,
      opticalSize: other.opticalSize,
      color: other.color,
      opacity: other.opacity,
      shadows: other.shadows,
      applyTextScaling: other.applyTextScaling,
    );
  }

  /// Called by [IconTheme.of] to convert this instance to an [IconThemeData]
  /// that fits the given [BuildContext].
  ///
  /// This method gives the ambient [IconThemeData] a chance to update itself,
  /// after it's been retrieved by [IconTheme.of], and before being returned as
  /// the final result. For instance, [CupertinoIconThemeData] overrides this method
  /// to resolve [color], in case [color] is a [CupertinoDynamicColor] and needs
  /// to be resolved against the given [BuildContext] before it can be used as a
  /// regular [Color].
  ///
  /// The default implementation returns this [IconThemeData] as-is.
  ///
  /// See also:
  ///
  ///  * [CupertinoIconThemeData.resolve] an implementation that resolves
  ///    the color of [CupertinoIconThemeData] before returning.
  IconThemeData resolve(BuildContext context) => this;

  /// Whether all the properties (except shadows) of this object are non-null.
  bool get isConcrete =>
      size != null &&
      fill != null &&
      weight != null &&
      grade != null &&
      opticalSize != null &&
      color != null &&
      opacity != null &&
      applyTextScaling != null;

  /// The default for [Icon.size].
  ///
  /// Falls back to 24.0.
  final double? size;

  /// The default for [Icon.fill].
  ///
  /// Falls back to 0.0.
  final double? fill;

  /// The default for [Icon.weight].
  ///
  /// Falls back to 400.0.
  final double? weight;

  /// The default for [Icon.grade].
  ///
  /// Falls back to 0.0.
  final double? grade;

  /// The default for [Icon.opticalSize].
  ///
  /// Falls back to 48.0.
  final double? opticalSize;

  /// The default for [Icon.color].
  ///
  /// In material apps, if there is a [Theme] without any [IconTheme]s
  /// specified, icon colors default to white if [ThemeData.brightness] is dark
  /// and black if [ThemeData.brightness] is light.
  ///
  /// Otherwise, falls back to black.
  final Color? color;

  /// An opacity to apply to both explicit and default icon colors.
  ///
  /// Falls back to 1.0.
  double? get opacity => _opacity == null ? null : clampDouble(_opacity, 0.0, 1.0);
  final double? _opacity;

  /// The default for [Icon.shadows].
  final List<Shadow>? shadows;

  /// The default for [Icon.applyTextScaling].
  final bool? applyTextScaling;

  /// Linearly interpolate between two icon theme data objects.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static IconThemeData lerp(IconThemeData? a, IconThemeData? b, double t) {
    if (identical(a, b) && a != null) {
      return a;
    }
    return IconThemeData(
      size: ui.lerpDouble(a?.size, b?.size, t),
      fill: ui.lerpDouble(a?.fill, b?.fill, t),
      weight: ui.lerpDouble(a?.weight, b?.weight, t),
      grade: ui.lerpDouble(a?.grade, b?.grade, t),
      opticalSize: ui.lerpDouble(a?.opticalSize, b?.opticalSize, t),
      color: Color.lerp(a?.color, b?.color, t),
      opacity: ui.lerpDouble(a?.opacity, b?.opacity, t),
      shadows: Shadow.lerpList(a?.shadows, b?.shadows, t),
      applyTextScaling: t < 0.5 ? a?.applyTextScaling : b?.applyTextScaling,
    );
  }

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is IconThemeData &&
        other.size == size &&
        other.fill == fill &&
        other.weight == weight &&
        other.grade == grade &&
        other.opticalSize == opticalSize &&
        other.color == color &&
        other.opacity == opacity &&
        listEquals(other.shadows, shadows) &&
        other.applyTextScaling == applyTextScaling;
  }

  @override
  int get hashCode => Object.hash(
    size,
    fill,
    weight,
    grade,
    opticalSize,
    color,
    opacity,
    shadows == null ? null : Object.hashAll(shadows!),
    applyTextScaling,
  );

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('size', size, defaultValue: null));
    properties.add(DoubleProperty('fill', fill, defaultValue: null));
    properties.add(DoubleProperty('weight', weight, defaultValue: null));
    properties.add(DoubleProperty('grade', grade, defaultValue: null));
    properties.add(DoubleProperty('opticalSize', opticalSize, defaultValue: null));
    properties.add(ColorProperty('color', color, defaultValue: null));
    properties.add(DoubleProperty('opacity', opacity, defaultValue: null));
    properties.add(IterableProperty<Shadow>('shadows', shadows, defaultValue: null));
    properties.add(
      DiagnosticsProperty<bool>('applyTextScaling', applyTextScaling, defaultValue: null),
    );
  }
}
