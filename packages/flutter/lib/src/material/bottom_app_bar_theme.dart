// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'bottom_app_bar.dart';
/// @docImport 'material.dart';
library;

import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'theme.dart';

/// Defines default property values for descendant [BottomAppBar] widgets.
///
/// Descendant widgets obtain the current [BottomAppBarTheme] object using
/// `BottomAppBarTheme.of(context)`. Instances of [BottomAppBarTheme] can be
/// customized with [BottomAppBarTheme.copyWith].
///
/// Typically a [BottomAppBarTheme] is specified as part of the overall [Theme]
/// with [ThemeData.bottomAppBarTheme].
///
/// All [BottomAppBarTheme] properties are `null` by default. When null, the
/// [BottomAppBar] constructor provides defaults.
///
/// See also:
///
///  * [ThemeData], which describes the overall theme information for the
///    application.
@immutable
class BottomAppBarTheme with Diagnosticable {
  /// Creates a theme that can be used for [ThemeData.bottomAppBarTheme].
  const BottomAppBarTheme({
    this.color,
    this.elevation,
    this.shape,
    this.height,
    this.surfaceTintColor,
    this.shadowColor,
    this.padding,
  });

  /// Overrides the default value for [BottomAppBar.color].
  final Color? color;

  /// Overrides the default value for [BottomAppBar.elevation].
  final double? elevation;

  /// Overrides the default value for [BottomAppBar.shape].
  final NotchedShape? shape;

  /// Overrides the default value for [BottomAppBar.height].
  final double? height;

  /// Overrides the default value for [BottomAppBar.surfaceTintColor].
  ///
  /// If null, [BottomAppBar] will not display an overlay color.
  ///
  /// See [Material.surfaceTintColor] for more details.
  final Color? surfaceTintColor;

  /// Overrides the default value for [BottomAppBar.shadowColor].
  final Color? shadowColor;

  /// Overrides the default value for [BottomAppBar.padding].
  final EdgeInsetsGeometry? padding;

  /// Creates a copy of this object but with the given fields replaced with the
  /// new values.
  BottomAppBarTheme copyWith({
    Color? color,
    double? elevation,
    NotchedShape? shape,
    double? height,
    Color? surfaceTintColor,
    Color? shadowColor,
    EdgeInsetsGeometry? padding,
  }) {
    return BottomAppBarTheme(
      color: color ?? this.color,
      elevation: elevation ?? this.elevation,
      shape: shape ?? this.shape,
      height: height ?? this.height,
      surfaceTintColor: surfaceTintColor ?? this.surfaceTintColor,
      shadowColor: shadowColor ?? this.shadowColor,
      padding: padding ?? this.padding,
    );
  }

  /// The [ThemeData.bottomAppBarTheme] property of the ambient [Theme].
  static BottomAppBarTheme of(BuildContext context) {
    return Theme.of(context).bottomAppBarTheme;
  }

  /// Linearly interpolate between two BAB themes.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static BottomAppBarTheme lerp(BottomAppBarTheme? a, BottomAppBarTheme? b, double t) {
    if (identical(a, b) && a != null) {
      return a;
    }
    return BottomAppBarTheme(
      color: Color.lerp(a?.color, b?.color, t),
      elevation: lerpDouble(a?.elevation, b?.elevation, t),
      shape: t < 0.5 ? a?.shape : b?.shape,
      height: lerpDouble(a?.height, b?.height, t),
      surfaceTintColor: Color.lerp(a?.surfaceTintColor, b?.surfaceTintColor, t),
      shadowColor: Color.lerp(a?.shadowColor, b?.shadowColor, t),
      padding: EdgeInsetsGeometry.lerp(a?.padding, b?.padding, t),
    );
  }

  @override
  int get hashCode =>
      Object.hash(color, elevation, shape, height, surfaceTintColor, shadowColor, padding);

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is BottomAppBarTheme &&
        other.color == color &&
        other.elevation == elevation &&
        other.shape == shape &&
        other.height == height &&
        other.surfaceTintColor == surfaceTintColor &&
        other.shadowColor == shadowColor &&
        other.padding == padding;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(ColorProperty('color', color, defaultValue: null));
    properties.add(DiagnosticsProperty<double>('elevation', elevation, defaultValue: null));
    properties.add(DiagnosticsProperty<NotchedShape>('shape', shape, defaultValue: null));
    properties.add(DiagnosticsProperty<double>('height', height, defaultValue: null));
    properties.add(ColorProperty('surfaceTintColor', surfaceTintColor, defaultValue: null));
    properties.add(ColorProperty('shadowColor', shadowColor, defaultValue: null));
    properties.add(DiagnosticsProperty<EdgeInsetsGeometry>('padding', padding, defaultValue: null));
  }
}
