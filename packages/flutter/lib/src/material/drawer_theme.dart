// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'drawer.dart';
library;

import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// Defines default property values for descendant [Drawer] widgets.
///
/// Descendant widgets obtain the current [DrawerThemeData] object
/// using `DrawerTheme.of(context)`. Instances of [DrawerThemeData] can be
/// customized with [DrawerThemeData.copyWith].
///
/// Typically a [DrawerThemeData] is specified as part of the
/// overall [Theme] with [ThemeData.drawerTheme].
///
/// All [DrawerThemeData] properties are `null` by default.
///
/// See also:
///
///  * [DrawerTheme], an [InheritedWidget] that propagates the theme down its
///    subtree.
///  * [ThemeData], which describes the overall theme information for the
///    application and can customize a drawer using [ThemeData.drawerTheme].
@immutable
class DrawerThemeData with Diagnosticable {
  /// Creates a theme that can be used for [ThemeData.drawerTheme] and
  /// [DrawerTheme].
  const DrawerThemeData({
    this.backgroundColor,
    this.scrimColor,
    this.elevation,
    this.shadowColor,
    this.surfaceTintColor,
    this.shape,
    this.endShape,
    this.width,
    this.clipBehavior,
  });

  /// Overrides the default value of [Drawer.backgroundColor].
  final Color? backgroundColor;

  /// Overrides the default value of [DrawerController.scrimColor].
  final Color? scrimColor;

  /// Overrides the default value of [Drawer.elevation].
  final double? elevation;

  /// Overrides the default value for [Drawer.shadowColor].
  final Color? shadowColor;

  /// Overrides the default value for [Drawer.surfaceTintColor].
  final Color? surfaceTintColor;

  /// Overrides the default value of [Drawer.shape].
  final ShapeBorder? shape;

  /// Overrides the default value of [Drawer.shape] for an end drawer.
  final ShapeBorder? endShape;

  /// Overrides the default value of [Drawer.width].
  final double? width;

  /// Overrides the default value of [Drawer.clipBehavior].
  final Clip? clipBehavior;

  /// Creates a copy of this object with the given fields replaced with the
  /// new values.
  DrawerThemeData copyWith({
    Color? backgroundColor,
    Color? scrimColor,
    double? elevation,
    Color? shadowColor,
    Color? surfaceTintColor,
    ShapeBorder? shape,
    ShapeBorder? endShape,
    double? width,
    Clip? clipBehavior,
  }) {
    return DrawerThemeData(
      backgroundColor: backgroundColor ?? this.backgroundColor,
      scrimColor: scrimColor ?? this.scrimColor,
      elevation: elevation ?? this.elevation,
      shadowColor: shadowColor ?? this.shadowColor,
      surfaceTintColor: surfaceTintColor ?? this.surfaceTintColor,
      shape: shape ?? this.shape,
      endShape: endShape ?? this.endShape,
      width: width ?? this.width,
      clipBehavior: clipBehavior ?? this.clipBehavior,
    );
  }

  /// Linearly interpolate between two drawer themes.
  ///
  /// If both arguments are null then null is returned.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static DrawerThemeData? lerp(DrawerThemeData? a, DrawerThemeData? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    return DrawerThemeData(
      backgroundColor: Color.lerp(a?.backgroundColor, b?.backgroundColor, t),
      scrimColor: Color.lerp(a?.scrimColor, b?.scrimColor, t),
      elevation: lerpDouble(a?.elevation, b?.elevation, t),
      shadowColor: Color.lerp(a?.shadowColor, b?.shadowColor, t),
      surfaceTintColor: Color.lerp(a?.surfaceTintColor, b?.surfaceTintColor, t),
      shape: ShapeBorder.lerp(a?.shape, b?.shape, t),
      endShape: ShapeBorder.lerp(a?.endShape, b?.endShape, t),
      width: lerpDouble(a?.width, b?.width, t),
      clipBehavior: t < 0.5 ? a?.clipBehavior : b?.clipBehavior,
    );
  }

  @override
  int get hashCode => Object.hash(
    backgroundColor,
    scrimColor,
    elevation,
    shadowColor,
    surfaceTintColor,
    shape,
    endShape,
    width,
    clipBehavior,
  );

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is DrawerThemeData &&
        other.backgroundColor == backgroundColor &&
        other.scrimColor == scrimColor &&
        other.elevation == elevation &&
        other.shadowColor == shadowColor &&
        other.surfaceTintColor == surfaceTintColor &&
        other.shape == shape &&
        other.endShape == endShape &&
        other.width == width &&
        other.clipBehavior == clipBehavior;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(ColorProperty('backgroundColor', backgroundColor, defaultValue: null));
    properties.add(ColorProperty('scrimColor', scrimColor, defaultValue: null));
    properties.add(DoubleProperty('elevation', elevation, defaultValue: null));
    properties.add(ColorProperty('shadowColor', shadowColor, defaultValue: null));
    properties.add(ColorProperty('surfaceTintColor', surfaceTintColor, defaultValue: null));
    properties.add(DiagnosticsProperty<ShapeBorder>('shape', shape, defaultValue: null));
    properties.add(DiagnosticsProperty<ShapeBorder>('endShape', endShape, defaultValue: null));
    properties.add(DoubleProperty('width', width, defaultValue: null));
    properties.add(DiagnosticsProperty<Clip>('clipBehavior', clipBehavior, defaultValue: null));
  }
}

/// An inherited widget that defines visual properties for [Drawer]s in this
/// widget's subtree.
///
/// Values specified here are used for [Drawer] properties that are not
/// given an explicit non-null value.
///
/// Using this would allow you to override the [ThemeData.drawerTheme].
class DrawerTheme extends InheritedTheme {
  /// Creates a theme that defines the [DrawerThemeData] properties for a
  /// [Drawer].
  const DrawerTheme({super.key, required this.data, required super.child});

  /// Specifies the background color, scrim color, elevation, and shape for
  /// descendant [Drawer] widgets.
  final DrawerThemeData data;

  /// The closest instance of this class that encloses the given context.
  ///
  /// If there is no enclosing [DrawerTheme] widget, then
  /// [ThemeData.drawerTheme] is used.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// DrawerThemeData theme = DrawerTheme.of(context);
  /// ```
  static DrawerThemeData of(BuildContext context) {
    final DrawerTheme? drawerTheme = context.dependOnInheritedWidgetOfExactType<DrawerTheme>();
    return drawerTheme?.data ?? Theme.of(context).drawerTheme;
  }

  @override
  Widget wrap(BuildContext context, Widget child) {
    return DrawerTheme(data: data, child: child);
  }

  @override
  bool updateShouldNotify(DrawerTheme oldWidget) => data != oldWidget.data;
}
