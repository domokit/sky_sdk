// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'badge.dart';
library;

import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// Overrides the default properties values for descendant [Badge] widgets.
///
/// Descendant widgets obtain the current [BadgeThemeData] object
/// using `BadgeTheme.of(context)`. Instances of [BadgeThemeData] can
/// be customized with [BadgeThemeData.copyWith].
///
/// Typically a [BadgeThemeData] is specified as part of the
/// overall [Theme] with [ThemeData.badgeTheme].
///
/// All [BadgeThemeData] properties are `null` by default.
/// When null, the [Badge] will use the values from [ThemeData]
/// if they exist, otherwise it will provide its own defaults.
///
/// See also:
///
///  * [ThemeData], which describes the overall theme information for the
///    application.
@immutable
class BadgeThemeData with Diagnosticable {
  /// Creates the set of color, style, and size properties used to configure [Badge].
  const BadgeThemeData({
    this.backgroundColor,
    this.textColor,
    this.smallSize,
    this.largeSize,
    this.textStyle,
    this.padding,
    this.alignment,
    this.offset,
  });

  /// Overrides the default value for [Badge.backgroundColor].
  final Color? backgroundColor;

  /// Overrides the default value for [Badge.textColor].
  final Color? textColor;

  /// Overrides the default value for [Badge.smallSize].
  final double? smallSize;

  /// Overrides the default value for [Badge.largeSize].
  final double? largeSize;

  /// Overrides the default value for [Badge.textStyle].
  final TextStyle? textStyle;

  /// Overrides the default value for [Badge.padding].
  final EdgeInsetsGeometry? padding;

  /// Overrides the default value for [Badge.alignment].
  final AlignmentGeometry? alignment;

  /// Overrides the default value for [Badge.offset].
  final Offset? offset;

  /// Creates a copy of this object but with the given fields replaced with the
  /// new values.
  BadgeThemeData copyWith({
    Color? backgroundColor,
    Color? textColor,
    double? smallSize,
    double? largeSize,
    TextStyle? textStyle,
    EdgeInsetsGeometry? padding,
    AlignmentGeometry? alignment,
    Offset? offset,
  }) {
    return BadgeThemeData(
      backgroundColor: backgroundColor ?? this.backgroundColor,
      textColor: textColor ?? this.textColor,
      smallSize: smallSize ?? this.smallSize,
      largeSize: largeSize ?? this.largeSize,
      textStyle: textStyle ?? this.textStyle,
      padding: padding ?? this.padding,
      alignment: alignment ?? this.alignment,
      offset: offset ?? this.offset,
    );
  }

  /// Linearly interpolate between two [Badge] themes.
  static BadgeThemeData lerp(BadgeThemeData? a, BadgeThemeData? b, double t) {
    if (identical(a, b) && a != null) {
      return a;
    }
    return BadgeThemeData(
      backgroundColor: Color.lerp(a?.backgroundColor, b?.backgroundColor, t),
      textColor: Color.lerp(a?.textColor, b?.textColor, t),
      smallSize: lerpDouble(a?.smallSize, b?.smallSize, t),
      largeSize: lerpDouble(a?.largeSize, b?.largeSize, t),
      textStyle: TextStyle.lerp(a?.textStyle, b?.textStyle, t),
      padding: EdgeInsetsGeometry.lerp(a?.padding, b?.padding, t),
      alignment: AlignmentGeometry.lerp(a?.alignment, b?.alignment, t),
      offset: Offset.lerp(a?.offset, b?.offset, t),
    );
  }

  @override
  int get hashCode => Object.hash(
    backgroundColor,
    textColor,
    smallSize,
    largeSize,
    textStyle,
    padding,
    alignment,
    offset,
  );

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is BadgeThemeData &&
        other.backgroundColor == backgroundColor &&
        other.textColor == textColor &&
        other.smallSize == smallSize &&
        other.largeSize == largeSize &&
        other.textStyle == textStyle &&
        other.padding == padding &&
        other.alignment == alignment &&
        other.offset == offset;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(ColorProperty('backgroundColor', backgroundColor, defaultValue: null));
    properties.add(ColorProperty('textColor', textColor, defaultValue: null));
    properties.add(DoubleProperty('smallSize', smallSize, defaultValue: null));
    properties.add(DoubleProperty('largeSize', largeSize, defaultValue: null));
    properties.add(DiagnosticsProperty<TextStyle>('textStyle', textStyle, defaultValue: null));
    properties.add(DiagnosticsProperty<EdgeInsetsGeometry>('padding', padding, defaultValue: null));
    properties.add(
      DiagnosticsProperty<AlignmentGeometry>('alignment', alignment, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<Offset>('offset', offset, defaultValue: null));
  }
}

/// An inherited widget that overrides the default color style, and size
/// parameters for [Badge]s in this widget's subtree.
///
/// Values specified here override the defaults for [Badge] properties which
/// are not given an explicit non-null value.
class BadgeTheme extends InheritedTheme {
  /// Creates a theme that overrides the default color parameters for [Badge]s
  /// in this widget's subtree.
  const BadgeTheme({super.key, required this.data, required super.child});

  /// Specifies the default color and size overrides for descendant [Badge] widgets.
  final BadgeThemeData data;

  /// The closest instance of this class that encloses the given context.
  ///
  /// If there is no enclosing [BadgeTheme] widget, then
  /// [ThemeData.badgeTheme] is used.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// BadgeThemeData theme = BadgeTheme.of(context);
  /// ```
  static BadgeThemeData of(BuildContext context) {
    final BadgeTheme? badgeTheme = context.dependOnInheritedWidgetOfExactType<BadgeTheme>();
    return badgeTheme?.data ?? Theme.of(context).badgeTheme;
  }

  @override
  Widget wrap(BuildContext context, Widget child) {
    return BadgeTheme(data: data, child: child);
  }

  @override
  bool updateShouldNotify(BadgeTheme oldWidget) => data != oldWidget.data;
}
