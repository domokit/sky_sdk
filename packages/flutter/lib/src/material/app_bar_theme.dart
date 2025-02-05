// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'app_bar.dart';
library;

import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';

import 'theme.dart';

/// Overrides the default values of visual properties for descendant
/// [AppBar] widgets.
///
/// Descendant widgets obtain the current [AppBarTheme] object with
/// `AppBarTheme.of(context)`. Instances of [AppBarTheme] can be customized
/// with [AppBarTheme.copyWith].
///
/// Typically an [AppBarTheme] is specified as part of the overall [Theme] with
/// [ThemeData.appBarTheme].
///
/// All [AppBarTheme] properties are `null` by default. When null, the [AppBar]
/// compute its own default values, typically based on the overall theme's
/// [ThemeData.colorScheme], [ThemeData.textTheme], and [ThemeData.iconTheme].
@immutable
class AppBarTheme with Diagnosticable {
  /// Creates a theme that can be used for [ThemeData.appBarTheme].
  const AppBarTheme({
    Color? color,
    Color? backgroundColor,
    this.foregroundColor,
    this.elevation,
    this.scrolledUnderElevation,
    this.shadowColor,
    this.surfaceTintColor,
    this.shape,
    this.iconTheme,
    this.actionsIconTheme,
    this.centerTitle,
    this.titleSpacing,
    this.leadingWidth,
    this.toolbarHeight,
    this.toolbarTextStyle,
    this.titleTextStyle,
    this.systemOverlayStyle,
    this.actionsPadding,
  }) : assert(
         color == null || backgroundColor == null,
         'The color and backgroundColor parameters mean the same thing. Only specify one.',
       ),
       backgroundColor = backgroundColor ?? color;

  /// Overrides the default value of [AppBar.backgroundColor] in all
  /// descendant [AppBar] widgets.
  ///
  /// See also:
  ///
  ///  * [foregroundColor], which overrides the default value of
  ///    [AppBar.foregroundColor] in all descendant [AppBar] widgets.
  final Color? backgroundColor;

  /// Overrides the default value of [AppBar.foregroundColor] in all
  /// descendant [AppBar] widgets.
  ///
  /// See also:
  ///
  ///  * [backgroundColor], which overrides the default value of
  ///    [AppBar.backgroundColor] in all descendant [AppBar] widgets.
  final Color? foregroundColor;

  /// Overrides the default value of [AppBar.elevation] in all
  /// descendant [AppBar] widgets.
  final double? elevation;

  /// Overrides the default value of [AppBar.scrolledUnderElevation] in all
  /// descendant [AppBar] widgets.
  final double? scrolledUnderElevation;

  /// Overrides the default value of [AppBar.shadowColor] in all
  /// descendant [AppBar] widgets.
  final Color? shadowColor;

  /// Overrides the default value of [AppBar.surfaceTintColor] in all
  /// descendant [AppBar] widgets.
  final Color? surfaceTintColor;

  /// Overrides the default value of [AppBar.shape] in all
  /// descendant [AppBar] widgets.
  final ShapeBorder? shape;

  /// Overrides the default value of [AppBar.iconTheme] in all
  /// descendant [AppBar] widgets.
  ///
  /// See also:
  ///
  ///  * [actionsIconTheme], which overrides the default value of
  ///    [AppBar.actionsIconTheme] in all descendant [AppBar] widgets.
  ///  * [foregroundColor], which overrides the default value
  ///    [AppBar.foregroundColor] in all descendant [AppBar] widgets.
  final IconThemeData? iconTheme;

  /// Overrides the default value of [AppBar.actionsIconTheme] in all
  /// descendant [AppBar] widgets.
  ///
  /// See also:
  ///
  ///  * [iconTheme], which overrides the default value of
  ///    [AppBar.iconTheme] in all descendant [AppBar] widgets.
  ///  * [foregroundColor], which overrides the default value
  ///    [AppBar.foregroundColor] in all descendant [AppBar] widgets.
  final IconThemeData? actionsIconTheme;

  /// Overrides the default value of [AppBar.centerTitle]
  /// property in all descendant [AppBar] widgets.
  final bool? centerTitle;

  /// Overrides the default value of the obsolete [AppBar.titleSpacing]
  /// property in all descendant [AppBar] widgets.
  ///
  /// If null, [AppBar] uses default value of [NavigationToolbar.kMiddleSpacing].
  final double? titleSpacing;

  /// Overrides the default value of the [AppBar.leadingWidth]
  /// property in all descendant [AppBar] widgets.
  final double? leadingWidth;

  /// Overrides the default value of the [AppBar.toolbarHeight]
  /// property in all descendant [AppBar] widgets.
  ///
  /// See also:
  ///
  ///  * [AppBar.preferredHeightFor], which computes the overall
  ///    height of an AppBar widget, taking this value into account.
  final double? toolbarHeight;

  /// Overrides the default value of the obsolete [AppBar.toolbarTextStyle]
  /// property in all descendant [AppBar] widgets.
  ///
  /// See also:
  ///
  ///  * [titleTextStyle], which overrides the default of [AppBar.titleTextStyle]
  ///    in all descendant [AppBar] widgets.
  final TextStyle? toolbarTextStyle;

  /// Overrides the default value of [AppBar.titleTextStyle]
  /// property in all descendant [AppBar] widgets.
  ///
  /// See also:
  ///
  ///  * [toolbarTextStyle], which overrides the default of [AppBar.toolbarTextStyle]
  ///    in all descendant [AppBar] widgets.
  final TextStyle? titleTextStyle;

  /// Overrides the default value of [AppBar.systemOverlayStyle]
  /// property in all descendant [AppBar] widgets.
  final SystemUiOverlayStyle? systemOverlayStyle;

  /// Overrides the default value of [AppBar.actionsPadding]
  /// property in all descendant [AppBar] widgets.
  final EdgeInsetsGeometry? actionsPadding;

  /// Creates a copy of this object with the given fields replaced with the
  /// new values.
  AppBarTheme copyWith({
    IconThemeData? actionsIconTheme,
    Color? color,
    Color? backgroundColor,
    Color? foregroundColor,
    double? elevation,
    double? scrolledUnderElevation,
    Color? shadowColor,
    Color? surfaceTintColor,
    ShapeBorder? shape,
    IconThemeData? iconTheme,
    bool? centerTitle,
    double? titleSpacing,
    double? leadingWidth,
    double? toolbarHeight,
    TextStyle? toolbarTextStyle,
    TextStyle? titleTextStyle,
    SystemUiOverlayStyle? systemOverlayStyle,
    EdgeInsetsGeometry? actionsPadding,
  }) {
    assert(
      color == null || backgroundColor == null,
      'The color and backgroundColor parameters mean the same thing. Only specify one.',
    );
    return AppBarTheme(
      backgroundColor: backgroundColor ?? color ?? this.backgroundColor,
      foregroundColor: foregroundColor ?? this.foregroundColor,
      elevation: elevation ?? this.elevation,
      scrolledUnderElevation: scrolledUnderElevation ?? this.scrolledUnderElevation,
      shadowColor: shadowColor ?? this.shadowColor,
      surfaceTintColor: surfaceTintColor ?? this.surfaceTintColor,
      shape: shape ?? this.shape,
      iconTheme: iconTheme ?? this.iconTheme,
      actionsIconTheme: actionsIconTheme ?? this.actionsIconTheme,
      centerTitle: centerTitle ?? this.centerTitle,
      titleSpacing: titleSpacing ?? this.titleSpacing,
      leadingWidth: leadingWidth ?? this.leadingWidth,
      toolbarHeight: toolbarHeight ?? this.toolbarHeight,
      toolbarTextStyle: toolbarTextStyle ?? this.toolbarTextStyle,
      titleTextStyle: titleTextStyle ?? this.titleTextStyle,
      systemOverlayStyle: systemOverlayStyle ?? this.systemOverlayStyle,
      actionsPadding: actionsPadding ?? this.actionsPadding,
    );
  }

  /// The [ThemeData.appBarTheme] property of the ambient [Theme].
  static AppBarTheme of(BuildContext context) {
    return Theme.of(context).appBarTheme;
  }

  /// Linearly interpolate between two AppBar themes.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static AppBarTheme lerp(AppBarTheme? a, AppBarTheme? b, double t) {
    if (identical(a, b) && a != null) {
      return a;
    }
    return AppBarTheme(
      backgroundColor: Color.lerp(a?.backgroundColor, b?.backgroundColor, t),
      foregroundColor: Color.lerp(a?.foregroundColor, b?.foregroundColor, t),
      elevation: lerpDouble(a?.elevation, b?.elevation, t),
      scrolledUnderElevation: lerpDouble(a?.scrolledUnderElevation, b?.scrolledUnderElevation, t),
      shadowColor: Color.lerp(a?.shadowColor, b?.shadowColor, t),
      surfaceTintColor: Color.lerp(a?.surfaceTintColor, b?.surfaceTintColor, t),
      shape: ShapeBorder.lerp(a?.shape, b?.shape, t),
      iconTheme: IconThemeData.lerp(a?.iconTheme, b?.iconTheme, t),
      actionsIconTheme: IconThemeData.lerp(a?.actionsIconTheme, b?.actionsIconTheme, t),
      centerTitle: t < 0.5 ? a?.centerTitle : b?.centerTitle,
      titleSpacing: lerpDouble(a?.titleSpacing, b?.titleSpacing, t),
      leadingWidth: lerpDouble(a?.leadingWidth, b?.leadingWidth, t),
      toolbarHeight: lerpDouble(a?.toolbarHeight, b?.toolbarHeight, t),
      toolbarTextStyle: TextStyle.lerp(a?.toolbarTextStyle, b?.toolbarTextStyle, t),
      titleTextStyle: TextStyle.lerp(a?.titleTextStyle, b?.titleTextStyle, t),
      systemOverlayStyle: t < 0.5 ? a?.systemOverlayStyle : b?.systemOverlayStyle,
      actionsPadding: EdgeInsetsGeometry.lerp(a?.actionsPadding, b?.actionsPadding, t),
    );
  }

  @override
  int get hashCode => Object.hash(
    backgroundColor,
    foregroundColor,
    elevation,
    scrolledUnderElevation,
    shadowColor,
    surfaceTintColor,
    shape,
    iconTheme,
    actionsIconTheme,
    centerTitle,
    titleSpacing,
    leadingWidth,
    toolbarHeight,
    toolbarTextStyle,
    titleTextStyle,
    systemOverlayStyle,
    actionsPadding,
  );

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is AppBarTheme &&
        other.backgroundColor == backgroundColor &&
        other.foregroundColor == foregroundColor &&
        other.elevation == elevation &&
        other.scrolledUnderElevation == scrolledUnderElevation &&
        other.shadowColor == shadowColor &&
        other.surfaceTintColor == surfaceTintColor &&
        other.shape == shape &&
        other.iconTheme == iconTheme &&
        other.actionsIconTheme == actionsIconTheme &&
        other.centerTitle == centerTitle &&
        other.titleSpacing == titleSpacing &&
        other.leadingWidth == leadingWidth &&
        other.toolbarHeight == toolbarHeight &&
        other.toolbarTextStyle == toolbarTextStyle &&
        other.titleTextStyle == titleTextStyle &&
        other.systemOverlayStyle == systemOverlayStyle &&
        other.actionsPadding == actionsPadding;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(ColorProperty('backgroundColor', backgroundColor, defaultValue: null));
    properties.add(ColorProperty('foregroundColor', foregroundColor, defaultValue: null));
    properties.add(DiagnosticsProperty<double>('elevation', elevation, defaultValue: null));
    properties.add(
      DiagnosticsProperty<double>(
        'scrolledUnderElevation',
        scrolledUnderElevation,
        defaultValue: null,
      ),
    );
    properties.add(ColorProperty('shadowColor', shadowColor, defaultValue: null));
    properties.add(ColorProperty('surfaceTintColor', surfaceTintColor, defaultValue: null));
    properties.add(DiagnosticsProperty<ShapeBorder>('shape', shape, defaultValue: null));
    properties.add(DiagnosticsProperty<IconThemeData>('iconTheme', iconTheme, defaultValue: null));
    properties.add(
      DiagnosticsProperty<IconThemeData>('actionsIconTheme', actionsIconTheme, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<bool>('centerTitle', centerTitle, defaultValue: null));
    properties.add(DiagnosticsProperty<double>('titleSpacing', titleSpacing, defaultValue: null));
    properties.add(DiagnosticsProperty<double>('leadingWidth', leadingWidth, defaultValue: null));
    properties.add(DiagnosticsProperty<double>('toolbarHeight', toolbarHeight, defaultValue: null));
    properties.add(
      DiagnosticsProperty<TextStyle>('toolbarTextStyle', toolbarTextStyle, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>('titleTextStyle', titleTextStyle, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<EdgeInsetsGeometry>('actionsPadding', actionsPadding, defaultValue: null),
    );
  }
}
