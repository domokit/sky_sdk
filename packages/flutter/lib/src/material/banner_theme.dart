// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'banner.dart';
library;

import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// Defines the visual properties of [MaterialBanner] widgets.
///
/// Descendant widgets obtain the current [MaterialBannerThemeData] object using
/// `MaterialBannerTheme.of(context)`. Instances of [MaterialBannerThemeData]
/// can be customized with [MaterialBannerThemeData.copyWith].
///
/// Typically a [MaterialBannerThemeData] is specified as part of the overall
/// [Theme] with [ThemeData.bannerTheme].
///
/// All [MaterialBannerThemeData] properties are `null` by default. When null,
/// the [MaterialBanner] will provide its own defaults.
///
/// See also:
///
///  * [ThemeData], which describes the overall theme information for the
///    application.
@immutable
class MaterialBannerThemeData with Diagnosticable {
  /// Creates a theme that can be used for [MaterialBannerTheme] or
  /// [ThemeData.bannerTheme].
  const MaterialBannerThemeData({
    this.backgroundColor,
    this.surfaceTintColor,
    this.shadowColor,
    this.dividerColor,
    this.contentTextStyle,
    this.elevation,
    this.padding,
    this.leadingPadding,
  });

  /// The background color of a [MaterialBanner].
  final Color? backgroundColor;

  /// Overrides the default value of [MaterialBanner.surfaceTintColor].
  final Color? surfaceTintColor;

  /// Overrides the default value of [MaterialBanner.shadowColor].
  final Color? shadowColor;

  /// Overrides the default value of [MaterialBanner.dividerColor].
  final Color? dividerColor;

  /// Used to configure the [DefaultTextStyle] for the [MaterialBanner.content]
  /// widget.
  final TextStyle? contentTextStyle;

  /// Default value for [MaterialBanner.elevation].
  //
  // If null, MaterialBanner uses a default of 0.0.
  final double? elevation;

  /// The amount of space by which to inset [MaterialBanner.content].
  final EdgeInsetsGeometry? padding;

  /// The amount of space by which to inset [MaterialBanner.leading].
  final EdgeInsetsGeometry? leadingPadding;

  /// Creates a copy of this object with the given fields replaced with the
  /// new values.
  MaterialBannerThemeData copyWith({
    Color? backgroundColor,
    Color? surfaceTintColor,
    Color? shadowColor,
    Color? dividerColor,
    TextStyle? contentTextStyle,
    double? elevation,
    EdgeInsetsGeometry? padding,
    EdgeInsetsGeometry? leadingPadding,
  }) {
    return MaterialBannerThemeData(
      backgroundColor: backgroundColor ?? this.backgroundColor,
      surfaceTintColor: surfaceTintColor ?? this.surfaceTintColor,
      shadowColor: shadowColor ?? this.shadowColor,
      dividerColor: dividerColor ?? this.dividerColor,
      contentTextStyle: contentTextStyle ?? this.contentTextStyle,
      elevation: elevation ?? this.elevation,
      padding: padding ?? this.padding,
      leadingPadding: leadingPadding ?? this.leadingPadding,
    );
  }

  /// Linearly interpolate between two Banner themes.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static MaterialBannerThemeData lerp(
    MaterialBannerThemeData? a,
    MaterialBannerThemeData? b,
    double t,
  ) {
    return MaterialBannerThemeData(
      backgroundColor: Color.lerp(a?.backgroundColor, b?.backgroundColor, t),
      surfaceTintColor: Color.lerp(a?.surfaceTintColor, b?.surfaceTintColor, t),
      shadowColor: Color.lerp(a?.shadowColor, b?.shadowColor, t),
      dividerColor: Color.lerp(a?.dividerColor, b?.dividerColor, t),
      contentTextStyle: TextStyle.lerp(a?.contentTextStyle, b?.contentTextStyle, t),
      elevation: lerpDouble(a?.elevation, b?.elevation, t),
      padding: EdgeInsetsGeometry.lerp(a?.padding, b?.padding, t),
      leadingPadding: EdgeInsetsGeometry.lerp(a?.leadingPadding, b?.leadingPadding, t),
    );
  }

  @override
  int get hashCode => Object.hash(
    backgroundColor,
    surfaceTintColor,
    shadowColor,
    dividerColor,
    contentTextStyle,
    elevation,
    padding,
    leadingPadding,
  );

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is MaterialBannerThemeData &&
        other.backgroundColor == backgroundColor &&
        other.surfaceTintColor == surfaceTintColor &&
        other.shadowColor == shadowColor &&
        other.dividerColor == dividerColor &&
        other.contentTextStyle == contentTextStyle &&
        other.elevation == elevation &&
        other.padding == padding &&
        other.leadingPadding == leadingPadding;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(ColorProperty('backgroundColor', backgroundColor, defaultValue: null));
    properties.add(ColorProperty('surfaceTintColor', surfaceTintColor, defaultValue: null));
    properties.add(ColorProperty('shadowColor', shadowColor, defaultValue: null));
    properties.add(ColorProperty('dividerColor', dividerColor, defaultValue: null));
    properties.add(
      DiagnosticsProperty<TextStyle>('contentTextStyle', contentTextStyle, defaultValue: null),
    );
    properties.add(DoubleProperty('elevation', elevation, defaultValue: null));
    properties.add(DiagnosticsProperty<EdgeInsetsGeometry>('padding', padding, defaultValue: null));
    properties.add(
      DiagnosticsProperty<EdgeInsetsGeometry>('leadingPadding', leadingPadding, defaultValue: null),
    );
  }
}

/// An inherited widget that defines the configuration for
/// [MaterialBanner]s in this widget's subtree.
///
/// Values specified here are used for [MaterialBanner] properties that are not
/// given an explicit non-null value.
class MaterialBannerTheme extends InheritedTheme {
  /// Creates a banner theme that controls the configurations for
  /// [MaterialBanner]s in its widget subtree.
  const MaterialBannerTheme({super.key, this.data, required super.child});

  /// The properties for descendant [MaterialBanner] widgets.
  final MaterialBannerThemeData? data;

  /// The closest instance of this class's [data] value that encloses the given
  /// context.
  ///
  /// If there is no ancestor, it returns [ThemeData.bannerTheme]. Applications
  /// can assume that the returned value will not be null.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// MaterialBannerThemeData theme = MaterialBannerTheme.of(context);
  /// ```
  static MaterialBannerThemeData of(BuildContext context) {
    final MaterialBannerTheme? bannerTheme =
        context.dependOnInheritedWidgetOfExactType<MaterialBannerTheme>();
    return bannerTheme?.data ?? Theme.of(context).bannerTheme;
  }

  @override
  Widget wrap(BuildContext context, Widget child) {
    return MaterialBannerTheme(data: data, child: child);
  }

  @override
  bool updateShouldNotify(MaterialBannerTheme oldWidget) => data != oldWidget.data;
}
