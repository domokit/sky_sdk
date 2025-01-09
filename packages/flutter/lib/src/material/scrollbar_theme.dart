// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'scrollbar.dart';
library;

import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'material_state.dart';
import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// Defines default property values for descendant [Scrollbar] widgets.
///
/// Descendant widgets obtain the current [ScrollbarThemeData] object with
/// `ScrollbarTheme.of(context)`. Instances of [ScrollbarThemeData] can be
/// customized with [ScrollbarThemeData.copyWith].
///
/// Typically the [ScrollbarThemeData] of a [ScrollbarTheme] is specified as
/// part of the overall [Theme] with [ThemeData.scrollbarTheme].
///
/// All [ScrollbarThemeData] properties are `null` by default. When null, the
/// [Scrollbar] computes its own default values, typically based on the overall
/// theme's [ThemeData.colorScheme].
///
/// See also:
///
///  * [ThemeData], which describes the overall theme information for the
///    application.
@immutable
class ScrollbarThemeData with Diagnosticable {
  /// Creates a theme that can be used for [ThemeData.scrollbarTheme].
  const ScrollbarThemeData({
    this.thumbVisibility,
    this.thickness,
    this.trackVisibility,
    this.radius,
    this.thumbColor,
    this.trackColor,
    this.trackBorderColor,
    this.crossAxisMargin,
    this.mainAxisMargin,
    this.minThumbLength,
    this.interactive,
  });

  /// Overrides the default value of [Scrollbar.thumbVisibility] in all
  /// descendant [Scrollbar] widgets.
  final MaterialStateProperty<bool?>? thumbVisibility;

  /// Overrides the default value of [Scrollbar.thickness] in all
  /// descendant [Scrollbar] widgets.
  ///
  /// Resolves in the following states:
  ///  * [WidgetState.hovered] on web and desktop platforms.
  final MaterialStateProperty<double?>? thickness;

  /// Overrides the default value of [Scrollbar.trackVisibility] in all
  /// descendant [Scrollbar] widgets.
  final MaterialStateProperty<bool?>? trackVisibility;

  /// Overrides the default value of [Scrollbar.interactive] in all
  /// descendant [Scrollbar] widgets.
  final bool? interactive;

  /// Overrides the default value of [Scrollbar.radius] in all
  /// descendant widgets.
  final Radius? radius;

  /// Overrides the default [Color] of the [Scrollbar] thumb in all descendant
  /// [Scrollbar] widgets.
  ///
  /// Resolves in the following states:
  ///  * [WidgetState.dragged].
  ///  * [WidgetState.hovered] on web and desktop platforms.
  final MaterialStateProperty<Color?>? thumbColor;

  /// Overrides the default [Color] of the [Scrollbar] track when
  /// [trackVisibility] is true in all descendant [Scrollbar] widgets.
  ///
  /// Resolves in the following states:
  ///  * [WidgetState.hovered] on web and desktop platforms.
  final MaterialStateProperty<Color?>? trackColor;

  /// Overrides the default [Color] of the [Scrollbar] track border when
  /// [trackVisibility] is true in all descendant [Scrollbar] widgets.
  ///
  /// Resolves in the following states:
  ///  * [WidgetState.hovered] on web and desktop platforms.
  final MaterialStateProperty<Color?>? trackBorderColor;

  /// Overrides the default value of the [ScrollbarPainter.crossAxisMargin]
  /// property in all descendant [Scrollbar] widgets.
  ///
  /// See also:
  ///
  ///  * [ScrollbarPainter.crossAxisMargin], which sets the distance from the
  ///    scrollbar's side to the nearest edge in logical pixels.
  final double? crossAxisMargin;

  /// Overrides the default value of the [ScrollbarPainter.mainAxisMargin]
  /// property in all descendant [Scrollbar] widgets.
  ///
  /// See also:
  ///
  ///  * [ScrollbarPainter.mainAxisMargin], which sets the distance from the
  ///    scrollbar's start and end to the edge of the viewport in logical pixels.
  final double? mainAxisMargin;

  /// Overrides the default value of the [ScrollbarPainter.minLength]
  /// property in all descendant [Scrollbar] widgets.
  ///
  /// See also:
  ///
  ///  * [ScrollbarPainter.minLength], which sets the preferred smallest size
  ///    the scrollbar can shrink to when the total scrollable extent is large,
  ///    the current visible viewport is small, and the viewport is not
  ///    overscrolled.
  final double? minThumbLength;

  /// Creates a copy of this object with the given fields replaced with the
  /// new values.
  ScrollbarThemeData copyWith({
    MaterialStateProperty<bool?>? thumbVisibility,
    MaterialStateProperty<double?>? thickness,
    MaterialStateProperty<bool?>? trackVisibility,
    bool? interactive,
    Radius? radius,
    MaterialStateProperty<Color?>? thumbColor,
    MaterialStateProperty<Color?>? trackColor,
    MaterialStateProperty<Color?>? trackBorderColor,
    double? crossAxisMargin,
    double? mainAxisMargin,
    double? minThumbLength,
  }) {
    return ScrollbarThemeData(
      thumbVisibility: thumbVisibility ?? this.thumbVisibility,
      thickness: thickness ?? this.thickness,
      trackVisibility: trackVisibility ?? this.trackVisibility,
      interactive: interactive ?? this.interactive,
      radius: radius ?? this.radius,
      thumbColor: thumbColor ?? this.thumbColor,
      trackColor: trackColor ?? this.trackColor,
      trackBorderColor: trackBorderColor ?? this.trackBorderColor,
      crossAxisMargin: crossAxisMargin ?? this.crossAxisMargin,
      mainAxisMargin: mainAxisMargin ?? this.mainAxisMargin,
      minThumbLength: minThumbLength ?? this.minThumbLength,
    );
  }

  /// Linearly interpolate between two Scrollbar themes.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static ScrollbarThemeData lerp(ScrollbarThemeData? a, ScrollbarThemeData? b, double t) {
    if (identical(a, b) && a != null) {
      return a;
    }
    return ScrollbarThemeData(
      thumbVisibility: MaterialStateProperty.lerp<bool?>(
        a?.thumbVisibility,
        b?.thumbVisibility,
        t,
        _lerpBool,
      ),
      thickness: MaterialStateProperty.lerp<double?>(a?.thickness, b?.thickness, t, lerpDouble),
      trackVisibility: MaterialStateProperty.lerp<bool?>(
        a?.trackVisibility,
        b?.trackVisibility,
        t,
        _lerpBool,
      ),
      interactive: _lerpBool(a?.interactive, b?.interactive, t),
      radius: Radius.lerp(a?.radius, b?.radius, t),
      thumbColor: MaterialStateProperty.lerp<Color?>(a?.thumbColor, b?.thumbColor, t, Color.lerp),
      trackColor: MaterialStateProperty.lerp<Color?>(a?.trackColor, b?.trackColor, t, Color.lerp),
      trackBorderColor: MaterialStateProperty.lerp<Color?>(
        a?.trackBorderColor,
        b?.trackBorderColor,
        t,
        Color.lerp,
      ),
      crossAxisMargin: lerpDouble(a?.crossAxisMargin, b?.crossAxisMargin, t),
      mainAxisMargin: lerpDouble(a?.mainAxisMargin, b?.mainAxisMargin, t),
      minThumbLength: lerpDouble(a?.minThumbLength, b?.minThumbLength, t),
    );
  }

  @override
  int get hashCode => Object.hash(
    thumbVisibility,
    thickness,
    trackVisibility,
    interactive,
    radius,
    thumbColor,
    trackColor,
    trackBorderColor,
    crossAxisMargin,
    mainAxisMargin,
    minThumbLength,
  );

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is ScrollbarThemeData &&
        other.thumbVisibility == thumbVisibility &&
        other.thickness == thickness &&
        other.trackVisibility == trackVisibility &&
        other.interactive == interactive &&
        other.radius == radius &&
        other.thumbColor == thumbColor &&
        other.trackColor == trackColor &&
        other.trackBorderColor == trackBorderColor &&
        other.crossAxisMargin == crossAxisMargin &&
        other.mainAxisMargin == mainAxisMargin &&
        other.minThumbLength == minThumbLength;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<bool?>>(
        'thumbVisibility',
        thumbVisibility,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<double?>>(
        'thickness',
        thickness,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<bool?>>(
        'trackVisibility',
        trackVisibility,
        defaultValue: null,
      ),
    );
    properties.add(DiagnosticsProperty<bool>('interactive', interactive, defaultValue: null));
    properties.add(DiagnosticsProperty<Radius>('radius', radius, defaultValue: null));
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<Color?>>(
        'thumbColor',
        thumbColor,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<Color?>>(
        'trackColor',
        trackColor,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<Color?>>(
        'trackBorderColor',
        trackBorderColor,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<double>('crossAxisMargin', crossAxisMargin, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<double>('mainAxisMargin', mainAxisMargin, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<double>('minThumbLength', minThumbLength, defaultValue: null),
    );
  }
}

bool? _lerpBool(bool? a, bool? b, double t) => t < 0.5 ? a : b;

/// Applies a scrollbar theme to descendant [Scrollbar] widgets.
///
/// Descendant widgets obtain the current theme's [ScrollbarThemeData] using
/// [ScrollbarTheme.of]. When a widget uses [ScrollbarTheme.of], it is
/// automatically rebuilt if the theme later changes.
///
/// A scrollbar theme can be specified as part of the overall Material theme
/// using [ThemeData.scrollbarTheme].
///
/// See also:
///
///  * [ScrollbarThemeData], which describes the configuration of a
///    scrollbar theme.
class ScrollbarTheme extends InheritedTheme {
  /// Constructs a scrollbar theme that configures all descendant [Scrollbar]
  /// widgets.
  const ScrollbarTheme({super.key, required this.data, required super.child});

  /// The properties used for all descendant [Scrollbar] widgets.
  final ScrollbarThemeData data;

  /// Returns the configuration [data] from the closest [ScrollbarTheme]
  /// ancestor. If there is no ancestor, it returns [ThemeData.scrollbarTheme].
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// ScrollbarThemeData theme = ScrollbarTheme.of(context);
  /// ```
  static ScrollbarThemeData of(BuildContext context) {
    final ScrollbarTheme? scrollbarTheme =
        context.dependOnInheritedWidgetOfExactType<ScrollbarTheme>();
    return scrollbarTheme?.data ?? Theme.of(context).scrollbarTheme;
  }

  @override
  Widget wrap(BuildContext context, Widget child) {
    return ScrollbarTheme(data: data, child: child);
  }

  @override
  bool updateShouldNotify(ScrollbarTheme oldWidget) => data != oldWidget.data;
}
