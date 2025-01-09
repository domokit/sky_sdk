// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'outlined_button.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'button_style.dart';
import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// A [ButtonStyle] that overrides the default appearance of
/// [OutlinedButton]s when it's used with [OutlinedButtonTheme] or with the
/// overall [Theme]'s [ThemeData.outlinedButtonTheme].
///
/// The [style]'s properties override [OutlinedButton]'s default style,
/// i.e. the [ButtonStyle] returned by [OutlinedButton.defaultStyleOf]. Only
/// the style's non-null property values or resolved non-null
/// [WidgetStateProperty] values are used.
///
/// See also:
///
///  * [OutlinedButtonTheme], the theme which is configured with this class.
///  * [OutlinedButton.defaultStyleOf], which returns the default [ButtonStyle]
///    for outlined buttons.
///  * [OutlinedButton.styleFrom], which converts simple values into a
///    [ButtonStyle] that's consistent with [OutlinedButton]'s defaults.
///  * [WidgetStateProperty.resolve], "resolve" a material state property
///    to a simple value based on a set of [WidgetState]s.
///  * [ThemeData.outlinedButtonTheme], which can be used to override the default
///    [ButtonStyle] for [OutlinedButton]s below the overall [Theme].
@immutable
class OutlinedButtonThemeData with Diagnosticable {
  /// Creates a [OutlinedButtonThemeData].
  ///
  /// The [style] may be null.
  const OutlinedButtonThemeData({this.style});

  /// Overrides for [OutlinedButton]'s default style.
  ///
  /// Non-null properties or non-null resolved [WidgetStateProperty]
  /// values override the [ButtonStyle] returned by
  /// [OutlinedButton.defaultStyleOf].
  ///
  /// If [style] is null, then this theme doesn't override anything.
  final ButtonStyle? style;

  /// Linearly interpolate between two outlined button themes.
  static OutlinedButtonThemeData? lerp(
    OutlinedButtonThemeData? a,
    OutlinedButtonThemeData? b,
    double t,
  ) {
    if (identical(a, b)) {
      return a;
    }
    return OutlinedButtonThemeData(style: ButtonStyle.lerp(a?.style, b?.style, t));
  }

  @override
  int get hashCode => style.hashCode;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is OutlinedButtonThemeData && other.style == style;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<ButtonStyle>('style', style, defaultValue: null));
  }
}

/// Overrides the default [ButtonStyle] of its [OutlinedButton] descendants.
///
/// See also:
///
///  * [OutlinedButtonThemeData], which is used to configure this theme.
///  * [OutlinedButton.defaultStyleOf], which returns the default [ButtonStyle]
///    for outlined buttons.
///  * [OutlinedButton.styleFrom], which converts simple values into a
///    [ButtonStyle] that's consistent with [OutlinedButton]'s defaults.
///  * [ThemeData.outlinedButtonTheme], which can be used to override the default
///    [ButtonStyle] for [OutlinedButton]s below the overall [Theme].
class OutlinedButtonTheme extends InheritedTheme {
  /// Create a [OutlinedButtonTheme].
  const OutlinedButtonTheme({super.key, required this.data, required super.child});

  /// The configuration of this theme.
  final OutlinedButtonThemeData data;

  /// The closest instance of this class that encloses the given context.
  ///
  /// If there is no enclosing [OutlinedButtonTheme] widget, then
  /// [ThemeData.outlinedButtonTheme] is used.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// OutlinedButtonThemeData theme = OutlinedButtonTheme.of(context);
  /// ```
  static OutlinedButtonThemeData of(BuildContext context) {
    final OutlinedButtonTheme? buttonTheme =
        context.dependOnInheritedWidgetOfExactType<OutlinedButtonTheme>();
    return buttonTheme?.data ?? Theme.of(context).outlinedButtonTheme;
  }

  @override
  Widget wrap(BuildContext context, Widget child) {
    return OutlinedButtonTheme(data: data, child: child);
  }

  @override
  bool updateShouldNotify(OutlinedButtonTheme oldWidget) => data != oldWidget.data;
}
