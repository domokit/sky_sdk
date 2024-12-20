// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'filled_button.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'button_style.dart';
import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// A [ButtonStyle] that overrides the default appearance of
/// [FilledButton]s when it's used with [FilledButtonTheme] or with the
/// overall [Theme]'s [ThemeData.filledButtonTheme].
///
/// The [style]'s properties override [FilledButton]'s default style,
/// i.e. the [ButtonStyle] returned by [FilledButton.defaultStyleOf]. Only
/// the style's non-null property values or resolved non-null
/// [WidgetStateProperty] values are used.
///
/// See also:
///
///  * [FilledButtonTheme], the theme which is configured with this class.
///  * [FilledButton.defaultStyleOf], which returns the default [ButtonStyle]
///    for text buttons.
///  * [FilledButton.styleFrom], which converts simple values into a
///    [ButtonStyle] that's consistent with [FilledButton]'s defaults.
///  * [WidgetStateProperty.resolve], "resolve" a material state property
///    to a simple value based on a set of [WidgetState]s.
///  * [ThemeData.filledButtonTheme], which can be used to override the default
///    [ButtonStyle] for [FilledButton]s below the overall [Theme].
@immutable
class FilledButtonThemeData with Diagnosticable {
  /// Creates an [FilledButtonThemeData].
  ///
  /// The [style] may be null.
  const FilledButtonThemeData({this.style});

  /// Overrides for [FilledButton]'s default style.
  ///
  /// Non-null properties or non-null resolved [WidgetStateProperty]
  /// values override the [ButtonStyle] returned by
  /// [FilledButton.defaultStyleOf].
  ///
  /// If [style] is null, then this theme doesn't override anything.
  final ButtonStyle? style;

  /// Linearly interpolate between two filled button themes.
  static FilledButtonThemeData? lerp(FilledButtonThemeData? a, FilledButtonThemeData? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    return FilledButtonThemeData(style: ButtonStyle.lerp(a?.style, b?.style, t));
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
    return other is FilledButtonThemeData && other.style == style;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<ButtonStyle>('style', style, defaultValue: null));
  }
}

/// Overrides the default [ButtonStyle] of its [FilledButton] descendants.
///
/// See also:
///
///  * [FilledButtonThemeData], which is used to configure this theme.
///  * [FilledButton.defaultStyleOf], which returns the default [ButtonStyle]
///    for filled buttons.
///  * [FilledButton.styleFrom], which converts simple values into a
///    [ButtonStyle] that's consistent with [FilledButton]'s defaults.
///  * [ThemeData.filledButtonTheme], which can be used to override the default
///    [ButtonStyle] for [FilledButton]s below the overall [Theme].
class FilledButtonTheme extends InheritedTheme {
  /// Create a [FilledButtonTheme].
  const FilledButtonTheme({super.key, required this.data, required super.child});

  /// The configuration of this theme.
  final FilledButtonThemeData data;

  /// The closest instance of this class that encloses the given context.
  ///
  /// If there is no enclosing [FilledButtonTheme] widget, then
  /// [ThemeData.filledButtonTheme] is used.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// FilledButtonThemeData theme = FilledButtonTheme.of(context);
  /// ```
  static FilledButtonThemeData of(BuildContext context) {
    final FilledButtonTheme? buttonTheme =
        context.dependOnInheritedWidgetOfExactType<FilledButtonTheme>();
    return buttonTheme?.data ?? Theme.of(context).filledButtonTheme;
  }

  @override
  Widget wrap(BuildContext context, Widget child) {
    return FilledButtonTheme(data: data, child: child);
  }

  @override
  bool updateShouldNotify(FilledButtonTheme oldWidget) => data != oldWidget.data;
}
