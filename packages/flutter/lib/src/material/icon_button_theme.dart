// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'icon_button.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'button_style.dart';
import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// A [ButtonStyle] that overrides the default appearance of
/// [IconButton]s when it's used with the [IconButton], the [IconButtonTheme] or the
/// overall [Theme]'s [ThemeData.iconButtonTheme].
///
/// The [IconButton] will be affected by [IconButtonTheme] and [IconButtonThemeData]
/// only if [ThemeData.useMaterial3] is set to true; otherwise, [IconTheme] will be used.
///
/// The [style]'s properties override [IconButton]'s default style. Only
/// the style's non-null property values or resolved non-null
/// [WidgetStateProperty] values are used.
///
/// See also:
///
///  * [IconButtonTheme], the theme which is configured with this class.
///  * [IconButton.styleFrom], which converts simple values into a
///    [ButtonStyle] that's consistent with [IconButton]'s defaults.
///  * [WidgetStateProperty.resolve], "resolve" a material state property
///    to a simple value based on a set of [WidgetState]s.
///  * [ThemeData.iconButtonTheme], which can be used to override the default
///    [ButtonStyle] for [IconButton]s below the overall [Theme].
@immutable
class IconButtonThemeData with Diagnosticable {
  /// Creates a [IconButtonThemeData].
  ///
  /// The [style] may be null.
  const IconButtonThemeData({this.style});

  /// Overrides for [IconButton]'s default style if [ThemeData.useMaterial3]
  /// is set to true.
  ///
  /// Non-null properties or non-null resolved [WidgetStateProperty]
  /// values override the default [ButtonStyle] in [IconButton].
  ///
  /// If [style] is null, then this theme doesn't override anything.
  final ButtonStyle? style;

  /// Linearly interpolate between two icon button themes.
  static IconButtonThemeData? lerp(IconButtonThemeData? a, IconButtonThemeData? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    return IconButtonThemeData(style: ButtonStyle.lerp(a?.style, b?.style, t));
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
    return other is IconButtonThemeData && other.style == style;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<ButtonStyle>('style', style, defaultValue: null));
  }
}

/// Overrides the default [ButtonStyle] of its [IconButton] descendants.
///
/// See also:
///
///  * [IconButtonThemeData], which is used to configure this theme.
///  * [IconButton.styleFrom], which converts simple values into a
///    [ButtonStyle] that's consistent with [IconButton]'s defaults.
///  * [ThemeData.iconButtonTheme], which can be used to override the default
///    [ButtonStyle] for [IconButton]s below the overall [Theme].
class IconButtonTheme extends InheritedTheme {
  /// Create a [IconButtonTheme].
  const IconButtonTheme({super.key, required this.data, required super.child});

  /// The configuration of this theme.
  final IconButtonThemeData data;

  /// The closest instance of this class that encloses the given context.
  ///
  /// If there is no enclosing [IconButtonTheme] widget, then
  /// [ThemeData.iconButtonTheme] is used.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// IconButtonThemeData theme = IconButtonTheme.of(context);
  /// ```
  static IconButtonThemeData of(BuildContext context) {
    final IconButtonTheme? buttonTheme =
        context.dependOnInheritedWidgetOfExactType<IconButtonTheme>();
    return buttonTheme?.data ?? Theme.of(context).iconButtonTheme;
  }

  @override
  Widget wrap(BuildContext context, Widget child) {
    return IconButtonTheme(data: data, child: child);
  }

  @override
  bool updateShouldNotify(IconButtonTheme oldWidget) => data != oldWidget.data;
}
