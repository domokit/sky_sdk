// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'menu_theme.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'button_style.dart';
import 'menu_anchor.dart';
import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// A [ButtonStyle] theme that overrides the default appearance of
/// [SubmenuButton]s and [MenuItemButton]s when it's used with a
/// [MenuButtonTheme] or with the overall [Theme]'s [ThemeData.menuTheme].
///
/// The [style]'s properties override [MenuItemButton]'s and [SubmenuButton]'s
/// default style, i.e. the [ButtonStyle] returned by
/// [MenuItemButton.defaultStyleOf] and [SubmenuButton.defaultStyleOf]. Only the
/// style's non-null property values or resolved non-null
/// [WidgetStateProperty] values are used.
///
/// See also:
///
/// * [MenuButtonTheme], the theme which is configured with this class.
/// * [MenuTheme], the theme used to configure the look of the menus these
///   buttons reside in.
/// * [MenuItemButton.defaultStyleOf] and [SubmenuButton.defaultStyleOf] which
///   return the default [ButtonStyle]s for menu buttons.
/// * [MenuItemButton.styleFrom] and [SubmenuButton.styleFrom], which converts
///   simple values into a [ButtonStyle] that's consistent with their respective
///   defaults.
/// * [WidgetStateProperty.resolve], "resolve" a material state property to a
///   simple value based on a set of [WidgetState]s.
/// * [ThemeData.menuButtonTheme], which can be used to override the default
///   [ButtonStyle] for [MenuItemButton]s and [SubmenuButton]s below the overall
///   [Theme].
/// * [MenuAnchor], a widget which hosts cascading menus.
/// * [MenuBar], a widget which defines a menu bar of buttons hosting cascading
///   menus.
@immutable
class MenuButtonThemeData with Diagnosticable {
  /// Creates a [MenuButtonThemeData].
  ///
  /// The [style] may be null.
  const MenuButtonThemeData({this.style});

  /// Overrides for [SubmenuButton] and [MenuItemButton]'s default style.
  ///
  /// Non-null properties or non-null resolved [WidgetStateProperty] values
  /// override the [ButtonStyle] returned by [SubmenuButton.defaultStyleOf] or
  /// [MenuItemButton.defaultStyleOf].
  ///
  /// If [style] is null, then this theme doesn't override anything.
  final ButtonStyle? style;

  /// Linearly interpolate between two menu button themes.
  static MenuButtonThemeData? lerp(MenuButtonThemeData? a, MenuButtonThemeData? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    return MenuButtonThemeData(style: ButtonStyle.lerp(a?.style, b?.style, t));
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
    return other is MenuButtonThemeData && other.style == style;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<ButtonStyle>('style', style, defaultValue: null));
  }
}

/// Overrides the default [ButtonStyle] of its [MenuItemButton] and
/// [SubmenuButton] descendants.
///
/// See also:
///
/// * [MenuButtonThemeData], which is used to configure this theme.
/// * [MenuTheme], the theme used to configure the look of the menus themselves.
/// * [MenuItemButton.defaultStyleOf] and [SubmenuButton.defaultStyleOf] which
///   return the default [ButtonStyle]s for menu buttons.
/// * [MenuItemButton.styleFrom] and [SubmenuButton.styleFrom], which converts
///   simple values into a [ButtonStyle] that's consistent with their respective
///   defaults.
/// * [ThemeData.menuButtonTheme], which can be used to override the default
///   [ButtonStyle] for [MenuItemButton]s and [SubmenuButton]s below the overall
///   [Theme].
class MenuButtonTheme extends InheritedTheme {
  /// Create a [MenuButtonTheme].
  const MenuButtonTheme({super.key, required this.data, required super.child});

  /// The configuration of this theme.
  final MenuButtonThemeData data;

  /// The closest instance of this class that encloses the given context.
  ///
  /// If there is no enclosing [MenuButtonTheme] widget, then
  /// [ThemeData.menuButtonTheme] is used.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// MenuButtonThemeData theme = MenuButtonTheme.of(context);
  /// ```
  static MenuButtonThemeData of(BuildContext context) {
    final MenuButtonTheme? buttonTheme =
        context.dependOnInheritedWidgetOfExactType<MenuButtonTheme>();
    return buttonTheme?.data ?? Theme.of(context).menuButtonTheme;
  }

  @override
  Widget wrap(BuildContext context, Widget child) {
    return MenuButtonTheme(data: data, child: child);
  }

  @override
  bool updateShouldNotify(MenuButtonTheme oldWidget) => data != oldWidget.data;
}
