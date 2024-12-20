// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'menu_bar_theme.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'material_state.dart';
import 'menu_anchor.dart';
import 'menu_style.dart';
import 'theme.dart';

// Examples can assume:
// late Widget child;

/// Defines the configuration of the submenus created by the [SubmenuButton],
/// [MenuBar], or [MenuAnchor] widgets.
///
/// Descendant widgets obtain the current [MenuThemeData] object using
/// `MenuTheme.of(context)`.
///
/// Typically, a [MenuThemeData] is specified as part of the overall [Theme]
/// with [ThemeData.menuTheme]. Otherwise, [MenuTheme] can be used to configure
/// its own widget subtree.
///
/// All [MenuThemeData] properties are `null` by default. If any of these
/// properties are null, the menu bar will provide its own defaults.
///
/// See also:
///
/// * [ThemeData], which describes the overall theme for the application.
/// * [MenuBarThemeData], which describes the theme for the menu bar itself in a
///   [MenuBar] widget.
@immutable
class MenuThemeData with Diagnosticable {
  /// Creates a const set of properties used to configure [MenuTheme].
  const MenuThemeData({this.style, this.submenuIcon});

  /// The [MenuStyle] of a [SubmenuButton] menu.
  ///
  /// Any values not set in the [MenuStyle] will use the menu default for that
  /// property.
  final MenuStyle? style;

  /// If provided, the widget replaces the default [SubmenuButton] arrow icon.
  ///
  /// Resolves in the following states:
  ///  * [WidgetState.disabled].
  ///  * [WidgetState.hovered].
  ///  * [WidgetState.focused].
  final MaterialStateProperty<Widget?>? submenuIcon;

  /// Linearly interpolate between two menu button themes.
  static MenuThemeData? lerp(MenuThemeData? a, MenuThemeData? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    return MenuThemeData(
      style: MenuStyle.lerp(a?.style, b?.style, t),
      submenuIcon: t < 0.5 ? a?.submenuIcon : b?.submenuIcon,
    );
  }

  @override
  int get hashCode => Object.hash(style, submenuIcon);

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is MenuThemeData && other.style == style && other.submenuIcon == submenuIcon;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<MenuStyle>('style', style, defaultValue: null));
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<Widget?>>(
        'submenuIcon',
        submenuIcon,
        defaultValue: null,
      ),
    );
  }
}

/// An inherited widget that defines the configuration in this widget's
/// descendants for menus created by the [SubmenuButton], [MenuBar], or
/// [MenuAnchor] widgets.
///
/// Values specified here are used for [SubmenuButton]'s menu properties that
/// are not given an explicit non-null value.
///
/// See also:
///
/// * [MenuThemeData], a configuration object that holds attributes of a menu
///   used by this theme.
/// * [MenuBarTheme], which does the same thing for the [MenuBar] widget.
/// * [MenuBar], a widget that manages [MenuItemButton]s.
/// * [MenuAnchor], a widget that creates a region that has a submenu.
/// * [MenuItemButton], a widget that is a selectable item in a menu bar menu.
/// * [SubmenuButton], a widget that specifies an item with a cascading submenu
///   in a [MenuBar] menu.
class MenuTheme extends InheritedTheme {
  /// Creates a const theme that controls the configurations for the menus
  /// created by the [SubmenuButton] or [MenuAnchor] widgets.
  const MenuTheme({super.key, required this.data, required super.child});

  /// The properties for [MenuBar] and [MenuItemButton] in this widget's
  /// descendants.
  final MenuThemeData data;

  /// Returns the closest instance of this class's [data] value that encloses
  /// the given context. If there is no ancestor, it returns
  /// [ThemeData.menuTheme].
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// Widget build(BuildContext context) {
  ///   return MenuTheme(
  ///     data: const MenuThemeData(
  ///       style: MenuStyle(
  ///         backgroundColor: WidgetStatePropertyAll<Color>(Colors.red),
  ///       ),
  ///     ),
  ///     child: child,
  ///   );
  /// }
  /// ```
  static MenuThemeData of(BuildContext context) {
    final MenuTheme? menuTheme = context.dependOnInheritedWidgetOfExactType<MenuTheme>();
    return menuTheme?.data ?? Theme.of(context).menuTheme;
  }

  @override
  Widget wrap(BuildContext context, Widget child) {
    return MenuTheme(data: data, child: child);
  }

  @override
  bool updateShouldNotify(MenuTheme oldWidget) => data != oldWidget.data;
}
