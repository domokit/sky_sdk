// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'navigation_bar.dart';
library;

import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'material_state.dart';
import 'navigation_drawer.dart';
import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// Defines default property values for descendant [NavigationDrawer]
/// widgets.
///
/// Descendant widgets obtain the current [NavigationDrawerThemeData] object
/// using `NavigationDrawerTheme.of(context)`. Instances of
/// [NavigationDrawerThemeData] can be customized with
/// [NavigationDrawerThemeData.copyWith].
///
/// Typically a [NavigationDrawerThemeData] is specified as part of the
/// overall [Theme] with [ThemeData.navigationDrawerTheme]. Alternatively, a
/// [NavigationDrawerTheme] inherited widget can be used to theme [NavigationDrawer]s
/// in a subtree of widgets.
///
/// All [NavigationDrawerThemeData] properties are `null` by default.
/// When null, the [NavigationDrawer] will provide its own defaults based on the
/// overall [Theme]'s textTheme and colorScheme. See the individual
/// [NavigationDrawer] properties for details.
///
/// See also:
///
///  * [ThemeData], which describes the overall theme information for the
///    application.
@immutable
class NavigationDrawerThemeData with Diagnosticable {
  /// Creates a theme that can be used for [ThemeData.navigationDrawerTheme] and
  /// [NavigationDrawerTheme].
  const NavigationDrawerThemeData({
    this.tileHeight,
    this.backgroundColor,
    this.elevation,
    this.shadowColor,
    this.surfaceTintColor,
    this.indicatorColor,
    this.indicatorShape,
    this.indicatorSize,
    this.labelTextStyle,
    this.iconTheme,
  });

  /// Overrides the default height of [NavigationDrawerDestination].
  final double? tileHeight;

  /// Overrides the default value of [NavigationDrawer.backgroundColor].
  final Color? backgroundColor;

  /// Overrides the default value of [NavigationDrawer.elevation].
  final double? elevation;

  /// Overrides the default value of [NavigationDrawer.shadowColor].
  final Color? shadowColor;

  /// Overrides the default value of [NavigationDrawer.surfaceTintColor].
  final Color? surfaceTintColor;

  /// Overrides the default value of [NavigationDrawer]'s selection indicator.
  final Color? indicatorColor;

  /// Overrides the default shape of the [NavigationDrawer]'s selection indicator.
  final ShapeBorder? indicatorShape;

  /// Overrides the default size of the [NavigationDrawer]'s selection indicator.
  final Size? indicatorSize;

  /// The style to merge with the default text style for
  /// [NavigationDestination] labels.
  ///
  /// You can use this to specify a different style when the label is selected.
  final MaterialStateProperty<TextStyle?>? labelTextStyle;

  /// The theme to merge with the default icon theme for
  /// [NavigationDestination] icons.
  ///
  /// You can use this to specify a different icon theme when the icon is
  /// selected.
  final MaterialStateProperty<IconThemeData?>? iconTheme;

  /// Creates a copy of this object with the given fields replaced with the
  /// new values.
  NavigationDrawerThemeData copyWith({
    double? tileHeight,
    Color? backgroundColor,
    double? elevation,
    Color? shadowColor,
    Color? surfaceTintColor,
    Color? indicatorColor,
    ShapeBorder? indicatorShape,
    Size? indicatorSize,
    MaterialStateProperty<TextStyle?>? labelTextStyle,
    MaterialStateProperty<IconThemeData?>? iconTheme,
  }) {
    return NavigationDrawerThemeData(
      tileHeight: tileHeight ?? this.tileHeight,
      backgroundColor: backgroundColor ?? this.backgroundColor,
      elevation: elevation ?? this.elevation,
      shadowColor: shadowColor ?? this.shadowColor,
      surfaceTintColor: surfaceTintColor ?? this.surfaceTintColor,
      indicatorColor: indicatorColor ?? this.indicatorColor,
      indicatorShape: indicatorShape ?? this.indicatorShape,
      indicatorSize: indicatorSize ?? this.indicatorSize,
      labelTextStyle: labelTextStyle ?? this.labelTextStyle,
      iconTheme: iconTheme ?? this.iconTheme,
    );
  }

  /// Linearly interpolate between two navigation rail themes.
  ///
  /// If both arguments are null then null is returned.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static NavigationDrawerThemeData? lerp(
    NavigationDrawerThemeData? a,
    NavigationDrawerThemeData? b,
    double t,
  ) {
    if (identical(a, b)) {
      return a;
    }
    return NavigationDrawerThemeData(
      tileHeight: lerpDouble(a?.tileHeight, b?.tileHeight, t),
      backgroundColor: Color.lerp(a?.backgroundColor, b?.backgroundColor, t),
      elevation: lerpDouble(a?.elevation, b?.elevation, t),
      shadowColor: Color.lerp(a?.shadowColor, b?.shadowColor, t),
      surfaceTintColor: Color.lerp(a?.surfaceTintColor, b?.surfaceTintColor, t),
      indicatorColor: Color.lerp(a?.indicatorColor, b?.indicatorColor, t),
      indicatorShape: ShapeBorder.lerp(a?.indicatorShape, b?.indicatorShape, t),
      indicatorSize: Size.lerp(a?.indicatorSize, a?.indicatorSize, t),
      labelTextStyle: MaterialStateProperty.lerp<TextStyle?>(
        a?.labelTextStyle,
        b?.labelTextStyle,
        t,
        TextStyle.lerp,
      ),
      iconTheme: MaterialStateProperty.lerp<IconThemeData?>(
        a?.iconTheme,
        b?.iconTheme,
        t,
        IconThemeData.lerp,
      ),
    );
  }

  @override
  int get hashCode => Object.hash(
    tileHeight,
    backgroundColor,
    elevation,
    shadowColor,
    surfaceTintColor,
    indicatorColor,
    indicatorShape,
    indicatorSize,
    labelTextStyle,
    iconTheme,
  );

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is NavigationDrawerThemeData &&
        other.tileHeight == tileHeight &&
        other.backgroundColor == backgroundColor &&
        other.elevation == elevation &&
        other.shadowColor == shadowColor &&
        other.surfaceTintColor == surfaceTintColor &&
        other.indicatorColor == indicatorColor &&
        other.indicatorShape == indicatorShape &&
        other.indicatorSize == indicatorSize &&
        other.labelTextStyle == labelTextStyle &&
        other.iconTheme == iconTheme;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('tileHeight', tileHeight, defaultValue: null));
    properties.add(ColorProperty('backgroundColor', backgroundColor, defaultValue: null));
    properties.add(DoubleProperty('elevation', elevation, defaultValue: null));
    properties.add(ColorProperty('shadowColor', shadowColor, defaultValue: null));
    properties.add(ColorProperty('surfaceTintColor', surfaceTintColor, defaultValue: null));
    properties.add(ColorProperty('indicatorColor', indicatorColor, defaultValue: null));
    properties.add(
      DiagnosticsProperty<ShapeBorder>('indicatorShape', indicatorShape, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<Size>('indicatorSize', indicatorSize, defaultValue: null));
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<TextStyle?>>(
        'labelTextStyle',
        labelTextStyle,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<IconThemeData?>>(
        'iconTheme',
        iconTheme,
        defaultValue: null,
      ),
    );
  }
}

/// An inherited widget that defines visual properties for [NavigationDrawer]s and
/// [NavigationDestination]s in this widget's subtree.
///
/// Values specified here are used for [NavigationDrawer] properties that are not
/// given an explicit non-null value.
///
/// See also:
///
///  * [ThemeData.navigationDrawerTheme], which describes the
///    [NavigationDrawerThemeData] in the overall theme for the application.
class NavigationDrawerTheme extends InheritedTheme {
  /// Creates a navigation rail theme that controls the
  /// [NavigationDrawerThemeData] properties for a [NavigationDrawer].
  const NavigationDrawerTheme({super.key, required this.data, required super.child});

  /// Specifies the background color, label text style, icon theme, and label
  /// type values for descendant [NavigationDrawer] widgets.
  final NavigationDrawerThemeData data;

  /// The closest instance of this class that encloses the given context.
  ///
  /// If there is no enclosing [NavigationDrawerTheme] widget, then
  /// [ThemeData.navigationDrawerTheme] is used.
  static NavigationDrawerThemeData of(BuildContext context) {
    final NavigationDrawerTheme? navigationDrawerTheme =
        context.dependOnInheritedWidgetOfExactType<NavigationDrawerTheme>();
    return navigationDrawerTheme?.data ?? Theme.of(context).navigationDrawerTheme;
  }

  @override
  Widget wrap(BuildContext context, Widget child) {
    return NavigationDrawerTheme(data: data, child: child);
  }

  @override
  bool updateShouldNotify(NavigationDrawerTheme oldWidget) => data != oldWidget.data;
}
