// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'navigation_bar.dart';
library;

import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'navigation_rail.dart';
import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// Defines default property values for descendant [NavigationRail]
/// widgets.
///
/// Descendant widgets obtain the current [NavigationRailThemeData] object
/// using `NavigationRailTheme.of(context)`. Instances of
/// [NavigationRailThemeData] can be customized with
/// [NavigationRailThemeData.copyWith].
///
/// Typically a [NavigationRailThemeData] is specified as part of the
/// overall [Theme] with [ThemeData.navigationRailTheme].
///
/// All [NavigationRailThemeData] properties are `null` by default.
/// When null, the [NavigationRail] will use the values from [ThemeData]
/// if they exist, otherwise it will provide its own defaults based on the
/// overall [Theme]'s textTheme and colorScheme. See the individual
/// [NavigationRail] properties for details.
///
/// See also:
///
///  * [ThemeData], which describes the overall theme information for the
///    application.
@immutable
class NavigationRailThemeData with Diagnosticable {
  /// Creates a theme that can be used for [ThemeData.navigationRailTheme].
  const NavigationRailThemeData({
    this.backgroundColor,
    this.elevation,
    this.unselectedLabelTextStyle,
    this.selectedLabelTextStyle,
    this.unselectedIconTheme,
    this.selectedIconTheme,
    this.groupAlignment,
    this.labelType,
    this.useIndicator,
    this.indicatorColor,
    this.indicatorShape,
    this.minWidth,
    this.minExtendedWidth,
  });

  /// Color to be used for the [NavigationRail]'s background.
  final Color? backgroundColor;

  /// The z-coordinate to be used for the [NavigationRail]'s elevation.
  final double? elevation;

  /// The style to merge with the default text style for
  /// [NavigationRailDestination] labels, when the destination is not selected.
  final TextStyle? unselectedLabelTextStyle;

  /// The style to merge with the default text style for
  /// [NavigationRailDestination] labels, when the destination is selected.
  final TextStyle? selectedLabelTextStyle;

  /// The theme to merge with the default icon theme for
  /// [NavigationRailDestination] icons, when the destination is not selected.
  final IconThemeData? unselectedIconTheme;

  /// The theme to merge with the default icon theme for
  /// [NavigationRailDestination] icons, when the destination is selected.
  final IconThemeData? selectedIconTheme;

  /// The alignment for the [NavigationRailDestination]s as they are positioned
  /// within the [NavigationRail].
  final double? groupAlignment;

  /// The type that defines the layout and behavior of the labels in the
  /// [NavigationRail].
  final NavigationRailLabelType? labelType;

  /// Whether or not the selected [NavigationRailDestination] should include a
  /// [NavigationIndicator].
  final bool? useIndicator;

  /// Overrides the default value of [NavigationRail]'s selection indicator color,
  /// when [useIndicator] is true.
  final Color? indicatorColor;

  /// Overrides the default shape of the [NavigationRail]'s selection indicator.
  final ShapeBorder? indicatorShape;

  /// Overrides the default value of [NavigationRail]'s minimum width when it
  /// is not extended.
  final double? minWidth;

  /// Overrides the default value of [NavigationRail]'s minimum width when it
  /// is extended.
  final double? minExtendedWidth;

  /// Creates a copy of this object with the given fields replaced with the
  /// new values.
  NavigationRailThemeData copyWith({
    Color? backgroundColor,
    double? elevation,
    TextStyle? unselectedLabelTextStyle,
    TextStyle? selectedLabelTextStyle,
    IconThemeData? unselectedIconTheme,
    IconThemeData? selectedIconTheme,
    double? groupAlignment,
    NavigationRailLabelType? labelType,
    bool? useIndicator,
    Color? indicatorColor,
    ShapeBorder? indicatorShape,
    double? minWidth,
    double? minExtendedWidth,
  }) {
    return NavigationRailThemeData(
      backgroundColor: backgroundColor ?? this.backgroundColor,
      elevation: elevation ?? this.elevation,
      unselectedLabelTextStyle: unselectedLabelTextStyle ?? this.unselectedLabelTextStyle,
      selectedLabelTextStyle: selectedLabelTextStyle ?? this.selectedLabelTextStyle,
      unselectedIconTheme: unselectedIconTheme ?? this.unselectedIconTheme,
      selectedIconTheme: selectedIconTheme ?? this.selectedIconTheme,
      groupAlignment: groupAlignment ?? this.groupAlignment,
      labelType: labelType ?? this.labelType,
      useIndicator: useIndicator ?? this.useIndicator,
      indicatorColor: indicatorColor ?? this.indicatorColor,
      indicatorShape: indicatorShape ?? this.indicatorShape,
      minWidth: minWidth ?? this.minWidth,
      minExtendedWidth: minExtendedWidth ?? this.minExtendedWidth,
    );
  }

  /// Linearly interpolate between two navigation rail themes.
  ///
  /// If both arguments are null then null is returned.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static NavigationRailThemeData? lerp(
    NavigationRailThemeData? a,
    NavigationRailThemeData? b,
    double t,
  ) {
    if (identical(a, b)) {
      return a;
    }
    return NavigationRailThemeData(
      backgroundColor: Color.lerp(a?.backgroundColor, b?.backgroundColor, t),
      elevation: lerpDouble(a?.elevation, b?.elevation, t),
      unselectedLabelTextStyle: TextStyle.lerp(
        a?.unselectedLabelTextStyle,
        b?.unselectedLabelTextStyle,
        t,
      ),
      selectedLabelTextStyle: TextStyle.lerp(
        a?.selectedLabelTextStyle,
        b?.selectedLabelTextStyle,
        t,
      ),
      unselectedIconTheme:
          a?.unselectedIconTheme == null && b?.unselectedIconTheme == null
              ? null
              : IconThemeData.lerp(a?.unselectedIconTheme, b?.unselectedIconTheme, t),
      selectedIconTheme:
          a?.selectedIconTheme == null && b?.selectedIconTheme == null
              ? null
              : IconThemeData.lerp(a?.selectedIconTheme, b?.selectedIconTheme, t),
      groupAlignment: lerpDouble(a?.groupAlignment, b?.groupAlignment, t),
      labelType: t < 0.5 ? a?.labelType : b?.labelType,
      useIndicator: t < 0.5 ? a?.useIndicator : b?.useIndicator,
      indicatorColor: Color.lerp(a?.indicatorColor, b?.indicatorColor, t),
      indicatorShape: ShapeBorder.lerp(a?.indicatorShape, b?.indicatorShape, t),
      minWidth: lerpDouble(a?.minWidth, b?.minWidth, t),
      minExtendedWidth: lerpDouble(a?.minExtendedWidth, b?.minExtendedWidth, t),
    );
  }

  @override
  int get hashCode => Object.hash(
    backgroundColor,
    elevation,
    unselectedLabelTextStyle,
    selectedLabelTextStyle,
    unselectedIconTheme,
    selectedIconTheme,
    groupAlignment,
    labelType,
    useIndicator,
    indicatorColor,
    indicatorShape,
    minWidth,
    minExtendedWidth,
  );

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is NavigationRailThemeData &&
        other.backgroundColor == backgroundColor &&
        other.elevation == elevation &&
        other.unselectedLabelTextStyle == unselectedLabelTextStyle &&
        other.selectedLabelTextStyle == selectedLabelTextStyle &&
        other.unselectedIconTheme == unselectedIconTheme &&
        other.selectedIconTheme == selectedIconTheme &&
        other.groupAlignment == groupAlignment &&
        other.labelType == labelType &&
        other.useIndicator == useIndicator &&
        other.indicatorColor == indicatorColor &&
        other.indicatorShape == indicatorShape &&
        other.minWidth == minWidth &&
        other.minExtendedWidth == minExtendedWidth;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    const NavigationRailThemeData defaultData = NavigationRailThemeData();

    properties.add(
      ColorProperty('backgroundColor', backgroundColor, defaultValue: defaultData.backgroundColor),
    );
    properties.add(DoubleProperty('elevation', elevation, defaultValue: defaultData.elevation));
    properties.add(
      DiagnosticsProperty<TextStyle>(
        'unselectedLabelTextStyle',
        unselectedLabelTextStyle,
        defaultValue: defaultData.unselectedLabelTextStyle,
      ),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>(
        'selectedLabelTextStyle',
        selectedLabelTextStyle,
        defaultValue: defaultData.selectedLabelTextStyle,
      ),
    );
    properties.add(
      DiagnosticsProperty<IconThemeData>(
        'unselectedIconTheme',
        unselectedIconTheme,
        defaultValue: defaultData.unselectedIconTheme,
      ),
    );
    properties.add(
      DiagnosticsProperty<IconThemeData>(
        'selectedIconTheme',
        selectedIconTheme,
        defaultValue: defaultData.selectedIconTheme,
      ),
    );
    properties.add(
      DoubleProperty('groupAlignment', groupAlignment, defaultValue: defaultData.groupAlignment),
    );
    properties.add(
      DiagnosticsProperty<NavigationRailLabelType>(
        'labelType',
        labelType,
        defaultValue: defaultData.labelType,
      ),
    );
    properties.add(
      DiagnosticsProperty<bool>(
        'useIndicator',
        useIndicator,
        defaultValue: defaultData.useIndicator,
      ),
    );
    properties.add(
      ColorProperty('indicatorColor', indicatorColor, defaultValue: defaultData.indicatorColor),
    );
    properties.add(
      DiagnosticsProperty<ShapeBorder>('indicatorShape', indicatorShape, defaultValue: null),
    );
    properties.add(DoubleProperty('minWidth', minWidth, defaultValue: defaultData.minWidth));
    properties.add(
      DoubleProperty(
        'minExtendedWidth',
        minExtendedWidth,
        defaultValue: defaultData.minExtendedWidth,
      ),
    );
  }
}

/// An inherited widget that defines visual properties for [NavigationRail]s and
/// [NavigationRailDestination]s in this widget's subtree.
///
/// Values specified here are used for [NavigationRail] properties that are not
/// given an explicit non-null value.
class NavigationRailTheme extends InheritedTheme {
  /// Creates a navigation rail theme that controls the
  /// [NavigationRailThemeData] properties for a [NavigationRail].
  const NavigationRailTheme({super.key, required this.data, required super.child});

  /// Specifies the background color, elevation, label text style, icon theme,
  /// group alignment, and label type and border values for descendant
  /// [NavigationRail] widgets.
  final NavigationRailThemeData data;

  /// The closest instance of this class that encloses the given context.
  ///
  /// If there is no enclosing [NavigationRailTheme] widget, then
  /// [ThemeData.navigationRailTheme] is used.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// NavigationRailThemeData theme = NavigationRailTheme.of(context);
  /// ```
  static NavigationRailThemeData of(BuildContext context) {
    final NavigationRailTheme? navigationRailTheme =
        context.dependOnInheritedWidgetOfExactType<NavigationRailTheme>();
    return navigationRailTheme?.data ?? Theme.of(context).navigationRailTheme;
  }

  @override
  Widget wrap(BuildContext context, Widget child) {
    return NavigationRailTheme(data: data, child: child);
  }

  @override
  bool updateShouldNotify(NavigationRailTheme oldWidget) => data != oldWidget.data;
}
