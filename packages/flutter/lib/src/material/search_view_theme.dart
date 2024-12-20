// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'search_anchor.dart';
/// @docImport 'search_bar_theme.dart';
library;

import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// Defines the configuration of the search views created by the [SearchAnchor]
/// widget.
///
/// Descendant widgets obtain the current [SearchViewThemeData] object using
/// `SearchViewTheme.of(context)`.
///
/// Typically, a [SearchViewThemeData] is specified as part of the overall [Theme]
/// with [ThemeData.searchViewTheme]. Otherwise, [SearchViewTheme] can be used
/// to configure its own widget subtree.
///
/// All [SearchViewThemeData] properties are `null` by default. If any of these
/// properties are null, the search view will provide its own defaults.
///
/// See also:
///
/// * [ThemeData], which describes the overall theme for the application.
/// * [SearchBarThemeData], which describes the theme for the search bar itself in a
///   [SearchBar] widget.
/// * [SearchAnchor], which is used to open a search view route.
@immutable
class SearchViewThemeData with Diagnosticable {
  /// Creates a theme that can be used for [ThemeData.searchViewTheme].
  const SearchViewThemeData({
    this.backgroundColor,
    this.elevation,
    this.surfaceTintColor,
    this.constraints,
    this.padding,
    this.barPadding,
    this.shrinkWrap,
    this.side,
    this.shape,
    this.headerHeight,
    this.headerTextStyle,
    this.headerHintStyle,
    this.dividerColor,
  });

  /// Overrides the default value of the [SearchAnchor.viewBackgroundColor].
  final Color? backgroundColor;

  /// Overrides the default value of the [SearchAnchor.viewElevation].
  final double? elevation;

  /// Overrides the default value of the [SearchAnchor.viewSurfaceTintColor].
  final Color? surfaceTintColor;

  /// Overrides the default value of the [SearchAnchor.viewSide].
  final BorderSide? side;

  /// Overrides the default value of the [SearchAnchor.viewShape].
  final OutlinedBorder? shape;

  /// Overrides the default value of the [SearchAnchor.headerHeight].
  final double? headerHeight;

  /// Overrides the default value for [SearchAnchor.headerTextStyle].
  final TextStyle? headerTextStyle;

  /// Overrides the default value for [SearchAnchor.headerHintStyle].
  final TextStyle? headerHintStyle;

  /// Overrides the value of size constraints for [SearchAnchor.viewConstraints].
  final BoxConstraints? constraints;

  /// Overrides the value of the padding for [SearchAnchor.viewPadding].
  final EdgeInsetsGeometry? padding;

  /// Overrides the value of the padding for [SearchAnchor.viewBarPadding]
  final EdgeInsetsGeometry? barPadding;

  /// Overrides the value of the shrink wrap for [SearchAnchor.shrinkWrap].
  final bool? shrinkWrap;

  /// Overrides the value of the divider color for [SearchAnchor.dividerColor].
  final Color? dividerColor;

  /// Creates a copy of this object but with the given fields replaced with the
  /// new values.
  SearchViewThemeData copyWith({
    Color? backgroundColor,
    double? elevation,
    Color? surfaceTintColor,
    BorderSide? side,
    OutlinedBorder? shape,
    double? headerHeight,
    TextStyle? headerTextStyle,
    TextStyle? headerHintStyle,
    BoxConstraints? constraints,
    EdgeInsetsGeometry? padding,
    EdgeInsetsGeometry? barPadding,
    bool? shrinkWrap,
    Color? dividerColor,
  }) {
    return SearchViewThemeData(
      backgroundColor: backgroundColor ?? this.backgroundColor,
      elevation: elevation ?? this.elevation,
      surfaceTintColor: surfaceTintColor ?? this.surfaceTintColor,
      side: side ?? this.side,
      shape: shape ?? this.shape,
      headerHeight: headerHeight ?? this.headerHeight,
      headerTextStyle: headerTextStyle ?? this.headerTextStyle,
      headerHintStyle: headerHintStyle ?? this.headerHintStyle,
      constraints: constraints ?? this.constraints,
      padding: padding ?? this.padding,
      barPadding: barPadding ?? this.barPadding,
      shrinkWrap: shrinkWrap ?? this.shrinkWrap,
      dividerColor: dividerColor ?? this.dividerColor,
    );
  }

  /// Linearly interpolate between two [SearchViewThemeData]s.
  static SearchViewThemeData? lerp(SearchViewThemeData? a, SearchViewThemeData? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    return SearchViewThemeData(
      backgroundColor: Color.lerp(a?.backgroundColor, b?.backgroundColor, t),
      elevation: lerpDouble(a?.elevation, b?.elevation, t),
      surfaceTintColor: Color.lerp(a?.surfaceTintColor, b?.surfaceTintColor, t),
      side: _lerpSides(a?.side, b?.side, t),
      shape: OutlinedBorder.lerp(a?.shape, b?.shape, t),
      headerHeight: lerpDouble(a?.headerHeight, b?.headerHeight, t),
      headerTextStyle: TextStyle.lerp(a?.headerTextStyle, b?.headerTextStyle, t),
      headerHintStyle: TextStyle.lerp(a?.headerTextStyle, b?.headerTextStyle, t),
      constraints: BoxConstraints.lerp(a?.constraints, b?.constraints, t),
      padding: EdgeInsetsGeometry.lerp(a?.padding, b?.padding, t),
      barPadding: EdgeInsetsGeometry.lerp(a?.barPadding, b?.barPadding, t),
      shrinkWrap: t < 0.5 ? a?.shrinkWrap : b?.shrinkWrap,
      dividerColor: Color.lerp(a?.dividerColor, b?.dividerColor, t),
    );
  }

  @override
  int get hashCode => Object.hash(
    backgroundColor,
    elevation,
    surfaceTintColor,
    side,
    shape,
    headerHeight,
    headerTextStyle,
    headerHintStyle,
    constraints,
    padding,
    barPadding,
    shrinkWrap,
    dividerColor,
  );

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is SearchViewThemeData &&
        other.backgroundColor == backgroundColor &&
        other.elevation == elevation &&
        other.surfaceTintColor == surfaceTintColor &&
        other.side == side &&
        other.shape == shape &&
        other.headerHeight == headerHeight &&
        other.headerTextStyle == headerTextStyle &&
        other.headerHintStyle == headerHintStyle &&
        other.constraints == constraints &&
        other.padding == padding &&
        other.barPadding == barPadding &&
        other.shrinkWrap == shrinkWrap &&
        other.dividerColor == dividerColor;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      DiagnosticsProperty<Color?>('backgroundColor', backgroundColor, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<double?>('elevation', elevation, defaultValue: null));
    properties.add(
      DiagnosticsProperty<Color?>('surfaceTintColor', surfaceTintColor, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<BorderSide?>('side', side, defaultValue: null));
    properties.add(DiagnosticsProperty<OutlinedBorder?>('shape', shape, defaultValue: null));
    properties.add(DiagnosticsProperty<double?>('headerHeight', headerHeight, defaultValue: null));
    properties.add(
      DiagnosticsProperty<TextStyle?>('headerTextStyle', headerTextStyle, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<TextStyle?>('headerHintStyle', headerHintStyle, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<BoxConstraints>('constraints', constraints, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<EdgeInsetsGeometry?>('padding', padding, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<EdgeInsetsGeometry?>('barPadding', barPadding, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<bool?>('shrinkWrap', shrinkWrap, defaultValue: null));
    properties.add(DiagnosticsProperty<Color?>('dividerColor', dividerColor, defaultValue: null));
  }

  // Special case because BorderSide.lerp() doesn't support null arguments
  static BorderSide? _lerpSides(BorderSide? a, BorderSide? b, double t) {
    if (a == null || b == null) {
      return null;
    }
    if (identical(a, b)) {
      return a;
    }
    return BorderSide.lerp(a, b, t);
  }
}

/// An inherited widget that defines the configuration in this widget's
/// descendants for search view created by the [SearchAnchor] widget.
///
/// A search view theme can be specified as part of the overall Material theme using
/// [ThemeData.searchViewTheme].
///
/// See also:
///
///  * [SearchViewThemeData], which describes the actual configuration of a search view
///    theme.
class SearchViewTheme extends InheritedTheme {
  /// Creates a const theme that controls the configurations for the search view
  /// created by the [SearchAnchor] widget.
  const SearchViewTheme({super.key, required this.data, required super.child});

  /// The properties used for all descendant [SearchAnchor] widgets.
  final SearchViewThemeData data;

  /// Returns the configuration [data] from the closest [SearchViewTheme] ancestor.
  /// If there is no ancestor, it returns [ThemeData.searchViewTheme].
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// SearchViewThemeData theme = SearchViewTheme.of(context);
  /// ```
  static SearchViewThemeData of(BuildContext context) {
    final SearchViewTheme? searchViewTheme =
        context.dependOnInheritedWidgetOfExactType<SearchViewTheme>();
    return searchViewTheme?.data ?? Theme.of(context).searchViewTheme;
  }

  @override
  Widget wrap(BuildContext context, Widget child) {
    return SearchViewTheme(data: data, child: child);
  }

  @override
  bool updateShouldNotify(SearchViewTheme oldWidget) => data != oldWidget.data;
}
