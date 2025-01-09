// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'search_anchor.dart';
library;

import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';

import 'material_state.dart';
import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// Defines default property values for descendant [SearchBar] widgets.
///
/// Descendant widgets obtain the current [SearchBarThemeData] object using
/// `SearchBarTheme.of(context)`. Instances of [SearchBarThemeData] can be customized
/// with [SearchBarThemeData.copyWith].
///
/// Typically a [SearchBarThemeData] is specified as part of the overall [Theme]
/// with [ThemeData.searchBarTheme].
///
/// All [SearchBarThemeData] properties are `null` by default. When null, the
/// [SearchBar] will use the values from [ThemeData] if they exist, otherwise it
/// will provide its own defaults based on the overall [Theme]'s colorScheme.
/// See the individual [SearchBar] properties for details.
///
/// See also:
///
///  * [ThemeData], which describes the overall theme information for the
///    application.
@immutable
class SearchBarThemeData with Diagnosticable {
  /// Creates a theme that can be used for [ThemeData.searchBarTheme].
  const SearchBarThemeData({
    this.elevation,
    this.backgroundColor,
    this.shadowColor,
    this.surfaceTintColor,
    this.overlayColor,
    this.side,
    this.shape,
    this.padding,
    this.textStyle,
    this.hintStyle,
    this.constraints,
    this.textCapitalization,
  });

  /// Overrides the default value of the [SearchBar.elevation].
  final MaterialStateProperty<double?>? elevation;

  /// Overrides the default value of the [SearchBar.backgroundColor].
  final MaterialStateProperty<Color?>? backgroundColor;

  /// Overrides the default value of the [SearchBar.shadowColor].
  final MaterialStateProperty<Color?>? shadowColor;

  /// Overrides the default value of the [SearchBar.surfaceTintColor].
  final MaterialStateProperty<Color?>? surfaceTintColor;

  /// Overrides the default value of the [SearchBar.overlayColor].
  final MaterialStateProperty<Color?>? overlayColor;

  /// Overrides the default value of the [SearchBar.side].
  final MaterialStateProperty<BorderSide?>? side;

  /// Overrides the default value of the [SearchBar.shape].
  final MaterialStateProperty<OutlinedBorder?>? shape;

  /// Overrides the default value for [SearchBar.padding].
  final MaterialStateProperty<EdgeInsetsGeometry?>? padding;

  /// Overrides the default value for [SearchBar.textStyle].
  final MaterialStateProperty<TextStyle?>? textStyle;

  /// Overrides the default value for [SearchBar.hintStyle].
  final MaterialStateProperty<TextStyle?>? hintStyle;

  /// Overrides the value of size constraints for [SearchBar].
  final BoxConstraints? constraints;

  /// Overrides the value of [SearchBar.textCapitalization].
  final TextCapitalization? textCapitalization;

  /// Creates a copy of this object but with the given fields replaced with the
  /// new values.
  SearchBarThemeData copyWith({
    MaterialStateProperty<double?>? elevation,
    MaterialStateProperty<Color?>? backgroundColor,
    MaterialStateProperty<Color?>? shadowColor,
    MaterialStateProperty<Color?>? surfaceTintColor,
    MaterialStateProperty<Color?>? overlayColor,
    MaterialStateProperty<BorderSide?>? side,
    MaterialStateProperty<OutlinedBorder?>? shape,
    MaterialStateProperty<EdgeInsetsGeometry?>? padding,
    MaterialStateProperty<TextStyle?>? textStyle,
    MaterialStateProperty<TextStyle?>? hintStyle,
    BoxConstraints? constraints,
    TextCapitalization? textCapitalization,
  }) {
    return SearchBarThemeData(
      elevation: elevation ?? this.elevation,
      backgroundColor: backgroundColor ?? this.backgroundColor,
      shadowColor: shadowColor ?? this.shadowColor,
      surfaceTintColor: surfaceTintColor ?? this.surfaceTintColor,
      overlayColor: overlayColor ?? this.overlayColor,
      side: side ?? this.side,
      shape: shape ?? this.shape,
      padding: padding ?? this.padding,
      textStyle: textStyle ?? this.textStyle,
      hintStyle: hintStyle ?? this.hintStyle,
      constraints: constraints ?? this.constraints,
      textCapitalization: textCapitalization ?? this.textCapitalization,
    );
  }

  /// Linearly interpolate between two [SearchBarThemeData]s.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static SearchBarThemeData? lerp(SearchBarThemeData? a, SearchBarThemeData? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    return SearchBarThemeData(
      elevation: MaterialStateProperty.lerp<double?>(a?.elevation, b?.elevation, t, lerpDouble),
      backgroundColor: MaterialStateProperty.lerp<Color?>(
        a?.backgroundColor,
        b?.backgroundColor,
        t,
        Color.lerp,
      ),
      shadowColor: MaterialStateProperty.lerp<Color?>(
        a?.shadowColor,
        b?.shadowColor,
        t,
        Color.lerp,
      ),
      surfaceTintColor: MaterialStateProperty.lerp<Color?>(
        a?.surfaceTintColor,
        b?.surfaceTintColor,
        t,
        Color.lerp,
      ),
      overlayColor: MaterialStateProperty.lerp<Color?>(
        a?.overlayColor,
        b?.overlayColor,
        t,
        Color.lerp,
      ),
      side: _lerpSides(a?.side, b?.side, t),
      shape: MaterialStateProperty.lerp<OutlinedBorder?>(
        a?.shape,
        b?.shape,
        t,
        OutlinedBorder.lerp,
      ),
      padding: MaterialStateProperty.lerp<EdgeInsetsGeometry?>(
        a?.padding,
        b?.padding,
        t,
        EdgeInsetsGeometry.lerp,
      ),
      textStyle: MaterialStateProperty.lerp<TextStyle?>(
        a?.textStyle,
        b?.textStyle,
        t,
        TextStyle.lerp,
      ),
      hintStyle: MaterialStateProperty.lerp<TextStyle?>(
        a?.hintStyle,
        b?.hintStyle,
        t,
        TextStyle.lerp,
      ),
      constraints: BoxConstraints.lerp(a?.constraints, b?.constraints, t),
      textCapitalization: t < 0.5 ? a?.textCapitalization : b?.textCapitalization,
    );
  }

  @override
  int get hashCode => Object.hash(
    elevation,
    backgroundColor,
    shadowColor,
    surfaceTintColor,
    overlayColor,
    side,
    shape,
    padding,
    textStyle,
    hintStyle,
    constraints,
    textCapitalization,
  );

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is SearchBarThemeData &&
        other.elevation == elevation &&
        other.backgroundColor == backgroundColor &&
        other.shadowColor == shadowColor &&
        other.surfaceTintColor == surfaceTintColor &&
        other.overlayColor == overlayColor &&
        other.side == side &&
        other.shape == shape &&
        other.padding == padding &&
        other.textStyle == textStyle &&
        other.hintStyle == hintStyle &&
        other.constraints == constraints &&
        other.textCapitalization == textCapitalization;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<double?>>(
        'elevation',
        elevation,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<Color?>>(
        'backgroundColor',
        backgroundColor,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<Color?>>(
        'shadowColor',
        shadowColor,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<Color?>>(
        'surfaceTintColor',
        surfaceTintColor,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<Color?>>(
        'overlayColor',
        overlayColor,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<BorderSide?>>('side', side, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<OutlinedBorder?>>(
        'shape',
        shape,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<EdgeInsetsGeometry?>>(
        'padding',
        padding,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<TextStyle?>>(
        'textStyle',
        textStyle,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<TextStyle?>>(
        'hintStyle',
        hintStyle,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<BoxConstraints>('constraints', constraints, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<TextCapitalization>(
        'textCapitalization',
        textCapitalization,
        defaultValue: null,
      ),
    );
  }

  // Special case because BorderSide.lerp() doesn't support null arguments
  static MaterialStateProperty<BorderSide?>? _lerpSides(
    MaterialStateProperty<BorderSide?>? a,
    MaterialStateProperty<BorderSide?>? b,
    double t,
  ) {
    if (identical(a, b)) {
      return a;
    }
    return MaterialStateBorderSide.lerp(a, b, t);
  }
}

/// Applies a search bar theme to descendant [SearchBar] widgets.
///
/// Descendant widgets obtain the current theme's [SearchBarTheme] object using
/// [SearchBarTheme.of]. When a widget uses [SearchBarTheme.of], it is automatically
/// rebuilt if the theme later changes.
///
/// A search bar theme can be specified as part of the overall Material theme using
/// [ThemeData.searchBarTheme].
///
/// See also:
///
///  * [SearchBarThemeData], which describes the actual configuration of a search bar
///    theme.
class SearchBarTheme extends InheritedWidget {
  /// Constructs a search bar theme that configures all descendant [SearchBar] widgets.
  const SearchBarTheme({super.key, required this.data, required super.child});

  /// The properties used for all descendant [SearchBar] widgets.
  final SearchBarThemeData data;

  /// Returns the configuration [data] from the closest [SearchBarTheme] ancestor.
  /// If there is no ancestor, it returns [ThemeData.searchBarTheme].
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// SearchBarThemeData theme = SearchBarTheme.of(context);
  /// ```
  static SearchBarThemeData of(BuildContext context) {
    final SearchBarTheme? searchBarTheme =
        context.dependOnInheritedWidgetOfExactType<SearchBarTheme>();
    return searchBarTheme?.data ?? Theme.of(context).searchBarTheme;
  }

  @override
  bool updateShouldNotify(SearchBarTheme oldWidget) => data != oldWidget.data;
}
