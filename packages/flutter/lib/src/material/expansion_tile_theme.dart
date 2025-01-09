// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'expansion_tile.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// Used with [ExpansionTileTheme] to define default property values for
/// descendant [ExpansionTile] widgets.
///
/// Descendant widgets obtain the current [ExpansionTileThemeData] object
/// using `ExpansionTileTheme.of(context)`. Instances of
/// [ExpansionTileThemeData] can be customized with
/// [ExpansionTileThemeData.copyWith].
///
/// A [ExpansionTileThemeData] is often specified as part of the
/// overall [Theme] with [ThemeData.expansionTileTheme].
///
/// All [ExpansionTileThemeData] properties are `null` by default.
/// When a theme property is null, the [ExpansionTile]  will provide its own
/// default based on the overall [Theme]'s textTheme and
/// colorScheme. See the individual [ExpansionTile] properties for details.
///
/// See also:
///
///  * [ThemeData], which describes the overall theme information for the
///    application.
///  * [ExpansionTileTheme] which overrides the default [ExpansionTileTheme]
///    of its [ExpansionTile] descendants.
///  * [ThemeData.textTheme], text with a color that contrasts with the card
///    and canvas colors.
///  * [ThemeData.colorScheme], the thirteen colors that most Material widget
///    default colors are based on.
@immutable
class ExpansionTileThemeData with Diagnosticable {
  /// Creates a [ExpansionTileThemeData].
  const ExpansionTileThemeData({
    this.backgroundColor,
    this.collapsedBackgroundColor,
    this.tilePadding,
    this.expandedAlignment,
    this.childrenPadding,
    this.iconColor,
    this.collapsedIconColor,
    this.textColor,
    this.collapsedTextColor,
    this.shape,
    this.collapsedShape,
    this.clipBehavior,
    this.expansionAnimationStyle,
  });

  /// Overrides the default value of [ExpansionTile.backgroundColor].
  final Color? backgroundColor;

  /// Overrides the default value of [ExpansionTile.collapsedBackgroundColor].
  final Color? collapsedBackgroundColor;

  /// Overrides the default value of [ExpansionTile.tilePadding].
  final EdgeInsetsGeometry? tilePadding;

  /// Overrides the default value of [ExpansionTile.expandedAlignment].
  final AlignmentGeometry? expandedAlignment;

  /// Overrides the default value of [ExpansionTile.childrenPadding].
  final EdgeInsetsGeometry? childrenPadding;

  /// Overrides the default value of [ExpansionTile.iconColor].
  final Color? iconColor;

  /// Overrides the default value of [ExpansionTile.collapsedIconColor].
  final Color? collapsedIconColor;

  /// Overrides the default value of [ExpansionTile.textColor].
  final Color? textColor;

  /// Overrides the default value of [ExpansionTile.collapsedTextColor].
  final Color? collapsedTextColor;

  /// Overrides the default value of [ExpansionTile.shape].
  final ShapeBorder? shape;

  /// Overrides the default value of [ExpansionTile.collapsedShape].
  final ShapeBorder? collapsedShape;

  /// Overrides the default value of [ExpansionTile.clipBehavior].
  final Clip? clipBehavior;

  /// Overrides the default value of [ExpansionTile.expansionAnimationStyle].
  final AnimationStyle? expansionAnimationStyle;

  /// Creates a copy of this object with the given fields replaced with the
  /// new values.
  ExpansionTileThemeData copyWith({
    Color? backgroundColor,
    Color? collapsedBackgroundColor,
    EdgeInsetsGeometry? tilePadding,
    AlignmentGeometry? expandedAlignment,
    EdgeInsetsGeometry? childrenPadding,
    Color? iconColor,
    Color? collapsedIconColor,
    Color? textColor,
    Color? collapsedTextColor,
    ShapeBorder? shape,
    ShapeBorder? collapsedShape,
    Clip? clipBehavior,
    AnimationStyle? expansionAnimationStyle,
  }) {
    return ExpansionTileThemeData(
      backgroundColor: backgroundColor ?? this.backgroundColor,
      collapsedBackgroundColor: collapsedBackgroundColor ?? this.collapsedBackgroundColor,
      tilePadding: tilePadding ?? this.tilePadding,
      expandedAlignment: expandedAlignment ?? this.expandedAlignment,
      childrenPadding: childrenPadding ?? this.childrenPadding,
      iconColor: iconColor ?? this.iconColor,
      collapsedIconColor: collapsedIconColor ?? this.collapsedIconColor,
      textColor: textColor ?? this.textColor,
      collapsedTextColor: collapsedTextColor ?? this.collapsedTextColor,
      shape: shape ?? this.shape,
      collapsedShape: collapsedShape ?? this.collapsedShape,
      clipBehavior: clipBehavior ?? this.clipBehavior,
      expansionAnimationStyle: expansionAnimationStyle ?? this.expansionAnimationStyle,
    );
  }

  /// Linearly interpolate between ExpansionTileThemeData objects.
  static ExpansionTileThemeData? lerp(
    ExpansionTileThemeData? a,
    ExpansionTileThemeData? b,
    double t,
  ) {
    if (identical(a, b)) {
      return a;
    }
    return ExpansionTileThemeData(
      backgroundColor: Color.lerp(a?.backgroundColor, b?.backgroundColor, t),
      collapsedBackgroundColor: Color.lerp(
        a?.collapsedBackgroundColor,
        b?.collapsedBackgroundColor,
        t,
      ),
      tilePadding: EdgeInsetsGeometry.lerp(a?.tilePadding, b?.tilePadding, t),
      expandedAlignment: AlignmentGeometry.lerp(a?.expandedAlignment, b?.expandedAlignment, t),
      childrenPadding: EdgeInsetsGeometry.lerp(a?.childrenPadding, b?.childrenPadding, t),
      iconColor: Color.lerp(a?.iconColor, b?.iconColor, t),
      collapsedIconColor: Color.lerp(a?.collapsedIconColor, b?.collapsedIconColor, t),
      textColor: Color.lerp(a?.textColor, b?.textColor, t),
      collapsedTextColor: Color.lerp(a?.collapsedTextColor, b?.collapsedTextColor, t),
      shape: ShapeBorder.lerp(a?.shape, b?.shape, t),
      collapsedShape: ShapeBorder.lerp(a?.collapsedShape, b?.collapsedShape, t),
      clipBehavior: t < 0.5 ? a?.clipBehavior : b?.clipBehavior,
      expansionAnimationStyle: t < 0.5 ? a?.expansionAnimationStyle : b?.expansionAnimationStyle,
    );
  }

  @override
  int get hashCode {
    return Object.hash(
      backgroundColor,
      collapsedBackgroundColor,
      tilePadding,
      expandedAlignment,
      childrenPadding,
      iconColor,
      collapsedIconColor,
      textColor,
      collapsedTextColor,
      shape,
      collapsedShape,
      clipBehavior,
      expansionAnimationStyle,
    );
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is ExpansionTileThemeData &&
        other.backgroundColor == backgroundColor &&
        other.collapsedBackgroundColor == collapsedBackgroundColor &&
        other.tilePadding == tilePadding &&
        other.expandedAlignment == expandedAlignment &&
        other.childrenPadding == childrenPadding &&
        other.iconColor == iconColor &&
        other.collapsedIconColor == collapsedIconColor &&
        other.textColor == textColor &&
        other.collapsedTextColor == collapsedTextColor &&
        other.shape == shape &&
        other.collapsedShape == collapsedShape &&
        other.clipBehavior == clipBehavior &&
        other.expansionAnimationStyle == expansionAnimationStyle;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(ColorProperty('backgroundColor', backgroundColor, defaultValue: null));
    properties.add(
      ColorProperty('collapsedBackgroundColor', collapsedBackgroundColor, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<EdgeInsetsGeometry>('tilePadding', tilePadding, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<AlignmentGeometry>(
        'expandedAlignment',
        expandedAlignment,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<EdgeInsetsGeometry>(
        'childrenPadding',
        childrenPadding,
        defaultValue: null,
      ),
    );
    properties.add(ColorProperty('iconColor', iconColor, defaultValue: null));
    properties.add(ColorProperty('collapsedIconColor', collapsedIconColor, defaultValue: null));
    properties.add(ColorProperty('textColor', textColor, defaultValue: null));
    properties.add(ColorProperty('collapsedTextColor', collapsedTextColor, defaultValue: null));
    properties.add(DiagnosticsProperty<ShapeBorder>('shape', shape, defaultValue: null));
    properties.add(
      DiagnosticsProperty<ShapeBorder>('collapsedShape', collapsedShape, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<Clip>('clipBehavior', clipBehavior, defaultValue: null));
    properties.add(
      DiagnosticsProperty<AnimationStyle>(
        'expansionAnimationStyle',
        expansionAnimationStyle,
        defaultValue: null,
      ),
    );
  }
}

/// Overrides the default [ExpansionTileTheme] of its [ExpansionTile] descendants.
///
/// See also:
///
///  * [ExpansionTileThemeData], which is used to configure this theme.
///  * [ThemeData.expansionTileTheme], which can be used to override the default
///    [ExpansionTileTheme] for [ExpansionTile]s below the overall [Theme].
class ExpansionTileTheme extends InheritedTheme {
  /// Applies the given theme [data] to [child].
  const ExpansionTileTheme({super.key, required this.data, required super.child});

  /// Specifies color, alignment, and text style values for
  /// descendant [ExpansionTile] widgets.
  final ExpansionTileThemeData data;

  /// The closest instance of this class that encloses the given context.
  ///
  /// If there is no enclosing [ExpansionTileTheme] widget, then
  /// [ThemeData.expansionTileTheme] is used.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// ExpansionTileThemeData theme = ExpansionTileTheme.of(context);
  /// ```
  static ExpansionTileThemeData of(BuildContext context) {
    final ExpansionTileTheme? inheritedTheme =
        context.dependOnInheritedWidgetOfExactType<ExpansionTileTheme>();
    return inheritedTheme?.data ?? Theme.of(context).expansionTileTheme;
  }

  @override
  Widget wrap(BuildContext context, Widget child) {
    return ExpansionTileTheme(data: data, child: child);
  }

  @override
  bool updateShouldNotify(ExpansionTileTheme oldWidget) => data != oldWidget.data;
}
