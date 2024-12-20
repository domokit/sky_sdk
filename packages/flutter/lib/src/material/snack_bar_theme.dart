// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'bottom_navigation_bar.dart';
/// @docImport 'color_scheme.dart';
/// @docImport 'colors.dart';
/// @docImport 'floating_action_button.dart';
/// @docImport 'navigation_bar.dart';
/// @docImport 'scaffold.dart';
/// @docImport 'snack_bar.dart';
library;

import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'material_state.dart';
import 'theme.dart';

/// Defines where a [SnackBar] should appear within a [Scaffold] and how its
/// location should be adjusted when the scaffold also includes a
/// [FloatingActionButton] or a [BottomNavigationBar].
enum SnackBarBehavior {
  /// Fixes the [SnackBar] at the bottom of the [Scaffold].
  ///
  /// The exception is that the [SnackBar] will be shown above a
  /// [BottomNavigationBar] or a [NavigationBar]. Additionally, the [SnackBar]
  /// will cause other non-fixed widgets inside [Scaffold] to be pushed above
  /// (for example, the [FloatingActionButton]).
  fixed,

  /// This behavior will cause [SnackBar] to be shown above other widgets in the
  /// [Scaffold]. This includes being displayed above a [BottomNavigationBar] or
  /// a [NavigationBar], and a [FloatingActionButton] when its location is on the
  /// bottom. When the floating action button location is on the top, this behavior
  /// will cause the [SnackBar] to be shown above other widgets in the [Scaffold]
  /// except the floating action button.
  ///
  /// See <https://material.io/design/components/snackbars.html> for more details.
  floating,
}

/// Customizes default property values for [SnackBar] widgets.
///
/// Descendant widgets obtain the current [SnackBarThemeData] object using
/// `Theme.of(context).snackBarTheme`. Instances of [SnackBarThemeData] can be
/// customized with [SnackBarThemeData.copyWith].
///
/// Typically a [SnackBarThemeData] is specified as part of the overall [Theme]
/// with [ThemeData.snackBarTheme]. The default for [ThemeData.snackBarTheme]
/// provides all `null` properties.
///
/// All [SnackBarThemeData] properties are `null` by default. When null, the
/// [SnackBar] will provide its own defaults.
///
/// See also:
///
///  * [ThemeData], which describes the overall theme information for the
///    application.
@immutable
class SnackBarThemeData with Diagnosticable {
  /// Creates a theme that can be used for [ThemeData.snackBarTheme].
  ///
  /// The [elevation] must be null or non-negative.
  const SnackBarThemeData({
    this.backgroundColor,
    this.actionTextColor,
    this.disabledActionTextColor,
    this.contentTextStyle,
    this.elevation,
    this.shape,
    this.behavior,
    this.width,
    this.insetPadding,
    this.showCloseIcon,
    this.closeIconColor,
    this.actionOverflowThreshold,
    this.actionBackgroundColor,
    this.disabledActionBackgroundColor,
    this.dismissDirection,
  }) : assert(elevation == null || elevation >= 0.0),
       assert(
         width == null || identical(behavior, SnackBarBehavior.floating),
         'Width can only be set if behaviour is SnackBarBehavior.floating',
       ),
       assert(
         actionOverflowThreshold == null ||
             (actionOverflowThreshold >= 0 && actionOverflowThreshold <= 1),
         'Action overflow threshold must be between 0 and 1 inclusive',
       ),
       assert(
         actionBackgroundColor is! MaterialStateColor || disabledActionBackgroundColor == null,
         'disabledBackgroundColor must not be provided when background color is '
         'a MaterialStateColor',
       );

  /// Overrides the default value for [SnackBar.backgroundColor].
  ///
  /// If null, [SnackBar] defaults to dark grey: `Color(0xFF323232)`.
  final Color? backgroundColor;

  /// Overrides the default value for [SnackBarAction.textColor].
  ///
  /// If null, [SnackBarAction] defaults to [ColorScheme.secondary] of
  /// [ThemeData.colorScheme].
  final Color? actionTextColor;

  /// Overrides the default value for [SnackBarAction.disabledTextColor].
  ///
  /// If null, [SnackBarAction] defaults to [ColorScheme.onSurface] with its
  /// opacity set to 0.30 if the [Theme]'s brightness is [Brightness.dark], 0.38
  /// otherwise.
  final Color? disabledActionTextColor;

  /// Used to configure the [DefaultTextStyle] for the [SnackBar.content] widget.
  ///
  /// If null, [SnackBar] defines its default.
  final TextStyle? contentTextStyle;

  /// Overrides the default value for [SnackBar.elevation].
  ///
  /// If null, [SnackBar] uses a default of 6.0.
  final double? elevation;

  /// Overrides the default value for [SnackBar.shape].
  ///
  /// If null, [SnackBar] provides different defaults depending on the
  /// [SnackBarBehavior]. For [SnackBarBehavior.fixed], no overriding shape is
  /// specified, so the [SnackBar] is rectangular. For
  /// [SnackBarBehavior.floating], it uses a [RoundedRectangleBorder] with a
  /// circular corner radius of 4.0.
  final ShapeBorder? shape;

  /// Overrides the default value for [SnackBar.behavior].
  ///
  /// If null, [SnackBar] will default to [SnackBarBehavior.fixed].
  final SnackBarBehavior? behavior;

  /// Overrides the default value for [SnackBar.width].
  ///
  /// If this property is null, then the snack bar will take up the full device
  /// width less the margin. This value is only used when [behavior] is
  /// [SnackBarBehavior.floating].
  final double? width;

  /// Overrides the default value for [SnackBar.margin].
  ///
  /// This value is only used when [behavior] is [SnackBarBehavior.floating].
  final EdgeInsets? insetPadding;

  /// Overrides the default value for [SnackBar.showCloseIcon].
  ///
  /// Whether to show an optional "Close" icon.
  final bool? showCloseIcon;

  /// Overrides the default value for [SnackBar.closeIconColor].
  ///
  /// This value is only used if [showCloseIcon] is true.
  final Color? closeIconColor;

  /// Overrides the default value for [SnackBar.actionOverflowThreshold].
  ///
  /// Must be a value between 0 and 1, if present.
  final double? actionOverflowThreshold;

  /// Overrides default value for [SnackBarAction.backgroundColor].
  ///
  /// If null, [SnackBarAction] falls back to [Colors.transparent].
  final Color? actionBackgroundColor;

  /// Overrides default value for [SnackBarAction.disabledBackgroundColor].
  ///
  /// If null, [SnackBarAction] falls back to [Colors.transparent].
  final Color? disabledActionBackgroundColor;

  /// Overrides the default value for [SnackBar.dismissDirection].
  ///
  /// If null, [SnackBar] will default to [DismissDirection.down].
  final DismissDirection? dismissDirection;

  /// Creates a copy of this object with the given fields replaced with the
  /// new values.
  SnackBarThemeData copyWith({
    Color? backgroundColor,
    Color? actionTextColor,
    Color? disabledActionTextColor,
    TextStyle? contentTextStyle,
    double? elevation,
    ShapeBorder? shape,
    SnackBarBehavior? behavior,
    double? width,
    EdgeInsets? insetPadding,
    bool? showCloseIcon,
    Color? closeIconColor,
    double? actionOverflowThreshold,
    Color? actionBackgroundColor,
    Color? disabledActionBackgroundColor,
    DismissDirection? dismissDirection,
  }) {
    return SnackBarThemeData(
      backgroundColor: backgroundColor ?? this.backgroundColor,
      actionTextColor: actionTextColor ?? this.actionTextColor,
      disabledActionTextColor: disabledActionTextColor ?? this.disabledActionTextColor,
      contentTextStyle: contentTextStyle ?? this.contentTextStyle,
      elevation: elevation ?? this.elevation,
      shape: shape ?? this.shape,
      behavior: behavior ?? this.behavior,
      width: width ?? this.width,
      insetPadding: insetPadding ?? this.insetPadding,
      showCloseIcon: showCloseIcon ?? this.showCloseIcon,
      closeIconColor: closeIconColor ?? this.closeIconColor,
      actionOverflowThreshold: actionOverflowThreshold ?? this.actionOverflowThreshold,
      actionBackgroundColor: actionBackgroundColor ?? this.actionBackgroundColor,
      disabledActionBackgroundColor:
          disabledActionBackgroundColor ?? this.disabledActionBackgroundColor,
      dismissDirection: dismissDirection ?? this.dismissDirection,
    );
  }

  /// Linearly interpolate between two SnackBar Themes.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static SnackBarThemeData lerp(SnackBarThemeData? a, SnackBarThemeData? b, double t) {
    if (identical(a, b) && a != null) {
      return a;
    }
    return SnackBarThemeData(
      backgroundColor: Color.lerp(a?.backgroundColor, b?.backgroundColor, t),
      actionTextColor: Color.lerp(a?.actionTextColor, b?.actionTextColor, t),
      disabledActionTextColor: Color.lerp(
        a?.disabledActionTextColor,
        b?.disabledActionTextColor,
        t,
      ),
      contentTextStyle: TextStyle.lerp(a?.contentTextStyle, b?.contentTextStyle, t),
      elevation: lerpDouble(a?.elevation, b?.elevation, t),
      shape: ShapeBorder.lerp(a?.shape, b?.shape, t),
      behavior: t < 0.5 ? a?.behavior : b?.behavior,
      width: lerpDouble(a?.width, b?.width, t),
      insetPadding: EdgeInsets.lerp(a?.insetPadding, b?.insetPadding, t),
      closeIconColor: Color.lerp(a?.closeIconColor, b?.closeIconColor, t),
      actionOverflowThreshold: lerpDouble(
        a?.actionOverflowThreshold,
        b?.actionOverflowThreshold,
        t,
      ),
      actionBackgroundColor: Color.lerp(a?.actionBackgroundColor, b?.actionBackgroundColor, t),
      disabledActionBackgroundColor: Color.lerp(
        a?.disabledActionBackgroundColor,
        b?.disabledActionBackgroundColor,
        t,
      ),
      dismissDirection: t < 0.5 ? a?.dismissDirection : b?.dismissDirection,
    );
  }

  @override
  int get hashCode => Object.hash(
    backgroundColor,
    actionTextColor,
    disabledActionTextColor,
    contentTextStyle,
    elevation,
    shape,
    behavior,
    width,
    insetPadding,
    showCloseIcon,
    closeIconColor,
    actionOverflowThreshold,
    actionBackgroundColor,
    disabledActionBackgroundColor,
    dismissDirection,
  );

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is SnackBarThemeData &&
        other.backgroundColor == backgroundColor &&
        other.actionTextColor == actionTextColor &&
        other.disabledActionTextColor == disabledActionTextColor &&
        other.contentTextStyle == contentTextStyle &&
        other.elevation == elevation &&
        other.shape == shape &&
        other.behavior == behavior &&
        other.width == width &&
        other.insetPadding == insetPadding &&
        other.showCloseIcon == showCloseIcon &&
        other.closeIconColor == closeIconColor &&
        other.actionOverflowThreshold == actionOverflowThreshold &&
        other.actionBackgroundColor == actionBackgroundColor &&
        other.disabledActionBackgroundColor == disabledActionBackgroundColor &&
        other.dismissDirection == dismissDirection;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(ColorProperty('backgroundColor', backgroundColor, defaultValue: null));
    properties.add(ColorProperty('actionTextColor', actionTextColor, defaultValue: null));
    properties.add(
      ColorProperty('disabledActionTextColor', disabledActionTextColor, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>('contentTextStyle', contentTextStyle, defaultValue: null),
    );
    properties.add(DoubleProperty('elevation', elevation, defaultValue: null));
    properties.add(DiagnosticsProperty<ShapeBorder>('shape', shape, defaultValue: null));
    properties.add(DiagnosticsProperty<SnackBarBehavior>('behavior', behavior, defaultValue: null));
    properties.add(DoubleProperty('width', width, defaultValue: null));
    properties.add(
      DiagnosticsProperty<EdgeInsets>('insetPadding', insetPadding, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<bool>('showCloseIcon', showCloseIcon, defaultValue: null));
    properties.add(ColorProperty('closeIconColor', closeIconColor, defaultValue: null));
    properties.add(
      DoubleProperty('actionOverflowThreshold', actionOverflowThreshold, defaultValue: null),
    );
    properties.add(
      ColorProperty('actionBackgroundColor', actionBackgroundColor, defaultValue: null),
    );
    properties.add(
      ColorProperty(
        'disabledActionBackgroundColor',
        disabledActionBackgroundColor,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<DismissDirection>(
        'dismissDirection',
        dismissDirection,
        defaultValue: null,
      ),
    );
  }
}
