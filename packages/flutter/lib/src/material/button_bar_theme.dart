// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'button_bar.dart';
/// @docImport 'dropdown.dart';
/// @docImport 'text_theme.dart';
library;

import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'button_theme.dart';
import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// Defines the visual properties of [ButtonBar] widgets.
///
/// Used by [ButtonBarTheme] to control the visual properties of [ButtonBar]
/// instances in a widget subtree.
///
/// To obtain this configuration, use [ButtonBarTheme.of] to access the closest
/// ancestor [ButtonBarTheme] of the current [BuildContext].
///
/// See also:
///
///  * [ButtonBarTheme], an [InheritedWidget] that propagates the theme down
///    its subtree.
///  * [ButtonBar], which uses this to configure itself and its children
///    button widgets.
@Deprecated(
  'Use OverflowBar instead. '
  'This feature was deprecated after v3.21.0-10.0.pre.',
)
@immutable
class ButtonBarThemeData with Diagnosticable {
  /// Constructs the set of properties used to configure [ButtonBar] widgets.
  ///
  /// Both [buttonMinWidth] and [buttonHeight] must be non-negative if they
  /// are not null.
  @Deprecated(
    'Use OverflowBar instead. '
    'This feature was deprecated after v3.21.0-10.0.pre.',
  )
  const ButtonBarThemeData({
    this.alignment,
    this.mainAxisSize,
    this.buttonTextTheme,
    this.buttonMinWidth,
    this.buttonHeight,
    this.buttonPadding,
    this.buttonAlignedDropdown,
    this.layoutBehavior,
    this.overflowDirection,
  }) : assert(buttonMinWidth == null || buttonMinWidth >= 0.0),
       assert(buttonHeight == null || buttonHeight >= 0.0);

  /// How the children should be placed along the horizontal axis.
  final MainAxisAlignment? alignment;

  /// How much horizontal space is available. See [Row.mainAxisSize].
  final MainAxisSize? mainAxisSize;

  /// Defines a [ButtonBar] button's base colors, and the defaults for
  /// the button's minimum size, internal padding, and shape.
  ///
  /// This will override the surrounding [ButtonThemeData.textTheme] setting
  /// for buttons contained in the [ButtonBar].
  ///
  /// Despite the name, this property is not a [TextTheme], its value is not a
  /// collection of [TextStyle]s.
  final ButtonTextTheme? buttonTextTheme;

  /// The minimum width for [ButtonBar] buttons.
  ///
  /// This will override the surrounding [ButtonThemeData.minWidth] setting
  /// for buttons contained in the [ButtonBar].
  ///
  /// The actual horizontal space allocated for a button's child is
  /// at least this value less the theme's horizontal [ButtonThemeData.padding].
  final double? buttonMinWidth;

  /// The minimum height for [ButtonBar] buttons.
  ///
  /// This will override the surrounding [ButtonThemeData.height] setting
  /// for buttons contained in the [ButtonBar].
  final double? buttonHeight;

  /// Padding for a [ButtonBar] button's child (typically the button's label).
  ///
  /// This will override the surrounding [ButtonThemeData.padding] setting
  /// for buttons contained in the [ButtonBar].
  final EdgeInsetsGeometry? buttonPadding;

  /// If true, then a [DropdownButton] menu's width will match the [ButtonBar]
  /// button's width.
  ///
  /// If false, then the dropdown's menu will be wider than
  /// its button. In either case the dropdown button will line up the leading
  /// edge of the menu's value with the leading edge of the values
  /// displayed by the menu items.
  ///
  /// This will override the surrounding [ButtonThemeData.alignedDropdown] setting
  /// for buttons contained in the [ButtonBar].
  ///
  /// This property only affects [DropdownButton] contained in a [ButtonBar]
  /// and its menu.
  final bool? buttonAlignedDropdown;

  /// Defines whether a [ButtonBar] should size itself with a minimum size
  /// constraint or with padding.
  final ButtonBarLayoutBehavior? layoutBehavior;

  /// Defines the vertical direction of a [ButtonBar]'s children if it
  /// overflows.
  ///
  /// If the [ButtonBar]'s children do not fit into a single row, then they
  /// are arranged in a column. The first action is at the top of the
  /// column if this property is set to [VerticalDirection.down], since it
  /// "starts" at the top and "ends" at the bottom. On the other hand,
  /// the first action will be at the bottom of the column if this
  /// property is set to [VerticalDirection.up], since it "starts" at the
  /// bottom and "ends" at the top.
  final VerticalDirection? overflowDirection;

  /// Creates a copy of this object but with the given fields replaced with the
  /// new values.
  ButtonBarThemeData copyWith({
    MainAxisAlignment? alignment,
    MainAxisSize? mainAxisSize,
    ButtonTextTheme? buttonTextTheme,
    double? buttonMinWidth,
    double? buttonHeight,
    EdgeInsetsGeometry? buttonPadding,
    bool? buttonAlignedDropdown,
    ButtonBarLayoutBehavior? layoutBehavior,
    VerticalDirection? overflowDirection,
  }) {
    return ButtonBarThemeData(
      alignment: alignment ?? this.alignment,
      mainAxisSize: mainAxisSize ?? this.mainAxisSize,
      buttonTextTheme: buttonTextTheme ?? this.buttonTextTheme,
      buttonMinWidth: buttonMinWidth ?? this.buttonMinWidth,
      buttonHeight: buttonHeight ?? this.buttonHeight,
      buttonPadding: buttonPadding ?? this.buttonPadding,
      buttonAlignedDropdown: buttonAlignedDropdown ?? this.buttonAlignedDropdown,
      layoutBehavior: layoutBehavior ?? this.layoutBehavior,
      overflowDirection: overflowDirection ?? this.overflowDirection,
    );
  }

  /// Linearly interpolate between two button bar themes.
  ///
  /// If both arguments are null, then null is returned.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static ButtonBarThemeData? lerp(ButtonBarThemeData? a, ButtonBarThemeData? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    return ButtonBarThemeData(
      alignment: t < 0.5 ? a?.alignment : b?.alignment,
      mainAxisSize: t < 0.5 ? a?.mainAxisSize : b?.mainAxisSize,
      buttonTextTheme: t < 0.5 ? a?.buttonTextTheme : b?.buttonTextTheme,
      buttonMinWidth: lerpDouble(a?.buttonMinWidth, b?.buttonMinWidth, t),
      buttonHeight: lerpDouble(a?.buttonHeight, b?.buttonHeight, t),
      buttonPadding: EdgeInsetsGeometry.lerp(a?.buttonPadding, b?.buttonPadding, t),
      buttonAlignedDropdown: t < 0.5 ? a?.buttonAlignedDropdown : b?.buttonAlignedDropdown,
      layoutBehavior: t < 0.5 ? a?.layoutBehavior : b?.layoutBehavior,
      overflowDirection: t < 0.5 ? a?.overflowDirection : b?.overflowDirection,
    );
  }

  @override
  int get hashCode => Object.hash(
    alignment,
    mainAxisSize,
    buttonTextTheme,
    buttonMinWidth,
    buttonHeight,
    buttonPadding,
    buttonAlignedDropdown,
    layoutBehavior,
    overflowDirection,
  );

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is ButtonBarThemeData &&
        other.alignment == alignment &&
        other.mainAxisSize == mainAxisSize &&
        other.buttonTextTheme == buttonTextTheme &&
        other.buttonMinWidth == buttonMinWidth &&
        other.buttonHeight == buttonHeight &&
        other.buttonPadding == buttonPadding &&
        other.buttonAlignedDropdown == buttonAlignedDropdown &&
        other.layoutBehavior == layoutBehavior &&
        other.overflowDirection == overflowDirection;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      DiagnosticsProperty<MainAxisAlignment>('alignment', alignment, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<MainAxisSize>('mainAxisSize', mainAxisSize, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<ButtonTextTheme>('textTheme', buttonTextTheme, defaultValue: null),
    );
    properties.add(DoubleProperty('minWidth', buttonMinWidth, defaultValue: null));
    properties.add(DoubleProperty('height', buttonHeight, defaultValue: null));
    properties.add(
      DiagnosticsProperty<EdgeInsetsGeometry>('padding', buttonPadding, defaultValue: null),
    );
    properties.add(
      FlagProperty(
        'buttonAlignedDropdown',
        value: buttonAlignedDropdown,
        ifTrue: 'dropdown width matches button',
      ),
    );
    properties.add(
      DiagnosticsProperty<ButtonBarLayoutBehavior>(
        'layoutBehavior',
        layoutBehavior,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<VerticalDirection>(
        'overflowDirection',
        overflowDirection,
        defaultValue: null,
      ),
    );
  }
}

/// Applies a button bar theme to descendant [ButtonBar] widgets.
///
/// A button bar theme describes the layout and properties for the buttons
/// contained in a [ButtonBar].
///
/// Descendant widgets obtain the current theme's [ButtonBarTheme] object using
/// [ButtonBarTheme.of]. When a widget uses [ButtonBarTheme.of], it is automatically
/// rebuilt if the theme later changes.
///
/// A button bar theme can be specified as part of the overall Material theme
/// using [ThemeData.buttonBarTheme].
///
/// See also:
///
///  * [ButtonBarThemeData], which describes the actual configuration of a button
///    bar theme.
class ButtonBarTheme extends InheritedWidget {
  /// Constructs a button bar theme that configures all descendant [ButtonBar]
  /// widgets.
  const ButtonBarTheme({super.key, required this.data, required super.child});

  /// The properties used for all descendant [ButtonBar] widgets.
  final ButtonBarThemeData data;

  /// Returns the configuration [data] from the closest [ButtonBarTheme]
  /// ancestor. If there is no ancestor, it returns [ThemeData.buttonBarTheme].
  /// Applications can assume that the returned value will not be null.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// ButtonBarThemeData theme = ButtonBarTheme.of(context);
  /// ```
  static ButtonBarThemeData of(BuildContext context) {
    final ButtonBarTheme? buttonBarTheme =
        context.dependOnInheritedWidgetOfExactType<ButtonBarTheme>();
    return buttonBarTheme?.data ?? Theme.of(context).buttonBarTheme;
  }

  @override
  bool updateShouldNotify(ButtonBarTheme oldWidget) => data != oldWidget.data;
}
