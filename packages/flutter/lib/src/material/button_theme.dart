// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'button.dart';
/// @docImport 'button_bar.dart';
/// @docImport 'dropdown.dart';
/// @docImport 'elevated_button.dart';
/// @docImport 'elevated_button_theme.dart';
/// @docImport 'filled_button.dart';
/// @docImport 'filled_button_theme.dart';
/// @docImport 'material.dart';
/// @docImport 'outlined_button.dart';
/// @docImport 'outlined_button_theme.dart';
/// @docImport 'text_button.dart';
/// @docImport 'text_button_theme.dart';
/// @docImport 'text_theme.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'color_scheme.dart';
import 'colors.dart';
import 'constants.dart';
import 'material_button.dart';
import 'theme.dart';
import 'theme_data.dart' show MaterialTapTargetSize;

// Examples can assume:
// late BuildContext context;

/// Used with [ButtonTheme] and [ButtonThemeData] to define a button's base
/// colors, and the defaults for the button's minimum size, internal padding,
/// and shape.
enum ButtonTextTheme {
  /// Button text is black or white depending on [ThemeData.brightness].
  normal,

  /// Button text is [ColorScheme.secondary].
  accent,

  /// Button text is based on [ThemeData.primaryColor].
  primary,
}

/// Used with [ButtonTheme] and [ButtonThemeData] to define how the button bar
/// should size itself with either constraints or internal padding.
enum ButtonBarLayoutBehavior {
  /// Button bars will be constrained to a minimum height of 52.
  ///
  /// This setting is require to create button bars which conform to the
  /// Material Design specification.
  constrained,

  /// Button bars will calculate their padding from the button theme padding.
  padded,
}

/// Used with [ButtonThemeData] to configure the color and geometry of buttons.
///
/// This class is planned to be deprecated in a future release.
/// Please use one or more of these buttons and associated themes instead:
///
///  * [ElevatedButton], [ElevatedButtonTheme], [ElevatedButtonThemeData],
///  * [FilledButton], [FilledButtonTheme], [FilledButtonThemeData],
///  * [OutlinedButton], [OutlinedButtonTheme], [OutlinedButtonThemeData]
///  * [TextButton], [TextButtonTheme], [TextButtonThemeData],
///
/// A button theme can be specified as part of the overall Material theme
/// using [ThemeData.buttonTheme]. The Material theme's button theme data
/// can be overridden with [ButtonTheme].
///
/// The actual appearance of buttons depends on the button theme, the
/// button's enabled state, its elevation (if any), and the overall [Theme].
///
/// See also:
///
///  * [RawMaterialButton], which can be used to configure a button that doesn't
///    depend on any inherited themes.
class ButtonTheme extends InheritedTheme {
  /// Creates a button theme.
  ButtonTheme({
    super.key,
    ButtonTextTheme textTheme = ButtonTextTheme.normal,
    ButtonBarLayoutBehavior layoutBehavior = ButtonBarLayoutBehavior.padded,
    double minWidth = 88.0,
    double height = 36.0,
    EdgeInsetsGeometry? padding,
    ShapeBorder? shape,
    bool alignedDropdown = false,
    Color? buttonColor,
    Color? disabledColor,
    Color? focusColor,
    Color? hoverColor,
    Color? highlightColor,
    Color? splashColor,
    ColorScheme? colorScheme,
    MaterialTapTargetSize? materialTapTargetSize,
    required super.child,
  }) : assert(minWidth >= 0.0),
       assert(height >= 0.0),
       data = ButtonThemeData(
         textTheme: textTheme,
         minWidth: minWidth,
         height: height,
         padding: padding,
         shape: shape,
         alignedDropdown: alignedDropdown,
         layoutBehavior: layoutBehavior,
         buttonColor: buttonColor,
         disabledColor: disabledColor,
         focusColor: focusColor,
         hoverColor: hoverColor,
         highlightColor: highlightColor,
         splashColor: splashColor,
         colorScheme: colorScheme,
         materialTapTargetSize: materialTapTargetSize,
       );

  /// Creates a button theme from [data].
  const ButtonTheme.fromButtonThemeData({super.key, required this.data, required super.child});

  /// Specifies the color and geometry of buttons.
  final ButtonThemeData data;

  /// The closest instance of this class that encloses the given context.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// ButtonThemeData theme = ButtonTheme.of(context);
  /// ```
  static ButtonThemeData of(BuildContext context) {
    final ButtonTheme? inheritedButtonTheme =
        context.dependOnInheritedWidgetOfExactType<ButtonTheme>();
    ButtonThemeData? buttonTheme = inheritedButtonTheme?.data;
    if (buttonTheme?.colorScheme == null) {
      // if buttonTheme or buttonTheme.colorScheme is null
      final ThemeData theme = Theme.of(context);
      buttonTheme ??= theme.buttonTheme;
      if (buttonTheme.colorScheme == null) {
        buttonTheme = buttonTheme.copyWith(
          colorScheme: theme.buttonTheme.colorScheme ?? theme.colorScheme,
        );
        assert(buttonTheme.colorScheme != null);
      }
    }
    return buttonTheme!;
  }

  @override
  Widget wrap(BuildContext context, Widget child) {
    return ButtonTheme.fromButtonThemeData(data: data, child: child);
  }

  @override
  bool updateShouldNotify(ButtonTheme oldWidget) => data != oldWidget.data;
}

/// Used with [ButtonTheme] to configure the color and geometry of buttons.
///
/// This class is planned to be deprecated in a future release.
/// Please use one or more of these buttons and associated themes instead:
///
///  * [TextButton], [TextButtonTheme], [TextButtonThemeData],
///  * [ElevatedButton], [ElevatedButtonTheme], [ElevatedButtonThemeData],
///  * [OutlinedButton], [OutlinedButtonTheme], [OutlinedButtonThemeData]
///
/// A button theme can be specified as part of the overall Material theme
/// using [ThemeData.buttonTheme]. The Material theme's button theme data
/// can be overridden with [ButtonTheme].
@immutable
class ButtonThemeData with Diagnosticable {
  /// Create a button theme object that can be used with [ButtonTheme]
  /// or [ThemeData].
  ///
  /// The [minWidth] and [height] parameters must greater than or equal to zero.
  ///
  /// The ButtonTheme's methods that have a [MaterialButton] parameter and
  /// have a name with a `get` prefix are used to configure a
  /// [RawMaterialButton].
  const ButtonThemeData({
    this.textTheme = ButtonTextTheme.normal,
    this.minWidth = 88.0,
    this.height = 36.0,
    EdgeInsetsGeometry? padding,
    ShapeBorder? shape,
    this.layoutBehavior = ButtonBarLayoutBehavior.padded,
    this.alignedDropdown = false,
    Color? buttonColor,
    Color? disabledColor,
    Color? focusColor,
    Color? hoverColor,
    Color? highlightColor,
    Color? splashColor,
    this.colorScheme,
    MaterialTapTargetSize? materialTapTargetSize,
  }) : assert(minWidth >= 0.0),
       assert(height >= 0.0),
       _buttonColor = buttonColor,
       _disabledColor = disabledColor,
       _focusColor = focusColor,
       _hoverColor = hoverColor,
       _highlightColor = highlightColor,
       _splashColor = splashColor,
       _padding = padding,
       _shape = shape,
       _materialTapTargetSize = materialTapTargetSize;

  /// The minimum width for buttons.
  ///
  /// The actual horizontal space allocated for a button's child is
  /// at least this value less the theme's horizontal [padding].
  ///
  /// Defaults to 88.0 logical pixels.
  final double minWidth;

  /// The minimum height for buttons.
  ///
  /// Defaults to 36.0 logical pixels.
  final double height;

  /// Defines a button's base colors, and the defaults for the button's minimum
  /// size, internal padding, and shape.
  ///
  /// Despite the name, this property is not a [TextTheme], its value is not a
  /// collection of [TextStyle]s.
  final ButtonTextTheme textTheme;

  /// Defines whether a [ButtonBar] should size itself with a minimum size
  /// constraint or with padding.
  ///
  /// Defaults to [ButtonBarLayoutBehavior.padded].
  final ButtonBarLayoutBehavior layoutBehavior;

  /// Convenience that returns [minWidth] and [height] as a
  /// [BoxConstraints] object.
  BoxConstraints get constraints {
    return BoxConstraints(minWidth: minWidth, minHeight: height);
  }

  /// Padding for a button's child (typically the button's label).
  ///
  /// Defaults to 24.0 on the left and right if [textTheme] is
  /// [ButtonTextTheme.primary], 16.0 on the left and right otherwise.
  ///
  /// See also:
  ///
  ///  * [getPadding], which is used to calculate padding for the button's
  ///    child (typically the button's label).
  EdgeInsetsGeometry get padding =>
      _padding ??
      switch (textTheme) {
        ButtonTextTheme.normal => const EdgeInsets.symmetric(horizontal: 16.0),
        ButtonTextTheme.accent => const EdgeInsets.symmetric(horizontal: 16.0),
        ButtonTextTheme.primary => const EdgeInsets.symmetric(horizontal: 24.0),
      };
  final EdgeInsetsGeometry? _padding;

  /// The shape of a button's material.
  ///
  /// The button's highlight and splash are clipped to this shape. If the
  /// button has an elevation, then its drop shadow is defined by this
  /// shape as well.
  ///
  /// Defaults to a rounded rectangle with circular corner radii of 4.0 if
  /// [textTheme] is [ButtonTextTheme.primary], a rounded rectangle with
  /// circular corner radii of 2.0 otherwise.
  ///
  /// See also:
  ///
  ///  * [getShape], which is used to calculate the shape of the button's
  ///    [Material].
  ShapeBorder get shape =>
      _shape ??
      switch (textTheme) {
        ButtonTextTheme.normal || ButtonTextTheme.accent => const RoundedRectangleBorder(
          borderRadius: BorderRadius.all(Radius.circular(2.0)),
        ),
        ButtonTextTheme.primary => const RoundedRectangleBorder(
          borderRadius: BorderRadius.all(Radius.circular(4.0)),
        ),
      };
  final ShapeBorder? _shape;

  /// If true, then a [DropdownButton] menu's width will match the button's
  /// width.
  ///
  /// If false (the default), then the dropdown's menu will be wider than
  /// its button. In either case the dropdown button will line up the leading
  /// edge of the menu's value with the leading edge of the values
  /// displayed by the menu items.
  ///
  /// This property only affects [DropdownButton] and its menu.
  final bool alignedDropdown;

  /// The background fill color.
  ///
  /// This property is null by default.
  ///
  /// If the button is in the focused, hovering, or highlighted state, then the
  /// `focusColor`, `hoverColor`, or `highlightColor` will take precedence over
  /// the `buttonColor`.
  ///
  /// See also:
  ///
  ///  * [getFillColor], which is used to compute the background fill color.
  final Color? _buttonColor;

  /// The background fill color when disabled.
  ///
  /// This property is null by default.
  ///
  /// See also:
  ///
  ///  * [getDisabledFillColor], which is to compute background fill color for
  ///    disabled state.
  final Color? _disabledColor;

  /// The fill color of the button when it has the input focus.
  ///
  /// This property is null by default.
  ///
  /// If the button is in the hovering or highlighted state, then the `hoverColor`
  /// or `highlightColor` will take precedence over the `focusColor`.
  ///
  /// See also:
  ///
  ///  * [getFocusColor], which is used to compute the fill color of the button
  ///    when it has input focus.
  final Color? _focusColor;

  /// The fill color of the button when a pointer is hovering over it.
  ///
  /// This property is null by default.
  ///
  /// If the button is in the highlighted state, then the `highlightColor` will
  /// take precedence over the `hoverColor`.
  ///
  /// See also:
  ///
  ///  * [getHoverColor], which is used to compute the fill color of the button
  ///    when it has input focus.
  final Color? _hoverColor;

  /// The color of the overlay that appears when a button is pressed.
  ///
  /// This property is null by default.
  ///
  /// See also:
  ///
  ///  * [getHighlightColor], which is used to compute the color of the overlay
  ///    that appears when the `button` is pressed.
  final Color? _highlightColor;

  /// The color of the ink "splash" overlay that appears when a button is tapped.
  ///
  /// This property is null by default.
  ///
  /// See also:
  ///
  ///  * [getSplashColor], which is used to compute the color of the ink
  ///    "splash" overlay that appears when the (enabled) `button` is tapped.
  final Color? _splashColor;

  /// A set of thirteen colors that can be used to derive the button theme's
  /// colors.
  ///
  /// This property was added much later than the theme's set of highly specific
  /// colors, like [ThemeData.highlightColor] and [ThemeData.splashColor] etc.
  ///
  /// The colors for new button classes can be defined exclusively in terms of
  /// [colorScheme]. When it's possible, the existing buttons will (continue to)
  /// gradually migrate to it.
  final ColorScheme? colorScheme;

  // The minimum size of a button's tap target.
  //
  // This property is null by default.
  final MaterialTapTargetSize? _materialTapTargetSize;

  /// The [button]'s overall brightness.
  ///
  /// Returns the button's [MaterialButton.colorBrightness] if it is non-null,
  /// otherwise the color scheme's [ColorScheme.brightness] is returned.
  Brightness getBrightness(MaterialButton button) {
    return button.colorBrightness ?? colorScheme!.brightness;
  }

  /// Defines the [button]'s base colors, and the defaults for the button's
  /// minimum size, internal padding, and shape.
  ///
  /// Despite the name, this property is not the [TextTheme] whose
  /// [TextTheme.labelLarge] is used as the button text's [TextStyle].
  ButtonTextTheme getTextTheme(MaterialButton button) => button.textTheme ?? textTheme;

  /// The foreground color of the [button]'s text and icon when
  /// [MaterialButton.onPressed] is null (when MaterialButton.enabled is false).
  ///
  /// Returns the button's [MaterialButton.disabledColor] if it is non-null.
  /// Otherwise the color scheme's [ColorScheme.onSurface] color is returned
  /// with its opacity set to 0.38.
  ///
  /// If [MaterialButton.textColor] is a [WidgetStateProperty<Color>], it will be
  /// used as the `disabledTextColor`. It will be resolved in the [WidgetState.disabled] state.
  Color getDisabledTextColor(MaterialButton button) {
    return button.textColor ?? button.disabledTextColor ?? colorScheme!.onSurface.withOpacity(0.38);
  }

  /// The [button]'s background color when [MaterialButton.onPressed] is null
  /// (when [MaterialButton.enabled] is false).
  ///
  /// Returns the button's [MaterialButton.disabledColor] if it is non-null.
  ///
  /// Otherwise the value of the `disabledColor` constructor parameter
  /// is returned, if it is non-null.
  ///
  /// Otherwise the color scheme's [ColorScheme.onSurface] color is returned
  /// with its opacity set to 0.38.
  Color getDisabledFillColor(MaterialButton button) {
    return button.disabledColor ?? _disabledColor ?? colorScheme!.onSurface.withOpacity(0.38);
  }

  /// The button's background fill color or null for buttons that don't have
  /// a background color.
  ///
  /// Returns [MaterialButton.color] if it is non-null and the button
  /// is enabled.
  ///
  /// Otherwise, returns [MaterialButton.disabledColor] if it is non-null and
  /// the button is disabled.
  ///
  /// Otherwise the fill color depends on the value of [getTextTheme].
  ///
  ///  * [ButtonTextTheme.normal] or [ButtonTextTheme.accent], the
  ///    color scheme's [ColorScheme.primary] color if the [button] is enabled
  ///    the value of [getDisabledFillColor] otherwise.
  ///  * [ButtonTextTheme.primary], if the [button] is enabled then the value
  ///    of the `buttonColor` constructor parameter if it is non-null,
  ///    otherwise the color scheme's ColorScheme.primary color. If the button
  ///    is not enabled then the colorScheme's [ColorScheme.onSurface] color
  ///    with opacity 0.12.
  Color? getFillColor(MaterialButton button) {
    final Color? fillColor = button.enabled ? button.color : button.disabledColor;
    if (fillColor != null) {
      return fillColor;
    }

    if (button.runtimeType == MaterialButton) {
      return null;
    }

    if (button.enabled && _buttonColor != null) {
      return _buttonColor;
    }

    switch (getTextTheme(button)) {
      case ButtonTextTheme.normal:
      case ButtonTextTheme.accent:
        return button.enabled ? colorScheme!.primary : getDisabledFillColor(button);
      case ButtonTextTheme.primary:
        return button.enabled
            ? _buttonColor ?? colorScheme!.primary
            : colorScheme!.onSurface.withOpacity(0.12);
    }
  }

  /// The foreground color of the [button]'s text and icon.
  ///
  /// If [button] is not [MaterialButton.enabled], the value of
  /// [getDisabledTextColor] is returned. If the button is enabled and
  /// [MaterialButton.textColor] is non-null, then [MaterialButton.textColor]
  /// is returned.
  ///
  /// Otherwise the text color depends on the value of [getTextTheme]
  /// and [getBrightness].
  ///
  ///  * [ButtonTextTheme.normal]: [Colors.white] is used if [getBrightness]
  ///    resolves to [Brightness.dark]. [Colors.black87] is used if
  ///    [getBrightness] resolves to [Brightness.light].
  ///  * [ButtonTextTheme.accent]: [ColorScheme.secondary] of [colorScheme].
  ///  * [ButtonTextTheme.primary]: If [getFillColor] is dark then [Colors.white],
  ///    otherwise [Colors.black].
  Color getTextColor(MaterialButton button) {
    if (!button.enabled) {
      return getDisabledTextColor(button);
    }

    if (button.textColor != null) {
      return button.textColor!;
    }

    switch (getTextTheme(button)) {
      case ButtonTextTheme.normal:
        return getBrightness(button) == Brightness.dark ? Colors.white : Colors.black87;

      case ButtonTextTheme.accent:
        return colorScheme!.secondary;

      case ButtonTextTheme.primary:
        final Color? fillColor = getFillColor(button);
        final bool fillIsDark =
            fillColor != null
                ? ThemeData.estimateBrightnessForColor(fillColor) == Brightness.dark
                : getBrightness(button) == Brightness.dark;
        return fillIsDark ? Colors.white : Colors.black;
    }
  }

  /// The color of the ink "splash" overlay that appears when the (enabled)
  /// [button] is tapped.
  ///
  /// Returns the button's [MaterialButton.splashColor] if it is non-null.
  ///
  /// Otherwise, returns the value of the `splashColor` constructor parameter
  /// it is non-null.
  ///
  /// Otherwise, returns the value of the `splashColor` constructor parameter
  /// if it is non-null and [getTextTheme] is not [ButtonTextTheme.primary].
  ///
  /// Otherwise, returns [getTextColor] with an opacity of 0.12.
  Color getSplashColor(MaterialButton button) {
    if (button.splashColor != null) {
      return button.splashColor!;
    }

    if (_splashColor != null) {
      switch (getTextTheme(button)) {
        case ButtonTextTheme.normal:
        case ButtonTextTheme.accent:
          return _splashColor;
        case ButtonTextTheme.primary:
          break;
      }
    }

    return getTextColor(button).withOpacity(0.12);
  }

  /// The fill color of the button when it has input focus.
  ///
  /// Returns the button's [MaterialButton.focusColor] if it is non-null.
  /// Otherwise the focus color depends on [getTextTheme]:
  ///
  ///  * [ButtonTextTheme.normal], [ButtonTextTheme.accent]: returns the
  ///    value of the `focusColor` constructor parameter if it is non-null,
  ///    otherwise the value of [getTextColor] with opacity 0.12.
  ///  * [ButtonTextTheme.primary], returns [Colors.transparent].
  Color getFocusColor(MaterialButton button) {
    return button.focusColor ?? _focusColor ?? getTextColor(button).withOpacity(0.12);
  }

  /// The fill color of the button when it has input focus.
  ///
  /// Returns the button's [MaterialButton.focusColor] if it is non-null.
  /// Otherwise the focus color depends on [getTextTheme]:
  ///
  ///  * [ButtonTextTheme.normal], [ButtonTextTheme.accent],
  ///    [ButtonTextTheme.primary]: returns the value of the `focusColor`
  ///    constructor parameter if it is non-null, otherwise the value of
  ///    [getTextColor] with opacity 0.04.
  Color getHoverColor(MaterialButton button) {
    return button.hoverColor ?? _hoverColor ?? getTextColor(button).withOpacity(0.04);
  }

  /// The color of the overlay that appears when the [button] is pressed.
  ///
  /// Returns the button's [MaterialButton.highlightColor] if it is non-null.
  /// Otherwise the highlight color depends on [getTextTheme]:
  ///
  ///  * [ButtonTextTheme.normal], [ButtonTextTheme.accent]: returns the
  ///    value of the `highlightColor` constructor parameter if it is non-null,
  ///    otherwise the value of [getTextColor] with opacity 0.16.
  ///  * [ButtonTextTheme.primary], returns [Colors.transparent].
  Color getHighlightColor(MaterialButton button) {
    if (button.highlightColor != null) {
      return button.highlightColor!;
    }

    switch (getTextTheme(button)) {
      case ButtonTextTheme.normal:
      case ButtonTextTheme.accent:
        return _highlightColor ?? getTextColor(button).withOpacity(0.16);
      case ButtonTextTheme.primary:
        return Colors.transparent;
    }
  }

  /// The [button]'s elevation when it is enabled and has not been pressed.
  ///
  /// Returns the button's [MaterialButton.elevation] if it is non-null,
  /// otherwise it is 2.0.
  double getElevation(MaterialButton button) => button.elevation ?? 2.0;

  /// The [button]'s elevation when it is enabled and has focus.
  ///
  /// Returns the button's [MaterialButton.focusElevation] if it is non-null,
  /// otherwise the highlight elevation is 4.0.
  double getFocusElevation(MaterialButton button) => button.focusElevation ?? 4.0;

  /// The [button]'s elevation when it is enabled and has focus.
  ///
  /// Returns the button's [MaterialButton.hoverElevation] if it is non-null,
  /// otherwise the highlight elevation is 4.0.
  double getHoverElevation(MaterialButton button) => button.hoverElevation ?? 4.0;

  /// The [button]'s elevation when it is enabled and has been pressed.
  ///
  /// Returns the button's [MaterialButton.highlightElevation] if it is non-null,
  /// otherwise the highlight elevation is 8.0.
  double getHighlightElevation(MaterialButton button) => button.highlightElevation ?? 8.0;

  /// The [button]'s elevation when [MaterialButton.onPressed] is null (when
  /// MaterialButton.enabled is false).
  ///
  /// Returns the button's [MaterialButton.elevation] if it is non-null.
  ///
  /// Otherwise the disabled elevation is 0.0.
  double getDisabledElevation(MaterialButton button) => button.disabledElevation ?? 0.0;

  /// Padding for the [button]'s child (typically the button's label).
  ///
  /// Returns the button's [MaterialButton.padding] if it is non-null,
  /// otherwise, returns the `padding` of the constructor parameter if it is
  /// non-null.
  ///
  /// Otherwise, returns horizontal padding of 24.0 on the left and right if
  /// [getTextTheme] is [ButtonTextTheme.primary], 16.0 on the left and right
  /// otherwise.
  EdgeInsetsGeometry getPadding(MaterialButton button) {
    return button.padding ??
        _padding ??
        switch (getTextTheme(button)) {
          ButtonTextTheme.normal => const EdgeInsets.symmetric(horizontal: 16.0),
          ButtonTextTheme.accent => const EdgeInsets.symmetric(horizontal: 16.0),
          ButtonTextTheme.primary => const EdgeInsets.symmetric(horizontal: 24.0),
        };
  }

  /// The shape of the [button]'s [Material].
  ///
  /// Returns the button's [MaterialButton.shape] if it is non-null, otherwise
  /// [shape] is returned.
  ShapeBorder getShape(MaterialButton button) => button.shape ?? shape;

  /// The duration of the [button]'s highlight animation.
  ///
  /// Returns the button's [MaterialButton.animationDuration] it if is non-null,
  /// otherwise 200ms.
  Duration getAnimationDuration(MaterialButton button) {
    return button.animationDuration ?? kThemeChangeDuration;
  }

  /// The [BoxConstraints] that the define the [button]'s size.
  ///
  /// By default this method just returns [constraints]. Subclasses
  /// could override this method to return a value that was,
  /// for example, based on the button's type.
  BoxConstraints getConstraints(MaterialButton button) => constraints;

  /// The minimum size of the [button]'s tap target.
  ///
  /// Returns the button's [MaterialButton.materialTapTargetSize] if it is non-null.
  ///
  /// Otherwise the value of the `materialTapTargetSize` constructor
  /// parameter is returned if that's non-null.
  ///
  /// Otherwise [MaterialTapTargetSize.padded] is returned.
  MaterialTapTargetSize getMaterialTapTargetSize(MaterialButton button) {
    return button.materialTapTargetSize ?? _materialTapTargetSize ?? MaterialTapTargetSize.padded;
  }

  /// Creates a copy of this button theme data object with the matching fields
  /// replaced with the non-null parameter values.
  ButtonThemeData copyWith({
    ButtonTextTheme? textTheme,
    ButtonBarLayoutBehavior? layoutBehavior,
    double? minWidth,
    double? height,
    EdgeInsetsGeometry? padding,
    ShapeBorder? shape,
    bool? alignedDropdown,
    Color? buttonColor,
    Color? disabledColor,
    Color? focusColor,
    Color? hoverColor,
    Color? highlightColor,
    Color? splashColor,
    ColorScheme? colorScheme,
    MaterialTapTargetSize? materialTapTargetSize,
  }) {
    return ButtonThemeData(
      textTheme: textTheme ?? this.textTheme,
      layoutBehavior: layoutBehavior ?? this.layoutBehavior,
      minWidth: minWidth ?? this.minWidth,
      height: height ?? this.height,
      padding: padding ?? this.padding,
      shape: shape ?? this.shape,
      alignedDropdown: alignedDropdown ?? this.alignedDropdown,
      buttonColor: buttonColor ?? _buttonColor,
      disabledColor: disabledColor ?? _disabledColor,
      focusColor: focusColor ?? _focusColor,
      hoverColor: hoverColor ?? _hoverColor,
      highlightColor: highlightColor ?? _highlightColor,
      splashColor: splashColor ?? _splashColor,
      colorScheme: colorScheme ?? this.colorScheme,
      materialTapTargetSize: materialTapTargetSize ?? _materialTapTargetSize,
    );
  }

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is ButtonThemeData &&
        other.textTheme == textTheme &&
        other.minWidth == minWidth &&
        other.height == height &&
        other.padding == padding &&
        other.shape == shape &&
        other.alignedDropdown == alignedDropdown &&
        other._buttonColor == _buttonColor &&
        other._disabledColor == _disabledColor &&
        other._focusColor == _focusColor &&
        other._hoverColor == _hoverColor &&
        other._highlightColor == _highlightColor &&
        other._splashColor == _splashColor &&
        other.colorScheme == colorScheme &&
        other._materialTapTargetSize == _materialTapTargetSize;
  }

  @override
  int get hashCode => Object.hash(
    textTheme,
    minWidth,
    height,
    padding,
    shape,
    alignedDropdown,
    _buttonColor,
    _disabledColor,
    _focusColor,
    _hoverColor,
    _highlightColor,
    _splashColor,
    colorScheme,
    _materialTapTargetSize,
  );

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    const ButtonThemeData defaultTheme = ButtonThemeData();
    properties.add(
      EnumProperty<ButtonTextTheme>('textTheme', textTheme, defaultValue: defaultTheme.textTheme),
    );
    properties.add(DoubleProperty('minWidth', minWidth, defaultValue: defaultTheme.minWidth));
    properties.add(DoubleProperty('height', height, defaultValue: defaultTheme.height));
    properties.add(
      DiagnosticsProperty<EdgeInsetsGeometry>(
        'padding',
        padding,
        defaultValue: defaultTheme.padding,
      ),
    );
    properties.add(
      DiagnosticsProperty<ShapeBorder>('shape', shape, defaultValue: defaultTheme.shape),
    );
    properties.add(
      FlagProperty(
        'alignedDropdown',
        value: alignedDropdown,
        defaultValue: defaultTheme.alignedDropdown,
        ifTrue: 'dropdown width matches button',
      ),
    );
    properties.add(ColorProperty('buttonColor', _buttonColor, defaultValue: null));
    properties.add(ColorProperty('disabledColor', _disabledColor, defaultValue: null));
    properties.add(ColorProperty('focusColor', _focusColor, defaultValue: null));
    properties.add(ColorProperty('hoverColor', _hoverColor, defaultValue: null));
    properties.add(ColorProperty('highlightColor', _highlightColor, defaultValue: null));
    properties.add(ColorProperty('splashColor', _splashColor, defaultValue: null));
    properties.add(
      DiagnosticsProperty<ColorScheme>(
        'colorScheme',
        colorScheme,
        defaultValue: defaultTheme.colorScheme,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialTapTargetSize>(
        'materialTapTargetSize',
        _materialTapTargetSize,
        defaultValue: null,
      ),
    );
  }
}
