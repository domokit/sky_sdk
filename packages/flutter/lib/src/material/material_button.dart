// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'elevated_button.dart';
/// @docImport 'elevated_button_theme.dart';
/// @docImport 'icon_button.dart';
/// @docImport 'outlined_button.dart';
/// @docImport 'outlined_button_theme.dart';
/// @docImport 'text_button.dart';
/// @docImport 'text_button_theme.dart';
/// @docImport 'text_theme.dart';
library;

import 'dart:ui';

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'button.dart';
import 'button_theme.dart';
import 'constants.dart';
import 'ink_well.dart';
import 'material.dart';
import 'theme.dart';
import 'theme_data.dart';

/// A utility class for building Material buttons that depend on the
/// ambient [ButtonTheme] and [Theme].
///
/// This class is planned to be deprecated in a future release.
/// Please use one or more of these buttons and associated themes instead:
///
///  * [TextButton], [TextButtonTheme], [TextButtonThemeData],
///  * [ElevatedButton], [ElevatedButtonTheme], [ElevatedButtonThemeData],
///  * [OutlinedButton], [OutlinedButtonTheme], [OutlinedButtonThemeData]
///
/// The button's size will expand to fit the child widget, if necessary.
///
/// MaterialButtons whose [onPressed] and [onLongPress] callbacks are null will be disabled. To have
/// an enabled button, make sure to pass a non-null value for [onPressed] or [onLongPress].
///
/// To create a button directly, without inheriting theme defaults, use
/// [RawMaterialButton].
///
/// If you want an ink-splash effect for taps, but don't want to use a button,
/// consider using [InkWell] directly.
///
/// See also:
///
///  * [IconButton], to create buttons that contain icons rather than text.
class MaterialButton extends StatelessWidget {
  /// Creates a Material Design button.
  ///
  /// To create a custom Material button consider using [TextButton],
  /// [ElevatedButton], or [OutlinedButton].
  ///
  /// The [elevation], [hoverElevation], [focusElevation], [highlightElevation],
  /// and [disabledElevation] arguments must be non-negative, if specified.
  const MaterialButton({
    super.key,
    required this.onPressed,
    this.onLongPress,
    this.onHighlightChanged,
    this.mouseCursor,
    this.textTheme,
    this.textColor,
    this.disabledTextColor,
    this.color,
    this.disabledColor,
    this.focusColor,
    this.hoverColor,
    this.highlightColor,
    this.splashColor,
    this.colorBrightness,
    this.elevation,
    this.focusElevation,
    this.hoverElevation,
    this.highlightElevation,
    this.disabledElevation,
    this.padding,
    this.visualDensity,
    this.shape,
    this.clipBehavior = Clip.none,
    this.focusNode,
    this.autofocus = false,
    this.materialTapTargetSize,
    this.animationDuration,
    this.minWidth,
    this.height,
    this.enableFeedback = true,
    this.child,
  }) : assert(elevation == null || elevation >= 0.0),
       assert(focusElevation == null || focusElevation >= 0.0),
       assert(hoverElevation == null || hoverElevation >= 0.0),
       assert(highlightElevation == null || highlightElevation >= 0.0),
       assert(disabledElevation == null || disabledElevation >= 0.0);

  /// The callback that is called when the button is tapped or otherwise activated.
  ///
  /// If this callback and [onLongPress] are null, then the button will be disabled.
  ///
  /// See also:
  ///
  ///  * [enabled], which is true if the button is enabled.
  final VoidCallback? onPressed;

  /// The callback that is called when the button is long-pressed.
  ///
  /// If this callback and [onPressed] are null, then the button will be disabled.
  ///
  /// See also:
  ///
  ///  * [enabled], which is true if the button is enabled.
  final VoidCallback? onLongPress;

  /// Called by the underlying [InkWell] widget's [InkWell.onHighlightChanged]
  /// callback.
  ///
  /// If [onPressed] changes from null to non-null while a gesture is ongoing,
  /// this can fire during the build phase (in which case calling
  /// [State.setState] is not allowed).
  final ValueChanged<bool>? onHighlightChanged;

  /// {@macro flutter.material.RawMaterialButton.mouseCursor}
  ///
  /// If this property is null, [WidgetStateMouseCursor.clickable] will be used.
  final MouseCursor? mouseCursor;

  /// Defines the button's base colors, and the defaults for the button's minimum
  /// size, internal padding, and shape.
  ///
  /// Defaults to `ButtonTheme.of(context).textTheme`.
  final ButtonTextTheme? textTheme;

  /// The color to use for this button's text.
  ///
  /// The button's [Material.textStyle] will be the current theme's button text
  /// style, [TextTheme.labelLarge] of [ThemeData.textTheme], configured with this
  /// color.
  ///
  /// The default text color depends on the button theme's text theme,
  /// [ButtonThemeData.textTheme].
  ///
  /// If [textColor] is a [WidgetStateProperty<Color>], [disabledTextColor]
  /// will be ignored.
  ///
  /// See also:
  ///
  ///  * [disabledTextColor], the text color to use when the button has been
  ///    disabled.
  final Color? textColor;

  /// The color to use for this button's text when the button is disabled.
  ///
  /// The button's [Material.textStyle] will be the current theme's button text
  /// style, [TextTheme.labelLarge] of [ThemeData.textTheme], configured with this
  /// color.
  ///
  /// The default value is the theme's disabled color,
  /// [ThemeData.disabledColor].
  ///
  /// If [textColor] is a [WidgetStateProperty<Color>], [disabledTextColor]
  /// will be ignored.
  ///
  /// See also:
  ///
  ///  * [textColor] - The color to use for this button's text when the button is [enabled].
  final Color? disabledTextColor;

  /// The button's fill color, displayed by its [Material], while it
  /// is in its default (unpressed, [enabled]) state.
  ///
  /// See also:
  ///
  ///  * [disabledColor] - the fill color of the button when the button is disabled.
  final Color? color;

  /// The fill color of the button when the button is disabled.
  ///
  /// The default value of this color is the theme's disabled color,
  /// [ThemeData.disabledColor].
  ///
  /// See also:
  ///
  ///  * [color] - the fill color of the button when the button is [enabled].
  final Color? disabledColor;

  /// The splash color of the button's [InkWell].
  ///
  /// The ink splash indicates that the button has been touched. It
  /// appears on top of the button's child and spreads in an expanding
  /// circle beginning where the touch occurred.
  ///
  /// The default splash color is the current theme's splash color,
  /// [ThemeData.splashColor].
  ///
  /// The appearance of the splash can be configured with the theme's splash
  /// factory, [ThemeData.splashFactory].
  final Color? splashColor;

  /// The fill color of the button's [Material] when it has the input focus.
  ///
  /// The button changed focus color when the button has the input focus. It
  /// appears behind the button's child.
  final Color? focusColor;

  /// The fill color of the button's [Material] when a pointer is hovering over
  /// it.
  ///
  /// The button changes fill color when a pointer is hovering over the button.
  /// It appears behind the button's child.
  final Color? hoverColor;

  /// The highlight color of the button's [InkWell].
  ///
  /// The highlight indicates that the button is actively being pressed. It
  /// appears on top of the button's child and quickly spreads to fill
  /// the button, and then fades out.
  ///
  /// If [textTheme] is [ButtonTextTheme.primary], the default highlight color is
  /// transparent (in other words the highlight doesn't appear). Otherwise it's
  /// the current theme's highlight color, [ThemeData.highlightColor].
  final Color? highlightColor;

  /// The z-coordinate at which to place this button relative to its parent.
  ///
  /// This controls the size of the shadow below the raised button.
  ///
  /// Defaults to 2, the appropriate elevation for raised buttons. The value
  /// is always non-negative.
  ///
  /// See also:
  ///
  ///  * [TextButton], a button with no elevation or fill color.
  ///  * [focusElevation], the elevation when the button is focused.
  ///  * [hoverElevation], the elevation when a pointer is hovering over the
  ///    button.
  ///  * [disabledElevation], the elevation when the button is disabled.
  ///  * [highlightElevation], the elevation when the button is pressed.
  final double? elevation;

  /// The elevation for the button's [Material] when the button
  /// is [enabled] and a pointer is hovering over it.
  ///
  /// Defaults to 4.0. The value is always non-negative.
  ///
  /// See also:
  ///
  ///  * [elevation], the default elevation.
  ///  * [focusElevation], the elevation when the button is focused.
  ///  * [disabledElevation], the elevation when the button is disabled.
  ///  * [highlightElevation], the elevation when the button is pressed.
  final double? hoverElevation;

  /// The elevation for the button's [Material] when the button
  /// is [enabled] and has the input focus.
  ///
  /// Defaults to 4.0. The value is always non-negative.
  ///
  /// See also:
  ///
  ///  * [elevation], the default elevation.
  ///  * [hoverElevation], the elevation when a pointer is hovering over the
  ///    button.
  ///  * [disabledElevation], the elevation when the button is disabled.
  ///  * [highlightElevation], the elevation when the button is pressed.
  final double? focusElevation;

  /// The elevation for the button's [Material] relative to its parent when the
  /// button is [enabled] and pressed.
  ///
  /// This controls the size of the shadow below the button. When a tap
  /// down gesture occurs within the button, its [InkWell] displays a
  /// [highlightColor] "highlight".
  ///
  /// Defaults to 8.0. The value is always non-negative.
  ///
  /// See also:
  ///
  ///  * [elevation], the default elevation.
  ///  * [focusElevation], the elevation when the button is focused.
  ///  * [hoverElevation], the elevation when a pointer is hovering over the
  ///    button.
  ///  * [disabledElevation], the elevation when the button is disabled.
  final double? highlightElevation;

  /// The elevation for the button's [Material] relative to its parent when the
  /// button is not [enabled].
  ///
  /// Defaults to 0.0. The value is always non-negative.
  ///
  /// See also:
  ///
  ///  * [elevation], the default elevation.
  ///  * [highlightElevation], the elevation when the button is pressed.
  final double? disabledElevation;

  /// The theme brightness to use for this button.
  ///
  /// Defaults to the theme's brightness in [ThemeData.brightness]. Setting
  /// this value determines the button text's colors based on
  /// [ButtonThemeData.getTextColor].
  ///
  /// See also:
  ///
  ///  * [ButtonTextTheme], uses [Brightness] to determine text color.
  final Brightness? colorBrightness;

  /// The button's label.
  ///
  /// Often a [Text] widget in all caps.
  final Widget? child;

  /// Whether the button is enabled or disabled.
  ///
  /// Buttons are disabled by default. To enable a button, set its [onPressed]
  /// or [onLongPress] properties to a non-null value.
  bool get enabled => onPressed != null || onLongPress != null;

  /// The internal padding for the button's [child].
  ///
  /// Defaults to the value from the current [ButtonTheme],
  /// [ButtonThemeData.padding].
  final EdgeInsetsGeometry? padding;

  /// Defines how compact the button's layout will be.
  ///
  /// {@macro flutter.material.themedata.visualDensity}
  ///
  /// See also:
  ///
  ///  * [ThemeData.visualDensity], which specifies the [visualDensity] for all
  ///    widgets within a [Theme].
  final VisualDensity? visualDensity;

  /// The shape of the button's [Material].
  ///
  /// The button's highlight and splash are clipped to this shape. If the
  /// button has an elevation, then its drop shadow is defined by this
  /// shape as well.
  ///
  /// Defaults to the value from the current [ButtonTheme],
  /// [ButtonThemeData.shape].
  final ShapeBorder? shape;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.none].
  final Clip clipBehavior;

  /// {@macro flutter.widgets.Focus.focusNode}
  final FocusNode? focusNode;

  /// {@macro flutter.widgets.Focus.autofocus}
  final bool autofocus;

  /// Defines the duration of animated changes for [shape] and [elevation].
  ///
  /// The default value is [kThemeChangeDuration].
  final Duration? animationDuration;

  /// Configures the minimum size of the tap target.
  ///
  /// Defaults to [ThemeData.materialTapTargetSize].
  ///
  /// See also:
  ///
  ///  * [MaterialTapTargetSize], for a description of how this affects tap targets.
  final MaterialTapTargetSize? materialTapTargetSize;

  /// The smallest horizontal extent that the button will occupy.
  ///
  /// Defaults to the value from the current [ButtonTheme].
  final double? minWidth;

  /// The vertical extent of the button.
  ///
  /// Defaults to the value from the current [ButtonTheme].
  final double? height;

  /// Whether detected gestures should provide acoustic and/or haptic feedback.
  ///
  /// For example, on Android a tap will produce a clicking sound and a
  /// long-press will produce a short vibration, when feedback is enabled.
  ///
  /// See also:
  ///
  ///  * [Feedback] for providing platform-specific feedback to certain actions.
  final bool enableFeedback;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final ButtonThemeData buttonTheme = ButtonTheme.of(context);

    return RawMaterialButton(
      onPressed: onPressed,
      onLongPress: onLongPress,
      enableFeedback: enableFeedback,
      onHighlightChanged: onHighlightChanged,
      mouseCursor: mouseCursor,
      fillColor: buttonTheme.getFillColor(this),
      textStyle: theme.textTheme.labelLarge!.copyWith(color: buttonTheme.getTextColor(this)),
      focusColor: focusColor ?? buttonTheme.getFocusColor(this),
      hoverColor: hoverColor ?? buttonTheme.getHoverColor(this),
      highlightColor: highlightColor ?? theme.highlightColor,
      splashColor: splashColor ?? theme.splashColor,
      elevation: buttonTheme.getElevation(this),
      focusElevation: buttonTheme.getFocusElevation(this),
      hoverElevation: buttonTheme.getHoverElevation(this),
      highlightElevation: buttonTheme.getHighlightElevation(this),
      padding: buttonTheme.getPadding(this),
      visualDensity: visualDensity ?? theme.visualDensity,
      constraints: buttonTheme.getConstraints(this).copyWith(minWidth: minWidth, minHeight: height),
      shape: buttonTheme.getShape(this),
      clipBehavior: clipBehavior,
      focusNode: focusNode,
      autofocus: autofocus,
      animationDuration: buttonTheme.getAnimationDuration(this),
      materialTapTargetSize: materialTapTargetSize ?? theme.materialTapTargetSize,
      disabledElevation: disabledElevation ?? 0.0,
      child: child,
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(FlagProperty('enabled', value: enabled, ifFalse: 'disabled'));
    properties.add(
      DiagnosticsProperty<ButtonTextTheme>('textTheme', textTheme, defaultValue: null),
    );
    properties.add(ColorProperty('textColor', textColor, defaultValue: null));
    properties.add(ColorProperty('disabledTextColor', disabledTextColor, defaultValue: null));
    properties.add(ColorProperty('color', color, defaultValue: null));
    properties.add(ColorProperty('disabledColor', disabledColor, defaultValue: null));
    properties.add(ColorProperty('focusColor', focusColor, defaultValue: null));
    properties.add(ColorProperty('hoverColor', hoverColor, defaultValue: null));
    properties.add(ColorProperty('highlightColor', highlightColor, defaultValue: null));
    properties.add(ColorProperty('splashColor', splashColor, defaultValue: null));
    properties.add(
      DiagnosticsProperty<Brightness>('colorBrightness', colorBrightness, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<EdgeInsetsGeometry>('padding', padding, defaultValue: null));
    properties.add(
      DiagnosticsProperty<VisualDensity>('visualDensity', visualDensity, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<ShapeBorder>('shape', shape, defaultValue: null));
    properties.add(DiagnosticsProperty<FocusNode>('focusNode', focusNode, defaultValue: null));
    properties.add(
      DiagnosticsProperty<MaterialTapTargetSize>(
        'materialTapTargetSize',
        materialTapTargetSize,
        defaultValue: null,
      ),
    );
  }
}
