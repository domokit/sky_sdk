// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'action_buttons.dart';
/// @docImport 'app_bar.dart';
/// @docImport 'elevated_button.dart';
/// @docImport 'ink_decoration.dart';
/// @docImport 'outlined_button.dart';
/// @docImport 'text_button.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'button_style.dart';
import 'button_style_button.dart';
import 'color_scheme.dart';
import 'colors.dart';
import 'constants.dart';
import 'debug.dart';
import 'icon_button_theme.dart';
import 'icons.dart';
import 'ink_well.dart';
import 'material.dart';
import 'material_state.dart';
import 'theme.dart';
import 'theme_data.dart';
import 'tooltip.dart';

// Examples can assume:
// late BuildContext context;

// Minimum logical pixel size of the IconButton.
// See: <https://material.io/design/usability/accessibility.html#layout-typography>.
const double _kMinButtonSize = kMinInteractiveDimension;

enum _IconButtonVariant { standard, filled, filledTonal, outlined }

/// A Material Design icon button.
///
/// An icon button is a picture printed on a [Material] widget that reacts to
/// touches by filling with color (ink).
///
/// Icon buttons are commonly used in the [AppBar.actions] field, but they can
/// be used in many other places as well.
///
/// If the [onPressed] callback is null, then the button will be disabled and
/// will not react to touch.
///
/// Requires one of its ancestors to be a [Material] widget. In Material Design 3,
/// this requirement no longer exists because this widget builds a subclass of
/// [ButtonStyleButton].
///
/// The hit region of an icon button will, if possible, be at least
/// kMinInteractiveDimension pixels in size, regardless of the actual
/// [iconSize], to satisfy the [touch target size](https://material.io/design/layout/spacing-methods.html#touch-targets)
/// requirements in the Material Design specification. The [alignment] controls
/// how the icon itself is positioned within the hit region.
///
/// {@tool dartpad}
/// This sample shows an [IconButton] that uses the Material icon "volume_up" to
/// increase the volume.
///
/// ![](https://flutter.github.io/assets-for-api-docs/assets/material/icon_button.png)
///
/// ** See code in examples/api/lib/material/icon_button/icon_button.0.dart **
/// {@end-tool}
///
/// ### Icon sizes
///
/// When creating an icon button with an [Icon], do not override the
/// icon's size with its [Icon.size] parameter, use the icon button's
/// [iconSize] parameter instead. For example do this:
///
/// ```dart
/// IconButton(
///   iconSize: 72,
///   icon: const Icon(Icons.favorite),
///   onPressed: () {
///     // ...
///   },
/// ),
/// ```
///
/// Avoid doing this:
///
/// ```dart
/// IconButton(
///   icon: const Icon(Icons.favorite, size: 72),
///   onPressed: () {
///     // ...
///   },
/// ),
/// ```
///
/// If you do, the button's size will be based on the default icon
/// size, not 72, which may produce unexpected layouts and clipping
/// issues.
///
/// ### Adding a filled background
///
/// Icon buttons don't support specifying a background color or other
/// background decoration because typically the icon is just displayed
/// on top of the parent widget's background. Icon buttons that appear
/// in [AppBar.actions] are an example of this.
///
/// It's easy enough to create an icon button with a filled background
/// using the [Ink] widget. The [Ink] widget renders a decoration on
/// the underlying [Material] along with the splash and highlight
/// [InkResponse] contributed by descendant widgets.
///
/// {@tool dartpad}
/// In this sample the icon button's background color is defined with an [Ink]
/// widget whose child is an [IconButton]. The icon button's filled background
/// is a light shade of blue, it's a filled circle, and it's as big as the
/// button is.
///
/// ![](https://flutter.github.io/assets-for-api-docs/assets/material/icon_button_background.png)
///
/// ** See code in examples/api/lib/material/icon_button/icon_button.1.dart **
/// {@end-tool}
///
/// Material Design 3 introduced new types (standard and contained) of [IconButton]s.
/// The default [IconButton] is the standard type. To create a filled icon button,
/// use [IconButton.filled]; to create a filled tonal icon button, use [IconButton.filledTonal];
/// to create a outlined icon button, use [IconButton.outlined].
///
/// Material Design 3 also treats [IconButton]s as toggle buttons. In order
/// to not break existing apps, the toggle feature can be optionally controlled
/// by the [isSelected] property.
///
/// If [isSelected] is null it will behave as a normal button. If [isSelected] is not
/// null then it will behave as a toggle button. If [isSelected] is true then it will
/// show [selectedIcon], if it false it will show the normal [icon].
///
/// In Material Design 3, both [IconTheme] and [IconButtonTheme] are used to override the default style
/// of [IconButton]. If both themes exist, the [IconButtonTheme] will override [IconTheme] no matter
/// which is closer to the [IconButton]. Each [IconButton]'s property is resolved by the order of
/// precedence: widget property, [IconButtonTheme] property, [IconTheme] property and
/// internal default property value.
///
/// In Material Design 3, the [IconButton.visualDensity] defaults to [VisualDensity.standard]
/// for all platforms; otherwise the button will have a rounded rectangle shape if
/// the [IconButton.visualDensity] is set to [VisualDensity.compact]. Users can
/// customize it by using [IconButtonTheme], [IconButton.style] or [IconButton.visualDensity].
///
/// {@tool dartpad}
/// This sample shows creation of [IconButton] widgets for standard, filled,
/// filled tonal and outlined types, as described in: https://m3.material.io/components/icon-buttons/overview
///
/// ** See code in examples/api/lib/material/icon_button/icon_button.2.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This sample shows creation of [IconButton] widgets with toggle feature for
/// standard, filled, filled tonal and outlined types, as described
/// in: https://m3.material.io/components/icon-buttons/overview
///
/// ** See code in examples/api/lib/material/icon_button/icon_button.3.dart **
/// {@end-tool}
///
/// See also:
///
///  * [Icons], the library of Material Icons.
///  * [BackButton], an icon button for a "back" affordance which adapts to the
///    current platform's conventions.
///  * [CloseButton], an icon button for closing pages.
///  * [AppBar], to show a toolbar at the top of an application.
///  * [TextButton], [ElevatedButton], [OutlinedButton], for buttons with text labels and an optional icon.
///  * [InkResponse] and [InkWell], for the ink splash effect itself.
class IconButton extends StatelessWidget {
  /// Creates an icon button.
  ///
  /// Icon buttons are commonly used in the [AppBar.actions] field, but they can
  /// be used in many other places as well.
  ///
  /// Requires one of its ancestors to be a [Material] widget. This requirement
  /// no longer exists if [ThemeData.useMaterial3] is set to true.
  ///
  /// The [icon] argument must be specified, and is typically either an [Icon]
  /// or an [ImageIcon].
  const IconButton({
    super.key,
    this.iconSize,
    this.visualDensity,
    this.padding,
    this.alignment,
    this.splashRadius,
    this.color,
    this.focusColor,
    this.hoverColor,
    this.highlightColor,
    this.splashColor,
    this.disabledColor,
    required this.onPressed,
    this.mouseCursor,
    this.focusNode,
    this.autofocus = false,
    this.tooltip,
    this.enableFeedback,
    this.constraints,
    this.style,
    this.isSelected,
    this.selectedIcon,
    required this.icon,
  }) : assert(splashRadius == null || splashRadius > 0),
       _variant = _IconButtonVariant.standard;

  /// Create a filled variant of IconButton.
  ///
  /// Filled icon buttons have higher visual impact and should be used for
  /// high emphasis actions, such as turning off a microphone or camera.
  const IconButton.filled({
    super.key,
    this.iconSize,
    this.visualDensity,
    this.padding,
    this.alignment,
    this.splashRadius,
    this.color,
    this.focusColor,
    this.hoverColor,
    this.highlightColor,
    this.splashColor,
    this.disabledColor,
    required this.onPressed,
    this.mouseCursor,
    this.focusNode,
    this.autofocus = false,
    this.tooltip,
    this.enableFeedback,
    this.constraints,
    this.style,
    this.isSelected,
    this.selectedIcon,
    required this.icon,
  }) : assert(splashRadius == null || splashRadius > 0),
       _variant = _IconButtonVariant.filled;

  /// Create a filled tonal variant of IconButton.
  ///
  /// Filled tonal icon buttons are a middle ground between filled and outlined
  /// icon buttons. They’re useful in contexts where the button requires slightly
  /// more emphasis than an outline would give, such as a secondary action paired
  /// with a high emphasis action.
  const IconButton.filledTonal({
    super.key,
    this.iconSize,
    this.visualDensity,
    this.padding,
    this.alignment,
    this.splashRadius,
    this.color,
    this.focusColor,
    this.hoverColor,
    this.highlightColor,
    this.splashColor,
    this.disabledColor,
    required this.onPressed,
    this.mouseCursor,
    this.focusNode,
    this.autofocus = false,
    this.tooltip,
    this.enableFeedback,
    this.constraints,
    this.style,
    this.isSelected,
    this.selectedIcon,
    required this.icon,
  }) : assert(splashRadius == null || splashRadius > 0),
       _variant = _IconButtonVariant.filledTonal;

  /// Create an outlined variant of IconButton.
  ///
  /// Outlined icon buttons are medium-emphasis buttons. They’re useful when an
  /// icon button needs more emphasis than a standard icon button but less than
  /// a filled or filled tonal icon button.
  const IconButton.outlined({
    super.key,
    this.iconSize,
    this.visualDensity,
    this.padding,
    this.alignment,
    this.splashRadius,
    this.color,
    this.focusColor,
    this.hoverColor,
    this.highlightColor,
    this.splashColor,
    this.disabledColor,
    required this.onPressed,
    this.mouseCursor,
    this.focusNode,
    this.autofocus = false,
    this.tooltip,
    this.enableFeedback,
    this.constraints,
    this.style,
    this.isSelected,
    this.selectedIcon,
    required this.icon,
  }) : assert(splashRadius == null || splashRadius > 0),
       _variant = _IconButtonVariant.outlined;

  /// The size of the icon inside the button.
  ///
  /// If null, uses [IconThemeData.size]. If it is also null, the default size
  /// is 24.0.
  ///
  /// The size given here is passed down to the widget in the [icon] property
  /// via an [IconTheme]. Setting the size here instead of in, for example, the
  /// [Icon.size] property allows the [IconButton] to size the splash area to
  /// fit the [Icon]. If you were to set the size of the [Icon] using
  /// [Icon.size] instead, then the [IconButton] would default to 24.0 and then
  /// the [Icon] itself would likely get clipped.
  ///
  /// This property is only used when [icon] is or contains an [Icon] widget. It will be
  /// ignored if other widgets are used, such as an [Image].
  ///
  /// If [ThemeData.useMaterial3] is set to true and this is null, the size of the
  /// [IconButton] would default to 24.0. The size given here is passed down to the
  /// [ButtonStyle.iconSize] property.
  final double? iconSize;

  /// Defines how compact the icon button's layout will be.
  ///
  /// {@macro flutter.material.themedata.visualDensity}
  ///
  /// This property can be null. If null, it defaults to [VisualDensity.standard]
  /// in Material Design 3 to make sure the button will be circular on all platforms.
  ///
  /// See also:
  ///
  ///  * [ThemeData.visualDensity], which specifies the [visualDensity] for all
  ///    widgets within a [Theme].
  final VisualDensity? visualDensity;

  /// The padding around the button's icon. The entire padded icon will react
  /// to input gestures.
  ///
  /// This property can be null. If null, it defaults to 8.0 padding on all sides.
  final EdgeInsetsGeometry? padding;

  /// Defines how the icon is positioned within the IconButton.
  ///
  /// This property can be null. If null, it defaults to [Alignment.center].
  ///
  /// See also:
  ///
  ///  * [Alignment], a class with convenient constants typically used to
  ///    specify an [AlignmentGeometry].
  ///  * [AlignmentDirectional], like [Alignment] for specifying alignments
  ///    relative to text direction.
  final AlignmentGeometry? alignment;

  /// The splash radius.
  ///
  /// If [ThemeData.useMaterial3] is set to true, this will not be used.
  ///
  /// If null, default splash radius of [Material.defaultSplashRadius] is used.
  final double? splashRadius;

  /// The icon to display inside the button.
  ///
  /// The [Icon.size] and [Icon.color] of the icon is configured automatically
  /// based on the [iconSize] and [color] properties of _this_ widget using an
  /// [IconTheme] and therefore should not be explicitly given in the icon
  /// widget.
  ///
  /// See [Icon], [ImageIcon].
  final Widget icon;

  /// The color for the button when it has the input focus.
  ///
  /// If [ThemeData.useMaterial3] is set to true, this [focusColor] will be mapped
  /// to be the [ButtonStyle.overlayColor] in focused state, which paints on top of
  /// the button, as an overlay. Therefore, using a color with some transparency
  /// is recommended. For example, one could customize the [focusColor] below:
  ///
  /// ```dart
  /// IconButton(
  ///   focusColor: Colors.orange.withOpacity(0.3),
  ///   icon: const Icon(Icons.sunny),
  ///   onPressed: () {
  ///     // ...
  ///   },
  /// )
  /// ```
  ///
  /// Defaults to [ThemeData.focusColor] of the ambient theme.
  final Color? focusColor;

  /// The color for the button when a pointer is hovering over it.
  ///
  /// If [ThemeData.useMaterial3] is set to true, this [hoverColor] will be mapped
  /// to be the [ButtonStyle.overlayColor] in hovered state, which paints on top of
  /// the button, as an overlay. Therefore, using a color with some transparency
  /// is recommended. For example, one could customize the [hoverColor] below:
  ///
  /// ```dart
  /// IconButton(
  ///   hoverColor: Colors.orange.withOpacity(0.3),
  ///   icon: const Icon(Icons.ac_unit),
  ///   onPressed: () {
  ///     // ...
  ///   },
  /// )
  /// ```
  ///
  /// Defaults to [ThemeData.hoverColor] of the ambient theme.
  final Color? hoverColor;

  /// The color to use for the icon inside the button, if the icon is enabled.
  /// Defaults to leaving this up to the [icon] widget.
  ///
  /// The icon is enabled if [onPressed] is not null.
  ///
  /// ```dart
  /// IconButton(
  ///   color: Colors.blue,
  ///   icon: const Icon(Icons.sunny_snowing),
  ///   onPressed: () {
  ///     // ...
  ///   },
  /// )
  /// ```
  final Color? color;

  /// The primary color of the button when the button is in the down (pressed) state.
  /// The splash is represented as a circular overlay that appears above the
  /// [highlightColor] overlay. The splash overlay has a center point that matches
  /// the hit point of the user touch event. The splash overlay will expand to
  /// fill the button area if the touch is held for long enough time. If the splash
  /// color has transparency then the highlight and button color will show through.
  ///
  /// If [ThemeData.useMaterial3] is set to true, this will not be used. Use
  /// [highlightColor] instead to show the overlay color of the button when the button
  /// is in the pressed state.
  ///
  /// Defaults to the Theme's splash color, [ThemeData.splashColor].
  final Color? splashColor;

  /// The secondary color of the button when the button is in the down (pressed)
  /// state. The highlight color is represented as a solid color that is overlaid over the
  /// button color (if any). If the highlight color has transparency, the button color
  /// will show through. The highlight fades in quickly as the button is held down.
  ///
  /// If [ThemeData.useMaterial3] is set to true, this [highlightColor] will be mapped
  /// to be the [ButtonStyle.overlayColor] in pressed state, which paints on top
  /// of the button, as an overlay. Therefore, using a color with some transparency
  /// is recommended. For example, one could customize the [highlightColor] below:
  ///
  /// ```dart
  /// IconButton(
  ///   highlightColor: Colors.orange.withOpacity(0.3),
  ///   icon: const Icon(Icons.question_mark),
  ///   onPressed: () {
  ///     // ...
  ///   },
  /// )
  /// ```
  ///
  /// Defaults to the Theme's highlight color, [ThemeData.highlightColor].
  final Color? highlightColor;

  /// The color to use for the icon inside the button, if the icon is disabled.
  /// Defaults to the [ThemeData.disabledColor] of the current [Theme].
  ///
  /// The icon is disabled if [onPressed] is null.
  final Color? disabledColor;

  /// The callback that is called when the button is tapped or otherwise activated.
  ///
  /// If this is set to null, the button will be disabled.
  final VoidCallback? onPressed;

  /// {@macro flutter.material.RawMaterialButton.mouseCursor}
  ///
  /// If set to null, will default to
  /// - [SystemMouseCursors.basic], if [onPressed] is null
  /// - [SystemMouseCursors.click], otherwise
  final MouseCursor? mouseCursor;

  /// {@macro flutter.widgets.Focus.focusNode}
  final FocusNode? focusNode;

  /// {@macro flutter.widgets.Focus.autofocus}
  final bool autofocus;

  /// Text that describes the action that will occur when the button is pressed.
  ///
  /// This text is displayed when the user long-presses on the button and is
  /// used for accessibility.
  final String? tooltip;

  /// Whether detected gestures should provide acoustic and/or haptic feedback.
  ///
  /// For example, on Android a tap will produce a clicking sound and a
  /// long-press will produce a short vibration, when feedback is enabled.
  ///
  /// See also:
  ///
  ///  * [Feedback] for providing platform-specific feedback to certain actions.
  final bool? enableFeedback;

  /// Optional size constraints for the button.
  ///
  /// When unspecified, defaults to:
  /// ```dart
  /// const BoxConstraints(
  ///   minWidth: kMinInteractiveDimension,
  ///   minHeight: kMinInteractiveDimension,
  /// )
  /// ```
  /// where [kMinInteractiveDimension] is 48.0, and then with visual density
  /// applied.
  ///
  /// The default constraints ensure that the button is accessible.
  /// Specifying this parameter enables creation of buttons smaller than
  /// the minimum size, but it is not recommended.
  ///
  /// The visual density uses the [visualDensity] parameter if specified,
  /// and `Theme.of(context).visualDensity` otherwise.
  final BoxConstraints? constraints;

  /// Customizes this button's appearance.
  ///
  /// Non-null properties of this style override the corresponding
  /// properties in [_IconButtonM3.themeStyleOf] and [_IconButtonM3.defaultStyleOf].
  /// [WidgetStateProperty]s that resolve to non-null values will similarly
  /// override the corresponding [WidgetStateProperty]s in [_IconButtonM3.themeStyleOf]
  /// and [_IconButtonM3.defaultStyleOf].
  ///
  /// The [style] is only used for Material 3 [IconButton]. If [ThemeData.useMaterial3]
  /// is set to true, [style] is preferred for icon button customization, and any
  /// parameters defined in [style] will override the same parameters in [IconButton].
  ///
  /// For example, if [IconButton]'s [visualDensity] is set to [VisualDensity.standard]
  /// and [style]'s [visualDensity] is set to [VisualDensity.compact],
  /// the icon button will have [VisualDensity.compact] to define the button's layout.
  ///
  /// Null by default.
  final ButtonStyle? style;

  /// The optional selection state of the icon button.
  ///
  /// If this property is null, the button will behave as a normal push button,
  /// otherwise, the button will toggle between showing [icon] and [selectedIcon]
  /// based on the value of [isSelected]. If true, it will show [selectedIcon],
  /// if false it will show [icon].
  ///
  /// This property is only used if [ThemeData.useMaterial3] is true.
  final bool? isSelected;

  /// The icon to display inside the button when [isSelected] is true. This property
  /// can be null. The original [icon] will be used for both selected and unselected
  /// status if it is null.
  ///
  /// The [Icon.size] and [Icon.color] of the icon is configured automatically
  /// based on the [iconSize] and [color] properties using an [IconTheme] and
  /// therefore should not be explicitly configured in the icon widget.
  ///
  /// This property is only used if [ThemeData.useMaterial3] is true.
  ///
  /// See also:
  ///
  /// * [Icon], for icons based on glyphs from fonts instead of images.
  /// * [ImageIcon], for showing icons from [AssetImage]s or other [ImageProvider]s.
  final Widget? selectedIcon;

  final _IconButtonVariant _variant;

  /// A static convenience method that constructs an icon button
  /// [ButtonStyle] given simple values. This method is only used for Material 3.
  ///
  /// The [foregroundColor] color is used to create a [WidgetStateProperty]
  /// [ButtonStyle.foregroundColor] value. Specify a value for [foregroundColor]
  /// to specify the color of the button's icons. The [hoverColor], [focusColor]
  /// and [highlightColor] colors are used to indicate the hover, focus,
  /// and pressed states if [overlayColor] isn't specified.
  ///
  /// If [overlayColor] is specified and its value is [Colors.transparent]
  /// then the pressed/focused/hovered highlights are effectively defeated.
  /// Otherwise a [WidgetStateProperty] with the same opacities as the
  /// default is created.
  ///
  /// Use [backgroundColor] for the button's background fill color. Use [disabledForegroundColor]
  /// and [disabledBackgroundColor] to specify the button's disabled icon and fill color.
  ///
  /// Similarly, the [enabledMouseCursor] and [disabledMouseCursor]
  /// parameters are used to construct [ButtonStyle].mouseCursor.
  ///
  /// All of the other parameters are either used directly or used to
  /// create a [WidgetStateProperty] with a single value for all
  /// states.
  ///
  /// All parameters default to null, by default this method returns
  /// a [ButtonStyle] that doesn't override anything.
  ///
  /// For example, to override the default icon color for a
  /// [IconButton], as well as its overlay color, with all of the
  /// standard opacity adjustments for the pressed, focused, and
  /// hovered states, one could write:
  ///
  /// ```dart
  /// IconButton(
  ///   icon: const Icon(Icons.pets),
  ///   style: IconButton.styleFrom(foregroundColor: Colors.green),
  ///   onPressed: () {
  ///     // ...
  ///   },
  /// ),
  /// ```
  static ButtonStyle styleFrom({
    Color? foregroundColor,
    Color? backgroundColor,
    Color? disabledForegroundColor,
    Color? disabledBackgroundColor,
    Color? focusColor,
    Color? hoverColor,
    Color? highlightColor,
    Color? shadowColor,
    Color? surfaceTintColor,
    Color? overlayColor,
    double? elevation,
    Size? minimumSize,
    Size? fixedSize,
    Size? maximumSize,
    double? iconSize,
    BorderSide? side,
    OutlinedBorder? shape,
    EdgeInsetsGeometry? padding,
    MouseCursor? enabledMouseCursor,
    MouseCursor? disabledMouseCursor,
    VisualDensity? visualDensity,
    MaterialTapTargetSize? tapTargetSize,
    Duration? animationDuration,
    bool? enableFeedback,
    AlignmentGeometry? alignment,
    InteractiveInkFeatureFactory? splashFactory,
  }) {
    final Color? overlayFallback = overlayColor ?? foregroundColor;
    WidgetStateProperty<Color?>? overlayColorProp;
    if ((hoverColor ?? focusColor ?? highlightColor ?? overlayFallback) != null) {
      overlayColorProp = switch (overlayColor) {
        Color(a: 0.0) => WidgetStatePropertyAll<Color>(overlayColor),
        _ => WidgetStateProperty<Color?>.fromMap(<WidgetState, Color?>{
          WidgetState.pressed: highlightColor ?? overlayFallback?.withOpacity(0.1),
          WidgetState.hovered: hoverColor ?? overlayFallback?.withOpacity(0.08),
          WidgetState.focused: focusColor ?? overlayFallback?.withOpacity(0.1),
        }),
      };
    }

    return ButtonStyle(
      backgroundColor: ButtonStyleButton.defaultColor(backgroundColor, disabledBackgroundColor),
      foregroundColor: ButtonStyleButton.defaultColor(foregroundColor, disabledForegroundColor),
      overlayColor: overlayColorProp,
      shadowColor: ButtonStyleButton.allOrNull<Color>(shadowColor),
      surfaceTintColor: ButtonStyleButton.allOrNull<Color>(surfaceTintColor),
      elevation: ButtonStyleButton.allOrNull<double>(elevation),
      padding: ButtonStyleButton.allOrNull<EdgeInsetsGeometry>(padding),
      minimumSize: ButtonStyleButton.allOrNull<Size>(minimumSize),
      fixedSize: ButtonStyleButton.allOrNull<Size>(fixedSize),
      maximumSize: ButtonStyleButton.allOrNull<Size>(maximumSize),
      iconSize: ButtonStyleButton.allOrNull<double>(iconSize),
      side: ButtonStyleButton.allOrNull<BorderSide>(side),
      shape: ButtonStyleButton.allOrNull<OutlinedBorder>(shape),
      mouseCursor: WidgetStateProperty<MouseCursor?>.fromMap(<WidgetStatesConstraint, MouseCursor?>{
        WidgetState.disabled: disabledMouseCursor,
        WidgetState.any: enabledMouseCursor,
      }),
      visualDensity: visualDensity,
      tapTargetSize: tapTargetSize,
      animationDuration: animationDuration,
      enableFeedback: enableFeedback,
      alignment: alignment,
      splashFactory: splashFactory,
    );
  }

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);

    if (theme.useMaterial3) {
      final Size? minSize =
          constraints == null ? null : Size(constraints!.minWidth, constraints!.minHeight);
      final Size? maxSize =
          constraints == null ? null : Size(constraints!.maxWidth, constraints!.maxHeight);

      ButtonStyle adjustedStyle = styleFrom(
        visualDensity: visualDensity,
        foregroundColor: color,
        disabledForegroundColor: disabledColor,
        focusColor: focusColor,
        hoverColor: hoverColor,
        highlightColor: highlightColor,
        padding: padding,
        minimumSize: minSize,
        maximumSize: maxSize,
        iconSize: iconSize,
        alignment: alignment,
        enabledMouseCursor: mouseCursor,
        disabledMouseCursor: mouseCursor,
        enableFeedback: enableFeedback,
      );
      if (style != null) {
        adjustedStyle = style!.merge(adjustedStyle);
      }

      Widget effectiveIcon = icon;
      if ((isSelected ?? false) && selectedIcon != null) {
        effectiveIcon = selectedIcon!;
      }

      return _SelectableIconButton(
        style: adjustedStyle,
        onPressed: onPressed,
        autofocus: autofocus,
        focusNode: focusNode,
        isSelected: isSelected,
        variant: _variant,
        tooltip: tooltip,
        child: effectiveIcon,
      );
    }

    assert(debugCheckHasMaterial(context));

    Color? currentColor;
    if (onPressed != null) {
      currentColor = color;
    } else {
      currentColor = disabledColor ?? theme.disabledColor;
    }

    final VisualDensity effectiveVisualDensity = visualDensity ?? theme.visualDensity;

    final BoxConstraints unadjustedConstraints =
        constraints ?? const BoxConstraints(minWidth: _kMinButtonSize, minHeight: _kMinButtonSize);
    final BoxConstraints adjustedConstraints = effectiveVisualDensity.effectiveConstraints(
      unadjustedConstraints,
    );
    final double effectiveIconSize = iconSize ?? IconTheme.of(context).size ?? 24.0;
    final EdgeInsetsGeometry effectivePadding = padding ?? const EdgeInsets.all(8.0);
    final AlignmentGeometry effectiveAlignment = alignment ?? Alignment.center;
    final bool effectiveEnableFeedback = enableFeedback ?? true;

    Widget result = ConstrainedBox(
      constraints: adjustedConstraints,
      child: Padding(
        padding: effectivePadding,
        child: SizedBox(
          height: effectiveIconSize,
          width: effectiveIconSize,
          child: Align(
            alignment: effectiveAlignment,
            child: IconTheme.merge(
              data: IconThemeData(size: effectiveIconSize, color: currentColor),
              child: icon,
            ),
          ),
        ),
      ),
    );

    result = InkResponse(
      focusNode: focusNode,
      autofocus: autofocus,
      canRequestFocus: onPressed != null,
      onTap: onPressed,
      mouseCursor:
          mouseCursor ?? (onPressed == null ? SystemMouseCursors.basic : SystemMouseCursors.click),
      enableFeedback: effectiveEnableFeedback,
      focusColor: focusColor ?? theme.focusColor,
      hoverColor: hoverColor ?? theme.hoverColor,
      highlightColor: highlightColor ?? theme.highlightColor,
      splashColor: splashColor ?? theme.splashColor,
      radius:
          splashRadius ??
          math.max(
            Material.defaultSplashRadius,
            (effectiveIconSize + math.min(effectivePadding.horizontal, effectivePadding.vertical)) *
                0.7,
            // x 0.5 for diameter -> radius and + 40% overflow derived from other Material apps.
          ),
      child: result,
    );

    if (tooltip != null) {
      result = Tooltip(message: tooltip, child: result);
    }

    return Semantics(button: true, enabled: onPressed != null, child: result);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(StringProperty('tooltip', tooltip, defaultValue: null, quoted: false));
    properties.add(ObjectFlagProperty<VoidCallback>('onPressed', onPressed, ifNull: 'disabled'));
    properties.add(ColorProperty('color', color, defaultValue: null));
    properties.add(ColorProperty('disabledColor', disabledColor, defaultValue: null));
    properties.add(ColorProperty('focusColor', focusColor, defaultValue: null));
    properties.add(ColorProperty('hoverColor', hoverColor, defaultValue: null));
    properties.add(ColorProperty('highlightColor', highlightColor, defaultValue: null));
    properties.add(ColorProperty('splashColor', splashColor, defaultValue: null));
    properties.add(DiagnosticsProperty<EdgeInsetsGeometry>('padding', padding, defaultValue: null));
    properties.add(DiagnosticsProperty<FocusNode>('focusNode', focusNode, defaultValue: null));
  }
}

class _SelectableIconButton extends StatefulWidget {
  const _SelectableIconButton({
    this.isSelected,
    this.style,
    this.focusNode,
    required this.variant,
    required this.autofocus,
    required this.onPressed,
    this.tooltip,
    required this.child,
  });

  final bool? isSelected;
  final ButtonStyle? style;
  final FocusNode? focusNode;
  final _IconButtonVariant variant;
  final bool autofocus;
  final VoidCallback? onPressed;
  final String? tooltip;
  final Widget child;

  @override
  State<_SelectableIconButton> createState() => _SelectableIconButtonState();
}

class _SelectableIconButtonState extends State<_SelectableIconButton> {
  late final MaterialStatesController statesController;

  @override
  void initState() {
    super.initState();
    if (widget.isSelected == null) {
      statesController = MaterialStatesController();
    } else {
      statesController = MaterialStatesController(<MaterialState>{
        if (widget.isSelected!) MaterialState.selected,
      });
    }
  }

  @override
  void didUpdateWidget(_SelectableIconButton oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.isSelected == null) {
      if (statesController.value.contains(MaterialState.selected)) {
        statesController.update(MaterialState.selected, false);
      }
      return;
    }
    if (widget.isSelected != oldWidget.isSelected) {
      statesController.update(MaterialState.selected, widget.isSelected!);
    }
  }

  @override
  Widget build(BuildContext context) {
    final bool toggleable = widget.isSelected != null;

    return _IconButtonM3(
      statesController: statesController,
      style: widget.style,
      autofocus: widget.autofocus,
      focusNode: widget.focusNode,
      onPressed: widget.onPressed,
      variant: widget.variant,
      toggleable: toggleable,
      tooltip: widget.tooltip,
      child: Semantics(selected: widget.isSelected, child: widget.child),
    );
  }

  @override
  void dispose() {
    statesController.dispose();
    super.dispose();
  }
}

class _IconButtonM3 extends ButtonStyleButton {
  const _IconButtonM3({
    required super.onPressed,
    super.style,
    super.focusNode,
    super.autofocus = false,
    super.statesController,
    required this.variant,
    required this.toggleable,
    super.tooltip,
    required Widget super.child,
  }) : super(onLongPress: null, onHover: null, onFocusChange: null, clipBehavior: Clip.none);

  final _IconButtonVariant variant;
  final bool toggleable;

  /// ## Material 3 defaults
  ///
  /// If [ThemeData.useMaterial3] is set to true the following defaults will
  /// be used:
  ///
  /// * `textStyle` - null
  /// * `backgroundColor` - transparent
  /// * `foregroundColor`
  ///   * disabled - Theme.colorScheme.onSurface(0.38)
  ///   * selected - Theme.colorScheme.primary
  ///   * others - Theme.colorScheme.onSurfaceVariant
  /// * `overlayColor`
  ///   * selected
  ///      * hovered - Theme.colorScheme.primary(0.08)
  ///      * focused or pressed - Theme.colorScheme.primary(0.1)
  ///   * hovered - Theme.colorScheme.onSurfaceVariant(0.08)
  ///   * pressed or focused - Theme.colorScheme.onSurfaceVariant(0.1)
  ///   * others - null
  /// * `shadowColor` - null
  /// * `surfaceTintColor` - null
  /// * `elevation` - 0
  /// * `padding` - all(8)
  /// * `minimumSize` - Size(40, 40)
  /// * `fixedSize` - null
  /// * `maximumSize` - Size.infinite
  /// * `iconSize` - 24
  /// * `side` - null
  /// * `shape` - StadiumBorder()
  /// * `mouseCursor`
  ///   * disabled - SystemMouseCursors.basic
  ///   * others - SystemMouseCursors.click
  /// * `visualDensity` - VisualDensity.standard
  /// * `tapTargetSize` - theme.materialTapTargetSize
  /// * `animationDuration` - kThemeChangeDuration
  /// * `enableFeedback` - true
  /// * `alignment` - Alignment.center
  /// * `splashFactory` - Theme.splashFactory
  @override
  ButtonStyle defaultStyleOf(BuildContext context) {
    return switch (variant) {
      _IconButtonVariant.filled => _FilledIconButtonDefaultsM3(context, toggleable),
      _IconButtonVariant.filledTonal => _FilledTonalIconButtonDefaultsM3(context, toggleable),
      _IconButtonVariant.outlined => _OutlinedIconButtonDefaultsM3(context, toggleable),
      _IconButtonVariant.standard => _IconButtonDefaultsM3(context, toggleable),
    };
  }

  /// Returns the [IconButtonThemeData.style] of the closest [IconButtonTheme] ancestor.
  /// The color and icon size can also be configured by the [IconTheme] if the same property
  /// has a null value in [IconButtonTheme]. However, if any of the properties exist
  /// in both [IconButtonTheme] and [IconTheme], [IconTheme] will be overridden.
  @override
  ButtonStyle? themeStyleOf(BuildContext context) {
    final IconThemeData iconTheme = IconTheme.of(context);
    final bool isDefaultSize = iconTheme.size == const IconThemeData.fallback().size;
    final bool isDefaultColor = identical(iconTheme.color, switch (Theme.of(context).brightness) {
      Brightness.light => kDefaultIconDarkColor,
      Brightness.dark => kDefaultIconLightColor,
    });

    final ButtonStyle iconThemeStyle = IconButton.styleFrom(
      foregroundColor: isDefaultColor ? null : iconTheme.color,
      iconSize: isDefaultSize ? null : iconTheme.size,
    );

    return IconButtonTheme.of(context).style?.merge(iconThemeStyle) ?? iconThemeStyle;
  }
}

// BEGIN GENERATED TOKEN PROPERTIES - IconButton

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _IconButtonDefaultsM3 extends ButtonStyle {
  _IconButtonDefaultsM3(this.context, this.toggleable)
    : super(
        animationDuration: kThemeChangeDuration,
        enableFeedback: true,
        alignment: Alignment.center,
      );

  final BuildContext context;
  final bool toggleable;
  late final ColorScheme _colors = Theme.of(context).colorScheme;

  // No default text style

  @override
  MaterialStateProperty<Color?>? get backgroundColor =>
    const MaterialStatePropertyAll<Color?>(Colors.transparent);

  @override
  MaterialStateProperty<Color?>? get foregroundColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return _colors.onSurface.withOpacity(0.38);
      }
      if (states.contains(MaterialState.selected)) {
        return _colors.primary;
      }
      return _colors.onSurfaceVariant;
    });

 @override
  MaterialStateProperty<Color?>? get overlayColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        if (states.contains(MaterialState.pressed)) {
          return _colors.primary.withOpacity(0.1);
        }
        if (states.contains(MaterialState.hovered)) {
          return _colors.primary.withOpacity(0.08);
        }
        if (states.contains(MaterialState.focused)) {
          return _colors.primary.withOpacity(0.1);
        }
      }
      if (states.contains(MaterialState.pressed)) {
        return _colors.onSurfaceVariant.withOpacity(0.1);
      }
      if (states.contains(MaterialState.hovered)) {
        return _colors.onSurfaceVariant.withOpacity(0.08);
      }
      if (states.contains(MaterialState.focused)) {
        return _colors.onSurfaceVariant.withOpacity(0.1);
      }
      return Colors.transparent;
    });

  @override
  MaterialStateProperty<double>? get elevation =>
    const MaterialStatePropertyAll<double>(0.0);

  @override
  MaterialStateProperty<Color>? get shadowColor =>
    const MaterialStatePropertyAll<Color>(Colors.transparent);

  @override
  MaterialStateProperty<Color>? get surfaceTintColor =>
    const MaterialStatePropertyAll<Color>(Colors.transparent);

  @override
  MaterialStateProperty<EdgeInsetsGeometry>? get padding =>
    const MaterialStatePropertyAll<EdgeInsetsGeometry>(EdgeInsets.all(8.0));

  @override
  MaterialStateProperty<Size>? get minimumSize =>
    const MaterialStatePropertyAll<Size>(Size(40.0, 40.0));

  // No default fixedSize

  @override
  MaterialStateProperty<Size>? get maximumSize =>
    const MaterialStatePropertyAll<Size>(Size.infinite);

  @override
  MaterialStateProperty<double>? get iconSize =>
    const MaterialStatePropertyAll<double>(24.0);

  @override
  MaterialStateProperty<BorderSide?>? get side => null;

  @override
  MaterialStateProperty<OutlinedBorder>? get shape =>
    const MaterialStatePropertyAll<OutlinedBorder>(StadiumBorder());

  @override
  MaterialStateProperty<MouseCursor?>? get mouseCursor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return SystemMouseCursors.basic;
      }
      return SystemMouseCursors.click;
    });

  @override
  VisualDensity? get visualDensity => VisualDensity.standard;

  @override
  MaterialTapTargetSize? get tapTargetSize => Theme.of(context).materialTapTargetSize;

  @override
  InteractiveInkFeatureFactory? get splashFactory => Theme.of(context).splashFactory;
}
// dart format on

// END GENERATED TOKEN PROPERTIES - IconButton

// BEGIN GENERATED TOKEN PROPERTIES - FilledIconButton

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _FilledIconButtonDefaultsM3 extends ButtonStyle {
  _FilledIconButtonDefaultsM3(this.context, this.toggleable)
    : super(
        animationDuration: kThemeChangeDuration,
        enableFeedback: true,
        alignment: Alignment.center,
      );

  final BuildContext context;
  final bool toggleable;
  late final ColorScheme _colors = Theme.of(context).colorScheme;

  // No default text style

  @override
  MaterialStateProperty<Color?>? get backgroundColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return _colors.onSurface.withOpacity(0.12);
      }
      if (states.contains(MaterialState.selected)) {
        return _colors.primary;
      }
      if (toggleable) { // toggleable but unselected case
        return _colors.surfaceContainerHighest;
      }
      return _colors.primary;
    });

  @override
  MaterialStateProperty<Color?>? get foregroundColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return _colors.onSurface.withOpacity(0.38);
      }
      if (states.contains(MaterialState.selected)) {
        return _colors.onPrimary;
      }
      if (toggleable) { // toggleable but unselected case
        return _colors.primary;
      }
      return _colors.onPrimary;
    });

 @override
  MaterialStateProperty<Color?>? get overlayColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        if (states.contains(MaterialState.pressed)) {
          return _colors.onPrimary.withOpacity(0.1);
        }
        if (states.contains(MaterialState.hovered)) {
          return _colors.onPrimary.withOpacity(0.08);
        }
        if (states.contains(MaterialState.focused)) {
          return _colors.onPrimary.withOpacity(0.1);
        }
      }
      if (toggleable) { // toggleable but unselected case
        if (states.contains(MaterialState.pressed)) {
          return _colors.primary.withOpacity(0.1);
        }
        if (states.contains(MaterialState.hovered)) {
          return _colors.primary.withOpacity(0.08);
        }
        if (states.contains(MaterialState.focused)) {
          return _colors.primary.withOpacity(0.1);
        }
      }
      if (states.contains(MaterialState.pressed)) {
        return _colors.onPrimary.withOpacity(0.1);
      }
      if (states.contains(MaterialState.hovered)) {
        return _colors.onPrimary.withOpacity(0.08);
      }
      if (states.contains(MaterialState.focused)) {
        return _colors.onPrimary.withOpacity(0.1);
      }
      return Colors.transparent;
    });

  @override
  MaterialStateProperty<double>? get elevation =>
    const MaterialStatePropertyAll<double>(0.0);

  @override
  MaterialStateProperty<Color>? get shadowColor =>
    const MaterialStatePropertyAll<Color>(Colors.transparent);

  @override
  MaterialStateProperty<Color>? get surfaceTintColor =>
    const MaterialStatePropertyAll<Color>(Colors.transparent);

  @override
  MaterialStateProperty<EdgeInsetsGeometry>? get padding =>
    const MaterialStatePropertyAll<EdgeInsetsGeometry>(EdgeInsets.all(8.0));

  @override
  MaterialStateProperty<Size>? get minimumSize =>
    const MaterialStatePropertyAll<Size>(Size(40.0, 40.0));

  // No default fixedSize

  @override
  MaterialStateProperty<Size>? get maximumSize =>
    const MaterialStatePropertyAll<Size>(Size.infinite);

  @override
  MaterialStateProperty<double>? get iconSize =>
    const MaterialStatePropertyAll<double>(24.0);

  @override
  MaterialStateProperty<BorderSide?>? get side => null;

  @override
  MaterialStateProperty<OutlinedBorder>? get shape =>
    const MaterialStatePropertyAll<OutlinedBorder>(StadiumBorder());

  @override
  MaterialStateProperty<MouseCursor?>? get mouseCursor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return SystemMouseCursors.basic;
      }
      return SystemMouseCursors.click;
    });

  @override
  VisualDensity? get visualDensity => VisualDensity.standard;

  @override
  MaterialTapTargetSize? get tapTargetSize => Theme.of(context).materialTapTargetSize;

  @override
  InteractiveInkFeatureFactory? get splashFactory => Theme.of(context).splashFactory;
}
// dart format on

// END GENERATED TOKEN PROPERTIES - FilledIconButton

// BEGIN GENERATED TOKEN PROPERTIES - FilledTonalIconButton

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _FilledTonalIconButtonDefaultsM3 extends ButtonStyle {
  _FilledTonalIconButtonDefaultsM3(this.context, this.toggleable)
    : super(
        animationDuration: kThemeChangeDuration,
        enableFeedback: true,
        alignment: Alignment.center,
      );

  final BuildContext context;
  final bool toggleable;
  late final ColorScheme _colors = Theme.of(context).colorScheme;

  // No default text style

  @override
  MaterialStateProperty<Color?>? get backgroundColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return _colors.onSurface.withOpacity(0.12);
      }
      if (states.contains(MaterialState.selected)) {
        return _colors.secondaryContainer;
      }
      if (toggleable) { // toggleable but unselected case
        return _colors.surfaceContainerHighest;
      }
      return _colors.secondaryContainer;
    });

  @override
  MaterialStateProperty<Color?>? get foregroundColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return _colors.onSurface.withOpacity(0.38);
      }
      if (states.contains(MaterialState.selected)) {
        return _colors.onSecondaryContainer;
      }
      if (toggleable) { // toggleable but unselected case
        return _colors.onSurfaceVariant;
      }
      return _colors.onSecondaryContainer;
    });

 @override
  MaterialStateProperty<Color?>? get overlayColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        if (states.contains(MaterialState.pressed)) {
          return _colors.onSecondaryContainer.withOpacity(0.1);
        }
        if (states.contains(MaterialState.hovered)) {
          return _colors.onSecondaryContainer.withOpacity(0.08);
        }
        if (states.contains(MaterialState.focused)) {
          return _colors.onSecondaryContainer.withOpacity(0.1);
        }
      }
      if (toggleable) { // toggleable but unselected case
        if (states.contains(MaterialState.pressed)) {
          return _colors.onSurfaceVariant.withOpacity(0.1);
        }
        if (states.contains(MaterialState.hovered)) {
          return _colors.onSurfaceVariant.withOpacity(0.08);
        }
        if (states.contains(MaterialState.focused)) {
          return _colors.onSurfaceVariant.withOpacity(0.1);
        }
      }
      if (states.contains(MaterialState.pressed)) {
        return _colors.onSecondaryContainer.withOpacity(0.1);
      }
      if (states.contains(MaterialState.hovered)) {
        return _colors.onSecondaryContainer.withOpacity(0.08);
      }
      if (states.contains(MaterialState.focused)) {
        return _colors.onSecondaryContainer.withOpacity(0.1);
      }
      return Colors.transparent;
    });

  @override
  MaterialStateProperty<double>? get elevation =>
    const MaterialStatePropertyAll<double>(0.0);

  @override
  MaterialStateProperty<Color>? get shadowColor =>
    const MaterialStatePropertyAll<Color>(Colors.transparent);

  @override
  MaterialStateProperty<Color>? get surfaceTintColor =>
    const MaterialStatePropertyAll<Color>(Colors.transparent);

  @override
  MaterialStateProperty<EdgeInsetsGeometry>? get padding =>
    const MaterialStatePropertyAll<EdgeInsetsGeometry>(EdgeInsets.all(8.0));

  @override
  MaterialStateProperty<Size>? get minimumSize =>
    const MaterialStatePropertyAll<Size>(Size(40.0, 40.0));

  // No default fixedSize

  @override
  MaterialStateProperty<Size>? get maximumSize =>
    const MaterialStatePropertyAll<Size>(Size.infinite);

  @override
  MaterialStateProperty<double>? get iconSize =>
    const MaterialStatePropertyAll<double>(24.0);

  @override
  MaterialStateProperty<BorderSide?>? get side => null;

  @override
  MaterialStateProperty<OutlinedBorder>? get shape =>
    const MaterialStatePropertyAll<OutlinedBorder>(StadiumBorder());

  @override
  MaterialStateProperty<MouseCursor?>? get mouseCursor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return SystemMouseCursors.basic;
      }
      return SystemMouseCursors.click;
    });

  @override
  VisualDensity? get visualDensity => VisualDensity.standard;

  @override
  MaterialTapTargetSize? get tapTargetSize => Theme.of(context).materialTapTargetSize;

  @override
  InteractiveInkFeatureFactory? get splashFactory => Theme.of(context).splashFactory;
}
// dart format on

// END GENERATED TOKEN PROPERTIES - FilledTonalIconButton

// BEGIN GENERATED TOKEN PROPERTIES - OutlinedIconButton

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _OutlinedIconButtonDefaultsM3 extends ButtonStyle {
  _OutlinedIconButtonDefaultsM3(this.context, this.toggleable)
    : super(
        animationDuration: kThemeChangeDuration,
        enableFeedback: true,
        alignment: Alignment.center,
      );

  final BuildContext context;
  final bool toggleable;
  late final ColorScheme _colors = Theme.of(context).colorScheme;

  // No default text style

  @override
  MaterialStateProperty<Color?>? get backgroundColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        if (states.contains(MaterialState.selected)) {
          return _colors.onSurface.withOpacity(0.12);
        }
        return Colors.transparent;
      }
      if (states.contains(MaterialState.selected)) {
        return _colors.inverseSurface;
      }
      return Colors.transparent;
    });

  @override
  MaterialStateProperty<Color?>? get foregroundColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return _colors.onSurface.withOpacity(0.38);
      }
      if (states.contains(MaterialState.selected)) {
        return _colors.onInverseSurface;
      }
      return _colors.onSurfaceVariant;
    });

 @override
  MaterialStateProperty<Color?>? get overlayColor =>    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        if (states.contains(MaterialState.pressed)) {
          return _colors.onInverseSurface.withOpacity(0.1);
        }
        if (states.contains(MaterialState.hovered)) {
          return _colors.onInverseSurface.withOpacity(0.08);
        }
        if (states.contains(MaterialState.focused)) {
          return _colors.onInverseSurface.withOpacity(0.08);
        }
      }
      if (states.contains(MaterialState.pressed)) {
        return _colors.onSurface.withOpacity(0.1);
      }
      if (states.contains(MaterialState.hovered)) {
        return _colors.onSurfaceVariant.withOpacity(0.08);
      }
      if (states.contains(MaterialState.focused)) {
        return _colors.onSurfaceVariant.withOpacity(0.08);
      }
      return Colors.transparent;
    });

  @override
  MaterialStateProperty<double>? get elevation =>
    const MaterialStatePropertyAll<double>(0.0);

  @override
  MaterialStateProperty<Color>? get shadowColor =>
    const MaterialStatePropertyAll<Color>(Colors.transparent);

  @override
  MaterialStateProperty<Color>? get surfaceTintColor =>
    const MaterialStatePropertyAll<Color>(Colors.transparent);

  @override
  MaterialStateProperty<EdgeInsetsGeometry>? get padding =>
    const MaterialStatePropertyAll<EdgeInsetsGeometry>(EdgeInsets.all(8.0));

  @override
  MaterialStateProperty<Size>? get minimumSize =>
    const MaterialStatePropertyAll<Size>(Size(40.0, 40.0));

  // No default fixedSize

  @override
  MaterialStateProperty<Size>? get maximumSize =>
    const MaterialStatePropertyAll<Size>(Size.infinite);

  @override
  MaterialStateProperty<double>? get iconSize =>
    const MaterialStatePropertyAll<double>(24.0);

  @override
  MaterialStateProperty<BorderSide?>? get side =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return null;
      } else {
        if (states.contains(MaterialState.disabled)) {
          return BorderSide(color: _colors.onSurface.withOpacity(0.12));
        }
        return BorderSide(color: _colors.outline);
      }
    });

  @override
  MaterialStateProperty<OutlinedBorder>? get shape =>
    const MaterialStatePropertyAll<OutlinedBorder>(StadiumBorder());

  @override
  MaterialStateProperty<MouseCursor?>? get mouseCursor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return SystemMouseCursors.basic;
      }
      return SystemMouseCursors.click;
    });

  @override
  VisualDensity? get visualDensity => VisualDensity.standard;

  @override
  MaterialTapTargetSize? get tapTargetSize => Theme.of(context).materialTapTargetSize;

  @override
  InteractiveInkFeatureFactory? get splashFactory => Theme.of(context).splashFactory;
}
// dart format on

// END GENERATED TOKEN PROPERTIES - OutlinedIconButton
