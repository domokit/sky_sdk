// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'filled_button.dart';
/// @docImport 'material.dart';
/// @docImport 'outlined_button.dart';
/// @docImport 'text_button.dart';
library;

import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'button_style.dart';
import 'button_style_button.dart';
import 'color_scheme.dart';
import 'colors.dart';
import 'constants.dart';
import 'elevated_button_theme.dart';
import 'ink_ripple.dart';
import 'ink_well.dart';
import 'material_state.dart';
import 'theme.dart';
import 'theme_data.dart';

/// A Material Design "elevated button".
///
/// Use elevated buttons to add dimension to otherwise mostly flat
/// layouts, e.g. in long busy lists of content, or in wide
/// spaces. Avoid using elevated buttons on already-elevated content
/// such as dialogs or cards.
///
/// An elevated button is a label [child] displayed on a [Material]
/// widget whose [Material.elevation] increases when the button is
/// pressed. The label's [Text] and [Icon] widgets are displayed in
/// [style]'s [ButtonStyle.foregroundColor] and the button's filled
/// background is the [ButtonStyle.backgroundColor].
///
/// The elevated button's default style is defined by
/// [defaultStyleOf]. The style of this elevated button can be
/// overridden with its [style] parameter. The style of all elevated
/// buttons in a subtree can be overridden with the
/// [ElevatedButtonTheme], and the style of all of the elevated
/// buttons in an app can be overridden with the [Theme]'s
/// [ThemeData.elevatedButtonTheme] property.
///
/// The static [styleFrom] method is a convenient way to create a
/// elevated button [ButtonStyle] from simple values.
///
/// If [onPressed] and [onLongPress] callbacks are null, then the
/// button will be disabled.
///
/// {@tool dartpad}
/// This sample produces an enabled and a disabled ElevatedButton.
///
/// ** See code in examples/api/lib/material/elevated_button/elevated_button.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [FilledButton], a filled button that doesn't elevate when pressed.
///  * [FilledButton.tonal], a filled button variant that uses a secondary fill color.
///  * [OutlinedButton], a button with an outlined border and no fill color.
///  * [TextButton], a button with no outline or fill color.
///  * <https://material.io/design/components/buttons.html>
///  * <https://m3.material.io/components/buttons>
class ElevatedButton extends ButtonStyleButton {
  /// Create an ElevatedButton.
  const ElevatedButton({
    super.key,
    required super.onPressed,
    super.onLongPress,
    super.onHover,
    super.onFocusChange,
    super.style,
    super.focusNode,
    super.autofocus = false,
    super.clipBehavior,
    super.statesController,
    required super.child,
  });

  /// Create an elevated button from a pair of widgets that serve as the button's
  /// [icon] and [label].
  ///
  /// The icon and label are arranged in a row and padded by 12 logical pixels
  /// at the start, and 16 at the end, with an 8 pixel gap in between.
  ///
  /// If [icon] is null, will create an [ElevatedButton] instead.
  ///
  /// {@macro flutter.material.ButtonStyleButton.iconAlignment}
  ///
  factory ElevatedButton.icon({
    Key? key,
    required VoidCallback? onPressed,
    VoidCallback? onLongPress,
    ValueChanged<bool>? onHover,
    ValueChanged<bool>? onFocusChange,
    ButtonStyle? style,
    FocusNode? focusNode,
    bool? autofocus,
    Clip? clipBehavior,
    MaterialStatesController? statesController,
    Widget? icon,
    required Widget label,
    IconAlignment? iconAlignment,
  }) {
    if (icon == null) {
      return ElevatedButton(
        key: key,
        onPressed: onPressed,
        onLongPress: onLongPress,
        onHover: onHover,
        onFocusChange: onFocusChange,
        style: style,
        focusNode: focusNode,
        autofocus: autofocus ?? false,
        clipBehavior: clipBehavior ?? Clip.none,
        statesController: statesController,
        child: label,
      );
    }
    return _ElevatedButtonWithIcon(
      key: key,
      onPressed: onPressed,
      onLongPress: onLongPress,
      onHover: onHover,
      onFocusChange: onFocusChange,
      style: style,
      focusNode: focusNode,
      autofocus: autofocus ?? false,
      clipBehavior: clipBehavior ?? Clip.none,
      statesController: statesController,
      icon: icon,
      label: label,
      iconAlignment: iconAlignment,
    );
  }

  /// A static convenience method that constructs an elevated button
  /// [ButtonStyle] given simple values.
  ///
  /// The [foregroundColor] and [disabledForegroundColor] colors are used
  /// to create a [WidgetStateProperty] [ButtonStyle.foregroundColor], and
  /// a derived [ButtonStyle.overlayColor] if [overlayColor] isn't specified.
  ///
  /// If [overlayColor] is specified and its value is [Colors.transparent]
  /// then the pressed/focused/hovered highlights are effectively defeated.
  /// Otherwise a [WidgetStateProperty] with the same opacities as the
  /// default is created.
  ///
  /// The [backgroundColor] and [disabledBackgroundColor] colors are
  /// used to create a [WidgetStateProperty] [ButtonStyle.backgroundColor].
  ///
  /// Similarly, the [enabledMouseCursor] and [disabledMouseCursor]
  /// parameters are used to construct [ButtonStyle.mouseCursor].
  ///
  /// The [iconColor], [disabledIconColor] are used to construct
  /// [ButtonStyle.iconColor] and [iconSize] is used to construct
  /// [ButtonStyle.iconSize].
  ///
  /// The button's elevations are defined relative to the [elevation]
  /// parameter. The disabled elevation is the same as the parameter
  /// value, [elevation] + 2 is used when the button is hovered
  /// or focused, and elevation + 6 is used when the button is pressed.
  ///
  /// All of the other parameters are either used directly or used to
  /// create a [WidgetStateProperty] with a single value for all
  /// states.
  ///
  /// All parameters default to null, by default this method returns
  /// a [ButtonStyle] that doesn't override anything.
  ///
  /// For example, to override the default text and icon colors for an
  /// [ElevatedButton], as well as its overlay color, with all of the
  /// standard opacity adjustments for the pressed, focused, and
  /// hovered states, one could write:
  ///
  /// ```dart
  /// ElevatedButton(
  ///   style: ElevatedButton.styleFrom(foregroundColor: Colors.green),
  ///   onPressed: () {
  ///     // ...
  ///   },
  ///   child: const Text('Jump'),
  /// ),
  /// ```
  ///
  /// And to change the fill color:
  ///
  /// ```dart
  /// ElevatedButton(
  ///   style: ElevatedButton.styleFrom(backgroundColor: Colors.green),
  ///   onPressed: () {
  ///     // ...
  ///   },
  ///   child: const Text('Meow'),
  /// ),
  /// ```
  ///
  static ButtonStyle styleFrom({
    Color? foregroundColor,
    Color? backgroundColor,
    Color? disabledForegroundColor,
    Color? disabledBackgroundColor,
    Color? shadowColor,
    Color? surfaceTintColor,
    Color? iconColor,
    double? iconSize,
    IconAlignment? iconAlignment,
    Color? disabledIconColor,
    Color? overlayColor,
    double? elevation,
    TextStyle? textStyle,
    EdgeInsetsGeometry? padding,
    Size? minimumSize,
    Size? fixedSize,
    Size? maximumSize,
    BorderSide? side,
    OutlinedBorder? shape,
    MouseCursor? enabledMouseCursor,
    MouseCursor? disabledMouseCursor,
    VisualDensity? visualDensity,
    MaterialTapTargetSize? tapTargetSize,
    Duration? animationDuration,
    bool? enableFeedback,
    AlignmentGeometry? alignment,
    InteractiveInkFeatureFactory? splashFactory,
    ButtonLayerBuilder? backgroundBuilder,
    ButtonLayerBuilder? foregroundBuilder,
  }) {
    final MaterialStateProperty<Color?>? overlayColorProp = switch ((
      foregroundColor,
      overlayColor,
    )) {
      (null, null) => null,
      (_, Color(a: 0.0)) => WidgetStatePropertyAll<Color?>(overlayColor),
      (_, final Color color) ||
      (final Color color, _) => WidgetStateProperty<Color?>.fromMap(<WidgetState, Color?>{
        WidgetState.pressed: color.withOpacity(0.1),
        WidgetState.hovered: color.withOpacity(0.08),
        WidgetState.focused: color.withOpacity(0.1),
      }),
    };

    WidgetStateProperty<double>? elevationValue;
    if (elevation != null) {
      elevationValue = WidgetStateProperty<double>.fromMap(<WidgetStatesConstraint, double>{
        WidgetState.disabled: 0,
        WidgetState.pressed: elevation + 6,
        WidgetState.hovered: elevation + 2,
        WidgetState.focused: elevation + 2,
        WidgetState.any: elevation,
      });
    }

    return ButtonStyle(
      textStyle: MaterialStatePropertyAll<TextStyle?>(textStyle),
      backgroundColor: ButtonStyleButton.defaultColor(backgroundColor, disabledBackgroundColor),
      foregroundColor: ButtonStyleButton.defaultColor(foregroundColor, disabledForegroundColor),
      overlayColor: overlayColorProp,
      shadowColor: ButtonStyleButton.allOrNull<Color>(shadowColor),
      surfaceTintColor: ButtonStyleButton.allOrNull<Color>(surfaceTintColor),
      iconColor: ButtonStyleButton.defaultColor(iconColor, disabledIconColor),
      iconSize: ButtonStyleButton.allOrNull<double>(iconSize),
      iconAlignment: iconAlignment,
      elevation: elevationValue,
      padding: ButtonStyleButton.allOrNull<EdgeInsetsGeometry>(padding),
      minimumSize: ButtonStyleButton.allOrNull<Size>(minimumSize),
      fixedSize: ButtonStyleButton.allOrNull<Size>(fixedSize),
      maximumSize: ButtonStyleButton.allOrNull<Size>(maximumSize),
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
      backgroundBuilder: backgroundBuilder,
      foregroundBuilder: foregroundBuilder,
    );
  }

  /// Defines the button's default appearance.
  ///
  /// The button [child]'s [Text] and [Icon] widgets are rendered with
  /// the [ButtonStyle]'s foreground color. The button's [InkWell] adds
  /// the style's overlay color when the button is focused, hovered
  /// or pressed. The button's background color becomes its [Material]
  /// color.
  ///
  /// All of the ButtonStyle's defaults appear below. In this list
  /// "Theme.foo" is shorthand for `Theme.of(context).foo`. Color
  /// scheme values like "onSurface(0.38)" are shorthand for
  /// `onSurface.withOpacity(0.38)`. [WidgetStateProperty] valued
  /// properties that are not followed by a sublist have the same
  /// value for all states, otherwise the values are as specified for
  /// each state, and "others" means all other states.
  ///
  /// {@template flutter.material.elevated_button.default_font_size}
  /// The "default font size" below refers to the font size specified in the
  /// [defaultStyleOf] method (or 14.0 if unspecified), scaled by the
  /// `MediaQuery.textScalerOf(context).scale` method. The names of the
  /// EdgeInsets constructors and `EdgeInsetsGeometry.lerp` have been abbreviated
  /// for readability.
  /// {@endtemplate}
  ///
  /// The color of the [ButtonStyle.textStyle] is not used, the
  /// [ButtonStyle.foregroundColor] color is used instead.
  ///
  /// ## Material 2 defaults
  ///
  /// * `textStyle` - Theme.textTheme.button
  /// * `backgroundColor`
  ///   * disabled - Theme.colorScheme.onSurface(0.12)
  ///   * others - Theme.colorScheme.primary
  /// * `foregroundColor`
  ///   * disabled - Theme.colorScheme.onSurface(0.38)
  ///   * others - Theme.colorScheme.onPrimary
  /// * `overlayColor`
  ///   * hovered - Theme.colorScheme.onPrimary(0.08)
  ///   * focused or pressed - Theme.colorScheme.onPrimary(0.12)
  /// * `shadowColor` - Theme.shadowColor
  /// * `elevation`
  ///   * disabled - 0
  ///   * default - 2
  ///   * hovered or focused - 4
  ///   * pressed - 8
  /// * `padding`
  ///   * `default font size <= 14` - horizontal(16)
  ///   * `14 < default font size <= 28` - lerp(horizontal(16), horizontal(8))
  ///   * `28 < default font size <= 36` - lerp(horizontal(8), horizontal(4))
  ///   * `36 < default font size` - horizontal(4)
  /// * `minimumSize` - Size(64, 36)
  /// * `fixedSize` - null
  /// * `maximumSize` - Size.infinite
  /// * `side` - null
  /// * `shape` - RoundedRectangleBorder(borderRadius: BorderRadius.circular(4))
  /// * `mouseCursor`
  ///   * disabled - SystemMouseCursors.basic
  ///   * others - SystemMouseCursors.click
  /// * `visualDensity` - theme.visualDensity
  /// * `tapTargetSize` - theme.materialTapTargetSize
  /// * `animationDuration` - kThemeChangeDuration
  /// * `enableFeedback` - true
  /// * `alignment` - Alignment.center
  /// * `splashFactory` - InkRipple.splashFactory
  ///
  /// The default padding values for the [ElevatedButton.icon] factory are slightly different:
  ///
  /// * `padding`
  ///   * `default font size <= 14` - start(12) end(16)
  ///   * `14 < default font size <= 28` - lerp(start(12) end(16), horizontal(8))
  ///   * `28 < default font size <= 36` - lerp(horizontal(8), horizontal(4))
  ///   * `36 < default font size` - horizontal(4)
  ///
  /// The default value for `side`, which defines the appearance of the button's
  /// outline, is null. That means that the outline is defined by the button
  /// shape's [OutlinedBorder.side]. Typically the default value of an
  /// [OutlinedBorder]'s side is [BorderSide.none], so an outline is not drawn.
  ///
  /// ## Material 3 defaults
  ///
  /// If [ThemeData.useMaterial3] is set to true the following defaults will
  /// be used:
  ///
  /// * `textStyle` - Theme.textTheme.labelLarge
  /// * `backgroundColor`
  ///   * disabled - Theme.colorScheme.onSurface(0.12)
  ///   * others - Theme.colorScheme.surfaceContainerLow
  /// * `foregroundColor`
  ///   * disabled - Theme.colorScheme.onSurface(0.38)
  ///   * others - Theme.colorScheme.primary
  /// * `overlayColor`
  ///   * hovered - Theme.colorScheme.primary(0.08)
  ///   * focused or pressed - Theme.colorScheme.primary(0.1)
  /// * `shadowColor` - Theme.colorScheme.shadow
  /// * `surfaceTintColor` - Colors.transparent
  /// * `elevation`
  ///   * disabled - 0
  ///   * default - 1
  ///   * hovered - 3
  ///   * focused or pressed - 1
  /// * `padding`
  ///   * `default font size <= 14` - horizontal(24)
  ///   * `14 < default font size <= 28` - lerp(horizontal(24), horizontal(12))
  ///   * `28 < default font size <= 36` - lerp(horizontal(12), horizontal(6))
  ///   * `36 < default font size` - horizontal(6)
  /// * `minimumSize` - Size(64, 40)
  /// * `fixedSize` - null
  /// * `maximumSize` - Size.infinite
  /// * `side` - null
  /// * `shape` - StadiumBorder()
  /// * `mouseCursor`
  ///   * disabled - SystemMouseCursors.basic
  ///   * others - SystemMouseCursors.click
  /// * `visualDensity` - Theme.visualDensity
  /// * `tapTargetSize` - Theme.materialTapTargetSize
  /// * `animationDuration` - kThemeChangeDuration
  /// * `enableFeedback` - true
  /// * `alignment` - Alignment.center
  /// * `splashFactory` - Theme.splashFactory
  ///
  /// For the [ElevatedButton.icon] factory, the start (generally the left) value of
  /// [ButtonStyle.padding] is reduced from 24 to 16.

  @override
  ButtonStyle defaultStyleOf(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final ColorScheme colorScheme = theme.colorScheme;

    return Theme.of(context).useMaterial3
        ? _ElevatedButtonDefaultsM3(context)
        : styleFrom(
          backgroundColor: colorScheme.primary,
          foregroundColor: colorScheme.onPrimary,
          disabledBackgroundColor: colorScheme.onSurface.withOpacity(0.12),
          disabledForegroundColor: colorScheme.onSurface.withOpacity(0.38),
          shadowColor: theme.shadowColor,
          elevation: 2,
          textStyle: theme.textTheme.labelLarge,
          padding: _scaledPadding(context),
          minimumSize: const Size(64, 36),
          maximumSize: Size.infinite,
          shape: const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4))),
          enabledMouseCursor: SystemMouseCursors.click,
          disabledMouseCursor: SystemMouseCursors.basic,
          visualDensity: theme.visualDensity,
          tapTargetSize: theme.materialTapTargetSize,
          animationDuration: kThemeChangeDuration,
          enableFeedback: true,
          alignment: Alignment.center,
          splashFactory: InkRipple.splashFactory,
        );
  }

  /// Returns the [ElevatedButtonThemeData.style] of the closest
  /// [ElevatedButtonTheme] ancestor.
  @override
  ButtonStyle? themeStyleOf(BuildContext context) {
    return ElevatedButtonTheme.of(context).style;
  }
}

EdgeInsetsGeometry _scaledPadding(BuildContext context) {
  final ThemeData theme = Theme.of(context);
  final double padding1x = theme.useMaterial3 ? 24.0 : 16.0;
  final double defaultFontSize = theme.textTheme.labelLarge?.fontSize ?? 14.0;
  final double effectiveTextScale = MediaQuery.textScalerOf(context).scale(defaultFontSize) / 14.0;

  return ButtonStyleButton.scaledPadding(
    EdgeInsets.symmetric(horizontal: padding1x),
    EdgeInsets.symmetric(horizontal: padding1x / 2),
    EdgeInsets.symmetric(horizontal: padding1x / 2 / 2),
    effectiveTextScale,
  );
}

class _ElevatedButtonWithIcon extends ElevatedButton {
  _ElevatedButtonWithIcon({
    super.key,
    required super.onPressed,
    super.onLongPress,
    super.onHover,
    super.onFocusChange,
    super.style,
    super.focusNode,
    bool? autofocus,
    super.clipBehavior,
    super.statesController,
    required Widget icon,
    required Widget label,
    IconAlignment? iconAlignment,
  }) : super(
         autofocus: autofocus ?? false,
         child: _ElevatedButtonWithIconChild(
           icon: icon,
           label: label,
           buttonStyle: style,
           iconAlignment: iconAlignment,
         ),
       );

  @override
  ButtonStyle defaultStyleOf(BuildContext context) {
    final bool useMaterial3 = Theme.of(context).useMaterial3;
    final ButtonStyle buttonStyle = super.defaultStyleOf(context);
    final double defaultFontSize =
        buttonStyle.textStyle?.resolve(const <MaterialState>{})?.fontSize ?? 14.0;
    final double effectiveTextScale =
        MediaQuery.textScalerOf(context).scale(defaultFontSize) / 14.0;

    final EdgeInsetsGeometry scaledPadding =
        useMaterial3
            ? ButtonStyleButton.scaledPadding(
              const EdgeInsetsDirectional.fromSTEB(16, 0, 24, 0),
              const EdgeInsetsDirectional.fromSTEB(8, 0, 12, 0),
              const EdgeInsetsDirectional.fromSTEB(4, 0, 6, 0),
              effectiveTextScale,
            )
            : ButtonStyleButton.scaledPadding(
              const EdgeInsetsDirectional.fromSTEB(12, 0, 16, 0),
              const EdgeInsets.symmetric(horizontal: 8),
              const EdgeInsetsDirectional.fromSTEB(8, 0, 4, 0),
              effectiveTextScale,
            );
    return buttonStyle.copyWith(
      padding: MaterialStatePropertyAll<EdgeInsetsGeometry>(scaledPadding),
    );
  }
}

class _ElevatedButtonWithIconChild extends StatelessWidget {
  const _ElevatedButtonWithIconChild({
    required this.label,
    required this.icon,
    required this.buttonStyle,
    required this.iconAlignment,
  });

  final Widget label;
  final Widget icon;
  final ButtonStyle? buttonStyle;
  final IconAlignment? iconAlignment;

  @override
  Widget build(BuildContext context) {
    final double defaultFontSize =
        buttonStyle?.textStyle?.resolve(const <MaterialState>{})?.fontSize ?? 14.0;
    final double scale =
        clampDouble(MediaQuery.textScalerOf(context).scale(defaultFontSize) / 14.0, 1.0, 2.0) - 1.0;
    final double gap = lerpDouble(8, 4, scale)!;
    final ElevatedButtonThemeData elevatedButtonTheme = ElevatedButtonTheme.of(context);
    final IconAlignment effectiveIconAlignment =
        iconAlignment ??
        elevatedButtonTheme.style?.iconAlignment ??
        buttonStyle?.iconAlignment ??
        IconAlignment.start;
    return Row(
      mainAxisSize: MainAxisSize.min,
      children:
          effectiveIconAlignment == IconAlignment.start
              ? <Widget>[icon, SizedBox(width: gap), Flexible(child: label)]
              : <Widget>[Flexible(child: label), SizedBox(width: gap), icon],
    );
  }
}

// BEGIN GENERATED TOKEN PROPERTIES - ElevatedButton

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _ElevatedButtonDefaultsM3 extends ButtonStyle {
  _ElevatedButtonDefaultsM3(this.context)
   : super(
       animationDuration: kThemeChangeDuration,
       enableFeedback: true,
       alignment: Alignment.center,
     );

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;

  @override
  MaterialStateProperty<TextStyle?> get textStyle =>
    MaterialStatePropertyAll<TextStyle?>(Theme.of(context).textTheme.labelLarge);

  @override
  MaterialStateProperty<Color?>? get backgroundColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return _colors.onSurface.withOpacity(0.12);
      }
      return _colors.surfaceContainerLow;
    });

  @override
  MaterialStateProperty<Color?>? get foregroundColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return _colors.onSurface.withOpacity(0.38);
      }
      return _colors.primary;
    });

  @override
  MaterialStateProperty<Color?>? get overlayColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.pressed)) {
        return _colors.primary.withOpacity(0.1);
      }
      if (states.contains(MaterialState.hovered)) {
        return _colors.primary.withOpacity(0.08);
      }
      if (states.contains(MaterialState.focused)) {
        return _colors.primary.withOpacity(0.1);
      }
      return null;
    });

  @override
  MaterialStateProperty<Color>? get shadowColor =>
    MaterialStatePropertyAll<Color>(_colors.shadow);

  @override
  MaterialStateProperty<Color>? get surfaceTintColor =>
    const MaterialStatePropertyAll<Color>(Colors.transparent);

  @override
  MaterialStateProperty<double>? get elevation =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return 0.0;
      }
      if (states.contains(MaterialState.pressed)) {
        return 1.0;
      }
      if (states.contains(MaterialState.hovered)) {
        return 3.0;
      }
      if (states.contains(MaterialState.focused)) {
        return 1.0;
      }
      return 1.0;
    });

  @override
  MaterialStateProperty<EdgeInsetsGeometry>? get padding =>
    MaterialStatePropertyAll<EdgeInsetsGeometry>(_scaledPadding(context));

  @override
  MaterialStateProperty<Size>? get minimumSize =>
    const MaterialStatePropertyAll<Size>(Size(64.0, 40.0));

  // No default fixedSize

  @override
  MaterialStateProperty<double>? get iconSize =>
    const MaterialStatePropertyAll<double>(18.0);

  @override
  MaterialStateProperty<Color>? get iconColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return _colors.onSurface.withOpacity(0.38);
      }
      if (states.contains(MaterialState.pressed)) {
        return _colors.primary;
      }
      if (states.contains(MaterialState.hovered)) {
        return _colors.primary;
      }
      if (states.contains(MaterialState.focused)) {
        return _colors.primary;
      }
      return _colors.primary;
    });
  }

  @override
  MaterialStateProperty<Size>? get maximumSize =>
    const MaterialStatePropertyAll<Size>(Size.infinite);

  // No default side

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
  VisualDensity? get visualDensity => Theme.of(context).visualDensity;

  @override
  MaterialTapTargetSize? get tapTargetSize => Theme.of(context).materialTapTargetSize;

  @override
  InteractiveInkFeatureFactory? get splashFactory => Theme.of(context).splashFactory;
}
// dart format on

// END GENERATED TOKEN PROPERTIES - ElevatedButton
