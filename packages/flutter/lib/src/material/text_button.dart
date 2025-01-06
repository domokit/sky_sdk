// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'elevated_button.dart';
/// @docImport 'filled_button.dart';
/// @docImport 'material.dart';
/// @docImport 'outlined_button.dart';
library;

import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'button_style.dart';
import 'button_style_button.dart';
import 'color_scheme.dart';
import 'colors.dart';
import 'constants.dart';
import 'ink_ripple.dart';
import 'ink_well.dart';
import 'material_state.dart';
import 'text_button_theme.dart';
import 'theme.dart';
import 'theme_data.dart';

/// A Material Design "Text Button".
///
/// Use text buttons on toolbars, in dialogs, or inline with other
/// content but offset from that content with padding so that the
/// button's presence is obvious. Text buttons do not have visible
/// borders and must therefore rely on their position relative to
/// other content for context. In dialogs and cards, they should be
/// grouped together in one of the bottom corners. Avoid using text
/// buttons where they would blend in with other content, for example
/// in the middle of lists.
///
/// A text button is a label [child] displayed on a (zero elevation)
/// [Material] widget. The label's [Text] and [Icon] widgets are
/// displayed in the [style]'s [ButtonStyle.foregroundColor]. The
/// button reacts to touches by filling with the [style]'s
/// [ButtonStyle.backgroundColor].
///
/// The text button's default style is defined by [defaultStyleOf].
/// The style of this text button can be overridden with its [style]
/// parameter. The style of all text buttons in a subtree can be
/// overridden with the [TextButtonTheme] and the style of all of the
/// text buttons in an app can be overridden with the [Theme]'s
/// [ThemeData.textButtonTheme] property.
///
/// The static [styleFrom] method is a convenient way to create a
/// text button [ButtonStyle] from simple values.
///
/// If the [onPressed] and [onLongPress] callbacks are null, then this
/// button will be disabled, it will not react to touch.
///
/// {@tool dartpad}
/// This sample shows various ways to configure TextButtons, from the
/// simplest default appearance to versions that don't resemble
/// Material Design at all.
///
/// ** See code in examples/api/lib/material/text_button/text_button.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This sample demonstrates using the [statesController] parameter to create a button
/// that adds support for [WidgetState.selected].
///
/// ** See code in examples/api/lib/material/text_button/text_button.1.dart **
/// {@end-tool}
///
/// See also:
///
///  * [ElevatedButton], a filled button whose material elevates when pressed.
///  * [FilledButton], a filled button that doesn't elevate when pressed.
///  * [FilledButton.tonal], a filled button variant that uses a secondary fill color.
///  * [OutlinedButton], a button with an outlined border and no fill color.
///  * <https://material.io/design/components/buttons.html>
///  * <https://m3.material.io/components/buttons>
class TextButton extends ButtonStyleButton {
  /// Create a [TextButton].
  const TextButton({
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
    super.isSemanticButton,
    required Widget super.child,
  });

  /// Create a text button from a pair of widgets that serve as the button's
  /// [icon] and [label].
  ///
  /// The icon and label are arranged in a row and padded by 8 logical pixels
  /// at the ends, with an 8 pixel gap in between.
  ///
  /// If [icon] is null, will create a [TextButton] instead.
  ///
  /// {@macro flutter.material.ButtonStyleButton.iconAlignment}
  ///
  factory TextButton.icon({
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
      return TextButton(
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
    return _TextButtonWithIcon(
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

  /// A static convenience method that constructs a text button
  /// [ButtonStyle] given simple values.
  ///
  /// The [foregroundColor] and [disabledForegroundColor] colors are used
  /// to create a [WidgetStateProperty] [ButtonStyle.foregroundColor], and
  /// a derived [ButtonStyle.overlayColor] if [overlayColor] isn't specified.
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
  /// If [overlayColor] is specified and its value is [Colors.transparent]
  /// then the pressed/focused/hovered highlights are effectively defeated.
  /// Otherwise a [WidgetStateProperty] with the same opacities as the
  /// default is created.
  ///
  /// All of the other parameters are either used directly or used to
  /// create a [WidgetStateProperty] with a single value for all
  /// states.
  ///
  /// All parameters default to null. By default this method returns
  /// a [ButtonStyle] that doesn't override anything.
  ///
  /// For example, to override the default text and icon colors for a
  /// [TextButton], as well as its overlay color, with all of the
  /// standard opacity adjustments for the pressed, focused, and
  /// hovered states, one could write:
  ///
  /// ```dart
  /// TextButton(
  ///   style: TextButton.styleFrom(foregroundColor: Colors.green),
  ///   child: const Text('Give Kate a mix tape'),
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
    final MaterialStateProperty<Color?>? backgroundColorProp = switch ((
      backgroundColor,
      disabledBackgroundColor,
    )) {
      (_?, null) => MaterialStatePropertyAll<Color?>(backgroundColor),
      (_, _) => ButtonStyleButton.defaultColor(backgroundColor, disabledBackgroundColor),
    };
    final MaterialStateProperty<Color?>? iconColorProp = switch ((iconColor, disabledIconColor)) {
      (_?, null) => MaterialStatePropertyAll<Color?>(iconColor),
      (_, _) => ButtonStyleButton.defaultColor(iconColor, disabledIconColor),
    };
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

    return ButtonStyle(
      textStyle: ButtonStyleButton.allOrNull<TextStyle>(textStyle),
      foregroundColor: ButtonStyleButton.defaultColor(foregroundColor, disabledForegroundColor),
      backgroundColor: backgroundColorProp,
      overlayColor: overlayColorProp,
      shadowColor: ButtonStyleButton.allOrNull<Color>(shadowColor),
      surfaceTintColor: ButtonStyleButton.allOrNull<Color>(surfaceTintColor),
      iconColor: iconColorProp,
      iconSize: ButtonStyleButton.allOrNull<double>(iconSize),
      iconAlignment: iconAlignment,
      elevation: ButtonStyleButton.allOrNull<double>(elevation),
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
  /// {@template flutter.material.text_button.default_style_of}
  /// The button [child]'s [Text] and [Icon] widgets are rendered with
  /// the [ButtonStyle]'s foreground color. The button's [InkWell] adds
  /// the style's overlay color when the button is focused, hovered
  /// or pressed. The button's background color becomes its [Material]
  /// color and is transparent by default.
  ///
  /// All of the [ButtonStyle]'s defaults appear below.
  ///
  /// In this list "Theme.foo" is shorthand for
  /// `Theme.of(context).foo`. Color scheme values like
  /// "onSurface(0.38)" are shorthand for
  /// `onSurface.withOpacity(0.38)`. [WidgetStateProperty] valued
  /// properties that are not followed by a sublist have the same
  /// value for all states, otherwise the values are as specified for
  /// each state and "others" means all other states.
  ///
  /// The "default font size" below refers to the font size specified in the
  /// [defaultStyleOf] method (or 14.0 if unspecified), scaled by the
  /// `MediaQuery.textScalerOf(context).scale` method. And the names of the
  /// EdgeInsets constructors and `EdgeInsetsGeometry.lerp` have been abbreviated
  /// for readability.
  ///
  /// The color of the [ButtonStyle.textStyle] is not used, the
  /// [ButtonStyle.foregroundColor] color is used instead.
  /// {@endtemplate}
  ///
  /// ## Material 2 defaults
  ///
  /// * `textStyle` - Theme.textTheme.button
  /// * `backgroundColor` - transparent
  /// * `foregroundColor`
  ///   * disabled - Theme.colorScheme.onSurface(0.38)
  ///   * others - Theme.colorScheme.primary
  /// * `overlayColor`
  ///   * hovered - Theme.colorScheme.primary(0.08)
  ///   * focused or pressed - Theme.colorScheme.primary(0.12)
  /// * `shadowColor` - Theme.shadowColor
  /// * `elevation` - 0
  /// * `padding`
  ///   * `default font size <= 14` - (horizontal(12), vertical(8))
  ///   * `14 < default font size <= 28` - lerp(all(8), horizontal(8))
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
  /// The default padding values for the [TextButton.icon] factory are slightly different:
  ///
  /// * `padding`
  ///   * `default font size <= 14` - all(8)
  ///   * `14 < default font size <= 28 `- lerp(all(8), horizontal(4))
  ///   * `28 < default font size` - horizontal(4)
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
  /// {@template flutter.material.text_button.material3_defaults}
  /// * `textStyle` - Theme.textTheme.labelLarge
  /// * `backgroundColor` - transparent
  /// * `foregroundColor`
  ///   * disabled - Theme.colorScheme.onSurface(0.38)
  ///   * others - Theme.colorScheme.primary
  /// * `overlayColor`
  ///   * hovered - Theme.colorScheme.primary(0.08)
  ///   * focused or pressed - Theme.colorScheme.primary(0.1)
  ///   * others - null
  /// * `shadowColor` - Colors.transparent,
  /// * `surfaceTintColor` - null
  /// * `elevation` - 0
  /// * `padding`
  ///   * `default font size <= 14` - lerp(horizontal(12), horizontal(4))
  ///   * `14 < default font size <= 28` - lerp(all(8), horizontal(8))
  ///   * `28 < default font size <= 36` - lerp(horizontal(8), horizontal(4))
  ///   * `36 < default font size` - horizontal(4)
  /// * `minimumSize` - Size(64, 40)
  /// * `fixedSize` - null
  /// * `maximumSize` - Size.infinite
  /// * `side` - null
  /// * `shape` - StadiumBorder()
  /// * `mouseCursor`
  ///   * disabled - SystemMouseCursors.basic
  ///   * others - SystemMouseCursors.click
  /// * `visualDensity` - theme.visualDensity
  /// * `tapTargetSize` - theme.materialTapTargetSize
  /// * `animationDuration` - kThemeChangeDuration
  /// * `enableFeedback` - true
  /// * `alignment` - Alignment.center
  /// * `splashFactory` - Theme.splashFactory
  ///
  /// For the [TextButton.icon] factory, the end (generally the right) value of
  /// `padding` is increased from 12 to 16.
  /// {@endtemplate}
  @override
  ButtonStyle defaultStyleOf(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final ColorScheme colorScheme = theme.colorScheme;

    return Theme.of(context).useMaterial3
        ? _TextButtonDefaultsM3(context)
        : styleFrom(
          foregroundColor: colorScheme.primary,
          disabledForegroundColor: colorScheme.onSurface.withOpacity(0.38),
          backgroundColor: Colors.transparent,
          disabledBackgroundColor: Colors.transparent,
          shadowColor: theme.shadowColor,
          elevation: 0,
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

  /// Returns the [TextButtonThemeData.style] of the closest
  /// [TextButtonTheme] ancestor.
  @override
  ButtonStyle? themeStyleOf(BuildContext context) {
    return TextButtonTheme.of(context).style;
  }
}

EdgeInsetsGeometry _scaledPadding(BuildContext context) {
  final ThemeData theme = Theme.of(context);
  final double defaultFontSize = theme.textTheme.labelLarge?.fontSize ?? 14.0;
  final double effectiveTextScale = MediaQuery.textScalerOf(context).scale(defaultFontSize) / 14.0;
  return ButtonStyleButton.scaledPadding(
    theme.useMaterial3
        ? const EdgeInsets.symmetric(horizontal: 12, vertical: 8)
        : const EdgeInsets.all(8),
    const EdgeInsets.symmetric(horizontal: 8),
    const EdgeInsets.symmetric(horizontal: 4),
    effectiveTextScale,
  );
}

class _TextButtonWithIcon extends TextButton {
  _TextButtonWithIcon({
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
         child: _TextButtonWithIconChild(
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
    final EdgeInsetsGeometry scaledPadding = ButtonStyleButton.scaledPadding(
      useMaterial3 ? const EdgeInsetsDirectional.fromSTEB(12, 8, 16, 8) : const EdgeInsets.all(8),
      const EdgeInsets.symmetric(horizontal: 4),
      const EdgeInsets.symmetric(horizontal: 4),
      effectiveTextScale,
    );
    return buttonStyle.copyWith(
      padding: MaterialStatePropertyAll<EdgeInsetsGeometry>(scaledPadding),
    );
  }
}

class _TextButtonWithIconChild extends StatelessWidget {
  const _TextButtonWithIconChild({
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
    final TextButtonThemeData textButtonTheme = TextButtonTheme.of(context);
    final IconAlignment effectiveIconAlignment =
        iconAlignment ??
        textButtonTheme.style?.iconAlignment ??
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

// BEGIN GENERATED TOKEN PROPERTIES - TextButton

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _TextButtonDefaultsM3 extends ButtonStyle {
  _TextButtonDefaultsM3(this.context)
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
    const MaterialStatePropertyAll<Color>(Colors.transparent);

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
    const MaterialStatePropertyAll<Color>(Colors.transparent);

  @override
  MaterialStateProperty<Color>? get surfaceTintColor =>
    const MaterialStatePropertyAll<Color>(Colors.transparent);

  @override
  MaterialStateProperty<double>? get elevation =>
    const MaterialStatePropertyAll<double>(0.0);

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

// END GENERATED TOKEN PROPERTIES - TextButton
