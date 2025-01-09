// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class TimePickerTemplate extends TokenTemplate {
  const TimePickerTemplate(
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
    super.textThemePrefix = '_textTheme.',
  });

  static const String tokenGroup = 'md.comp.time-picker';
  static const String hourMinuteComponent = '$tokenGroup.time-selector';
  static const String dayPeriodComponent = '$tokenGroup.period-selector';
  static const String dialComponent = '$tokenGroup.clock-dial';
  static const String variant = '';

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends _TimePickerDefaults {
  _${blockName}DefaultsM3(this.context, { this.entryMode = TimePickerEntryMode.dial });

  final BuildContext context;
  final TimePickerEntryMode entryMode;

  late final ColorScheme _colors = Theme.of(context).colorScheme;
  late final TextTheme _textTheme = Theme.of(context).textTheme;

  @override
  Color get backgroundColor {
    return ${componentColor("$tokenGroup.container")};
  }

  @override
  ButtonStyle get cancelButtonStyle {
    return TextButton.styleFrom();
  }

  @override
  ButtonStyle get confirmButtonStyle {
    return TextButton.styleFrom();
  }

  @override
  BorderSide get dayPeriodBorderSide {
    return ${border('$dayPeriodComponent.outline')};
  }

  @override
  Color get dayPeriodColor {
    return MaterialStateColor.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return ${componentColor("$dayPeriodComponent.selected.container")};
      }
      // The unselected day period should match the overall picker dialog color.
      // Making it transparent enables that without being redundant and allows
      // the optional elevation overlay for dark mode to be visible.
      return Colors.transparent;
    });
  }

  @override
  OutlinedBorder get dayPeriodShape {
    return ${shape("$dayPeriodComponent.container")}.copyWith(side: dayPeriodBorderSide);
  }

  @override
  Size get dayPeriodPortraitSize {
    return ${size('$dayPeriodComponent.vertical.container')};
  }

  @override
  Size get dayPeriodLandscapeSize {
    return ${size('$dayPeriodComponent.horizontal.container')};
  }

  @override
  Size get dayPeriodInputSize {
    // Input size is eight pixels smaller than the portrait size in the spec,
    // but there's not token for it yet.
    return Size(dayPeriodPortraitSize.width, dayPeriodPortraitSize.height - 8);
  }

  @override
  Color get dayPeriodTextColor {
    return MaterialStateColor.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        if (states.contains(MaterialState.focused)) {
          return ${componentColor("$dayPeriodComponent.selected.focus.label-text")};
        }
        if (states.contains(MaterialState.hovered)) {
          return ${componentColor("$dayPeriodComponent.selected.hover.label-text")};
        }
        if (states.contains(MaterialState.pressed)) {
          return ${componentColor("$dayPeriodComponent.selected.pressed.label-text")};
        }
        return ${componentColor("$dayPeriodComponent.selected.label-text")};
      }
      if (states.contains(MaterialState.focused)) {
        return ${componentColor("$dayPeriodComponent.unselected.focus.label-text")};
      }
      if (states.contains(MaterialState.hovered)) {
        return ${componentColor("$dayPeriodComponent.unselected.hover.label-text")};
      }
      if (states.contains(MaterialState.pressed)) {
        return ${componentColor("$dayPeriodComponent.unselected.pressed.label-text")};
      }
      return ${componentColor("$dayPeriodComponent.unselected.label-text")};
    });
  }

  @override
  TextStyle get dayPeriodTextStyle {
    return ${textStyle("$dayPeriodComponent.label-text")}!.copyWith(color: dayPeriodTextColor);
  }

  @override
  Color get dialBackgroundColor {
    return ${componentColor(dialComponent)};
  }

  @override
  Color get dialHandColor {
    return ${componentColor('$dialComponent.selector.handle.container')};
  }

  @override
  Size get dialSize {
    return ${size("$dialComponent.container")};
  }

  @override
  double get handWidth {
    return ${size("$dialComponent.selector.track.container")}.width;
  }

  @override
  double get dotRadius {
    return ${size("$dialComponent.selector.handle.container")}.width / 2;
  }

  @override
  double get centerRadius {
    return ${size("$dialComponent.selector.center.container")}.width / 2;
  }

  @override
  Color get dialTextColor {
    return MaterialStateColor.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return ${componentColor('$dialComponent.selected.label-text')};
      }
      return ${componentColor('$dialComponent.unselected.label-text')};
    });
  }

  @override
  TextStyle get dialTextStyle {
    return ${textStyle('$dialComponent.label-text')}!;
  }

  @override
  double get elevation {
    return ${elevation("$tokenGroup.container")};
  }

  @override
  Color get entryModeIconColor {
    return _colors.onSurface;
  }

  @override
  TextStyle get helpTextStyle {
    return MaterialStateTextStyle.resolveWith((Set<MaterialState> states) {
      final TextStyle textStyle = ${textStyle('$tokenGroup.headline')}!;
      return textStyle.copyWith(color: ${componentColor('$tokenGroup.headline')});
    });
  }

  @override
  EdgeInsetsGeometry get padding {
    return const EdgeInsets.all(24);
  }

  @override
  Color get hourMinuteColor {
    return MaterialStateColor.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        Color overlayColor = ${componentColor('$hourMinuteComponent.selected.container')};
        if (states.contains(MaterialState.pressed)) {
          overlayColor = ${componentColor('$hourMinuteComponent.selected.pressed.state-layer')};
        } else if (states.contains(MaterialState.hovered)) {
          const double hoverOpacity = ${opacity('$hourMinuteComponent.hover.state-layer.opacity')};
          overlayColor = ${componentColor('$hourMinuteComponent.selected.hover.state-layer')}.withOpacity(hoverOpacity);
        } else if (states.contains(MaterialState.focused)) {
          const double focusOpacity = ${opacity('$hourMinuteComponent.focus.state-layer.opacity')};
          overlayColor = ${componentColor('$hourMinuteComponent.selected.focus.state-layer')}.withOpacity(focusOpacity);
        }
        return Color.alphaBlend(overlayColor, ${componentColor('$hourMinuteComponent.selected.container')});
      } else {
        Color overlayColor = ${componentColor('$hourMinuteComponent.unselected.container')};
        if (states.contains(MaterialState.pressed)) {
          overlayColor = ${componentColor('$hourMinuteComponent.unselected.pressed.state-layer')};
        } else if (states.contains(MaterialState.hovered)) {
          const double hoverOpacity = ${opacity('$hourMinuteComponent.hover.state-layer.opacity')};
          overlayColor = ${componentColor('$hourMinuteComponent.unselected.hover.state-layer')}.withOpacity(hoverOpacity);
        } else if (states.contains(MaterialState.focused)) {
          const double focusOpacity = ${opacity('$hourMinuteComponent.focus.state-layer.opacity')};
          overlayColor = ${componentColor('$hourMinuteComponent.unselected.focus.state-layer')}.withOpacity(focusOpacity);
        }
        return Color.alphaBlend(overlayColor, ${componentColor('$hourMinuteComponent.unselected.container')});
      }
    });
  }

  @override
  ShapeBorder get hourMinuteShape {
    return ${shape('$hourMinuteComponent.container')};
  }

  @override
  Size get hourMinuteSize {
    return ${size('$hourMinuteComponent.container')};
  }

  @override
  Size get hourMinuteSize24Hour {
    return Size(${size('$hourMinuteComponent.24h-vertical.container')}.width, hourMinuteSize.height);
  }

  @override
  Size get hourMinuteInputSize {
    // Input size is eight pixels smaller than the regular size in the spec, but
    // there's not token for it yet.
    return Size(hourMinuteSize.width, hourMinuteSize.height - 8);
  }

  @override
  Size get hourMinuteInputSize24Hour {
    // Input size is eight pixels smaller than the regular size in the spec, but
    // there's not token for it yet.
    return Size(hourMinuteSize24Hour.width, hourMinuteSize24Hour.height - 8);
  }

  @override
  Color get hourMinuteTextColor {
    return MaterialStateColor.resolveWith((Set<MaterialState> states) {
      return _hourMinuteTextColor.resolve(states);
    });
  }

  MaterialStateProperty<Color> get _hourMinuteTextColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        if (states.contains(MaterialState.pressed)) {
          return ${componentColor("$hourMinuteComponent.selected.pressed.label-text")};
        }
        if (states.contains(MaterialState.hovered)) {
          return ${componentColor("$hourMinuteComponent.selected.hover.label-text")};
        }
        if (states.contains(MaterialState.focused)) {
          return ${componentColor("$hourMinuteComponent.selected.focus.label-text")};
        }
        return ${componentColor("$hourMinuteComponent.selected.label-text")};
      } else {
        // unselected
        if (states.contains(MaterialState.pressed)) {
          return ${componentColor("$hourMinuteComponent.unselected.pressed.label-text")};
        }
        if (states.contains(MaterialState.hovered)) {
          return ${componentColor("$hourMinuteComponent.unselected.hover.label-text")};
        }
        if (states.contains(MaterialState.focused)) {
          return ${componentColor("$hourMinuteComponent.unselected.focus.label-text")};
        }
        return ${componentColor("$hourMinuteComponent.unselected.label-text")};
      }
    });
  }

  @override
  TextStyle get hourMinuteTextStyle {
    return MaterialStateTextStyle.resolveWith((Set<MaterialState> states) {
      // TODO(tahatesser): Update this when https://github.com/flutter/flutter/issues/131247 is fixed.
      // This is using the correct text style from Material 3 spec.
      // https://m3.material.io/components/time-pickers/specs#fd0b6939-edab-4058-82e1-93d163945215
      return switch (entryMode) {
        TimePickerEntryMode.dial || TimePickerEntryMode.dialOnly
          => _textTheme.displayLarge!.copyWith(color: _hourMinuteTextColor.resolve(states)),
        TimePickerEntryMode.input || TimePickerEntryMode.inputOnly
          => _textTheme.displayMedium!.copyWith(color: _hourMinuteTextColor.resolve(states)),
      };
    });
  }

  @override
  InputDecorationTheme get inputDecorationTheme {
    // This is NOT correct, but there's no token for
    // 'time-input.container.shape', so this is using the radius from the shape
    // for the hour/minute selector. It's a BorderRadiusGeometry, so we have to
    // resolve it before we can use it.
    final BorderRadius selectorRadius = ${shape('$hourMinuteComponent.container')}
      .borderRadius
      .resolve(Directionality.of(context));
    return InputDecorationTheme(
      contentPadding: EdgeInsets.zero,
      filled: true,
      // This should be derived from a token, but there isn't one for 'time-input'.
      fillColor: hourMinuteColor,
      // This should be derived from a token, but there isn't one for 'time-input'.
      focusColor: _colors.primaryContainer,
      enabledBorder: OutlineInputBorder(
        borderRadius: selectorRadius,
        borderSide: const BorderSide(color: Colors.transparent),
      ),
      errorBorder: OutlineInputBorder(
        borderRadius: selectorRadius,
        borderSide: BorderSide(color: _colors.error, width: 2),
      ),
      focusedBorder: OutlineInputBorder(
        borderRadius: selectorRadius,
        borderSide: BorderSide(color: _colors.primary, width: 2),
      ),
      focusedErrorBorder: OutlineInputBorder(
        borderRadius: selectorRadius,
        borderSide: BorderSide(color: _colors.error, width: 2),
      ),
      hintStyle: hourMinuteTextStyle.copyWith(color: _colors.onSurface.withOpacity(0.36)),
      // Prevent the error text from appearing.
      // TODO(rami-a): Remove this workaround once
      // https://github.com/flutter/flutter/issues/54104
      // is fixed.
      errorStyle: const TextStyle(fontSize: 0),
    );
  }

  @override
  ShapeBorder get shape {
    return ${shape("$tokenGroup.container")};
  }

  @override
  MaterialStateProperty<Color?>? get timeSelectorSeparatorColor {
    // TODO(tahatesser): Update this when tokens are available.
    // This is taken from https://m3.material.io/components/time-pickers/specs.
    return MaterialStatePropertyAll<Color>(_colors.onSurface);
  }

  @override
  MaterialStateProperty<TextStyle?>? get timeSelectorSeparatorTextStyle {
    // TODO(tahatesser): Update this when tokens are available.
    // This is taken from https://m3.material.io/components/time-pickers/specs.
    return MaterialStatePropertyAll<TextStyle?>(_textTheme.displayLarge);
  }
}
''';
}
