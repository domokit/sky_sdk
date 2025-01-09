// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class DatePickerTemplate extends TokenTemplate {
  const DatePickerTemplate(
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
    super.textThemePrefix = '_textTheme.',
  });

  String _layerOpacity(String layerToken) {
    if (tokenAvailable(layerToken)) {
      final String layerValue = getToken(layerToken) as String;
      if (tokenAvailable(layerValue)) {
        final String? opacityValue = opacity(layerValue);
        if (opacityValue != null) {
          return '.withOpacity($opacityValue)';
        }
      }
    }
    return '';
  }

  String _stateColor(String componentToken, String? type, String state) {
    final String baseColor = color(
      type != null
          ? '$componentToken.$type.$state.state-layer.color'
          : '$componentToken.$state.state-layer.color',
      '',
    );
    if (baseColor.isEmpty) {
      return 'null';
    }
    final String opacity = _layerOpacity('$componentToken.$state.state-layer.opacity');
    return '$baseColor$opacity';
  }

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends DatePickerThemeData {
  _${blockName}DefaultsM3(this.context)
    : super(
        elevation: ${elevation("md.comp.date-picker.modal.container")},
        shape: ${shape("md.comp.date-picker.modal.container")},
        // TODO(tahatesser): Update this to use token when gen_defaults
        // supports `CircleBorder` for fully rounded corners.
        dayShape: const MaterialStatePropertyAll<OutlinedBorder>(CircleBorder()),
        rangePickerElevation: ${elevation("md.comp.date-picker.modal.range-selection.container")},
        rangePickerShape: ${shape("md.comp.date-picker.modal.range-selection.container")},
      );

  final BuildContext context;
  late final ThemeData _theme = Theme.of(context);
  late final ColorScheme _colors = _theme.colorScheme;
  late final TextTheme _textTheme = _theme.textTheme;

  @override
  Color? get backgroundColor => ${componentColor("md.comp.date-picker.modal.container")};

  @override
  ButtonStyle get cancelButtonStyle {
    return TextButton.styleFrom();
  }

  @override
  ButtonStyle get confirmButtonStyle {
    return TextButton.styleFrom();
  }

  @override
  Color? get shadowColor => ${colorOrTransparent("md.comp.date-picker.modal.container.shadow-color")};

  @override
  Color? get surfaceTintColor => ${colorOrTransparent("md.comp.date-picker.modal.container.surface-tint-layer.color")};

  @override
  Color? get headerBackgroundColor => ${colorOrTransparent("md.comp.date-picker.modal.header.container.color")};

  @override
  Color? get headerForegroundColor => ${colorOrTransparent("md.comp.date-picker.modal.header.headline.color")};

  @override
  TextStyle? get headerHeadlineStyle => ${textStyle("md.comp.date-picker.modal.header.headline")};

  @override
  TextStyle? get headerHelpStyle => ${textStyle("md.comp.date-picker.modal.header.supporting-text")};

  @override
  TextStyle? get weekdayStyle => ${textStyle("md.comp.date-picker.modal.weekdays.label-text")}?.apply(
    color: ${componentColor("md.comp.date-picker.modal.weekdays.label-text")},
  );

  @override
  TextStyle? get dayStyle => ${textStyle("md.comp.date-picker.modal.date.label-text")};

  @override
  MaterialStateProperty<Color?>? get dayForegroundColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return ${componentColor('md.comp.date-picker.modal.date.selected.label-text')};
      } else if (states.contains(MaterialState.disabled)) {
        return ${componentColor('md.comp.date-picker.modal.date.unselected.label-text')}.withOpacity(0.38);
      }
      return ${componentColor('md.comp.date-picker.modal.date.unselected.label-text')};
    });

  @override
  MaterialStateProperty<Color?>? get dayBackgroundColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return ${componentColor('md.comp.date-picker.modal.date.selected.container')};
      }
      return ${componentColor('md.comp.date-picker.modal.date.unselected.container')};
    });

  @override
  MaterialStateProperty<Color?>? get dayOverlayColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        if (states.contains(MaterialState.pressed)) {
          return ${_stateColor('md.comp.date-picker.modal.date', 'selected', 'pressed')};
        }
        if (states.contains(MaterialState.hovered)) {
          return ${_stateColor('md.comp.date-picker.modal.date', 'selected', 'hover')};
        }
        if (states.contains(MaterialState.focused)) {
          return ${_stateColor('md.comp.date-picker.modal.date', 'selected', 'focus')};
        }
      } else {
        if (states.contains(MaterialState.pressed)) {
          return ${_stateColor('md.comp.date-picker.modal.date', 'unselected', 'pressed')};
        }
        if (states.contains(MaterialState.hovered)) {
          return ${_stateColor('md.comp.date-picker.modal.date', 'unselected', 'hover')};
        }
        if (states.contains(MaterialState.focused)) {
          return ${_stateColor('md.comp.date-picker.modal.date', 'unselected', 'focus')};
        }
      }
      return null;
    });

  @override
  MaterialStateProperty<Color?>? get todayForegroundColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return ${componentColor('md.comp.date-picker.modal.date.selected.label-text')};
      } else if (states.contains(MaterialState.disabled)) {
        return ${componentColor('md.comp.date-picker.modal.date.today.label-text')}.withOpacity(0.38);
      }
      return ${componentColor('md.comp.date-picker.modal.date.today.label-text')};
    });

  @override
  MaterialStateProperty<Color?>? get todayBackgroundColor => dayBackgroundColor;

  @override
  BorderSide? get todayBorder => ${border('md.comp.date-picker.modal.date.today.container.outline')};

  @override
  TextStyle? get yearStyle => ${textStyle("md.comp.date-picker.modal.year-selection.year.label-text")};

  @override
  MaterialStateProperty<Color?>? get yearForegroundColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return ${componentColor('md.comp.date-picker.modal.year-selection.year.selected.label-text')};
      } else if (states.contains(MaterialState.disabled)) {
        return ${componentColor('md.comp.date-picker.modal.year-selection.year.unselected.label-text')}.withOpacity(0.38);
      }
      return ${componentColor('md.comp.date-picker.modal.year-selection.year.unselected.label-text')};
    });

  @override
  MaterialStateProperty<Color?>? get yearBackgroundColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return ${componentColor('md.comp.date-picker.modal.year-selection.year.selected.container')};
      }
      return ${componentColor('md.comp.date-picker.modal.year-selection.year.unselected.container')};
    });

  @override
  MaterialStateProperty<Color?>? get yearOverlayColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        if (states.contains(MaterialState.pressed)) {
          return ${_stateColor('md.comp.date-picker.modal.year-selection.year', 'selected', 'pressed')};
        }
        if (states.contains(MaterialState.hovered)) {
          return ${_stateColor('md.comp.date-picker.modal.year-selection.year', 'selected', 'hover')};
        }
        if (states.contains(MaterialState.focused)) {
          return ${_stateColor('md.comp.date-picker.modal.year-selection.year', 'selected', 'focus')};
        }
      } else {
        if (states.contains(MaterialState.pressed)) {
          return ${_stateColor('md.comp.date-picker.modal.year-selection.year', 'unselected', 'pressed')};
        }
        if (states.contains(MaterialState.hovered)) {
          return ${_stateColor('md.comp.date-picker.modal.year-selection.year', 'unselected', 'hover')};
        }
        if (states.contains(MaterialState.focused)) {
          return ${_stateColor('md.comp.date-picker.modal.year-selection.year', 'unselected', 'focus')};
        }
      }
      return null;
    });

    @override
    Color? get rangePickerShadowColor => ${colorOrTransparent("md.comp.date-picker.modal.range-selection.container.shadow-color")};

    @override
    Color? get rangePickerSurfaceTintColor => ${colorOrTransparent("md.comp.date-picker.modal.range-selection.container.surface-tint-layer.color")};

    @override
    Color? get rangeSelectionBackgroundColor => ${colorOrTransparent("md.comp.date-picker.modal.range-selection.active-indicator.container.color")};

  @override
  MaterialStateProperty<Color?>? get rangeSelectionOverlayColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.pressed)) {
        return ${_stateColor('md.comp.date-picker.modal.range-selection.date.in-range', null, 'pressed')};
      }
      if (states.contains(MaterialState.hovered)) {
        return ${_stateColor('md.comp.date-picker.modal.range-selection.date.in-range', null, 'hover')};
      }
      if (states.contains(MaterialState.focused)) {
        return ${_stateColor('md.comp.date-picker.modal.range-selection.date.in-range', null, 'focus')};
      }
      return null;
    });

  @override
  Color? get rangePickerHeaderBackgroundColor => ${colorOrTransparent("md.comp.date-picker.modal.header.container.color")};

  @override
  Color? get rangePickerHeaderForegroundColor => ${colorOrTransparent("md.comp.date-picker.modal.header.headline.color")};

  @override
  TextStyle? get rangePickerHeaderHeadlineStyle => ${textStyle("md.comp.date-picker.modal.range-selection.header.headline")};

  @override
  TextStyle? get rangePickerHeaderHelpStyle => ${textStyle("md.comp.date-picker.modal.range-selection.month.subhead")};
}
''';
}
