// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class InputDecoratorTemplate extends TokenTemplate {
  const InputDecoratorTemplate(
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
    super.textThemePrefix = '_textTheme.',
  });

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends InputDecorationTheme {
   _${blockName}DefaultsM3(this.context)
    : super();

  final BuildContext context;

  late final ColorScheme _colors = Theme.of(context).colorScheme;
  late final TextTheme _textTheme = Theme.of(context).textTheme;

  // For InputDecorator, focused state should take precedence over hovered state.
  // For instance, the focused state increases border width (2dp) and applies bright
  // colors (primary color or error color) while the hovered state has the same border
  // than the non-focused state (1dp) and uses a color a little darker than non-focused
  // state. On desktop, it is also very common that a text field is focused and hovered
  // because users often rely on mouse selection.
  // For other widgets, hovered state takes precedence over focused state, because it
  // is mainly used to determine the overlay color,
  // see https://github.com/flutter/flutter/pull/125905.

  @override
  TextStyle? get hintStyle => MaterialStateTextStyle.resolveWith((Set<MaterialState> states) {
    if (states.contains(MaterialState.disabled)) {
      return TextStyle(color: ${componentColor('md.comp.filled-text-field.disabled.supporting-text')});
    }
    return TextStyle(color: ${componentColor('md.comp.filled-text-field.supporting-text')});
  });

  @override
  Color? get fillColor => MaterialStateColor.resolveWith((Set<MaterialState> states) {
    if (states.contains(MaterialState.disabled)) {
      return ${componentColor("md.comp.filled-text-field.disabled.container")};
    }
    return ${componentColor("md.comp.filled-text-field.container")};
  });

  @override
  BorderSide? get activeIndicatorBorder => MaterialStateBorderSide.resolveWith((Set<MaterialState> states) {
    if (states.contains(MaterialState.disabled)) {
      return ${border('md.comp.filled-text-field.disabled.active-indicator')};
    }
    if (states.contains(MaterialState.error)) {
      if (states.contains(MaterialState.focused)) {
        return ${mergedBorder('md.comp.filled-text-field.error.focus.active-indicator', 'md.comp.filled-text-field.focus.active-indicator')};
      }
      if (states.contains(MaterialState.hovered)) {
        return ${border('md.comp.filled-text-field.error.hover.active-indicator')};
      }
      return ${border('md.comp.filled-text-field.error.active-indicator')};
    }
    if (states.contains(MaterialState.focused)) {
      return ${border('md.comp.filled-text-field.focus.active-indicator')};
    }
    if (states.contains(MaterialState.hovered)) {
      return ${border('md.comp.filled-text-field.hover.active-indicator')};
    }
    return ${border('md.comp.filled-text-field.active-indicator')};
    });

  @override
  BorderSide? get outlineBorder => MaterialStateBorderSide.resolveWith((Set<MaterialState> states) {
    if (states.contains(MaterialState.disabled)) {
      return ${border('md.comp.outlined-text-field.disabled.outline')};
    }
    if (states.contains(MaterialState.error)) {
      if (states.contains(MaterialState.focused)) {
        return ${mergedBorder('md.comp.outlined-text-field.error.focus.outline', 'md.comp.outlined-text-field.focus.outline')};
      }
      if (states.contains(MaterialState.hovered)) {
        return ${border('md.comp.outlined-text-field.error.hover.outline')};
      }
      return ${border('md.comp.outlined-text-field.error.outline')};
    }
    if (states.contains(MaterialState.focused)) {
      return ${border('md.comp.outlined-text-field.focus.outline')};
    }
    if (states.contains(MaterialState.hovered)) {
      return ${border('md.comp.outlined-text-field.hover.outline')};
    }
    return ${border('md.comp.outlined-text-field.outline')};
  });

  @override
  Color? get iconColor => ${componentColor("md.comp.filled-text-field.leading-icon")};

  @override
  Color? get prefixIconColor => MaterialStateColor.resolveWith((Set<MaterialState> states) {
    if (states.contains(MaterialState.disabled)) {
      return ${componentColor('md.comp.filled-text-field.disabled.leading-icon')};
    }${componentColor('md.comp.filled-text-field.error.leading-icon') == componentColor('md.comp.filled-text-field.leading-icon') ? '' : '''
    if (states.contains(MaterialState.error)) {
      if (states.contains(MaterialState.hovered)) {
        return ${componentColor('md.comp.filled-text-field.error.hover.leading-icon')};
      }
      if (states.contains(MaterialState.focused)) {
        return ${componentColor('md.comp.filled-text-field.error.focus.leading-icon')};
      }
      return ${componentColor('md.comp.filled-text-field.error.leading-icon')};
    }'''}${componentColor('md.comp.filled-text-field.hover.leading-icon') == componentColor('md.comp.filled-text-field.leading-icon') ? '' : '''
    if (states.contains(MaterialState.hovered)) {
      return ${componentColor('md.comp.filled-text-field.hover.leading-icon')};
    }'''}${componentColor('md.comp.filled-text-field.focus.leading-icon') == componentColor('md.comp.filled-text-field.leading-icon') ? '' : '''
    if (states.contains(MaterialState.focused)) {
      return ${componentColor('md.comp.filled-text-field.focus.leading-icon')};
    }'''}
    return ${componentColor('md.comp.filled-text-field.leading-icon')};
  });

  @override
  Color? get suffixIconColor => MaterialStateColor.resolveWith((Set<MaterialState> states) {
    if (states.contains(MaterialState.disabled)) {
      return ${componentColor('md.comp.filled-text-field.disabled.trailing-icon')};
    }
    if (states.contains(MaterialState.error)) {
      if (states.contains(MaterialState.hovered)) {
        return ${componentColor('md.comp.filled-text-field.error.hover.trailing-icon')};
      }${componentColor('md.comp.filled-text-field.error.trailing-icon') == componentColor('md.comp.filled-text-field.error.focus.trailing-icon') ? '' : '''
      if (states.contains(MaterialState.focused)) {
        return ${componentColor('md.comp.filled-text-field.error.focus.trailing-icon')};
      }'''}
      return ${componentColor('md.comp.filled-text-field.error.trailing-icon')};
    }${componentColor('md.comp.filled-text-field.hover.trailing-icon') == componentColor('md.comp.filled-text-field.trailing-icon') ? '' : '''
    if (states.contains(MaterialState.hovered)) {
      return ${componentColor('md.comp.filled-text-field.hover.trailing-icon')};
    }'''}${componentColor('md.comp.filled-text-field.focus.trailing-icon') == componentColor('md.comp.filled-text-field.trailing-icon') ? '' : '''
    if (states.contains(MaterialState.focused)) {
      return ${componentColor('md.comp.filled-text-field.focus.trailing-icon')};
    }'''}
    return ${componentColor('md.comp.filled-text-field.trailing-icon')};
  });

  @override
  TextStyle? get labelStyle => MaterialStateTextStyle.resolveWith((Set<MaterialState> states) {
    final TextStyle textStyle = ${textStyle("md.comp.filled-text-field.label-text")} ?? const TextStyle();
    if (states.contains(MaterialState.disabled)) {
      return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.disabled.label-text')});
    }
    if (states.contains(MaterialState.error)) {
      if (states.contains(MaterialState.focused)) {
        return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.error.focus.label-text')});
      }
      if (states.contains(MaterialState.hovered)) {
        return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.error.hover.label-text')});
      }
      return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.error.label-text')});
    }
    if (states.contains(MaterialState.focused)) {
      return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.focus.label-text')});
    }
    if (states.contains(MaterialState.hovered)) {
      return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.hover.label-text')});
    }
    return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.label-text')});
  });

  @override
  TextStyle? get floatingLabelStyle => MaterialStateTextStyle.resolveWith((Set<MaterialState> states) {
    final TextStyle textStyle = ${textStyle("md.comp.filled-text-field.label-text")} ?? const TextStyle();
    if (states.contains(MaterialState.disabled)) {
      return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.disabled.label-text')});
    }
    if (states.contains(MaterialState.error)) {
      if (states.contains(MaterialState.focused)) {
        return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.error.focus.label-text')});
      }
      if (states.contains(MaterialState.hovered)) {
        return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.error.hover.label-text')});
      }
      return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.error.label-text')});
    }
    if (states.contains(MaterialState.focused)) {
      return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.focus.label-text')});
    }
    if (states.contains(MaterialState.hovered)) {
      return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.hover.label-text')});
    }
    return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.label-text')});
  });

  @override
  TextStyle? get helperStyle => MaterialStateTextStyle.resolveWith((Set<MaterialState> states) {
    final TextStyle textStyle = ${textStyle("md.comp.filled-text-field.supporting-text")} ?? const TextStyle();
    if (states.contains(MaterialState.disabled)) {
      return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.disabled.supporting-text')});
    }${componentColor('md.comp.filled-text-field.focus.supporting-text') == componentColor('md.comp.filled-text-field.supporting-text') ? '' : '''
    if (states.contains(MaterialState.focused)) {
      return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.focus.supporting-text')});
    }'''}${componentColor('md.comp.filled-text-field.hover.supporting-text') == componentColor('md.comp.filled-text-field.supporting-text') ? '' : '''
    if (states.contains(MaterialState.hovered)) {
      return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.hover.supporting-text')});
    }'''}
    return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.supporting-text')});
  });

  @override
  TextStyle? get errorStyle => MaterialStateTextStyle.resolveWith((Set<MaterialState> states) {
    final TextStyle textStyle = ${textStyle("md.comp.filled-text-field.supporting-text")} ?? const TextStyle();${componentColor('md.comp.filled-text-field.error.hover.supporting-text') == componentColor('md.comp.filled-text-field.error.supporting-text') ? '' : '''
    if (states.contains(MaterialState.focused)) {
      return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.error.focus.supporting-text')});
    }'''}${componentColor('md.comp.filled-text-field.error.focus.supporting-text') == componentColor('md.comp.filled-text-field.error.supporting-text') ? '' : '''
    if (states.contains(MaterialState.hovered)) {
      return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.error.hover.supporting-text')});
    }'''}
    return textStyle.copyWith(color: ${componentColor('md.comp.filled-text-field.error.supporting-text')});
  });
}
''';

  /// Generate a [BorderSide] for the given components.
  String mergedBorder(String componentToken1, String componentToken2) {
    final String borderColor =
        componentColor(componentToken1) != 'null'
            ? componentColor(componentToken1)
            : componentColor(componentToken2);
    final double width =
        (getToken('$componentToken1.width', optional: true) ??
                getToken('$componentToken1.height', optional: true) ??
                getToken('$componentToken2.width', optional: true) ??
                getToken('$componentToken2.height', optional: true) ??
                1.0)
            as double;
    return 'BorderSide(color: $borderColor${width != 1.0 ? ", width: $width" : ""})';
  }
}
