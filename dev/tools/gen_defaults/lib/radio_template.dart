// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class RadioTemplate extends TokenTemplate {
  const RadioTemplate(
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
  });

  @override
  String generate() => '''
class _RadioDefaultsM3 extends RadioThemeData {
  _RadioDefaultsM3(this.context);

  final BuildContext context;
  late final ThemeData _theme = Theme.of(context);
  late final ColorScheme _colors = _theme.colorScheme;

  @override
  MaterialStateProperty<Color> get fillColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        if (states.contains(MaterialState.disabled)) {
          return ${componentColor('md.comp.radio-button.disabled.selected.icon')};
        }
        if (states.contains(MaterialState.pressed)) {
          return ${componentColor('md.comp.radio-button.selected.pressed.icon')};
        }
        if (states.contains(MaterialState.hovered)) {
          return ${componentColor('md.comp.radio-button.selected.hover.icon')};
        }
        if (states.contains(MaterialState.focused)) {
          return ${componentColor('md.comp.radio-button.selected.focus.icon')};
        }
        return ${componentColor('md.comp.radio-button.selected.icon')};
      }
      if (states.contains(MaterialState.disabled)) {
        return ${componentColor('md.comp.radio-button.disabled.unselected.icon')};
      }
      if (states.contains(MaterialState.pressed)) {
        return ${componentColor('md.comp.radio-button.unselected.pressed.icon')};
      }
      if (states.contains(MaterialState.hovered)) {
        return ${componentColor('md.comp.radio-button.unselected.hover.icon')};
      }
      if (states.contains(MaterialState.focused)) {
        return ${componentColor('md.comp.radio-button.unselected.focus.icon')};
      }
      return ${componentColor('md.comp.radio-button.unselected.icon')};
    });
  }

  @override
  MaterialStateProperty<Color> get overlayColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        if (states.contains(MaterialState.pressed)) {
          return ${componentColor('md.comp.radio-button.selected.pressed.state-layer')};
        }
        if (states.contains(MaterialState.hovered)) {
          return ${componentColor('md.comp.radio-button.selected.hover.state-layer')};
        }
        if (states.contains(MaterialState.focused)) {
          return ${componentColor('md.comp.radio-button.selected.focus.state-layer')};
        }
        return Colors.transparent;
      }
      if (states.contains(MaterialState.pressed)) {
        return ${componentColor('md.comp.radio-button.unselected.pressed.state-layer')};
      }
      if (states.contains(MaterialState.hovered)) {
        return ${componentColor('md.comp.radio-button.unselected.hover.state-layer')};
      }
      if (states.contains(MaterialState.focused)) {
        return ${componentColor('md.comp.radio-button.unselected.focus.state-layer')};
      }
      return Colors.transparent;
    });
  }

  @override
  MaterialTapTargetSize get materialTapTargetSize => _theme.materialTapTargetSize;

  @override
  VisualDensity get visualDensity => _theme.visualDensity;
}
''';
}
