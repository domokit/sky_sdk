// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class SegmentedButtonTemplate extends TokenTemplate {
  const SegmentedButtonTemplate(
    this.tokenGroup,
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
  });

  final String tokenGroup;

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

  String _stateColor(String componentToken, String type, String state) {
    final String baseColor = color('$componentToken.$type.$state.state-layer.color', '');
    if (baseColor.isEmpty) {
      return 'null';
    }
    final String opacity = _layerOpacity('$componentToken.$state.state-layer.opacity');
    return '$baseColor$opacity';
  }

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends SegmentedButtonThemeData {
  _${blockName}DefaultsM3(this.context);
  final BuildContext context;
  late final ThemeData _theme = Theme.of(context);
  late final ColorScheme _colors = _theme.colorScheme;
  @override ButtonStyle? get style {
    return ButtonStyle(
      textStyle: MaterialStatePropertyAll<TextStyle?>(${textStyle('$tokenGroup.label-text')}),
      backgroundColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
        if (states.contains(MaterialState.disabled)) {
          return ${componentColor('$tokenGroup.disabled')};
        }
        if (states.contains(MaterialState.selected)) {
          return ${componentColor('$tokenGroup.selected.container')};
        }
        return ${componentColor('$tokenGroup.unselected.container')};
      }),
      foregroundColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
        if (states.contains(MaterialState.disabled)) {
          return ${componentColor('$tokenGroup.disabled.label-text')};
        }
        if (states.contains(MaterialState.selected)) {
          if (states.contains(MaterialState.pressed)) {
            return ${componentColor('$tokenGroup.selected.pressed.label-text')};
          }
          if (states.contains(MaterialState.hovered)) {
            return ${componentColor('$tokenGroup.selected.hover.label-text')};
          }
          if (states.contains(MaterialState.focused)) {
            return ${componentColor('$tokenGroup.selected.focus.label-text')};
          }
          return ${componentColor('$tokenGroup.selected.label-text')};
        } else {
          if (states.contains(MaterialState.pressed)) {
            return ${componentColor('$tokenGroup.unselected.pressed.label-text')};
          }
          if (states.contains(MaterialState.hovered)) {
            return ${componentColor('$tokenGroup.unselected.hover.label-text')};
          }
          if (states.contains(MaterialState.focused)) {
            return ${componentColor('$tokenGroup.unselected.focus.label-text')};
          }
          return ${componentColor('$tokenGroup.unselected.label-text')};
        }
      }),
      overlayColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
        if (states.contains(MaterialState.selected)) {
          if (states.contains(MaterialState.pressed)) {
            return ${_stateColor(tokenGroup, 'selected', 'pressed')};
          }
          if (states.contains(MaterialState.hovered)) {
            return ${_stateColor(tokenGroup, 'selected', 'hover')};
          }
          if (states.contains(MaterialState.focused)) {
            return ${_stateColor(tokenGroup, 'selected', 'focus')};
          }
        } else {
          if (states.contains(MaterialState.pressed)) {
            return ${_stateColor(tokenGroup, 'unselected', 'pressed')};
          }
          if (states.contains(MaterialState.hovered)) {
            return ${_stateColor(tokenGroup, 'unselected', 'hover')};
          }
          if (states.contains(MaterialState.focused)) {
            return ${_stateColor(tokenGroup, 'unselected', 'focus')};
          }
        }
        return null;
      }),
      surfaceTintColor: const MaterialStatePropertyAll<Color>(Colors.transparent),
      elevation: const MaterialStatePropertyAll<double>(0),
      iconSize: const MaterialStatePropertyAll<double?>(${getToken('$tokenGroup.with-icon.icon.size')}),
      side: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
        if (states.contains(MaterialState.disabled)) {
          return ${border("$tokenGroup.disabled.outline")};
        }
        return ${border("$tokenGroup.outline")};
      }),
      shape: const MaterialStatePropertyAll<OutlinedBorder>(${shape(tokenGroup, '')}),
      minimumSize: const MaterialStatePropertyAll<Size?>(Size.fromHeight(${getToken('$tokenGroup.container.height')})),
    );
  }
  @override
  Widget? get selectedIcon => const Icon(Icons.check);

  static WidgetStateProperty<Color?> resolveStateColor(
    Color? unselectedColor,
    Color? selectedColor,
    Color? overlayColor,
  ) {
    final Color? selected = overlayColor ?? selectedColor;
    final Color? unselected = overlayColor ?? unselectedColor;
    return WidgetStateProperty<Color?>.fromMap(
      <WidgetStatesConstraint, Color?>{
        WidgetState.selected & WidgetState.pressed: selected?.withOpacity(0.1),
        WidgetState.selected & WidgetState.hovered: selected?.withOpacity(0.08),
        WidgetState.selected & WidgetState.focused: selected?.withOpacity(0.1),
        WidgetState.pressed: unselected?.withOpacity(0.1),
        WidgetState.hovered: unselected?.withOpacity(0.08),
        WidgetState.focused: unselected?.withOpacity(0.1),
        WidgetState.any: Colors.transparent,
      },
    );
  }
}
''';
}
