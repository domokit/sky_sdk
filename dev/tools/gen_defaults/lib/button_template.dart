// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class ButtonTemplate extends TokenTemplate {
  const ButtonTemplate(
    this.tokenGroup,
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
  });

  final String tokenGroup;

  String _backgroundColor() {
    if (tokenAvailable('$tokenGroup.container.color')) {
      return '''

    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return ${componentColor('$tokenGroup.disabled.container')};
      }
      return ${componentColor('$tokenGroup.container')};
    })''';
    }
    return '''

    const MaterialStatePropertyAll<Color>(Colors.transparent)''';
  }

  String _elevation() {
    if (tokenAvailable('$tokenGroup.container.elevation')) {
      return '''

    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return ${elevation("$tokenGroup.disabled.container")};
      }
      if (states.contains(MaterialState.pressed)) {
        return ${elevation("$tokenGroup.pressed.container")};
      }
      if (states.contains(MaterialState.hovered)) {
        return ${elevation("$tokenGroup.hover.container")};
      }
      if (states.contains(MaterialState.focused)) {
        return ${elevation("$tokenGroup.focus.container")};
      }
      return ${elevation("$tokenGroup.container")};
    })''';
    }
    return '''

    const MaterialStatePropertyAll<double>(0.0)''';
  }

  String _elevationColor(String token) {
    if (tokenAvailable(token)) {
      return 'MaterialStatePropertyAll<Color>(${color(token)})';
    } else {
      return 'const MaterialStatePropertyAll<Color>(Colors.transparent)';
    }
  }

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends ButtonStyle {
  _${blockName}DefaultsM3(this.context)
   : super(
       animationDuration: kThemeChangeDuration,
       enableFeedback: true,
       alignment: Alignment.center,
     );

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;

  @override
  MaterialStateProperty<TextStyle?> get textStyle =>
    MaterialStatePropertyAll<TextStyle?>(${textStyle("$tokenGroup.label-text")});

  @override
  MaterialStateProperty<Color?>? get backgroundColor =>${_backgroundColor()};

  @override
  MaterialStateProperty<Color?>? get foregroundColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return ${componentColor('$tokenGroup.disabled.label-text')};
      }
      return ${componentColor('$tokenGroup.label-text')};
    });

  @override
  MaterialStateProperty<Color?>? get overlayColor =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.pressed)) {
        return ${componentColor('$tokenGroup.pressed.state-layer')};
      }
      if (states.contains(MaterialState.hovered)) {
        return ${componentColor('$tokenGroup.hover.state-layer')};
      }
      if (states.contains(MaterialState.focused)) {
        return ${componentColor('$tokenGroup.focus.state-layer')};
      }
      return null;
    });

  @override
  MaterialStateProperty<Color>? get shadowColor =>
    ${_elevationColor("$tokenGroup.container.shadow-color")};

  @override
  MaterialStateProperty<Color>? get surfaceTintColor =>
    ${_elevationColor("$tokenGroup.container.surface-tint-layer.color")};

  @override
  MaterialStateProperty<double>? get elevation =>${_elevation()};

  @override
  MaterialStateProperty<EdgeInsetsGeometry>? get padding =>
    MaterialStatePropertyAll<EdgeInsetsGeometry>(_scaledPadding(context));

  @override
  MaterialStateProperty<Size>? get minimumSize =>
    const MaterialStatePropertyAll<Size>(Size(64.0, ${getToken("$tokenGroup.container.height")}));

  // No default fixedSize

  @override
  MaterialStateProperty<double>? get iconSize =>
    const MaterialStatePropertyAll<double>(${getToken("$tokenGroup.with-icon.icon.size")});

  @override
  MaterialStateProperty<Color>? get iconColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return ${color('$tokenGroup.with-icon.disabled.icon.color')}.withOpacity(${opacity("$tokenGroup.with-icon.disabled.icon.opacity")});
      }
      if (states.contains(MaterialState.pressed)) {
        return ${color('$tokenGroup.with-icon.pressed.icon.color')};
      }
      if (states.contains(MaterialState.hovered)) {
        return ${color('$tokenGroup.with-icon.hover.icon.color')};
      }
      if (states.contains(MaterialState.focused)) {
        return ${color('$tokenGroup.with-icon.focus.icon.color')};
      }
      return ${color('$tokenGroup.with-icon.icon.color')};
    });
  }

  @override
  MaterialStateProperty<Size>? get maximumSize =>
    const MaterialStatePropertyAll<Size>(Size.infinite);

${tokenAvailable("$tokenGroup.outline.color") ? '''
  @override
  MaterialStateProperty<BorderSide>? get side =>
    MaterialStateProperty.resolveWith((Set<MaterialState> states) {
    if (states.contains(MaterialState.disabled)) {
      return ${border("$tokenGroup.disabled.outline")};
    }
    if (states.contains(MaterialState.focused)) {
      return ${border('$tokenGroup.focus.outline')};
    }
    return ${border("$tokenGroup.outline")};
  });''' : '''
  // No default side'''}

  @override
  MaterialStateProperty<OutlinedBorder>? get shape =>
    const MaterialStatePropertyAll<OutlinedBorder>(${shape("$tokenGroup.container", '')});

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
''';
}
