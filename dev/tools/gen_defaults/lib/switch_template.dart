// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class SwitchTemplate extends TokenTemplate {
  const SwitchTemplate(
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
  });

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends SwitchThemeData {
  _${blockName}DefaultsM3(this.context);

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;

  @override
  MaterialStateProperty<Color> get thumbColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        if (states.contains(MaterialState.selected)) {
          return ${componentColor('md.comp.switch.disabled.selected.handle')};
        }
        return ${componentColor('md.comp.switch.disabled.unselected.handle')};
      }
      if (states.contains(MaterialState.selected)) {
        if (states.contains(MaterialState.pressed)) {
          return ${componentColor('md.comp.switch.selected.pressed.handle')};
        }
        if (states.contains(MaterialState.hovered)) {
          return ${componentColor('md.comp.switch.selected.hover.handle')};
        }
        if (states.contains(MaterialState.focused)) {
          return ${componentColor('md.comp.switch.selected.focus.handle')};
        }
        return ${componentColor('md.comp.switch.selected.handle')};
      }
      if (states.contains(MaterialState.pressed)) {
        return ${componentColor('md.comp.switch.unselected.pressed.handle')};
      }
      if (states.contains(MaterialState.hovered)) {
        return ${componentColor('md.comp.switch.unselected.hover.handle')};
      }
      if (states.contains(MaterialState.focused)) {
        return ${componentColor('md.comp.switch.unselected.focus.handle')};
      }
      return ${componentColor('md.comp.switch.unselected.handle')};
    });
  }

  @override
  MaterialStateProperty<Color> get trackColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        if (states.contains(MaterialState.selected)) {
          return ${componentColor('md.comp.switch.disabled.selected.track')}.withOpacity(${opacity('md.comp.switch.disabled.track.opacity')});
        }
        return ${componentColor('md.comp.switch.disabled.unselected.track')}.withOpacity(${opacity('md.comp.switch.disabled.track.opacity')});
      }
      if (states.contains(MaterialState.selected)) {
        if (states.contains(MaterialState.pressed)) {
          return ${componentColor('md.comp.switch.selected.pressed.track')};
        }
        if (states.contains(MaterialState.hovered)) {
          return ${componentColor('md.comp.switch.selected.hover.track')};
        }
        if (states.contains(MaterialState.focused)) {
          return ${componentColor('md.comp.switch.selected.focus.track')};
        }
        return ${componentColor('md.comp.switch.selected.track')};
      }
      if (states.contains(MaterialState.pressed)) {
        return ${componentColor('md.comp.switch.unselected.pressed.track')};
      }
      if (states.contains(MaterialState.hovered)) {
        return ${componentColor('md.comp.switch.unselected.hover.track')};
      }
      if (states.contains(MaterialState.focused)) {
        return ${componentColor('md.comp.switch.unselected.focus.track')};
      }
      return ${componentColor('md.comp.switch.unselected.track')};
    });
  }

  @override
  MaterialStateProperty<Color?> get trackOutlineColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return Colors.transparent;
      }
      if (states.contains(MaterialState.disabled)) {
        return ${componentColor('md.comp.switch.disabled.unselected.track.outline')}.withOpacity(${opacity('md.comp.switch.disabled.track.opacity')});
      }
      return ${componentColor('md.comp.switch.unselected.track.outline')};
    });
  }

  @override
  MaterialStateProperty<Color?> get overlayColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        if (states.contains(MaterialState.pressed)) {
          return ${componentColor('md.comp.switch.selected.pressed.state-layer')};
        }
        if (states.contains(MaterialState.hovered)) {
          return ${componentColor('md.comp.switch.selected.hover.state-layer')};
        }
        if (states.contains(MaterialState.focused)) {
          return ${componentColor('md.comp.switch.selected.focus.state-layer')};
        }
        return null;
      }
      if (states.contains(MaterialState.pressed)) {
        return ${componentColor('md.comp.switch.unselected.pressed.state-layer')};
      }
      if (states.contains(MaterialState.hovered)) {
        return ${componentColor('md.comp.switch.unselected.hover.state-layer')};
      }
      if (states.contains(MaterialState.focused)) {
        return ${componentColor('md.comp.switch.unselected.focus.state-layer')};
      }
      return null;
    });
  }

  @override
  MaterialStateProperty<MouseCursor> get mouseCursor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states)
      => MaterialStateMouseCursor.clickable.resolve(states));
  }

  @override
  MaterialStatePropertyAll<double> get trackOutlineWidth => const MaterialStatePropertyAll<double>(${getToken('md.comp.switch.track.outline.width')});

  @override
  double get splashRadius => ${getToken('md.comp.switch.state-layer.size')} / 2;

  @override
  EdgeInsetsGeometry? get padding => const EdgeInsets.symmetric(horizontal: 4);
}

class _SwitchConfigM3 with _SwitchConfig {
  _SwitchConfigM3(this.context)
    : _colors = Theme.of(context).colorScheme;

  BuildContext context;
  final ColorScheme _colors;

  static const double iconSize = ${getToken('md.comp.switch.unselected.icon.size')};

  @override
  double get activeThumbRadius => ${getToken('md.comp.switch.selected.handle.width')} / 2;

  @override
  MaterialStateProperty<Color> get iconColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        if (states.contains(MaterialState.selected)) {
          return ${componentColor('md.comp.switch.disabled.selected.icon')};
        }
        return ${componentColor('md.comp.switch.disabled.unselected.icon')};
      }
      if (states.contains(MaterialState.selected)) {
        if (states.contains(MaterialState.pressed)) {
          return ${componentColor('md.comp.switch.selected.pressed.icon')};
        }
        if (states.contains(MaterialState.hovered)) {
          return ${componentColor('md.comp.switch.selected.hover.icon')};
        }
        if (states.contains(MaterialState.focused)) {
          return ${componentColor('md.comp.switch.selected.focus.icon')};
        }
        return ${componentColor('md.comp.switch.selected.icon')};
      }
      if (states.contains(MaterialState.pressed)) {
        return ${componentColor('md.comp.switch.unselected.pressed.icon')};
      }
      if (states.contains(MaterialState.hovered)) {
        return ${componentColor('md.comp.switch.unselected.hover.icon')};
      }
      if (states.contains(MaterialState.focused)) {
        return ${componentColor('md.comp.switch.unselected.focus.icon')};
      }
      return ${componentColor('md.comp.switch.unselected.icon')};
    });
  }

  @override
  double get inactiveThumbRadius => ${getToken('md.comp.switch.unselected.handle.width')} / 2;

  @override
  double get pressedThumbRadius => ${getToken('md.comp.switch.pressed.handle.width')} / 2;

  @override
  double get switchHeight => switchMinSize.height + 8.0;

  @override
  double get switchHeightCollapsed => switchMinSize.height;

  @override
  double get switchWidth => 52.0;

  @override
  double get thumbRadiusWithIcon => ${getToken('md.comp.switch.with-icon.handle.width')} / 2;

  @override
  List<BoxShadow>? get thumbShadow => kElevationToShadow[0];

  @override
  double get trackHeight => ${getToken('md.comp.switch.track.height')};

  @override
  double get trackWidth => ${getToken('md.comp.switch.track.width')};

  // The thumb size at the middle of the track. Hand coded default based on the animation specs.
  @override
  Size get transitionalThumbSize => const Size(34, 22);

  // Hand coded default based on the animation specs.
  @override
  int get toggleDuration => 300;

  // Hand coded default based on the animation specs.
  @override
  double? get thumbOffset => null;

  @override
  Size get switchMinSize => const Size(kMinInteractiveDimension, kMinInteractiveDimension - 8.0);
}
''';
}
