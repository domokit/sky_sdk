// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class MenuTemplate extends TokenTemplate {
  const MenuTemplate(
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
  });

  @override
  String generate() => '''
class _MenuBarDefaultsM3 extends MenuStyle {
  _MenuBarDefaultsM3(this.context)
    : super(
      elevation: const MaterialStatePropertyAll<double?>(${elevation('md.comp.menu.container')}),
      shape: const MaterialStatePropertyAll<OutlinedBorder>(_defaultMenuBorder),
      alignment: AlignmentDirectional.bottomStart,
    );

  static const RoundedRectangleBorder _defaultMenuBorder =
    ${shape('md.comp.menu.container', '')};

  final BuildContext context;

  late final ColorScheme _colors = Theme.of(context).colorScheme;

  @override
  MaterialStateProperty<Color?> get backgroundColor {
    return MaterialStatePropertyAll<Color?>(${componentColor('md.comp.menu.container')});
  }

  @override
  MaterialStateProperty<Color?>? get shadowColor {
    return MaterialStatePropertyAll<Color?>(${color('md.comp.menu.container.shadow-color')});
  }

  @override
  MaterialStateProperty<Color?>? get surfaceTintColor {
    return const MaterialStatePropertyAll<Color?>(${colorOrTransparent('md.comp.menu.container.surface-tint-layer')});
  }

  @override
  MaterialStateProperty<EdgeInsetsGeometry?>? get padding {
    return const MaterialStatePropertyAll<EdgeInsetsGeometry>(
      EdgeInsetsDirectional.symmetric(
        horizontal: _kTopLevelMenuHorizontalMinPadding
      ),
    );
  }

  @override
  VisualDensity get visualDensity => Theme.of(context).visualDensity;
}

class _MenuButtonDefaultsM3 extends ButtonStyle {
  _MenuButtonDefaultsM3(this.context)
    : super(
      animationDuration: kThemeChangeDuration,
      enableFeedback: true,
      alignment: AlignmentDirectional.centerStart,
    );

  final BuildContext context;

  late final ColorScheme _colors = Theme.of(context).colorScheme;
  late final TextTheme _textTheme = Theme.of(context).textTheme;

  @override
  MaterialStateProperty<Color?>? get backgroundColor {
    return ButtonStyleButton.allOrNull<Color>(Colors.transparent);
  }

  // No default shadow color

  // No default surface tint color

  @override
  MaterialStateProperty<double>? get elevation {
    return ButtonStyleButton.allOrNull<double>(0.0);
  }

  @override
  MaterialStateProperty<Color?>? get foregroundColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return ${componentColor('md.comp.list.list-item.disabled.label-text')};
      }
      if (states.contains(MaterialState.pressed)) {
        return ${componentColor('md.comp.list.list-item.pressed.label-text')};
      }
      if (states.contains(MaterialState.hovered)) {
        return ${componentColor('md.comp.list.list-item.hover.label-text')};
      }
      if (states.contains(MaterialState.focused)) {
        return ${componentColor('md.comp.list.list-item.focus.label-text')};
      }
      return ${componentColor('md.comp.list.list-item.label-text')};
    });
  }

  @override
  MaterialStateProperty<Color?>? get iconColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return ${componentColor('md.comp.list.list-item.disabled.leading-icon')};
      }
      if (states.contains(MaterialState.pressed)) {
        return ${componentColor('md.comp.list.list-item.pressed.leading-icon.icon')};
      }
      if (states.contains(MaterialState.hovered)) {
        return ${componentColor('md.comp.list.list-item.hover.leading-icon.icon')};
      }
      if (states.contains(MaterialState.focused)) {
        return ${componentColor('md.comp.list.list-item.focus.leading-icon.icon')};
      }
      return ${componentColor('md.comp.list.list-item.leading-icon')};
    });
  }

  // No default fixedSize

  @override
  MaterialStateProperty<double>? get iconSize {
    return const MaterialStatePropertyAll<double>(${getToken("md.comp.list.list-item.leading-icon.size")});
  }

  @override
  MaterialStateProperty<Size>? get maximumSize {
    return ButtonStyleButton.allOrNull<Size>(Size.infinite);
  }

  @override
  MaterialStateProperty<Size>? get minimumSize {
    return ButtonStyleButton.allOrNull<Size>(const Size(64.0, 48.0));
  }

  @override
  MaterialStateProperty<MouseCursor?>? get mouseCursor {
    return MaterialStateProperty.resolveWith(
      (Set<MaterialState> states) {
        if (states.contains(MaterialState.disabled)) {
          return SystemMouseCursors.basic;
        }
        return SystemMouseCursors.click;
      },
    );
  }

  @override
  MaterialStateProperty<Color?>? get overlayColor {
    return MaterialStateProperty.resolveWith(
      (Set<MaterialState> states) {
        if (states.contains(MaterialState.pressed)) {
          return ${componentColor('md.comp.list.list-item.pressed.state-layer')};
        }
        if (states.contains(MaterialState.hovered)) {
          return ${componentColor('md.comp.list.list-item.hover.state-layer')};
        }
        if (states.contains(MaterialState.focused)) {
          return ${componentColor('md.comp.list.list-item.focus.state-layer')};
        }
        return Colors.transparent;
      },
    );
  }

  @override
  MaterialStateProperty<EdgeInsetsGeometry>? get padding {
    return ButtonStyleButton.allOrNull<EdgeInsetsGeometry>(_scaledPadding(context));
  }

  // No default side

  @override
  MaterialStateProperty<OutlinedBorder>? get shape {
    return ButtonStyleButton.allOrNull<OutlinedBorder>(const RoundedRectangleBorder());
  }

  @override
  InteractiveInkFeatureFactory? get splashFactory => Theme.of(context).splashFactory;

  @override
  MaterialTapTargetSize? get tapTargetSize => Theme.of(context).materialTapTargetSize;

  @override
  MaterialStateProperty<TextStyle?> get textStyle {
    // TODO(tahatesser): This is taken from https://m3.material.io/components/menus/specs
    // Update this when the token is available.
    return MaterialStatePropertyAll<TextStyle?>(_textTheme.labelLarge);
  }

  @override
  VisualDensity? get visualDensity => Theme.of(context).visualDensity;

  // The horizontal padding number comes from the spec.
  EdgeInsetsGeometry _scaledPadding(BuildContext context) {
    VisualDensity visualDensity = Theme.of(context).visualDensity;
    // When horizontal VisualDensity is greater than zero, set it to zero
    // because the [ButtonStyleButton] has already handle the padding based on the density.
    // However, the [ButtonStyleButton] doesn't allow the [VisualDensity] adjustment
    // to reduce the width of the left/right padding, so we need to handle it here if
    // the density is less than zero, such as on desktop platforms.
    if (visualDensity.horizontal > 0) {
      visualDensity = VisualDensity(vertical: visualDensity.vertical);
    }
    // Since the threshold paddings used below are empirical values determined
    // at a font size of 14.0, 14.0 is used as the base value for scaling the
    // padding.
    final double fontSize = Theme.of(context).textTheme.labelLarge?.fontSize ?? 14.0;
    final double fontSizeRatio = MediaQuery.textScalerOf(context).scale(fontSize) / 14.0;
    return ButtonStyleButton.scaledPadding(
      EdgeInsets.symmetric(horizontal: math.max(
        _kMenuViewPadding,
        _kLabelItemDefaultSpacing + visualDensity.baseSizeAdjustment.dx,
      )),
      EdgeInsets.symmetric(horizontal: math.max(
        _kMenuViewPadding,
        8 + visualDensity.baseSizeAdjustment.dx,
      )),
      const EdgeInsets.symmetric(horizontal: _kMenuViewPadding),
      fontSizeRatio,
    );
  }
}

class _MenuDefaultsM3 extends MenuStyle {
  _MenuDefaultsM3(this.context)
    : super(
      elevation: const MaterialStatePropertyAll<double?>(${elevation('md.comp.menu.container')}),
      shape: const MaterialStatePropertyAll<OutlinedBorder>(_defaultMenuBorder),
      alignment: AlignmentDirectional.topEnd,
    );

  static const RoundedRectangleBorder _defaultMenuBorder =
    ${shape('md.comp.menu.container', '')};

  final BuildContext context;

  late final ColorScheme _colors = Theme.of(context).colorScheme;

  @override
  MaterialStateProperty<Color?> get backgroundColor {
    return MaterialStatePropertyAll<Color?>(${componentColor('md.comp.menu.container')});
  }

  @override
  MaterialStateProperty<Color?>? get surfaceTintColor {
    return ${componentColor('md.comp.menu.container.surface-tint-layer') == 'null' ? 'const MaterialStatePropertyAll<Color?>(Colors.transparent)' : 'MaterialStatePropertyAll<Color?>(${componentColor('md.comp.menu.container.surface-tint-layer')})'};
  }

  @override
  MaterialStateProperty<Color?>? get shadowColor {
    return MaterialStatePropertyAll<Color?>(${color('md.comp.menu.container.shadow-color')});
  }

  @override
  MaterialStateProperty<EdgeInsetsGeometry?>? get padding {
    return const MaterialStatePropertyAll<EdgeInsetsGeometry>(
      EdgeInsetsDirectional.symmetric(vertical: _kMenuVerticalMinPadding),
    );
  }

  @override
  VisualDensity get visualDensity => Theme.of(context).visualDensity;
}
''';
}
