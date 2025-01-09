// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class PopupMenuTemplate extends TokenTemplate {
  const PopupMenuTemplate(
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
    super.textThemePrefix = '_textTheme.',
  });

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends PopupMenuThemeData {
  _${blockName}DefaultsM3(this.context)
    : super(elevation: ${elevation('md.comp.menu.container')});

  final BuildContext context;
  late final ThemeData _theme = Theme.of(context);
  late final ColorScheme _colors = _theme.colorScheme;
  late final TextTheme _textTheme = _theme.textTheme;

  @override MaterialStateProperty<TextStyle?>? get labelTextStyle {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
    // TODO(quncheng): Update this hard-coded value to use the latest tokens.
    final TextStyle style = _textTheme.labelLarge!;
      if (states.contains(MaterialState.disabled)) {
        return style.apply(color: ${componentColor('md.comp.list.list-item.disabled.label-text')});
      }
      return style.apply(color: ${componentColor('md.comp.list.list-item.label-text')});
    });
  }

  @override
  Color? get color => ${componentColor('md.comp.menu.container')};

  @override
  Color? get shadowColor => ${color("md.comp.menu.container.shadow-color")};

  @override
  Color? get surfaceTintColor => ${colorOrTransparent("md.comp.menu.container.surface-tint-layer.color")};

  @override
  ShapeBorder? get shape => ${shape("md.comp.menu.container")};

  // TODO(bleroux): This is taken from https://m3.material.io/components/menus/specs
  // Update this when the token is available.
  @override
  EdgeInsets? get menuPadding => const EdgeInsets.symmetric(vertical: 8.0);

  // TODO(tahatesser): This is taken from https://m3.material.io/components/menus/specs
  // Update this when the token is available.
  static EdgeInsets menuItemPadding  = const EdgeInsets.symmetric(horizontal: 12.0);
}''';
}
