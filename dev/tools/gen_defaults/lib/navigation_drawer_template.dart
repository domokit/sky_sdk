// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class NavigationDrawerTemplate extends TokenTemplate {
  const NavigationDrawerTemplate(
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
    super.textThemePrefix = '_textTheme.',
  });

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends NavigationDrawerThemeData {
  _${blockName}DefaultsM3(this.context)
    : super(
        elevation: ${elevation("md.comp.navigation-drawer.modal.container")},
        tileHeight: ${getToken("md.comp.navigation-drawer.active-indicator.height")},
        indicatorShape: ${shape("md.comp.navigation-drawer.active-indicator")},
        indicatorSize: const Size(${getToken("md.comp.navigation-drawer.active-indicator.width")}, ${getToken("md.comp.navigation-drawer.active-indicator.height")}),
      );

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;
  late final TextTheme _textTheme = Theme.of(context).textTheme;

  @override
  Color? get backgroundColor => ${componentColor("md.comp.navigation-drawer.modal.container")};

  @override
  Color? get surfaceTintColor => ${colorOrTransparent("md.comp.navigation-drawer.container.surface-tint-layer.color")};

  @override
  Color? get shadowColor => ${colorOrTransparent("md.comp.navigation-drawer.container.shadow-color")};

  @override
  Color? get indicatorColor => ${componentColor("md.comp.navigation-drawer.active-indicator")};

  @override
  MaterialStateProperty<IconThemeData?>? get iconTheme {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      return IconThemeData(
        size: ${getToken("md.comp.navigation-drawer.icon.size")},
        color: states.contains(MaterialState.disabled)
          ? _colors.onSurfaceVariant.withOpacity(0.38)
          : states.contains(MaterialState.selected)
            ? ${componentColor("md.comp.navigation-drawer.active.icon")}
            : ${componentColor("md.comp.navigation-drawer.inactive.icon")},
      );
    });
  }

  @override
  MaterialStateProperty<TextStyle?>? get labelTextStyle {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      final TextStyle style = ${textStyle("md.comp.navigation-drawer.label-text")}!;
      return style.apply(
        color: states.contains(MaterialState.disabled)
          ? _colors.onSurfaceVariant.withOpacity(0.38)
          : states.contains(MaterialState.selected)
            ? ${componentColor("md.comp.navigation-drawer.active.label-text")}
            : ${componentColor("md.comp.navigation-drawer.inactive.label-text")},
      );
    });
  }
}
''';
}
