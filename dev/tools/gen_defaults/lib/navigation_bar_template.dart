// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class NavigationBarTemplate extends TokenTemplate {
  const NavigationBarTemplate(
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
    super.textThemePrefix = '_textTheme.',
  });

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends NavigationBarThemeData {
  _${blockName}DefaultsM3(this.context)
    : super(
        height: ${getToken("md.comp.navigation-bar.container.height")},
        elevation: ${elevation("md.comp.navigation-bar.container")},
        labelBehavior: NavigationDestinationLabelBehavior.alwaysShow,
      );

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;
  late final TextTheme _textTheme = Theme.of(context).textTheme;

  @override
  Color? get backgroundColor => ${componentColor("md.comp.navigation-bar.container")};

  @override
  Color? get shadowColor => ${colorOrTransparent("md.comp.navigation-bar.container.shadow-color")};

  @override
  Color? get surfaceTintColor => ${colorOrTransparent("md.comp.navigation-bar.container.surface-tint-layer.color")};

  @override
  MaterialStateProperty<IconThemeData?>? get iconTheme {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      return IconThemeData(
        size: ${getToken("md.comp.navigation-bar.icon.size")},
        color: states.contains(MaterialState.disabled)
          ? _colors.onSurfaceVariant.withOpacity(0.38)
          : states.contains(MaterialState.selected)
            ? ${componentColor("md.comp.navigation-bar.active.icon")}
            : ${componentColor("md.comp.navigation-bar.inactive.icon")},
      );
    });
  }

  @override
  Color? get indicatorColor => ${componentColor("md.comp.navigation-bar.active-indicator")};

  @override
  ShapeBorder? get indicatorShape => ${shape("md.comp.navigation-bar.active-indicator")};

  @override
  MaterialStateProperty<TextStyle?>? get labelTextStyle {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
    final TextStyle style = ${textStyle("md.comp.navigation-bar.label-text")}!;
      return style.apply(
        color: states.contains(MaterialState.disabled)
          ? _colors.onSurfaceVariant.withOpacity(0.38)
          : states.contains(MaterialState.selected)
            ? ${componentColor("md.comp.navigation-bar.active.label-text")}
            : ${componentColor("md.comp.navigation-bar.inactive.label-text")}
      );
    });
  }

  @override
  EdgeInsetsGeometry? get labelPadding => const EdgeInsets.only(top: 4);
}
''';
}
