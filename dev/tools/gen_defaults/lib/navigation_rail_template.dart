// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class NavigationRailTemplate extends TokenTemplate {
  const NavigationRailTemplate(
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
    super.textThemePrefix = '_textTheme.',
  });

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends NavigationRailThemeData {
  _${blockName}DefaultsM3(this.context)
    : super(
        elevation: ${elevation("md.comp.navigation-rail.container")},
        groupAlignment: -1,
        labelType: NavigationRailLabelType.none,
        useIndicator: true,
        minWidth: ${getToken('md.comp.navigation-rail.container.width')},
        minExtendedWidth: 256,
      );

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;
  late final TextTheme _textTheme = Theme.of(context).textTheme;

  @override Color? get backgroundColor => ${componentColor("md.comp.navigation-rail.container")};

  @override TextStyle? get unselectedLabelTextStyle {
    return ${textStyle("md.comp.navigation-rail.label-text")}!.copyWith(color: ${componentColor("md.comp.navigation-rail.inactive.focus.label-text")});
  }

  @override TextStyle? get selectedLabelTextStyle {
    return ${textStyle("md.comp.navigation-rail.label-text")}!.copyWith(color: ${componentColor("md.comp.navigation-rail.active.focus.label-text")});
  }

  @override IconThemeData? get unselectedIconTheme {
    return IconThemeData(
      size: ${getToken("md.comp.navigation-rail.icon.size")},
      color: ${componentColor("md.comp.navigation-rail.inactive.icon")},
    );
  }

  @override IconThemeData? get selectedIconTheme {
    return IconThemeData(
      size: ${getToken("md.comp.navigation-rail.icon.size")},
      color: ${componentColor("md.comp.navigation-rail.active.icon")},
    );
  }

  @override Color? get indicatorColor => ${componentColor("md.comp.navigation-rail.active-indicator")};

  @override ShapeBorder? get indicatorShape => ${shape("md.comp.navigation-rail.active-indicator")};
}
''';
}
