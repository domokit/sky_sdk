// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class SearchViewTemplate extends TokenTemplate {
  const SearchViewTemplate(
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
    super.textThemePrefix = '_textTheme.',
  });

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends ${blockName}ThemeData {
  _${blockName}DefaultsM3(this.context, {required this.isFullScreen});

  final BuildContext context;
  final bool isFullScreen;
  late final ColorScheme _colors = Theme.of(context).colorScheme;
  late final TextTheme _textTheme = Theme.of(context).textTheme;

  static double fullScreenBarHeight = ${getToken('md.comp.search-view.full-screen.header.container.height')};

  @override
  Color? get backgroundColor => ${componentColor('md.comp.search-view.container')};

  @override
  double? get elevation => ${elevation('md.comp.search-view.container')};

  @override
  Color? get surfaceTintColor => ${colorOrTransparent('md.comp.search-view.container.surface-tint-layer.color')};

  // No default side

  @override
  OutlinedBorder? get shape => isFullScreen
    ? ${shape('md.comp.search-view.full-screen.container')}
    : ${shape('md.comp.search-view.docked.container')};

  @override
  TextStyle? get headerTextStyle => ${textStyleWithColor('md.comp.search-view.header.input-text')};

  @override
  TextStyle? get headerHintStyle => ${textStyleWithColor('md.comp.search-view.header.supporting-text')};

  @override
  BoxConstraints get constraints => const BoxConstraints(minWidth: 360.0, minHeight: 240.0);

  @override
  EdgeInsetsGeometry? get barPadding => const EdgeInsets.symmetric(horizontal: 8.0);

  @override
  bool get shrinkWrap => false;

  @override
  Color? get dividerColor => ${componentColor('md.comp.search-view.divider')};
}
''';
}
