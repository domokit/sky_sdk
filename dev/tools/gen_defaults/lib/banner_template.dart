// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class BannerTemplate extends TokenTemplate {
  const BannerTemplate(
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
    super.textThemePrefix = '_textTheme.',
  });

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends MaterialBannerThemeData {
  _${blockName}DefaultsM3(this.context)
    : super(elevation: ${elevation("md.comp.banner.container")});

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;
  late final TextTheme _textTheme = Theme.of(context).textTheme;

  @override
  Color? get backgroundColor => ${componentColor("md.comp.banner.container")};

  @override
  Color? get surfaceTintColor => ${colorOrTransparent("md.comp.banner.container.surface-tint-layer.color")};

  @override
  Color? get dividerColor => ${color("md.comp.divider.color")};

  @override
  TextStyle? get contentTextStyle => ${textStyle("md.comp.banner.supporting-text")};
}
''';
}
