// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class BadgeTemplate extends TokenTemplate {
  const BadgeTemplate(
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
  });

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends BadgeThemeData {
  _${blockName}DefaultsM3(this.context) : super(
    smallSize: ${getToken("md.comp.badge.size")},
    largeSize: ${getToken("md.comp.badge.large.size")},
    padding: const EdgeInsets.symmetric(horizontal: 4),
    alignment: AlignmentDirectional.topEnd,
  );

  final BuildContext context;
  late final ThemeData _theme = Theme.of(context);
  late final ColorScheme _colors = _theme.colorScheme;

  @override
  Color? get backgroundColor => ${color("md.comp.badge.color")};

  @override
  Color? get textColor => ${color("md.comp.badge.large.label-text.color")};

  @override
  TextStyle? get textStyle => ${textStyle("md.comp.badge.large.label-text")};
}
''';
}
