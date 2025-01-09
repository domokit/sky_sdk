// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class ExpansionTileTemplate extends TokenTemplate {
  const ExpansionTileTemplate(
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
  });

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends ExpansionTileThemeData {
  _${blockName}DefaultsM3(this.context);

  final BuildContext context;
  late final ThemeData _theme = Theme.of(context);
  late final ColorScheme _colors = _theme.colorScheme;

  @override
  Color? get textColor => ${componentColor('md.comp.list.list-item.label-text')};

  @override
  Color? get iconColor => ${componentColor('md.comp.list.list-item.selected.trailing-icon')};

  @override
  Color? get collapsedTextColor => ${componentColor('md.comp.list.list-item.label-text')};

  @override
  Color? get collapsedIconColor => ${componentColor('md.comp.list.list-item.trailing-icon')};
}
''';
}
