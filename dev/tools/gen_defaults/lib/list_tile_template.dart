// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class ListTileTemplate extends TokenTemplate {
  const ListTileTemplate(
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
    super.textThemePrefix = '_textTheme.',
  });

  static const String tokenGroup = 'md.comp.list.list-item';

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends ListTileThemeData {
  _${blockName}DefaultsM3(this.context)
    : super(
        contentPadding: const EdgeInsetsDirectional.only(start: 16.0, end: 24.0),
        minLeadingWidth: 24,
        minVerticalPadding: 8,
        shape: ${shape("$tokenGroup.container")},
      );

  final BuildContext context;
  late final ThemeData _theme = Theme.of(context);
  late final ColorScheme _colors = _theme.colorScheme;
  late final TextTheme _textTheme = _theme.textTheme;

  @override
  Color? get tileColor =>  Colors.transparent;

  @override
  TextStyle? get titleTextStyle => ${textStyle("$tokenGroup.label-text")}!.copyWith(color: ${componentColor('$tokenGroup.label-text')});

  @override
  TextStyle? get subtitleTextStyle => ${textStyle("$tokenGroup.supporting-text")}!.copyWith(color: ${componentColor('$tokenGroup.supporting-text')});

  @override
  TextStyle? get leadingAndTrailingTextStyle => ${textStyle("$tokenGroup.trailing-supporting-text")}!.copyWith(color: ${componentColor('$tokenGroup.trailing-supporting-text')});

  @override
  Color? get selectedColor => ${componentColor('$tokenGroup.selected.trailing-icon')};

  @override
  Color? get iconColor => ${componentColor('$tokenGroup.trailing-icon')};
}
''';
}
