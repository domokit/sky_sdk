// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class SnackbarTemplate extends TokenTemplate {
  const SnackbarTemplate(
    this.tokenGroup,
    super.blockName,
    super.fileName,
    super.tokens, {
    super.colorSchemePrefix = '_colors.',
  });

  final String tokenGroup;

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends SnackBarThemeData {
    _${blockName}DefaultsM3(this.context);

  final BuildContext context;
  late final ThemeData _theme = Theme.of(context);
  late final ColorScheme _colors = _theme.colorScheme;

  @override
  Color get backgroundColor => ${componentColor("$tokenGroup.container")};

  @override
  Color get actionTextColor =>  MaterialStateColor.resolveWith((Set<MaterialState> states) {
    if (states.contains(MaterialState.disabled)) {
      return ${componentColor("$tokenGroup.action.pressed.label-text")};
    }
    if (states.contains(MaterialState.pressed)) {
      return ${componentColor("$tokenGroup.action.pressed.label-text")};
    }
    if (states.contains(MaterialState.hovered)) {
      return ${componentColor("$tokenGroup.action.hover.label-text")};
    }
    if (states.contains(MaterialState.focused)) {
      return ${componentColor("$tokenGroup.action.focus.label-text")};
    }
    return ${componentColor("$tokenGroup.action.label-text")};
  });

  @override
  Color get disabledActionTextColor =>
    ${componentColor("$tokenGroup.action.pressed.label-text")};


  @override
  TextStyle get contentTextStyle =>
    ${textStyle("$tokenGroup.supporting-text")}!.copyWith
      (color:  ${componentColor("$tokenGroup.supporting-text")},
    );

  @override
  double get elevation => ${elevation("$tokenGroup.container")};

  @override
  ShapeBorder get shape => ${shape("$tokenGroup.container")};

  @override
  SnackBarBehavior get behavior => SnackBarBehavior.fixed;

  @override
  EdgeInsets get insetPadding => const EdgeInsets.fromLTRB(15.0, 5.0, 15.0, 10.0);

  @override
  bool get showCloseIcon => false;

  @override
  Color? get closeIconColor => ${componentColor("$tokenGroup.icon")};

  @override
  double get actionOverflowThreshold => 0.25;
}
''';
}
