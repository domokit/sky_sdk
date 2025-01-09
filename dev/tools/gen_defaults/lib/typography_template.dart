// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class TypographyTemplate extends TokenTemplate {
  const TypographyTemplate(super.blockName, super.fileName, super.tokens);

  @override
  String generate() => '''
abstract final class _M3Typography {
  ${_textTheme('englishLike', 'alphabetic')}

  ${_textTheme('dense', 'ideographic')}

  ${_textTheme('tall', 'alphabetic')}
}
''';

  String _textTheme(String name, String baseline) {
    final StringBuffer theme = StringBuffer('static const TextTheme $name = TextTheme(\n');
    theme.writeln(
      '    displayLarge: ${_textStyleDef('md.sys.typescale.display-large', '$name displayLarge 2021', baseline)},',
    );
    theme.writeln(
      '    displayMedium: ${_textStyleDef('md.sys.typescale.display-medium', '$name displayMedium 2021', baseline)},',
    );
    theme.writeln(
      '    displaySmall: ${_textStyleDef('md.sys.typescale.display-small', '$name displaySmall 2021', baseline)},',
    );
    theme.writeln(
      '    headlineLarge: ${_textStyleDef('md.sys.typescale.headline-large', '$name headlineLarge 2021', baseline)},',
    );
    theme.writeln(
      '    headlineMedium: ${_textStyleDef('md.sys.typescale.headline-medium', '$name headlineMedium 2021', baseline)},',
    );
    theme.writeln(
      '    headlineSmall: ${_textStyleDef('md.sys.typescale.headline-small', '$name headlineSmall 2021', baseline)},',
    );
    theme.writeln(
      '    titleLarge: ${_textStyleDef('md.sys.typescale.title-large', '$name titleLarge 2021', baseline)},',
    );
    theme.writeln(
      '    titleMedium: ${_textStyleDef('md.sys.typescale.title-medium', '$name titleMedium 2021', baseline)},',
    );
    theme.writeln(
      '    titleSmall: ${_textStyleDef('md.sys.typescale.title-small', '$name titleSmall 2021', baseline)},',
    );
    theme.writeln(
      '    labelLarge: ${_textStyleDef('md.sys.typescale.label-large', '$name labelLarge 2021', baseline)},',
    );
    theme.writeln(
      '    labelMedium: ${_textStyleDef('md.sys.typescale.label-medium', '$name labelMedium 2021', baseline)},',
    );
    theme.writeln(
      '    labelSmall: ${_textStyleDef('md.sys.typescale.label-small', '$name labelSmall 2021', baseline)},',
    );
    theme.writeln(
      '    bodyLarge: ${_textStyleDef('md.sys.typescale.body-large', '$name bodyLarge 2021', baseline)},',
    );
    theme.writeln(
      '    bodyMedium: ${_textStyleDef('md.sys.typescale.body-medium', '$name bodyMedium 2021', baseline)},',
    );
    theme.writeln(
      '    bodySmall: ${_textStyleDef('md.sys.typescale.body-small', '$name bodySmall 2021', baseline)},',
    );
    theme.write('  );');
    return theme.toString();
  }

  String _textStyleDef(String tokenPrefix, String debugLabel, String baseline) {
    final StringBuffer style = StringBuffer("TextStyle(debugLabel: '$debugLabel'");
    style.write(', inherit: false');
    style.write(', fontSize: ${_fontSize(tokenPrefix)}');
    style.write(', fontWeight: ${_fontWeight(tokenPrefix)}');
    style.write(', letterSpacing: ${_fontSpacing(tokenPrefix)}');
    style.write(', height: ${_fontHeight(tokenPrefix)}');
    style.write(', textBaseline: TextBaseline.$baseline');
    style.write(', leadingDistribution: TextLeadingDistribution.even');
    style.write(')');
    return style.toString();
  }

  String _fontSize(String textStyleTokenName) {
    return getToken('$textStyleTokenName.size').toString();
  }

  String _fontWeight(String textStyleTokenName) {
    final String weightValue =
        getToken(getToken('$textStyleTokenName.weight') as String).toString();
    return 'FontWeight.w$weightValue';
  }

  String _fontSpacing(String textStyleTokenName) {
    return getToken('$textStyleTokenName.tracking').toString();
  }

  String _fontHeight(String textStyleTokenName) {
    final double size = getToken('$textStyleTokenName.size') as double;
    final double lineHeight = getToken('$textStyleTokenName.line-height') as double;
    return (lineHeight / size).toStringAsFixed(2);
  }
}
