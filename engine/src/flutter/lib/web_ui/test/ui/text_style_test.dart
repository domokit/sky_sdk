// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/ui.dart' as ui;

import '../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(emulateTesterEnvironment: false, setUpTestViewDimensions: false);

  test('blanks are equal to each other', () {
    final ui.TextStyle a = ui.TextStyle();
    final ui.TextStyle b = ui.TextStyle();
    expect(a, b);
    expect(a.hashCode, b.hashCode);
  });

  test('each property individually equal', () {
    for (final String property in _populatorsA.keys) {
      final _TextStylePropertyPopulator populator = _populatorsA[property]!;

      final _TestTextStyleBuilder aBuilder = _TestTextStyleBuilder();
      populator(aBuilder);
      final ui.TextStyle a = aBuilder.build();

      final _TestTextStyleBuilder bBuilder = _TestTextStyleBuilder();
      populator(bBuilder);
      final ui.TextStyle b = bBuilder.build();

      expect(reason: '$property property is equal', a, b);
      expect(reason: '$property hashCode is equal', a.hashCode, b.hashCode);
    }
  });

  test('each property individually not equal', () {
    for (final String property in _populatorsA.keys) {
      final _TextStylePropertyPopulator populatorA = _populatorsA[property]!;

      final _TestTextStyleBuilder aBuilder = _TestTextStyleBuilder();
      populatorA(aBuilder);
      final ui.TextStyle a = aBuilder.build();

      final _TextStylePropertyPopulator populatorB = _populatorsB[property]!;
      final _TestTextStyleBuilder bBuilder = _TestTextStyleBuilder();
      populatorB(bBuilder);
      final ui.TextStyle b = bBuilder.build();

      expect(reason: '$property property is not equal', a, isNot(b));
      expect(reason: '$property hashCode is not equal', a.hashCode, isNot(b.hashCode));
    }
  });

  // `color` and `foreground` cannot be used at the same time, so each test skips
  // one or the other to be able to test all variations.
  for (final String skipProperty in const <String>['color', 'foreground']) {
    test('all properties (except $skipProperty) altogether equal', () {
      final _TestTextStyleBuilder aBuilder = _TestTextStyleBuilder();
      final _TestTextStyleBuilder bBuilder = _TestTextStyleBuilder();

      for (final String property in _populatorsA.keys) {
        if (property == skipProperty) {
          continue;
        }
        final _TextStylePropertyPopulator populator = _populatorsA[property]!;
        populator(aBuilder);
        populator(bBuilder);
      }

      final ui.TextStyle a = aBuilder.build();
      final ui.TextStyle b = bBuilder.build();

      expect(a, b);
      expect(a.hashCode, b.hashCode);
    });

    test('all properties (except $skipProperty) altogether not equal', () {
      final _TestTextStyleBuilder aBuilder = _TestTextStyleBuilder();
      final _TestTextStyleBuilder bBuilder = _TestTextStyleBuilder();

      for (final String property in _populatorsA.keys) {
        if (property == skipProperty) {
          continue;
        }
        final _TextStylePropertyPopulator populatorA = _populatorsA[property]!;
        populatorA(aBuilder);

        final _TextStylePropertyPopulator populatorB = _populatorsB[property]!;
        populatorB(bBuilder);
      }

      final ui.TextStyle a = aBuilder.build();
      final ui.TextStyle b = bBuilder.build();

      expect(a, isNot(b));
      expect(a.hashCode, isNot(b.hashCode));
    });
  }

  test('toString() with color', () {
    final _TestTextStyleBuilder builder = _TestTextStyleBuilder();

    for (final String property in _populatorsA.keys) {
      if (property == 'foreground') {
        continue;
      }
      final _TextStylePropertyPopulator populator = _populatorsA[property]!;
      populator(builder);
    }

    final ui.TextStyle style = builder.build();

    expect(
      style.toString(),
      'TextStyle('
      'color: ${const ui.Color(0xff000000)}, '
      'decoration: TextDecoration.none, '
      'decorationColor: ${const ui.Color(0xffaa0000)}, '
      'decorationStyle: TextDecorationStyle.solid, '
      'decorationThickness: ${1.0}, '
      'fontWeight: FontWeight.w400, '
      'fontStyle: FontStyle.normal, '
      'textBaseline: TextBaseline.alphabetic, '
      'fontFamily: Arial, '
      'fontFamilyFallback: [Roboto], '
      'fontSize: 12.0, '
      'letterSpacing: 1.2x, '
      'wordSpacing: 2.3x, '
      'height: 13.0x, '
      'leadingDistribution: TextLeadingDistribution.proportional, '
      'locale: en_US, '
      'background: Paint(), '
      'foreground: unspecified, '
      'shadows: [TextShadow(${const ui.Color(0xff000000)}, Offset(0.0, 0.0), ${0.0})], '
      "fontFeatures: [FontFeature('case', 1)], "
      "fontVariations: [FontVariation('ital', 0.1)]"
      ')',
    );
  });

  test('toString() with foreground', () {
    final _TestTextStyleBuilder builder = _TestTextStyleBuilder();

    for (final String property in _populatorsA.keys) {
      if (property == 'color') {
        continue;
      }
      final _TextStylePropertyPopulator populator = _populatorsA[property]!;
      populator(builder);
    }

    final ui.TextStyle style = builder.build();

    expect(
      style.toString(),
      'TextStyle('
      'color: unspecified, '
      'decoration: TextDecoration.none, '
      'decorationColor: ${const ui.Color(0xffaa0000)}, '
      'decorationStyle: TextDecorationStyle.solid, '
      'decorationThickness: ${1.0}, '
      'fontWeight: FontWeight.w400, '
      'fontStyle: FontStyle.normal, '
      'textBaseline: TextBaseline.alphabetic, '
      'fontFamily: Arial, '
      'fontFamilyFallback: [Roboto], '
      'fontSize: 12.0, '
      'letterSpacing: 1.2x, '
      'wordSpacing: 2.3x, '
      'height: 13.0x, '
      'leadingDistribution: TextLeadingDistribution.proportional, '
      'locale: en_US, '
      'background: Paint(), '
      'foreground: Paint(), '
      'shadows: [TextShadow(${const ui.Color(0xff000000)}, Offset(0.0, 0.0), ${0.0})], '
      "fontFeatures: [FontFeature('case', 1)], "
      "fontVariations: [FontVariation('ital', 0.1)]"
      ')',
    );
  });
}

typedef _TextStylePropertyPopulator = void Function(_TestTextStyleBuilder builder);

// Paint equality is based on identity, so all the paints below are different,
// even though they express the same paint.
final ui.Paint _backgroundA = ui.Paint();
final ui.Paint _foregroundA = ui.Paint();
final ui.Paint _backgroundB = ui.Paint();
final ui.Paint _foregroundB = ui.Paint();

// Intentionally do not use const List expressions to make sure Object.hashAll is used to compute hashCode
final Map<String, _TextStylePropertyPopulator> _populatorsA = <String, _TextStylePropertyPopulator>{
  'color': (_TestTextStyleBuilder builder) {
    builder.color = const ui.Color(0xff000000);
  },
  'decoration': (_TestTextStyleBuilder builder) {
    builder.decoration = ui.TextDecoration.none;
  },
  'decorationColor': (_TestTextStyleBuilder builder) {
    builder.decorationColor = const ui.Color(0xffaa0000);
  },
  'decorationStyle': (_TestTextStyleBuilder builder) {
    builder.decorationStyle = ui.TextDecorationStyle.solid;
  },
  'decorationThickness': (_TestTextStyleBuilder builder) {
    builder.decorationThickness = 1.0;
  },
  'fontWeight': (_TestTextStyleBuilder builder) {
    builder.fontWeight = ui.FontWeight.w400;
  },
  'fontStyle': (_TestTextStyleBuilder builder) {
    builder.fontStyle = ui.FontStyle.normal;
  },
  'textBaseline': (_TestTextStyleBuilder builder) {
    builder.textBaseline = ui.TextBaseline.alphabetic;
  },
  'fontFamily': (_TestTextStyleBuilder builder) {
    builder.fontFamily = 'Arial';
  },
  'fontFamilyFallback': (_TestTextStyleBuilder builder) {
    builder.fontFamilyFallback = <String>['Roboto'];
  },
  'fontSize': (_TestTextStyleBuilder builder) {
    builder.fontSize = 12;
  },
  'letterSpacing': (_TestTextStyleBuilder builder) {
    builder.letterSpacing = 1.2;
  },
  'wordSpacing': (_TestTextStyleBuilder builder) {
    builder.wordSpacing = 2.3;
  },
  'height': (_TestTextStyleBuilder builder) {
    builder.height = 13;
  },
  'leadingDistribution': (_TestTextStyleBuilder builder) {
    builder.leadingDistribution = ui.TextLeadingDistribution.proportional;
  },
  'locale': (_TestTextStyleBuilder builder) {
    builder.locale = const ui.Locale('en', 'US');
  },
  'background': (_TestTextStyleBuilder builder) {
    builder.background = _backgroundA;
  },
  'foreground': (_TestTextStyleBuilder builder) {
    builder.foreground = _foregroundA;
  },
  'shadows': (_TestTextStyleBuilder builder) {
    builder.shadows = <ui.Shadow>[const ui.Shadow()];
  },
  'fontFeatures': (_TestTextStyleBuilder builder) {
    builder.fontFeatures = <ui.FontFeature>[const ui.FontFeature.caseSensitiveForms()];
  },
  'fontVariations': (_TestTextStyleBuilder builder) {
    builder.fontVariations = <ui.FontVariation>[const ui.FontVariation.italic(0.1)];
  },
};

// Intentionally do not use const List expressions to make sure Object.hashAll is used to compute hashCode
final Map<String, _TextStylePropertyPopulator> _populatorsB = <String, _TextStylePropertyPopulator>{
  'color': (_TestTextStyleBuilder builder) {
    builder.color = const ui.Color(0xffbb0000);
  },
  'decoration': (_TestTextStyleBuilder builder) {
    builder.decoration = ui.TextDecoration.lineThrough;
  },
  'decorationColor': (_TestTextStyleBuilder builder) {
    builder.decorationColor = const ui.Color(0xffcc0000);
  },
  'decorationStyle': (_TestTextStyleBuilder builder) {
    builder.decorationStyle = ui.TextDecorationStyle.dotted;
  },
  'decorationThickness': (_TestTextStyleBuilder builder) {
    builder.decorationThickness = 1.4;
  },
  'fontWeight': (_TestTextStyleBuilder builder) {
    builder.fontWeight = ui.FontWeight.w600;
  },
  'fontStyle': (_TestTextStyleBuilder builder) {
    builder.fontStyle = ui.FontStyle.italic;
  },
  'textBaseline': (_TestTextStyleBuilder builder) {
    builder.textBaseline = ui.TextBaseline.ideographic;
  },
  'fontFamily': (_TestTextStyleBuilder builder) {
    builder.fontFamily = 'Noto';
  },
  'fontFamilyFallback': (_TestTextStyleBuilder builder) {
    builder.fontFamilyFallback = <String>['Verdana'];
  },
  'fontSize': (_TestTextStyleBuilder builder) {
    builder.fontSize = 12.1;
  },
  'letterSpacing': (_TestTextStyleBuilder builder) {
    builder.letterSpacing = 1.25;
  },
  'wordSpacing': (_TestTextStyleBuilder builder) {
    builder.wordSpacing = 2.35;
  },
  'height': (_TestTextStyleBuilder builder) {
    builder.height = 13.1;
  },
  'leadingDistribution': (_TestTextStyleBuilder builder) {
    builder.leadingDistribution = ui.TextLeadingDistribution.even;
  },
  'locale': (_TestTextStyleBuilder builder) {
    builder.locale = const ui.Locale('fr', 'CA');
  },
  'background': (_TestTextStyleBuilder builder) {
    builder.background = _backgroundB;
  },
  'foreground': (_TestTextStyleBuilder builder) {
    builder.foreground = _foregroundB;
  },
  'shadows': (_TestTextStyleBuilder builder) {
    builder.shadows = <ui.Shadow>[const ui.Shadow(blurRadius: 5)];
  },
  'fontFeatures': (_TestTextStyleBuilder builder) {
    builder.fontFeatures = <ui.FontFeature>[const ui.FontFeature.alternative(2)];
  },
  'fontVariations': (_TestTextStyleBuilder builder) {
    builder.fontVariations = <ui.FontVariation>[const ui.FontVariation.italic(0.4)];
  },
};

class _TestTextStyleBuilder {
  ui.Color? color;
  ui.TextDecoration? decoration;
  ui.Color? decorationColor;
  ui.TextDecorationStyle? decorationStyle;
  double? decorationThickness;
  ui.FontWeight? fontWeight;
  ui.FontStyle? fontStyle;
  ui.TextBaseline? textBaseline;
  String? fontFamily;
  List<String>? fontFamilyFallback;
  double? fontSize;
  double? letterSpacing;
  double? wordSpacing;
  double? height;
  ui.TextLeadingDistribution? leadingDistribution;
  ui.Locale? locale;
  ui.Paint? background;
  ui.Paint? foreground;
  List<ui.Shadow>? shadows;
  List<ui.FontFeature>? fontFeatures;
  List<ui.FontVariation>? fontVariations;

  ui.TextStyle build() {
    return ui.TextStyle(
      color: color,
      decoration: decoration,
      decorationColor: decorationColor,
      decorationStyle: decorationStyle,
      decorationThickness: decorationThickness,
      fontWeight: fontWeight,
      fontStyle: fontStyle,
      textBaseline: textBaseline,
      fontFamily: fontFamily,
      fontFamilyFallback: fontFamilyFallback,
      fontSize: fontSize,
      letterSpacing: letterSpacing,
      wordSpacing: wordSpacing,
      height: height,
      leadingDistribution: leadingDistribution,
      locale: locale,
      background: background,
      foreground: foreground,
      shadows: shadows,
      fontFeatures: fontFeatures,
      fontVariations: fontVariations,
    );
  }
}
