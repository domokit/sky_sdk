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
    final ui.ParagraphStyle a = ui.ParagraphStyle();
    final ui.ParagraphStyle b = ui.ParagraphStyle();
    expect(a, b);
    expect(a.hashCode, b.hashCode);
  });

  test('each property individually equal', () {
    for (final String property in _populatorsA.keys) {
      final _ParagraphStylePropertyPopulator populator = _populatorsA[property]!;

      final _TestParagraphStyleBuilder aBuilder = _TestParagraphStyleBuilder();
      populator(aBuilder);
      final ui.ParagraphStyle a = aBuilder.build();

      final _TestParagraphStyleBuilder bBuilder = _TestParagraphStyleBuilder();
      populator(bBuilder);
      final ui.ParagraphStyle b = bBuilder.build();

      expect(reason: '$property property is equal', a, b);
      expect(reason: '$property hashCode is equal', a.hashCode, b.hashCode);
    }
  });

  test('each property individually not equal', () {
    for (final String property in _populatorsA.keys) {
      final _ParagraphStylePropertyPopulator populatorA = _populatorsA[property]!;

      final _TestParagraphStyleBuilder aBuilder = _TestParagraphStyleBuilder();
      populatorA(aBuilder);
      final ui.ParagraphStyle a = aBuilder.build();

      final _ParagraphStylePropertyPopulator populatorB = _populatorsB[property]!;
      final _TestParagraphStyleBuilder bBuilder = _TestParagraphStyleBuilder();
      populatorB(bBuilder);
      final ui.ParagraphStyle b = bBuilder.build();

      expect(reason: '$property property is not equal', a, isNot(b));
      expect(reason: '$property hashCode is not equal', a.hashCode, isNot(b.hashCode));
    }
  });

  test('all properties altogether equal', () {
    final _TestParagraphStyleBuilder aBuilder = _TestParagraphStyleBuilder();
    final _TestParagraphStyleBuilder bBuilder = _TestParagraphStyleBuilder();

    for (final String property in _populatorsA.keys) {
      final _ParagraphStylePropertyPopulator populator = _populatorsA[property]!;
      populator(aBuilder);
      populator(bBuilder);
    }

    final ui.ParagraphStyle a = aBuilder.build();
    final ui.ParagraphStyle b = bBuilder.build();

    expect(a, b);
    expect(a.hashCode, b.hashCode);
  });

  test('all properties altogether not equal', () {
    final _TestParagraphStyleBuilder aBuilder = _TestParagraphStyleBuilder();
    final _TestParagraphStyleBuilder bBuilder = _TestParagraphStyleBuilder();

    for (final String property in _populatorsA.keys) {
      final _ParagraphStylePropertyPopulator populatorA = _populatorsA[property]!;
      populatorA(aBuilder);

      final _ParagraphStylePropertyPopulator populatorB = _populatorsB[property]!;
      populatorB(bBuilder);
    }

    final ui.ParagraphStyle a = aBuilder.build();
    final ui.ParagraphStyle b = bBuilder.build();

    expect(a, isNot(b));
    expect(a.hashCode, isNot(b.hashCode));
  });
}

typedef _ParagraphStylePropertyPopulator = void Function(_TestParagraphStyleBuilder builder);

final Map<String, _ParagraphStylePropertyPopulator> _populatorsA =
    <String, _ParagraphStylePropertyPopulator>{
      'textAlign': (_TestParagraphStyleBuilder builder) {
        builder.textAlign = ui.TextAlign.left;
      },
      'textDirection': (_TestParagraphStyleBuilder builder) {
        builder.textDirection = ui.TextDirection.rtl;
      },
      'fontWeight': (_TestParagraphStyleBuilder builder) {
        builder.fontWeight = ui.FontWeight.w400;
      },
      'fontStyle': (_TestParagraphStyleBuilder builder) {
        builder.fontStyle = ui.FontStyle.normal;
      },
      'maxLines': (_TestParagraphStyleBuilder builder) {
        builder.maxLines = 1;
      },
      'fontFamily': (_TestParagraphStyleBuilder builder) {
        builder.fontFamily = 'Arial';
      },
      'fontSize': (_TestParagraphStyleBuilder builder) {
        builder.fontSize = 12;
      },
      'height': (_TestParagraphStyleBuilder builder) {
        builder.height = 13;
      },
      'textHeightBehavior': (_TestParagraphStyleBuilder builder) {
        builder.textHeightBehavior = const ui.TextHeightBehavior();
      },
      'strutStyle': (_TestParagraphStyleBuilder builder) {
        builder.strutStyle = ui.StrutStyle(fontFamily: 'Times New Roman');
      },
      'ellipsis': (_TestParagraphStyleBuilder builder) {
        builder.ellipsis = '...';
      },
      'locale': (_TestParagraphStyleBuilder builder) {
        builder.locale = const ui.Locale('en', 'US');
      },
    };

final Map<String, _ParagraphStylePropertyPopulator> _populatorsB =
    <String, _ParagraphStylePropertyPopulator>{
      'textAlign': (_TestParagraphStyleBuilder builder) {
        builder.textAlign = ui.TextAlign.right;
      },
      'textDirection': (_TestParagraphStyleBuilder builder) {
        builder.textDirection = ui.TextDirection.ltr;
      },
      'fontWeight': (_TestParagraphStyleBuilder builder) {
        builder.fontWeight = ui.FontWeight.w600;
      },
      'fontStyle': (_TestParagraphStyleBuilder builder) {
        builder.fontStyle = ui.FontStyle.italic;
      },
      'maxLines': (_TestParagraphStyleBuilder builder) {
        builder.maxLines = 2;
      },
      'fontFamily': (_TestParagraphStyleBuilder builder) {
        builder.fontFamily = 'Noto';
      },
      'fontSize': (_TestParagraphStyleBuilder builder) {
        builder.fontSize = 12.1;
      },
      'height': (_TestParagraphStyleBuilder builder) {
        builder.height = 13.1;
      },
      'textHeightBehavior': (_TestParagraphStyleBuilder builder) {
        builder.textHeightBehavior = const ui.TextHeightBehavior(applyHeightToFirstAscent: false);
      },
      'strutStyle': (_TestParagraphStyleBuilder builder) {
        builder.strutStyle = ui.StrutStyle(fontFamily: 'sans-serif');
      },
      'ellipsis': (_TestParagraphStyleBuilder builder) {
        builder.ellipsis = '___';
      },
      'locale': (_TestParagraphStyleBuilder builder) {
        builder.locale = const ui.Locale('fr', 'CA');
      },
    };

class _TestParagraphStyleBuilder {
  ui.TextAlign? textAlign;
  ui.TextDirection? textDirection;
  ui.FontWeight? fontWeight;
  ui.FontStyle? fontStyle;
  int? maxLines;
  String? fontFamily;
  double? fontSize;
  double? height;
  ui.TextHeightBehavior? textHeightBehavior;
  ui.StrutStyle? strutStyle;
  String? ellipsis;
  ui.Locale? locale;

  ui.ParagraphStyle build() {
    return ui.ParagraphStyle(
      textAlign: textAlign,
      textDirection: textDirection,
      fontWeight: fontWeight,
      fontStyle: fontStyle,
      maxLines: maxLines,
      fontFamily: fontFamily,
      fontSize: fontSize,
      height: height,
      textHeightBehavior: textHeightBehavior,
      strutStyle: strutStyle,
      ellipsis: ellipsis,
      locale: locale,
    );
  }
}
