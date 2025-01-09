// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This is testing some of the named constants.
// ignore_for_file: use_named_constants

import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';
import 'dart:ui';

import 'package:path/path.dart' as path;
import 'package:test/test.dart';

Future<Uint8List> readFile(String fileName) async {
  final File file = File(path.join('flutter', 'testing', 'resources', fileName));
  return file.readAsBytes();
}

void testFontWeight() {
  test('FontWeight.lerp works with non-null values', () {
    expect(FontWeight.lerp(FontWeight.w400, FontWeight.w600, .5), equals(FontWeight.w500));
  });

  test('FontWeight.lerp returns null if a and b are null', () {
    expect(FontWeight.lerp(null, null, 0), isNull);
  });

  test('FontWeight.lerp returns FontWeight.w400 if a is null', () {
    expect(FontWeight.lerp(null, FontWeight.w400, 0), equals(FontWeight.w400));
  });

  test('FontWeight.lerp returns FontWeight.w400 if b is null', () {
    expect(FontWeight.lerp(FontWeight.w400, null, 1), equals(FontWeight.w400));
  });

  test('FontWeights have the correct value', () {
    expect(FontWeight.w100.value, 100);
    expect(FontWeight.w200.value, 200);
    expect(FontWeight.w300.value, 300);
    expect(FontWeight.w400.value, 400);
    expect(FontWeight.w500.value, 500);
    expect(FontWeight.w600.value, 600);
    expect(FontWeight.w700.value, 700);
    expect(FontWeight.w800.value, 800);
    expect(FontWeight.w900.value, 900);
  });
}

void testParagraphStyle() {
  final ParagraphStyle ps0 = ParagraphStyle(textDirection: TextDirection.ltr, fontSize: 14.0);
  final ParagraphStyle ps1 = ParagraphStyle(textDirection: TextDirection.rtl, fontSize: 14.0);
  final ParagraphStyle ps2 = ParagraphStyle(
    textAlign: TextAlign.center,
    fontWeight: FontWeight.w800,
    fontSize: 10.0,
    height: 100.0,
  );
  final ParagraphStyle ps3 = ParagraphStyle(
    fontWeight: FontWeight.w700,
    fontSize: 12.0,
    height: 123.0,
  );
  final ParagraphStyle ps4 = ParagraphStyle(
    fontWeight: FontWeight.w700,
    fontSize: 12.0,
    height: kTextHeightNone,
  );

  test('ParagraphStyle toString works', () {
    expect(
      ps0.toString(),
      equals(
        'ParagraphStyle(textAlign: unspecified, textDirection: TextDirection.ltr, fontWeight: unspecified, fontStyle: unspecified, maxLines: unspecified, textHeightBehavior: unspecified, fontFamily: unspecified, fontSize: 14.0, height: unspecified, strutStyle: unspecified, ellipsis: unspecified, locale: unspecified)',
      ),
    );
    expect(
      ps1.toString(),
      equals(
        'ParagraphStyle(textAlign: unspecified, textDirection: TextDirection.rtl, fontWeight: unspecified, fontStyle: unspecified, maxLines: unspecified, textHeightBehavior: unspecified, fontFamily: unspecified, fontSize: 14.0, height: unspecified, strutStyle: unspecified, ellipsis: unspecified, locale: unspecified)',
      ),
    );
    expect(
      ps2.toString(),
      equals(
        'ParagraphStyle(textAlign: TextAlign.center, textDirection: unspecified, fontWeight: FontWeight.w800, fontStyle: unspecified, maxLines: unspecified, textHeightBehavior: unspecified, fontFamily: unspecified, fontSize: 10.0, height: 100.0x, strutStyle: unspecified, ellipsis: unspecified, locale: unspecified)',
      ),
    );
    expect(
      ps3.toString(),
      equals(
        'ParagraphStyle(textAlign: unspecified, textDirection: unspecified, fontWeight: FontWeight.w700, fontStyle: unspecified, maxLines: unspecified, textHeightBehavior: unspecified, fontFamily: unspecified, fontSize: 12.0, height: 123.0x, strutStyle: unspecified, ellipsis: unspecified, locale: unspecified)',
      ),
    );
    expect(
      ps4.toString(),
      equals(
        'ParagraphStyle(textAlign: unspecified, textDirection: unspecified, fontWeight: FontWeight.w700, fontStyle: unspecified, maxLines: unspecified, textHeightBehavior: unspecified, fontFamily: unspecified, fontSize: 12.0, height: unspecified, strutStyle: unspecified, ellipsis: unspecified, locale: unspecified)',
      ),
    );
  });
}

void testTextStyle() {
  final TextStyle ts0 = TextStyle(fontWeight: FontWeight.w700, fontSize: 12.0, height: 123.0);
  final TextStyle ts1 = TextStyle(
    color: const Color(0xFF00FF00),
    fontWeight: FontWeight.w800,
    fontSize: 10.0,
    height: 100.0,
  );
  final TextStyle ts2 = TextStyle(fontFamily: 'test');
  final TextStyle ts3 = TextStyle(
    fontFamily: 'foo',
    fontFamilyFallback: <String>['Roboto', 'test'],
  );
  final TextStyle ts4 = TextStyle(leadingDistribution: TextLeadingDistribution.even);
  final TextStyle ts5 = TextStyle(height: kTextHeightNone);

  test('TextStyle toString works', () {
    expect(
      ts0.toString(),
      equals(
        'TextStyle(color: unspecified, decoration: unspecified, decorationColor: unspecified, decorationStyle: unspecified, decorationThickness: unspecified, fontWeight: FontWeight.w700, fontStyle: unspecified, textBaseline: unspecified, fontFamily: unspecified, fontFamilyFallback: unspecified, fontSize: 12.0, letterSpacing: unspecified, wordSpacing: unspecified, height: 123.0x, leadingDistribution: unspecified, locale: unspecified, background: unspecified, foreground: unspecified, shadows: unspecified, fontFeatures: unspecified, fontVariations: unspecified)',
      ),
    );
    expect(
      ts1.toString(),
      equals(
        'TextStyle(color: ${const Color(0xFF00FF00)}, decoration: unspecified, decorationColor: unspecified, decorationStyle: unspecified, decorationThickness: unspecified, fontWeight: FontWeight.w800, fontStyle: unspecified, textBaseline: unspecified, fontFamily: unspecified, fontFamilyFallback: unspecified, fontSize: 10.0, letterSpacing: unspecified, wordSpacing: unspecified, height: 100.0x, leadingDistribution: unspecified, locale: unspecified, background: unspecified, foreground: unspecified, shadows: unspecified, fontFeatures: unspecified, fontVariations: unspecified)',
      ),
    );
    expect(
      ts2.toString(),
      equals(
        'TextStyle(color: unspecified, decoration: unspecified, decorationColor: unspecified, decorationStyle: unspecified, decorationThickness: unspecified, fontWeight: unspecified, fontStyle: unspecified, textBaseline: unspecified, fontFamily: test, fontFamilyFallback: unspecified, fontSize: unspecified, letterSpacing: unspecified, wordSpacing: unspecified, height: unspecified, leadingDistribution: unspecified, locale: unspecified, background: unspecified, foreground: unspecified, shadows: unspecified, fontFeatures: unspecified, fontVariations: unspecified)',
      ),
    );
    expect(
      ts3.toString(),
      equals(
        'TextStyle(color: unspecified, decoration: unspecified, decorationColor: unspecified, decorationStyle: unspecified, decorationThickness: unspecified, fontWeight: unspecified, fontStyle: unspecified, textBaseline: unspecified, fontFamily: foo, fontFamilyFallback: [Roboto, test], fontSize: unspecified, letterSpacing: unspecified, wordSpacing: unspecified, height: unspecified, leadingDistribution: unspecified, locale: unspecified, background: unspecified, foreground: unspecified, shadows: unspecified, fontFeatures: unspecified, fontVariations: unspecified)',
      ),
    );
    expect(
      ts4.toString(),
      equals(
        'TextStyle(color: unspecified, decoration: unspecified, decorationColor: unspecified, decorationStyle: unspecified, decorationThickness: unspecified, fontWeight: unspecified, fontStyle: unspecified, textBaseline: unspecified, fontFamily: unspecified, fontFamilyFallback: unspecified, fontSize: unspecified, letterSpacing: unspecified, wordSpacing: unspecified, height: unspecified, leadingDistribution: TextLeadingDistribution.even, locale: unspecified, background: unspecified, foreground: unspecified, shadows: unspecified, fontFeatures: unspecified, fontVariations: unspecified)',
      ),
    );
    expect(
      ts5.toString(),
      equals(
        'TextStyle(color: unspecified, decoration: unspecified, decorationColor: unspecified, decorationStyle: unspecified, decorationThickness: unspecified, fontWeight: unspecified, fontStyle: unspecified, textBaseline: unspecified, fontFamily: unspecified, fontFamilyFallback: unspecified, fontSize: unspecified, letterSpacing: unspecified, wordSpacing: unspecified, height: kTextHeightNone, leadingDistribution: unspecified, locale: unspecified, background: unspecified, foreground: unspecified, shadows: unspecified, fontFeatures: unspecified, fontVariations: unspecified)',
      ),
    );
  });
}

void testTextHeightBehavior() {
  const TextHeightBehavior behavior0 = TextHeightBehavior();
  const TextHeightBehavior behavior1 = TextHeightBehavior(
    applyHeightToFirstAscent: false,
    applyHeightToLastDescent: false,
  );
  const TextHeightBehavior behavior2 = TextHeightBehavior(applyHeightToFirstAscent: false);
  const TextHeightBehavior behavior3 = TextHeightBehavior(applyHeightToLastDescent: false);
  const TextHeightBehavior behavior4 = TextHeightBehavior(
    applyHeightToLastDescent: false,
    leadingDistribution: TextLeadingDistribution.even,
  );

  test('TextHeightBehavior default constructor works', () {
    expect(behavior0.applyHeightToFirstAscent, equals(true));
    expect(behavior0.applyHeightToLastDescent, equals(true));

    expect(behavior1.applyHeightToFirstAscent, equals(false));
    expect(behavior1.applyHeightToLastDescent, equals(false));

    expect(behavior2.applyHeightToFirstAscent, equals(false));
    expect(behavior2.applyHeightToLastDescent, equals(true));

    expect(behavior3.applyHeightToFirstAscent, equals(true));
    expect(behavior3.applyHeightToLastDescent, equals(false));

    expect(behavior4.applyHeightToLastDescent, equals(false));
    expect(behavior4.leadingDistribution, equals(TextLeadingDistribution.even));
  });

  test('TextHeightBehavior toString works', () {
    expect(
      behavior0.toString(),
      equals(
        'TextHeightBehavior(applyHeightToFirstAscent: true, applyHeightToLastDescent: true, leadingDistribution: TextLeadingDistribution.proportional)',
      ),
    );
    expect(
      behavior1.toString(),
      equals(
        'TextHeightBehavior(applyHeightToFirstAscent: false, applyHeightToLastDescent: false, leadingDistribution: TextLeadingDistribution.proportional)',
      ),
    );
    expect(
      behavior2.toString(),
      equals(
        'TextHeightBehavior(applyHeightToFirstAscent: false, applyHeightToLastDescent: true, leadingDistribution: TextLeadingDistribution.proportional)',
      ),
    );
    expect(
      behavior3.toString(),
      equals(
        'TextHeightBehavior(applyHeightToFirstAscent: true, applyHeightToLastDescent: false, leadingDistribution: TextLeadingDistribution.proportional)',
      ),
    );
    expect(
      behavior4.toString(),
      equals(
        'TextHeightBehavior(applyHeightToFirstAscent: true, applyHeightToLastDescent: false, leadingDistribution: TextLeadingDistribution.even)',
      ),
    );
  });
}

void testTextRange() {
  test('TextRange empty ranges are correct', () {
    const TextRange range = TextRange(start: -1, end: -1);
    expect(range, equals(const TextRange.collapsed(-1)));
    expect(range, equals(TextRange.empty));
  });
  test('TextRange isValid works', () {
    expect(TextRange.empty.isValid, isFalse);
    expect(const TextRange(start: 0, end: 0).isValid, isTrue);
    expect(const TextRange(start: 0, end: 10).isValid, isTrue);
    expect(const TextRange(start: 10, end: 10).isValid, isTrue);
    expect(const TextRange(start: -1, end: 10).isValid, isFalse);
    expect(const TextRange(start: 10, end: 0).isValid, isTrue);
    expect(const TextRange(start: 10, end: -1).isValid, isFalse);
  });
  test('TextRange isCollapsed works', () {
    expect(TextRange.empty.isCollapsed, isTrue);
    expect(const TextRange(start: 0, end: 0).isCollapsed, isTrue);
    expect(const TextRange(start: 0, end: 10).isCollapsed, isFalse);
    expect(const TextRange(start: 10, end: 10).isCollapsed, isTrue);
    expect(const TextRange(start: -1, end: 10).isCollapsed, isFalse);
    expect(const TextRange(start: 10, end: 0).isCollapsed, isFalse);
    expect(const TextRange(start: 10, end: -1).isCollapsed, isFalse);
  });
  test('TextRange isNormalized works', () {
    expect(TextRange.empty.isNormalized, isTrue);
    expect(const TextRange(start: 0, end: 0).isNormalized, isTrue);
    expect(const TextRange(start: 0, end: 10).isNormalized, isTrue);
    expect(const TextRange(start: 10, end: 10).isNormalized, isTrue);
    expect(const TextRange(start: -1, end: 10).isNormalized, isTrue);
    expect(const TextRange(start: 10, end: 0).isNormalized, isFalse);
    expect(const TextRange(start: 10, end: -1).isNormalized, isFalse);
  });
  test('TextRange textBefore works', () {
    expect(const TextRange(start: 0, end: 0).textBefore('hello'), isEmpty);
    expect(const TextRange(start: 1, end: 1).textBefore('hello'), equals('h'));
    expect(const TextRange(start: 1, end: 2).textBefore('hello'), equals('h'));
    expect(const TextRange(start: 5, end: 5).textBefore('hello'), equals('hello'));
    expect(const TextRange(start: 0, end: 5).textBefore('hello'), isEmpty);
  });
  test('TextRange textAfter works', () {
    expect(const TextRange(start: 0, end: 0).textAfter('hello'), equals('hello'));
    expect(const TextRange(start: 1, end: 1).textAfter('hello'), equals('ello'));
    expect(const TextRange(start: 1, end: 2).textAfter('hello'), equals('llo'));
    expect(const TextRange(start: 5, end: 5).textAfter('hello'), isEmpty);
    expect(const TextRange(start: 0, end: 5).textAfter('hello'), isEmpty);
  });
  test('TextRange textInside works', () {
    expect(const TextRange(start: 0, end: 0).textInside('hello'), isEmpty);
    expect(const TextRange(start: 1, end: 1).textInside('hello'), isEmpty);
    expect(const TextRange(start: 1, end: 2).textInside('hello'), equals('e'));
    expect(const TextRange(start: 5, end: 5).textInside('hello'), isEmpty);
    expect(const TextRange(start: 0, end: 5).textInside('hello'), equals('hello'));
  });
}

void testGlyphInfo() {
  test('constructor', () {
    const Rect testRect = Rect.fromLTWH(1, 2, 3, 4);
    const TextRange testRange = TextRange(start: 5, end: 6);
    const TextDirection testDirection = TextDirection.ltr;
    final GlyphInfo info = GlyphInfo(testRect, testRange, testDirection);
    expect(info.graphemeClusterLayoutBounds, testRect);
    expect(info.graphemeClusterCodeUnitRange, testRange);
    expect(info.writingDirection, testDirection);
  });
}

void testLoadFontFromList() {
  test('loadFontFromList will send platform message after font is loaded', () async {
    late String message;
    channelBuffers.setListener('flutter/system', (
      ByteData? data,
      PlatformMessageResponseCallback? callback,
    ) {
      assert(data != null);
      final Uint8List list = data!.buffer.asUint8List(data.offsetInBytes, data.lengthInBytes);
      message = utf8.decode(list);
    });
    final Uint8List fontData = Uint8List(0);
    await loadFontFromList(fontData, fontFamily: 'fake');
    expect(message, '{"type":"fontsChange"}');
    channelBuffers.clearListener('flutter/system');
  });
}

void testFontFeatureClass() {
  test('FontFeature class', () {
    expect(const FontFeature.alternative(1), const FontFeature('aalt'));
    expect(const FontFeature.alternative(5), const FontFeature('aalt', 5));
    expect(const FontFeature.alternativeFractions(), const FontFeature('afrc'));
    expect(const FontFeature.contextualAlternates(), const FontFeature('calt'));
    expect(const FontFeature.caseSensitiveForms(), const FontFeature('case'));
    expect(FontFeature.characterVariant(1), const FontFeature('cv01'));
    expect(FontFeature.characterVariant(18), const FontFeature('cv18'));
    expect(FontFeature.characterVariant(99), const FontFeature('cv99'));
    expect(const FontFeature.denominator(), const FontFeature('dnom'));
    expect(const FontFeature.fractions(), const FontFeature('frac'));
    expect(const FontFeature.historicalForms(), const FontFeature('hist'));
    expect(const FontFeature.historicalLigatures(), const FontFeature('hlig'));
    expect(const FontFeature.liningFigures(), const FontFeature('lnum'));
    expect(const FontFeature.localeAware(), const FontFeature('locl'));
    expect(const FontFeature.localeAware(), const FontFeature('locl'));
    expect(const FontFeature.localeAware(enable: false), const FontFeature('locl', 0));
    expect(const FontFeature.notationalForms(), const FontFeature('nalt'));
    expect(const FontFeature.notationalForms(5), const FontFeature('nalt', 5));
    expect(const FontFeature.numerators(), const FontFeature('numr'));
    expect(const FontFeature.oldstyleFigures(), const FontFeature('onum'));
    expect(const FontFeature.ordinalForms(), const FontFeature('ordn'));
    expect(const FontFeature.proportionalFigures(), const FontFeature('pnum'));
    expect(const FontFeature.randomize(), const FontFeature('rand'));
    expect(const FontFeature.stylisticAlternates(), const FontFeature('salt'));
    expect(const FontFeature.scientificInferiors(), const FontFeature('sinf'));
    expect(FontFeature.stylisticSet(1), const FontFeature('ss01'));
    expect(FontFeature.stylisticSet(18), const FontFeature('ss18'));
    expect(const FontFeature.subscripts(), const FontFeature('subs'));
    expect(const FontFeature.superscripts(), const FontFeature('sups'));
    expect(const FontFeature.swash(), const FontFeature('swsh'));
    expect(const FontFeature.swash(0), const FontFeature('swsh', 0));
    expect(const FontFeature.swash(5), const FontFeature('swsh', 5));
    expect(const FontFeature.tabularFigures(), const FontFeature('tnum'));
    expect(const FontFeature.slashedZero(), const FontFeature('zero'));
    expect(const FontFeature.enable('TEST'), const FontFeature('TEST'));
    expect(const FontFeature.disable('TEST'), const FontFeature('TEST', 0));
    expect(const FontFeature('FEAT', 1000).feature, 'FEAT');
    expect(const FontFeature('FEAT', 1000).value, 1000);
    expect(const FontFeature('FEAT', 1000).toString(), "FontFeature('FEAT', 1000)");
  });
}

void testFontVariation() {
  test('FontVariation', () async {
    final Uint8List fontData = await readFile('RobotoSlab-VariableFont_wght.ttf');
    await loadFontFromList(fontData, fontFamily: 'RobotoSerif');

    final ParagraphBuilder baseBuilder = ParagraphBuilder(
      ParagraphStyle(fontFamily: 'RobotoSerif', fontSize: 40.0),
    );
    baseBuilder.addText('Hello');
    final Paragraph baseParagraph = baseBuilder.build();
    baseParagraph.layout(const ParagraphConstraints(width: double.infinity));
    final double baseWidth = baseParagraph.minIntrinsicWidth;

    final ParagraphBuilder wideBuilder = ParagraphBuilder(
      ParagraphStyle(fontFamily: 'RobotoSerif', fontSize: 40.0),
    );
    wideBuilder.pushStyle(
      TextStyle(
        fontFamily: 'RobotoSerif',
        fontSize: 40.0,
        fontVariations: <FontVariation>[const FontVariation('wght', 900.0)],
      ),
    );
    wideBuilder.addText('Hello');
    final Paragraph wideParagraph = wideBuilder.build();
    wideParagraph.layout(const ParagraphConstraints(width: double.infinity));
    final double wideWidth = wideParagraph.minIntrinsicWidth;

    expect(wideWidth, greaterThan(baseWidth));
  });

  test('FontVariation constructors', () async {
    expect(const FontVariation.weight(123.0).axis, 'wght');
    expect(const FontVariation.weight(123.0).value, 123.0);
    expect(const FontVariation.width(123.0).axis, 'wdth');
    expect(const FontVariation.width(123.0).value, 123.0);
    expect(const FontVariation.slant(45.0).axis, 'slnt');
    expect(const FontVariation.slant(45.0).value, 45.0);
    expect(const FontVariation.opticalSize(67.0).axis, 'opsz');
    expect(const FontVariation.opticalSize(67.0).value, 67.0);
    expect(const FontVariation.italic(0.8).axis, 'ital');
    expect(const FontVariation.italic(0.8).value, 0.8);
  });

  test('FontVariation.lerp', () async {
    expect(
      FontVariation.lerp(const FontVariation.weight(100.0), const FontVariation.weight(300.0), 0.5),
      const FontVariation.weight(200.0),
    );
    expect(
      FontVariation.lerp(const FontVariation.slant(0.0), const FontVariation.slant(-80.0), 0.25),
      const FontVariation.slant(-20.0),
    );
    expect(
      FontVariation.lerp(const FontVariation.width(90.0), const FontVariation.italic(0.2), 0.1),
      const FontVariation.width(90.0),
    );
    expect(
      FontVariation.lerp(const FontVariation.width(90.0), const FontVariation.italic(0.2), 0.9),
      const FontVariation.italic(0.2),
    );
  });
}

void testGetWordBoundary() {
  test('GetWordBoundary', () async {
    final Uint8List fontData = await readFile('RobotoSlab-VariableFont_wght.ttf');
    await loadFontFromList(fontData, fontFamily: 'RobotoSerif');

    final ParagraphBuilder builder = ParagraphBuilder(
      ParagraphStyle(fontFamily: 'RobotoSerif', fontSize: 40.0),
    );
    builder.addText('Hello team');
    final Paragraph paragraph = builder.build();
    paragraph.layout(const ParagraphConstraints(width: double.infinity));

    TextRange range = paragraph.getWordBoundary(
      const TextPosition(offset: 5, affinity: TextAffinity.upstream),
    );
    expect(range.start, 0);
    expect(range.end, 5);

    range = paragraph.getWordBoundary(const TextPosition(offset: 5));
    expect(range.start, 5);
    expect(range.end, 6);
  });
}

void main() {
  testFontWeight();
  testParagraphStyle();
  testTextStyle();
  testTextHeightBehavior();
  testTextRange();
  testGlyphInfo();
  testLoadFontFromList();
  testFontFeatureClass();
  testFontVariation();
  testGetWordBoundary();
}
