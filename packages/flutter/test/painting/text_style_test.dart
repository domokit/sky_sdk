// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui'
    as ui
    show FontFeature, FontVariation, ParagraphStyle, Shadow, TextStyle, lerpDouble;

import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

// This matcher verifies ui.TextStyle.toString (from dart:ui) reports a superset
// of the given TextStyle's (from painting.dart) properties.
class _DartUiTextStyleToStringMatcher extends Matcher {
  _DartUiTextStyleToStringMatcher(this.textStyle);

  final TextStyle textStyle;

  late final List<String> propertiesInOrder = <String>[
    _propertyToString('color', textStyle.color),
    _propertyToString('decoration', textStyle.decoration),
    _propertyToString('decorationColor', textStyle.decorationColor),
    _propertyToString('decorationStyle', textStyle.decorationStyle),
    _propertyToString('decorationThickness', textStyle.decorationThickness),
    _propertyToString('fontWeight', textStyle.fontWeight),
    _propertyToString('fontStyle', textStyle.fontStyle),
    _propertyToString('textBaseline', textStyle.textBaseline),
    _propertyToString('fontFamily', textStyle.fontFamily),
    _propertyToString('fontFamilyFallback', textStyle.fontFamilyFallback),
    _propertyToString('fontSize', textStyle.fontSize),
    _propertyToString('letterSpacing', textStyle.letterSpacing),
    _propertyToString('wordSpacing', textStyle.wordSpacing),
    _propertyToString('height', textStyle.height),
    _propertyToString('leadingDistribution', textStyle.leadingDistribution),
    _propertyToString('locale', textStyle.locale),
    _propertyToString('background', textStyle.background),
    _propertyToString('foreground', textStyle.foreground),
    _propertyToString('shadows', textStyle.shadows),
    _propertyToString('fontFeatures', textStyle.fontFeatures),
    _propertyToString('fontVariations', textStyle.fontVariations),
  ];

  static String _propertyToString(String name, Object? property) =>
      '$name: ${property ?? 'unspecified'}';

  @override
  Description describe(Description description) => description.add('is a superset of $textStyle.');

  @override
  bool matches(dynamic item, Map<dynamic, dynamic> matchState) {
    final String description = item.toString();
    const String prefix = 'TextStyle(';
    const String suffix = ')';
    if (!description.startsWith(prefix) || !description.endsWith(suffix)) {
      return false;
    }

    final String propertyDescription = description.substring(
      prefix.length,
      description.length - suffix.length,
    );
    int startIndex = 0;
    for (final String property in propertiesInOrder) {
      startIndex = propertyDescription.indexOf(property, startIndex);
      if (startIndex < 0) {
        matchState['missingProperty'] = property;
        return false;
      }
      startIndex += property.length;
    }
    return true;
  }

  @override
  Description describeMismatch(
    dynamic item,
    Description mismatchDescription,
    Map<dynamic, dynamic> matchState,
    bool verbose,
  ) {
    final Description description = super.describeMismatch(
      item,
      mismatchDescription,
      matchState,
      verbose,
    );
    final String itemAsString = item.toString();
    final String? property = matchState['missingProperty'] as String?;
    if (property != null) {
      description.add("expect property: '$property'");
      final int propertyIndex = propertiesInOrder.indexOf(property);
      if (propertyIndex > 0) {
        final String lastProperty = propertiesInOrder[propertyIndex - 1];
        description.add(" after: '$lastProperty'\n");
        description.add('but found: ${itemAsString.substring(itemAsString.indexOf(lastProperty))}');
      }
      description.add('\n');
    }
    return description;
  }
}

Matcher matchesToStringOf(TextStyle textStyle) => _DartUiTextStyleToStringMatcher(textStyle);

void main() {
  test('TextStyle control test', () {
    expect(
      const TextStyle(inherit: false).toString(),
      equals('TextStyle(inherit: false, <no style specified>)'),
    );
    expect(const TextStyle().toString(), equals('TextStyle(<all styles inherited>)'));

    const TextStyle s1 = TextStyle(fontSize: 10.0, fontWeight: FontWeight.w800, height: 123.0);
    expect(s1.fontFamily, isNull);
    expect(s1.fontSize, 10.0);
    expect(s1.fontWeight, FontWeight.w800);
    expect(s1.height, 123.0);
    expect(s1, equals(s1));
    expect(
      s1.toString(),
      equals('TextStyle(inherit: true, size: 10.0, weight: 800, height: 123.0x)'),
    );

    // Check that the inherit flag can be set with copyWith().
    expect(
      s1.copyWith(inherit: false).toString(),
      equals('TextStyle(inherit: false, size: 10.0, weight: 800, height: 123.0x)'),
    );

    final TextStyle s2 = s1.copyWith(
      color: const Color(0xFF00FF00),
      height: 100.0,
      leadingDistribution: TextLeadingDistribution.even,
    );
    expect(s1.fontFamily, isNull);
    expect(s1.fontSize, 10.0);
    expect(s1.fontWeight, FontWeight.w800);
    expect(s1.height, 123.0);
    expect(s1.color, isNull);
    expect(s2.fontFamily, isNull);
    expect(s2.fontSize, 10.0);
    expect(s2.fontWeight, FontWeight.w800);
    expect(s2.height, 100.0);
    expect(s2.color, const Color(0xFF00FF00));
    expect(s2.leadingDistribution, TextLeadingDistribution.even);
    expect(s2, isNot(equals(s1)));
    expect(
      s2.toString(),
      equals(
        'TextStyle(inherit: true, color: ${const Color(0xff00ff00)}, size: 10.0, weight: 800, height: 100.0x, leadingDistribution: even)',
      ),
    );

    final TextStyle s3 = s1.apply(fontSizeFactor: 2.0, fontSizeDelta: -2.0, fontWeightDelta: -4);
    expect(s1.fontFamily, isNull);
    expect(s1.fontSize, 10.0);
    expect(s1.fontWeight, FontWeight.w800);
    expect(s1.height, 123.0);
    expect(s1.color, isNull);
    expect(s3.fontFamily, isNull);
    expect(s3.fontSize, 18.0);
    expect(s3.fontWeight, FontWeight.w400);
    expect(s3.height, 123.0);
    expect(s3.color, isNull);
    expect(s3, isNot(equals(s1)));

    expect(s1.apply(fontWeightDelta: -10).fontWeight, FontWeight.w100);
    expect(s1.apply(fontWeightDelta: 2).fontWeight, FontWeight.w900);
    expect(s1.merge(null), equals(s1));

    final TextStyle s4 = s2.merge(s1);
    expect(s1.fontFamily, isNull);
    expect(s1.fontSize, 10.0);
    expect(s1.fontWeight, FontWeight.w800);
    expect(s1.height, 123.0);
    expect(s1.color, isNull);
    expect(s2.fontFamily, isNull);
    expect(s2.fontSize, 10.0);
    expect(s2.fontWeight, FontWeight.w800);
    expect(s2.height, 100.0);
    expect(s2.color, const Color(0xFF00FF00));
    expect(s2.leadingDistribution, TextLeadingDistribution.even);
    expect(s2, isNot(equals(s1)));
    expect(s2, isNot(equals(s4)));
    expect(s4.fontFamily, isNull);
    expect(s4.fontSize, 10.0);
    expect(s4.fontWeight, FontWeight.w800);
    expect(s4.height, 123.0);
    expect(s4.color, const Color(0xFF00FF00));
    expect(s4.leadingDistribution, TextLeadingDistribution.even);

    final TextStyle s5 = TextStyle.lerp(s1, s3, 0.25)!;
    expect(s1.fontFamily, isNull);
    expect(s1.fontSize, 10.0);
    expect(s1.fontWeight, FontWeight.w800);
    expect(s1.height, 123.0);
    expect(s1.color, isNull);
    expect(s3.fontFamily, isNull);
    expect(s3.fontSize, 18.0);
    expect(s3.fontWeight, FontWeight.w400);
    expect(s3.height, 123.0);
    expect(s3.color, isNull);
    expect(s3, isNot(equals(s1)));
    expect(s3, isNot(equals(s5)));
    expect(s5.fontFamily, isNull);
    expect(s5.fontSize, 12.0);
    expect(s5.fontWeight, FontWeight.w700);
    expect(s5.height, 123.0);
    expect(s5.color, isNull);

    expect(TextStyle.lerp(null, null, 0.5), isNull);

    final TextStyle s6 = TextStyle.lerp(null, s3, 0.25)!;
    expect(s3.fontFamily, isNull);
    expect(s3.fontSize, 18.0);
    expect(s3.fontWeight, FontWeight.w400);
    expect(s3.height, 123.0);
    expect(s3.color, isNull);
    expect(s3, isNot(equals(s6)));
    expect(s6.fontFamily, isNull);
    expect(s6.fontSize, isNull);
    expect(s6.fontWeight, FontWeight.w400);
    expect(s6.height, isNull);
    expect(s6.color, isNull);

    final TextStyle s7 = TextStyle.lerp(null, s3, 0.75)!;
    expect(s3.fontFamily, isNull);
    expect(s3.fontSize, 18.0);
    expect(s3.fontWeight, FontWeight.w400);
    expect(s3.height, 123.0);
    expect(s3.color, isNull);
    expect(s3, equals(s7));
    expect(s7.fontFamily, isNull);
    expect(s7.fontSize, 18.0);
    expect(s7.fontWeight, FontWeight.w400);
    expect(s7.height, 123.0);
    expect(s7.color, isNull);

    final TextStyle s8 = TextStyle.lerp(s3, null, 0.25)!;
    expect(s3.fontFamily, isNull);
    expect(s3.fontSize, 18.0);
    expect(s3.fontWeight, FontWeight.w400);
    expect(s3.height, 123.0);
    expect(s3.color, isNull);
    expect(s3, equals(s8));
    expect(s8.fontFamily, isNull);
    expect(s8.fontSize, 18.0);
    expect(s8.fontWeight, FontWeight.w400);
    expect(s8.height, 123.0);
    expect(s8.color, isNull);

    final TextStyle s9 = TextStyle.lerp(s3, null, 0.75)!;
    expect(s3.fontFamily, isNull);
    expect(s3.fontSize, 18.0);
    expect(s3.fontWeight, FontWeight.w400);
    expect(s3.height, 123.0);
    expect(s3.color, isNull);
    expect(s3, isNot(equals(s9)));
    expect(s9.fontFamily, isNull);
    expect(s9.fontSize, isNull);
    expect(s9.fontWeight, FontWeight.w400);
    expect(s9.height, isNull);
    expect(s9.color, isNull);

    final ui.TextStyle ts5 = s5.getTextStyle();
    expect(ts5, equals(ui.TextStyle(fontWeight: FontWeight.w700, fontSize: 12.0, height: 123.0)));
    expect(ts5, matchesToStringOf(s5));
    final ui.TextStyle ts2 = s2.getTextStyle();
    expect(
      ts2,
      equals(
        ui.TextStyle(
          color: const Color(0xFF00FF00),
          fontWeight: FontWeight.w800,
          fontSize: 10.0,
          height: 100.0,
          leadingDistribution: TextLeadingDistribution.even,
        ),
      ),
    );
    expect(ts2, matchesToStringOf(s2));

    final ui.ParagraphStyle ps2 = s2.getParagraphStyle(textAlign: TextAlign.center);
    expect(
      ps2,
      equals(
        ui.ParagraphStyle(
          textAlign: TextAlign.center,
          fontWeight: FontWeight.w800,
          fontSize: 10.0,
          height: 100.0,
          textHeightBehavior: const TextHeightBehavior(
            leadingDistribution: TextLeadingDistribution.even,
          ),
        ),
      ),
    );
    final ui.ParagraphStyle ps5 = s5.getParagraphStyle();
    expect(
      ps5,
      equals(ui.ParagraphStyle(fontWeight: FontWeight.w700, fontSize: 12.0, height: 123.0)),
    );
  });

  test('TextStyle with text direction', () {
    final ui.ParagraphStyle ps6 = const TextStyle().getParagraphStyle(
      textDirection: TextDirection.ltr,
    );
    expect(ps6, equals(ui.ParagraphStyle(textDirection: TextDirection.ltr, fontSize: 14.0)));

    final ui.ParagraphStyle ps7 = const TextStyle().getParagraphStyle(
      textDirection: TextDirection.rtl,
    );
    expect(ps7, equals(ui.ParagraphStyle(textDirection: TextDirection.rtl, fontSize: 14.0)));
  });

  test('TextStyle using package font', () {
    const TextStyle s6 = TextStyle(fontFamily: 'test');
    expect(s6.fontFamily, 'test');
    expect(s6.getTextStyle(), matchesToStringOf(s6));

    const TextStyle s7 = TextStyle(fontFamily: 'test', package: 'p');
    expect(s7.fontFamily, 'packages/p/test');
    expect(s7.getTextStyle(), matchesToStringOf(s7));

    const TextStyle s8 = TextStyle(fontFamilyFallback: <String>['test', 'test2'], package: 'p');
    expect(s8.fontFamilyFallback![0], 'packages/p/test');
    expect(s8.fontFamilyFallback![1], 'packages/p/test2');
    expect(s8.fontFamilyFallback!.length, 2);

    const TextStyle s9 = TextStyle(package: 'p');
    expect(s9.fontFamilyFallback, null);

    const TextStyle s10 = TextStyle(fontFamilyFallback: <String>[], package: 'p');
    expect(s10.fontFamilyFallback, <String>[]);

    // Ensure that package prefix is not duplicated after copying.
    final TextStyle s11 = s8.copyWith();
    expect(s11.fontFamilyFallback![0], 'packages/p/test');
    expect(s11.fontFamilyFallback![1], 'packages/p/test2');
    expect(s11.fontFamilyFallback!.length, 2);
    expect(s8, s11);

    // Ensure that package prefix is not duplicated after applying.
    final TextStyle s12 = s8.apply();
    expect(s12.fontFamilyFallback![0], 'packages/p/test');
    expect(s12.fontFamilyFallback![1], 'packages/p/test2');
    expect(s12.fontFamilyFallback!.length, 2);
    expect(s8, s12);
  });

  test('TextStyle package font merge', () {
    const TextStyle s1 = TextStyle(
      package: 'p',
      fontFamily: 'font1',
      fontFamilyFallback: <String>['fallback1'],
    );
    const TextStyle s2 = TextStyle(
      package: 'p',
      fontFamily: 'font2',
      fontFamilyFallback: <String>['fallback2'],
    );

    final TextStyle emptyMerge = const TextStyle().merge(s1);
    expect(emptyMerge.fontFamily, 'packages/p/font1');
    expect(emptyMerge.fontFamilyFallback, <String>['packages/p/fallback1']);

    final TextStyle lerp1 = TextStyle.lerp(s1, s2, 0)!;
    expect(lerp1.fontFamily, 'packages/p/font1');
    expect(lerp1.fontFamilyFallback, <String>['packages/p/fallback1']);

    final TextStyle lerp2 = TextStyle.lerp(s1, s2, 1.0)!;
    expect(lerp2.fontFamily, 'packages/p/font2');
    expect(lerp2.fontFamilyFallback, <String>['packages/p/fallback2']);
  });

  test('TextStyle font family fallback', () {
    const TextStyle s1 = TextStyle(fontFamilyFallback: <String>['Roboto', 'test']);
    expect(s1.fontFamilyFallback![0], 'Roboto');
    expect(s1.fontFamilyFallback![1], 'test');
    expect(s1.fontFamilyFallback!.length, 2);

    const TextStyle s2 = TextStyle(
      fontFamily: 'foo',
      fontFamilyFallback: <String>['Roboto', 'test'],
    );
    expect(s2.fontFamilyFallback![0], 'Roboto');
    expect(s2.fontFamilyFallback![1], 'test');
    expect(s2.fontFamily, 'foo');
    expect(s2.fontFamilyFallback!.length, 2);

    const TextStyle s3 = TextStyle(fontFamily: 'foo');
    expect(s3.fontFamily, 'foo');
    expect(s3.fontFamilyFallback, null);

    const TextStyle s4 = TextStyle(fontFamily: 'foo', fontFamilyFallback: <String>[]);
    expect(s4.fontFamily, 'foo');
    expect(s4.fontFamilyFallback, <String>[]);
    expect(s4.fontFamilyFallback!.isEmpty, true);

    final ui.TextStyle uis1 = s2.getTextStyle();
    expect(uis1, matchesToStringOf(s2));

    expect(s2.apply().fontFamily, 'foo');
    expect(s2.apply().fontFamilyFallback, const <String>['Roboto', 'test']);
    expect(s2.apply(fontFamily: 'bar').fontFamily, 'bar');
    expect(
      s2.apply(fontFamilyFallback: const <String>['Banana']).fontFamilyFallback,
      const <String>['Banana'],
    );
  });

  test('TextStyle.debugLabel', () {
    const TextStyle unknown = TextStyle();
    const TextStyle foo = TextStyle(debugLabel: 'foo', fontSize: 1.0);
    const TextStyle bar = TextStyle(debugLabel: 'bar', fontSize: 2.0);
    const TextStyle baz = TextStyle(debugLabel: 'baz', fontSize: 3.0);

    expect(unknown.debugLabel, null);
    expect(unknown.toString(), 'TextStyle(<all styles inherited>)');
    expect(unknown.copyWith().debugLabel, null);
    expect(unknown.copyWith(debugLabel: '123').debugLabel, '123');
    expect(unknown.apply().debugLabel, null);

    expect(foo.debugLabel, 'foo');
    expect(foo.toString(), 'TextStyle(debugLabel: foo, inherit: true, size: 1.0)');
    expect(foo.merge(bar).debugLabel, '(foo).merge(bar)');
    expect(foo.merge(bar).merge(baz).debugLabel, '((foo).merge(bar)).merge(baz)');
    expect(foo.copyWith().debugLabel, '(foo).copyWith');
    expect(foo.apply().debugLabel, '(foo).apply');
    expect(TextStyle.lerp(foo, bar, 0.5)!.debugLabel, 'lerp(foo ⎯0.5→ bar)');
    expect(
      TextStyle.lerp(foo.merge(bar), baz, 0.51)!.copyWith().debugLabel,
      '(lerp((foo).merge(bar) ⎯0.5→ baz)).copyWith',
    );
  });

  test('TextStyle.hashCode', () {
    const TextStyle a = TextStyle(
      fontFamilyFallback: <String>['Roboto'],
      shadows: <ui.Shadow>[ui.Shadow()],
      fontFeatures: <ui.FontFeature>[ui.FontFeature('abcd')],
      fontVariations: <ui.FontVariation>[ui.FontVariation('wght', 123.0)],
    );
    const TextStyle b = TextStyle(
      fontFamilyFallback: <String>['Noto'],
      shadows: <ui.Shadow>[ui.Shadow()],
      fontFeatures: <ui.FontFeature>[ui.FontFeature('abcd')],
      fontVariations: <ui.FontVariation>[ui.FontVariation('wght', 123.0)],
    );
    expect(a.hashCode, a.hashCode);
    expect(a.hashCode, isNot(equals(b.hashCode)));

    const TextStyle c = TextStyle(leadingDistribution: TextLeadingDistribution.even);
    const TextStyle d = TextStyle(leadingDistribution: TextLeadingDistribution.proportional);
    expect(c.hashCode, c.hashCode);
    expect(c.hashCode, isNot(d.hashCode));
  });

  test('TextStyle shadows', () {
    const ui.Shadow shadow1 = ui.Shadow(blurRadius: 1.0, offset: Offset(1.0, 1.0));
    const ui.Shadow shadow2 = ui.Shadow(
      blurRadius: 2.0,
      color: Color(0xFF111111),
      offset: Offset(2.0, 2.0),
    );
    const ui.Shadow shadow3 = ui.Shadow(
      blurRadius: 3.0,
      color: Color(0xFF222222),
      offset: Offset(3.0, 3.0),
    );
    const ui.Shadow shadow4 = ui.Shadow(
      blurRadius: 4.0,
      color: Color(0xFF333333),
      offset: Offset(4.0, 4.0),
    );

    const TextStyle s1 = TextStyle(shadows: <ui.Shadow>[shadow1, shadow2]);
    const TextStyle s2 = TextStyle(shadows: <ui.Shadow>[shadow3, shadow4]);

    final TextStyle lerp12 = TextStyle.lerp(s1, s2, 0.5)!;

    expect(lerp12.shadows, hasLength(2));
    expect(
      lerp12.shadows?[0].blurRadius,
      ui.lerpDouble(shadow1.blurRadius, shadow3.blurRadius, 0.5),
    );
    expect(lerp12.shadows?[0].color, Color.lerp(shadow1.color, shadow3.color, 0.5));
    expect(lerp12.shadows?[0].offset, Offset.lerp(shadow1.offset, shadow3.offset, 0.5));
    expect(
      lerp12.shadows?[1].blurRadius,
      ui.lerpDouble(shadow2.blurRadius, shadow4.blurRadius, 0.5),
    );
    expect(lerp12.shadows?[1].color, Color.lerp(shadow2.color, shadow4.color, 0.5));
    expect(lerp12.shadows?[1].offset, Offset.lerp(shadow2.offset, shadow4.offset, 0.5));
  });

  test('TextStyle foreground and color combos', () {
    const Color red = Color.fromARGB(255, 255, 0, 0);
    const Color blue = Color.fromARGB(255, 0, 0, 255);
    const TextStyle redTextStyle = TextStyle(color: red);
    const TextStyle blueTextStyle = TextStyle(color: blue);
    final TextStyle redPaintTextStyle = TextStyle(foreground: Paint()..color = red);
    final TextStyle bluePaintTextStyle = TextStyle(foreground: Paint()..color = blue);

    // merge/copyWith
    final TextStyle redBlueBothForegroundMerged = redTextStyle.merge(blueTextStyle);
    expect(redBlueBothForegroundMerged.color, blue);
    expect(redBlueBothForegroundMerged.foreground, isNull);

    final TextStyle redBlueBothPaintMerged = redPaintTextStyle.merge(bluePaintTextStyle);
    expect(redBlueBothPaintMerged.color, null);
    expect(redBlueBothPaintMerged.foreground, bluePaintTextStyle.foreground);

    final TextStyle redPaintBlueColorMerged = redPaintTextStyle.merge(blueTextStyle);
    expect(redPaintBlueColorMerged.color, null);
    expect(redPaintBlueColorMerged.foreground, redPaintTextStyle.foreground);

    final TextStyle blueColorRedPaintMerged = blueTextStyle.merge(redPaintTextStyle);
    expect(blueColorRedPaintMerged.color, null);
    expect(blueColorRedPaintMerged.foreground, redPaintTextStyle.foreground);

    // apply
    expect(redPaintTextStyle.apply(color: blue).color, isNull);
    expect(redPaintTextStyle.apply(color: blue).foreground!.color, red);
    expect(redTextStyle.apply(color: blue).color, blue);

    // lerp
    expect(TextStyle.lerp(redTextStyle, blueTextStyle, .25)!.color, Color.lerp(red, blue, .25));
    expect(TextStyle.lerp(redTextStyle, bluePaintTextStyle, .25)!.color, isNull);
    expect(TextStyle.lerp(redTextStyle, bluePaintTextStyle, .25)!.foreground!.color, red);
    expect(TextStyle.lerp(redTextStyle, bluePaintTextStyle, .75)!.foreground!.color, blue);

    expect(TextStyle.lerp(redPaintTextStyle, bluePaintTextStyle, .25)!.color, isNull);
    expect(TextStyle.lerp(redPaintTextStyle, bluePaintTextStyle, .25)!.foreground!.color, red);
    expect(TextStyle.lerp(redPaintTextStyle, bluePaintTextStyle, .75)!.foreground!.color, blue);
  });

  test('backgroundColor', () {
    const TextStyle s1 = TextStyle();
    expect(s1.backgroundColor, isNull);
    expect(s1.toString(), 'TextStyle(<all styles inherited>)');

    const TextStyle s2 = TextStyle(backgroundColor: Color(0xFF00FF00));
    expect(s2.backgroundColor, const Color(0xFF00FF00));
    expect(s2.toString(), 'TextStyle(inherit: true, backgroundColor: ${const Color(0xff00ff00)})');

    final ui.TextStyle ts2 = s2.getTextStyle();

    // TODO(matanlurey): Remove when https://github.com/flutter/flutter/issues/112498 is resolved.
    // The web implementation never includes "dither: ..." as a property, and after #112498 neither
    // does non-web (as there will no longer be a user-visible "dither" property). So, relax the
    // test to just check for the color by using a regular expression.
    expect(ts2.toString(), matches(RegExp(r'background: Paint\(Color\(.*\).*\)')));
  });

  test('TextStyle background and backgroundColor combos', () {
    const Color red = Color.fromARGB(255, 255, 0, 0);
    const Color blue = Color.fromARGB(255, 0, 0, 255);
    const TextStyle redTextStyle = TextStyle(backgroundColor: red);
    const TextStyle blueTextStyle = TextStyle(backgroundColor: blue);
    final TextStyle redPaintTextStyle = TextStyle(background: Paint()..color = red);
    final TextStyle bluePaintTextStyle = TextStyle(background: Paint()..color = blue);

    // merge/copyWith
    final TextStyle redBlueBothForegroundMerged = redTextStyle.merge(blueTextStyle);
    expect(redBlueBothForegroundMerged.backgroundColor, blue);
    expect(redBlueBothForegroundMerged.foreground, isNull);

    final TextStyle redBlueBothPaintMerged = redPaintTextStyle.merge(bluePaintTextStyle);
    expect(redBlueBothPaintMerged.backgroundColor, null);
    expect(redBlueBothPaintMerged.background, bluePaintTextStyle.background);

    final TextStyle redPaintBlueColorMerged = redPaintTextStyle.merge(blueTextStyle);
    expect(redPaintBlueColorMerged.backgroundColor, null);
    expect(redPaintBlueColorMerged.background, redPaintTextStyle.background);

    final TextStyle blueColorRedPaintMerged = blueTextStyle.merge(redPaintTextStyle);
    expect(blueColorRedPaintMerged.backgroundColor, null);
    expect(blueColorRedPaintMerged.background, redPaintTextStyle.background);

    // apply
    expect(redPaintTextStyle.apply(backgroundColor: blue).backgroundColor, isNull);
    expect(redPaintTextStyle.apply(backgroundColor: blue).background!.color, red);
    expect(redTextStyle.apply(backgroundColor: blue).backgroundColor, blue);

    // lerp
    expect(
      TextStyle.lerp(redTextStyle, blueTextStyle, .25)!.backgroundColor,
      Color.lerp(red, blue, .25),
    );
    expect(TextStyle.lerp(redTextStyle, bluePaintTextStyle, .25)!.backgroundColor, isNull);
    expect(TextStyle.lerp(redTextStyle, bluePaintTextStyle, .25)!.background!.color, red);
    expect(TextStyle.lerp(redTextStyle, bluePaintTextStyle, .75)!.background!.color, blue);

    expect(TextStyle.lerp(redPaintTextStyle, bluePaintTextStyle, .25)!.backgroundColor, isNull);
    expect(TextStyle.lerp(redPaintTextStyle, bluePaintTextStyle, .25)!.background!.color, red);
    expect(TextStyle.lerp(redPaintTextStyle, bluePaintTextStyle, .75)!.background!.color, blue);
  });

  test('TextStyle strut textScaler', () {
    const TextStyle style0 = TextStyle(fontSize: 10);
    final ui.ParagraphStyle paragraphStyle0 = style0.getParagraphStyle(
      textScaler: const TextScaler.linear(2.5),
    );

    const TextStyle style1 = TextStyle(fontSize: 25);
    final ui.ParagraphStyle paragraphStyle1 = style1.getParagraphStyle();

    expect(paragraphStyle0 == paragraphStyle1, true);
  });

  test('TextStyle apply', () {
    const TextStyle style = TextStyle(
      fontSize: 10,
      shadows: <ui.Shadow>[],
      fontStyle: FontStyle.normal,
      fontFeatures: <ui.FontFeature>[],
      fontVariations: <ui.FontVariation>[],
      textBaseline: TextBaseline.alphabetic,
      leadingDistribution: TextLeadingDistribution.even,
    );
    expect(style.apply().shadows, const <ui.Shadow>[]);
    expect(
      style.apply(shadows: const <ui.Shadow>[ui.Shadow(blurRadius: 2.0)]).shadows,
      const <ui.Shadow>[ui.Shadow(blurRadius: 2.0)],
    );
    expect(style.apply().fontStyle, FontStyle.normal);
    expect(style.apply(fontStyle: FontStyle.italic).fontStyle, FontStyle.italic);
    expect(style.apply().locale, isNull);
    expect(
      style.apply(locale: const Locale.fromSubtags(languageCode: 'es')).locale,
      const Locale.fromSubtags(languageCode: 'es'),
    );
    expect(style.apply().fontFeatures, const <ui.FontFeature>[]);
    expect(
      style.apply(fontFeatures: const <ui.FontFeature>[ui.FontFeature.enable('test')]).fontFeatures,
      const <ui.FontFeature>[ui.FontFeature.enable('test')],
    );
    expect(style.apply().fontVariations, const <ui.FontVariation>[]);
    expect(
      style
          .apply(fontVariations: const <ui.FontVariation>[ui.FontVariation('test', 100.0)])
          .fontVariations,
      const <ui.FontVariation>[ui.FontVariation('test', 100.0)],
    );
    expect(style.apply().textBaseline, TextBaseline.alphabetic);
    expect(
      style.apply(textBaseline: TextBaseline.ideographic).textBaseline,
      TextBaseline.ideographic,
    );
    expect(style.apply().leadingDistribution, TextLeadingDistribution.even);
    expect(
      style.apply(leadingDistribution: TextLeadingDistribution.proportional).leadingDistribution,
      TextLeadingDistribution.proportional,
    );

    expect(
      const TextStyle(height: kTextHeightNone).apply(heightFactor: 1000, heightDelta: 1000).height,
      kTextHeightNone,
    );
  });

  test('TextStyle fontFamily and package', () {
    expect(
      const TextStyle(fontFamily: 'fontFamily', package: 'foo') !=
          const TextStyle(fontFamily: 'fontFamily', package: 'bar'),
      true,
    );
    expect(
      const TextStyle(fontFamily: 'fontFamily', package: 'foo').hashCode !=
          const TextStyle(package: 'bar', fontFamily: 'fontFamily').hashCode,
      true,
    );
    expect(const TextStyle(fontFamily: 'fontFamily').fontFamily, 'fontFamily');
    expect(const TextStyle(fontFamily: 'fontFamily').fontFamily, 'fontFamily');
    expect(
      const TextStyle(fontFamily: 'fontFamily').copyWith(package: 'bar').fontFamily,
      'packages/bar/fontFamily',
    );
    expect(
      const TextStyle(fontFamily: 'fontFamily', package: 'foo').fontFamily,
      'packages/foo/fontFamily',
    );
    expect(
      const TextStyle(fontFamily: 'fontFamily', package: 'foo').copyWith(package: 'bar').fontFamily,
      'packages/bar/fontFamily',
    );
    expect(
      const TextStyle().merge(const TextStyle(fontFamily: 'fontFamily', package: 'bar')).fontFamily,
      'packages/bar/fontFamily',
    );
    expect(
      const TextStyle().apply(fontFamily: 'fontFamily', package: 'foo').fontFamily,
      'packages/foo/fontFamily',
    );
    expect(
      const TextStyle(
        fontFamily: 'fontFamily',
        package: 'foo',
      ).apply(fontFamily: 'fontFamily', package: 'bar').fontFamily,
      'packages/bar/fontFamily',
    );
  });

  test('TextStyle.lerp identical a,b', () {
    expect(TextStyle.lerp(null, null, 0), null);
    const TextStyle style = TextStyle();
    expect(identical(TextStyle.lerp(style, style, 0.5), style), true);
  });

  test('Throws when lerping between inherit:true and inherit:false with unspecified fields', () {
    const TextStyle fromStyle = TextStyle();
    const TextStyle toStyle = TextStyle(inherit: false);
    expect(() => TextStyle.lerp(fromStyle, toStyle, 0.5), throwsFlutterError);
    expect(TextStyle.lerp(fromStyle, fromStyle, 0.5), fromStyle);
  });

  test(
    'Does not throw when lerping between inherit:true and inherit:false but fully specified styles',
    () {
      const TextStyle fromStyle = TextStyle();
      const TextStyle toStyle = TextStyle(
        inherit: false,
        color: Color(0x87654321),
        backgroundColor: Color(0x12345678),
        fontSize: 20,
        letterSpacing: 1,
        wordSpacing: 1,
        height: 20,
        decorationColor: Color(0x11111111),
        decorationThickness: 5,
      );
      expect(TextStyle.lerp(fromStyle, toStyle, 1), toStyle);
    },
  );

  test('lerpFontVariations', () {
    // nil cases
    expect(
      lerpFontVariations(const <FontVariation>[], const <FontVariation>[], 0.0),
      const <FontVariation>[],
    );
    expect(
      lerpFontVariations(const <FontVariation>[], const <FontVariation>[], 0.5),
      const <FontVariation>[],
    );
    expect(
      lerpFontVariations(const <FontVariation>[], const <FontVariation>[], 1.0),
      const <FontVariation>[],
    );
    expect(lerpFontVariations(null, const <FontVariation>[], 0.0), null);
    expect(lerpFontVariations(const <FontVariation>[], null, 0.0), const <FontVariation>[]);
    expect(lerpFontVariations(null, null, 0.0), null);
    expect(lerpFontVariations(null, const <FontVariation>[], 0.5), const <FontVariation>[]);
    expect(lerpFontVariations(const <FontVariation>[], null, 0.5), null);
    expect(lerpFontVariations(null, null, 0.5), null);
    expect(lerpFontVariations(null, const <FontVariation>[], 1.0), const <FontVariation>[]);
    expect(lerpFontVariations(const <FontVariation>[], null, 1.0), null);
    expect(lerpFontVariations(null, null, 1.0), null);

    const FontVariation w100 = FontVariation.weight(100.0);
    const FontVariation w120 = FontVariation.weight(120.0);
    const FontVariation w150 = FontVariation.weight(150.0);
    const FontVariation w200 = FontVariation.weight(200.0);
    const FontVariation w300 = FontVariation.weight(300.0);
    const FontVariation w1000 = FontVariation.weight(1000.0);

    // one axis
    expect(
      lerpFontVariations(const <FontVariation>[w100], const <FontVariation>[w200], 0.0),
      const <FontVariation>[w100],
    );
    expect(
      lerpFontVariations(const <FontVariation>[w100], const <FontVariation>[w200], 0.2),
      const <FontVariation>[w120],
    );
    expect(
      lerpFontVariations(const <FontVariation>[w100], const <FontVariation>[w200], 0.5),
      const <FontVariation>[w150],
    );
    expect(
      lerpFontVariations(const <FontVariation>[w100], const <FontVariation>[w200], 2.0),
      const <FontVariation>[w300],
    );

    // weird one axis cases
    expect(
      lerpFontVariations(const <FontVariation>[w100, w1000], const <FontVariation>[w300], 0.0),
      const <FontVariation>[w100, w1000],
    );
    expect(
      lerpFontVariations(const <FontVariation>[w100, w1000], const <FontVariation>[w300], 0.5),
      const <FontVariation>[w200],
    );
    expect(
      lerpFontVariations(const <FontVariation>[w100, w1000], const <FontVariation>[w300], 1.0),
      const <FontVariation>[w300],
    );
    expect(
      lerpFontVariations(const <FontVariation>[w100, w1000], const <FontVariation>[], 0.5),
      const <FontVariation>[],
    );

    const FontVariation sn80 = FontVariation.slant(-80.0);
    const FontVariation sn40 = FontVariation.slant(-40.0);
    const FontVariation s0 = FontVariation.slant(0.0);
    const FontVariation sp40 = FontVariation.slant(40.0);
    const FontVariation sp80 = FontVariation.slant(80.0);

    // two axis matched order
    expect(
      lerpFontVariations(const <FontVariation>[w100, sn80], const <FontVariation>[w300, sp80], 0.5),
      const <FontVariation>[w200, s0],
    );

    // two axis unmatched order
    expect(
      lerpFontVariations(const <FontVariation>[sn80, w100], const <FontVariation>[w300, sp80], 0.0),
      const <FontVariation>[sn80, w100],
    );
    expect(
      lerpFontVariations(const <FontVariation>[sn80, w100], const <FontVariation>[w300, sp80], 0.5),
      unorderedMatches(const <FontVariation>[s0, w200]),
    );
    expect(
      lerpFontVariations(const <FontVariation>[sn80, w100], const <FontVariation>[w300, sp80], 1.0),
      const <FontVariation>[w300, sp80],
    );

    // two axis with duplicates
    expect(
      lerpFontVariations(
        const <FontVariation>[sn80, w100, sp80],
        const <FontVariation>[w300, sp80],
        0.5,
      ),
      unorderedMatches(const <FontVariation>[sp80, w200]),
    );

    // mixed axis counts
    expect(
      lerpFontVariations(const <FontVariation>[sn80, w100], const <FontVariation>[w300], 0.5),
      const <FontVariation>[w200],
    );
    expect(
      lerpFontVariations(const <FontVariation>[sn80], const <FontVariation>[w300], 0.0),
      const <FontVariation>[sn80],
    );
    expect(
      lerpFontVariations(const <FontVariation>[sn80], const <FontVariation>[w300], 0.1),
      const <FontVariation>[sn80],
    );
    expect(
      lerpFontVariations(const <FontVariation>[sn80], const <FontVariation>[w300], 0.9),
      const <FontVariation>[w300],
    );
    expect(
      lerpFontVariations(const <FontVariation>[sn80], const <FontVariation>[w300], 1.0),
      const <FontVariation>[w300],
    );
    expect(
      lerpFontVariations(
        const <FontVariation>[sn40, s0, w100],
        const <FontVariation>[sp40, w300, sp80],
        0.5,
      ),
      anyOf(
        equals(const <FontVariation>[s0, w200, sp40]),
        equals(const <FontVariation>[s0, sp40, w200]),
      ),
    );
  });
}
