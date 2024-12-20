// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  testWidgets('Can set opacity for an Icon', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: IconTheme(
          data: IconThemeData(color: Color(0xFF666666), opacity: 0.5),
          child: Icon(IconData(0xd0a0, fontFamily: 'Arial')),
        ),
      ),
    );
    final RichText text = tester.widget(find.byType(RichText));
    expect(text.text.style!.color, const Color(0xFF666666).withOpacity(0.5));
  });

  testWidgets('Icon sizing - no theme, default size', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(textDirection: TextDirection.ltr, child: Center(child: Icon(null))),
    );

    final RenderBox renderObject = tester.renderObject(find.byType(Icon));
    expect(renderObject.size, equals(const Size.square(24.0)));
  });

  testWidgets('Icon sizing - no theme, explicit size', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: Icon(null, size: 96.0)),
      ),
    );

    final RenderBox renderObject = tester.renderObject(find.byType(Icon));
    expect(renderObject.size, equals(const Size.square(96.0)));
  });

  testWidgets('Icon sizing - sized theme', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: IconTheme(data: IconThemeData(size: 36.0), child: Icon(null))),
      ),
    );

    final RenderBox renderObject = tester.renderObject(find.byType(Icon));
    expect(renderObject.size, equals(const Size.square(36.0)));
  });

  testWidgets('Icon sizing - sized theme, explicit size', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: IconTheme(data: IconThemeData(size: 36.0), child: Icon(null, size: 48.0)),
        ),
      ),
    );

    final RenderBox renderObject = tester.renderObject(find.byType(Icon));
    expect(renderObject.size, equals(const Size.square(48.0)));
  });

  testWidgets('Icon sizing - sizeless theme, default size', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: IconTheme(data: IconThemeData(), child: Icon(null))),
      ),
    );

    final RenderBox renderObject = tester.renderObject(find.byType(Icon));
    expect(renderObject.size, equals(const Size.square(24.0)));
  });

  testWidgets('Icon sizing - no theme, default size, considering text scaler', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(textScaler: _TextDoubler()),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(child: Icon(null, applyTextScaling: true)),
        ),
      ),
    );

    final RenderBox renderObject = tester.renderObject(find.byType(Icon));
    expect(renderObject.size, equals(const Size.square(48.0)));
  });

  testWidgets('Icon sizing - no theme, explicit size, considering text scaler', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(textScaler: _TextDoubler()),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(child: Icon(null, size: 96.0, applyTextScaling: true)),
        ),
      ),
    );

    final RenderBox renderObject = tester.renderObject(find.byType(Icon));
    expect(renderObject.size, equals(const Size.square(192.0)));
  });

  testWidgets('Icon sizing - sized theme, considering text scaler', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(textScaler: _TextDoubler()),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: IconTheme(
              data: IconThemeData(size: 36.0, applyTextScaling: true),
              child: Icon(null),
            ),
          ),
        ),
      ),
    );

    final RenderBox renderObject = tester.renderObject(find.byType(Icon));
    expect(renderObject.size, equals(const Size.square(72.0)));
  });

  testWidgets('Icon sizing - sized theme, explicit size, considering text scale', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(textScaler: _TextDoubler()),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: IconTheme(
              data: IconThemeData(size: 36.0, applyTextScaling: true),
              child: Icon(null, size: 48.0, applyTextScaling: false),
            ),
          ),
        ),
      ),
    );

    final RenderBox renderObject = tester.renderObject(find.byType(Icon));
    expect(renderObject.size, equals(const Size.square(48.0)));
  });

  testWidgets('Icon sizing - sizeless theme, default size, default consideration for text scaler', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(textScaler: _TextDoubler()),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(child: IconTheme(data: IconThemeData(), child: Icon(null))),
        ),
      ),
    );

    final RenderBox renderObject = tester.renderObject(find.byType(Icon));
    expect(renderObject.size, equals(const Size.square(24.0)));
  });

  testWidgets('Icon with custom font', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: Icon(IconData(0x41, fontFamily: 'Roboto'))),
      ),
    );

    final RichText richText = tester.firstWidget(find.byType(RichText));
    expect(richText.text.style!.fontFamily, equals('Roboto'));
  });

  testWidgets("Icon's TextStyle makes sure the font body is vertically center-aligned", (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/138592.
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: Icon(IconData(0x41))),
      ),
    );

    final RichText richText = tester.firstWidget(find.byType(RichText));
    expect(richText.text.style?.height, 1.0);
    expect(richText.text.style?.leadingDistribution, TextLeadingDistribution.even);
  });

  testWidgets('Icon with custom fontFamilyFallback', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: Icon(IconData(0x41, fontFamilyFallback: <String>['FallbackFont']))),
      ),
    );

    final RichText richText = tester.firstWidget(find.byType(RichText));
    expect(richText.text.style!.fontFamilyFallback, equals(<String>['FallbackFont']));
  });

  testWidgets('Icon with semantic label', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: Icon(Icons.title, semanticLabel: 'a label')),
      ),
    );

    expect(semantics, includesNodeWith(label: 'a label'));

    semantics.dispose();
  });

  testWidgets('Null icon with semantic label', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: Icon(null, semanticLabel: 'a label')),
      ),
    );

    expect(semantics, includesNodeWith(label: 'a label'));

    semantics.dispose();
  });

  testWidgets("Changing semantic label from null doesn't rebuild tree ", (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: Icon(Icons.time_to_leave)),
      ),
    );

    final Element richText1 = tester.element(find.byType(RichText));

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: Icon(Icons.time_to_leave, semanticLabel: 'a label')),
      ),
    );

    final Element richText2 = tester.element(find.byType(RichText));

    // Compare a leaf Element in the Icon subtree before and after changing the
    // semanticLabel to make sure the subtree was not rebuilt.
    expect(richText2, same(richText1));
  });

  testWidgets('IconData comparison', (WidgetTester tester) async {
    expect(const IconData(123), const IconData(123));
    expect(const IconData(123), isNot(const IconData(123, matchTextDirection: true)));
    expect(const IconData(123), isNot(const IconData(123, fontFamily: 'f')));
    expect(const IconData(123), isNot(const IconData(123, fontPackage: 'p')));
    expect(const IconData(123).hashCode, const IconData(123).hashCode);
    expect(
      const IconData(123).hashCode,
      isNot(const IconData(123, matchTextDirection: true).hashCode),
    );
    expect(const IconData(123).hashCode, isNot(const IconData(123, fontFamily: 'f').hashCode));
    expect(const IconData(123).hashCode, isNot(const IconData(123, fontPackage: 'p').hashCode));
    expect(const IconData(123).toString(), 'IconData(U+0007B)');
  });

  testWidgets('Fill, weight, grade, and optical size variations are passed', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const Directionality(textDirection: TextDirection.ltr, child: Icon(Icons.abc)),
    );

    RichText text = tester.widget(find.byType(RichText));
    expect(text.text.style!.fontVariations, <FontVariation>[
      const FontVariation('FILL', 0.0),
      const FontVariation('wght', 400.0),
      const FontVariation('GRAD', 0.0),
      const FontVariation('opsz', 48.0),
    ]);

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Icon(Icons.abc, fill: 0.5, weight: 300, grade: 200, opticalSize: 48),
      ),
    );

    text = tester.widget(find.byType(RichText));
    expect(text.text.style!.fontVariations, isNotNull);
    expect(text.text.style!.fontVariations, <FontVariation>[
      const FontVariation('FILL', 0.5),
      const FontVariation('wght', 300.0),
      const FontVariation('GRAD', 200.0),
      const FontVariation('opsz', 48.0),
    ]);
  });

  testWidgets('Fill, weight, grade, and optical size can be set at the theme-level', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: IconTheme(
          data: IconThemeData(fill: 0.2, weight: 3.0, grade: 4.0, opticalSize: 5.0),
          child: Icon(Icons.abc),
        ),
      ),
    );

    final RichText text = tester.widget(find.byType(RichText));
    expect(text.text.style!.fontVariations, <FontVariation>[
      const FontVariation('FILL', 0.2),
      const FontVariation('wght', 3.0),
      const FontVariation('GRAD', 4.0),
      const FontVariation('opsz', 5.0),
    ]);
  });

  testWidgets('Theme-level fill, weight, grade, and optical size can be overridden', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: IconTheme(
          data: IconThemeData(fill: 0.2, weight: 3.0, grade: 4.0, opticalSize: 5.0),
          child: Icon(Icons.abc, fill: 0.6, weight: 7.0, grade: 8.0, opticalSize: 9.0),
        ),
      ),
    );

    final RichText text = tester.widget(find.byType(RichText));
    expect(text.text.style!.fontVariations, isNotNull);
    expect(text.text.style!.fontVariations, <FontVariation>[
      const FontVariation('FILL', 0.6),
      const FontVariation('wght', 7.0),
      const FontVariation('GRAD', 8.0),
      const FontVariation('opsz', 9.0),
    ]);
  });

  testWidgets("TextStyle's blendMode should bind in foreground property", (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: IconTheme(
            data: IconThemeData(opacity: 0.5),
            child: Icon(IconData(0x41), blendMode: BlendMode.clear),
          ),
        ),
      ),
    );

    final RichText richText = tester.firstWidget(find.byType(RichText));
    expect(richText.text.style?.color, isNull);
    expect(richText.text.style?.foreground?.blendMode, BlendMode.clear);
  });

  test('Throws if given invalid values', () {
    expect(() => Icon(Icons.abc, fill: -0.1), throwsAssertionError);
    expect(() => Icon(Icons.abc, fill: 1.1), throwsAssertionError);
    expect(() => Icon(Icons.abc, weight: -0.1), throwsAssertionError);
    expect(() => Icon(Icons.abc, weight: 0.0), throwsAssertionError);
    expect(() => Icon(Icons.abc, opticalSize: -0.1), throwsAssertionError);
    expect(() => Icon(Icons.abc, opticalSize: 0), throwsAssertionError);
  });
}

final class _TextDoubler extends TextScaler {
  const _TextDoubler();

  @override
  double scale(double fontSize) => fontSize * 2.0;

  @override
  double get textScaleFactor => 2.0;
}
