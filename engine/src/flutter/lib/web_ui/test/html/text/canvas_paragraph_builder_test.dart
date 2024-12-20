// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../../common/test_initialization.dart';
import '../paragraph/helper.dart';

/// Some text measurements are sensitive to browser implementations. Position
/// info in the following tests only pass in Chrome, they are slightly different
/// on each browser. So we need to ignore position info on non-Chrome browsers
/// when comparing expectations with actual output.
bool get isBlink => ui_web.browser.browserEngine == ui_web.BrowserEngine.blink;

String fontFamilyToAttribute(String fontFamily) {
  fontFamily = canonicalizeFontFamily(fontFamily)!;
  if (ui_web.browser.browserEngine == ui_web.BrowserEngine.firefox) {
    return fontFamily.replaceAll('"', '&quot;');
  } else if (ui_web.browser.browserEngine == ui_web.BrowserEngine.blink ||
      ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit) {
    return fontFamily.replaceAll('"', '');
  }
  return fontFamily;
}

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(withImplicitView: true);

  test('empty paragraph', () {
    final CanvasParagraph paragraph1 = rich(
      EngineParagraphStyle(),
      (CanvasParagraphBuilder builder) {},
    );
    expect(paragraph1.plainText, '');
    expect(paragraph1.spans, hasLength(1));
    expect(paragraph1.spans.single.start, 0);
    expect(paragraph1.spans.single.end, 0);

    final CanvasParagraph paragraph2 = rich(EngineParagraphStyle(), (
      CanvasParagraphBuilder builder,
    ) {
      builder.addText('');
    });
    expect(paragraph2.plainText, '');
    expect(paragraph2.spans, hasLength(1));
    expect(paragraph2.spans.single.start, 0);
    expect(paragraph2.spans.single.end, 0);
  });

  test('Builds a text-only canvas paragraph', () {
    final EngineParagraphStyle style = EngineParagraphStyle(fontSize: 13.0);
    final CanvasParagraphBuilder builder = CanvasParagraphBuilder(style);

    builder.addText('Hello');

    final CanvasParagraph paragraph = builder.build();
    expect(paragraph.paragraphStyle, style);
    expect(paragraph.plainText, 'Hello');
    expect(paragraph.spans, hasLength(1));

    paragraph.layout(const ParagraphConstraints(width: double.infinity));
    expectOuterHtml(
      paragraph,
      '<flt-paragraph style="${paragraphStyle()}">'
      '<flt-span style="${spanStyle(top: 0, left: 0, width: 13 * 5, fontSize: 13)}">'
      'Hello'
      '</flt-span>'
      '</flt-paragraph>',
      ignorePositions: !isBlink,
    );

    // Should break "Hello" into "Hel" and "lo".
    paragraph.layout(const ParagraphConstraints(width: 39.0));
    expectOuterHtml(
      paragraph,
      '<flt-paragraph style="${paragraphStyle()}">'
      '<flt-span style="${spanStyle(top: 0, left: 0, width: 13 * 3, fontSize: 13)}">'
      'Hel'
      '</flt-span>'
      '<flt-span style="${spanStyle(top: 13, left: 0, width: 13 * 2, fontSize: 13)}">'
      'lo'
      '</flt-span>'
      '</flt-paragraph>',
      ignorePositions: !isBlink,
    );

    final ParagraphSpan span = paragraph.spans.single;
    expect(getSpanText(paragraph, span), 'Hello');
    expect(span.style, styleWithDefaults(fontSize: 13.0));
  });

  test('Correct defaults', () {
    final EngineParagraphStyle style = EngineParagraphStyle();
    final CanvasParagraphBuilder builder = CanvasParagraphBuilder(style);

    builder.addText('Hello');

    final CanvasParagraph paragraph = builder.build();
    expect(paragraph.paragraphStyle, style);
    expect(paragraph.plainText, 'Hello');
    expect(paragraph.spans, hasLength(1));

    paragraph.layout(const ParagraphConstraints(width: double.infinity));
    expectOuterHtml(
      paragraph,
      '<flt-paragraph style="${paragraphStyle()}">'
      '<flt-span style="${spanStyle(top: 0, left: 0, width: 14 * 5)}">'
      'Hello'
      '</flt-span>'
      '</flt-paragraph>',
      ignorePositions: !isBlink,
    );

    final ParagraphSpan span = paragraph.spans.single;
    expect(span.style, styleWithDefaults());
  });

  test('Sets correct styles for max-lines', () {
    final EngineParagraphStyle style = EngineParagraphStyle(maxLines: 2);
    final CanvasParagraphBuilder builder = CanvasParagraphBuilder(style);

    builder.addText('Hello');

    final CanvasParagraph paragraph = builder.build();
    expect(paragraph.paragraphStyle, style);
    expect(paragraph.plainText, 'Hello');

    paragraph.layout(const ParagraphConstraints(width: double.infinity));
    expectOuterHtml(
      paragraph,
      '<flt-paragraph style="${paragraphStyle()}">'
      '<flt-span style="${spanStyle(top: 0, left: 0, width: 14 * 5)}">'
      'Hello'
      '</flt-span>'
      '</flt-paragraph>',
      ignorePositions: !isBlink,
    );
  });

  test('Sets correct styles for ellipsis', () {
    final EngineParagraphStyle style = EngineParagraphStyle(ellipsis: '...');
    final CanvasParagraphBuilder builder = CanvasParagraphBuilder(style);

    builder.addText('HelloWorld');

    final CanvasParagraph paragraph = builder.build();
    expect(paragraph.paragraphStyle, style);
    expect(paragraph.plainText, 'HelloWorld');

    paragraph.layout(const ParagraphConstraints(width: 100.0));
    expectOuterHtml(
      paragraph,
      '<flt-paragraph style="${paragraphStyle()}">'
      '<flt-span style="${spanStyle(top: 0, left: 0, width: 14 * 4)}">'
      'Hell'
      '</flt-span>'
      '<flt-span style="${spanStyle(top: 0, left: 14 * 4, width: 14 * 3)}">'
      '...'
      '</flt-span>'
      '</flt-paragraph>',
      ignorePositions: !isBlink,
    );
  });

  test('Builds a single-span paragraph with complex styles', () {
    final EngineParagraphStyle style = EngineParagraphStyle(fontSize: 13.0, height: 1.5);
    final CanvasParagraphBuilder builder = CanvasParagraphBuilder(style);

    builder.pushStyle(TextStyle(fontSize: 9.0));
    builder.pushStyle(TextStyle(fontWeight: FontWeight.bold));
    builder.pushStyle(TextStyle(fontSize: 40.0));
    builder.pop();
    builder.pushStyle(TextStyle(fontStyle: FontStyle.italic, letterSpacing: 2.0));
    builder.addText('Hello');

    final CanvasParagraph paragraph = builder.build();
    expect(paragraph.plainText, 'Hello');
    expect(paragraph.spans, hasLength(1));

    paragraph.layout(const ParagraphConstraints(width: double.infinity));
    expectOuterHtml(
      paragraph,
      '<flt-paragraph style="${paragraphStyle()}">'
      '<flt-span style="${spanStyle(top: 0, left: 0, width: (9 + 2) * 5, lineHeight: 1.5 * 9, fontSize: 9, fontWeight: 'bold', fontStyle: 'italic', letterSpacing: 2)}">'
      'Hello'
      '</flt-span>'
      '</flt-paragraph>',
      ignorePositions: !isBlink,
    );

    final ParagraphSpan span = paragraph.spans.single;
    expect(getSpanText(paragraph, span), 'Hello');
    expect(
      span.style,
      styleWithDefaults(
        height: 1.5,
        fontSize: 9.0,
        fontWeight: FontWeight.bold,
        fontStyle: FontStyle.italic,
        letterSpacing: 2.0,
      ),
    );
  });

  test('Builds a multi-span paragraph', () {
    final EngineParagraphStyle style = EngineParagraphStyle(fontSize: 13.0);
    final CanvasParagraphBuilder builder = CanvasParagraphBuilder(style);

    builder.pushStyle(TextStyle(fontWeight: FontWeight.bold));
    builder.addText('Hello');
    builder.pop();
    builder.pushStyle(TextStyle(fontStyle: FontStyle.italic));
    builder.addText(' world');

    final CanvasParagraph paragraph = builder.build();
    expect(paragraph.plainText, 'Hello world');
    expect(paragraph.spans, hasLength(2));

    paragraph.layout(const ParagraphConstraints(width: double.infinity));
    expectOuterHtml(
      paragraph,
      '<flt-paragraph style="${paragraphStyle()}">'
      '<flt-span style="${spanStyle(top: 0, left: 0, width: 13 * 5, fontSize: 13, fontWeight: 'bold')}">'
      'Hello'
      '</flt-span>'
      '<flt-span style="${spanStyle(top: 0, left: 65, width: 13 * 1, fontSize: 13, fontStyle: 'italic')}">'
      ' '
      '</flt-span>'
      '<flt-span style="${spanStyle(top: 0, left: 78, width: 13 * 5, fontSize: 13, fontStyle: 'italic')}">'
      'world'
      '</flt-span>'
      '</flt-paragraph>',
      ignorePositions: !isBlink,
    );

    // Should break "Hello world" into 2 lines: "Hello " and "world".
    paragraph.layout(const ParagraphConstraints(width: 75.0));
    expectOuterHtml(
      paragraph,
      '<flt-paragraph style="${paragraphStyle()}">'
      '<flt-span style="${spanStyle(top: 0, left: 0, width: 13 * 5, fontSize: 13, fontWeight: 'bold')}">'
      'Hello'
      '</flt-span>'
      '<flt-span style="${spanStyle(top: 0, left: 65, width: 0, fontSize: 13, fontStyle: 'italic')}">'
      ' '
      '</flt-span>'
      '<flt-span style="${spanStyle(top: 13, left: 0, width: 13 * 5, fontSize: 13, fontStyle: 'italic')}">'
      'world'
      '</flt-span>'
      '</flt-paragraph>',
      ignorePositions: !isBlink,
    );

    final ParagraphSpan hello = paragraph.spans.first;
    expect(getSpanText(paragraph, hello), 'Hello');
    expect(hello.style, styleWithDefaults(fontSize: 13.0, fontWeight: FontWeight.bold));

    final ParagraphSpan world = paragraph.spans.last;
    expect(getSpanText(paragraph, world), ' world');
    expect(world.style, styleWithDefaults(fontSize: 13.0, fontStyle: FontStyle.italic));
  });

  test('Builds a multi-span paragraph with complex styles', () {
    final EngineParagraphStyle style = EngineParagraphStyle(fontSize: 13.0);
    final CanvasParagraphBuilder builder = CanvasParagraphBuilder(style);

    builder.pushStyle(TextStyle(fontWeight: FontWeight.bold));
    builder.pushStyle(TextStyle(height: 2.0));
    builder.addText('Hello');
    builder.pop(); // pop TextStyle(height: 2.0).
    builder.pushStyle(TextStyle(fontStyle: FontStyle.italic));
    builder.addText(' world');
    builder.pushStyle(TextStyle(fontWeight: FontWeight.normal));
    builder.addText('!');

    final CanvasParagraph paragraph = builder.build();
    expect(paragraph.plainText, 'Hello world!');
    expect(paragraph.spans, hasLength(3));

    paragraph.layout(const ParagraphConstraints(width: double.infinity));
    expectOuterHtml(
      paragraph,
      '<flt-paragraph style="${paragraphStyle()}">'
      '<flt-span style="${spanStyle(top: 0, left: 0, width: 13 * 5, lineHeight: 2 * 13, fontSize: 13, fontWeight: 'bold')}">'
      'Hello'
      '</flt-span>'
      '<flt-span style="${spanStyle(top: 6, left: 65, width: 13 * 1, fontSize: 13, fontWeight: 'bold', fontStyle: 'italic')}">'
      ' '
      '</flt-span>'
      '<flt-span style="${spanStyle(top: 6, left: 78, width: 13 * 5, fontSize: 13, fontWeight: 'bold', fontStyle: 'italic')}">'
      'world'
      '</flt-span>'
      '<flt-span style="${spanStyle(top: 6, left: 143, width: 13 * 1, fontSize: 13, fontWeight: 'normal', fontStyle: 'italic')}">'
      '!'
      '</flt-span>'
      '</flt-paragraph>',
      ignorePositions: !isBlink,
    );

    final ParagraphSpan hello = paragraph.spans[0];
    expect(getSpanText(paragraph, hello), 'Hello');
    expect(
      hello.style,
      styleWithDefaults(fontSize: 13.0, fontWeight: FontWeight.bold, height: 2.0),
    );

    final ParagraphSpan world = paragraph.spans[1];
    expect(getSpanText(paragraph, world), ' world');
    expect(
      world.style,
      styleWithDefaults(fontSize: 13.0, fontWeight: FontWeight.bold, fontStyle: FontStyle.italic),
    );

    final ParagraphSpan bang = paragraph.spans[2];
    expect(getSpanText(paragraph, bang), '!');
    expect(
      bang.style,
      styleWithDefaults(fontSize: 13.0, fontWeight: FontWeight.normal, fontStyle: FontStyle.italic),
    );
  });

  test('Paragraph with new lines generates correct DOM', () {
    final EngineParagraphStyle style = EngineParagraphStyle(fontSize: 13.0);
    final CanvasParagraphBuilder builder = CanvasParagraphBuilder(style);

    builder.addText('First\nSecond ');
    builder.pushStyle(TextStyle(fontStyle: FontStyle.italic));
    builder.addText('ThirdLongLine');

    final CanvasParagraph paragraph = builder.build();
    expect(paragraph.plainText, 'First\nSecond ThirdLongLine');
    expect(paragraph.spans, hasLength(2));

    // There's a new line between "First" and "Second", but "Second" and
    // "ThirdLongLine" remain together since constraints are infinite.
    paragraph.layout(const ParagraphConstraints(width: double.infinity));
    expectOuterHtml(
      paragraph,
      '<flt-paragraph style="${paragraphStyle()}">'
      '<flt-span style="${spanStyle(top: 0, left: 0, width: 13 * 5, fontSize: 13)}">'
      'First'
      '</flt-span>'
      '<flt-span style="${spanStyle(top: 13, left: 0, width: 13 * 6, fontSize: 13)}">'
      'Second'
      '</flt-span>'
      '<flt-span style="${spanStyle(top: 13, left: 13 * 6, width: 13 * 1, fontSize: 13)}">'
      ' '
      '</flt-span>'
      '<flt-span style="${spanStyle(top: 13, left: 13 * 7, width: 13 * 13, fontSize: 13, fontStyle: 'italic')}">'
      'ThirdLongLine'
      '</flt-span>'
      '</flt-paragraph>',
      ignorePositions: !isBlink,
    );

    // Should break the paragraph into "First", "Second " and "ThirdLongLine".
    paragraph.layout(const ParagraphConstraints(width: 180.0));
    expectOuterHtml(
      paragraph,
      '<flt-paragraph style="${paragraphStyle()}">'
      '<flt-span style="${spanStyle(top: 0, left: 0, width: 13 * 5, fontSize: 13)}">'
      'First'
      '</flt-span>'
      '<flt-span style="${spanStyle(top: 13, left: 0, width: 13 * 6, fontSize: 13)}">'
      'Second'
      '</flt-span>'
      '<flt-span style="${spanStyle(top: 13, left: 13 * 6, width: 0, fontSize: 13)}">'
      ' '
      '</flt-span>'
      '<flt-span style="${spanStyle(top: 26, left: 0, width: 13 * 13, fontSize: 13, fontStyle: 'italic')}">'
      'ThirdLongLine'
      '</flt-span>'
      '</flt-paragraph>',
      ignorePositions: !isBlink,
    );
  });

  test('various font sizes', () {
    // Paragraphs and spans force the FlutterTest font in test mode. We need to
    // trick them into thinking they are not in test mode, so they use the
    // provided font family.
    ui_web.debugEmulateFlutterTesterEnvironment = false;
    final EngineParagraphStyle style = EngineParagraphStyle(fontSize: 12.0, fontFamily: 'first');
    final CanvasParagraphBuilder builder = CanvasParagraphBuilder(style);

    builder.addText('First ');
    builder.pushStyle(TextStyle(fontSize: 18.0, fontFamily: 'second'));
    builder.addText('Second ');
    builder.pushStyle(TextStyle(fontSize: 10.0, fontFamily: 'third'));
    builder.addText('Third');

    final CanvasParagraph paragraph = builder.build();
    expect(paragraph.plainText, 'First Second Third');
    expect(paragraph.spans, hasLength(3));

    // The paragraph should take the font size and family from the span with the
    // greatest font size.
    paragraph.layout(const ParagraphConstraints(width: double.infinity));
    expectOuterHtml(
      paragraph,
      '<flt-paragraph style="${paragraphStyle()}">'
      '<flt-span style="${spanStyle(top: null, left: null, width: null, fontSize: 12, fontFamily: 'first')}">'
      'First'
      '</flt-span>'
      '<flt-span style="${spanStyle(top: null, left: null, width: null, fontSize: 12, fontFamily: 'first')}">'
      ' '
      '</flt-span>'
      '<flt-span style="${spanStyle(top: null, left: null, width: null, fontSize: 18, fontFamily: 'second')}">'
      'Second'
      '</flt-span>'
      '<flt-span style="${spanStyle(top: null, left: null, width: null, fontSize: 18, fontFamily: 'second')}">'
      ' '
      '</flt-span>'
      '<flt-span style="${spanStyle(top: null, left: null, width: null, fontSize: 10, fontFamily: 'third')}">'
      'Third'
      '</flt-span>'
      '</flt-paragraph>',
      // Since we are using unknown font families, we can't predict the text
      // measurements.
      ignorePositions: true,
    );
    ui_web.debugEmulateFlutterTesterEnvironment = true;
  });

  // Regression test for https://github.com/flutter/flutter/issues/108431.
  // Set dir attribute for RTL fragments in order to let the browser
  // handle mirrored characters.
  test('Sets "dir" attribute for RTL fragment', () {
    final EngineParagraphStyle style = EngineParagraphStyle(
      fontSize: 20.0,
      textDirection: TextDirection.rtl,
    );
    final CanvasParagraphBuilder builder = CanvasParagraphBuilder(style);

    builder.addText('(1)');

    final CanvasParagraph paragraph = builder.build();
    expect(paragraph.paragraphStyle, style);
    expect(paragraph.plainText, '(1)');

    paragraph.layout(const ParagraphConstraints(width: double.infinity));
    expectOuterHtml(
      paragraph,
      '<flt-paragraph style="${paragraphStyle()}">'
      '<flt-span dir="rtl" style="${spanStyle(top: null, left: null, width: null, fontSize: 20)}">'
      '('
      '</flt-span>'
      '<flt-span style="${spanStyle(top: null, left: null, width: null, fontSize: 20)}">'
      '1'
      '</flt-span>'
      '<flt-span dir="rtl" style="${spanStyle(top: null, left: null, width: null, fontSize: 20)}">'
      ')'
      '</flt-span>'
      '</flt-paragraph>',
      ignorePositions: true,
    );
  });
}

const String defaultFontFamily = 'FlutterTest';
const num defaultFontSize = 14;

String paragraphStyle() {
  return <String>['position: absolute;', 'white-space: pre;'].join(' ');
}

String spanStyle({
  required num? top,
  required num? left,
  required num? width,
  String fontFamily = defaultFontFamily,
  num fontSize = defaultFontSize,
  String? fontWeight,
  String? fontStyle,
  num? lineHeight,
  num? letterSpacing,
}) {
  return <String>[
    'font-size: ${fontSize}px;',
    if (fontWeight != null) 'font-weight: $fontWeight;',
    if (fontStyle != null) 'font-style: $fontStyle;',
    'font-family: ${fontFamilyToAttribute(fontFamily)};',
    if (letterSpacing != null) 'letter-spacing: ${letterSpacing}px;',
    'position: absolute;',
    if (top != null) 'top: ${top}px;',
    if (left != null) 'left: ${left}px;',
    if (width != null) 'width: ${width}px;',
    'line-height: ${lineHeight ?? fontSize}px;',
  ].join(' ');
}

TextStyle styleWithDefaults({
  String fontFamily = StyleManager.defaultFontFamily,
  double fontSize = StyleManager.defaultFontSize,
  FontWeight? fontWeight,
  FontStyle? fontStyle,
  double? height,
  double? letterSpacing,
}) {
  return TextStyle(
    fontFamily: fontFamily,
    fontSize: fontSize,
    fontWeight: fontWeight,
    fontStyle: fontStyle,
    height: height,
    letterSpacing: letterSpacing,
  );
}

void expectOuterHtml(CanvasParagraph paragraph, String expected, {required bool ignorePositions}) {
  String outerHtml = paragraph.toDomElement().outerHTML!;
  if (ignorePositions) {
    outerHtml = removeMeasurementInfo(outerHtml);
    expected = removeMeasurementInfo(expected);
  }

  expect(outerHtml, expected);
}

/// Removes CSS styles that are based on text measurement from the given html
/// string.
///
/// Examples: top, left, line-height, width.
///
/// This is needed when the measurement is unknown or could be different
/// depending on browser and environment.
String removeMeasurementInfo(String outerHtml) {
  return outerHtml
      .replaceAll(RegExp(r'\s*line-height:\s*[\d\.]+px\s*;\s*'), '')
      .replaceAll(RegExp(r'\s*width:\s*[\d\.]+px\s*;\s*'), '')
      .replaceAll(RegExp(r'\s*top:\s*[\d\.]+px\s*;\s*'), '')
      .replaceAll(RegExp(r'\s*left:\s*[\d\.]+px\s*;\s*'), '');
}
