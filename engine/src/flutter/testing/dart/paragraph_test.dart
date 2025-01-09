// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:test/test.dart';

void main() {
  // Ahem font uses a constant ideographic/alphabetic baseline ratio.
  const double kAhemBaselineRatio = 1.25;

  test('predictably lays out a single-line paragraph - Ahem', () {
    for (final double fontSize in <double>[10.0, 20.0, 30.0, 40.0]) {
      final ParagraphBuilder builder = ParagraphBuilder(
        ParagraphStyle(
          fontFamily: 'Ahem',
          fontStyle: FontStyle.normal,
          fontWeight: FontWeight.normal,
          fontSize: fontSize,
        ),
      );
      builder.addText('Test');
      final Paragraph paragraph = builder.build();
      paragraph.layout(const ParagraphConstraints(width: 400.0));

      expect(paragraph.height, closeTo(fontSize, 0.001));
      expect(paragraph.width, closeTo(400.0, 0.001));
      expect(paragraph.minIntrinsicWidth, closeTo(fontSize * 4.0, 0.001));
      expect(paragraph.maxIntrinsicWidth, closeTo(fontSize * 4.0, 0.001));
      expect(paragraph.alphabeticBaseline, closeTo(fontSize * .8, 0.001));
      expect(
        paragraph.ideographicBaseline,
        closeTo(paragraph.alphabeticBaseline * kAhemBaselineRatio, 0.001),
      );
    }
  });

  test('predictably lays out a single-line paragraph - FlutterTest', () {
    for (final double fontSize in <double>[10.0, 20.0, 30.0, 40.0]) {
      final ParagraphBuilder builder = ParagraphBuilder(
        ParagraphStyle(
          fontFamily: 'FlutterTest',
          fontStyle: FontStyle.normal,
          fontWeight: FontWeight.normal,
          fontSize: fontSize,
        ),
      );
      builder.addText('Test');
      final Paragraph paragraph = builder.build();
      paragraph.layout(const ParagraphConstraints(width: 400.0));

      expect(paragraph.height, fontSize);
      expect(paragraph.width, 400.0);
      expect(paragraph.minIntrinsicWidth, fontSize * 4.0);
      expect(paragraph.maxIntrinsicWidth, fontSize * 4.0);
      expect(paragraph.alphabeticBaseline, fontSize * 0.75);
      expect(paragraph.ideographicBaseline, fontSize);
    }
  });

  test('predictably lays out a multi-line paragraph', () {
    for (final double fontSize in <double>[10.0, 20.0, 30.0, 40.0]) {
      final ParagraphBuilder builder = ParagraphBuilder(
        ParagraphStyle(
          fontFamily: 'Ahem',
          fontStyle: FontStyle.normal,
          fontWeight: FontWeight.normal,
          fontSize: fontSize,
        ),
      );
      builder.addText('Test Ahem');
      final Paragraph paragraph = builder.build();
      paragraph.layout(ParagraphConstraints(width: fontSize * 5.0));

      expect(paragraph.height, closeTo(fontSize * 2.0, 0.001)); // because it wraps
      expect(paragraph.width, closeTo(fontSize * 5.0, 0.001));
      expect(paragraph.minIntrinsicWidth, closeTo(fontSize * 4.0, 0.001));

      // TODO(yjbanov): see https://github.com/flutter/flutter/issues/21965
      expect(paragraph.maxIntrinsicWidth, closeTo(fontSize * 9.0, 0.001));
      expect(paragraph.alphabeticBaseline, closeTo(fontSize * .8, 0.001));
      expect(
        paragraph.ideographicBaseline,
        closeTo(paragraph.alphabeticBaseline * kAhemBaselineRatio, 0.001),
      );
    }
  });

  test('getLineBoundary', () {
    const double fontSize = 10.0;
    final ParagraphBuilder builder = ParagraphBuilder(
      ParagraphStyle(
        fontFamily: 'Ahem',
        fontStyle: FontStyle.normal,
        fontWeight: FontWeight.normal,
        fontSize: fontSize,
      ),
    );
    builder.addText('Test Ahem');
    final Paragraph paragraph = builder.build();
    paragraph.layout(const ParagraphConstraints(width: fontSize * 5.0));

    // Wraps to two lines.
    expect(paragraph.height, closeTo(fontSize * 2.0, 0.001));

    const TextPosition wrapPositionDown = TextPosition(offset: 5);
    TextRange line = paragraph.getLineBoundary(wrapPositionDown);
    expect(line.start, 5);
    expect(line.end, 9);

    const TextPosition wrapPositionUp = TextPosition(offset: 5, affinity: TextAffinity.upstream);
    line = paragraph.getLineBoundary(wrapPositionUp);
    expect(line.start, 0);
    expect(line.end, 5);

    const TextPosition wrapPositionStart = TextPosition(offset: 0);
    line = paragraph.getLineBoundary(wrapPositionStart);
    expect(line.start, 0);
    expect(line.end, 5);

    const TextPosition wrapPositionEnd = TextPosition(offset: 9);
    line = paragraph.getLineBoundary(wrapPositionEnd);
    expect(line.start, 5);
    expect(line.end, 9);
  });

  test('getLineBoundary RTL', () {
    const double fontSize = 10.0;
    final ParagraphBuilder builder = ParagraphBuilder(
      ParagraphStyle(
        fontFamily: 'Ahem',
        fontStyle: FontStyle.normal,
        fontWeight: FontWeight.normal,
        fontSize: fontSize,
        textDirection: TextDirection.rtl,
      ),
    );
    builder.addText('القاهرةالقاهرة');
    final Paragraph paragraph = builder.build();
    paragraph.layout(const ParagraphConstraints(width: fontSize * 5.0));

    // Wraps to three lines.
    expect(paragraph.height, closeTo(fontSize * 3.0, 0.001));

    const TextPosition wrapPositionDown = TextPosition(offset: 5);
    TextRange line = paragraph.getLineBoundary(wrapPositionDown);
    expect(line.start, 5);
    expect(line.end, 10);

    const TextPosition wrapPositionUp = TextPosition(offset: 5, affinity: TextAffinity.upstream);
    line = paragraph.getLineBoundary(wrapPositionUp);
    expect(line.start, 0);
    expect(line.end, 5);

    const TextPosition wrapPositionStart = TextPosition(offset: 0);
    line = paragraph.getLineBoundary(wrapPositionStart);
    expect(line.start, 0);
    expect(line.end, 5);

    const TextPosition wrapPositionEnd = TextPosition(offset: 9);
    line = paragraph.getLineBoundary(wrapPositionEnd);
    expect(line.start, 5);
    expect(line.end, 10);
  });

  test('getLineBoundary empty line', () {
    const double fontSize = 10.0;
    final ParagraphBuilder builder = ParagraphBuilder(
      ParagraphStyle(
        fontFamily: 'Ahem',
        fontStyle: FontStyle.normal,
        fontWeight: FontWeight.normal,
        fontSize: fontSize,
        textDirection: TextDirection.rtl,
      ),
    );
    builder.addText('Test\n\nAhem');
    final Paragraph paragraph = builder.build();
    paragraph.layout(const ParagraphConstraints(width: fontSize * 5.0));

    // Three lines due to line breaks, with the middle line being empty.
    expect(paragraph.height, closeTo(fontSize * 3.0, 0.001));

    const TextPosition emptyLinePosition = TextPosition(offset: 5);
    TextRange line = paragraph.getLineBoundary(emptyLinePosition);
    expect(line.start, 5);
    expect(line.end, 5);

    // Since these are hard newlines, TextAffinity has no effect here.
    const TextPosition emptyLinePositionUpstream = TextPosition(
      offset: 5,
      affinity: TextAffinity.upstream,
    );
    line = paragraph.getLineBoundary(emptyLinePositionUpstream);
    expect(line.start, 5);
    expect(line.end, 5);

    const TextPosition endOfFirstLinePosition = TextPosition(offset: 4);
    line = paragraph.getLineBoundary(endOfFirstLinePosition);
    expect(line.start, 0);
    expect(line.end, 4);

    const TextPosition startOfLastLinePosition = TextPosition(offset: 6);
    line = paragraph.getLineBoundary(startOfLastLinePosition);
    expect(line.start, 6);
    expect(line.end, 10);
  });

  test('getLineMetricsAt', () {
    const double fontSize = 10.0;
    final ParagraphBuilder builder = ParagraphBuilder(
      ParagraphStyle(fontSize: fontSize, textDirection: TextDirection.rtl, height: 2.0),
    );
    builder.addText('Test\npppp');
    final Paragraph paragraph = builder.build();
    paragraph.layout(const ParagraphConstraints(width: 100.0));
    final LineMetrics? line = paragraph.getLineMetricsAt(1);
    expect(line?.hardBreak, isTrue);
    expect(line?.ascent, 15.0);
    expect(line?.descent, 5.0);
    expect(line?.height, 20.0);
    expect(line?.width, 4 * 10.0);
    expect(line?.left, 100.0 - 40.0);
    expect(line?.baseline, 20.0 + 15.0);
    expect(line?.lineNumber, 1);
  });

  test('line number', () {
    const double fontSize = 10.0;
    final ParagraphBuilder builder = ParagraphBuilder(ParagraphStyle(fontSize: fontSize));
    builder.addText('Test\n\nTest');
    final Paragraph paragraph = builder.build();
    paragraph.layout(const ParagraphConstraints(width: 100.0));
    expect(paragraph.numberOfLines, 3);
    expect(paragraph.getLineNumberAt(4), 0); // first LF
    expect(paragraph.getLineNumberAt(5), 1); // second LF
    expect(paragraph.getLineNumberAt(6), 2); // "T" in the second "Test"
  });

  test('empty paragraph', () {
    const double fontSize = 10.0;
    final Paragraph paragraph = ParagraphBuilder(ParagraphStyle(fontSize: fontSize)).build();
    paragraph.layout(const ParagraphConstraints(width: double.infinity));

    expect(paragraph.getClosestGlyphInfoForOffset(Offset.zero), isNull);
    expect(paragraph.getGlyphInfoAt(0), isNull);

    expect(paragraph.getLineMetricsAt(0), isNull);
    expect(paragraph.numberOfLines, 0);
    expect(paragraph.getLineNumberAt(0), isNull);

    expect(paragraph.getGlyphInfoAt(0), isNull);
    expect(paragraph.getClosestGlyphInfoForOffset(Offset.zero), isNull);
  });

  test('OOB indices as input', () {
    const double fontSize = 10.0;
    final ParagraphBuilder builder = ParagraphBuilder(
      ParagraphStyle(fontSize: fontSize, maxLines: 1, ellipsis: 'BBB'),
    )..addText('A' * 100);
    final Paragraph paragraph = builder.build();
    paragraph.layout(const ParagraphConstraints(width: 100));

    expect(paragraph.numberOfLines, 1);

    expect(paragraph.getLineMetricsAt(-1), isNull);
    expect(paragraph.getLineMetricsAt(0)?.lineNumber, 0);
    expect(paragraph.getLineMetricsAt(1), isNull);

    expect(paragraph.getLineNumberAt(-1), isNull);
    expect(paragraph.getLineNumberAt(0), 0);
    expect(paragraph.getLineNumberAt(6), 0);
    // The last 3 characters on the first line are ellipsized with BBB.
    expect(paragraph.getLineMetricsAt(7), isNull);

    expect(paragraph.getGlyphInfoAt(-1), isNull);
    expect(
      paragraph.getGlyphInfoAt(0)?.graphemeClusterCodeUnitRange,
      const TextRange(start: 0, end: 1),
    );
    expect(
      paragraph.getGlyphInfoAt(6)?.graphemeClusterCodeUnitRange,
      const TextRange(start: 6, end: 7),
    );
    expect(paragraph.getGlyphInfoAt(7), isNull);
    expect(paragraph.getGlyphInfoAt(200), isNull);
  });

  test('querying glyph info', () {
    const double fontSize = 10.0;
    final ParagraphBuilder builder = ParagraphBuilder(ParagraphStyle(fontSize: fontSize));
    builder.addText('Test\nTest');
    final Paragraph paragraph = builder.build();
    paragraph.layout(const ParagraphConstraints(width: double.infinity));

    final GlyphInfo? bottomRight = paragraph.getClosestGlyphInfoForOffset(const Offset(99.0, 99.0));
    final GlyphInfo? last = paragraph.getGlyphInfoAt(8);
    expect(bottomRight, equals(last));
    expect(bottomRight, isNot(paragraph.getGlyphInfoAt(0)));

    expect(bottomRight?.graphemeClusterLayoutBounds, const Rect.fromLTWH(30, 10, 10, 10));
    expect(bottomRight?.graphemeClusterCodeUnitRange, const TextRange(start: 8, end: 9));
    expect(bottomRight?.writingDirection, TextDirection.ltr);
  });

  test('painting a disposed paragraph does not crash', () {
    final Paragraph paragraph = ParagraphBuilder(ParagraphStyle()).build();
    paragraph.dispose();

    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);

    void callback() {
      canvas.drawParagraph(paragraph, Offset.zero);
    }

    expect(callback, throwsA(isA<AssertionError>()));
  });

  test('rounding hack disabled', () {
    const double fontSize = 1.25;
    const String text = '12345';

    expect((fontSize * text.length).truncate(), isNot(fontSize * text.length));
    final ParagraphBuilder builder = ParagraphBuilder(ParagraphStyle(fontSize: fontSize));
    builder.addText(text);
    final Paragraph paragraph =
        builder.build()..layout(const ParagraphConstraints(width: text.length * fontSize));
    expect(paragraph.maxIntrinsicWidth, text.length * fontSize);
    switch (paragraph.computeLineMetrics()) {
      case [LineMetrics(width: final double width)]:
        expect(width, text.length * fontSize);
      case final List<LineMetrics> metrics:
        expect(metrics, hasLength(1));
    }
  });

  test('kTextHeightNone unsets the height multiplier', () {
    const double fontSize = 10;
    final ParagraphBuilder builder = ParagraphBuilder(
      ParagraphStyle(fontSize: fontSize, height: 10),
    );
    builder.pushStyle(TextStyle(height: kTextHeightNone));
    builder.addText('A');
    final Paragraph paragraph = builder.build()..layout(const ParagraphConstraints(width: 1000));
    expect(paragraph.height, fontSize);
  });

  test('kTextHeightNone ParagraphStyle', () {
    const double fontSize = 10;
    final ParagraphBuilder builder = ParagraphBuilder(
      ParagraphStyle(fontSize: fontSize, height: kTextHeightNone),
    );
    builder.addText('A');
    final Paragraph paragraph = builder.build()..layout(const ParagraphConstraints(width: 1000));
    expect(paragraph.height, fontSize);
  });

  test('kTextHeightNone StrutStyle', () {
    const double fontSize = 10;
    final ParagraphBuilder builder = ParagraphBuilder(
      ParagraphStyle(
        fontSize: 100,
        strutStyle: StrutStyle(forceStrutHeight: true, height: kTextHeightNone, fontSize: fontSize),
      ),
    );
    builder.addText('A');
    final Paragraph paragraph = builder.build()..layout(const ParagraphConstraints(width: 1000));
    expect(paragraph.height, fontSize);
  });
}
