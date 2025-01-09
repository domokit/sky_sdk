// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:test/test.dart';

void main() {
  test('Should be able to build and layout a paragraph', () {
    final ParagraphBuilder builder = ParagraphBuilder(ParagraphStyle());
    builder.addText('Hello');
    final Paragraph paragraph = builder.build();
    expect(paragraph, isNotNull);

    paragraph.layout(const ParagraphConstraints(width: 800.0));
    expect(paragraph.width, isNonZero);
    expect(paragraph.height, isNonZero);
  });

  test('PushStyle should not segfault after build()', () {
    final ParagraphBuilder paragraphBuilder = ParagraphBuilder(ParagraphStyle());
    paragraphBuilder.build();
    expect(() {
      paragraphBuilder.pushStyle(TextStyle());
    }, throwsStateError);
  });

  test('GetRectsForRange smoke test', () {
    final ParagraphBuilder builder = ParagraphBuilder(ParagraphStyle());
    builder.addText('Hello');
    final Paragraph paragraph = builder.build();
    expect(paragraph, isNotNull);

    paragraph.layout(const ParagraphConstraints(width: 800.0));
    expect(paragraph.width, isNonZero);
    expect(paragraph.height, isNonZero);

    final List<TextBox> boxes = paragraph.getBoxesForRange(0, 3);
    expect(boxes.length, 1);
    expect(boxes.first.left, 0.0);
    expect(boxes.first.top, 0.0);
    expect(boxes.first.right, 42.0);
    expect(boxes.first.bottom, 14.0);
    expect(boxes.first.direction, TextDirection.ltr);
  });

  test('LineMetrics smoke test', () {
    final ParagraphBuilder builder = ParagraphBuilder(ParagraphStyle());
    builder.addText('Hello');
    final Paragraph paragraph = builder.build();
    expect(paragraph, isNotNull);

    paragraph.layout(const ParagraphConstraints(width: 800.0));
    expect(paragraph.width, isNonZero);
    expect(paragraph.height, isNonZero);

    final List<LineMetrics> metrics = paragraph.computeLineMetrics();
    expect(metrics.length, 1);
    expect(metrics.first.hardBreak, true);
    expect(metrics.first.ascent, 10.5);
    expect(metrics.first.descent, 3.5);
    expect(metrics.first.unscaledAscent, 10.5);
    expect(metrics.first.height, 14.0);
    expect(metrics.first.width, 70.0);
    expect(metrics.first.left, 0.0);
    expect(metrics.first.baseline, 10.5);
    expect(metrics.first.lineNumber, 0);
  });
}
