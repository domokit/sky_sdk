// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

TestLine l(
  String? displayText,
  int? startIndex,
  int? endIndex, {
  bool? hardBreak,
  double? height,
  double? width,
  double? widthWithTrailingSpaces,
  double? left,
  double? baseline,
}) {
  return TestLine(
    displayText: displayText,
    startIndex: startIndex,
    endIndex: endIndex,
    hardBreak: hardBreak,
    height: height,
    width: width,
    widthWithTrailingSpaces: widthWithTrailingSpaces,
    left: left,
    baseline: baseline,
  );
}

void expectLines(CanvasParagraph paragraph, List<TestLine> expectedLines) {
  final List<ParagraphLine> lines = paragraph.lines;
  expect(lines, hasLength(expectedLines.length));
  for (int i = 0; i < lines.length; i++) {
    final ParagraphLine line = lines[i];
    final TestLine expectedLine = expectedLines[i];

    expect(
      line.lineNumber,
      i,
      reason: 'line #$i had the wrong `lineNumber`. Expected: $i. Actual: ${line.lineNumber}',
    );
    if (expectedLine.displayText != null) {
      final String displayText = line.getText(paragraph);
      expect(
        displayText,
        expectedLine.displayText,
        reason:
            'line #$i had a different `displayText` value: "$displayText" vs. "${expectedLine.displayText}"',
      );
    }
    if (expectedLine.startIndex != null) {
      expect(
        line.startIndex,
        expectedLine.startIndex,
        reason:
            'line #$i had a different `startIndex` value: "${line.startIndex}" vs. "${expectedLine.startIndex}"',
      );
    }
    if (expectedLine.endIndex != null) {
      expect(
        line.endIndex,
        expectedLine.endIndex,
        reason:
            'line #$i had a different `endIndex` value: "${line.endIndex}" vs. "${expectedLine.endIndex}"',
      );
    }
    if (expectedLine.hardBreak != null) {
      expect(
        line.hardBreak,
        expectedLine.hardBreak,
        reason:
            'line #$i had a different `hardBreak` value: "${line.hardBreak}" vs. "${expectedLine.hardBreak}"',
      );
    }
    if (expectedLine.height != null) {
      expect(
        line.height,
        expectedLine.height,
        reason:
            'line #$i had a different `height` value: "${line.height}" vs. "${expectedLine.height}"',
      );
    }
    if (expectedLine.width != null) {
      expect(
        line.width,
        expectedLine.width,
        reason:
            'line #$i had a different `width` value: "${line.width}" vs. "${expectedLine.width}"',
      );
    }
    if (expectedLine.widthWithTrailingSpaces != null) {
      expect(
        line.widthWithTrailingSpaces,
        expectedLine.widthWithTrailingSpaces,
        reason:
            'line #$i had a different `widthWithTrailingSpaces` value: "${line.widthWithTrailingSpaces}" vs. "${expectedLine.widthWithTrailingSpaces}"',
      );
    }
    if (expectedLine.left != null) {
      expect(
        line.left,
        expectedLine.left,
        reason: 'line #$i had a different `left` value: "${line.left}" vs. "${expectedLine.left}"',
      );
    }
    if (expectedLine.baseline != null) {
      expect(
        line.baseline,
        expectedLine.baseline,
        reason:
            'line #$i had a different `baseline` value: "${line.baseline}" vs. "${expectedLine.baseline}"',
      );
    }
  }
}

class TestLine {
  TestLine({
    this.displayText,
    this.startIndex,
    this.endIndex,
    this.hardBreak,
    this.height,
    this.width,
    this.widthWithTrailingSpaces,
    this.left,
    this.baseline,
  });

  final String? displayText;
  final int? startIndex;
  final int? endIndex;
  final bool? hardBreak;
  final double? height;
  final double? width;
  final double? widthWithTrailingSpaces;
  final double? left;
  final double? baseline;
}
