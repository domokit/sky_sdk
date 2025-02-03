// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('list body and paragraph intrinsics', () {
    final RenderParagraph paragraph = RenderParagraph(
      const TextSpan(style: TextStyle(height: 1.0), text: 'Hello World'),
      textDirection: TextDirection.ltr,
    );
    final RenderListBody testBlock = RenderListBody(children: <RenderBox>[paragraph]);

    final double textWidth = paragraph.getMaxIntrinsicWidth(double.infinity);
    final double oneLineTextHeight = paragraph.getMinIntrinsicHeight(double.infinity);
    final double constrainedWidth = textWidth * 0.9;
    final double wrappedTextWidth = paragraph.getMinIntrinsicWidth(double.infinity);
    final double twoLinesTextHeight = paragraph.getMinIntrinsicHeight(constrainedWidth);
    final double manyLinesTextHeight = paragraph.getMinIntrinsicHeight(0.0);

    // paragraph
    expect(wrappedTextWidth, greaterThan(0.0));
    expect(wrappedTextWidth, lessThan(textWidth));
    expect(oneLineTextHeight, lessThan(twoLinesTextHeight));
    expect(twoLinesTextHeight, lessThan(oneLineTextHeight * 3.0));
    expect(manyLinesTextHeight, greaterThan(twoLinesTextHeight));
    expect(paragraph.getMaxIntrinsicHeight(double.infinity), equals(oneLineTextHeight));
    expect(paragraph.getMaxIntrinsicHeight(constrainedWidth), equals(twoLinesTextHeight));
    expect(paragraph.getMaxIntrinsicHeight(0.0), equals(manyLinesTextHeight));

    // vertical block (same expectations)
    expect(testBlock.getMinIntrinsicWidth(double.infinity), equals(wrappedTextWidth));
    expect(testBlock.getMaxIntrinsicWidth(double.infinity), equals(textWidth));
    expect(testBlock.getMinIntrinsicHeight(double.infinity), equals(oneLineTextHeight));
    expect(testBlock.getMinIntrinsicHeight(constrainedWidth), equals(twoLinesTextHeight));
    expect(testBlock.getMaxIntrinsicHeight(double.infinity), equals(oneLineTextHeight));
    expect(testBlock.getMaxIntrinsicHeight(constrainedWidth), equals(twoLinesTextHeight));
    expect(testBlock.getMinIntrinsicWidth(0.0), equals(wrappedTextWidth));
    expect(testBlock.getMaxIntrinsicWidth(0.0), equals(textWidth));
    expect(testBlock.getMinIntrinsicHeight(wrappedTextWidth), equals(twoLinesTextHeight));
    expect(testBlock.getMaxIntrinsicHeight(wrappedTextWidth), equals(twoLinesTextHeight));
    expect(testBlock.getMinIntrinsicHeight(0.0), equals(manyLinesTextHeight));
    expect(testBlock.getMaxIntrinsicHeight(0.0), equals(manyLinesTextHeight));

    // horizontal block (same expectations again)
    testBlock.axisDirection = AxisDirection.right;
    expect(testBlock.getMinIntrinsicWidth(double.infinity), equals(wrappedTextWidth));
    expect(testBlock.getMaxIntrinsicWidth(double.infinity), equals(textWidth));
    expect(testBlock.getMinIntrinsicHeight(double.infinity), equals(oneLineTextHeight));
    expect(testBlock.getMinIntrinsicHeight(constrainedWidth), equals(twoLinesTextHeight));
    expect(testBlock.getMaxIntrinsicHeight(double.infinity), equals(oneLineTextHeight));
    expect(testBlock.getMaxIntrinsicHeight(constrainedWidth), equals(twoLinesTextHeight));
    expect(testBlock.getMinIntrinsicWidth(0.0), equals(wrappedTextWidth));
    expect(testBlock.getMaxIntrinsicWidth(0.0), equals(textWidth));
    expect(testBlock.getMinIntrinsicHeight(wrappedTextWidth), equals(twoLinesTextHeight));
    expect(testBlock.getMaxIntrinsicHeight(wrappedTextWidth), equals(twoLinesTextHeight));
    expect(testBlock.getMinIntrinsicHeight(0.0), equals(manyLinesTextHeight));
    expect(testBlock.getMaxIntrinsicHeight(0.0), equals(manyLinesTextHeight));
  });

  test('textScaler affects intrinsics', () {
    final RenderParagraph paragraph = RenderParagraph(
      const TextSpan(style: TextStyle(fontSize: 10), text: 'Hello World'),
      textDirection: TextDirection.ltr,
    );

    expect(paragraph.getMaxIntrinsicWidth(double.infinity), 110);

    paragraph.textScaler = const TextScaler.linear(2);
    expect(paragraph.getMaxIntrinsicWidth(double.infinity), 220);
  });

  test('maxLines affects intrinsics', () {
    final RenderParagraph paragraph = RenderParagraph(
      TextSpan(style: const TextStyle(fontSize: 10), text: List<String>.filled(5, 'A').join('\n')),
      textDirection: TextDirection.ltr,
    );

    expect(paragraph.getMaxIntrinsicHeight(double.infinity), 50);

    paragraph.maxLines = 1;
    expect(paragraph.getMaxIntrinsicHeight(double.infinity), 10);
  });

  test('strutStyle affects intrinsics', () {
    final RenderParagraph paragraph = RenderParagraph(
      const TextSpan(style: TextStyle(fontSize: 10), text: 'Hello World'),
      textDirection: TextDirection.ltr,
    );

    expect(paragraph.getMaxIntrinsicHeight(double.infinity), 10);

    paragraph.strutStyle = const StrutStyle(fontSize: 100, forceStrutHeight: true);
    expect(paragraph.getMaxIntrinsicHeight(double.infinity), 100);
  });
}
