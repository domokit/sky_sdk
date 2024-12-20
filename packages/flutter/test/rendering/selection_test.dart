// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  const Rect rect = Rect.fromLTWH(100, 100, 200, 500);
  const Offset outsideTopLeft = Offset(50, 50);
  const Offset outsideLeft = Offset(50, 200);
  const Offset outsideBottomLeft = Offset(50, 700);
  const Offset outsideTop = Offset(200, 50);
  const Offset outsideTopRight = Offset(350, 50);
  const Offset outsideRight = Offset(350, 200);
  const Offset outsideBottomRight = Offset(350, 700);
  const Offset outsideBottom = Offset(200, 700);
  const Offset center = Offset(150, 300);

  group('selection utils', () {
    test('selectionBasedOnRect works', () {
      expect(SelectionUtils.getResultBasedOnRect(rect, outsideTopLeft), SelectionResult.previous);
      expect(SelectionUtils.getResultBasedOnRect(rect, outsideLeft), SelectionResult.previous);
      expect(SelectionUtils.getResultBasedOnRect(rect, outsideBottomLeft), SelectionResult.next);
      expect(SelectionUtils.getResultBasedOnRect(rect, outsideTop), SelectionResult.previous);
      expect(SelectionUtils.getResultBasedOnRect(rect, outsideTopRight), SelectionResult.previous);
      expect(SelectionUtils.getResultBasedOnRect(rect, outsideRight), SelectionResult.next);
      expect(SelectionUtils.getResultBasedOnRect(rect, outsideBottomRight), SelectionResult.next);
      expect(SelectionUtils.getResultBasedOnRect(rect, outsideBottom), SelectionResult.next);
      expect(SelectionUtils.getResultBasedOnRect(rect, center), SelectionResult.end);
    });

    test('adjustDragOffset works', () {
      // ltr
      expect(SelectionUtils.adjustDragOffset(rect, outsideTopLeft), rect.topLeft);
      expect(SelectionUtils.adjustDragOffset(rect, outsideLeft), rect.topLeft);
      expect(SelectionUtils.adjustDragOffset(rect, outsideBottomLeft), rect.bottomRight);
      expect(SelectionUtils.adjustDragOffset(rect, outsideTop), rect.topLeft);
      expect(SelectionUtils.adjustDragOffset(rect, outsideTopRight), rect.topLeft);
      expect(SelectionUtils.adjustDragOffset(rect, outsideRight), rect.bottomRight);
      expect(SelectionUtils.adjustDragOffset(rect, outsideBottomRight), rect.bottomRight);
      expect(SelectionUtils.adjustDragOffset(rect, outsideBottom), rect.bottomRight);
      expect(SelectionUtils.adjustDragOffset(rect, center), center);
      // rtl
      expect(
        SelectionUtils.adjustDragOffset(rect, outsideTopLeft, direction: TextDirection.rtl),
        rect.topRight,
      );
      expect(
        SelectionUtils.adjustDragOffset(rect, outsideLeft, direction: TextDirection.rtl),
        rect.topRight,
      );
      expect(
        SelectionUtils.adjustDragOffset(rect, outsideBottomLeft, direction: TextDirection.rtl),
        rect.bottomLeft,
      );
      expect(
        SelectionUtils.adjustDragOffset(rect, outsideTop, direction: TextDirection.rtl),
        rect.topRight,
      );
      expect(
        SelectionUtils.adjustDragOffset(rect, outsideTopRight, direction: TextDirection.rtl),
        rect.topRight,
      );
      expect(
        SelectionUtils.adjustDragOffset(rect, outsideRight, direction: TextDirection.rtl),
        rect.bottomLeft,
      );
      expect(
        SelectionUtils.adjustDragOffset(rect, outsideBottomRight, direction: TextDirection.rtl),
        rect.bottomLeft,
      );
      expect(
        SelectionUtils.adjustDragOffset(rect, outsideBottom, direction: TextDirection.rtl),
        rect.bottomLeft,
      );
      expect(SelectionUtils.adjustDragOffset(rect, center, direction: TextDirection.rtl), center);
    });
  });
}
