// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_api_samples/material/selection_container/selection_container.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets(
    'The SelectionContainer should transform the partial selection into an all selection',
    (WidgetTester tester) async {
      await tester.pumpWidget(const example.SelectionContainerExampleApp());

      expect(find.widgetWithText(AppBar, 'SelectionContainer Sample'), findsOne);

      final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Row 1'), matching: find.byType(RichText)),
      );
      final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Row 2'), matching: find.byType(RichText)),
      );
      final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Row 3'), matching: find.byType(RichText)),
      );
      final Rect paragraph1Rect = tester.getRect(find.text('Row 1'));
      final TestGesture gesture = await tester.startGesture(
        paragraph1Rect.topLeft,
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();

      await gesture.moveTo(paragraph1Rect.center);
      await tester.pump();
      expect(paragraph1.selections.first, const TextSelection(baseOffset: 0, extentOffset: 5));
      expect(paragraph2.selections.first, const TextSelection(baseOffset: 0, extentOffset: 5));
      expect(paragraph3.selections.first, const TextSelection(baseOffset: 0, extentOffset: 5));

      await gesture.up();
    },
  );
}
