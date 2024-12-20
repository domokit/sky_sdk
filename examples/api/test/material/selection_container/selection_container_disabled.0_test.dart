// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_api_samples/material/selection_container/selection_container_disabled.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('A SelectionContainer.disabled should disable selections', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.SelectionContainerDisabledExampleApp());

    expect(find.widgetWithText(AppBar, 'SelectionContainer.disabled Sample'), findsOne);

    final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
      find.descendant(of: find.text('Selectable text').first, matching: find.byType(RichText)),
    );
    final Rect paragraph1Rect = tester.getRect(find.text('Selectable text').first);
    final TestGesture gesture = await tester.startGesture(
      paragraph1Rect.centerLeft,
      kind: PointerDeviceKind.mouse,
    );
    addTearDown(gesture.removePointer);
    await tester.pump();

    await gesture.moveTo(paragraph1Rect.center);
    await tester.pump();
    expect(paragraph1.selections.first, const TextSelection(baseOffset: 0, extentOffset: 7));

    final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
      find.descendant(of: find.text('Non-selectable text'), matching: find.byType(RichText)),
    );
    final Rect paragraph2Rect = tester.getRect(find.text('Non-selectable text'));
    await gesture.moveTo(paragraph2Rect.center);
    // Should select the rest of paragraph 1.
    expect(paragraph1.selections.first, const TextSelection(baseOffset: 0, extentOffset: 15));
    // paragraph2 is in a disabled container.
    expect(paragraph2.selections, isEmpty);

    final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
      find.descendant(of: find.text('Selectable text').last, matching: find.byType(RichText)),
    );
    final Rect paragraph3Rect = tester.getRect(find.text('Selectable text').last);
    await gesture.moveTo(paragraph3Rect.center);
    expect(paragraph1.selections.first, const TextSelection(baseOffset: 0, extentOffset: 15));
    expect(paragraph2.selections, isEmpty);
    expect(paragraph3.selections.first, const TextSelection(baseOffset: 0, extentOffset: 7));

    await gesture.up();
  });
}
