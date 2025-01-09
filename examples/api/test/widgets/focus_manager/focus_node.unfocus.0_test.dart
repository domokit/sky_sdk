// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/widgets/focus_manager/focus_node.unfocus.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Unfocusing with UnfocusDisposition.scope gives the focus to the parent scope', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.UnfocusExampleApp());

    // Focuses the first text field.
    await tester.tap(find.byType(TextField).first);
    await tester.pump();

    // Changes the focus to the unfocus button.
    for (int i = 0; i < 6; i++) {
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pump();
    }

    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump();

    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.pump();

    // After pressing tab once, the focus is on the first text field.
    final EditableText firstEditableText = tester.firstWidget(find.byType(EditableText));
    expect(firstEditableText.focusNode.hasFocus, true);
  });

  testWidgets(
    'Unfocusing with UnfocusDisposition.previouslyFocusedChild gives the focus the previously focused child',
    (WidgetTester tester) async {
      await tester.pumpWidget(const example.UnfocusExampleApp());

      // Focuses the first text field.
      await tester.tap(find.byType(TextField).first);
      await tester.pump();

      // Changes the focus to the second radio button.
      for (int i = 0; i < 5; i++) {
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.pump();
      }

      await tester.sendKeyEvent(LogicalKeyboardKey.space);
      await tester.pumpAndSettle();

      // Changes the focus to the unfocus button.
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pump();

      await tester.sendKeyEvent(LogicalKeyboardKey.enter);
      await tester.pumpAndSettle();

      // After pressing tab twice, the focus is on the first text field.
      for (int i = 0; i < 2; i++) {
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.pump();
      }

      final EditableText firstEditableText = tester.firstWidget(find.byType(EditableText));
      expect(firstEditableText.focusNode.hasFocus, true);
    },
  );
}
