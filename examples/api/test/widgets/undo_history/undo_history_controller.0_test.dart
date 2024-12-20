// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/undo_history/undo_history_controller.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('The undo history controller should undo and redo the history changes', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.UndoHistoryControllerExampleApp());

    // Equals to UndoHistoryState._kThrottleDuration.
    const Duration kThrottleDuration = Duration(milliseconds: 500);

    expect(find.byType(TextField), findsOne);
    expect(find.widgetWithText(TextButton, 'Undo'), findsOne);
    expect(find.widgetWithText(TextButton, 'Redo'), findsOne);

    await tester.enterText(find.byType(TextField), '1st change');
    await tester.pump(kThrottleDuration);
    expect(find.text('1st change'), findsOne);

    await tester.enterText(find.byType(TextField), '2nd change');
    await tester.pump(kThrottleDuration);
    expect(find.text('2nd change'), findsOne);

    await tester.enterText(find.byType(TextField), '3rd change');
    await tester.pump(kThrottleDuration);
    expect(find.text('3rd change'), findsOne);

    await tester.tap(find.text('Undo'));
    await tester.pump();
    expect(find.text('2nd change'), findsOne);

    await tester.tap(find.text('Undo'));
    await tester.pump();
    expect(find.text('1st change'), findsOne);

    await tester.tap(find.text('Redo'));
    await tester.pump();
    expect(find.text('2nd change'), findsOne);

    await tester.tap(find.text('Redo'));
    await tester.pump();
    expect(find.text('3rd change'), findsOne);
  });
}
