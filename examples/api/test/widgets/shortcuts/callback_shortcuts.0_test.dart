// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter_api_samples/widgets/shortcuts/callback_shortcuts.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Verify correct labels are displayed', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CallbackShortcutsApp());

    expect(find.text('CallbackShortcuts Sample'), findsOneWidget);
    expect(find.text('Press the up arrow key to add to the counter'), findsOneWidget);
    expect(find.text('Press the down arrow key to subtract from the counter'), findsOneWidget);
    expect(find.text('count: 0'), findsOneWidget);
  });

  testWidgets('Up and down arrow press updates counter', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CallbackShortcutsApp());

    int counter = 0;

    while (counter < 10) {
      expect(find.text('count: $counter'), findsOneWidget);

      // Increment the counter.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      await tester.pump();

      counter++;
    }

    while (counter >= 0) {
      expect(find.text('count: $counter'), findsOneWidget);

      // Decrement the counter.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pump();

      counter--;
    }
  });
}
