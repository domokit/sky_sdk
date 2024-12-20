// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/focus_scope/focus.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Adds children through button', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FocusExampleApp());
    expect(find.byIcon(Icons.add), findsOneWidget);
    expect(find.text('CHILD 0'), findsOneWidget);

    for (int i = 1; i <= 20; i += 1) {
      await tester.tap(find.byIcon(Icons.add));
      await tester.pumpAndSettle();
      expect(find.text('CHILD $i'), findsOneWidget);
      expect(find.textContaining('CHILD '), findsNWidgets(i + 1));
    }
  });

  testWidgets('Inserts focus nodes', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FocusExampleApp());
    expect(find.byIcon(Icons.add), findsOneWidget);

    for (int i = 0; i <= 10; i += 1) {
      expect(find.text('CHILD $i'), findsOneWidget);
      final ActionChip chip = tester.widget<ActionChip>(
        find.ancestor(of: find.text('CHILD $i'), matching: find.byType(ActionChip)),
      );
      expect(chip.focusNode, isNotNull);
      expect(chip.focusNode!.hasPrimaryFocus, isTrue);
      expect(chip.focusNode!.debugLabel, 'Child $i');

      await tester.tap(find.byIcon(Icons.add));
      await tester.pumpAndSettle();
    }
  });
}
