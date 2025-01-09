// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/actions/actions.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  Color? getSaveButtonColor(WidgetTester tester) {
    final ButtonStyleButton button = tester.widget<ButtonStyleButton>(
      find.descendant(
        of: find.byType(example.SaveButton),
        matching: find.byWidgetPredicate((Widget widget) => widget is TextButton),
      ),
    );

    return button.style?.foregroundColor?.resolve(<WidgetState>{});
  }

  testWidgets('Increments and decrements value', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ActionsExampleApp());

    int value = 0;

    while (value < 10) {
      expect(find.text('Value: $value'), findsOneWidget);

      // Increment the value.
      await tester.tap(find.byIcon(Icons.exposure_plus_1));
      await tester.pump();

      value++;
    }

    while (value >= 0) {
      expect(find.text('Value: $value'), findsOneWidget);

      // Decrement the value.
      await tester.tap(find.byIcon(Icons.exposure_minus_1));
      await tester.pump();

      value--;
    }
  });

  testWidgets('SaveButton indicates dirty status', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ActionsExampleApp());

    // Verify that initial color is green, as the value is not marked as dirty.
    Color? saveButtonColor = getSaveButtonColor(tester);
    expect(saveButtonColor, equals(Colors.green));

    // Decrement the value, which marks it as dirty.
    await tester.tap(find.byIcon(Icons.exposure_minus_1));
    await tester.pump();
    expect(find.text('Value: -1'), findsOneWidget);

    // Verify that the color is red, as the value is marked as dirty.
    saveButtonColor = getSaveButtonColor(tester);
    expect(saveButtonColor, equals(Colors.red));

    // Increment the value.
    await tester.tap(find.byIcon(Icons.exposure_plus_1));
    await tester.pump();
    expect(find.text('Value: 0'), findsOneWidget);

    // Verify that the color is red, as the value is still marked as dirty.
    saveButtonColor = getSaveButtonColor(tester);
    expect(saveButtonColor, equals(Colors.red));
  });

  testWidgets('SaveButton tap resets dirty status and adds log', (WidgetTester tester) async {
    final List<String?> log = <String?>[];

    final DebugPrintCallback originalDebugPrint = debugPrint;
    debugPrint = (String? message, {int? wrapWidth}) {
      log.add(message);
    };

    await tester.pumpWidget(const example.ActionsExampleApp());

    // Verify that value is not marked as dirty.
    Color? saveButtonColor = getSaveButtonColor(tester);
    expect(saveButtonColor, equals(Colors.green));
    expect(
      find.descendant(of: find.byType(example.SaveButton), matching: find.text('0')),
      findsOneWidget,
    );

    // Decrement the value, which marks it as dirty.
    await tester.tap(find.byIcon(Icons.exposure_minus_1));
    await tester.pump();
    expect(find.text('Value: -1'), findsOneWidget);

    // Verify that value is marked as dirty.
    saveButtonColor = getSaveButtonColor(tester);
    expect(saveButtonColor, equals(Colors.red));
    expect(
      find.descendant(of: find.byType(example.SaveButton), matching: find.text('0')),
      findsOneWidget,
    );

    // Tap SaveButton to reset dirty status.
    await tester.tap(find.byType(example.SaveButton));
    await tester.pump();

    // Verify log record.
    expect(log.length, equals(1));
    expect(log.last, equals('Saved Data: -1'));

    // Verify that value is no more marked as dirty.
    saveButtonColor = getSaveButtonColor(tester);
    expect(saveButtonColor, equals(Colors.green));
    expect(
      find.descendant(of: find.byType(example.SaveButton), matching: find.text('-1')),
      findsOneWidget,
    );

    debugPrint = originalDebugPrint;
  });
}
