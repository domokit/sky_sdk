// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/restoration_properties/restorable_value.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Increments answer on OutlinedButton tap', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RestorableValueExampleApp());

    // Verify that the initial answer value in the example equals 42.
    expect(find.text('42'), findsOneWidget);

    // Tap the button to increment the answer value by 1.
    await tester.tap(find.byType(OutlinedButton));
    await tester.pump();

    // Verify that the answer value increased by 1.
    expect(find.text('43'), findsOneWidget);
  });

  testWidgets('Restores answer value after restart', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: RootRestorationScope(
          restorationId: 'root',
          child: example.RestorableValueExample(restorationId: 'child'),
        ),
      ),
    );

    // The initial answer value in the example equals 42.
    expect(find.text('42'), findsOneWidget);

    // Tap the button 10 times to change the answer value.
    for (int i = 0; i < 10; i++) {
      await tester.tap(find.byType(OutlinedButton));
      await tester.pump();
    }

    // Verify that the answer value increased by 10.
    expect(find.text('52'), findsOneWidget);

    // Simulate restoring the state of the widget tree after the application
    // is restarted.
    await tester.restartAndRestore();

    // Verify that the answer value is restored correctly.
    expect(find.text('52'), findsOneWidget);
  });
}
