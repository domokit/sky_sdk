// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/scaffold/scaffold.of.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Verify correct labels are displayed', (WidgetTester tester) async {
    await tester.pumpWidget(const example.OfExampleApp());

    expect(find.text('Scaffold.of Example'), findsOneWidget);
    expect(find.text('SHOW BOTTOM SHEET'), findsOneWidget);
  });

  testWidgets('Bottom sheet can be shown', (WidgetTester tester) async {
    await tester.pumpWidget(const example.OfExampleApp());

    expect(find.text('BottomSheet'), findsNothing);
    expect(find.widgetWithText(ElevatedButton, 'Close BottomSheet'), findsNothing);

    // Tap the button to show the bottom sheet.
    await tester.tap(find.widgetWithText(ElevatedButton, 'SHOW BOTTOM SHEET'));
    await tester.pumpAndSettle();

    expect(find.text('BottomSheet'), findsOneWidget);
    expect(find.widgetWithText(ElevatedButton, 'Close BottomSheet'), findsOneWidget);
  });

  testWidgets('Bottom sheet can be closed', (WidgetTester tester) async {
    await tester.pumpWidget(const example.OfExampleApp());

    expect(find.text('BottomSheet'), findsNothing);

    // Tap the button to show the bottom sheet.
    await tester.tap(find.widgetWithText(ElevatedButton, 'SHOW BOTTOM SHEET'));
    await tester.pumpAndSettle();

    expect(find.text('BottomSheet'), findsOneWidget);

    // Tap the button to close the bottom sheet.
    await tester.tap(find.widgetWithText(ElevatedButton, 'Close BottomSheet'));
    await tester.pumpAndSettle();

    expect(find.text('BottomSheet'), findsNothing);
  });
}
