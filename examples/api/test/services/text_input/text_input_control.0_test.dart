// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/services/text_input/text_input_control.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Enter text using the VKB', (WidgetTester tester) async {
    await tester.pumpWidget(const example.TextInputControlExampleApp());
    await tester.pumpAndSettle();

    await tester.tap(
      find.descendant(
        of: find.byType(example.MyVirtualKeyboard),
        matching: find.widgetWithText(ElevatedButton, 'A'),
      ),
    );
    await tester.pumpAndSettle();
    expect(find.widgetWithText(TextField, 'A'), findsOneWidget);

    await tester.tap(
      find.descendant(
        of: find.byType(example.MyVirtualKeyboard),
        matching: find.widgetWithText(ElevatedButton, 'B'),
      ),
    );
    await tester.pumpAndSettle();
    expect(find.widgetWithText(TextField, 'AB'), findsOneWidget);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pumpAndSettle();

    await tester.tap(
      find.descendant(
        of: find.byType(example.MyVirtualKeyboard),
        matching: find.widgetWithText(ElevatedButton, 'C'),
      ),
    );
    await tester.pumpAndSettle();
    expect(find.widgetWithText(TextField, 'ACB'), findsOneWidget);
  });
}
