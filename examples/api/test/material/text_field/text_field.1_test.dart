// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/text_field/text_field.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Dialog shows submitted TextField value', (WidgetTester tester) async {
    // This example is also used to illustrate special character counting.
    const String sampleText = 'Some sample text 👨‍👩‍👦';
    await tester.pumpWidget(const example.TextFieldExampleApp());

    expect(find.byType(TextField), findsOneWidget);
    expect(find.byType(AlertDialog), findsNothing);
    expect(find.text('Thanks!'), findsNothing);
    expect(find.widgetWithText(TextButton, 'OK'), findsNothing);
    expect(
      find.text('You typed "$sampleText", which has the length ${sampleText.length}.'),
      findsNothing,
    );

    await tester.enterText(find.byType(TextField), sampleText);
    await tester.testTextInput.receiveAction(TextInputAction.done);
    await tester.pumpAndSettle();
    expect(find.byType(AlertDialog), findsOneWidget);
    expect(find.text('Thanks!'), findsOneWidget);
    expect(find.widgetWithText(TextButton, 'OK'), findsOneWidget);
    expect(
      find.text('You typed "$sampleText", which has length ${sampleText.characters.length}.'),
      findsOneWidget,
    );
  });
}
