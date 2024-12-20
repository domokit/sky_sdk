// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/form/form.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Form Smoke Test', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FormExampleApp());
    expect(find.widgetWithText(AppBar, 'Form Sample'), findsOneWidget);

    final Finder textField = find.byType(TextField);
    final Finder button = find.byType(ElevatedButton);
    final TextField textFieldWidget = tester.widget<TextField>(textField);

    expect(textField, findsOneWidget);
    expect(button, findsOneWidget);

    expect(textFieldWidget.controller?.text, '');
    await tester.tap(button);
    await tester.pumpAndSettle();
    expect(find.text('Please enter some text'), findsOneWidget);

    await tester.enterText(textField, 'Hello World');
    expect(textFieldWidget.controller?.text, 'Hello World');
    await tester.tap(button);
    await tester.pumpAndSettle();
    expect(find.text('Please enter some text'), findsNothing);
  });
}
