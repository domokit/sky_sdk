// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/editable_text/editable_text.on_changed.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Verify correct labels are displayed', (WidgetTester tester) async {
    await tester.pumpWidget(const example.OnChangedExampleApp());

    expect(find.text('What number comes next in the sequence?'), findsOneWidget);
    expect(find.text('1, 1, 2, 3, 5, 8...?'), findsOneWidget);
  });

  testWidgets('Does not show dialog when answer is not correct', (WidgetTester tester) async {
    await tester.pumpWidget(const example.OnChangedExampleApp());

    await tester.enterText(find.byType(TextField), '33');
    await tester.pumpAndSettle();

    expect(find.byType(AlertDialog), findsNothing);
  });

  testWidgets('Shows dialog when answer is correct', (WidgetTester tester) async {
    await tester.pumpWidget(const example.OnChangedExampleApp());

    await tester.enterText(find.byType(TextField), '13');
    await tester.pumpAndSettle();

    expect(find.byType(AlertDialog), findsOneWidget);
    expect(find.text('That is correct!'), findsOneWidget);
    expect(find.text('13 is the right answer.'), findsOneWidget);
    expect(find.text('OK'), findsOneWidget);
  });

  testWidgets('Closes dialog on OK button tap', (WidgetTester tester) async {
    await tester.pumpWidget(const example.OnChangedExampleApp());

    await tester.enterText(find.byType(TextField), '13');
    await tester.pumpAndSettle();

    expect(find.byType(AlertDialog), findsOneWidget);

    await tester.tap(find.ancestor(of: find.text('OK'), matching: find.byType(TextButton)));
    await tester.pumpAndSettle();

    expect(find.byType(AlertDialog), findsNothing);
  });
}
