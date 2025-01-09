// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/text_form_field/text_form_field.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  group('TextFormFieldExample2 Widget Tests', () {
    testWidgets('Input validation handles empty, incorrect, and short usernames', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(const example.TextFormFieldExampleApp());
      final Finder textFormField = find.byType(TextFormField);
      final Finder saveButton = find.byType(TextButton);

      await tester.enterText(textFormField, '');
      await tester.pump();
      await tester.tap(saveButton);
      await tester.pump();
      expect(find.text('This field is required'), findsOneWidget);

      await tester.enterText(textFormField, 'jo hn');
      await tester.tap(saveButton);
      await tester.pump();
      expect(find.text('Username must not contain any spaces'), findsOneWidget);

      await tester.enterText(textFormField, 'jo');
      await tester.tap(saveButton);
      await tester.pump();
      expect(find.text('Username should be at least 3 characters long'), findsOneWidget);

      await tester.enterText(textFormField, '1jo');
      await tester.tap(saveButton);
      await tester.pump();
      expect(find.text('Username must not start with a number'), findsOneWidget);
    });

    testWidgets('Async validation feedback is handled correctly', (WidgetTester tester) async {
      await tester.pumpWidget(const example.TextFormFieldExampleApp());
      final Finder textFormField = find.byType(TextFormField);
      final Finder saveButton = find.byType(TextButton);

      // Simulate entering a username already taken.
      await tester.enterText(textFormField, 'jack');
      await tester.pump();
      await tester.tap(saveButton);
      await tester.pump();
      expect(find.text('Username jack is already taken'), findsNothing);
      await tester.pump(example.kFakeHttpRequestDuration);
      expect(find.text('Username jack is already taken'), findsOneWidget);

      await tester.enterText(textFormField, 'alex');
      await tester.pump();
      await tester.tap(saveButton);
      await tester.pump();
      expect(find.text('Username alex is already taken'), findsNothing);
      await tester.pump(example.kFakeHttpRequestDuration);
      expect(find.text('Username alex is already taken'), findsOneWidget);

      await tester.enterText(textFormField, 'jack');
      await tester.pump();
      await tester.tap(saveButton);
      await tester.pump();
      expect(find.text('Username jack is already taken'), findsNothing);
      await tester.pump(example.kFakeHttpRequestDuration);
      expect(find.text('Username jack is already taken'), findsOneWidget);
    });

    testWidgets('Loading spinner displays correctly when saving', (WidgetTester tester) async {
      await tester.pumpWidget(const example.TextFormFieldExampleApp());
      final Finder textFormField = find.byType(TextFormField);
      final Finder saveButton = find.byType(TextButton);
      await tester.enterText(textFormField, 'alexander');
      await tester.pump();
      await tester.tap(saveButton);
      await tester.pump();
      expect(find.byType(CircularProgressIndicator), findsOneWidget);
      await tester.pump(example.kFakeHttpRequestDuration);
      expect(find.byType(CircularProgressIndicator), findsNothing);
    });
  });
}
