// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:a11y_assessments/use_cases/text_field_password.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'test_utils.dart';

void main() {
  testWidgets('text field password can run', (WidgetTester tester) async {
    await pumpsUseCase(tester, TextFieldPasswordUseCase());
    expect(find.byType(TextField), findsExactly(2));

    // Test the enabled password
    {
      final Finder finder = find.byKey(const Key('enabled password'));
      await tester.tap(finder);
      await tester.pumpAndSettle();
      await tester.enterText(finder, 'abc');
      await tester.pumpAndSettle();
      expect(find.text('abc'), findsOneWidget);
    }

    // Test the disabled password
    {
      final Finder finder = find.byKey(const Key('disabled password'));
      final TextField passwordField = tester.widget<TextField>(finder);
      expect(passwordField.enabled, isFalse);
    }
  });

  testWidgets('text field passwords do not have hint text', (WidgetTester tester) async {
    await pumpsUseCase(tester, TextFieldPasswordUseCase());
    expect(find.byType(TextField), findsExactly(2));

    // Test the enabled password
    {
      final Finder finder = find.byKey(const Key('enabled password'));
      final TextField textField = tester.widget<TextField>(finder);
      expect(textField.decoration?.hintText, isNull);
    }

    // Test the disabled password
    {
      final Finder finder = find.byKey(const Key('disabled password'));
      final TextField textField = tester.widget<TextField>(finder);
      expect(textField.decoration?.hintText, isNull);
    }
  });

  testWidgets('text field password demo page has one h1 tag', (WidgetTester tester) async {
    await pumpsUseCase(tester, TextFieldPasswordUseCase());
    final Finder findHeadingLevelOnes = find.bySemanticsLabel('TextField password Demo');
    await tester.pumpAndSettle();
    expect(findHeadingLevelOnes, findsOne);
  });
}
