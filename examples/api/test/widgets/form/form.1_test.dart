// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/form/form.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can go back when form is clean', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FormApp());

    expect(find.text('Are you sure?'), findsNothing);

    await tester.tap(find.text('Go back'));
    await tester.pumpAndSettle();

    expect(find.text('Are you sure?'), findsNothing);
  });

  testWidgets('Cannot go back when form is dirty', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FormApp());

    expect(find.text('Are you sure?'), findsNothing);

    await tester.enterText(find.byType(TextFormField), 'some new text');

    await tester.tap(find.text('Go back'));
    await tester.pumpAndSettle();

    expect(find.text('Are you sure?'), findsOneWidget);
  });
}
