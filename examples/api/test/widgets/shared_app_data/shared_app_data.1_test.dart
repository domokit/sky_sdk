// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/shared_app_data/shared_app_data.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  example.SharedObject getSharedObject(WidgetTester tester) {
    final BuildContext context = tester.element(find.byType(example.CustomWidget));
    return example.SharedObject.of(context);
  }

  testWidgets('Verify correct labels are displayed', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SharedAppDataExampleApp());

    final example.SharedObject sharedObject = getSharedObject(tester);

    expect(find.text('SharedAppData Sample'), findsOneWidget);
    expect(find.text('Replace $sharedObject'), findsOneWidget);
  });

  testWidgets('Button tap resets SharedObject', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SharedAppDataExampleApp());

    for (int i = 0; i < 10; i++) {
      final example.SharedObject sharedObject = getSharedObject(tester);

      final Finder buttonFinder = find.ancestor(
        of: find.text('Replace $sharedObject'),
        matching: find.byType(ElevatedButton),
      );

      expect(buttonFinder, findsOneWidget);

      await tester.tap(buttonFinder);
      await tester.pump();
    }
  });
}
