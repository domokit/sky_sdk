// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/shared_app_data/shared_app_data.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Verify correct labels are displayed', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SharedAppDataExampleApp());

    expect(find.text('SharedAppData Sample'), findsOneWidget);
    expect(find.text('foo: initial'), findsOneWidget);
    expect(find.text('bar: initial'), findsOneWidget);
    expect(find.text('change foo'), findsOneWidget);
    expect(find.text('change bar'), findsOneWidget);
  });

  testWidgets('foo value can be updated', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SharedAppDataExampleApp());

    int counter = 0;

    while (counter < 10) {
      counter++;

      await tester.tap(
        find.ancestor(of: find.text('change foo'), matching: find.byType(ElevatedButton)),
      );
      await tester.pump();

      expect(find.text('foo: FOO $counter'), findsOneWidget);
    }
  });

  testWidgets('bar value can be updated', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SharedAppDataExampleApp());

    int counter = 0;

    while (counter < 10) {
      counter++;

      await tester.tap(
        find.ancestor(of: find.text('change bar'), matching: find.byType(ElevatedButton)),
      );
      await tester.pump();

      expect(find.text('bar: BAR $counter'), findsOneWidget);
    }
  });

  testWidgets('foo and bar values update independently of one another', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.SharedAppDataExampleApp());

    int fooCounter = 0;
    int barCounter = 0;

    for (int i = 0; i < 20; i++) {
      if (i.isEven) {
        fooCounter++;
      } else {
        barCounter++;
      }

      await tester.tap(
        find.ancestor(
          of: i.isEven ? find.text('change foo') : find.text('change bar'),
          matching: find.byType(ElevatedButton),
        ),
      );
      await tester.pump();

      expect(find.text('foo: ${fooCounter == 0 ? 'initial' : 'FOO $fooCounter'}'), findsOneWidget);
      expect(find.text('bar: ${barCounter == 0 ? 'initial' : 'BAR $barCounter'}'), findsOneWidget);
    }
  });
}
