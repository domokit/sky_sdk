// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/value_listenable_builder/value_listenable_builder.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Tapping FAB increments counter', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ValueListenableBuilderExampleApp());

    String getCount() {
      return (tester.widget(
                find.descendant(of: find.byType(example.CountDisplay), matching: find.byType(Text)),
              )
              as Text)
          .data!;
    }

    expect(find.text('You have pushed the button this many times:'), findsOneWidget);
    expect(find.text('0'), findsOneWidget);
    expect(find.byIcon(Icons.plus_one), findsOneWidget);
    expect(getCount(), equals('0'));

    await tester.tap(find.byType(FloatingActionButton).first);
    await tester.pumpAndSettle();
    expect(getCount(), equals('1'));
  });
}
