// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/transitions/listenable_builder.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Increments counter', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ListenableBuilderExample());
    expect(find.text('ListenableBuilder Example'), findsOneWidget);
    expect(find.text('Current counter value:'), findsOneWidget);
    expect(find.byIcon(Icons.add), findsOneWidget);
    expect(
      find.descendant(of: find.byType(FloatingActionButton), matching: find.byIcon(Icons.add)),
      findsOneWidget,
    );

    expect(find.text('0'), findsOneWidget);
    expect(find.text('1'), findsNothing);
    await tester.tap(find.byIcon(Icons.add));
    await tester.pump();
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsOneWidget);

    for (int i = 0; i < 4; i++) {
      await tester.tap(find.byIcon(Icons.add));
    }
    await tester.pump();
    expect(find.text('5'), findsOneWidget);
  });
}
