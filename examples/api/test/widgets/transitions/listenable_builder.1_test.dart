// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/transitions/listenable_builder.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Tapping FAB increments counter', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ListenableBuilderExample());

    String getCount() =>
        (tester.widget(
                  find.descendant(
                    of: find.byType(ListenableBuilder).last,
                    matching: find.byType(Text),
                  ),
                )
                as Text)
            .data!;

    expect(find.text('Current counter value:'), findsOneWidget);
    expect(find.text('0'), findsOneWidget);
    expect(find.byIcon(Icons.add), findsOneWidget);
    expect(getCount(), equals('0'));

    await tester.tap(find.byType(FloatingActionButton).first);
    await tester.pumpAndSettle();
    expect(getCount(), equals('1'));
  });
}
