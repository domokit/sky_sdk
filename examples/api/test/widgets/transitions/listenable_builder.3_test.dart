// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/transitions/listenable_builder.3.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Tapping FAB adds to values', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ListenableBuilderExample());

    final Finder listContent = find.byWidgetPredicate(
      (Widget widget) => widget is example.ListBody,
    );

    expect(find.text('Current values:'), findsOneWidget);
    expect(find.byIcon(Icons.add), findsOneWidget);
    expect((tester.widget(listContent) as example.ListBody).listNotifier.values.isEmpty, isTrue);

    await tester.tap(find.byType(FloatingActionButton).first);
    await tester.pumpAndSettle();
    expect((tester.widget(listContent) as example.ListBody).listNotifier.values.isEmpty, isFalse);
    expect((tester.widget(listContent) as example.ListBody).listNotifier.values, <int>[1464685455]);
    expect(find.text('1464685455'), findsOneWidget);
  });
}
