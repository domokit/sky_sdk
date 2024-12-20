// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/pop_scope/pop_scope.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

import '../navigator_utils.dart';

void main() {
  testWidgets('Can choose to stay on page', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NavigatorPopHandlerApp());

    expect(find.text('Page One'), findsOneWidget);

    await tester.tap(find.text('Next page'));
    await tester.pumpAndSettle();
    expect(find.text('Page One'), findsNothing);
    expect(find.text('Page Two'), findsOneWidget);

    await simulateSystemBack();
    await tester.pumpAndSettle();
    expect(find.text('Page One'), findsNothing);
    expect(find.text('Page Two'), findsOneWidget);
    expect(find.text('Are you sure?'), findsOneWidget);

    await tester.tap(find.text('Never mind'));
    await tester.pumpAndSettle();
    expect(find.text('Page One'), findsNothing);
    expect(find.text('Page Two'), findsOneWidget);
  });

  testWidgets('Can choose to go back with pop result', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NavigatorPopHandlerApp());

    expect(find.text('Page One'), findsOneWidget);
    expect(find.text('Page Two'), findsNothing);

    await tester.tap(find.text('Next page'));
    await tester.pumpAndSettle();
    expect(find.text('Page One'), findsNothing);
    expect(find.text('Page Two'), findsOneWidget);

    await tester.enterText(find.byType(TextFormField).first, 'John');
    await tester.pumpAndSettle();
    await tester.enterText(find.byType(TextFormField).last, 'Apple');
    await tester.pumpAndSettle();

    await tester.tap(find.text('Go back'));
    await tester.pumpAndSettle();
    expect(find.text('Page One'), findsNothing);
    expect(find.text('Page Two'), findsOneWidget);
    expect(find.text('Are you sure?'), findsOneWidget);

    await tester.tap(find.text('Leave'));
    await tester.pumpAndSettle();
    expect(find.text('Page One'), findsOneWidget);
    expect(find.text('Page Two'), findsNothing);
    expect(find.text('Are you sure?'), findsNothing);
    expect(find.text('Hello John, whose favorite food is Apple.'), findsOneWidget);
  });
}
