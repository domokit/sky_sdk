// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_api_samples/widgets/page/page_can_pop.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

import '../navigator_utils.dart';

void main() {
  testWidgets('Can choose to stay on page', (WidgetTester tester) async {
    await tester.pumpWidget(const example.PageApiExampleApp());

    expect(find.text('Home'), findsOneWidget);

    await tester.tap(find.text('Go to details'));
    await tester.pumpAndSettle();
    expect(find.text('Home'), findsNothing);
    expect(find.text('Details'), findsOneWidget);

    await simulateSystemBack();
    await tester.pumpAndSettle();
    expect(find.text('Are you sure?'), findsOneWidget);

    await tester.tap(find.text('Cancel'));
    await tester.pumpAndSettle();
    expect(find.text('Home'), findsNothing);
    expect(find.text('Details'), findsOneWidget);
  });

  testWidgets('Can choose to go back', (WidgetTester tester) async {
    await tester.pumpWidget(const example.PageApiExampleApp());

    expect(find.text('Home'), findsOneWidget);

    await tester.tap(find.text('Go to details'));
    await tester.pumpAndSettle();
    expect(find.text('Home'), findsNothing);
    expect(find.text('Details'), findsOneWidget);

    await simulateSystemBack();
    await tester.pumpAndSettle();
    expect(find.text('Are you sure?'), findsOneWidget);

    await tester.tap(find.text('Confirm'));
    await tester.pumpAndSettle();
    expect(find.text('Details'), findsNothing);
    expect(find.text('Home'), findsOneWidget);
  });
}
