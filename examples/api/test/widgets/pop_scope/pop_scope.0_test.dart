// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_api_samples/widgets/pop_scope/pop_scope.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

import '../navigator_utils.dart';

void main() {
  testWidgets('Can choose to stay on page', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NavigatorPopHandlerApp());

    expect(find.text('Page One'), findsOneWidget);
    expect(find.text('Page Two'), findsNothing);
    expect(find.text('Are you sure?'), findsNothing);

    await tester.tap(find.text('Next page'));
    await tester.pumpAndSettle();
    expect(find.text('Page One'), findsNothing);
    expect(find.text('Page Two'), findsOneWidget);
    expect(find.text('Are you sure?'), findsNothing);

    await simulateSystemBack();
    await tester.pumpAndSettle();
    expect(find.text('Page One'), findsNothing);
    expect(find.text('Page Two'), findsOneWidget);
    expect(find.text('Are you sure?'), findsOneWidget);

    await tester.tap(find.text('Nevermind'));
    await tester.pumpAndSettle();
    expect(find.text('Page One'), findsNothing);
    expect(find.text('Page Two'), findsOneWidget);
    expect(find.text('Are you sure?'), findsNothing);
  });

  testWidgets('Can choose to go back', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NavigatorPopHandlerApp());

    expect(find.text('Page One'), findsOneWidget);
    expect(find.text('Page Two'), findsNothing);
    expect(find.text('Are you sure?'), findsNothing);

    await tester.tap(find.text('Next page'));
    await tester.pumpAndSettle();
    expect(find.text('Page One'), findsNothing);
    expect(find.text('Page Two'), findsOneWidget);
    expect(find.text('Are you sure?'), findsNothing);

    await simulateSystemBack();
    await tester.pumpAndSettle();
    expect(find.text('Page One'), findsNothing);
    expect(find.text('Page Two'), findsOneWidget);
    expect(find.text('Are you sure?'), findsOneWidget);

    await tester.tap(find.text('Leave'));
    await tester.pumpAndSettle();
    expect(find.text('Page One'), findsOneWidget);
    expect(find.text('Page Two'), findsNothing);
    expect(find.text('Are you sure?'), findsNothing);
  });
}
