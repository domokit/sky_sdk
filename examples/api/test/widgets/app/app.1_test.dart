// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_api_samples/widgets/app/app.1.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

import '../../widgets/navigator_utils.dart';

void main() {
  testWidgets('Can navigate through all of the routes with system back gestures', (WidgetTester tester) async {
    await tester.pumpWidget(
      const example.WidgetsAppExample(),
    );

    expect(find.text('Home Page'), findsOneWidget);
    expect(find.text('Leaf Page'), findsNothing);
    expect(find.text('Nested Home Page'), findsNothing);
    expect(find.text('Nested Leaf Page'), findsNothing);

    await tester.tap(find.text('Go to leaf page'));
    await tester.pumpAndSettle();

    expect(find.text('Home Page'), findsNothing);
    expect(find.text('Leaf Page'), findsOneWidget);
    expect(find.text('Nested Home Page'), findsNothing);
    expect(find.text('Nested Leaf Page'), findsNothing);

    await tester.tap(find.text('Go to nested Navigator page'));
    await tester.pumpAndSettle();

    expect(find.text('Home Page'), findsNothing);
    expect(find.text('Leaf Page'), findsNothing);
    expect(find.text('Nested Home Page'), findsOneWidget);
    expect(find.text('Nested Leaf Page'), findsNothing);

    await tester.tap(find.text('Go to nested leaf page'));
    await tester.pumpAndSettle();

    expect(find.text('Home Page'), findsNothing);
    expect(find.text('Leaf Page'), findsNothing);
    expect(find.text('Nested Home Page'), findsNothing);
    expect(find.text('Nested Leaf Page'), findsOneWidget);

    await simulateSystemBack();
    await tester.pumpAndSettle();

    expect(find.text('Home Page'), findsNothing);
    expect(find.text('Leaf Page'), findsNothing);
    expect(find.text('Nested Home Page'), findsOneWidget);
    expect(find.text('Nested Leaf Page'), findsNothing);

    await simulateSystemBack();
    await tester.pumpAndSettle();

    expect(find.text('Home Page'), findsNothing);
    expect(find.text('Leaf Page'), findsOneWidget);
    expect(find.text('Nested Home Page'), findsNothing);
    expect(find.text('Nested Leaf Page'), findsNothing);

    await simulateSystemBack();
    await tester.pumpAndSettle();

    expect(find.text('Home Page'), findsOneWidget);
    expect(find.text('Leaf Page'), findsNothing);
    expect(find.text('Nested Home Page'), findsNothing);
    expect(find.text('Nested Leaf Page'), findsNothing);
  });
}
