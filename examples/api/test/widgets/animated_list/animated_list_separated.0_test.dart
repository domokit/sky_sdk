// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/animated_list/animated_list_separated.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Items can be selected, added, and removed from AnimatedList.separated', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.AnimatedListSeparatedSample());

    expect(find.text('Item 0'), findsOneWidget);
    expect(find.text('Separator 0'), findsOneWidget);
    expect(find.text('Item 1'), findsOneWidget);
    expect(find.text('Separator 1'), findsOneWidget);
    expect(find.text('Item 2'), findsOneWidget);

    // Add an item at the end of the list
    await tester.tap(find.byIcon(Icons.add_circle));
    await tester.pumpAndSettle();
    expect(find.text('Separator 2'), findsOneWidget);
    expect(find.text('Item 3'), findsOneWidget);

    // Select Item 1.
    await tester.tap(find.text('Item 1'));
    await tester.pumpAndSettle();

    // Add item at the top of the list
    await tester.tap(find.byIcon(Icons.add_circle));
    await tester.pumpAndSettle();
    expect(find.text('Item 4'), findsOneWidget);
    // Contrary to the behavior for insertion at other places,
    // the Separator for the last item of the list will be added
    // before that item instead of after it.
    expect(find.text('Separator 4'), findsOneWidget);

    // Remove selected item.
    await tester.tap(find.byIcon(Icons.remove_circle));

    // Item animation is not completed.
    await tester.pump();
    expect(find.text('Item 1'), findsOneWidget);
    expect(find.text('Separator 1'), findsNothing);
    expect(find.text('Removing separator'), findsOneWidget);

    // When the animation completes, Item 1 disappears.
    await tester.pumpAndSettle();
    expect(find.text('Item 1'), findsNothing);
    expect(find.text('Separator 1'), findsNothing);
    expect(find.text('Removing separator'), findsNothing);
  });
}
