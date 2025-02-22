// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/raw_menu_anchor/menu_controller_decorator.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Menu opens and closes', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MenuControllerDecoratorApp());

    // Open the menu.
    await tester.tap(find.text('Open'));
    await tester.pump();

    final Finder message =
        find
            .ancestor(of: find.textContaining('ANIMATION STATUS:'), matching: find.byType(Material))
            .first;

    expect(find.text('Open'), findsNothing);
    expect(find.text('Close'), findsOneWidget);
    expect(message, findsOneWidget);
    expect(tester.getRect(message), const Rect.fromLTRB(400.0, 328.0, 400.0, 328.0));

    await tester.pump(const Duration(milliseconds: 500));

    expect(
      tester.getRect(message),
      rectMoreOrLessEquals(const Rect.fromLTRB(323.7, 326.2, 476.3, 533.2), epsilon: 0.1),
    );

    await tester.pump(const Duration(milliseconds: 1100));

    expect(
      tester.getRect(message),
      rectMoreOrLessEquals(const Rect.fromLTRB(325.0, 328.0, 475.0, 528.0), epsilon: 0.1),
    );

    // Close the menu.
    await tester.tap(find.text('Close'));
    await tester.pump();

    await tester.pump(const Duration(milliseconds: 150));

    expect(
      tester.getRect(message),
      rectMoreOrLessEquals(const Rect.fromLTRB(382.4, 345.9, 417.6, 356.9), epsilon: 0.1),
    );

    await tester.pump(const Duration(milliseconds: 920));

    expect(find.widgetWithText(Material, 'ANIMATION STATUS:'), findsNothing);
    expect(find.text('Close'), findsNothing);
    expect(find.text('Open'), findsOneWidget);
  });

  testWidgets('Panel text contains the animation status', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MenuControllerDecoratorApp());

    // Open the menu.
    await tester.tap(find.text('Open'));
    await tester.pump();

    expect(find.textContaining('forward'), findsOneWidget);

    await tester.pump(const Duration(milliseconds: 1100));

    expect(find.textContaining('completed'), findsOneWidget);

    await tester.tapAt(Offset.zero);
    await tester.pump();

    expect(find.textContaining('reverse'), findsOneWidget);

    await tester.pump(const Duration(milliseconds: 920));

    // The panel text should be removed when the animation is dismissed.
    expect(find.textContaining('reverse'), findsNothing);
  });
}
