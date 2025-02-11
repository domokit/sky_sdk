// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/widgets/raw_menu_anchor/raw_menu_anchor.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

String get catLabel => example.Animal.cat.label;
String get kittenLabel => example.Animal.kitten.label;
String get felisCatusLabel => example.Animal.felisCatus.label;
String get dogLabel => example.Animal.dog.label;

void main() {
  testWidgets('Menu opens and closes', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RawMenuAnchorApp());
    final Finder button = find.text('Select One');

    // Open the menu.
    await tester.tap(button);
    await tester.pump();

    expect(find.text(catLabel), findsOneWidget);
    expect(find.text(kittenLabel), findsOneWidget);
    expect(find.text(felisCatusLabel), findsOneWidget);
    expect(find.text(dogLabel), findsOneWidget);
    expect(
      tester.getRect(find.ancestor(of: find.text(catLabel), matching: find.byType(TapRegion))),
      rectMoreOrLessEquals(const Rect.fromLTRB(447.2, 328.0, 626.3, 532.0), epsilon: 0.1),
    );

    // Close the menu.
    await tester.tap(button);
    await tester.pump();

    expect(find.text(catLabel), findsNothing);
    expect(find.text(kittenLabel), findsNothing);
    expect(find.text(felisCatusLabel), findsNothing);
    expect(find.text(dogLabel), findsNothing);
  });

  testWidgets('Can traverse menu', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RawMenuAnchorApp());

    await tester.tap(find.text('Select One'));
    await tester.pump();

    expect(find.text(catLabel), findsOneWidget);

    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);

    expect(primaryFocus?.debugLabel, contains(kittenLabel));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown); // Felis catus

    expect(primaryFocus?.debugLabel, contains(felisCatusLabel));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown); // Dog

    expect(primaryFocus?.debugLabel, contains(dogLabel));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp); // Felis catus
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp); // Kitten
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp); // Cat
    await tester.sendKeyEvent(LogicalKeyboardKey.enter); // Select Cat
    await tester.pump();
    await tester.pump();

    expect(find.text(catLabel), findsOneWidget);
    expect(find.text(felisCatusLabel), findsNothing);
  });

  testWidgets('Check appears next to selected item', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RawMenuAnchorApp());

    await tester.tap(find.text('Select One'));
    await tester.pump();

    // Select Kitten
    await tester.tap(find.text(kittenLabel));
    await tester.pump();
    await tester.pump();

    expect(find.text(catLabel), findsNothing);

    await tester.tap(find.text(kittenLabel));
    await tester.pump();

    expect(
      tester.getRect(find.ancestor(of: find.text(catLabel), matching: find.byType(TapRegion))),
      rectMoreOrLessEquals(const Rect.fromLTRB(447.2, 328.0, 626.3, 532.0), epsilon: 0.1),
    );

    expect(
      tester.widget(find.widgetWithIcon(MenuItemButton, Icons.check)),
      equals(tester.widget(find.widgetWithText(MenuItemButton, kittenLabel))),
    );
  });
}
