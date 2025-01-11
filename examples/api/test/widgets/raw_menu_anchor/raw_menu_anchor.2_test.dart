// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart' show Icons, MenuItemButton;
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_api_samples/widgets/raw_menu_anchor/raw_menu_anchor.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

final Finder opacityFinder = find.descendant(
  of: find.byType(example.ShiftingMenuOverlay),
  matching: find.byType(Opacity),
);

IconData? iconByMenuItemLabel(String label, WidgetTester tester) {
  final Finder finder = find.widgetWithText(MenuItemButton, label);
  final Icon? icon = tester.widget<MenuItemButton>(finder).trailingIcon as Icon?;
  return icon?.icon;
}

void main() {
  testWidgets('Menu animates open', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MenuOverlayBuilderApp());

    // Open the menu.
    await tester.tap(find.text('Select One'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));

    expect(find.byType(example.ShiftingMenuOverlay), findsOneWidget);
    expect(
      tester.getRect(find.byType(example.ShiftingMenuOverlay)),
      rectMoreOrLessEquals(const Rect.fromLTRB(449.2, 272.0, 674.2, 460.0), epsilon: 0.1),
    );

    Opacity opacity = tester.widget<Opacity>(opacityFinder);

    expect(opacity.opacity, moreOrLessEquals(0.2525, epsilon: 0.001));

    await tester.pump(const Duration(milliseconds: 50));

    opacity = tester.widget<Opacity>(opacityFinder);

    expect(opacity.opacity, moreOrLessEquals(0.5632, epsilon: 0.001));

    await tester.pump(const Duration(milliseconds: 50));

    opacity = tester.widget<Opacity>(opacityFinder);

    expect(opacity.opacity, moreOrLessEquals(0.8564, epsilon: 0.001));

    await tester.pumpAndSettle();

    // The menu should be fully open.
    opacity = tester.widget<Opacity>(opacityFinder);

    expect(opacity.opacity, moreOrLessEquals(1.0, epsilon: 0.001));
    expect(find.text('Cat'), findsOneWidget);
    expect(find.text('Kitten'), findsOneWidget);
    expect(find.text('Felis catus'), findsOneWidget);
    expect(find.text('Dog'), findsOneWidget);
  });

  testWidgets('Can traverse menu', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MenuOverlayBuilderApp());

    await tester.tap(find.text('Select One'));
    await tester.pumpAndSettle();

    expect(find.text('Cat'), findsOneWidget);

    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);

    expect(primaryFocus?.debugLabel, equals('MenuItemButton(Text("Kitten"))'));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown); // Felis catus

    expect(primaryFocus?.debugLabel, equals('MenuItemButton(Text("Felis catus"))'));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown); // Dog

    expect(primaryFocus?.debugLabel, equals('MenuItemButton(Text("Dog"))'));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp); // Felis catus
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp); // Kitten
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp); // Cat
    await tester.sendKeyEvent(LogicalKeyboardKey.enter); // Select Cat
    await tester.pump();
    await tester.pump();

    expect(find.text('Cat'), findsOneWidget);
    expect(find.text('Felis catus'), findsNothing);
  });

  testWidgets('Menu position shifts over selected item', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MenuOverlayBuilderApp());

    await tester.tap(find.text('Select One'));
    await tester.pumpAndSettle();

    expect(
      tester.getRect(find.byType(example.ShiftingMenuOverlay)),
      rectMoreOrLessEquals(const Rect.fromLTRB(449.2, 272.0, 674.2, 460.0), epsilon: 0.1),
    );

    await tester.tap(find.text('Kitten'));
    await tester.pumpAndSettle();

    expect(find.text('Cat'), findsNothing);

    await tester.tap(find.text('Kitten'));
    await tester.pumpAndSettle();

    expect(
      tester.getRect(find.byType(example.ShiftingMenuOverlay)),
      rectMoreOrLessEquals(const Rect.fromLTRB(449.2, 228.0, 674.2, 416.0), epsilon: 0.1),
    );

    expect(iconByMenuItemLabel('Cat', tester), isNull);
    expect(iconByMenuItemLabel('Kitten', tester), Icons.check);
  });
}
