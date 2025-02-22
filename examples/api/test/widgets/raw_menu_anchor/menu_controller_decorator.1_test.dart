// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/raw_menu_anchor/menu_controller_decorator.1.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

String getPanelText(int i, AnimationStatus status) => 'Panel $i:\n${status.name}';

Future<TestGesture> hoverOver(WidgetTester tester, Offset location) async {
  final TestGesture gesture = await tester.createGesture(kind: ui.PointerDeviceKind.mouse);
  addTearDown(gesture.removePointer);
  await gesture.moveTo(location);
  return gesture;
}

void main() {
  testWidgets('Root menu opens when anchor button is pressed', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NestedMenuControllerDecoratorApp());

    final Finder button = find.byType(FilledButton);
    await tester.tap(button);
    await tester.pump();

    final Finder panel =
        find
            .ancestor(of: find.textContaining('Submenu 0'), matching: find.byType(ExcludeFocus))
            .first;

    expect(
      tester.getRect(panel),
      rectMoreOrLessEquals(const ui.Rect.fromLTRB(347.8, 324.0, 524.8, 324.0), epsilon: 0.1),
    );
    await tester.pump(const Duration(milliseconds: 100));

    expect(
      tester.getRect(panel),
      rectMoreOrLessEquals(const ui.Rect.fromLTRB(347.8, 324.0, 524.8, 499.7), epsilon: 0.1),
    );

    await tester.pump(const Duration(milliseconds: 101));
    expect(
      tester.getRect(panel),
      rectMoreOrLessEquals(const ui.Rect.fromLTRB(347.8, 324.0, 524.8, 516.0), epsilon: 0.1),
    );

    expect(find.textContaining('Submenu'), findsNWidgets(4));
  });

  testWidgets('Hover traversal opens one submenu at a time', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NestedMenuControllerDecoratorApp());

    // Open root menu
    await tester.tap(find.byType(FilledButton));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));

    final Finder menuItem = find.widgetWithText(MenuItemButton, 'Submenu 0').first;
    await hoverOver(tester, tester.getCenter(menuItem));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 201));

    for (int i = 1; i < 4; i++) {
      final Finder menuItem = find.widgetWithText(MenuItemButton, 'Submenu $i').first;
      await hoverOver(tester, tester.getCenter(menuItem));
      await tester.pump();

      expect(find.text(getPanelText(i - 1, AnimationStatus.reverse)), findsOneWidget);
      expect(find.text(getPanelText(i, AnimationStatus.forward)), findsOneWidget);

      await tester.pump(const Duration(milliseconds: 201));

      expect(find.text(getPanelText(i - 1, AnimationStatus.dismissed)), findsNothing);
      expect(find.text(getPanelText(i, AnimationStatus.completed)), findsOneWidget);
    }
  });

  testWidgets('Submenu opens at expected rate', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NestedMenuControllerDecoratorApp());

    // Open root menu
    await tester.tap(find.byType(FilledButton));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 201));

    // Start hovering over submenu item
    final Finder menuItem = find.widgetWithText(MenuItemButton, 'Submenu 0').first;
    await hoverOver(tester, tester.getCenter(menuItem));
    await tester.pump();

    final Finder panel =
        find
            .ancestor(of: find.textContaining('Panel 0'), matching: find.byType(ExcludeFocus))
            .first;

    // 25% through, 70% height
    await tester.pump(const Duration(milliseconds: 50));
    expect(tester.getSize(panel).height, moreOrLessEquals(0.7 * 120, epsilon: 1));

    // 50% through, 91.5% height
    await tester.pump(const Duration(milliseconds: 50));
    expect(tester.getSize(panel).height, moreOrLessEquals(0.91 * 120, epsilon: 1));

    // 100% through, full height
    await tester.pump(const Duration(milliseconds: 101));
    expect(tester.getSize(panel).height, moreOrLessEquals(120, epsilon: 1));

    // Close submenu
    final Finder menuItem1 = find.widgetWithText(MenuItemButton, 'Submenu 1').first;
    await hoverOver(tester, tester.getCenter(menuItem1));
    await tester.pump();

    // 25% through, ~98% height
    await tester.pump(const Duration(milliseconds: 50));
    expect(tester.getSize(panel).height, moreOrLessEquals(0.98 * 120, epsilon: 1));

    // 50% through, ~91.5% height
    await tester.pump(const Duration(milliseconds: 50));
    expect(tester.getSize(panel).height, moreOrLessEquals(0.91 * 120, epsilon: 1));
  });

  testWidgets('Outside tap closes all menus', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NestedMenuControllerDecoratorApp());

    // Open root menu and submenu
    await tester.tap(find.byType(FilledButton));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 201));

    await hoverOver(
      tester,
      tester.getCenter(find.widgetWithText(MenuItemButton, 'Submenu 0').first),
    );
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 201));

    expect(find.textContaining(AnimationStatus.completed.name), findsOneWidget);
    expect(find.textContaining(AnimationStatus.reverse.name), findsNothing);

    // Tap outside
    await tester.tapAt(const Offset(10, 10));
    await tester.pump();

    expect(find.textContaining(AnimationStatus.completed.name), findsNothing);
    expect(find.textContaining(AnimationStatus.reverse.name), findsOneWidget);
  });
}
