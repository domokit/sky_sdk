// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/material/menu_anchor/menu_bar.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can open menu', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MenuBarApp());

    final Finder menuBarFinder = find.byType(MenuBar);
    final MenuBar menuBar = tester.widget<MenuBar>(menuBarFinder);
    expect(menuBar.children, isNotEmpty);
    expect(menuBar.children.length, equals(1));

    final Finder menuButtonFinder = find.byType(SubmenuButton).first;
    await tester.tap(menuButtonFinder);
    await tester.pumpAndSettle();

    expect(find.text('About'), findsOneWidget);
    expect(find.text('Show Message'), findsOneWidget);
    expect(find.text('Reset Message'), findsOneWidget);
    expect(find.text('Background Color'), findsOneWidget);
    expect(find.text('Red Background'), findsNothing);
    expect(find.text('Green Background'), findsNothing);
    expect(find.text('Blue Background'), findsNothing);
    expect(find.text(example.MenuBarApp.kMessage), findsNothing);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pumpAndSettle();

    expect(find.text('About'), findsOneWidget);
    expect(find.text('Show Message'), findsOneWidget);
    expect(find.text('Reset Message'), findsOneWidget);
    expect(find.text('Background Color'), findsOneWidget);
    expect(find.text('Red Background'), findsOneWidget);
    expect(find.text('Green Background'), findsOneWidget);
    expect(find.text('Blue Background'), findsOneWidget);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pumpAndSettle();

    expect(find.text(example.MenuBarApp.kMessage), findsOneWidget);
    expect(find.text('Last Selected: Show Message'), findsOneWidget);
  });

  testWidgets('Shortcuts work', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MenuBarApp());

    expect(find.text(example.MenuBarApp.kMessage), findsNothing);

    await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyS);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
    await tester.pump();

    expect(find.text(example.MenuBarApp.kMessage), findsOneWidget);
    await tester.sendKeyEvent(LogicalKeyboardKey.escape);
    await tester.pump();

    expect(find.text(example.MenuBarApp.kMessage), findsNothing);
    expect(find.text('Last Selected: Reset Message'), findsOneWidget);

    await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyR);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
    await tester.pump();

    expect(find.text('Last Selected: Red Background'), findsOneWidget);

    await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyG);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
    await tester.pump();

    expect(find.text('Last Selected: Green Background'), findsOneWidget);

    await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyB);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
    await tester.pump();

    expect(find.text('Last Selected: Blue Background'), findsOneWidget);
  });

  testWidgets('MenuBar is wrapped in a SafeArea', (WidgetTester tester) async {
    const double safeAreaPadding = 100.0;
    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(padding: EdgeInsets.symmetric(vertical: safeAreaPadding)),
        child: example.MenuBarApp(),
      ),
    );

    expect(tester.getTopLeft(find.byType(MenuBar)), const Offset(0.0, safeAreaPadding));
  });
}
