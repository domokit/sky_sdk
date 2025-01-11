// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_api_samples/widgets/raw_menu_anchor/raw_menu_anchor.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

T findMenuPanelDescendent<T extends Widget>(WidgetTester tester) {
  return tester.firstWidget<T>(
    find.descendant(of: find.byType(RawMenuPanel), matching: find.byType(T)),
  );
}

List<Rect> collectOverlays({bool clipped = true}) {
  final List<Rect> menuRects = <Rect>[];
  final Finder finder = find.byType(RawMenuPanel);
  for (final Element candidate in finder.evaluate().toList()) {
    final RenderBox box = candidate.renderObject! as RenderBox;
    final Offset topLeft = box.localToGlobal(box.size.topLeft(Offset.zero));
    menuRects.add(topLeft & box.size);
  }
  return menuRects;
}

void main() {
  testWidgets('Menu opens and displays expected items', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SimpleMenuApp());
    await tester.tap(find.text('Edit'));
    await tester.pump();

    expect(find.text('Cut'), findsOneWidget);
    expect(find.text('Copy'), findsOneWidget);
    expect(find.text('Paste'), findsOneWidget);
    expect(
      collectOverlays().first,
      rectMoreOrLessEquals(const Rect.fromLTRB(359.8, 335.0, 479.8, 442.0), epsilon: 0.1),
    );

    // Tap outside the menu to close.
    await tester.tapAt(Offset.zero);
    await tester.pumpAndSettle();

    expect(find.text('Cut'), findsNothing);
  });

  testWidgets('Activating a menu item closes the menu and displays selected item text', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.SimpleMenuApp());
    await tester.tap(find.text('Edit'));
    await tester.pump();

    await tester.tap(find.text('Cut'));
    await tester.pump();
    await tester.pump();

    expect(find.text('Cut'), findsNothing);
    expect(find.text('Selected: Cut'), findsOneWidget);
  });

  testWidgets('Menu can take focus', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SimpleMenuApp());

    // Tap the 'Edit' button and trigger a frame.
    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();

    expect(primaryFocus?.debugLabel, equals('MenuItemButton(Text("Paste"))'));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);

    expect(primaryFocus?.debugLabel, equals('MenuItemButton(Text("Copy"))'));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);

    expect(primaryFocus?.debugLabel, equals('MenuItemButton(Text("Cut"))'));

    await tester.sendKeyEvent(LogicalKeyboardKey.escape);
    await tester.pump();

    expect(find.text('Cut'), findsNothing);
  });

  testWidgets('Platform Brightness does not affect menu appearance', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(platformBrightness: Brightness.dark),
        child: example.SimpleMenuApp(),
      ),
    );

    await tester.tap(find.text('Edit'));
    await tester.pump();

    expect(
      findMenuPanelDescendent<Container>(tester).decoration,
      RawMenuPanel.lightSurfaceDecoration,
    );
  });
}
