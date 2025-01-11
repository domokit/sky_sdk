// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/semantics.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_api_samples/widgets/raw_menu_anchor/raw_menu_anchor.1.dart' as example;
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
  // These tests were copied from the Material version of this sample and
  // adapted to work with a menu without submenus.
  testWidgets('Can open and close menu on desktop', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ContextMenuApp());

    await tester.tapAt(const Offset(100, 200), buttons: kSecondaryButton);
    await tester.pump();

    expect(collectOverlays().first, equals(const Rect.fromLTRB(100.0, 200.0, 280.0, 435.0)));
    expect(find.text('Cut'), findsOneWidget);

    // Make sure tapping in a different place causes the menu to move.
    await tester.tapAt(const Offset(200, 100), buttons: kSecondaryButton);
    await tester.pump();

    expect(collectOverlays().first, equals(const Rect.fromLTRB(200.0, 100.0, 380.0, 335.0)));
    expect(find.text('Cut'), findsOneWidget);

    // Tap outside the menu to close.
    await tester.tapAt(Offset.zero);
    await tester.pump();

    expect(find.text('Cut'), findsNothing);
  }, variant: TargetPlatformVariant.desktop());

  testWidgets('Can open and close menu on mobile', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ContextMenuApp());

    await tester.longPressAt(const Offset(100, 200));
    await tester.pump();

    expect(collectOverlays().first, equals(const Rect.fromLTRB(100.0, 200.0, 280.0, 435.0)));
    expect(find.text('Cut'), findsOneWidget);

    // Make sure tapping in a different place causes the menu to move.
    await tester.longPressAt(const Offset(200, 100));
    await tester.pump();

    expect(collectOverlays().first, equals(const Rect.fromLTRB(200.0, 100.0, 380.0, 335.0)));
    expect(find.text('Cut'), findsOneWidget);

    // Tap outside the menu to close.
    await tester.tapAt(Offset.zero);
    await tester.pump();

    expect(find.text('Cut'), findsNothing);
  }, variant: TargetPlatformVariant.mobile());

  testWidgets('Can traverse menu', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ContextMenuApp());

    await tester.tapAt(const Offset(100, 200), buttons: kSecondaryButton);
    await tester.pump();

    expect(primaryFocus!.debugLabel, equals('MenuItemButton(Text("Undo"))'));
    expect(tester.getSemantics(find.text('Format')).hasFlag(SemanticsFlag.isExpanded), isFalse);
    expect(find.text('Undo'), findsOneWidget);
    expect(find.text('Redo'), findsOneWidget);
    expect(find.text('Cut'), findsOneWidget);
    expect(find.text('Copy'), findsOneWidget);
    expect(find.text('Paste'), findsOneWidget);
    expect(find.text('Format'), findsOneWidget);
    expect(find.text('Bold'), findsNothing);
    expect(find.text('Italic'), findsNothing);
    expect(find.text('Underline'), findsNothing);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();

    expect(primaryFocus!.debugLabel, equals('MenuItemButton(Text("Bold"))'));
    expect(tester.getSemantics(find.text('Format')).hasFlag(SemanticsFlag.isExpanded), isTrue);
    expect(find.text('Undo'), findsOneWidget);
    expect(find.text('Redo'), findsOneWidget);
    expect(find.text('Cut'), findsOneWidget);
    expect(find.text('Copy'), findsOneWidget);
    expect(find.text('Paste'), findsOneWidget);
    expect(find.text('Format'), findsOneWidget);
    expect(find.text('Bold'), findsOneWidget);
    expect(find.text('Italic'), findsOneWidget);
    expect(find.text('Underline'), findsOneWidget);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pump();

    expect(primaryFocus!.debugLabel, equals('MenuItemButton(Text("Format"))'));
    expect(tester.getSemantics(find.text('Format')).hasFlag(SemanticsFlag.isExpanded), isFalse);
    expect(find.text('Bold'), findsNothing);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();

    expect(primaryFocus!.debugLabel, equals('MenuItemButton(Text("Undo"))'));

    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump();
    await tester.pump();

    expect(find.text('Selected: Undo'), findsOneWidget);
  }, variant: TargetPlatformVariant.desktop());

  testWidgets('Platform Brightness does not affect menu appearance', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(platformBrightness: Brightness.dark),
        child: example.ContextMenuApp(),
      ),
    );

    await tester.tapAt(const Offset(100, 200), buttons: kSecondaryButton);
    await tester.pump();

    expect(
      findMenuPanelDescendent<Container>(tester).decoration,
      RawMenuPanel.lightSurfaceDecoration,
    );
  }, variant: TargetPlatformVariant.desktop());

  testWidgets(
    'Browser context menu is disabled',
    (WidgetTester tester) async {
      await tester.pumpWidget(const example.ContextMenuApp());

      await tester.tapAt(const Offset(100, 200), buttons: kSecondaryButton);
      await tester.pump();

      if (kIsWeb) {
        expect(BrowserContextMenu.enabled, isFalse);
      }
    },
    skip: !kIsWeb, // [intended] Browser context menu is only enabled on the web
  );
}
