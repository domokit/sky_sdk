// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_api_samples/widgets/raw_menu_anchor/raw_menu_anchor.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

Future<TestGesture> hoverOver(WidgetTester tester, Offset location) async {
  final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
  addTearDown(gesture.removePointer);
  await gesture.moveTo(location);
  await tester.pumpAndSettle();
  return gesture;
}

void main() {
  testWidgets('Initializes with correct number of menu items in expected position', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.RawMenuAnchorGroupApp());
    expect(find.byType(RawMenuAnchorGroup).evaluate().length, 1);
    for (final example.MenuItem item in example.menuItems) {
      expect(find.text(item.label), findsOneWidget);
    }
    expect(find.byType(RawMenuAnchor).evaluate().length, 4);
    expect(
      tester.getRect(find.byType(RawMenuAnchorGroup).first),
      const Rect.fromLTRB(233.0, 284.0, 567.0, 316.0),
    );
  });
  testWidgets('Menu can be traversed', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RawMenuAnchorGroupApp());

    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.pump();

    expect(primaryFocus?.debugLabel, contains('File'));
    expect(find.text('New'), findsNothing);

    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump();
    await tester.pump();

    expect(primaryFocus?.debugLabel, contains('File'));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);

    expect(primaryFocus?.debugLabel, contains('New'));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();

    expect(primaryFocus?.debugLabel, contains('New'));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pump();

    expect(primaryFocus?.debugLabel, contains('New'));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();

    expect(primaryFocus?.debugLabel, contains('New'));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);

    expect(primaryFocus?.debugLabel, contains('Share'));

    await tester.sendKeyEvent(LogicalKeyboardKey.escape);
    await tester.pump();

    expect(primaryFocus?.debugLabel, contains('File'));
    expect(find.text('Share'), findsNothing);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();

    expect(primaryFocus?.debugLabel, contains('Tools'));

    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump();
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);

    expect(primaryFocus?.debugLabel, contains('Spelling'));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);

    expect(primaryFocus?.debugLabel, contains('Grammar'));

    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump();
    await tester.pump();
    expect(find.text('Selected: Grammar'), findsOneWidget);
  });

  testWidgets('Hover traversal opens submenus when the root menu is open', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.RawMenuAnchorGroupApp());

    await hoverOver(tester, tester.getCenter(find.text('File')));
    await tester.pump();

    expect(find.text('New'), findsNothing);

    await tester.tap(find.text('File'));
    await tester.pump();
    await tester.pump();

    expect(find.text('New'), findsOneWidget);

    await hoverOver(tester, tester.getCenter(find.text('Tools')));
    await tester.pump();

    expect(find.text('Spelling'), findsOneWidget);

    await hoverOver(tester, Offset.zero);
    await tester.pump();

    expect(find.text('Spelling'), findsOneWidget);
    expect(
      WidgetsBinding.instance.focusManager.primaryFocus?.debugLabel,
      'MenuItemButton(Text("Tools"))',
    );

    await hoverOver(tester, tester.getCenter(find.text('Tools')));
    await tester.tap(find.text('Tools'));
    await tester.pump();
    await tester.pump();

    expect(find.text('Spelling'), findsNothing);
    expect(
      WidgetsBinding.instance.focusManager.primaryFocus?.debugLabel,
      'MenuItemButton(Text("Tools"))',
    );

    await hoverOver(tester, Offset.zero);
    await tester.pump();

    expect(
      WidgetsBinding.instance.focusManager.primaryFocus?.debugLabel,
      isNot('MenuItemButton(Text("Tools"))'),
    );
  });
}
