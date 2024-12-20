// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/material/menu_anchor/menu_anchor.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can open menu', (WidgetTester tester) async {
    Finder findMenu() {
      return find
          .ancestor(of: find.text(example.MenuEntry.about.label), matching: find.byType(FocusScope))
          .first;
    }

    await tester.pumpWidget(const example.ContextMenuApp());

    await tester.tapAt(const Offset(100, 200), buttons: kSecondaryButton);
    await tester.pumpAndSettle();
    expect(tester.getRect(findMenu()).left, equals(100.0));
    expect(tester.getRect(findMenu()).top, equals(200.0));
    expect(tester.getRect(findMenu()).right, closeTo(389.8, 0.1));
    expect(tester.getRect(findMenu()).bottom, equals(360.0));

    // Make sure tapping in a different place causes the menu to move.
    await tester.tapAt(const Offset(200, 100), buttons: kSecondaryButton);
    await tester.pump();

    expect(tester.getRect(findMenu()).left, equals(200.0));
    expect(tester.getRect(findMenu()).top, equals(100.0));
    expect(tester.getRect(findMenu()).right, closeTo(489.8, 0.1));
    expect(tester.getRect(findMenu()).bottom, equals(260.0));

    expect(find.text(example.MenuEntry.about.label), findsOneWidget);
    expect(find.text(example.MenuEntry.showMessage.label), findsOneWidget);
    expect(find.text(example.MenuEntry.hideMessage.label), findsNothing);
    expect(find.text('Background Color'), findsOneWidget);
    expect(find.text(example.MenuEntry.colorRed.label), findsNothing);
    expect(find.text(example.MenuEntry.colorGreen.label), findsNothing);
    expect(find.text(example.MenuEntry.colorBlue.label), findsNothing);
    expect(find.text(example.ContextMenuApp.kMessage), findsNothing);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();

    expect(find.text('Background Color'), findsOneWidget);

    // Focusing the background color item with the keyboard caused the submenu
    // to open. Tapping it should cause it to close.
    await tester.tap(find.text('Background Color'));
    await tester.pump();
    await tester.pumpAndSettle();

    expect(find.text(example.MenuEntry.colorRed.label), findsNothing);
    expect(find.text(example.MenuEntry.colorGreen.label), findsNothing);
    expect(find.text(example.MenuEntry.colorBlue.label), findsNothing);

    await tester.tap(find.text('Background Color'));
    await tester.pump();
    await tester.pumpAndSettle();

    expect(find.text(example.MenuEntry.colorRed.label), findsOneWidget);
    expect(find.text(example.MenuEntry.colorGreen.label), findsOneWidget);
    expect(find.text(example.MenuEntry.colorBlue.label), findsOneWidget);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pumpAndSettle();

    expect(find.text(example.ContextMenuApp.kMessage), findsOneWidget);
    expect(find.text('Last Selected: ${example.MenuEntry.showMessage.label}'), findsOneWidget);
  });

  testWidgets('Shortcuts work', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ContextMenuApp());

    // Open the menu so we can look for state changes reflected in the menu.
    await tester.tapAt(const Offset(100, 200), buttons: kSecondaryButton);
    await tester.pump();

    expect(find.text(example.MenuEntry.showMessage.label), findsOneWidget);
    expect(find.text(example.MenuEntry.hideMessage.label), findsNothing);
    expect(find.text(example.ContextMenuApp.kMessage), findsNothing);

    await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyS);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
    await tester.pump();
    // Need to pump twice because of the one frame delay in the notification to
    // update the overlay entry.
    await tester.pump();

    expect(find.text(example.MenuEntry.showMessage.label), findsNothing);
    expect(find.text(example.MenuEntry.hideMessage.label), findsOneWidget);
    expect(find.text(example.ContextMenuApp.kMessage), findsOneWidget);

    await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyS);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
    await tester.pump();
    await tester.pump();

    expect(find.text(example.MenuEntry.showMessage.label), findsOneWidget);
    expect(find.text(example.MenuEntry.hideMessage.label), findsNothing);
    expect(find.text(example.ContextMenuApp.kMessage), findsNothing);

    await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyR);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
    await tester.pump();

    expect(find.text('Last Selected: ${example.MenuEntry.colorRed.label}'), findsOneWidget);

    await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyG);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
    await tester.pump();

    expect(find.text('Last Selected: ${example.MenuEntry.colorGreen.label}'), findsOneWidget);

    await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyB);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
    await tester.pump();

    expect(find.text('Last Selected: ${example.MenuEntry.colorBlue.label}'), findsOneWidget);
  });
}
