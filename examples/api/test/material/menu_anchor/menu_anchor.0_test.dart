// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/material/menu_anchor/menu_anchor.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can open menu', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MenuApp());

    await tester.tap(find.byType(TextButton));
    await tester.pump();

    expect(find.text(example.MenuEntry.about.label), findsOneWidget);
    expect(find.text(example.MenuEntry.showMessage.label), findsOneWidget);
    expect(find.text(example.MenuEntry.hideMessage.label), findsNothing);
    expect(find.text('Background Color'), findsOneWidget);
    expect(find.text(example.MenuEntry.colorRed.label), findsNothing);
    expect(find.text(example.MenuEntry.colorGreen.label), findsNothing);
    expect(find.text(example.MenuEntry.colorBlue.label), findsNothing);
    expect(find.text(example.MenuApp.kMessage), findsNothing);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();

    expect(find.text('Background Color'), findsOneWidget);
    expect(find.text(example.MenuEntry.colorRed.label), findsOneWidget);
    expect(find.text(example.MenuEntry.colorGreen.label), findsOneWidget);
    expect(find.text(example.MenuEntry.colorBlue.label), findsOneWidget);

    await tester.tap(find.text('Background Color'));
    await tester.pump();

    expect(find.text(example.MenuEntry.colorRed.label), findsNothing);
    expect(find.text(example.MenuEntry.colorGreen.label), findsNothing);
    expect(find.text(example.MenuEntry.colorBlue.label), findsNothing);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pumpAndSettle();

    expect(find.text(example.MenuApp.kMessage), findsOneWidget);
    expect(find.text('Last Selected: ${example.MenuEntry.showMessage.label}'), findsOneWidget);
  });

  testWidgets('Shortcuts work', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MenuApp());

    // Open the menu so we can watch state changes resulting from the shortcuts
    // firing.
    await tester.tap(find.byType(TextButton));
    await tester.pump();

    expect(find.text(example.MenuEntry.showMessage.label), findsOneWidget);
    expect(find.text(example.MenuEntry.hideMessage.label), findsNothing);
    expect(find.text(example.MenuApp.kMessage), findsNothing);

    await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyS);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
    await tester.pump();
    // Need to pump twice because of the one frame delay in the notification to
    // update the overlay entry.
    await tester.pump();

    expect(find.text(example.MenuEntry.showMessage.label), findsNothing);
    expect(find.text(example.MenuEntry.hideMessage.label), findsOneWidget);
    expect(find.text(example.MenuApp.kMessage), findsOneWidget);

    await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyS);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
    await tester.pump();
    await tester.pump();

    expect(find.text(example.MenuEntry.showMessage.label), findsOneWidget);
    expect(find.text(example.MenuEntry.hideMessage.label), findsNothing);
    expect(find.text(example.MenuApp.kMessage), findsNothing);

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

  testWidgets('MenuAnchor is wrapped in a SafeArea', (WidgetTester tester) async {
    const double safeAreaPadding = 100.0;
    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(padding: EdgeInsets.symmetric(vertical: safeAreaPadding)),
        child: example.MenuApp(),
      ),
    );

    expect(tester.getTopLeft(find.byType(MenuAnchor)), const Offset(0.0, safeAreaPadding));
  });
}
