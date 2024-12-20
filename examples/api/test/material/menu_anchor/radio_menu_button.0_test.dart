// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/material/menu_anchor/radio_menu_button.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can open menu', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MenuApp());

    await tester.tap(find.byType(TextButton));
    await tester.pump();
    await tester.pump();

    expect(find.text('Red Background'), findsOneWidget);
    expect(find.text('Green Background'), findsOneWidget);
    expect(find.text('Blue Background'), findsOneWidget);
    expect(find.byType(Radio<Color>), findsNWidgets(3));
    expect(tester.widget<Container>(find.byType(Container)).color, equals(Colors.red));

    await tester.tap(find.text('Green Background'));
    await tester.pumpAndSettle();

    expect(tester.widget<Container>(find.byType(Container)).color, equals(Colors.green));
  });

  testWidgets('Shortcuts work', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MenuApp());

    // Open the menu so we can watch state changes resulting from the shortcuts
    // firing.
    await tester.tap(find.byType(TextButton));
    await tester.pump();

    expect(find.text('Red Background'), findsOneWidget);
    expect(find.text('Green Background'), findsOneWidget);
    expect(find.text('Blue Background'), findsOneWidget);
    expect(find.byType(Radio<Color>), findsNWidgets(3));
    expect(tester.widget<Container>(find.byType(Container)).color, equals(Colors.red));

    await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyG);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
    await tester.pump();
    // Need to pump twice because of the one frame delay in the notification to
    // update the overlay entry.
    await tester.pump();

    expect(
      tester
          .widget<Radio<Color>>(
            find.descendant(
              of: find.byType(RadioMenuButton<Color>).at(0),
              matching: find.byType(Radio<Color>),
            ),
          )
          .groupValue,
      equals(Colors.green),
    );
    expect(tester.widget<Container>(find.byType(Container)).color, equals(Colors.green));

    await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyR);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
    await tester.pump();
    await tester.pump();

    expect(
      tester
          .widget<Radio<Color>>(
            find.descendant(
              of: find.byType(RadioMenuButton<Color>).at(1),
              matching: find.byType(Radio<Color>),
            ),
          )
          .groupValue,
      equals(Colors.red),
    );
    expect(tester.widget<Container>(find.byType(Container)).color, equals(Colors.red));

    await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyB);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
    await tester.pump();
    await tester.pump();

    expect(
      tester
          .widget<Radio<Color>>(
            find.descendant(
              of: find.byType(RadioMenuButton<Color>).at(2),
              matching: find.byType(Radio<Color>),
            ),
          )
          .groupValue,
      equals(Colors.blue),
    );
    expect(tester.widget<Container>(find.byType(Container)).color, equals(Colors.blue));
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
