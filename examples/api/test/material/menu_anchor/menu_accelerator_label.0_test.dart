// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/material/menu_anchor/menu_accelerator_label.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can open menu', (WidgetTester tester) async {
    Finder findMenu(String label) {
      return find
          .ancestor(of: find.text(label, findRichText: true), matching: find.byType(FocusScope))
          .first;
    }

    await tester.pumpWidget(const example.MenuAcceleratorApp());

    await tester.sendKeyDownEvent(LogicalKeyboardKey.altLeft);
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.keyF, character: 'f');
    await tester.pumpAndSettle();
    await tester.pump();

    expect(find.text('About', findRichText: true), findsOneWidget);
    expect(tester.getRect(findMenu('About')).left, equals(4.0));
    expect(tester.getRect(findMenu('About')).top, equals(48.0));
    expect(tester.getRect(findMenu('About')).right, closeTo(98.5, 0.1));
    expect(tester.getRect(findMenu('About')).bottom, equals(208.0));

    expect(find.text('Save', findRichText: true), findsOneWidget);
    expect(find.text('Quit', findRichText: true), findsOneWidget);
    expect(find.text('Magnify', findRichText: true), findsNothing);
    expect(find.text('Minify', findRichText: true), findsNothing);

    // Open the About dialog.
    await tester.sendKeyEvent(LogicalKeyboardKey.keyA, character: 'a');
    await tester.sendKeyUpEvent(LogicalKeyboardKey.altLeft);
    await tester.pumpAndSettle();

    expect(find.text('Save', findRichText: true), findsNothing);
    expect(find.text('Quit', findRichText: true), findsNothing);
    expect(find.text('Magnify', findRichText: true), findsNothing);
    expect(find.text('Minify', findRichText: true), findsNothing);
    expect(find.text('Close'), findsOneWidget);

    await tester.tap(find.text('Close'));
    await tester.pumpAndSettle();
    expect(find.text('Close'), findsNothing);
  });

  testWidgets('MenuBar is wrapped in a SafeArea', (WidgetTester tester) async {
    const double safeAreaPadding = 100.0;
    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(padding: EdgeInsets.symmetric(vertical: safeAreaPadding)),
        child: example.MenuAcceleratorApp(),
      ),
    );

    expect(tester.getTopLeft(find.byType(MenuBar)), const Offset(0.0, safeAreaPadding));
  });
}
