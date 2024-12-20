// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/dialog/cupertino_popup_surface.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('CupertinoPopupSurface displays expected widgets in init state', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.PopupSurfaceApp());

    final Finder cupertinoButton = find.byType(CupertinoButton);
    expect(cupertinoButton, findsOneWidget);

    final Finder cupertinoSwitch = find.byType(CupertinoSwitch);
    expect(cupertinoSwitch, findsOneWidget);
  });

  testWidgets('CupertinoPopupSurface is displayed with painted surface', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.PopupSurfaceApp());

    // CupertinoSwitch is toggled on by default.
    expect(tester.widget<CupertinoSwitch>(find.byType(CupertinoSwitch)).value, isTrue);

    // Tap on the CupertinoButton to show the CupertinoPopupSurface.
    await tester.tap(find.byType(CupertinoButton));
    await tester.pumpAndSettle();

    // Make sure CupertinoPopupSurface is showing.
    final Finder cupertinoPopupSurface = find.byType(CupertinoPopupSurface);
    expect(cupertinoPopupSurface, findsOneWidget);

    // Confirm that CupertinoPopupSurface is painted with a ColoredBox.
    final Finder coloredBox = find.descendant(
      of: cupertinoPopupSurface,
      matching: find.byType(ColoredBox),
    );
    expect(coloredBox, findsOneWidget);
  });

  testWidgets('CupertinoPopupSurface is displayed without painted surface', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.PopupSurfaceApp());

    // Toggling off CupertinoSwitch and confirm its state.
    final Finder cupertinoSwitch = find.byType(CupertinoSwitch);
    await tester.tap(cupertinoSwitch);
    await tester.pumpAndSettle();
    expect(tester.widget<CupertinoSwitch>(cupertinoSwitch).value, isFalse);

    // Tap on the CupertinoButton to show the CupertinoPopupSurface.
    await tester.tap(find.byType(CupertinoButton));
    await tester.pumpAndSettle();

    // Make sure CupertinoPopupSurface is showing.
    final Finder cupertinoPopupSurface = find.byType(CupertinoPopupSurface);
    expect(cupertinoPopupSurface, findsOneWidget);

    // Confirm that CupertinoPopupSurface is not painted with a ColoredBox.
    final Finder coloredBox = find.descendant(
      of: cupertinoPopupSurface,
      matching: find.byType(ColoredBox),
    );
    expect(coloredBox, findsNothing);
  });
}
