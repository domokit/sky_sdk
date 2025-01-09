// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/menu_anchor/menu_anchor.3.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Menu button opens and closes the menu', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SimpleCascadingMenuApp());

    // Find the menu button.
    final Finder menuButton = find.byType(IconButton);
    expect(menuButton, findsOneWidget);

    // Tap the menu button to open the menu.
    await tester.tap(menuButton);
    await tester.pumpAndSettle();

    // Verify that the menu is open.
    expect(find.text('Revert'), findsOneWidget);

    // Tap the menu button again to close the menu.
    await tester.tap(menuButton);
    await tester.pumpAndSettle();

    // Verify that the menu is closed.
    expect(find.text('Revert'), findsNothing);
  });

  testWidgets('Does not show debug banner', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SimpleCascadingMenuApp());
    expect(find.byType(CheckedModeBanner), findsNothing);
  });
}
