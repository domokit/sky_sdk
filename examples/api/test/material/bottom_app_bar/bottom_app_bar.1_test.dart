// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/bottom_app_bar/bottom_app_bar.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('BottomAppBarDemo shows FloatingActionButton and responds to toggle', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.BottomAppBarDemo());

    expect(find.byType(FloatingActionButton), findsOneWidget);

    // Tap the 'Floating Action Button' switch to hide the FAB.
    await tester.tap(find.byType(SwitchListTile).first);
    await tester.pumpAndSettle();

    expect(find.byType(FloatingActionButton), findsNothing);
  });

  testWidgets('Notch can be toggled on and off', (WidgetTester tester) async {
    await tester.pumpWidget(const example.BottomAppBarDemo());

    // Check the BottomAppBar has a notch initially.
    BottomAppBar bottomAppBar = tester.widget(find.byType(BottomAppBar));
    expect(bottomAppBar.shape, isNotNull);

    // Toggle the 'Notch' switch to remove the notch.
    await tester.tap(find.byType(SwitchListTile).last);
    await tester.pump();

    bottomAppBar = tester.widget(find.byType(BottomAppBar));
    expect(bottomAppBar.shape, isNull);
  });

  testWidgets('FAB location can be changed', (WidgetTester tester) async {
    await tester.pumpWidget(const example.BottomAppBarDemo());

    final Offset initialPosition = tester.getCenter(find.byType(FloatingActionButton));

    // Verify the initial position is near the right side (docked to the end).
    final Size screenSize = tester.getSize(find.byType(Scaffold));
    expect(initialPosition.dx, greaterThan(screenSize.width * 0.5));

    // Tap the radio button to move the FAB to centerDocked.
    await tester.tap(find.text('Docked - Center'));
    await tester.pumpAndSettle();

    // Get the new FAB position (centerDocked).
    final Offset newPosition = tester.getCenter(find.byType(FloatingActionButton));

    expect(
      newPosition.dx,
      closeTo(screenSize.width * 0.5, 10), // Center of the screen.
    );
  });
}
