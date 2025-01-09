// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/bottom_app_bar/bottom_app_bar.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Floating Action Button visibility can be toggled', (WidgetTester tester) async {
    await tester.pumpWidget(const example.BottomAppBarDemo());

    expect(find.byType(FloatingActionButton), findsOneWidget);

    // Tap the switch to hide the FAB.
    await tester.tap(find.byType(SwitchListTile).first);
    await tester.pumpAndSettle();

    expect(find.byType(FloatingActionButton), findsNothing);
  });

  testWidgets('BottomAppBar elevation can be toggled', (WidgetTester tester) async {
    // Build the app.
    await tester.pumpWidget(const example.BottomAppBarDemo());

    // Verify the BottomAppBar has elevation initially.
    BottomAppBar bottomAppBar = tester.widget(find.byType(BottomAppBar));
    expect(bottomAppBar.elevation, isNot(0.0));

    await tester.tap(find.text('Bottom App Bar Elevation'));
    await tester.pumpAndSettle();

    bottomAppBar = tester.widget(find.byType(BottomAppBar));
    expect(bottomAppBar.elevation, equals(0.0));
  });

  testWidgets('BottomAppBar hides on scroll down and shows on scroll up', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.BottomAppBarDemo());

    // Ensure the BottomAppBar is visible initially.
    expect(find.byType(BottomAppBar), findsOneWidget);

    // Scroll down to hide the BottomAppBar.
    await tester.drag(find.byType(ListView), const Offset(0, -300));
    await tester.pumpAndSettle();

    // Verify the BottomAppBar is hidden.
    final Size hiddenSize = tester.getSize(find.byType(AnimatedContainer));
    expect(hiddenSize.height, equals(0.0)); // AnimatedContainer's height

    // Scroll up to show the BottomAppBar again.
    await tester.drag(find.byType(ListView), const Offset(0, 300));
    await tester.pumpAndSettle();

    // Verify the BottomAppBar is visible again.
    final Size visibleSize = tester.getSize(find.byType(AnimatedContainer));
    expect(visibleSize.height, equals(80.0));
  });

  testWidgets('SnackBar is shown when Open popup menu is pressed', (WidgetTester tester) async {
    await tester.pumpWidget(const example.BottomAppBarDemo());

    // Trigger the SnackBar.
    await tester.tap(find.byTooltip('Open popup menu'));
    await tester.pump();

    expect(find.text('Yay! A SnackBar!'), findsOneWidget);

    expect(find.text('Undo'), findsOneWidget);
  });
}
