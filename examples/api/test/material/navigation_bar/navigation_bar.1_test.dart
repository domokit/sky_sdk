// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/navigation_bar/navigation_bar.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Navigation bar updates label behavior when tapping buttons', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.NavigationBarApp());
    NavigationBar navigationBarWidget = tester.firstWidget(find.byType(NavigationBar));

    expect(find.text('Label behavior: alwaysShow'), findsOneWidget);

    /// Test alwaysShow label behavior button.
    await tester.tap(find.widgetWithText(ElevatedButton, 'alwaysShow'));
    await tester.pumpAndSettle();

    expect(find.text('Label behavior: alwaysShow'), findsOneWidget);
    expect(navigationBarWidget.labelBehavior, NavigationDestinationLabelBehavior.alwaysShow);

    /// Test onlyShowSelected label behavior button.
    await tester.tap(find.widgetWithText(ElevatedButton, 'onlyShowSelected'));
    await tester.pumpAndSettle();

    expect(find.text('Label behavior: onlyShowSelected'), findsOneWidget);
    navigationBarWidget = tester.firstWidget(find.byType(NavigationBar));
    expect(navigationBarWidget.labelBehavior, NavigationDestinationLabelBehavior.onlyShowSelected);

    /// Test alwaysHide label behavior button.
    await tester.tap(find.widgetWithText(ElevatedButton, 'alwaysHide'));
    await tester.pumpAndSettle();

    expect(find.text('Label behavior: alwaysHide'), findsOneWidget);
    navigationBarWidget = tester.firstWidget(find.byType(NavigationBar));
    expect(navigationBarWidget.labelBehavior, NavigationDestinationLabelBehavior.alwaysHide);
  });

  testWidgets('Overflow buttons are aligned in the center', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NavigationBarApp());

    final OverflowBar overflowBar = tester.widget<OverflowBar>(find.byType(OverflowBar));
    expect(overflowBar.overflowAlignment, OverflowBarAlignment.center);
    expect(overflowBar.overflowSpacing, 10.0);
  });
}
