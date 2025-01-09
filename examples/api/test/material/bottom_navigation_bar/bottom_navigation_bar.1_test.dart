// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/bottom_navigation_bar/bottom_navigation_bar.1.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('BottomNavigationBar Updates Screen Content', (WidgetTester tester) async {
    await tester.pumpWidget(const example.BottomNavigationBarExampleApp());

    expect(find.widgetWithText(AppBar, 'BottomNavigationBar Sample'), findsOneWidget);
    expect(find.byType(BottomNavigationBar), findsOneWidget);
    expect(find.widgetWithText(Center, 'Index 0: Home'), findsOneWidget);

    await tester.tap(find.byIcon(Icons.business));
    await tester.pumpAndSettle();
    expect(find.widgetWithText(Center, 'Index 1: Business'), findsOneWidget);

    await tester.tap(find.byIcon(Icons.school));
    await tester.pumpAndSettle();
    expect(find.widgetWithText(Center, 'Index 2: School'), findsOneWidget);

    await tester.tap(find.byIcon(Icons.settings));
    await tester.pumpAndSettle();
    expect(find.widgetWithText(Center, 'Index 3: Settings'), findsOneWidget);

    // Verify we can go back
    await tester.tap(find.byIcon(Icons.home));
    await tester.pumpAndSettle();
    expect(find.widgetWithText(Center, 'Index 0: Home'), findsOneWidget);
  });
}
