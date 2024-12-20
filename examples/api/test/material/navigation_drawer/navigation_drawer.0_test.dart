// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/navigation_drawer/navigation_drawer.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Navigation bar updates destination on tap', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NavigationDrawerApp());

    await tester.tap(find.text('Open Drawer'));
    await tester.pumpAndSettle();

    final NavigationDrawer navigationDrawerWidget = tester.firstWidget(
      find.byType(NavigationDrawer),
    );

    /// NavigationDestinations must be rendered
    expect(find.text('Messages'), findsNWidgets(2));
    expect(find.text('Profile'), findsNWidgets(2));
    expect(find.text('Settings'), findsNWidgets(2));

    /// Initial index must be zero
    expect(navigationDrawerWidget.selectedIndex, 0);
    expect(find.text('Page Index = 0'), findsOneWidget);

    /// Switch to second tab
    await tester.tap(find.ancestor(of: find.text('Profile'), matching: find.byType(InkWell)));
    await tester.pumpAndSettle();
    expect(find.text('Page Index = 1'), findsOneWidget);

    /// Switch to fourth tab
    await tester.tap(find.ancestor(of: find.text('Settings'), matching: find.byType(InkWell)));
    await tester.pumpAndSettle();
    expect(find.text('Page Index = 2'), findsOneWidget);
  });
}
