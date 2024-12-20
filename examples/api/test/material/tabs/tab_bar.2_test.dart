// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/tabs/tab_bar.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Switch tabs in the TabBar', (WidgetTester tester) async {
    const String primaryTabLabel1 = 'Flights';
    const String primaryTabLabel2 = 'Trips';
    const String primaryTabLabel3 = 'Explore';
    const String secondaryTabLabel1 = 'Overview';
    const String secondaryTabLabel2 = 'Specifications';

    await tester.pumpWidget(const example.TabBarApp());

    final TabBar primaryTabBar = tester.widget<TabBar>(find.byType(TabBar).last);
    expect(primaryTabBar.tabs.length, 3);

    final TabBar secondaryTabBar = tester.widget<TabBar>(find.byType(TabBar).first);
    expect(secondaryTabBar.tabs.length, 2);

    String tabBarViewText = '$primaryTabLabel2: $secondaryTabLabel1 tab';
    expect(find.text(tabBarViewText), findsOneWidget);

    await tester.tap(find.text(primaryTabLabel1));
    await tester.pumpAndSettle();

    tabBarViewText = '$primaryTabLabel1: $secondaryTabLabel1 tab';
    expect(find.text(tabBarViewText), findsOneWidget);

    await tester.tap(find.text(secondaryTabLabel2));
    await tester.pumpAndSettle();

    tabBarViewText = '$primaryTabLabel1: $secondaryTabLabel2 tab';
    expect(find.text(tabBarViewText), findsOneWidget);

    await tester.tap(find.text(primaryTabLabel2));
    await tester.pumpAndSettle();

    tabBarViewText = '$primaryTabLabel2: $secondaryTabLabel1 tab';
    expect(find.text(tabBarViewText), findsOneWidget);

    await tester.tap(find.text(secondaryTabLabel2));
    await tester.pumpAndSettle();

    tabBarViewText = '$primaryTabLabel2: $secondaryTabLabel2 tab';
    expect(find.text(tabBarViewText), findsOneWidget);

    await tester.tap(find.text(primaryTabLabel3));
    await tester.pumpAndSettle();

    tabBarViewText = '$primaryTabLabel3: $secondaryTabLabel1 tab';
    expect(find.text(tabBarViewText), findsOneWidget);

    await tester.tap(find.text(secondaryTabLabel2));
    await tester.pumpAndSettle();

    tabBarViewText = '$primaryTabLabel3: $secondaryTabLabel2 tab';
    expect(find.text(tabBarViewText), findsOneWidget);
  });
}
