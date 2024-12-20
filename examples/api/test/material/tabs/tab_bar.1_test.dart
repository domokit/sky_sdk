// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/tabs/tab_bar.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Switch tabs in the TabBar', (WidgetTester tester) async {
    await tester.pumpWidget(const example.TabBarApp());

    final TabBar tabBar = tester.widget<TabBar>(find.byType(TabBar));
    expect(tabBar.tabs.length, 3);

    final Finder tab1 = find.widgetWithIcon(Tab, Icons.cloud_outlined);
    final Finder tab2 = find.widgetWithIcon(Tab, Icons.beach_access_sharp);
    final Finder tab3 = find.widgetWithIcon(Tab, Icons.brightness_5_sharp);

    const String tabBarViewText1 = "It's cloudy here";
    const String tabBarViewText2 = "It's rainy here";
    const String tabBarViewText3 = "It's sunny here";

    expect(find.text(tabBarViewText1), findsOneWidget);
    expect(find.text(tabBarViewText2), findsNothing);
    expect(find.text(tabBarViewText3), findsNothing);

    await tester.tap(tab1);
    await tester.pumpAndSettle();

    expect(find.text(tabBarViewText1), findsOneWidget);
    expect(find.text(tabBarViewText2), findsNothing);
    expect(find.text(tabBarViewText3), findsNothing);

    await tester.tap(tab2);
    await tester.pumpAndSettle();

    expect(find.text(tabBarViewText1), findsNothing);
    expect(find.text(tabBarViewText2), findsOneWidget);
    expect(find.text(tabBarViewText3), findsNothing);

    await tester.tap(tab3);
    await tester.pumpAndSettle();

    expect(find.text(tabBarViewText1), findsNothing);
    expect(find.text(tabBarViewText2), findsNothing);
    expect(find.text(tabBarViewText3), findsOneWidget);
  });
}
