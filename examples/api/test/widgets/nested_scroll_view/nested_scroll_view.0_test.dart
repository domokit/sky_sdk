// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/nested_scroll_view/nested_scroll_view.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Shows all elements', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NestedScrollViewExampleApp());
    expect(find.byType(NestedScrollView), findsOneWidget);
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(find.byType(TabBarView), findsOneWidget);
    expect(find.byType(Tab), findsNWidgets(2));
    expect(find.byType(CustomScrollView), findsAtLeast(1));
    expect(find.text('Books'), findsOneWidget);
    expect(find.text('Tab 1'), findsOneWidget);
    expect(find.text('Tab 2'), findsOneWidget);
    expect(find.text('Item 0'), findsOneWidget);
    expect(find.text('Item 14'), findsNothing);
    expect(find.text('Item 14', skipOffstage: false), findsOneWidget);
    expect(find.textContaining(RegExp(r'Item \d\d?'), skipOffstage: false), findsAtLeast(15));

    await tester.tap(find.text('Tab 2'));
    await tester.pumpAndSettle();
    expect(find.textContaining(RegExp(r'Item \d\d?'), skipOffstage: false), findsAtLeast(15));
  });

  testWidgets('Shrinks app bar on scroll', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NestedScrollViewExampleApp());

    final double initialAppBarHeight = tester.getTopLeft(find.byType(TabBarView)).dy;
    expect(find.text('Item 1'), findsOneWidget);
    await tester.ensureVisible(find.text('Item 14', skipOffstage: false));
    await tester.pump();
    expect(find.text('Item 1'), findsNothing);

    expect(tester.getTopLeft(find.byType(TabBarView)).dy, lessThan(initialAppBarHeight));
  });
}
