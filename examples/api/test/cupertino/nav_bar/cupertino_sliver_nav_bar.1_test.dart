// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/nav_bar/cupertino_sliver_nav_bar.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

const Offset titleDragUp = Offset(0.0, -100.0);
const Offset bottomDragUp = Offset(0.0, -50.0);

void main() {
  testWidgets('Collapse and expand CupertinoSliverNavigationBar changes title position', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.SliverNavBarApp());

    // Large title is visible and at lower position.
    expect(tester.getBottomLeft(find.text('Contacts').first).dy, 88.0);
    await tester.fling(find.text('Drag me up'), titleDragUp, 500.0);
    await tester.pumpAndSettle();

    // Large title is hidden and at higher position.
    expect(
      tester.getBottomLeft(find.text('Contacts').first).dy,
      36.0 + 8.0,
    ); // Static part + _kNavBarBottomPadding.
  });

  testWidgets('Search field is hidden in bottom automatic mode', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SliverNavBarApp());

    // Navigate to a page with bottom automatic mode.
    final Finder nextButton = find.text('Bottom Automatic mode');
    expect(nextButton, findsOneWidget);
    await tester.tap(nextButton);
    await tester.pumpAndSettle();

    // Middle, large title, and search field are visible.
    expect(tester.getBottomLeft(find.text('Contacts Group').first).dy, 30.5);
    expect(tester.getBottomLeft(find.text('Family').first).dy, 88.0);
    expect(tester.getTopLeft(find.byType(CupertinoSearchTextField)).dy, 104.0);
    expect(tester.getBottomLeft(find.byType(CupertinoSearchTextField)).dy, 139.0);

    await tester.fling(find.text('Drag me up'), bottomDragUp, 50.0);
    await tester.pumpAndSettle();

    // Search field is hidden, but large title and middle title are visible.
    expect(tester.getBottomLeft(find.text('Contacts Group').first).dy, 30.5);
    expect(tester.getBottomLeft(find.text('Family').first).dy, 88.0);
    expect(tester.getTopLeft(find.byType(CupertinoSearchTextField)).dy, 104.0);
    expect(tester.getBottomLeft(find.byType(CupertinoSearchTextField)).dy, 104.0);

    await tester.fling(find.text('Drag me up'), titleDragUp, 50.0);
    await tester.pumpAndSettle();

    // Large title and search field are hidden and middle title is visible.
    expect(tester.getBottomLeft(find.text('Contacts Group').first).dy, 30.5);
    expect(
      tester.getBottomLeft(find.text('Family').first).dy,
      36.0 + 8.0,
    ); // Static part + _kNavBarBottomPadding.
    expect(tester.getBottomLeft(find.byType(CupertinoSearchTextField)).dy, 52.0);
  });

  testWidgets('Search field is always shown in bottom always mode', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SliverNavBarApp());

    // Navigate to a page with bottom always mode.
    final Finder nextButton = find.text('Bottom Always mode');
    expect(nextButton, findsOneWidget);
    await tester.tap(nextButton);
    await tester.pumpAndSettle();

    // Middle, large title, and search field are visible.
    expect(tester.getBottomLeft(find.text('Contacts Group').first).dy, 30.5);
    expect(tester.getBottomLeft(find.text('Family').first).dy, 88.0);
    expect(tester.getTopLeft(find.byType(CupertinoSearchTextField)).dy, 104.0);
    expect(tester.getBottomLeft(find.byType(CupertinoSearchTextField)).dy, 139.0);

    await tester.fling(find.text('Drag me up'), titleDragUp, 50.0);
    await tester.pumpAndSettle();

    // Large title is hidden, but search field and middle title are visible.
    expect(tester.getBottomLeft(find.text('Contacts Group').first).dy, 30.5);
    expect(
      tester.getBottomLeft(find.text('Family').first).dy,
      36.0 + 8.0,
    ); // Static part + _kNavBarBottomPadding.
    expect(tester.getTopLeft(find.byType(CupertinoSearchTextField)).dy, 52.0);
    expect(tester.getBottomLeft(find.byType(CupertinoSearchTextField)).dy, 87.0);
  });

  testWidgets('CupertinoSliverNavigationBar with previous route has back button', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.SliverNavBarApp());

    // Navigate to the first page.
    final Finder nextButton1 = find.text('Bottom Automatic mode');
    expect(nextButton1, findsOneWidget);
    await tester.tap(nextButton1);
    await tester.pumpAndSettle();
    expect(nextButton1, findsNothing);

    // Go back to the previous page.
    final Finder backButton1 = find.byType(CupertinoButton);
    expect(backButton1, findsOneWidget);
    await tester.tap(backButton1);
    await tester.pumpAndSettle();
    expect(nextButton1, findsOneWidget);

    // Navigate to the second page.
    final Finder nextButton2 = find.text('Bottom Always mode');
    expect(nextButton2, findsOneWidget);
    await tester.tap(nextButton2);
    await tester.pumpAndSettle();
    expect(nextButton2, findsNothing);

    // Go back to the previous page.
    final Finder backButton2 = find.byType(CupertinoButton);
    expect(backButton2, findsOneWidget);
    await tester.tap(backButton2);
    await tester.pumpAndSettle();
    expect(nextButton2, findsOneWidget);
  });
}
