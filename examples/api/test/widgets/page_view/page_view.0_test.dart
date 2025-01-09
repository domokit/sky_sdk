// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/page_view/page_view.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('PageView swipe gestures on mobile platforms', (WidgetTester tester) async {
    await tester.pumpWidget(const example.PageViewExampleApp());

    // Verify that first page is shown initially.
    expect(find.text('First Page'), findsOneWidget);

    // Swipe to the left.
    await tester.fling(find.text('First Page'), const Offset(-300.0, 0.0), 3000);
    await tester.pumpAndSettle();
    // Verify that the second page is shown.
    expect(find.text('Second Page'), findsOneWidget);

    // Swipe back to the right.
    await tester.fling(find.text('Second Page'), const Offset(300.0, 0.0), 3000);
    await tester.pumpAndSettle();
    // Verify that first page is shown.
    expect(find.text('First Page'), findsOneWidget);
  }, variant: TargetPlatformVariant.mobile());

  testWidgets(
    'PageView navigation using forward/backward buttons on desktop platforms',
    (WidgetTester tester) async {
      await tester.pumpWidget(const example.PageViewExampleApp());

      // Verify that first page is shown along with forward/backward buttons in page indicator.
      expect(find.text('First Page'), findsOneWidget);
      expect(find.byType(TabPageSelector), findsOneWidget);

      // Tap forward button on page indicator area.
      await tester.tap(find.byIcon(Icons.arrow_right_rounded));
      await tester.pumpAndSettle();
      // Verify that second page is shown.
      expect(find.text('Second Page'), findsOneWidget);

      // Verify that page indicator index is updated.
      final TabPageSelector pageIndicator = tester.widget<TabPageSelector>(
        find.byType(TabPageSelector),
      );
      expect(pageIndicator.controller?.index, 1);

      // Verify that page view index is also updated with same index to page indicator.
      final PageView pageView = tester.widget<PageView>(find.byType(PageView));
      expect(pageView.controller!.page, 1);

      // Tap backward button on page indicator area.
      await tester.tap(find.byIcon(Icons.arrow_left_rounded));
      await tester.pumpAndSettle();
      // Verify that first page is shown.
      expect(find.text('First Page'), findsOneWidget);

      // Tap backward button one more time.
      await tester.tap(find.byIcon(Icons.arrow_left_rounded));
      await tester.pumpAndSettle();
      // Verify that first page is still shown.
      expect(find.text('First Page'), findsOneWidget);
    },
    variant: TargetPlatformVariant.desktop(),
  );
}
