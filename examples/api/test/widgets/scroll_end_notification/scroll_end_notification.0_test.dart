// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/scroll_end_notification/scroll_end_notification.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('IsScrollingListenerApp smoke test', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ScrollEndNotificationApp());

    expect(find.byType(CustomScrollView), findsOneWidget);
    expect(find.byType(Scrollbar), findsOneWidget);

    ScrollPosition getScrollPosition() {
      return tester.widget<CustomScrollView>(find.byType(CustomScrollView)).controller!.position;
    }

    // Viewport is 600 pixels high, each item's height is 100, 6 items are visible.
    expect(getScrollPosition().viewportDimension, 600);
    expect(getScrollPosition().pixels, 0);
    expect(find.text('Item 0'), findsOneWidget);
    expect(find.text('Item 5'), findsOneWidget);

    // Small (< 100) scrolls don't trigger an auto-scroll
    await tester.drag(find.byType(Scrollbar), const Offset(0, -20.0));
    await tester.pumpAndSettle();
    expect(getScrollPosition().pixels, 20);
    expect(find.text('Item 0'), findsOneWidget);

    // Initial scroll is to 220: items 0,1 are scrolled off the top,
    // the bottom 80 pixels of item 2 are visible, items 4-7 are
    // completely visible, the first 20 pixels of item 8 are visible.
    // After the auto-scroll, items 3-8 are completely visible.
    await tester.drag(find.byType(Scrollbar), const Offset(0, -200.0));
    await tester.pumpAndSettle();
    expect(getScrollPosition().pixels, 300);
    expect(find.text('Item 0'), findsNothing);
    expect(find.text('Item 2'), findsNothing);
    expect(find.text('Item 3'), findsOneWidget);
    expect(find.text('Item 8'), findsOneWidget);
  });
}
