// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/scroll_end_notification/scroll_end_notification.1.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SliverAutoScroll example', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SliverAutoScrollExampleApp());

    final double itemHeight = tester.getSize(find.widgetWithText(Card, 'Item 0.15')).height;

    // The scroll view is 600 pixels high and the big orange
    // "AlignedItem" is preceded by 15 regular items. Scroll up enough
    // to make it partially visible.
    await tester.drag(find.byType(CustomScrollView), Offset(0, 600 - 15.5 * itemHeight));
    await tester.pumpAndSettle();

    final Finder alignedItem = find.widgetWithText(Card, 'Aligned Item');

    // The "AlignedItem" is now bottom aligned.
    expect(tester.getRect(alignedItem).bottom, 600);

    // Scrolling down a little (less then the big orange item's height) and no
    // auto-scroll occurs.
    await tester.drag(find.byType(CustomScrollView), Offset(0, itemHeight));
    await tester.pumpAndSettle();
    expect(tester.getRect(alignedItem).bottom, 600 + itemHeight);

    // Scroll up a little and the "AlignedItem" does not auto-scroll, because
    // it's fully visible.
    await tester.drag(find.byType(CustomScrollView), Offset(0, -2 * itemHeight));
    await tester.pumpAndSettle();
    expect(tester.getRect(alignedItem).bottom, 600 - itemHeight);

    // Scroll up far enough to make the AlignedItem partially visible and to trigger
    // an auto-scroll that aligns it with the top of the viewport.
    await tester.drag(find.byType(CustomScrollView), Offset(0, -600 + itemHeight * 1.5));
    await tester.pumpAndSettle();
    expect(tester.getRect(alignedItem).top, 0);

    // Scroll down a little and the "AlignedItem" does not auto-scroll because
    // it's fully visible.
    await tester.drag(find.byType(CustomScrollView), Offset(0, itemHeight));
    await tester.pumpAndSettle();
    expect(tester.getRect(alignedItem).top, itemHeight);
  });
}
