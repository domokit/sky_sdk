// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('PinnedHeaderSliver basics', (WidgetTester tester) async {
    Widget buildFrame({required Axis axis, required bool reverse}) {
      return MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            scrollDirection: axis,
            reverse: reverse,
            slivers: <Widget>[
              const PinnedHeaderSliver(child: Text('PinnedHeaderSliver')),
              SliverList(
                delegate: SliverChildBuilderDelegate(
                  (BuildContext context, int index) => Text('Item $index'),
                  childCount: 100,
                ),
              ),
            ],
          ),
        ),
      );
    }

    Rect getHeaderRect() => tester.getRect(find.text('PinnedHeaderSliver'));
    Rect getItemRect(int index) => tester.getRect(find.text('Item $index'));

    // axis: Axis.vertical, reverse: false
    {
      await tester.pumpWidget(buildFrame(axis: Axis.vertical, reverse: false));
      await tester.pumpAndSettle();
      final ScrollPosition position =
          tester.state<ScrollableState>(find.byType(Scrollable)).position;

      // The test viewport is 800 x 600 (width x height).
      // The header's child is at the top of the scroll view and all items are the same height.
      expect(getHeaderRect().topLeft, Offset.zero);
      expect(getHeaderRect().width, 800);
      expect(getHeaderRect().height, tester.getSize(find.text('PinnedHeaderSliver')).height);

      // First and last visible items
      final double itemHeight = getItemRect(0).height;
      final int visibleItemCount = (600 ~/ itemHeight) - 1; // less 1 for the header
      expect(find.text('Item 0'), findsOneWidget);
      expect(find.text('Item ${visibleItemCount - 1}'), findsOneWidget);

      // Scrolling up and down leaves the header at the top.
      position.moveTo(itemHeight * 5);
      await tester.pumpAndSettle();
      expect(getHeaderRect().top, 0);
      expect(getHeaderRect().width, 800);
      position.moveTo(itemHeight * -5);
      expect(getHeaderRect().top, 0);
      expect(getHeaderRect().width, 800);
    }

    // axis: Axis.horizontal, reverse: false
    {
      await tester.pumpWidget(buildFrame(axis: Axis.horizontal, reverse: false));
      final ScrollPosition position =
          tester.state<ScrollableState>(find.byType(Scrollable)).position;
      await tester.pumpAndSettle();

      expect(getHeaderRect().topLeft, Offset.zero);
      expect(getHeaderRect().height, 600);
      expect(getHeaderRect().width, tester.getSize(find.text('PinnedHeaderSliver')).width);

      // First and last visible items (assuming < 10 items visible)
      final double itemWidth = getItemRect(0).width;
      final int visibleItemCount = (800 - getHeaderRect().width) ~/ itemWidth;
      expect(find.text('Item 0'), findsOneWidget);
      expect(find.text('Item ${visibleItemCount - 1}'), findsOneWidget);

      // Scrolling left and right leaves the header on the left.
      position.moveTo(itemWidth * 5);
      await tester.pumpAndSettle();
      expect(getHeaderRect().left, 0);
      expect(getHeaderRect().height, 600);
      position.moveTo(itemWidth * -5);
      expect(getHeaderRect().left, 0);
      expect(getHeaderRect().height, 600);
    }

    // axis: Axis.vertical, reverse: true
    {
      await tester.pumpWidget(buildFrame(axis: Axis.vertical, reverse: true));
      await tester.pumpAndSettle();
      final ScrollPosition position =
          tester.state<ScrollableState>(find.byType(Scrollable)).position;

      expect(getHeaderRect().bottomLeft, const Offset(0, 600));
      expect(getHeaderRect().width, 800);
      expect(getHeaderRect().height, tester.getSize(find.text('PinnedHeaderSliver')).height);

      // First and last visible items
      final double itemHeight = getItemRect(0).height;
      final int visibleItemCount = (600 ~/ itemHeight) - 1; // less 1 for the header
      expect(find.text('Item 0'), findsOneWidget);
      expect(find.text('Item ${visibleItemCount - 1}'), findsOneWidget);

      // Scrolling up and down leaves the header at the bottom.
      position.moveTo(itemHeight * 5);
      await tester.pumpAndSettle();
      expect(getHeaderRect().bottomLeft, const Offset(0, 600));
      expect(getHeaderRect().width, 800);
      position.moveTo(itemHeight * -5);
      expect(getHeaderRect().bottomLeft, const Offset(0, 600));
      expect(getHeaderRect().width, 800);
    }

    // axis: Axis.horizontal, reverse: true
    {
      await tester.pumpWidget(buildFrame(axis: Axis.horizontal, reverse: true));
      final ScrollPosition position =
          tester.state<ScrollableState>(find.byType(Scrollable)).position;
      await tester.pumpAndSettle();

      expect(getHeaderRect().topRight, const Offset(800, 0));
      expect(getHeaderRect().height, 600);
      expect(getHeaderRect().width, tester.getSize(find.text('PinnedHeaderSliver')).width);

      // First and last visible items (assuming < 10 items visible)
      final double itemWidth = getItemRect(0).width;
      final int visibleItemCount = (800 - getHeaderRect().width) ~/ itemWidth;
      expect(find.text('Item 0'), findsOneWidget);
      expect(find.text('Item ${visibleItemCount - 1}'), findsOneWidget);

      // Scrolling left and right leaves the header on the right.
      position.moveTo(itemWidth * 5);
      await tester.pumpAndSettle();
      expect(getHeaderRect().topRight, const Offset(800, 0));
      expect(getHeaderRect().height, 600);
      position.moveTo(itemWidth * -5);
      expect(getHeaderRect().topRight, const Offset(800, 0));
      expect(getHeaderRect().height, 600);
    }
  });

  testWidgets('PinnedHeaderSliver: multiple headers layout one after the other', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            slivers: <Widget>[
              const PinnedHeaderSliver(child: Text('PinnedHeaderSliver 0')),
              const PinnedHeaderSliver(child: Text('PinnedHeaderSliver 1')),
              const PinnedHeaderSliver(child: Text('PinnedHeaderSliver 2')),
              SliverList(
                delegate: SliverChildBuilderDelegate(
                  (BuildContext context, int index) => Text('Item $index'),
                  childCount: 100,
                ),
              ),
            ],
          ),
        ),
      ),
    );

    final Rect rect0 = tester.getRect(find.text('PinnedHeaderSliver 0'));
    expect(rect0.top, 0);
    expect(rect0.width, 800);

    final Rect rect1 = tester.getRect(find.text('PinnedHeaderSliver 1'));
    expect(rect1.top, rect0.bottom);
    expect(rect1.width, 800);

    final Rect rect2 = tester.getRect(find.text('PinnedHeaderSliver 2'));
    expect(rect2.top, rect1.bottom);
    expect(rect2.width, 800);
  });

  testWidgets('PinnedHeaderSliver: headers that do not start at the top', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            slivers: <Widget>[
              SliverList(
                delegate: SliverChildBuilderDelegate(
                  (BuildContext context, int index) => Text('Item 0.$index'),
                  childCount: 2,
                ),
              ),
              const PinnedHeaderSliver(child: Text('PinnedHeaderSliver 0')),
              SliverList(
                delegate: SliverChildBuilderDelegate(
                  (BuildContext context, int index) => Text('Item 1.$index'),
                  childCount: 2,
                ),
              ),
              const PinnedHeaderSliver(child: Text('PinnedHeaderSliver 1')),
              SliverList(
                delegate: SliverChildBuilderDelegate(
                  (BuildContext context, int index) => Text('Item 2.$index'),
                  childCount: 2,
                ),
              ),
              const PinnedHeaderSliver(child: Text('PinnedHeaderSliver 2')),
              SliverList(
                delegate: SliverChildBuilderDelegate(
                  (BuildContext context, int index) => Text('Item $index'),
                  childCount: 100,
                ),
              ),
            ],
          ),
        ),
      ),
    );

    final double itemHeight = tester.getSize(find.text('Item 0.0')).height;
    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;

    // Scroll 'Item 0.0' and 'Item 0.1' off the top
    position.moveTo(itemHeight * 2);
    await tester.pumpAndSettle();

    // That leaves 'PinnedHeaderSliver 0' at the top
    final Rect rect0 = tester.getRect(find.text('PinnedHeaderSliver 0'));
    expect(rect0.top, 0);
    expect(rect0.width, 800);

    // Scroll 'Item 1.0' and 'Item 1.1' behind 'PinnedHeaderSliver 0'
    position.moveTo(itemHeight * 4);
    await tester.pumpAndSettle();

    // That leaves 'PinnedHeaderSliver 1' below 'PinnedHeaderSliver 0'
    final Rect rect1 = tester.getRect(find.text('PinnedHeaderSliver 1'));
    expect(rect1.top, rect0.bottom);
    expect(rect1.width, 800);

    // Scroll 'Item 2.0' and 'Item 2.1' behind 'PinnedHeaderSliver 1'
    position.moveTo(itemHeight * 6);
    await tester.pumpAndSettle();

    // That leaves 'PinnedHeaderSliver 2' below 'PinnedHeaderSliver 1'
    final Rect rect2 = tester.getRect(find.text('PinnedHeaderSliver 2'));
    expect(rect2.top, rect1.bottom);
    expect(rect2.width, 800);

    // Scroll some more. The headers are already as close to the top as they
    // can go - they will not have moved.
    position.moveTo(itemHeight * 10);
    await tester.pumpAndSettle();
    expect(tester.getRect(find.text('PinnedHeaderSliver 0')), rect0);
    expect(tester.getRect(find.text('PinnedHeaderSliver 1')), rect1);
    expect(tester.getRect(find.text('PinnedHeaderSliver 2')), rect2);
  });
}
