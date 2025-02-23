// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SliverResizingHeader basics', (WidgetTester tester) async {
    Widget buildFrame({required Axis axis, required bool reverse}) {
      final (Widget minPrototype, Widget maxPrototype) = switch (axis) {
        Axis.vertical => (const SizedBox(height: 100), const SizedBox(height: 300)),
        Axis.horizontal => (const SizedBox(width: 100), const SizedBox(width: 300)),
      };
      return MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            scrollDirection: axis,
            reverse: reverse,
            slivers: <Widget>[
              SliverResizingHeader(
                minExtentPrototype: minPrototype,
                maxExtentPrototype: maxPrototype,
                child: const SizedBox.expand(child: Text('header')),
              ),
              SliverList(
                delegate: SliverChildBuilderDelegate(
                  (BuildContext context, int index) => Text('item $index'),
                  childCount: 100,
                ),
              ),
            ],
          ),
        ),
      );
    }

    Rect getHeaderRect() => tester.getRect(find.text('header'));
    Rect getItemRect(int index) => tester.getRect(find.text('item $index'));

    // axis: Axis.vertical, reverse: false
    {
      await tester.pumpWidget(buildFrame(axis: Axis.vertical, reverse: false));
      await tester.pumpAndSettle();
      final ScrollPosition position =
          tester.state<ScrollableState>(find.byType(Scrollable)).position;

      // The test viewport is width=800 x height=600
      // The height=300 header is at the top of the scroll view and all items are the same height.
      expect(getHeaderRect().topLeft, Offset.zero);
      expect(getHeaderRect().width, 800);
      expect(getHeaderRect().height, 300);

      // First and last visible items
      final double itemHeight = getItemRect(0).height;
      final int visibleItemCount = 300 ~/ itemHeight; // 300 = viewport height - header height
      expect(find.text('item 0'), findsOneWidget);
      expect(find.text('item ${visibleItemCount - 1}'), findsOneWidget);

      // Scrolling up and down leaves the header at the top but changes its height
      // between the heights of the min and max extent prototypes.
      position.moveTo(200);
      await tester.pumpAndSettle();
      expect(getHeaderRect().topLeft, Offset.zero);
      expect(getHeaderRect().width, 800);
      expect(getHeaderRect().height, 100);
      position.moveTo(0);
      await tester.pumpAndSettle();
      expect(getHeaderRect().topLeft, Offset.zero);
      expect(getHeaderRect().width, 800);
      expect(getHeaderRect().height, 300);
    }

    // axis: Axis.horizontal, reverse: false
    {
      await tester.pumpWidget(buildFrame(axis: Axis.horizontal, reverse: false));
      await tester.pumpAndSettle();
      final ScrollPosition position =
          tester.state<ScrollableState>(find.byType(Scrollable)).position;

      // The width=300 header is at the left of the scroll view and all items are the same width.
      expect(getHeaderRect().topLeft, Offset.zero);
      expect(getHeaderRect().width, 300);
      expect(getHeaderRect().height, 600);

      // First and last visible items (assuming < 10 items visible)
      final double itemWidth = getItemRect(0).width;
      final int visibleItemCount = 500 ~/ itemWidth; // 500 = viewport width - header width
      expect(find.text('item 0'), findsOneWidget);
      expect(find.text('item ${visibleItemCount - 1}'), findsOneWidget);

      // Scrolling up and down leaves the header on the left but changes its width
      // between the heights of the min and max extent prototypes.
      position.moveTo(200);
      await tester.pumpAndSettle();
      expect(getHeaderRect().topLeft, Offset.zero);
      expect(getHeaderRect().height, 600);
      expect(getHeaderRect().width, 100);
      position.moveTo(0);
      await tester.pumpAndSettle();
      expect(getHeaderRect().topLeft, Offset.zero);
      expect(getHeaderRect().height, 600);
      expect(getHeaderRect().width, 300);
    }

    // axis: Axis.vertical, reverse: true
    {
      await tester.pumpWidget(buildFrame(axis: Axis.vertical, reverse: true));
      await tester.pumpAndSettle();
      final ScrollPosition position =
          tester.state<ScrollableState>(find.byType(Scrollable)).position;

      // The height=300 header is at the bottom of the scroll view and all items are the same height.
      expect(getHeaderRect().bottomLeft, const Offset(0, 600));
      expect(getHeaderRect().width, 800);
      expect(getHeaderRect().height, 300);

      // First and last visible items (assuming < 10 items visible)
      final double itemHeight = getItemRect(0).height;
      final int visibleItemCount = 300 ~/ itemHeight; // 300 = viewport height - header height
      expect(find.text('item 0'), findsOneWidget);
      expect(find.text('item ${visibleItemCount - 1}'), findsOneWidget);

      // Scrolling up and down leaves the header at the bottom but changes its height
      // between the heights of the min and max extent prototypes.
      position.moveTo(200);
      await tester.pumpAndSettle();
      expect(getHeaderRect().bottomLeft, const Offset(0, 600));
      expect(getHeaderRect().width, 800);
      expect(getHeaderRect().height, 100);
      position.moveTo(0);
      await tester.pumpAndSettle();
      expect(getHeaderRect().bottomLeft, const Offset(0, 600));
      expect(getHeaderRect().width, 800);
      expect(getHeaderRect().height, 300);
    }

    // axis: Axis.horizontal, reverse: true
    {
      await tester.pumpWidget(buildFrame(axis: Axis.horizontal, reverse: true));
      await tester.pumpAndSettle();
      final ScrollPosition position =
          tester.state<ScrollableState>(find.byType(Scrollable)).position;

      // The width=300 header is on the right of the scroll view and all items are the same width.
      expect(getHeaderRect().topRight, const Offset(800, 0));
      expect(getHeaderRect().width, 300);
      expect(getHeaderRect().height, 600);

      final double itemWidth = getItemRect(0).width;
      final int visibleItemCount = 500 ~/ itemWidth; // 500 = viewport width - header width
      expect(find.text('item 0'), findsOneWidget);
      expect(find.text('item ${visibleItemCount - 1}'), findsOneWidget);

      // Scrolling up and down leaves the header on the left but changes its width
      // between the heights of the min and max extent prototypes.
      position.moveTo(200);
      await tester.pumpAndSettle();
      expect(getHeaderRect().topRight, const Offset(800, 0));
      expect(getHeaderRect().height, 600);
      expect(getHeaderRect().width, 100);
      position.moveTo(0);
      await tester.pumpAndSettle();
      expect(getHeaderRect().topRight, const Offset(800, 0));
      expect(getHeaderRect().height, 600);
      expect(getHeaderRect().width, 300);
    }
  });

  testWidgets('SliverResizingHeader default minExtent is 0', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            slivers: <Widget>[
              const SliverResizingHeader(
                maxExtentPrototype: SizedBox(height: 300),
                child: SizedBox.expand(child: Text('header')),
              ),
              SliverList(
                delegate: SliverChildBuilderDelegate(
                  (BuildContext context, int index) => Text('item $index'),
                  childCount: 100,
                ),
              ),
            ],
          ),
        ),
      ),
    );

    expect(tester.getSize(find.text('header')).height, 300);

    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;

    position.moveTo(299);
    await tester.pumpAndSettle();
    expect(tester.getSize(find.text('header')).height, 1);

    position.moveTo(300);
    await tester.pumpAndSettle();
    expect(find.text('header'), findsNothing);
  });

  testWidgets(
    'SliverResizingHeader with identical min/max prototypes is effectively a pinned header',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: CustomScrollView(
              slivers: <Widget>[
                const SliverResizingHeader(
                  minExtentPrototype: SizedBox(height: 100),
                  maxExtentPrototype: SizedBox(height: 100),
                  child: SizedBox.expand(child: Text('header')),
                ),
                SliverList(
                  delegate: SliverChildBuilderDelegate(
                    (BuildContext context, int index) => Text('item $index'),
                    childCount: 100,
                  ),
                ),
              ],
            ),
          ),
        ),
      );

      expect(tester.getTopLeft(find.text('header')), Offset.zero);
      expect(tester.getSize(find.text('header')), const Size(800, 100));

      final ScrollPosition position =
          tester.state<ScrollableState>(find.byType(Scrollable)).position;

      position.moveTo(100);
      await tester.pumpAndSettle();
      expect(tester.getTopLeft(find.text('header')), Offset.zero);
      expect(tester.getSize(find.text('header')), const Size(800, 100));

      position.moveTo(0);
      await tester.pumpAndSettle();
      expect(tester.getTopLeft(find.text('header')), Offset.zero);
      expect(tester.getSize(find.text('header')), const Size(800, 100));
    },
  );

  testWidgets('SliverResizingHeader default maxExtent matches the child', (
    WidgetTester tester,
  ) async {
    final Key headerKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            slivers: <Widget>[
              SliverResizingHeader(child: SizedBox(key: headerKey, height: 300)),
              SliverList(
                delegate: SliverChildBuilderDelegate(
                  (BuildContext context, int index) => Text('item $index'),
                  childCount: 100,
                ),
              ),
            ],
          ),
        ),
      ),
    );

    expect(tester.getSize(find.byKey(headerKey)).height, 300);

    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;

    position.moveTo(299);
    await tester.pumpAndSettle();
    expect(tester.getSize(find.byKey(headerKey)).height, 1);

    position.moveTo(300);
    await tester.pumpAndSettle();
    expect(find.byKey(headerKey), findsNothing);
  });

  testWidgets('SliverResizingHeader overrides initial out of bounds child size', (
    WidgetTester tester,
  ) async {
    Widget buildFrame(double childHeight) {
      return MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            slivers: <Widget>[
              SliverResizingHeader(
                minExtentPrototype: const SizedBox(height: 100),
                maxExtentPrototype: const SizedBox(height: 300),
                child: SizedBox(height: childHeight, child: const Text('header')),
              ),
            ],
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(50));
    expect(tester.getSize(find.text('header')).height, 100);

    await tester.pumpWidget(buildFrame(350));
    expect(tester.getSize(find.text('header')).height, 300);
  });

  testWidgets('SliverResizingHeader update prototypes', (WidgetTester tester) async {
    Widget buildFrame(double minHeight, double maxHeight) {
      return MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            slivers: <Widget>[
              SliverResizingHeader(
                minExtentPrototype: SizedBox(height: minHeight),
                maxExtentPrototype: SizedBox(height: maxHeight),
                child: const SizedBox(height: 300, child: Text('header')),
              ),
              SliverList(
                delegate: SliverChildBuilderDelegate(
                  (BuildContext context, int index) => SizedBox(height: 50, child: Text('$index')),
                  childCount: 100,
                ),
              ),
            ],
          ),
        ),
      );
    }

    double getHeaderHeight() => tester.getSize(find.text('header')).height;

    await tester.pumpWidget(buildFrame(100, 300));
    expect(getHeaderHeight(), 300);

    // Scroll more than needed to reach the min and max header heights.

    await tester.drag(find.byType(CustomScrollView), const Offset(0, -300));
    await tester.pumpAndSettle();
    expect(getHeaderHeight(), 100);

    await tester.drag(find.byType(CustomScrollView), const Offset(0, 300));
    await tester.pumpAndSettle();
    expect(getHeaderHeight(), 300);

    // Change min,maxExtentPrototype widget heights from 150,200 to

    await tester.pumpWidget(buildFrame(150, 200));
    expect(getHeaderHeight(), 200);

    await tester.drag(find.byType(CustomScrollView), const Offset(0, -100));
    await tester.pumpAndSettle();
    expect(getHeaderHeight(), 150);

    await tester.drag(find.byType(CustomScrollView), const Offset(0, 100));
    await tester.pumpAndSettle();
    expect(getHeaderHeight(), 200);
  });

  testWidgets('SliverResizingHeader maxScrollObstructionExtent', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: NestedScrollView(
            headerSliverBuilder:
                (BuildContext context, _) => <Widget>[
                  SliverOverlapAbsorber(
                    handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                    sliver: const SliverResizingHeader(
                      minExtentPrototype: SizedBox(height: 100),
                      maxExtentPrototype: SizedBox(height: 300),
                      child: SizedBox.expand(child: Text('header')),
                    ),
                  ),
                ],
            body: Builder(
              builder:
                  (BuildContext context) => CustomScrollView(
                    slivers: <Widget>[
                      SliverOverlapInjector(
                        handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                      ),
                      SliverList(
                        delegate: SliverChildBuilderDelegate(
                          (BuildContext context, int index) =>
                              SizedBox(height: 50, child: Text('$index')),
                          childCount: 100,
                        ),
                      ),
                    ],
                  ),
            ),
          ),
        ),
      ),
    );

    double getHeaderHeight() => tester.getSize(find.text('header')).height;

    expect(getHeaderHeight(), 300);

    // After scrolling down 150px, the header height becomes 150px
    await tester.drag(find.byType(NestedScrollView), const Offset(0, -150));
    await tester.pumpAndSettle();
    expect(getHeaderHeight(), 150);

    // After scrolling down an additional 150px, the header height becomes 100px
    await tester.drag(find.byType(NestedScrollView), const Offset(0, -150));
    await tester.pumpAndSettle();
    expect(getHeaderHeight(), 100);
  });
}
