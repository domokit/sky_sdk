// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

class TestSliverPersistentHeaderDelegate extends SliverPersistentHeaderDelegate {
  TestSliverPersistentHeaderDelegate(this._maxExtent);

  final double _maxExtent;

  @override
  double get maxExtent => _maxExtent;

  @override
  double get minExtent => 16.0;

  @override
  Widget build(BuildContext context, double shrinkOffset, bool overlapsContent) {
    return Column(children: <Widget>[Container(height: minExtent), Expanded(child: Container())]);
  }

  @override
  bool shouldRebuild(TestSliverPersistentHeaderDelegate oldDelegate) => false;
}

class TestBehavior extends ScrollBehavior {
  const TestBehavior();

  @override
  Widget buildOverscrollIndicator(BuildContext context, Widget child, ScrollableDetails details) {
    return GlowingOverscrollIndicator(
      axisDirection: details.direction,
      color: const Color(0xFFFFFFFF),
      child: child,
    );
  }
}

class TestScrollPhysics extends ClampingScrollPhysics {
  const TestScrollPhysics({super.parent});

  @override
  TestScrollPhysics applyTo(ScrollPhysics? ancestor) {
    return TestScrollPhysics(parent: parent?.applyTo(ancestor) ?? ancestor);
  }

  @override
  Tolerance toleranceFor(ScrollMetrics metrics) => const Tolerance(velocity: 20.0, distance: 1.0);
}

void main() {
  testWidgets('Evil test of sliver features - 1', (WidgetTester tester) async {
    final GlobalKey centerKey = GlobalKey();
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: ScrollConfiguration(
            behavior: const TestBehavior(),
            child: Scrollbar(
              child: Scrollable(
                physics: const TestScrollPhysics(),
                viewportBuilder: (BuildContext context, ViewportOffset offset) {
                  return Viewport(
                    anchor: 0.25,
                    offset: offset,
                    center: centerKey,
                    slivers: <Widget>[
                      SliverToBoxAdapter(child: Container(height: 5.0)),
                      SliverToBoxAdapter(child: Container(height: 520.0)),
                      SliverToBoxAdapter(child: Container(height: 520.0)),
                      SliverToBoxAdapter(child: Container(height: 520.0)),
                      SliverPersistentHeader(
                        delegate: TestSliverPersistentHeaderDelegate(150.0),
                        pinned: true,
                      ),
                      SliverToBoxAdapter(child: Container(height: 520.0)),
                      SliverPadding(
                        padding: const EdgeInsets.all(50.0),
                        sliver: SliverToBoxAdapter(child: Container(height: 520.0)),
                      ),
                      SliverToBoxAdapter(child: Container(height: 520.0)),
                      SliverPersistentHeader(
                        delegate: TestSliverPersistentHeaderDelegate(150.0),
                        floating: true,
                      ),
                      SliverToBoxAdapter(child: Container(height: 520.0)),
                      SliverToBoxAdapter(
                        key: centerKey,
                        child: Container(height: 520.0),
                      ), // ------------------------ CENTER ------------------------
                      SliverPersistentHeader(
                        delegate: TestSliverPersistentHeaderDelegate(150.0),
                        pinned: true,
                      ),
                      SliverToBoxAdapter(child: Container(height: 520.0)),
                      SliverToBoxAdapter(child: Container(height: 520.0)),
                      SliverToBoxAdapter(child: Container(height: 520.0)),
                      SliverPadding(
                        padding: const EdgeInsets.all(50.0),
                        sliver: SliverPersistentHeader(
                          delegate: TestSliverPersistentHeaderDelegate(250.0),
                          pinned: true,
                        ),
                      ),
                      SliverToBoxAdapter(child: Container(height: 520.0)),
                      SliverPersistentHeader(
                        delegate: TestSliverPersistentHeaderDelegate(250.0),
                        pinned: true,
                      ),
                      SliverToBoxAdapter(child: Container(height: 5.0)),
                      SliverPersistentHeader(
                        delegate: TestSliverPersistentHeaderDelegate(250.0),
                        pinned: true,
                      ),
                      SliverToBoxAdapter(child: Container(height: 5.0)),
                      SliverPersistentHeader(
                        delegate: TestSliverPersistentHeaderDelegate(250.0),
                        pinned: true,
                      ),
                      SliverPersistentHeader(
                        delegate: TestSliverPersistentHeaderDelegate(250.0),
                        pinned: true,
                      ),
                      SliverToBoxAdapter(child: Container(height: 5.0)),
                      SliverPersistentHeader(
                        delegate: TestSliverPersistentHeaderDelegate(250.0),
                        pinned: true,
                      ),
                      SliverPersistentHeader(delegate: TestSliverPersistentHeaderDelegate(250.0)),
                      SliverToBoxAdapter(child: Container(height: 520.0)),
                      SliverPersistentHeader(
                        delegate: TestSliverPersistentHeaderDelegate(150.0),
                        floating: true,
                      ),
                      SliverToBoxAdapter(child: Container(height: 520.0)),
                      SliverPersistentHeader(
                        delegate: TestSliverPersistentHeaderDelegate(150.0),
                        floating: true,
                      ),
                      SliverToBoxAdapter(child: Container(height: 5.0)),
                      SliverList(
                        delegate: SliverChildListDelegate(<Widget>[
                          Container(height: 50.0),
                          Container(height: 50.0),
                          Container(height: 50.0),
                          Container(height: 50.0),
                          Container(height: 50.0),
                          Container(height: 50.0),
                          Container(height: 50.0),
                          Container(height: 50.0),
                          Container(height: 50.0),
                          Container(height: 50.0),
                          Container(height: 50.0),
                          Container(height: 50.0),
                          Container(height: 50.0),
                          Container(height: 50.0),
                          Container(height: 50.0),
                        ]),
                      ),
                      SliverPersistentHeader(delegate: TestSliverPersistentHeaderDelegate(250.0)),
                      SliverPersistentHeader(delegate: TestSliverPersistentHeaderDelegate(250.0)),
                      SliverPersistentHeader(delegate: TestSliverPersistentHeaderDelegate(250.0)),
                      SliverPadding(
                        padding: const EdgeInsets.symmetric(horizontal: 50.0),
                        sliver: SliverToBoxAdapter(child: Container(height: 520.0)),
                      ),
                      SliverToBoxAdapter(child: Container(height: 520.0)),
                      SliverToBoxAdapter(child: Container(height: 520.0)),
                      SliverToBoxAdapter(child: Container(height: 5.0)),
                    ],
                  );
                },
              ),
            ),
          ),
        ),
      ),
    );
    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;

    position.animateTo(10000.0, curve: Curves.linear, duration: const Duration(minutes: 1));
    await tester.pump(const Duration(milliseconds: 10));
    await tester.pump(const Duration(milliseconds: 10));
    await tester.pump(const Duration(milliseconds: 50));
    await tester.pumpAndSettle(const Duration(milliseconds: 122));

    position.animateTo(-10000.0, curve: Curves.linear, duration: const Duration(minutes: 1));
    await tester.pump(const Duration(milliseconds: 10));
    await tester.pump(const Duration(milliseconds: 10));
    await tester.pump(const Duration(milliseconds: 50));
    await tester.pumpAndSettle(const Duration(milliseconds: 122));

    position.animateTo(10000.0, curve: Curves.linear, duration: const Duration(minutes: 1));
    await tester.pump(const Duration(milliseconds: 10));
    await tester.pump(const Duration(milliseconds: 10));
    await tester.pump(const Duration(milliseconds: 50));
    await tester.pumpAndSettle(const Duration(milliseconds: 122));

    position.animateTo(-10000.0, curve: Curves.linear, duration: const Duration(seconds: 1));
    await tester.pump(const Duration(milliseconds: 10));
    await tester.pump(const Duration(milliseconds: 10));
    await tester.pump(const Duration(milliseconds: 50));
    await tester.pumpAndSettle(const Duration(milliseconds: 122));

    position.animateTo(10000.0, curve: Curves.linear, duration: const Duration(seconds: 1));
    await tester.pump(const Duration(milliseconds: 10));
    await tester.pump(const Duration(milliseconds: 10));
    await tester.pump(const Duration(milliseconds: 50));
    await tester.pumpAndSettle(const Duration(milliseconds: 122));
  });

  testWidgets('Removing offscreen items above and rescrolling does not crash', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: CustomScrollView(
          cacheExtent: 0.0,
          slivers: <Widget>[
            SliverFixedExtentList(
              itemExtent: 100.0,
              delegate: SliverChildBuilderDelegate((BuildContext context, int index) {
                return ColoredBox(color: Colors.blue, child: Text(index.toString()));
              }, childCount: 30),
            ),
          ],
        ),
      ),
    );

    await tester.drag(find.text('5'), const Offset(0.0, -500.0));
    await tester.pump();

    // Screen is 600px high. Moved bottom item 500px up. It's now at the top.
    expect(tester.getTopLeft(find.widgetWithText(ColoredBox, '5')).dy, 0.0);
    expect(tester.getBottomLeft(find.widgetWithText(ColoredBox, '10')).dy, 600.0);

    // Stop returning the first 3 items.
    await tester.pumpWidget(
      MaterialApp(
        home: CustomScrollView(
          cacheExtent: 0.0,
          slivers: <Widget>[
            SliverFixedExtentList(
              itemExtent: 100.0,
              delegate: SliverChildBuilderDelegate((BuildContext context, int index) {
                if (index > 3) {
                  return ColoredBox(color: Colors.blue, child: Text(index.toString()));
                }
                return null;
              }, childCount: 30),
            ),
          ],
        ),
      ),
    );

    await tester.drag(find.text('5'), const Offset(0.0, 400.0));
    await tester.pump();

    // Move up by 4 items, meaning item 1 would have been at the top but
    // 0 through 3 no longer exist, so item 4, 3 items down, is the first one.
    // Item 4 is also shifted to the top.
    expect(tester.getTopLeft(find.widgetWithText(ColoredBox, '4')).dy, 0.0);

    // Because the screen is still 600px, item 9 is now visible at the bottom instead
    // of what's supposed to be item 6 had we not re-shifted.
    expect(tester.getBottomLeft(find.widgetWithText(ColoredBox, '9')).dy, 600.0);
  });
}
