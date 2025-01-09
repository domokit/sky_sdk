// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SliverFloatingHeader basics', (WidgetTester tester) async {
    Widget buildFrame({required Axis axis, required bool reverse}) {
      return MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            scrollDirection: axis,
            reverse: reverse,
            slivers: <Widget>[
              SliverFloatingHeader(
                child: switch (axis) {
                  Axis.vertical => const SizedBox(height: 200, child: Text('header')),
                  Axis.horizontal => const SizedBox(width: 200, child: Text('header')),
                },
              ),
              SliverList(
                delegate: SliverChildBuilderDelegate((BuildContext context, int index) {
                  return switch (axis) {
                    Axis.vertical => SizedBox(height: 100, child: Text('item $index')),
                    Axis.horizontal => SizedBox(width: 100, child: Text('item $index')),
                  };
                }, childCount: 100),
              ),
            ],
          ),
        ),
      );
    }

    Rect getHeaderRect() => tester.getRect(find.text('header'));

    Future<int> scroll(Offset offset) async {
      await tester.timedDrag(
        find.byType(CustomScrollView),
        offset,
        const Duration(milliseconds: 500),
      );
      return tester.pumpAndSettle();
    }

    // axis: Axis.vertical, reverse: false
    {
      await tester.pumpWidget(buildFrame(axis: Axis.vertical, reverse: false));
      await tester.pumpAndSettle();

      // The test viewport is width=800 x height=600
      // The height=200 header is at the top of the scroll view and all items are the same height.
      expect(getHeaderRect().topLeft, Offset.zero);
      expect(getHeaderRect().width, 800);
      expect(getHeaderRect().height, 200);

      // First and last visible items, each item has height=100
      const int visibleItemCount = 4; // viewport height - header height = 400
      expect(find.text('item 0'), findsOneWidget);
      expect(find.text('item ${visibleItemCount - 1}'), findsOneWidget);

      // Scroll the header past the top of the viewport.
      await scroll(const Offset(0, -200));
      expect(find.text('header'), findsNothing);

      // Scroll in the opposite direction a little to trigger the appearance of the floating header.
      await scroll(const Offset(0, 25));
      expect(getHeaderRect(), const Rect.fromLTRB(0, 0, 800, 200));

      // Scrolling further in the same direction, leaves the header where it is.
      await scroll(const Offset(0, 25));
      expect(getHeaderRect(), const Rect.fromLTRB(0, 0, 800, 200));

      // Scroll in the original direction a little to trigger the header's disappearance.
      await scroll(const Offset(0, -25));
      expect(find.text('header'), findsNothing);
    }

    // axis: Axis.horizontal, reverse: false
    {
      await tester.pumpWidget(buildFrame(axis: Axis.horizontal, reverse: false));
      await tester.pumpAndSettle();

      expect(getHeaderRect().topLeft, Offset.zero);
      expect(getHeaderRect().width, 200);
      expect(getHeaderRect().height, 600);

      // First and last visible items. Each item has width=100
      const int visibleItemCount = 6; // 600 = viewport width - header width
      expect(find.text('item 0'), findsOneWidget);
      expect(find.text('item ${visibleItemCount - 1}'), findsOneWidget);

      // Scroll the header past the left edge of the viewport.
      await scroll(const Offset(-200, 0));
      expect(find.text('header'), findsNothing);

      // Scroll in the opposite direction a little to trigger the appearance of the floating header.
      await scroll(const Offset(25, 0));
      expect(getHeaderRect(), const Rect.fromLTRB(0, 0, 200, 600));

      // Scrolling further in the same direction, leaves the header where it is.
      await scroll(const Offset(25, 0));
      expect(getHeaderRect(), const Rect.fromLTRB(0, 0, 200, 600));

      // Scroll in the original direction a little to trigger the header's disappearance.
      await scroll(const Offset(-25, 0));
      expect(find.text('header'), findsNothing);
    }

    // axis: Axis.vertical, reverse: true
    {
      await tester.pumpWidget(buildFrame(axis: Axis.vertical, reverse: true));
      await tester.pumpAndSettle();

      expect(getHeaderRect().topLeft, const Offset(0, 400));
      expect(getHeaderRect().width, 800);
      expect(getHeaderRect().height, 200);

      // First and last visible items, each item has height=100
      const int visibleItemCount = 4; // viewport height - header height = 400
      expect(find.text('item 0'), findsOneWidget);
      expect(find.text('item ${visibleItemCount - 1}'), findsOneWidget);

      // Scroll the header past the bottom of the viewport.
      await scroll(const Offset(0, 200));
      expect(find.text('header'), findsNothing);

      // Scroll in the opposite direction a little to trigger the appearance of the floating header.
      await scroll(const Offset(0, -25));
      expect(getHeaderRect(), const Rect.fromLTRB(0, 400, 800, 600));

      // Scrolling further in the same direction, leaves the header where it is.
      await scroll(const Offset(0, -25));
      expect(getHeaderRect(), const Rect.fromLTRB(0, 400, 800, 600));

      // Scroll in the original direction a little to trigger the header's disappearance.
      await scroll(const Offset(0, 25));
      expect(find.text('header'), findsNothing);
    }

    // axis: Axis.horizontal, reverse: true
    {
      await tester.pumpWidget(buildFrame(axis: Axis.horizontal, reverse: true));
      await tester.pumpAndSettle();

      expect(getHeaderRect().topLeft, const Offset(600, 0));
      expect(getHeaderRect().width, 200);
      expect(getHeaderRect().height, 600);

      // First and last visible items. Each item has width=100
      const int visibleItemCount = 6; // 600 = viewport width - header width
      expect(find.text('item 0'), findsOneWidget);
      expect(find.text('item ${visibleItemCount - 1}'), findsOneWidget);

      // Scroll the header past the right edge of the viewport.
      await scroll(const Offset(200, 0));
      expect(find.text('header'), findsNothing);

      // Scroll in the opposite direction a little to trigger the appearance of the floating header.
      await scroll(const Offset(-25, 0));
      expect(getHeaderRect(), const Rect.fromLTRB(600, 0, 800, 600));

      // Scrolling further in the same direction, leaves the header where it is.
      await scroll(const Offset(-25, 0));
      expect(getHeaderRect(), const Rect.fromLTRB(600, 0, 800, 600));

      // Scroll in the original direction a little to trigger the header's disappearance.
      await scroll(const Offset(25, 0));
      expect(find.text('header'), findsNothing);
    }
  });

  testWidgets('SliverFloatingHeader override default AnimationStyle', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            slivers: <Widget>[
              SliverFloatingHeader(
                animationStyle: AnimationStyle(
                  curve: Curves.linear,
                  reverseCurve: Curves.linear,
                  duration: const Duration(seconds: 1),
                  reverseDuration: const Duration(seconds: 1),
                ),
                child: const SizedBox(height: 200, child: Text('header')),
              ),
              SliverList(
                delegate: SliverChildBuilderDelegate((BuildContext context, int index) {
                  return SizedBox(height: 100, child: Text('item $index'));
                }, childCount: 100),
              ),
            ],
          ),
        ),
      ),
    );

    Rect getHeaderRect() => tester.getRect(find.text('header'));

    Future<void> scroll(Offset offset) async {
      return tester.timedDrag(
        find.byType(CustomScrollView),
        offset,
        const Duration(milliseconds: 500),
      );
    }

    // The test viewport is width=800 x height=600
    // The height=200 header is at the top of the scroll view and all items are the same height.
    expect(getHeaderRect().topLeft, Offset.zero);
    expect(getHeaderRect().width, 800);
    expect(getHeaderRect().height, 200);

    // Scroll the header past the top of the viewport.
    await scroll(const Offset(0, -200));
    await tester.pumpAndSettle();
    expect(find.text('header'), findsNothing);

    // Scroll in the opposite direction a little to trigger the appearance of the floating header.
    await scroll(const Offset(0, 25));

    // Initially the header is where the drag left it => it's moved 25 downwards
    expect(getHeaderRect(), const Rect.fromLTRB(0, -175, 800, 25));

    // With a linear animation curve, after half the animation's duration (500ms), we'll
    // have moved downwards half of the remaining 175:
    await tester.pump(const Duration(milliseconds: 500));
    expect(getHeaderRect(), const Rect.fromLTRB(0, -175 / 2, 800, 200 - 175 / 2));

    // After the remainder of the animation's duration the header is back
    // where it started.
    await tester.pump(const Duration(milliseconds: 500));
    expect(getHeaderRect(), const Rect.fromLTRB(0, 0, 800, 200));
  });

  testWidgets('SliverFloatingHeader snapMode parameter', (WidgetTester tester) async {
    Widget buildFrame(FloatingHeaderSnapMode snapMode) {
      return MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            slivers: <Widget>[
              SliverFloatingHeader(
                snapMode: snapMode,
                child: const SizedBox(height: 200, child: Text('header')),
              ),
              SliverList(
                delegate: SliverChildBuilderDelegate((BuildContext context, int index) {
                  return SizedBox(height: 100, child: Text('item $index'));
                }, childCount: 100),
              ),
            ],
          ),
        ),
      );
    }

    Rect getHeaderRect() => tester.getRect(find.text('header'));
    double getItem0Y() => tester.getRect(find.text('item 0')).topLeft.dy;

    Future<void> scroll(Offset offset) async {
      return tester.timedDrag(
        find.byType(CustomScrollView),
        offset,
        const Duration(milliseconds: 500),
      );
    }

    // FloatingHeaderSnapMode.overlay
    {
      await tester.pumpWidget(buildFrame(FloatingHeaderSnapMode.overlay));
      await tester.pumpAndSettle();
      expect(getHeaderRect(), const Rect.fromLTRB(0, 0, 800, 200));
      expect(getItem0Y(), 200);

      // Scrolling in this direction will move more than 200 because
      // timedDrag() concludes with a fling and there's room for a
      // 200+ scroll.
      await scroll(const Offset(0, -200));
      await tester.pumpAndSettle();
      expect(find.text('header'), findsNothing);
      final double item0StartY = getItem0Y();
      expect(item0StartY, lessThan(0));

      // Trigger the appearance of the floating header. There's no
      // fling component to the scroll in this case because the scroll
      // offset is small.
      await scroll(const Offset(0, 25));
      await tester.pumpAndSettle();

      // Item0 has only moved as far as the scroll because
      // the snapMode is overlay.
      expect(getItem0Y(), item0StartY + 25);

      // Return the header and item0 to their initial layout.
      await scroll(const Offset(0, 200));
      await tester.pumpAndSettle();
      expect(getHeaderRect(), const Rect.fromLTRB(0, 0, 800, 200));
      expect(getItem0Y(), 200);
    }

    // FloatingHeaderSnapMode.scroll
    {
      await tester.pumpWidget(buildFrame(FloatingHeaderSnapMode.scroll));
      await tester.pumpAndSettle();
      expect(getHeaderRect(), const Rect.fromLTRB(0, 0, 800, 200));
      expect(getItem0Y(), 200);

      await scroll(const Offset(0, -200));
      await tester.pumpAndSettle();
      expect(find.text('header'), findsNothing);
      final double item0StartY = getItem0Y();
      expect(item0StartY, lessThan(0));

      // Trigger the appearance of the floating header.
      await scroll(const Offset(0, 25));
      await tester.pumpAndSettle();

      // Item0 has moved as far as the scroll (25) plus the height of
      // the header (200) because the snapMode is scroll and the
      // entire header had to snap in.
      expect(getItem0Y(), item0StartY + 200 + 25);
    }
  });
}
