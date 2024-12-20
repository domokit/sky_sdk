// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void verifyPaintPosition(GlobalKey key, Offset ideal, bool visible) {
  final RenderSliver target = key.currentContext!.findRenderObject()! as RenderSliver;
  expect(target.parent, isA<RenderViewport>());
  final SliverPhysicalParentData parentData = target.parentData! as SliverPhysicalParentData;
  final Offset actual = parentData.paintOffset;
  expect(actual, ideal);
  final SliverGeometry geometry = target.geometry!;
  expect(geometry.visible, visible);
}

void verifyActualBoxPosition(WidgetTester tester, Finder finder, int index, Rect ideal) {
  final RenderBox box = tester.renderObjectList<RenderBox>(finder).elementAt(index);
  final Rect rect = Rect.fromPoints(
    box.localToGlobal(Offset.zero),
    box.localToGlobal(box.size.bottomRight(Offset.zero)),
  );
  expect(rect, equals(ideal));
}

void main() {
  testWidgets("Sliver appbars - floating - scroll offset doesn't change", (
    WidgetTester tester,
  ) async {
    const double bigHeight = 1000.0;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[
            const BigSliver(height: bigHeight),
            SliverPersistentHeader(delegate: TestDelegate(), floating: true),
            const BigSliver(height: bigHeight),
          ],
        ),
      ),
    );
    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;
    final double max =
        bigHeight * 2.0 +
        TestDelegate().maxExtent -
        600.0; // 600 is the height of the test viewport
    assert(max < 10000.0);
    expect(max, 1600.0);
    expect(position.pixels, 0.0);
    expect(position.minScrollExtent, 0.0);
    expect(position.maxScrollExtent, max);
    position.animateTo(10000.0, curve: Curves.linear, duration: const Duration(minutes: 1));
    await tester.pumpAndSettle(const Duration(milliseconds: 50));
    expect(position.pixels, max);
    expect(position.minScrollExtent, 0.0);
    expect(position.maxScrollExtent, max);
  });

  testWidgets('Sliver appbars - floating - normal behavior works', (WidgetTester tester) async {
    final TestDelegate delegate = TestDelegate();
    const double bigHeight = 1000.0;
    GlobalKey key1, key2, key3;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[
            BigSliver(key: key1 = GlobalKey(), height: bigHeight),
            SliverPersistentHeader(key: key2 = GlobalKey(), delegate: delegate, floating: true),
            BigSliver(key: key3 = GlobalKey(), height: bigHeight),
          ],
        ),
      ),
    );
    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;

    verifyPaintPosition(key1, Offset.zero, true);
    verifyPaintPosition(key2, const Offset(0.0, 1000.0), false);
    verifyPaintPosition(key3, const Offset(0.0, 1200.0), false);

    position.animateTo(
      bigHeight - 600.0 + delegate.maxExtent,
      curve: Curves.linear,
      duration: const Duration(minutes: 1),
    );
    await tester.pumpAndSettle(const Duration(milliseconds: 1000));
    verifyPaintPosition(key1, Offset.zero, true);
    verifyPaintPosition(key2, Offset(0.0, 600.0 - delegate.maxExtent), true);
    verifyActualBoxPosition(
      tester,
      find.byType(Container),
      0,
      Rect.fromLTWH(0.0, 600.0 - delegate.maxExtent, 800.0, delegate.maxExtent),
    );
    verifyPaintPosition(key3, const Offset(0.0, 600.0), false);

    assert(delegate.maxExtent * 2.0 < 600.0); // make sure this fits on the test screen...
    position.animateTo(
      bigHeight - 600.0 + delegate.maxExtent * 2.0,
      curve: Curves.linear,
      duration: const Duration(minutes: 1),
    );
    await tester.pumpAndSettle(const Duration(milliseconds: 1000));
    verifyPaintPosition(key1, Offset.zero, true);
    verifyPaintPosition(key2, Offset(0.0, 600.0 - delegate.maxExtent * 2.0), true);
    verifyActualBoxPosition(
      tester,
      find.byType(Container),
      0,
      Rect.fromLTWH(0.0, 600.0 - delegate.maxExtent * 2.0, 800.0, delegate.maxExtent),
    );
    verifyPaintPosition(key3, Offset(0.0, 600.0 - delegate.maxExtent), true);

    position.animateTo(bigHeight, curve: Curves.linear, duration: const Duration(minutes: 1));
    await tester.pumpAndSettle(const Duration(milliseconds: 1000));
    verifyPaintPosition(key1, Offset.zero, false);
    verifyPaintPosition(key2, Offset.zero, true);
    verifyActualBoxPosition(
      tester,
      find.byType(Container),
      0,
      Rect.fromLTWH(0.0, 0.0, 800.0, delegate.maxExtent),
    );
    verifyPaintPosition(key3, Offset(0.0, delegate.maxExtent), true);

    position.animateTo(
      bigHeight + delegate.maxExtent * 0.1,
      curve: Curves.linear,
      duration: const Duration(minutes: 1),
    );
    await tester.pumpAndSettle(const Duration(milliseconds: 1000));
    verifyPaintPosition(key1, Offset.zero, false);
    verifyPaintPosition(key2, Offset.zero, true);
    verifyActualBoxPosition(
      tester,
      find.byType(Container),
      0,
      Rect.fromLTWH(0.0, 0.0, 800.0, delegate.maxExtent * 0.9),
    );
    verifyPaintPosition(key3, Offset(0.0, delegate.maxExtent * 0.9), true);

    position.animateTo(
      bigHeight + delegate.maxExtent * 0.5,
      curve: Curves.linear,
      duration: const Duration(minutes: 1),
    );
    await tester.pumpAndSettle(const Duration(milliseconds: 1000));
    verifyPaintPosition(key1, Offset.zero, false);
    verifyPaintPosition(key2, Offset.zero, true);
    verifyActualBoxPosition(
      tester,
      find.byType(Container),
      0,
      Rect.fromLTWH(0.0, 0.0, 800.0, delegate.maxExtent * 0.5),
    );
    verifyPaintPosition(key3, Offset(0.0, delegate.maxExtent * 0.5), true);

    position.animateTo(
      bigHeight + delegate.maxExtent * 0.9,
      curve: Curves.linear,
      duration: const Duration(minutes: 1),
    );
    await tester.pumpAndSettle(const Duration(milliseconds: 1000));
    verifyPaintPosition(key1, Offset.zero, false);
    verifyPaintPosition(key2, Offset.zero, true);
    verifyActualBoxPosition(
      tester,
      find.byType(Container),
      0,
      Rect.fromLTWH(0.0, -delegate.maxExtent * 0.4, 800.0, delegate.maxExtent * 0.5),
    );
    verifyPaintPosition(key3, Offset(0.0, delegate.maxExtent * 0.1), true);

    position.animateTo(
      bigHeight + delegate.maxExtent * 2.0,
      curve: Curves.linear,
      duration: const Duration(minutes: 1),
    );
    await tester.pumpAndSettle(const Duration(milliseconds: 1000));
    verifyPaintPosition(key1, Offset.zero, false);
    verifyPaintPosition(key2, Offset.zero, false);
    verifyPaintPosition(key3, Offset.zero, true);
  });

  testWidgets('Sliver appbars - floating - no floating behavior when animating', (
    WidgetTester tester,
  ) async {
    final TestDelegate delegate = TestDelegate();
    const double bigHeight = 1000.0;
    GlobalKey key1, key2, key3;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[
            BigSliver(key: key1 = GlobalKey(), height: bigHeight),
            SliverPersistentHeader(key: key2 = GlobalKey(), delegate: delegate, floating: true),
            BigSliver(key: key3 = GlobalKey(), height: bigHeight),
          ],
        ),
      ),
    );
    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;

    verifyPaintPosition(key1, Offset.zero, true);
    verifyPaintPosition(key2, const Offset(0.0, 1000.0), false);
    verifyPaintPosition(key3, const Offset(0.0, 1200.0), false);

    position.animateTo(
      bigHeight + delegate.maxExtent * 2.0,
      curve: Curves.linear,
      duration: const Duration(minutes: 1),
    );
    await tester.pumpAndSettle(const Duration(milliseconds: 1000));
    verifyPaintPosition(key1, Offset.zero, false);
    verifyPaintPosition(key2, Offset.zero, false);
    verifyPaintPosition(key3, Offset.zero, true);

    position.animateTo(
      bigHeight + delegate.maxExtent * 1.9,
      curve: Curves.linear,
      duration: const Duration(minutes: 1),
    );
    await tester.pumpAndSettle(const Duration(milliseconds: 1000));
    verifyPaintPosition(key1, Offset.zero, false);
    verifyPaintPosition(key2, Offset.zero, false);
    verifyPaintPosition(key3, Offset.zero, true);
  });

  testWidgets('Sliver appbars - floating - floating behavior when dragging down', (
    WidgetTester tester,
  ) async {
    final TestDelegate delegate = TestDelegate();
    const double bigHeight = 1000.0;
    GlobalKey key1, key2, key3;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[
            BigSliver(key: key1 = GlobalKey(), height: bigHeight),
            SliverPersistentHeader(key: key2 = GlobalKey(), delegate: delegate, floating: true),
            BigSliver(key: key3 = GlobalKey(), height: bigHeight),
          ],
        ),
      ),
    );
    final ScrollPositionWithSingleContext position =
        tester.state<ScrollableState>(find.byType(Scrollable)).position
            as ScrollPositionWithSingleContext;

    verifyPaintPosition(key1, Offset.zero, true);
    verifyPaintPosition(key2, const Offset(0.0, 1000.0), false);
    verifyPaintPosition(key3, const Offset(0.0, 1200.0), false);

    position.animateTo(
      bigHeight + delegate.maxExtent * 2.0,
      curve: Curves.linear,
      duration: const Duration(minutes: 1),
    );
    await tester.pumpAndSettle(const Duration(milliseconds: 1000));
    verifyPaintPosition(key1, Offset.zero, false);
    verifyPaintPosition(key2, Offset.zero, false);
    verifyPaintPosition(key3, Offset.zero, true);

    position.animateTo(
      bigHeight + delegate.maxExtent * 1.9,
      curve: Curves.linear,
      duration: const Duration(minutes: 1),
    );
    position.updateUserScrollDirection(ScrollDirection.forward);
    await tester.pumpAndSettle(const Duration(milliseconds: 1000));
    verifyPaintPosition(key1, Offset.zero, false);
    verifyPaintPosition(key2, Offset.zero, true);
    verifyActualBoxPosition(
      tester,
      find.byType(Container),
      0,
      Rect.fromLTWH(0.0, -delegate.maxExtent * 0.4, 800.0, delegate.maxExtent * 0.5),
    );
    verifyPaintPosition(key3, Offset.zero, true);
  });

  testWidgets('Sliver appbars - floating - overscroll gap is below header', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          physics: const BouncingScrollPhysics(),
          slivers: <Widget>[
            SliverPersistentHeader(delegate: TestDelegate(), floating: true),
            SliverList(
              delegate: SliverChildListDelegate(<Widget>[
                const SizedBox(height: 300.0, child: Text('X')),
              ]),
            ),
          ],
        ),
      ),
    );

    expect(tester.getTopLeft(find.byType(Container)), Offset.zero);
    expect(tester.getTopLeft(find.text('X')), const Offset(0.0, 200.0));

    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;
    position.jumpTo(-50.0);
    await tester.pump();

    expect(tester.getTopLeft(find.byType(Container)), Offset.zero);
    expect(tester.getTopLeft(find.text('X')), const Offset(0.0, 250.0));
  });

  group('Pointer scrolled floating', () {
    Widget buildTest(Widget sliver) {
      return MaterialApp(
        home: CustomScrollView(
          slivers: <Widget>[
            sliver,
            SliverFixedExtentList(
              itemExtent: 50.0,
              delegate: SliverChildBuilderDelegate(
                (BuildContext context, int index) => Text('Item $index'),
                childCount: 30,
              ),
            ),
          ],
        ),
      );
    }

    void verifyGeometry({
      required GlobalKey key,
      required bool visible,
      required double paintExtent,
    }) {
      final RenderSliver target = key.currentContext!.findRenderObject()! as RenderSliver;
      final SliverGeometry geometry = target.geometry!;
      expect(geometry.visible, visible);
      expect(geometry.paintExtent, paintExtent);
    }

    testWidgets('SliverAppBar', (WidgetTester tester) async {
      final GlobalKey appBarKey = GlobalKey();
      await tester.pumpWidget(
        buildTest(SliverAppBar(key: appBarKey, floating: true, title: const Text('Test Title'))),
      );

      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, visible: true, paintExtent: 56.0);

      // Pointer scroll the app bar away, we will scroll back less to validate the
      // app bar floats back in.
      final Offset point1 = tester.getCenter(find.text('Item 5'));
      final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
      testPointer.hover(point1);
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 300.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsNothing);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: appBarKey, paintExtent: 0.0, visible: false);

      // Scroll back to float in appbar
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -50.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 50.0, visible: true);

      // Float the rest of the way in.
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -250.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 56.0, visible: true);
    });

    testWidgets('SliverPersistentHeader', (WidgetTester tester) async {
      final GlobalKey headerKey = GlobalKey();
      await tester.pumpWidget(
        buildTest(
          SliverPersistentHeader(key: headerKey, floating: true, delegate: HeaderDelegate()),
        ),
      );

      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: headerKey, visible: true, paintExtent: 56.0);

      // Pointer scroll the app bar away, we will scroll back less to validate the
      // app bar floats back in.
      final Offset point1 = tester.getCenter(find.text('Item 5'));
      final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
      testPointer.hover(point1);
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 300.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsNothing);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: headerKey, paintExtent: 0.0, visible: false);

      // Scroll back to float in appbar
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -50.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: headerKey, paintExtent: 50.0, visible: true);

      // Float the rest of the way in.
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -250.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: headerKey, paintExtent: 56.0, visible: true);
    });

    testWidgets('and snapping SliverAppBar', (WidgetTester tester) async {
      final GlobalKey appBarKey = GlobalKey();
      await tester.pumpWidget(
        buildTest(
          SliverAppBar(key: appBarKey, floating: true, snap: true, title: const Text('Test Title')),
        ),
      );

      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, visible: true, paintExtent: 56.0);

      // Pointer scroll the app bar away, we will scroll back less to validate the
      // app bar floats back in and then snaps to full size.
      final Offset point1 = tester.getCenter(find.text('Item 5'));
      final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
      testPointer.hover(point1);
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 300.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsNothing);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: appBarKey, paintExtent: 0.0, visible: false);

      // Scroll back to float in appbar
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -30.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 30.0, visible: true);
      await tester.pumpAndSettle();
      // The snap animation should have completed and the app bar should be
      // fully expanded.
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 56.0, visible: true);

      // Float back out a bit and trigger snap close animation.
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 50.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 6.0, visible: true);
      await tester.pumpAndSettle();
      // The snap animation should have completed and the app bar should no
      // longer be visible.
      expect(find.text('Test Title'), findsNothing);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(find.byType(AppBar), findsNothing);
      verifyGeometry(key: appBarKey, paintExtent: 0.0, visible: false);
    });
  });
}

class HeaderDelegate extends SliverPersistentHeaderDelegate {
  @override
  Widget build(BuildContext context, double shrinkOffset, bool overlapsContent) {
    return Container(height: 56, color: Colors.red, child: const Text('Test Title'));
  }

  @override
  double get maxExtent => 56;

  @override
  double get minExtent => 56;

  @override
  bool shouldRebuild(SliverPersistentHeaderDelegate oldDelegate) => false;
}

class TestDelegate extends SliverPersistentHeaderDelegate {
  @override
  double get maxExtent => 200.0;

  @override
  double get minExtent => 100.0;

  @override
  Widget build(BuildContext context, double shrinkOffset, bool overlapsContent) {
    return Container(constraints: BoxConstraints(minHeight: minExtent, maxHeight: maxExtent));
  }

  @override
  bool shouldRebuild(TestDelegate oldDelegate) => false;
}

class RenderBigSliver extends RenderSliver {
  RenderBigSliver(double height) : _height = height;

  double get height => _height;
  double _height;
  set height(double value) {
    if (value == _height) {
      return;
    }
    _height = value;
    markNeedsLayout();
  }

  double get paintExtent =>
      (height - constraints.scrollOffset).clamp(0.0, constraints.remainingPaintExtent);

  @override
  void performLayout() {
    geometry = SliverGeometry(
      scrollExtent: height,
      paintExtent: paintExtent,
      maxPaintExtent: height,
    );
  }
}

class BigSliver extends LeafRenderObjectWidget {
  const BigSliver({super.key, required this.height});

  final double height;

  @override
  RenderBigSliver createRenderObject(BuildContext context) {
    return RenderBigSliver(height);
  }

  @override
  void updateRenderObject(BuildContext context, RenderBigSliver renderObject) {
    renderObject.height = height;
  }
}
