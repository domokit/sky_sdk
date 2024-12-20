// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void verifyPaintPosition(GlobalKey key, Offset ideal) {
  final RenderObject target = key.currentContext!.findRenderObject()!;
  expect(target.parent, isA<RenderViewport>());
  final SliverPhysicalParentData parentData = target.parentData! as SliverPhysicalParentData;
  final Offset actual = parentData.paintOffset;
  expect(actual, ideal);
}

void main() {
  testWidgets('Sliver appbars - scrolling', (WidgetTester tester) async {
    GlobalKey key1, key2, key3, key4, key5;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[
            BigSliver(key: key1 = GlobalKey()),
            SliverPersistentHeader(key: key2 = GlobalKey(), delegate: TestDelegate()),
            SliverPersistentHeader(key: key3 = GlobalKey(), delegate: TestDelegate()),
            BigSliver(key: key4 = GlobalKey()),
            BigSliver(key: key5 = GlobalKey()),
          ],
        ),
      ),
    );
    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;
    final double max =
        RenderBigSliver.height * 3.0 +
        TestDelegate().maxExtent * 2.0 -
        600.0; // 600 is the height of the test viewport
    assert(max < 10000.0);
    expect(max, 1450.0);
    expect(position.pixels, 0.0);
    expect(position.minScrollExtent, 0.0);
    expect(position.maxScrollExtent, max);
    position.animateTo(10000.0, curve: Curves.linear, duration: const Duration(minutes: 1));
    await tester.pumpAndSettle(const Duration(milliseconds: 10));
    expect(position.pixels, max);
    expect(position.minScrollExtent, 0.0);
    expect(position.maxScrollExtent, max);
    verifyPaintPosition(key1, Offset.zero);
    verifyPaintPosition(key2, Offset.zero);
    verifyPaintPosition(key3, Offset.zero);
    verifyPaintPosition(key4, Offset.zero);
    verifyPaintPosition(key5, const Offset(0.0, 50.0));
  });

  testWidgets('Sliver appbars - scrolling off screen', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();
    final TestDelegate delegate = TestDelegate();
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[
            const BigSliver(),
            SliverPersistentHeader(key: key, delegate: delegate),
            const BigSliver(),
            const BigSliver(),
          ],
        ),
      ),
    );
    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;
    position.animateTo(
      RenderBigSliver.height + delegate.maxExtent - 5.0,
      curve: Curves.linear,
      duration: const Duration(minutes: 1),
    );
    await tester.pumpAndSettle(const Duration(milliseconds: 1000));
    final RenderBox box = tester.renderObject<RenderBox>(find.text('Sliver App Bar'));
    final Rect rect = Rect.fromPoints(
      box.localToGlobal(Offset.zero),
      box.localToGlobal(box.size.bottomRight(Offset.zero)),
    );
    expect(rect, equals(const Rect.fromLTWH(0.0, -195.0, 800.0, 200.0)));
  });

  testWidgets('Sliver appbars - scrolling - overscroll gap is below header', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          physics: const BouncingScrollPhysics(),
          slivers: <Widget>[
            SliverPersistentHeader(delegate: TestDelegate()),
            SliverList(
              delegate: SliverChildListDelegate(<Widget>[
                const SizedBox(height: 300.0, child: Text('X')),
              ]),
            ),
          ],
        ),
      ),
    );

    expect(tester.getTopLeft(find.text('Sliver App Bar')), Offset.zero);
    expect(tester.getTopLeft(find.text('X')), const Offset(0.0, 200.0));

    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;
    position.jumpTo(-50.0);
    await tester.pump();

    expect(tester.getTopLeft(find.text('Sliver App Bar')), Offset.zero);
    expect(tester.getTopLeft(find.text('X')), const Offset(0.0, 250.0));
  });

  testWidgets('Sliver appbars const child delegate - scrolling - overscroll gap is below header', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          physics: const BouncingScrollPhysics(),
          slivers: <Widget>[
            SliverPersistentHeader(delegate: TestDelegate()),
            const SliverList(
              delegate: SliverChildListDelegate.fixed(<Widget>[
                SizedBox(height: 300.0, child: Text('X')),
              ]),
            ),
          ],
        ),
      ),
    );

    expect(tester.getTopLeft(find.text('Sliver App Bar')), Offset.zero);
    expect(tester.getTopLeft(find.text('X')), const Offset(0.0, 200.0));

    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;
    position.jumpTo(-50.0);
    await tester.pump();

    expect(tester.getTopLeft(find.text('Sliver App Bar')), Offset.zero);
    expect(tester.getTopLeft(find.text('X')), const Offset(0.0, 250.0));
  });

  group('has correct semantics when', () {
    testWidgets('within viewport', (WidgetTester tester) async {
      const double cacheExtent = 250;
      final SemanticsHandle handle = tester.ensureSemantics();

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: CustomScrollView(
            cacheExtent: cacheExtent,
            physics: const BouncingScrollPhysics(),
            slivers: <Widget>[
              SliverPersistentHeader(delegate: TestDelegate()),
              const SliverList(
                delegate: SliverChildListDelegate.fixed(<Widget>[
                  SizedBox(height: 300.0, child: Text('X')),
                ]),
              ),
            ],
          ),
        ),
      );

      final SemanticsFinder sliverAppBar = find.semantics.byLabel('Sliver App Bar');

      expect(sliverAppBar, findsOne);
      expect(sliverAppBar, containsSemantics(isHidden: false));
      handle.dispose();
    });

    testWidgets('partially scrolling off screen', (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      final TestDelegate delegate = TestDelegate();
      final SemanticsHandle handle = tester.ensureSemantics();
      const double cacheExtent = 250;
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: CustomScrollView(
            cacheExtent: cacheExtent,
            slivers: <Widget>[
              SliverPersistentHeader(key: key, delegate: delegate),
              const BigSliver(),
              const BigSliver(),
            ],
          ),
        ),
      );
      final ScrollPosition position =
          tester.state<ScrollableState>(find.byType(Scrollable)).position;
      position.animateTo(
        delegate.maxExtent - 20.0,
        curve: Curves.linear,
        duration: const Duration(minutes: 1),
      );
      await tester.pumpAndSettle(const Duration(milliseconds: 1000));
      final RenderBox box = tester.renderObject<RenderBox>(find.text('Sliver App Bar'));
      final Rect rect = Rect.fromPoints(
        box.localToGlobal(Offset.zero),
        box.localToGlobal(box.size.bottomRight(Offset.zero)),
      );
      expect(rect, equals(const Rect.fromLTWH(0.0, -180.0, 800.0, 200.0)));

      final SemanticsFinder sliverAppBar = find.semantics.byLabel('Sliver App Bar');

      expect(sliverAppBar, findsOne);
      expect(sliverAppBar, containsSemantics(isHidden: false));
      handle.dispose();
    });

    testWidgets('completely scrolling off screen but within cache extent', (
      WidgetTester tester,
    ) async {
      final GlobalKey key = GlobalKey();
      final TestDelegate delegate = TestDelegate();
      final SemanticsHandle handle = tester.ensureSemantics();
      const double cacheExtent = 250;
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: CustomScrollView(
            cacheExtent: cacheExtent,
            slivers: <Widget>[
              SliverPersistentHeader(key: key, delegate: delegate),
              const BigSliver(),
              const BigSliver(),
            ],
          ),
        ),
      );
      final ScrollPosition position =
          tester.state<ScrollableState>(find.byType(Scrollable)).position;
      position.animateTo(
        delegate.maxExtent + 20.0,
        curve: Curves.linear,
        duration: const Duration(minutes: 1),
      );
      await tester.pumpAndSettle(const Duration(milliseconds: 1000));

      final SemanticsFinder sliverAppBar = find.semantics.byLabel('Sliver App Bar');

      expect(sliverAppBar, findsOne);
      expect(sliverAppBar, containsSemantics(isHidden: true));
      handle.dispose();
    });

    testWidgets('completely scrolling off screen and not within cache extent', (
      WidgetTester tester,
    ) async {
      final GlobalKey key = GlobalKey();
      final TestDelegate delegate = TestDelegate();
      final SemanticsHandle handle = tester.ensureSemantics();
      const double cacheExtent = 250;
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: CustomScrollView(
            cacheExtent: cacheExtent,
            slivers: <Widget>[
              SliverPersistentHeader(key: key, delegate: delegate),
              const BigSliver(),
              const BigSliver(),
            ],
          ),
        ),
      );
      final ScrollPosition position =
          tester.state<ScrollableState>(find.byType(Scrollable)).position;
      position.animateTo(
        delegate.maxExtent + 300.0,
        curve: Curves.linear,
        duration: const Duration(minutes: 1),
      );
      await tester.pumpAndSettle(const Duration(milliseconds: 1000));

      final SemanticsFinder sliverAppBar = find.semantics.byLabel('Sliver App Bar');

      expect(sliverAppBar, findsNothing);
      handle.dispose();
    });
  });
}

class TestDelegate extends SliverPersistentHeaderDelegate {
  @override
  double get maxExtent => 200.0;

  @override
  double get minExtent => 200.0;

  @override
  Widget build(BuildContext context, double shrinkOffset, bool overlapsContent) {
    return SizedBox(height: maxExtent, child: const Text('Sliver App Bar'));
  }

  @override
  bool shouldRebuild(TestDelegate oldDelegate) => false;
}

class RenderBigSliver extends RenderSliver {
  static const double height = 550.0;
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
  const BigSliver({super.key});
  @override
  RenderBigSliver createRenderObject(BuildContext context) {
    return RenderBigSliver();
  }
}
