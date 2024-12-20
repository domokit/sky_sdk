// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
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
  testWidgets('Sliver appbars - pinned', (WidgetTester tester) async {
    const double bigHeight = 550.0;
    GlobalKey key1, key2, key3, key4, key5;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[
            BigSliver(key: key1 = GlobalKey(), height: bigHeight),
            SliverPersistentHeader(key: key2 = GlobalKey(), delegate: TestDelegate(), pinned: true),
            SliverPersistentHeader(key: key3 = GlobalKey(), delegate: TestDelegate(), pinned: true),
            BigSliver(key: key4 = GlobalKey(), height: bigHeight),
            BigSliver(key: key5 = GlobalKey(), height: bigHeight),
          ],
        ),
      ),
    );
    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;
    final double max =
        bigHeight * 3.0 +
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
    verifyPaintPosition(key1, Offset.zero, false);
    verifyPaintPosition(key2, Offset.zero, true);
    verifyPaintPosition(key3, const Offset(0.0, 100.0), true);
    verifyPaintPosition(key4, Offset.zero, true);
    verifyPaintPosition(key5, const Offset(0.0, 50.0), true);
  });

  testWidgets('Sliver appbars - toStringDeep of maxExtent that throws', (
    WidgetTester tester,
  ) async {
    final TestDelegateThatCanThrow delegateThatCanThrow = TestDelegateThatCanThrow();
    GlobalKey key;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[
            SliverPersistentHeader(
              key: key = GlobalKey(),
              delegate: delegateThatCanThrow,
              pinned: true,
            ),
          ],
        ),
      ),
    );
    await tester.pumpAndSettle(const Duration(milliseconds: 10));

    final RenderObject renderObject = key.currentContext!.findRenderObject()!;
    // The delegate must only start throwing immediately before calling
    // toStringDeep to avoid triggering spurious exceptions.
    // If the _RenderSliverPinnedPersistentHeaderForWidgets class was not
    // private it would make more sense to create an instance of it directly.
    delegateThatCanThrow.shouldThrow = true;
    expect(renderObject, hasAGoodToStringDeep);
    expect(
      renderObject.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        '_RenderSliverPinnedPersistentHeaderForWidgets#00000 relayoutBoundary=up1\n'
        ' │ parentData: paintOffset=Offset(0.0, 0.0) (can use size)\n'
        ' │ constraints: SliverConstraints(AxisDirection.down,\n'
        ' │   GrowthDirection.forward, ScrollDirection.idle, scrollOffset:\n'
        ' │   0.0, precedingScrollExtent: 0.0, remainingPaintExtent: 600.0,\n'
        ' │   crossAxisExtent: 800.0, crossAxisDirection:\n'
        ' │   AxisDirection.right, viewportMainAxisExtent: 600.0,\n'
        ' │   remainingCacheExtent: 850.0, cacheOrigin: 0.0)\n'
        ' │ geometry: SliverGeometry(scrollExtent: 200.0, paintExtent: 200.0,\n'
        ' │   maxPaintExtent: 200.0, hasVisualOverflow: true, cacheExtent:\n'
        ' │   200.0)\n'
        ' │ maxExtent: EXCEPTION (FlutterError)\n'
        ' │ child position: 0.0\n'
        ' │\n'
        ' └─child: RenderConstrainedBox#00000 relayoutBoundary=up2\n'
        '   │ parentData: <none> (can use size)\n'
        '   │ constraints: BoxConstraints(w=800.0, 0.0<=h<=200.0)\n'
        '   │ size: Size(800.0, 200.0)\n'
        '   │ additionalConstraints: BoxConstraints(0.0<=w<=Infinity,\n'
        '   │   100.0<=h<=200.0)\n'
        '   │\n'
        '   └─child: RenderLimitedBox#00000 relayoutBoundary=up3\n'
        '     │ parentData: <none> (can use size)\n'
        '     │ constraints: BoxConstraints(w=800.0, 100.0<=h<=200.0)\n'
        '     │ size: Size(800.0, 200.0)\n'
        '     │ maxWidth: 0.0\n'
        '     │ maxHeight: 0.0\n'
        '     │\n'
        '     └─child: RenderConstrainedBox#00000 relayoutBoundary=up4\n'
        '         parentData: <none> (can use size)\n'
        '         constraints: BoxConstraints(w=800.0, 100.0<=h<=200.0)\n'
        '         size: Size(800.0, 200.0)\n'
        '         additionalConstraints: BoxConstraints(biggest)\n',
      ),
    );
  });

  testWidgets('Sliver appbars - pinned with slow scroll', (WidgetTester tester) async {
    const double bigHeight = 550.0;
    GlobalKey key1, key2, key3, key4, key5;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[
            BigSliver(key: key1 = GlobalKey(), height: bigHeight),
            SliverPersistentHeader(key: key2 = GlobalKey(), delegate: TestDelegate(), pinned: true),
            SliverPersistentHeader(key: key3 = GlobalKey(), delegate: TestDelegate(), pinned: true),
            BigSliver(key: key4 = GlobalKey(), height: bigHeight),
            BigSliver(key: key5 = GlobalKey(), height: bigHeight),
          ],
        ),
      ),
    );

    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;
    verifyPaintPosition(key1, Offset.zero, true);
    verifyPaintPosition(key2, const Offset(0.0, 550.0), true);
    verifyPaintPosition(key3, const Offset(0.0, 750.0), false);
    verifyPaintPosition(key4, const Offset(0.0, 950.0), false);
    verifyPaintPosition(key5, const Offset(0.0, 1500.0), false);
    position.animateTo(550.0, curve: Curves.linear, duration: const Duration(minutes: 1));
    await tester.pumpAndSettle();
    verifyPaintPosition(key1, Offset.zero, false);
    verifyPaintPosition(key2, Offset.zero, true);
    verifyPaintPosition(key3, const Offset(0.0, 200.0), true);
    verifyPaintPosition(key4, const Offset(0.0, 400.0), true);
    verifyPaintPosition(key5, const Offset(0.0, 950.0), false);
    position.animateTo(600.0, curve: Curves.linear, duration: const Duration(minutes: 1));
    await tester.pumpAndSettle(const Duration(milliseconds: 200));
    verifyPaintPosition(key1, Offset.zero, false);
    verifyPaintPosition(key2, Offset.zero, true);
    verifyPaintPosition(key3, const Offset(0.0, 150.0), true);
    verifyPaintPosition(key4, const Offset(0.0, 350.0), true);
    verifyPaintPosition(key5, const Offset(0.0, 900.0), false);
    position.animateTo(650.0, curve: Curves.linear, duration: const Duration(minutes: 1));
    await tester.pumpAndSettle(const Duration(milliseconds: 300));
    verifyPaintPosition(key1, Offset.zero, false);
    verifyPaintPosition(key2, Offset.zero, true);
    verifyPaintPosition(key3, const Offset(0.0, 100.0), true);
    verifyActualBoxPosition(
      tester,
      find.byType(Container),
      1,
      const Rect.fromLTWH(0.0, 100.0, 800.0, 200.0),
    );
    verifyPaintPosition(key4, const Offset(0.0, 300.0), true);
    verifyPaintPosition(key5, const Offset(0.0, 850.0), false);
    position.animateTo(700.0, curve: Curves.linear, duration: const Duration(minutes: 1));
    await tester.pumpAndSettle(const Duration(milliseconds: 400));
    verifyPaintPosition(key1, Offset.zero, false);
    verifyPaintPosition(key2, Offset.zero, true);
    verifyPaintPosition(key3, const Offset(0.0, 100.0), true);
    verifyActualBoxPosition(
      tester,
      find.byType(Container),
      1,
      const Rect.fromLTWH(0.0, 100.0, 800.0, 200.0),
    );
    verifyPaintPosition(key4, const Offset(0.0, 250.0), true);
    verifyPaintPosition(key5, const Offset(0.0, 800.0), false);
    position.animateTo(750.0, curve: Curves.linear, duration: const Duration(minutes: 1));
    await tester.pumpAndSettle(const Duration(milliseconds: 500));
    verifyPaintPosition(key1, Offset.zero, false);
    verifyPaintPosition(key2, Offset.zero, true);
    verifyPaintPosition(key3, const Offset(0.0, 100.0), true);
    verifyActualBoxPosition(
      tester,
      find.byType(Container),
      1,
      const Rect.fromLTWH(0.0, 100.0, 800.0, 200.0),
    );
    verifyPaintPosition(key4, const Offset(0.0, 200.0), true);
    verifyPaintPosition(key5, const Offset(0.0, 750.0), false);
    position.animateTo(800.0, curve: Curves.linear, duration: const Duration(minutes: 1));
    await tester.pumpAndSettle(const Duration(milliseconds: 60));
    verifyPaintPosition(key1, Offset.zero, false);
    verifyPaintPosition(key2, Offset.zero, true);
    verifyPaintPosition(key3, const Offset(0.0, 100.0), true);
    verifyPaintPosition(key4, const Offset(0.0, 150.0), true);
    verifyPaintPosition(key5, const Offset(0.0, 700.0), false);
    position.animateTo(850.0, curve: Curves.linear, duration: const Duration(minutes: 1));
    await tester.pumpAndSettle(const Duration(milliseconds: 70));
    verifyPaintPosition(key1, Offset.zero, false);
    verifyPaintPosition(key2, Offset.zero, true);
    verifyPaintPosition(key3, const Offset(0.0, 100.0), true);
    verifyPaintPosition(key4, const Offset(0.0, 100.0), true);
    verifyPaintPosition(key5, const Offset(0.0, 650.0), false);
    position.animateTo(900.0, curve: Curves.linear, duration: const Duration(minutes: 1));
    await tester.pumpAndSettle(const Duration(milliseconds: 80));
    verifyPaintPosition(key1, Offset.zero, false);
    verifyPaintPosition(key2, Offset.zero, true);
    verifyPaintPosition(key3, const Offset(0.0, 100.0), true);
    verifyPaintPosition(key4, const Offset(0.0, 50.0), true);
    verifyPaintPosition(key5, const Offset(0.0, 600.0), false);
    position.animateTo(950.0, curve: Curves.linear, duration: const Duration(minutes: 1));
    await tester.pumpAndSettle(const Duration(milliseconds: 90));
    verifyPaintPosition(key1, Offset.zero, false);
    verifyPaintPosition(key2, Offset.zero, true);
    verifyPaintPosition(key3, const Offset(0.0, 100.0), true);
    verifyActualBoxPosition(
      tester,
      find.byType(Container),
      1,
      const Rect.fromLTWH(0.0, 100.0, 800.0, 100.0),
    );
    verifyPaintPosition(key4, Offset.zero, true);
    verifyPaintPosition(key5, const Offset(0.0, 550.0), true);
  });

  testWidgets('Sliver appbars - pinned with less overlap', (WidgetTester tester) async {
    const double bigHeight = 650.0;
    GlobalKey key1, key2, key3, key4, key5;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[
            BigSliver(key: key1 = GlobalKey(), height: bigHeight),
            SliverPersistentHeader(key: key2 = GlobalKey(), delegate: TestDelegate(), pinned: true),
            SliverPersistentHeader(key: key3 = GlobalKey(), delegate: TestDelegate(), pinned: true),
            BigSliver(key: key4 = GlobalKey(), height: bigHeight),
            BigSliver(key: key5 = GlobalKey(), height: bigHeight),
          ],
        ),
      ),
    );
    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;
    final double max =
        bigHeight * 3.0 +
        TestDelegate().maxExtent * 2.0 -
        600.0; // 600 is the height of the test viewport
    assert(max < 10000.0);
    expect(max, 1750.0);
    expect(position.pixels, 0.0);
    expect(position.minScrollExtent, 0.0);
    expect(position.maxScrollExtent, max);
    position.animateTo(10000.0, curve: Curves.linear, duration: const Duration(minutes: 1));
    await tester.pumpAndSettle(const Duration(milliseconds: 10));
    expect(position.pixels, max);
    expect(position.minScrollExtent, 0.0);
    expect(position.maxScrollExtent, max);
    verifyPaintPosition(key1, Offset.zero, false);
    verifyPaintPosition(key2, Offset.zero, true);
    verifyPaintPosition(key3, const Offset(0.0, 100.0), true);
    verifyPaintPosition(key4, Offset.zero, false);
    verifyPaintPosition(key5, Offset.zero, true);
  });

  testWidgets('Sliver appbars - overscroll gap is below header', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          physics: const BouncingScrollPhysics(),
          slivers: <Widget>[
            SliverPersistentHeader(delegate: TestDelegate(), pinned: true),
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

    position.jumpTo(50.0);
    await tester.pump();

    expect(tester.getTopLeft(find.byType(Container)), Offset.zero);
    expect(tester.getTopLeft(find.text('X')), const Offset(0.0, 150.0));

    position.jumpTo(150.0);
    await tester.pump();

    expect(tester.getTopLeft(find.byType(Container)), Offset.zero);
    expect(tester.getTopLeft(find.text('X')), const Offset(0.0, 50.0));
  });
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

class TestDelegateThatCanThrow extends SliverPersistentHeaderDelegate {
  bool shouldThrow = false;

  @override
  double get maxExtent {
    return shouldThrow ? throw FlutterError('Unavailable maxExtent') : 200.0;
  }

  @override
  double get minExtent {
    return shouldThrow ? throw FlutterError('Unavailable minExtent') : 100.0;
  }

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
