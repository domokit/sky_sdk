// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is separate from viewport_caching_test.dart because we can't use
// both testWidgets and rendering_tester in the same file - testWidgets will
// initialize a binding, which rendering_tester will attempt to re-initialize
// (or vice versa).

@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

class _TestSliverPersistentHeaderDelegate extends SliverPersistentHeaderDelegate {
  _TestSliverPersistentHeaderDelegate({
    this.key,
    required this.minExtent,
    required this.maxExtent,
    this.vsync = const TestVSync(),
    this.showOnScreenConfiguration = const PersistentHeaderShowOnScreenConfiguration(),
  });

  final Key? key;

  @override
  final double maxExtent;

  @override
  final double minExtent;

  @override
  final TickerProvider? vsync;

  @override
  final PersistentHeaderShowOnScreenConfiguration showOnScreenConfiguration;

  @override
  Widget build(BuildContext context, double shrinkOffset, bool overlapsContent) =>
      SizedBox.expand(key: key);

  @override
  bool shouldRebuild(_TestSliverPersistentHeaderDelegate oldDelegate) => true;
}

void main() {
  testWidgets('Scrollable widget scrollDirection update test', (WidgetTester tester) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);
    Widget buildFrame(Axis axis) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 100.0,
            width: 100.0,
            child: SingleChildScrollView(
              controller: controller,
              scrollDirection: axis,
              child: const SizedBox(width: 200, height: 200, child: SizedBox.shrink()),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(Axis.vertical));
    expect(controller.position.pixels, 0.0);

    // Change the SingleChildScrollView.scrollDirection to horizontal.
    await tester.pumpWidget(buildFrame(Axis.horizontal));
    expect(controller.position.pixels, 0.0);

    final TestGesture gesture = await tester.startGesture(const Offset(400.0, 300.0));
    // Drag in the vertical direction should not cause scrolling.
    await gesture.moveBy(const Offset(0.0, 10.0));
    expect(controller.position.pixels, 0.0);
    await gesture.moveBy(const Offset(0.0, -10.0));
    expect(controller.position.pixels, 0.0);

    // Drag in the horizontal direction should cause scrolling.
    await gesture.moveBy(const Offset(-10.0, 0.0));
    expect(controller.position.pixels, 10.0);
    await gesture.moveBy(const Offset(10.0, 0.0));
    expect(controller.position.pixels, 0.0);
  });

  testWidgets('Viewport getOffsetToReveal - down', (WidgetTester tester) async {
    final ScrollController controller = ScrollController(initialScrollOffset: 300.0);
    addTearDown(controller.dispose);
    List<Widget> children;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 200.0,
            width: 300.0,
            child: ListView(
              controller: controller,
              children:
                  children = List<Widget>.generate(20, (int i) {
                    return SizedBox(height: 100.0, width: 300.0, child: Text('Tile $i'));
                  }),
            ),
          ),
        ),
      ),
    );

    final RenderAbstractViewport viewport =
        tester.allRenderObjects.whereType<RenderAbstractViewport>().first;

    final RenderObject target = tester.renderObject(
      find.byWidget(children[5], skipOffstage: false),
    );
    RevealedOffset revealed = viewport.getOffsetToReveal(target, 0.0);
    expect(revealed.offset, 500.0);
    expect(revealed.rect, const Rect.fromLTWH(0.0, 0.0, 300.0, 100.0));

    revealed = viewport.getOffsetToReveal(target, 1.0);
    expect(revealed.offset, 400.0);
    expect(revealed.rect, const Rect.fromLTWH(0.0, 100.0, 300.0, 100.0));

    revealed = viewport.getOffsetToReveal(
      target,
      0.0,
      rect: const Rect.fromLTWH(40.0, 40.0, 10.0, 10.0),
    );
    expect(revealed.offset, 540.0);
    expect(revealed.rect, const Rect.fromLTWH(40.0, 0.0, 10.0, 10.0));

    revealed = viewport.getOffsetToReveal(
      target,
      1.0,
      rect: const Rect.fromLTWH(40.0, 40.0, 10.0, 10.0),
    );
    expect(revealed.offset, 350.0);
    expect(revealed.rect, const Rect.fromLTWH(40.0, 190.0, 10.0, 10.0));
  });

  testWidgets('Viewport getOffsetToReveal - right', (WidgetTester tester) async {
    final ScrollController controller = ScrollController(initialScrollOffset: 300.0);
    addTearDown(controller.dispose);
    List<Widget> children;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 300.0,
            width: 200.0,
            child: ListView(
              scrollDirection: Axis.horizontal,
              controller: controller,
              children:
                  children = List<Widget>.generate(20, (int i) {
                    return SizedBox(height: 300.0, width: 100.0, child: Text('Tile $i'));
                  }),
            ),
          ),
        ),
      ),
    );

    final RenderAbstractViewport viewport =
        tester.allRenderObjects.whereType<RenderAbstractViewport>().first;

    final RenderObject target = tester.renderObject(
      find.byWidget(children[5], skipOffstage: false),
    );
    RevealedOffset revealed = viewport.getOffsetToReveal(target, 0.0);
    expect(revealed.offset, 500.0);
    expect(revealed.rect, const Rect.fromLTWH(0.0, 0.0, 100.0, 300.0));

    revealed = viewport.getOffsetToReveal(target, 1.0);
    expect(revealed.offset, 400.0);
    expect(revealed.rect, const Rect.fromLTWH(100.0, 0.0, 100.0, 300.0));

    revealed = viewport.getOffsetToReveal(
      target,
      0.0,
      rect: const Rect.fromLTWH(40.0, 40.0, 10.0, 10.0),
    );
    expect(revealed.offset, 540.0);
    expect(revealed.rect, const Rect.fromLTWH(0.0, 40.0, 10.0, 10.0));

    revealed = viewport.getOffsetToReveal(
      target,
      1.0,
      rect: const Rect.fromLTWH(40.0, 40.0, 10.0, 10.0),
    );
    expect(revealed.offset, 350.0);
    expect(revealed.rect, const Rect.fromLTWH(190.0, 40.0, 10.0, 10.0));
  });

  testWidgets('Viewport getOffsetToReveal - up', (WidgetTester tester) async {
    final ScrollController controller = ScrollController(initialScrollOffset: 300.0);
    addTearDown(controller.dispose);
    List<Widget> children;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 200.0,
            width: 300.0,
            child: ListView(
              controller: controller,
              reverse: true,
              children:
                  children = List<Widget>.generate(20, (int i) {
                    return SizedBox(height: 100.0, width: 300.0, child: Text('Tile $i'));
                  }),
            ),
          ),
        ),
      ),
    );

    final RenderAbstractViewport viewport =
        tester.allRenderObjects.whereType<RenderAbstractViewport>().first;

    final RenderObject target = tester.renderObject(
      find.byWidget(children[5], skipOffstage: false),
    );
    RevealedOffset revealed = viewport.getOffsetToReveal(target, 0.0);
    expect(revealed.offset, 500.0);
    expect(revealed.rect, const Rect.fromLTWH(0.0, 100.0, 300.0, 100.0));

    revealed = viewport.getOffsetToReveal(target, 1.0);
    expect(revealed.offset, 400.0);
    expect(revealed.rect, const Rect.fromLTWH(0.0, 0.0, 300.0, 100.0));

    revealed = viewport.getOffsetToReveal(
      target,
      0.0,
      rect: const Rect.fromLTWH(40.0, 40.0, 10.0, 10.0),
    );
    expect(revealed.offset, 550.0);
    expect(revealed.rect, const Rect.fromLTWH(40.0, 190.0, 10.0, 10.0));

    revealed = viewport.getOffsetToReveal(
      target,
      1.0,
      rect: const Rect.fromLTWH(40.0, 40.0, 10.0, 10.0),
    );
    expect(revealed.offset, 360.0);
    expect(revealed.rect, const Rect.fromLTWH(40.0, 0.0, 10.0, 10.0));
  });

  testWidgets('Viewport getOffsetToReveal - left', (WidgetTester tester) async {
    final ScrollController controller = ScrollController(initialScrollOffset: 300.0);
    addTearDown(controller.dispose);
    List<Widget> children;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 300.0,
            width: 200.0,
            child: ListView(
              scrollDirection: Axis.horizontal,
              reverse: true,
              controller: controller,
              children:
                  children = List<Widget>.generate(20, (int i) {
                    return SizedBox(height: 300.0, width: 100.0, child: Text('Tile $i'));
                  }),
            ),
          ),
        ),
      ),
    );

    final RenderAbstractViewport viewport =
        tester.allRenderObjects.whereType<RenderAbstractViewport>().first;

    final RenderObject target = tester.renderObject(
      find.byWidget(children[5], skipOffstage: false),
    );
    RevealedOffset revealed = viewport.getOffsetToReveal(target, 0.0);
    expect(revealed.offset, 500.0);
    expect(revealed.rect, const Rect.fromLTWH(100.0, 0.0, 100.0, 300.0));

    revealed = viewport.getOffsetToReveal(target, 1.0);
    expect(revealed.offset, 400.0);
    expect(revealed.rect, const Rect.fromLTWH(0.0, 0.0, 100.0, 300.0));

    revealed = viewport.getOffsetToReveal(
      target,
      0.0,
      rect: const Rect.fromLTWH(40.0, 40.0, 10.0, 10.0),
    );
    expect(revealed.offset, 550.0);
    expect(revealed.rect, const Rect.fromLTWH(190.0, 40.0, 10.0, 10.0));

    revealed = viewport.getOffsetToReveal(
      target,
      1.0,
      rect: const Rect.fromLTWH(40.0, 40.0, 10.0, 10.0),
    );
    expect(revealed.offset, 360.0);
    expect(revealed.rect, const Rect.fromLTWH(0.0, 40.0, 10.0, 10.0));
  });

  testWidgets('Viewport getOffsetToReveal Sliver - down', (WidgetTester tester) async {
    final ScrollController controller = ScrollController(initialScrollOffset: 300.0);
    addTearDown(controller.dispose);
    final List<Widget> children = <Widget>[];
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 200.0,
            width: 300.0,
            child: CustomScrollView(
              controller: controller,
              slivers: List<Widget>.generate(20, (int i) {
                final Widget sliver = SliverToBoxAdapter(
                  child: SizedBox(height: 100.0, child: Text('Tile $i')),
                );
                children.add(sliver);
                return SliverPadding(
                  padding: const EdgeInsets.only(top: 22.0, bottom: 23.0),
                  sliver: sliver,
                );
              }),
            ),
          ),
        ),
      ),
    );

    final RenderAbstractViewport viewport =
        tester.allRenderObjects.whereType<RenderAbstractViewport>().first;

    final RenderObject target = tester.renderObject(
      find.byWidget(children[5], skipOffstage: false),
    );
    RevealedOffset revealed = viewport.getOffsetToReveal(target, 0.0);
    expect(revealed.offset, 5 * (100 + 22 + 23) + 22);

    revealed = viewport.getOffsetToReveal(target, 1.0);
    expect(revealed.offset, 5 * (100 + 22 + 23) + 22 - 100);

    // With rect specified.
    revealed = viewport.getOffsetToReveal(target, 0.0, rect: const Rect.fromLTRB(1, 2, 3, 4));
    expect(revealed.offset, 5 * (100 + 22 + 23) + 22 + 2);
    revealed = viewport.getOffsetToReveal(target, 1.0, rect: const Rect.fromLTRB(1, 2, 3, 4));
    expect(revealed.offset, 5 * (100 + 22 + 23) + 22 - (200 - 4));
  });

  testWidgets('Viewport getOffsetToReveal Sliver - right', (WidgetTester tester) async {
    final ScrollController controller = ScrollController(initialScrollOffset: 300.0);
    addTearDown(controller.dispose);
    final List<Widget> children = <Widget>[];
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 300.0,
            width: 200.0,
            child: CustomScrollView(
              scrollDirection: Axis.horizontal,
              controller: controller,
              slivers: List<Widget>.generate(20, (int i) {
                final Widget sliver = SliverToBoxAdapter(
                  child: SizedBox(width: 100.0, child: Text('Tile $i')),
                );
                children.add(sliver);
                return SliverPadding(
                  padding: const EdgeInsets.only(left: 22.0, right: 23.0),
                  sliver: sliver,
                );
              }),
            ),
          ),
        ),
      ),
    );

    final RenderAbstractViewport viewport =
        tester.allRenderObjects.whereType<RenderAbstractViewport>().first;

    final RenderObject target = tester.renderObject(
      find.byWidget(children[5], skipOffstage: false),
    );
    RevealedOffset revealed = viewport.getOffsetToReveal(target, 0.0);
    expect(revealed.offset, 5 * (100 + 22 + 23) + 22);

    revealed = viewport.getOffsetToReveal(target, 1.0);
    expect(revealed.offset, 5 * (100 + 22 + 23) + 22 - 100);

    // With rect specified.
    revealed = viewport.getOffsetToReveal(target, 0.0, rect: const Rect.fromLTRB(1, 2, 3, 4));
    expect(revealed.offset, 5 * (100 + 22 + 23) + 22 + 1);
    revealed = viewport.getOffsetToReveal(target, 1.0, rect: const Rect.fromLTRB(1, 2, 3, 4));
    expect(revealed.offset, 5 * (100 + 22 + 23) + 22 - (200 - 3));
  });

  testWidgets('Viewport getOffsetToReveal Sliver - up', (WidgetTester tester) async {
    final ScrollController controller = ScrollController(initialScrollOffset: 300.0);
    addTearDown(controller.dispose);
    final List<Widget> children = <Widget>[];
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 200.0,
            width: 300.0,
            child: CustomScrollView(
              controller: controller,
              reverse: true,
              slivers: List<Widget>.generate(20, (int i) {
                final Widget sliver = SliverToBoxAdapter(
                  child: SizedBox(height: 100.0, child: Text('Tile $i')),
                );
                children.add(sliver);
                return SliverPadding(
                  padding: const EdgeInsets.only(top: 22.0, bottom: 23.0),
                  sliver: sliver,
                );
              }),
            ),
          ),
        ),
      ),
    );

    final RenderAbstractViewport viewport =
        tester.allRenderObjects.whereType<RenderAbstractViewport>().first;

    final RenderObject target = tester.renderObject(
      find.byWidget(children[5], skipOffstage: false),
    );
    RevealedOffset revealed = viewport.getOffsetToReveal(target, 0.0);
    // Does not include the bottom padding of children[5] thus + 23 instead of + 22.
    expect(revealed.offset, 5 * (100 + 22 + 23) + 23);

    revealed = viewport.getOffsetToReveal(target, 1.0);
    expect(revealed.offset, 5 * (100 + 22 + 23) + 23 - 100);

    // With rect specified.
    revealed = viewport.getOffsetToReveal(target, 0.0, rect: const Rect.fromLTRB(1, 2, 3, 4));
    expect(revealed.offset, 5 * (100 + 22 + 23) + 23 + (100 - 4));
    revealed = viewport.getOffsetToReveal(target, 1.0, rect: const Rect.fromLTRB(1, 2, 3, 4));
    expect(revealed.offset, -200 + 6 * (100 + 22 + 23) - 22 - 2);
  });

  testWidgets('Viewport getOffsetToReveal Sliver - up - reverse growth', (
    WidgetTester tester,
  ) async {
    const Key centerKey = ValueKey<String>('center');
    const EdgeInsets padding = EdgeInsets.only(top: 22.0, bottom: 23.0);
    const Widget centerSliver = SliverPadding(
      key: centerKey,
      padding: padding,
      sliver: SliverToBoxAdapter(child: SizedBox(height: 100.0, child: Text('Tile center'))),
    );
    const Widget lowerItem = SizedBox(height: 100.0, child: Text('Tile lower'));
    const Widget lowerSliver = SliverPadding(
      padding: padding,
      sliver: SliverToBoxAdapter(child: lowerItem),
    );

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 200.0,
            width: 300.0,
            child: CustomScrollView(
              center: centerKey,
              reverse: true,
              slivers: <Widget>[lowerSliver, centerSliver],
            ),
          ),
        ),
      ),
    );

    final RenderAbstractViewport viewport =
        tester.allRenderObjects.whereType<RenderAbstractViewport>().first;

    final RenderObject target = tester.renderObject(find.byWidget(lowerItem, skipOffstage: false));
    RevealedOffset revealed = viewport.getOffsetToReveal(target, 0.0);
    expect(revealed.offset, -100 - 22);

    revealed = viewport.getOffsetToReveal(target, 1.0);
    expect(revealed.offset, -100 - 22 - 100);

    // With rect specified.
    revealed = viewport.getOffsetToReveal(target, 0.0, rect: const Rect.fromLTRB(1, 2, 3, 4));
    expect(revealed.offset, -22 - 4);
    revealed = viewport.getOffsetToReveal(target, 1.0, rect: const Rect.fromLTRB(1, 2, 3, 4));
    expect(revealed.offset, -200 - 22 - 2);
  });

  testWidgets('Viewport getOffsetToReveal Sliver - left - reverse growth', (
    WidgetTester tester,
  ) async {
    const Key centerKey = ValueKey<String>('center');
    const EdgeInsets padding = EdgeInsets.only(left: 22.0, right: 23.0);
    const Widget centerSliver = SliverPadding(
      key: centerKey,
      padding: padding,
      sliver: SliverToBoxAdapter(child: SizedBox(width: 100.0, child: Text('Tile center'))),
    );
    const Widget lowerItem = SizedBox(width: 100.0, child: Text('Tile lower'));
    const Widget lowerSliver = SliverPadding(
      padding: padding,
      sliver: SliverToBoxAdapter(child: lowerItem),
    );

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 200.0,
            width: 300.0,
            child: CustomScrollView(
              scrollDirection: Axis.horizontal,
              center: centerKey,
              reverse: true,
              slivers: <Widget>[lowerSliver, centerSliver],
            ),
          ),
        ),
      ),
    );

    final RenderAbstractViewport viewport =
        tester.allRenderObjects.whereType<RenderAbstractViewport>().first;

    final RenderObject target = tester.renderObject(find.byWidget(lowerItem, skipOffstage: false));
    RevealedOffset revealed = viewport.getOffsetToReveal(target, 0.0);
    expect(revealed.offset, -100 - 22);

    revealed = viewport.getOffsetToReveal(target, 1.0);
    expect(revealed.offset, -100 - 22 - 200);

    // With rect specified.
    revealed = viewport.getOffsetToReveal(target, 0.0, rect: const Rect.fromLTRB(1, 2, 3, 4));
    expect(revealed.offset, -22 - 3);
    revealed = viewport.getOffsetToReveal(target, 1.0, rect: const Rect.fromLTRB(1, 2, 3, 4));
    expect(revealed.offset, -300 - 22 - 1);
  });

  testWidgets('Viewport getOffsetToReveal Sliver - left', (WidgetTester tester) async {
    final ScrollController controller = ScrollController(initialScrollOffset: 300.0);
    addTearDown(controller.dispose);
    final List<Widget> children = <Widget>[];
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 300.0,
            width: 200.0,
            child: CustomScrollView(
              scrollDirection: Axis.horizontal,
              reverse: true,
              controller: controller,
              slivers: List<Widget>.generate(20, (int i) {
                final Widget sliver = SliverToBoxAdapter(
                  child: SizedBox(width: 100.0, child: Text('Tile $i')),
                );
                children.add(sliver);
                return SliverPadding(
                  padding: const EdgeInsets.only(left: 22.0, right: 23.0),
                  sliver: sliver,
                );
              }),
            ),
          ),
        ),
      ),
    );

    final RenderAbstractViewport viewport =
        tester.allRenderObjects.whereType<RenderAbstractViewport>().first;

    final RenderObject target = tester.renderObject(
      find.byWidget(children[5], skipOffstage: false),
    );
    RevealedOffset revealed = viewport.getOffsetToReveal(target, 0.0);
    expect(revealed.offset, 5 * (100 + 22 + 23) + 23);

    revealed = viewport.getOffsetToReveal(target, 1.0);
    expect(revealed.offset, 5 * (100 + 22 + 23) + 23 - 100);

    // With rect specified.
    revealed = viewport.getOffsetToReveal(target, 0.0, rect: const Rect.fromLTRB(1, 2, 3, 4));
    expect(revealed.offset, 6 * (100 + 22 + 23) - 22 - 3);
    revealed = viewport.getOffsetToReveal(target, 1.0, rect: const Rect.fromLTRB(1, 2, 3, 4));
    expect(revealed.offset, -200 + 6 * (100 + 22 + 23) - 22 - 1);
  });

  testWidgets('Nested Viewports showOnScreen', (WidgetTester tester) async {
    final List<ScrollController> controllersX = List<ScrollController>.generate(
      10,
      (int i) => ScrollController(initialScrollOffset: 400.0),
    );
    final ScrollController controllerY = ScrollController(initialScrollOffset: 400.0);

    addTearDown(() {
      controllerY.dispose();
      for (final ScrollController controller in controllersX) {
        controller.dispose();
      }
    });

    final List<List<Widget>> children = List<List<Widget>>.generate(10, (int y) {
      return List<Widget>.generate(10, (int x) {
        return SizedBox(height: 100.0, width: 100.0, child: Text('$x,$y'));
      });
    });

    /// Builds a grid:
    ///
    ///       <- x ->
    ///   0 1 2 3 4 5 6 7 8 9
    /// 0 c c c c c c c c c c
    /// 1 c c c c c c c c c c
    /// 2 c c c c c c c c c c
    /// 3 c c c c c c c c c c  y
    /// 4 c c c c v v c c c c
    /// 5 c c c c v v c c c c
    /// 6 c c c c c c c c c c
    /// 7 c c c c c c c c c c
    /// 8 c c c c c c c c c c
    /// 9 c c c c c c c c c c
    ///
    /// Each c is a 100x100 container, v are containers visible in initial
    /// viewport.

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 200.0,
            width: 200.0,
            child: ListView(
              controller: controllerY,
              children: List<Widget>.generate(10, (int y) {
                return SizedBox(
                  height: 100.0,
                  child: ListView(
                    scrollDirection: Axis.horizontal,
                    controller: controllersX[y],
                    children: children[y],
                  ),
                );
              }),
            ),
          ),
        ),
      ),
    );

    // Already in viewport
    tester.renderObject(find.byWidget(children[4][4], skipOffstage: false)).showOnScreen();
    await tester.pumpAndSettle();
    expect(controllersX[4].offset, 400.0);
    expect(controllerY.offset, 400.0);

    controllersX[4].jumpTo(400.0);
    controllerY.jumpTo(400.0);
    await tester.pumpAndSettle();

    // Above viewport
    tester.renderObject(find.byWidget(children[3][4], skipOffstage: false)).showOnScreen();
    await tester.pumpAndSettle();
    expect(controllersX[3].offset, 400.0);
    expect(controllerY.offset, 300.0);

    controllersX[3].jumpTo(400.0);
    controllerY.jumpTo(400.0);
    await tester.pumpAndSettle();

    // Below viewport
    tester.renderObject(find.byWidget(children[6][4], skipOffstage: false)).showOnScreen();
    await tester.pumpAndSettle();
    expect(controllersX[6].offset, 400.0);
    expect(controllerY.offset, 500.0);

    controllersX[6].jumpTo(400.0);
    controllerY.jumpTo(400.0);
    await tester.pumpAndSettle();

    // Left of viewport
    tester.renderObject(find.byWidget(children[4][3], skipOffstage: false)).showOnScreen();
    await tester.pumpAndSettle();
    expect(controllersX[4].offset, 300.0);
    expect(controllerY.offset, 400.0);

    controllersX[4].jumpTo(400.0);
    controllerY.jumpTo(400.0);
    await tester.pumpAndSettle();

    // Right of viewport
    tester.renderObject(find.byWidget(children[4][6], skipOffstage: false)).showOnScreen();
    await tester.pumpAndSettle();
    expect(controllersX[4].offset, 500.0);
    expect(controllerY.offset, 400.0);

    controllersX[4].jumpTo(400.0);
    controllerY.jumpTo(400.0);
    await tester.pumpAndSettle();

    // Above and left of viewport
    tester.renderObject(find.byWidget(children[3][3], skipOffstage: false)).showOnScreen();
    await tester.pumpAndSettle();
    expect(controllersX[3].offset, 300.0);
    expect(controllerY.offset, 300.0);

    controllersX[3].jumpTo(400.0);
    controllerY.jumpTo(400.0);
    await tester.pumpAndSettle();

    // Below and left of viewport
    tester.renderObject(find.byWidget(children[6][3], skipOffstage: false)).showOnScreen();
    await tester.pumpAndSettle();
    expect(controllersX[6].offset, 300.0);
    expect(controllerY.offset, 500.0);

    controllersX[6].jumpTo(400.0);
    controllerY.jumpTo(400.0);
    await tester.pumpAndSettle();

    // Above and right of viewport
    tester.renderObject(find.byWidget(children[3][6], skipOffstage: false)).showOnScreen();
    await tester.pumpAndSettle();
    expect(controllersX[3].offset, 500.0);
    expect(controllerY.offset, 300.0);

    controllersX[3].jumpTo(400.0);
    controllerY.jumpTo(400.0);
    await tester.pumpAndSettle();

    // Below and right of viewport
    tester.renderObject(find.byWidget(children[6][6], skipOffstage: false)).showOnScreen();
    await tester.pumpAndSettle();
    expect(controllersX[6].offset, 500.0);
    expect(controllerY.offset, 500.0);

    controllersX[6].jumpTo(400.0);
    controllerY.jumpTo(400.0);
    await tester.pumpAndSettle();

    // Below and right of viewport with animations
    tester
        .renderObject(find.byWidget(children[6][6], skipOffstage: false))
        .showOnScreen(duration: const Duration(seconds: 2));
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));
    expect(tester.hasRunningAnimations, isTrue);
    expect(controllersX[6].offset, greaterThan(400.0));
    expect(controllersX[6].offset, lessThan(500.0));
    expect(controllerY.offset, greaterThan(400.0));
    expect(controllerY.offset, lessThan(500.0));
    await tester.pumpAndSettle();
    expect(controllersX[6].offset, 500.0);
    expect(controllerY.offset, 500.0);
  });

  group('Nested viewports (same orientation) showOnScreen', () {
    final List<Widget> children = List<Widget>.generate(10, (int i) {
      return SizedBox(height: 100.0, width: 300.0, child: Text('$i'));
    });

    Future<void> buildNestedScroller({
      required WidgetTester tester,
      required ScrollController inner,
      required ScrollController outer,
    }) {
      return tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              height: 200.0,
              width: 300.0,
              child: ListView(
                controller: outer,
                children: <Widget>[
                  const SizedBox(height: 200.0),
                  SizedBox(
                    height: 200.0,
                    width: 300.0,
                    child: ListView(controller: inner, children: children),
                  ),
                  const SizedBox(height: 200.0),
                ],
              ),
            ),
          ),
        ),
      );
    }

    testWidgets('Reverse List showOnScreen', (WidgetTester tester) async {
      addTearDown(tester.view.reset);
      const double screenHeight = 400.0;
      const double screenWidth = 400.0;
      const double itemHeight = screenHeight / 10.0;
      const ValueKey<String> centerKey = ValueKey<String>('center');

      tester.view.devicePixelRatio = 1.0;
      tester.view.physicalSize = const Size(screenWidth, screenHeight);

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: CustomScrollView(
            center: centerKey,
            reverse: true,
            slivers: <Widget>[
              SliverList(
                delegate: SliverChildListDelegate(
                  List<Widget>.generate(
                    10,
                    (int index) => SizedBox(height: itemHeight, child: Text('Item ${-index - 1}')),
                  ),
                ),
              ),
              SliverList(
                key: centerKey,
                delegate: SliverChildListDelegate(
                  List<Widget>.generate(
                    1,
                    (int index) => const SizedBox(height: itemHeight, child: Text('Item 0')),
                  ),
                ),
              ),
              SliverList(
                delegate: SliverChildListDelegate(
                  List<Widget>.generate(
                    10,
                    (int index) => SizedBox(height: itemHeight, child: Text('Item ${index + 1}')),
                  ),
                ),
              ),
            ],
          ),
        ),
      );

      expect(find.text('Item -1'), findsNothing);

      final RenderBox itemNeg1 = tester.renderObject(find.text('Item -1', skipOffstage: false));

      itemNeg1.showOnScreen(duration: const Duration(seconds: 1));
      await tester.pumpAndSettle();

      expect(find.text('Item -1'), findsOneWidget);
    });

    testWidgets('in view in inner, but not in outer', (WidgetTester tester) async {
      final ScrollController inner = ScrollController();
      final ScrollController outer = ScrollController();
      addTearDown(inner.dispose);
      addTearDown(outer.dispose);

      await buildNestedScroller(tester: tester, inner: inner, outer: outer);
      expect(outer.offset, 0.0);
      expect(inner.offset, 0.0);

      tester.renderObject(find.byWidget(children[0], skipOffstage: false)).showOnScreen();
      await tester.pumpAndSettle();
      expect(inner.offset, 0.0);
      expect(outer.offset, 100.0);
    });

    testWidgets('not in view of neither inner nor outer', (WidgetTester tester) async {
      final ScrollController inner = ScrollController();
      final ScrollController outer = ScrollController();
      addTearDown(inner.dispose);
      addTearDown(outer.dispose);

      await buildNestedScroller(tester: tester, inner: inner, outer: outer);
      expect(outer.offset, 0.0);
      expect(inner.offset, 0.0);

      tester.renderObject(find.byWidget(children[4], skipOffstage: false)).showOnScreen();
      await tester.pumpAndSettle();
      expect(inner.offset, 300.0);
      expect(outer.offset, 200.0);
    });

    testWidgets('in view in inner and outer', (WidgetTester tester) async {
      final ScrollController inner = ScrollController(initialScrollOffset: 200.0);
      final ScrollController outer = ScrollController(initialScrollOffset: 200.0);
      addTearDown(inner.dispose);
      addTearDown(outer.dispose);

      await buildNestedScroller(tester: tester, inner: inner, outer: outer);
      expect(outer.offset, 200.0);
      expect(inner.offset, 200.0);

      tester.renderObject(find.byWidget(children[2])).showOnScreen();
      await tester.pumpAndSettle();
      expect(outer.offset, 200.0);
      expect(inner.offset, 200.0);
    });

    testWidgets('inner shown in outer, but item not visible', (WidgetTester tester) async {
      final ScrollController inner = ScrollController(initialScrollOffset: 200.0);
      final ScrollController outer = ScrollController(initialScrollOffset: 200.0);
      addTearDown(inner.dispose);
      addTearDown(outer.dispose);

      await buildNestedScroller(tester: tester, inner: inner, outer: outer);
      expect(outer.offset, 200.0);
      expect(inner.offset, 200.0);

      tester.renderObject(find.byWidget(children[5], skipOffstage: false)).showOnScreen();
      await tester.pumpAndSettle();
      expect(outer.offset, 200.0);
      expect(inner.offset, 400.0);
    });

    testWidgets('inner half shown in outer, item only visible in inner', (
      WidgetTester tester,
    ) async {
      final ScrollController inner = ScrollController();
      final ScrollController outer = ScrollController(initialScrollOffset: 100.0);
      addTearDown(inner.dispose);
      addTearDown(outer.dispose);

      await buildNestedScroller(tester: tester, inner: inner, outer: outer);
      expect(outer.offset, 100.0);
      expect(inner.offset, 0.0);

      tester.renderObject(find.byWidget(children[1])).showOnScreen();
      await tester.pumpAndSettle();
      expect(outer.offset, 200.0);
      expect(inner.offset, 0.0);
    });
  });

  testWidgets(
    'Nested Viewports showOnScreen with allowImplicitScrolling=false for inner viewport',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/20893.

      List<Widget> slivers;
      final ScrollController controllerX = ScrollController();
      final ScrollController controllerY = ScrollController();
      addTearDown(controllerX.dispose);
      addTearDown(controllerY.dispose);

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              height: 200.0,
              width: 200.0,
              child: ListView(
                controller: controllerY,
                children: <Widget>[
                  const SizedBox(height: 150.0),
                  SizedBox(
                    height: 100.0,
                    child: ListView(
                      physics: const PageScrollPhysics(), // Turns off `allowImplicitScrolling`
                      scrollDirection: Axis.horizontal,
                      controller: controllerX,
                      children:
                          slivers = <Widget>[Container(width: 150.0), Container(width: 150.0)],
                    ),
                  ),
                  const SizedBox(height: 150.0),
                ],
              ),
            ),
          ),
        ),
      );

      tester.renderObject(find.byWidget(slivers[1])).showOnScreen();
      await tester.pumpAndSettle();
      expect(controllerX.offset, 0.0);
      expect(controllerY.offset, 50.0);
    },
  );

  testWidgets(
    'Nested Viewports showOnScreen on Sliver with allowImplicitScrolling=false for inner viewport',
    (WidgetTester tester) async {
      Widget sliver;
      final ScrollController controllerX = ScrollController();
      final ScrollController controllerY = ScrollController();
      addTearDown(controllerX.dispose);
      addTearDown(controllerY.dispose);

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              height: 200.0,
              width: 200.0,
              child: ListView(
                controller: controllerY,
                children: <Widget>[
                  const SizedBox(height: 150.0),
                  SizedBox(
                    height: 100.0,
                    child: CustomScrollView(
                      physics: const PageScrollPhysics(), // Turns off `allowImplicitScrolling`
                      scrollDirection: Axis.horizontal,
                      controller: controllerX,
                      slivers: <Widget>[
                        SliverPadding(
                          padding: const EdgeInsets.all(25.0),
                          sliver: SliverToBoxAdapter(child: Container(width: 100.0)),
                        ),
                        SliverPadding(
                          padding: const EdgeInsets.all(25.0),
                          sliver: sliver = SliverToBoxAdapter(child: Container(width: 100.0)),
                        ),
                      ],
                    ),
                  ),
                  const SizedBox(height: 150.0),
                ],
              ),
            ),
          ),
        ),
      );

      tester.renderObject(find.byWidget(sliver)).showOnScreen();
      await tester.pumpAndSettle();
      expect(controllerX.offset, 0.0);
      expect(controllerY.offset, 25.0);
    },
  );

  testWidgets('Viewport showOnScreen with objects larger than viewport', (
    WidgetTester tester,
  ) async {
    List<Widget> children;
    final ScrollController controller = ScrollController(initialScrollOffset: 300.0);
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 200.0,
            child: ListView(
              controller: controller,
              children:
                  children = List<Widget>.generate(20, (int i) {
                    return SizedBox(height: 300.0, child: Text('Tile $i'));
                  }),
            ),
          ),
        ),
      ),
    );

    expect(controller.offset, 300.0);

    // Already aligned with leading edge, nothing happens.
    tester.renderObject(find.byWidget(children[1], skipOffstage: false)).showOnScreen();
    await tester.pumpAndSettle();
    expect(controller.offset, 300.0);

    // Above leading edge aligns trailing edges
    tester.renderObject(find.byWidget(children[0], skipOffstage: false)).showOnScreen();
    await tester.pumpAndSettle();
    expect(controller.offset, 100.0);

    // Below trailing edge aligns leading edges
    tester.renderObject(find.byWidget(children[1], skipOffstage: false)).showOnScreen();
    await tester.pumpAndSettle();
    expect(controller.offset, 300.0);

    controller.jumpTo(250.0);
    await tester.pumpAndSettle();
    expect(controller.offset, 250.0);

    // Partly visible across leading edge aligns trailing edges
    tester.renderObject(find.byWidget(children[0], skipOffstage: false)).showOnScreen();
    await tester.pumpAndSettle();
    expect(controller.offset, 100.0);

    controller.jumpTo(150.0);
    await tester.pumpAndSettle();
    expect(controller.offset, 150.0);

    // Partly visible across trailing edge aligns leading edges
    tester.renderObject(find.byWidget(children[1], skipOffstage: false)).showOnScreen();
    await tester.pumpAndSettle();
    expect(controller.offset, 300.0);
  });

  testWidgets(
    'Viewport showOnScreen should not scroll if the rect is already visible, even if it does not scroll linearly',
    (WidgetTester tester) async {
      List<Widget> children;
      final ScrollController controller = ScrollController(initialScrollOffset: 300.0);
      addTearDown(controller.dispose);

      const Key headerKey = Key('header');
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              height: 600.0,
              child: CustomScrollView(
                controller: controller,
                slivers:
                    children = List<Widget>.generate(20, (int i) {
                      return i == 10
                          ? SliverPersistentHeader(
                            pinned: true,
                            delegate: _TestSliverPersistentHeaderDelegate(
                              minExtent: 100,
                              maxExtent: 300,
                              key: headerKey,
                            ),
                          )
                          : SliverToBoxAdapter(
                            child: SizedBox(height: 300.0, child: Text('Tile $i')),
                          );
                    }),
              ),
            ),
          ),
        ),
      );

      controller.jumpTo(300.0 * 15);
      await tester.pumpAndSettle();

      final Finder pinnedHeaderContent = find.descendant(
        of: find.byWidget(children[10]),
        matching: find.byKey(headerKey),
      );

      // The persistent header is pinned to the leading edge thus still visible,
      // the viewport should not scroll.
      tester.renderObject(pinnedHeaderContent).showOnScreen();
      await tester.pumpAndSettle();
      expect(controller.offset, 300.0 * 15);

      // The 11th child will be partially obstructed by the persistent header,
      // the viewport should scroll to reveal it.
      controller.jumpTo(
        11 *
                300.0 // Preceding headers
                +
            200.0 // Shrinks the pinned header to minExtent
            +
            100.0, // Obstructs the leading 100 pixels of the 11th header
      );
      await tester.pumpAndSettle();

      tester.renderObject(find.byWidget(children[11], skipOffstage: false)).showOnScreen();
      await tester.pumpAndSettle();
      expect(controller.offset, lessThan(11 * 300.0 + 200.0 + 100.0));
    },
  );

  void testFloatingHeaderShowOnScreen({bool animated = true, Axis axis = Axis.vertical}) {
    final TickerProvider? vsync = animated ? const TestVSync() : null;
    const Key headerKey = Key('header');
    late List<Widget> children;
    final ScrollController controller = ScrollController(initialScrollOffset: 300.0);

    Widget buildList({required SliverPersistentHeader floatingHeader, bool reversed = false}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 400.0,
            width: 400.0,
            child: CustomScrollView(
              scrollDirection: axis,
              center: reversed ? const Key('19') : null,
              controller: controller,
              slivers:
                  children = List<Widget>.generate(20, (int i) {
                    return i == 10
                        ? floatingHeader
                        : SliverToBoxAdapter(
                          key: (i == 19) ? const Key('19') : null,
                          child: SizedBox(height: 300.0, width: 300, child: Text('Tile $i')),
                        );
                  }),
            ),
          ),
        ),
      );
    }

    double mainAxisExtent(WidgetTester tester, Finder finder) {
      final RenderObject renderObject = tester.renderObject(finder);
      if (renderObject is RenderSliver) {
        return renderObject.geometry!.paintExtent;
      }

      final RenderBox renderBox = renderObject as RenderBox;
      return switch (axis) {
        Axis.horizontal => renderBox.size.width,
        Axis.vertical => renderBox.size.height,
      };
    }

    group('animated: $animated, scrollDirection: $axis', () {
      testWidgets('RenderViewportBase.showOnScreen', (WidgetTester tester) async {
        await tester.pumpWidget(
          buildList(
            floatingHeader: SliverPersistentHeader(
              pinned: true,
              floating: true,
              delegate: _TestSliverPersistentHeaderDelegate(
                minExtent: 100,
                maxExtent: 300,
                key: headerKey,
                vsync: vsync,
              ),
            ),
          ),
        );

        final Finder pinnedHeaderContent = find.byKey(headerKey, skipOffstage: false);

        controller.jumpTo(300.0 * 15);
        await tester.pumpAndSettle();
        expect(mainAxisExtent(tester, pinnedHeaderContent), lessThan(300));

        // The persistent header is pinned to the leading edge thus still visible,
        // the viewport should not scroll.
        tester
            .renderObject(pinnedHeaderContent)
            .showOnScreen(
              descendant: tester.renderObject(pinnedHeaderContent),
              rect: Offset.zero & const Size(300, 300),
            );
        await tester.pumpAndSettle();
        // The header expands but doesn't move.
        expect(controller.offset, 300.0 * 15);
        expect(mainAxisExtent(tester, pinnedHeaderContent), 300);

        // The rect specifies that the persistent header needs to be 1 pixel away
        // from the leading edge of the viewport. Ignore the 1 pixel, the viewport
        // should not scroll.
        //
        // See: https://github.com/flutter/flutter/issues/25507.
        tester
            .renderObject(pinnedHeaderContent)
            .showOnScreen(
              descendant: tester.renderObject(pinnedHeaderContent),
              rect: const Offset(-1, -1) & const Size(300, 300),
            );
        await tester.pumpAndSettle();
        expect(controller.offset, 300.0 * 15);
        expect(mainAxisExtent(tester, pinnedHeaderContent), 300);
      });

      testWidgets('RenderViewportBase.showOnScreen twice almost instantly', (
        WidgetTester tester,
      ) async {
        // Regression test for https://github.com/flutter/flutter/issues/137901
        await tester.pumpWidget(
          buildList(
            floatingHeader: SliverPersistentHeader(
              pinned: true,
              floating: true,
              delegate: _TestSliverPersistentHeaderDelegate(
                minExtent: 100,
                maxExtent: 300,
                key: headerKey,
                vsync: vsync,
              ),
            ),
          ),
        );

        final Finder pinnedHeaderContent = find.byKey(headerKey, skipOffstage: false);

        controller.jumpTo(300.0 * 15);
        await tester.pumpAndSettle();
        expect(mainAxisExtent(tester, pinnedHeaderContent), lessThan(300));

        tester
            .renderObject(pinnedHeaderContent)
            .showOnScreen(
              descendant: tester.renderObject(pinnedHeaderContent),
              // Adding different rect to check if the second showOnScreen call
              // leads to a different result.
              // When the animation has forward status and the second showOnScreen
              // is called, the new animation won't start.
              rect: Offset.zero & const Size(150, 150),
              duration: const Duration(seconds: 3),
            );
        await tester.pump(const Duration(seconds: 1));

        tester
            .renderObject(pinnedHeaderContent)
            .showOnScreen(
              descendant: tester.renderObject(pinnedHeaderContent),
              rect: Offset.zero & const Size(300, 300),
            );

        await tester.pumpAndSettle();
        expect(controller.offset, 300.0 * 15);
        expect(mainAxisExtent(tester, pinnedHeaderContent), 150);
      });

      testWidgets('RenderViewportBase.showOnScreen but no child', (WidgetTester tester) async {
        await tester.pumpWidget(
          buildList(
            floatingHeader: SliverPersistentHeader(
              key: headerKey,
              pinned: true,
              floating: true,
              delegate: _TestSliverPersistentHeaderDelegate(
                minExtent: 100,
                maxExtent: 300,
                vsync: vsync,
              ),
            ),
          ),
        );

        final Finder pinnedHeaderContent = find.byKey(headerKey, skipOffstage: false);

        controller.jumpTo(300.0 * 15);
        await tester.pumpAndSettle();
        expect(mainAxisExtent(tester, pinnedHeaderContent), lessThan(300));

        // The persistent header is pinned to the leading edge thus still visible,
        // the viewport should not scroll.
        tester
            .renderObject(pinnedHeaderContent)
            .showOnScreen(rect: Offset.zero & const Size(300, 300));
        await tester.pumpAndSettle();
        // The header expands but doesn't move.
        expect(controller.offset, 300.0 * 15);
        expect(mainAxisExtent(tester, pinnedHeaderContent), 300);

        // The rect specifies that the persistent header needs to be 1 pixel away
        // from the leading edge of the viewport. Ignore the 1 pixel, the viewport
        // should not scroll.
        //
        // See: https://github.com/flutter/flutter/issues/25507.
        tester
            .renderObject(pinnedHeaderContent)
            .showOnScreen(rect: const Offset(-1, -1) & const Size(300, 300));
        await tester.pumpAndSettle();
        expect(controller.offset, 300.0 * 15);
        expect(mainAxisExtent(tester, pinnedHeaderContent), 300);
      });

      testWidgets('RenderViewportBase.showOnScreen with maxShowOnScreenExtent ', (
        WidgetTester tester,
      ) async {
        await tester.pumpWidget(
          buildList(
            floatingHeader: SliverPersistentHeader(
              pinned: true,
              floating: true,
              delegate: _TestSliverPersistentHeaderDelegate(
                minExtent: 100,
                maxExtent: 300,
                key: headerKey,
                vsync: vsync,
                showOnScreenConfiguration: const PersistentHeaderShowOnScreenConfiguration(
                  maxShowOnScreenExtent: 200,
                ),
              ),
            ),
          ),
        );

        final Finder pinnedHeaderContent = find.byKey(headerKey, skipOffstage: false);

        controller.jumpTo(300.0 * 15);
        await tester.pumpAndSettle();
        // childExtent was initially 100.
        expect(mainAxisExtent(tester, pinnedHeaderContent), 100);

        tester
            .renderObject(pinnedHeaderContent)
            .showOnScreen(
              descendant: tester.renderObject(pinnedHeaderContent),
              rect: Offset.zero & const Size(300, 300),
            );
        await tester.pumpAndSettle();
        // The header doesn't move. It would have expanded to 300 but
        // maxShowOnScreenExtent is 200, preventing it from doing so.
        expect(controller.offset, 300.0 * 15);
        expect(mainAxisExtent(tester, pinnedHeaderContent), 200);

        // ignoreLeading still works.
        tester
            .renderObject(pinnedHeaderContent)
            .showOnScreen(
              descendant: tester.renderObject(pinnedHeaderContent),
              rect: const Offset(-1, -1) & const Size(300, 300),
            );
        await tester.pumpAndSettle();
        expect(controller.offset, 300.0 * 15);
        expect(mainAxisExtent(tester, pinnedHeaderContent), 200);

        // Move the viewport so that its childExtent reaches 250.
        controller.jumpTo(300.0 * 10 + 50.0);
        await tester.pumpAndSettle();
        expect(mainAxisExtent(tester, pinnedHeaderContent), 250);

        // Doesn't move, doesn't expand or shrink, leading still ignored.
        tester
            .renderObject(pinnedHeaderContent)
            .showOnScreen(
              descendant: tester.renderObject(pinnedHeaderContent),
              rect: const Offset(-1, -1) & const Size(300, 300),
            );
        await tester.pumpAndSettle();
        expect(controller.offset, 300.0 * 10 + 50.0);
        expect(mainAxisExtent(tester, pinnedHeaderContent), 250);
      });

      testWidgets('RenderViewportBase.showOnScreen with minShowOnScreenExtent ', (
        WidgetTester tester,
      ) async {
        await tester.pumpWidget(
          buildList(
            floatingHeader: SliverPersistentHeader(
              pinned: true,
              floating: true,
              delegate: _TestSliverPersistentHeaderDelegate(
                minExtent: 100,
                maxExtent: 300,
                key: headerKey,
                vsync: vsync,
                showOnScreenConfiguration: const PersistentHeaderShowOnScreenConfiguration(
                  minShowOnScreenExtent: 200,
                ),
              ),
            ),
          ),
        );

        final Finder pinnedHeaderContent = find.byKey(headerKey, skipOffstage: false);

        controller.jumpTo(300.0 * 15);
        await tester.pumpAndSettle();
        // childExtent was initially 100.
        expect(mainAxisExtent(tester, pinnedHeaderContent), 100);

        tester
            .renderObject(pinnedHeaderContent)
            .showOnScreen(
              descendant: tester.renderObject(pinnedHeaderContent),
              rect: Offset.zero & const Size(110, 110),
            );
        await tester.pumpAndSettle();
        // The header doesn't move. It would have expanded to 110 but
        // minShowOnScreenExtent is 200, preventing it from doing so.
        expect(controller.offset, 300.0 * 15);
        expect(mainAxisExtent(tester, pinnedHeaderContent), 200);

        // ignoreLeading still works.
        tester
            .renderObject(pinnedHeaderContent)
            .showOnScreen(
              descendant: tester.renderObject(pinnedHeaderContent),
              rect: const Offset(-1, -1) & const Size(110, 110),
            );
        await tester.pumpAndSettle();
        expect(controller.offset, 300.0 * 15);
        expect(mainAxisExtent(tester, pinnedHeaderContent), 200);

        // Move the viewport so that its childExtent reaches 250.
        controller.jumpTo(300.0 * 10 + 50.0);
        await tester.pumpAndSettle();
        expect(mainAxisExtent(tester, pinnedHeaderContent), 250);

        // Doesn't move, doesn't expand or shrink, leading still ignored.
        tester
            .renderObject(pinnedHeaderContent)
            .showOnScreen(
              descendant: tester.renderObject(pinnedHeaderContent),
              rect: const Offset(-1, -1) & const Size(110, 110),
            );
        await tester.pumpAndSettle();
        expect(controller.offset, 300.0 * 10 + 50.0);
        expect(mainAxisExtent(tester, pinnedHeaderContent), 250);
      });

      testWidgets(
        'RenderViewportBase.showOnScreen should not scroll if the rect is already visible, '
        'even if it does not scroll linearly (reversed order version)',
        (WidgetTester tester) async {
          await tester.pumpWidget(
            buildList(
              floatingHeader: SliverPersistentHeader(
                pinned: true,
                floating: true,
                delegate: _TestSliverPersistentHeaderDelegate(
                  minExtent: 100,
                  maxExtent: 300,
                  key: headerKey,
                  vsync: vsync,
                ),
              ),
              reversed: true,
            ),
          );

          controller.jumpTo(-300.0 * 15);
          await tester.pumpAndSettle();

          final Finder pinnedHeaderContent = find.byKey(headerKey, skipOffstage: false);

          // The persistent header is pinned to the leading edge thus still visible,
          // the viewport should not scroll.
          tester.renderObject(pinnedHeaderContent).showOnScreen();
          await tester.pumpAndSettle();
          expect(controller.offset, -300.0 * 15);

          // children[9] will be partially obstructed by the persistent header,
          // the viewport should scroll to reveal it.
          controller.jumpTo(
            -8 *
                    300.0 // Preceding headers 11 - 18, children[11]'s top edge is aligned to the leading edge.
                    -
                400.0 // Viewport height. children[10] (the pinned header) becomes pinned at the bottom of the screen.
                -
                200.0 // Shrinks the pinned header to minExtent (100).
                -
                100.0, // Obstructs the leading 100 pixels of the 11th header
          );
          await tester.pumpAndSettle();

          tester.renderObject(find.byWidget(children[9], skipOffstage: false)).showOnScreen();
          await tester.pumpAndSettle();
          expect(controller.offset, -8 * 300.0 - 400.0 - 200.0);
        },
      );
    });
  }

  group('Floating header showOnScreen', () {
    testFloatingHeaderShowOnScreen();
    testFloatingHeaderShowOnScreen(axis: Axis.horizontal);
  });

  group('RenderViewport getOffsetToReveal renderBox to sliver coordinates conversion', () {
    const EdgeInsets padding = EdgeInsets.fromLTRB(22, 22, 34, 34);
    const Key centerKey = Key('5');
    Widget buildList({required Axis axis, bool reverse = false, bool reverseGrowth = false}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 400.0,
            width: 400.0,
            child: CustomScrollView(
              scrollDirection: axis,
              reverse: reverse,
              center: reverseGrowth ? centerKey : null,
              slivers: List<Widget>.generate(6, (int i) {
                return SliverPadding(
                  key: i == 5 ? centerKey : null,
                  padding: padding,
                  sliver: SliverToBoxAdapter(
                    child: Container(
                      padding: padding,
                      height: 300.0,
                      width: 300.0,
                      child: Text('Tile $i'),
                    ),
                  ),
                );
              }),
            ),
          ),
        ),
      );
    }

    testWidgets('up, forward growth', (WidgetTester tester) async {
      await tester.pumpWidget(buildList(axis: Axis.vertical, reverse: true));
      final RenderAbstractViewport viewport =
          tester.allRenderObjects.whereType<RenderAbstractViewport>().first;

      final RenderObject target = tester.renderObject(find.text('Tile 5', skipOffstage: false));
      final double revealOffset = viewport.getOffsetToReveal(target, 0.0).offset;
      expect(revealOffset, (300.0 + padding.horizontal) * 5 + 34.0 * 2);
    });

    testWidgets('up, reverse growth', (WidgetTester tester) async {
      await tester.pumpWidget(buildList(axis: Axis.vertical, reverse: true, reverseGrowth: true));
      final RenderAbstractViewport viewport =
          tester.allRenderObjects.whereType<RenderAbstractViewport>().first;

      final RenderObject target = tester.renderObject(find.text('Tile 0', skipOffstage: false));
      final double revealOffset = viewport.getOffsetToReveal(target, 0.0).offset;
      expect(revealOffset, -(300.0 + padding.horizontal) * 5 + 34.0 * 2);
    });

    testWidgets('right, forward growth', (WidgetTester tester) async {
      await tester.pumpWidget(buildList(axis: Axis.horizontal));
      final RenderAbstractViewport viewport =
          tester.allRenderObjects.whereType<RenderAbstractViewport>().first;

      final RenderObject target = tester.renderObject(find.text('Tile 5', skipOffstage: false));
      final double revealOffset = viewport.getOffsetToReveal(target, 0.0).offset;
      expect(revealOffset, (300.0 + padding.horizontal) * 5 + 22.0 * 2);
    });

    testWidgets('right, reverse growth', (WidgetTester tester) async {
      await tester.pumpWidget(buildList(axis: Axis.horizontal, reverseGrowth: true));
      final RenderAbstractViewport viewport =
          tester.allRenderObjects.whereType<RenderAbstractViewport>().first;

      final RenderObject target = tester.renderObject(find.text('Tile 0', skipOffstage: false));
      final double revealOffset = viewport.getOffsetToReveal(target, 0.0).offset;
      expect(revealOffset, -(300.0 + padding.horizontal) * 5 + 22.0 * 2);
    });

    testWidgets('down, forward growth', (WidgetTester tester) async {
      await tester.pumpWidget(buildList(axis: Axis.vertical));
      final RenderAbstractViewport viewport =
          tester.allRenderObjects.whereType<RenderAbstractViewport>().first;

      final RenderObject target = tester.renderObject(find.text('Tile 5', skipOffstage: false));
      final double revealOffset = viewport.getOffsetToReveal(target, 0.0).offset;
      expect(revealOffset, (300.0 + padding.horizontal) * 5 + 22.0 * 2);
    });

    testWidgets('down, reverse growth', (WidgetTester tester) async {
      await tester.pumpWidget(buildList(axis: Axis.vertical, reverseGrowth: true));
      final RenderAbstractViewport viewport =
          tester.allRenderObjects.whereType<RenderAbstractViewport>().first;

      final RenderObject target = tester.renderObject(find.text('Tile 0', skipOffstage: false));
      final double revealOffset = viewport.getOffsetToReveal(target, 0.0).offset;
      expect(revealOffset, -(300.0 + padding.horizontal) * 5 + 22.0 * 2);
    });

    testWidgets('left, forward growth', (WidgetTester tester) async {
      await tester.pumpWidget(buildList(axis: Axis.horizontal, reverse: true));
      final RenderAbstractViewport viewport =
          tester.allRenderObjects.whereType<RenderAbstractViewport>().first;

      final RenderObject target = tester.renderObject(find.text('Tile 5', skipOffstage: false));
      final double revealOffset = viewport.getOffsetToReveal(target, 0.0).offset;
      expect(revealOffset, (300.0 + padding.horizontal) * 5 + 34.0 * 2);
    });

    testWidgets('left, reverse growth', (WidgetTester tester) async {
      await tester.pumpWidget(buildList(axis: Axis.horizontal, reverse: true, reverseGrowth: true));
      final RenderAbstractViewport viewport =
          tester.allRenderObjects.whereType<RenderAbstractViewport>().first;

      final RenderObject target = tester.renderObject(find.text('Tile 0', skipOffstage: false));
      final double revealOffset = viewport.getOffsetToReveal(target, 0.0).offset;
      expect(revealOffset, -(300.0 + padding.horizontal) * 5 + 34.0 * 2);
    });

    testWidgets('will not assert on mismatched axis', (WidgetTester tester) async {
      await tester.pumpWidget(buildList(axis: Axis.vertical, reverse: true, reverseGrowth: true));
      final RenderAbstractViewport viewport =
          tester.allRenderObjects.whereType<RenderAbstractViewport>().first;

      final RenderObject target = tester.renderObject(find.text('Tile 0', skipOffstage: false));
      viewport.getOffsetToReveal(target, 0.0, axis: Axis.horizontal);
    });
  });

  testWidgets('RenderViewportBase.showOnScreen reports the correct targetRect', (
    WidgetTester tester,
  ) async {
    final ScrollController innerController = ScrollController();
    final ScrollController outerController = ScrollController();
    addTearDown(innerController.dispose);
    addTearDown(outerController.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 300.0,
            child: CustomScrollView(
              cacheExtent: 0,
              controller: outerController,
              slivers: <Widget>[
                SliverToBoxAdapter(
                  child: SizedBox(
                    height: 300,
                    child: CustomScrollView(
                      controller: innerController,
                      slivers: List<Widget>.generate(5, (int i) {
                        return SliverToBoxAdapter(
                          child: SizedBox(height: 300.0, child: Text('Tile $i')),
                        );
                      }),
                    ),
                  ),
                ),
                const SliverToBoxAdapter(child: SizedBox(height: 300.0, child: Text('hidden'))),
              ],
            ),
          ),
        ),
      ),
    );

    tester
        .renderObject(find.widgetWithText(SizedBox, 'Tile 1', skipOffstage: false).first)
        .showOnScreen();
    await tester.pumpAndSettle();
    // The inner viewport scrolls to reveal the 2nd tile.
    expect(innerController.offset, 300.0);
    expect(outerController.offset, 0);
  });

  group('unbounded constraints control test', () {
    Widget buildNestedWidget([Axis a1 = Axis.vertical, Axis a2 = Axis.horizontal]) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: ListView(
            scrollDirection: a1,
            children: List<Widget>.generate(10, (int y) {
              return ListView(scrollDirection: a2);
            }),
          ),
        ),
      );
    }

    Future<void> expectFlutterError({
      required Widget widget,
      required WidgetTester tester,
      required String message,
    }) async {
      final List<FlutterErrorDetails> errors = <FlutterErrorDetails>[];
      final FlutterExceptionHandler? oldHandler = FlutterError.onError;
      FlutterError.onError = (FlutterErrorDetails error) => errors.add(error);
      try {
        await tester.pumpWidget(widget);
      } finally {
        FlutterError.onError = oldHandler;
      }
      expect(errors, isNotEmpty);
      expect(errors.first.exception, isFlutterError);
      expect((errors.first.exception as FlutterError).toStringDeep(), message);
    }

    testWidgets('Horizontal viewport was given unbounded height', (WidgetTester tester) async {
      await expectFlutterError(
        widget: buildNestedWidget(),
        tester: tester,
        message:
            'FlutterError\n'
            '   Horizontal viewport was given unbounded height.\n'
            '   Viewports expand in the cross axis to fill their container and\n'
            '   constrain their children to match their extent in the cross axis.\n'
            '   In this case, a horizontal viewport was given an unlimited amount\n'
            '   of vertical space in which to expand.\n',
      );
    });

    testWidgets('Horizontal viewport was given unbounded width', (WidgetTester tester) async {
      await expectFlutterError(
        widget: buildNestedWidget(Axis.horizontal),
        tester: tester,
        message:
            'FlutterError\n'
            '   Horizontal viewport was given unbounded width.\n'
            '   Viewports expand in the scrolling direction to fill their\n'
            '   container. In this case, a horizontal viewport was given an\n'
            '   unlimited amount of horizontal space in which to expand. This\n'
            '   situation typically happens when a scrollable widget is nested\n'
            '   inside another scrollable widget.\n'
            '   If this widget is always nested in a scrollable widget there is\n'
            '   no need to use a viewport because there will always be enough\n'
            '   horizontal space for the children. In this case, consider using a\n'
            '   Row or Wrap instead. Otherwise, consider using a CustomScrollView\n'
            '   to concatenate arbitrary slivers into a single scrollable.\n',
      );
    });

    testWidgets('Vertical viewport was given unbounded width', (WidgetTester tester) async {
      await expectFlutterError(
        widget: buildNestedWidget(Axis.horizontal, Axis.vertical),
        tester: tester,
        message:
            'FlutterError\n'
            '   Vertical viewport was given unbounded width.\n'
            '   Viewports expand in the cross axis to fill their container and\n'
            '   constrain their children to match their extent in the cross axis.\n'
            '   In this case, a vertical viewport was given an unlimited amount\n'
            '   of horizontal space in which to expand.\n',
      );
    });

    testWidgets('Vertical viewport was given unbounded height', (WidgetTester tester) async {
      await expectFlutterError(
        widget: buildNestedWidget(Axis.vertical, Axis.vertical),
        tester: tester,
        message:
            'FlutterError\n'
            '   Vertical viewport was given unbounded height.\n'
            '   Viewports expand in the scrolling direction to fill their\n'
            '   container. In this case, a vertical viewport was given an\n'
            '   unlimited amount of vertical space in which to expand. This\n'
            '   situation typically happens when a scrollable widget is nested\n'
            '   inside another scrollable widget.\n'
            '   If this widget is always nested in a scrollable widget there is\n'
            '   no need to use a viewport because there will always be enough\n'
            '   vertical space for the children. In this case, consider using a\n'
            '   Column or Wrap instead. Otherwise, consider using a\n'
            '   CustomScrollView to concatenate arbitrary slivers into a single\n'
            '   scrollable.\n',
      );
    });
  });

  test('Viewport debugThrowIfNotCheckingIntrinsics() control test', () {
    final RenderViewport renderViewport = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
    );
    late FlutterError error;
    try {
      renderViewport.computeMinIntrinsicHeight(0);
    } on FlutterError catch (e) {
      error = e;
    }
    expect(
      error.toStringDeep(),
      'FlutterError\n'
      '   RenderViewport does not support returning intrinsic dimensions.\n'
      '   Calculating the intrinsic dimensions would require instantiating\n'
      '   every child of the viewport, which defeats the point of viewports\n'
      '   being lazy.\n'
      '   If you are merely trying to shrink-wrap the viewport in the main\n'
      '   axis direction, consider a RenderShrinkWrappingViewport render\n'
      '   object (ShrinkWrappingViewport widget), which achieves that\n'
      '   effect without implementing the intrinsic dimension API.\n',
    );

    final RenderShrinkWrappingViewport renderShrinkWrappingViewport = RenderShrinkWrappingViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
    );
    try {
      renderShrinkWrappingViewport.computeMinIntrinsicHeight(0);
    } on FlutterError catch (e) {
      error = e;
    }
    expect(error, isNotNull);
    expect(
      error.toStringDeep(),
      'FlutterError\n'
      '   RenderShrinkWrappingViewport does not support returning intrinsic\n'
      '   dimensions.\n'
      '   Calculating the intrinsic dimensions would require instantiating\n'
      '   every child of the viewport, which defeats the point of viewports\n'
      '   being lazy.\n'
      '   If you are merely trying to shrink-wrap the viewport in the main\n'
      '   axis direction, you should be able to achieve that effect by just\n'
      '   giving the viewport loose constraints, without needing to measure\n'
      '   its intrinsic dimensions.\n',
    );
  });

  group('Viewport childrenInPaintOrder control test', () {
    test('RenderViewport', () async {
      final List<RenderSliver> children = <RenderSliver>[
        RenderSliverToBoxAdapter(),
        RenderSliverToBoxAdapter(),
        RenderSliverToBoxAdapter(),
      ];

      final RenderViewport renderViewport = RenderViewport(
        crossAxisDirection: AxisDirection.right,
        offset: ViewportOffset.zero(),
        children: children,
      );

      // Children should be painted in reverse order to the list given
      expect(renderViewport.childrenInPaintOrder, equals(children.reversed));
      // childrenInPaintOrder should be reverse of childrenInHitTestOrder
      expect(
        renderViewport.childrenInPaintOrder,
        equals(renderViewport.childrenInHitTestOrder.toList().reversed),
      );
    });

    test('RenderShrinkWrappingViewport', () async {
      final List<RenderSliver> children = <RenderSliver>[
        RenderSliverToBoxAdapter(),
        RenderSliverToBoxAdapter(),
        RenderSliverToBoxAdapter(),
      ];

      final RenderShrinkWrappingViewport renderViewport = RenderShrinkWrappingViewport(
        crossAxisDirection: AxisDirection.right,
        offset: ViewportOffset.zero(),
        children: children,
      );

      // Children should be painted in reverse order to the list given
      expect(renderViewport.childrenInPaintOrder, equals(children.reversed));
      // childrenInPaintOrder should be reverse of childrenInHitTestOrder
      expect(
        renderViewport.childrenInPaintOrder,
        equals(renderViewport.childrenInHitTestOrder.toList().reversed),
      );
    });
  });

  group('Overscrolling RenderShrinkWrappingViewport', () {
    Widget buildSimpleShrinkWrap({
      ScrollController? controller,
      Axis scrollDirection = Axis.vertical,
      ScrollPhysics? physics,
    }) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(),
          child: ListView.builder(
            controller: controller,
            physics: physics,
            scrollDirection: scrollDirection,
            shrinkWrap: true,
            itemBuilder:
                (BuildContext context, int index) =>
                    SizedBox(height: 50, width: 50, child: Text('Item $index')),
            itemCount: 20,
            itemExtent: 50,
          ),
        ),
      );
    }

    Widget buildClippingShrinkWrap(ScrollController controller, {bool constrain = false}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(),
          child: ColoredBox(
            color: const Color(0xFF000000),
            child: Column(
              children: <Widget>[
                // Translucent boxes above and below the shrinkwrapped viewport
                // make it easily discernible if the viewport is not being
                // clipped properly.
                Opacity(
                  opacity: 0.5,
                  child: Container(height: 100, color: const Color(0xFF00B0FF)),
                ),
                Container(
                  height: constrain ? 150 : null,
                  color: const Color(0xFFF44336),
                  child: ListView.builder(
                    controller: controller,
                    shrinkWrap: true,
                    physics: const BouncingScrollPhysics(parent: AlwaysScrollableScrollPhysics()),
                    itemBuilder: (BuildContext context, int index) => Text('Item $index'),
                    itemCount: 10,
                  ),
                ),
                Opacity(
                  opacity: 0.5,
                  child: Container(height: 100, color: const Color(0xFF00B0FF)),
                ),
              ],
            ),
          ),
        ),
      );
    }

    testWidgets('constrained viewport correctly clips overflow', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/89717
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);

      await tester.pumpWidget(buildClippingShrinkWrap(controller, constrain: true));
      expect(controller.offset, 0.0);
      expect(tester.getTopLeft(find.text('Item 0')).dy, 100.0);
      expect(tester.getTopLeft(find.text('Item 9')).dy, 226.0);

      // Overscroll
      final TestGesture overscrollGesture = await tester.startGesture(
        tester.getCenter(find.text('Item 0')),
      );
      await overscrollGesture.moveBy(const Offset(0, 100));
      await tester.pump();
      expect(controller.offset, -100.0);
      expect(tester.getTopLeft(find.text('Item 0')).dy, 200.0);
      await expectLater(
        find.byType(Directionality),
        matchesGoldenFile('shrinkwrap_clipped_constrained_overscroll.png'),
      );
      await overscrollGesture.up();
      await tester.pumpAndSettle();
      expect(controller.offset, 0.0);
      expect(tester.getTopLeft(find.text('Item 0')).dy, 100.0);
      expect(tester.getTopLeft(find.text('Item 9')).dy, 226.0);
    });

    testWidgets('correctly clips overflow without constraints', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/89717
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);

      await tester.pumpWidget(buildClippingShrinkWrap(controller));
      expect(controller.offset, 0.0);
      expect(tester.getTopLeft(find.text('Item 0')).dy, 100.0);
      expect(tester.getTopLeft(find.text('Item 9')).dy, 226.0);

      // Overscroll
      final TestGesture overscrollGesture = await tester.startGesture(
        tester.getCenter(find.text('Item 0')),
      );
      await overscrollGesture.moveBy(const Offset(0, 100));
      await tester.pump();
      expect(controller.offset, -100.0);
      expect(tester.getTopLeft(find.text('Item 0')).dy, 200.0);
      await expectLater(
        find.byType(Directionality),
        matchesGoldenFile('shrinkwrap_clipped_overscroll.png'),
      );
      await overscrollGesture.up();
      await tester.pumpAndSettle();
      expect(controller.offset, 0.0);
      expect(tester.getTopLeft(find.text('Item 0')).dy, 100.0);
      expect(tester.getTopLeft(find.text('Item 9')).dy, 226.0);
    });

    testWidgets(
      'allows overscrolling on default platforms - vertical',
      (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/10949
        // Scrollables should overscroll by default on iOS and macOS
        final ScrollController controller = ScrollController();
        addTearDown(controller.dispose);

        await tester.pumpWidget(buildSimpleShrinkWrap(controller: controller));
        expect(controller.offset, 0.0);
        expect(tester.getTopLeft(find.text('Item 0')).dy, 0.0);
        // Check overscroll at both ends
        // Start
        TestGesture overscrollGesture = await tester.startGesture(
          tester.getCenter(find.byType(ListView)),
        );
        await overscrollGesture.moveBy(const Offset(0, 25));
        await tester.pump();
        expect(controller.offset, -25.0);
        expect(tester.getTopLeft(find.text('Item 0')).dy, 25.0);
        await overscrollGesture.up();
        await tester.pumpAndSettle();
        expect(controller.offset, 0.0);
        expect(tester.getTopLeft(find.text('Item 0')).dy, 0.0);

        // End
        final double maxExtent = controller.position.maxScrollExtent;
        controller.jumpTo(controller.position.maxScrollExtent);
        await tester.pumpAndSettle();
        expect(controller.offset, maxExtent);
        expect(tester.getBottomLeft(find.text('Item 19')).dy, 600.0);

        overscrollGesture = await tester.startGesture(tester.getCenter(find.byType(ListView)));
        await overscrollGesture.moveBy(const Offset(0, -25));
        await tester.pump();
        expect(controller.offset, greaterThan(maxExtent));
        expect(tester.getBottomLeft(find.text('Item 19')).dy, 575.0);
        await overscrollGesture.up();
        await tester.pumpAndSettle();
        expect(controller.offset, maxExtent);
        expect(tester.getBottomLeft(find.text('Item 19')).dy, 600.0);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'allows overscrolling on default platforms - horizontal',
      (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/10949
        // Scrollables should overscroll by default on iOS and macOS
        final ScrollController controller = ScrollController();
        addTearDown(controller.dispose);

        await tester.pumpWidget(
          buildSimpleShrinkWrap(controller: controller, scrollDirection: Axis.horizontal),
        );
        expect(controller.offset, 0.0);
        expect(tester.getTopLeft(find.text('Item 0')).dx, 0.0);
        // Check overscroll at both ends
        // Start
        TestGesture overscrollGesture = await tester.startGesture(
          tester.getCenter(find.byType(ListView)),
        );
        await overscrollGesture.moveBy(const Offset(25, 0));
        await tester.pump();
        expect(controller.offset, -25.0);
        expect(tester.getTopLeft(find.text('Item 0')).dx, 25.0);
        await overscrollGesture.up();
        await tester.pumpAndSettle();
        expect(controller.offset, 0.0);
        expect(tester.getTopLeft(find.text('Item 0')).dx, 0.0);

        // End
        final double maxExtent = controller.position.maxScrollExtent;
        controller.jumpTo(controller.position.maxScrollExtent);
        await tester.pumpAndSettle();
        expect(controller.offset, maxExtent);
        expect(tester.getTopRight(find.text('Item 19')).dx, 800.0);

        overscrollGesture = await tester.startGesture(tester.getCenter(find.byType(ListView)));
        await overscrollGesture.moveBy(const Offset(-25, 0));
        await tester.pump();
        expect(controller.offset, greaterThan(maxExtent));
        expect(tester.getTopRight(find.text('Item 19')).dx, 775.0);
        await overscrollGesture.up();
        await tester.pumpAndSettle();
        expect(controller.offset, maxExtent);
        expect(tester.getTopRight(find.text('Item 19')).dx, 800.0);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets('allows overscrolling per physics - vertical', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/10949
      // Scrollables should overscroll when the scroll physics allow
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);

      await tester.pumpWidget(
        buildSimpleShrinkWrap(controller: controller, physics: const BouncingScrollPhysics()),
      );
      expect(controller.offset, 0.0);
      expect(tester.getTopLeft(find.text('Item 0')).dy, 0.0);
      // Check overscroll at both ends
      // Start
      TestGesture overscrollGesture = await tester.startGesture(
        tester.getCenter(find.byType(ListView)),
      );
      await overscrollGesture.moveBy(const Offset(0, 25));
      await tester.pump();
      expect(controller.offset, -25.0);
      expect(tester.getTopLeft(find.text('Item 0')).dy, 25.0);
      await overscrollGesture.up();
      await tester.pumpAndSettle();
      expect(controller.offset, 0.0);
      expect(tester.getTopLeft(find.text('Item 0')).dy, 0.0);

      // End
      final double maxExtent = controller.position.maxScrollExtent;
      controller.jumpTo(controller.position.maxScrollExtent);
      await tester.pumpAndSettle();
      expect(controller.offset, maxExtent);
      expect(tester.getBottomLeft(find.text('Item 19')).dy, 600.0);

      overscrollGesture = await tester.startGesture(tester.getCenter(find.byType(ListView)));
      await overscrollGesture.moveBy(const Offset(0, -25));
      await tester.pump();
      expect(controller.offset, greaterThan(maxExtent));
      expect(tester.getBottomLeft(find.text('Item 19')).dy, 575.0);
      await overscrollGesture.up();
      await tester.pumpAndSettle();
      expect(controller.offset, maxExtent);
      expect(tester.getBottomLeft(find.text('Item 19')).dy, 600.0);
    });

    testWidgets('allows overscrolling per physics - horizontal', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/10949
      // Scrollables should overscroll when the scroll physics allow
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);

      await tester.pumpWidget(
        buildSimpleShrinkWrap(
          controller: controller,
          scrollDirection: Axis.horizontal,
          physics: const BouncingScrollPhysics(),
        ),
      );
      expect(controller.offset, 0.0);
      expect(tester.getTopLeft(find.text('Item 0')).dx, 0.0);
      // Check overscroll at both ends
      // Start
      TestGesture overscrollGesture = await tester.startGesture(
        tester.getCenter(find.byType(ListView)),
      );
      await overscrollGesture.moveBy(const Offset(25, 0));
      await tester.pump();
      expect(controller.offset, -25.0);
      expect(tester.getTopLeft(find.text('Item 0')).dx, 25.0);
      await overscrollGesture.up();
      await tester.pumpAndSettle();
      expect(controller.offset, 0.0);
      expect(tester.getTopLeft(find.text('Item 0')).dx, 0.0);

      // End
      final double maxExtent = controller.position.maxScrollExtent;
      controller.jumpTo(controller.position.maxScrollExtent);
      await tester.pumpAndSettle();
      expect(controller.offset, maxExtent);
      expect(tester.getTopRight(find.text('Item 19')).dx, 800.0);

      overscrollGesture = await tester.startGesture(tester.getCenter(find.byType(ListView)));
      await overscrollGesture.moveBy(const Offset(-25, 0));
      await tester.pump();
      expect(controller.offset, greaterThan(maxExtent));
      expect(tester.getTopRight(find.text('Item 19')).dx, 775.0);
      await overscrollGesture.up();
      await tester.pumpAndSettle();
      expect(controller.offset, maxExtent);
      expect(tester.getTopRight(find.text('Item 19')).dx, 800.0);
    });
  });

  testWidgets(
    'Handles infinite constraints when TargetPlatform is iOS or macOS',
    (WidgetTester tester) async {
      // regression test for https://github.com/flutter/flutter/issues/45866
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: const MediaQueryData(),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                GridView(
                  shrinkWrap: true,
                  gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
                    crossAxisCount: 3,
                    childAspectRatio: 3,
                    mainAxisSpacing: 3,
                    crossAxisSpacing: 3,
                  ),
                  children: const <Widget>[Text('a'), Text('b'), Text('c')],
                ),
              ],
            ),
          ),
        ),
      );

      expect(find.text('b'), findsOneWidget);
      await tester.drag(find.text('b'), const Offset(0, 200));
      await tester.pumpAndSettle();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('Viewport describeApproximateClip respects clipBehavior', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          clipBehavior: Clip.none,
          slivers: <Widget>[SliverToBoxAdapter(child: SizedBox(width: 20, height: 20))],
        ),
      ),
    );
    RenderViewport viewport = tester.allRenderObjects.whereType<RenderViewport>().first;
    expect(viewport.clipBehavior, Clip.none);
    bool visited = false;
    viewport.visitChildren((RenderObject child) {
      visited = true;
      expect(viewport.describeApproximatePaintClip(child as RenderSliver), null);
    });
    expect(visited, true);

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[SliverToBoxAdapter(child: SizedBox(width: 20, height: 20))],
        ),
      ),
    );
    viewport = tester.allRenderObjects.whereType<RenderViewport>().first;
    expect(viewport.clipBehavior, Clip.hardEdge);
    visited = false;
    viewport.visitChildren((RenderObject child) {
      visited = true;
      expect(
        viewport.describeApproximatePaintClip(child as RenderSliver),
        Offset.zero & viewport.size,
      );
    });
    expect(visited, true);
  });

  testWidgets('Shrinkwrapping viewport asserts bounded cross axis', (WidgetTester tester) async {
    final List<FlutterErrorDetails> errors = <FlutterErrorDetails>[];
    FlutterError.onError = (FlutterErrorDetails error) => errors.add(error);
    // Vertical
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          scrollDirection: Axis.horizontal,
          children: <Widget>[
            ListView(shrinkWrap: true, children: const <Widget>[SizedBox.square(dimension: 500)]),
          ],
        ),
      ),
    );

    expect(errors, isNotEmpty);
    expect(errors.first.exception, isFlutterError);
    FlutterError error = errors.first.exception as FlutterError;
    expect(
      error.toString(),
      contains('Viewports expand in the cross axis to fill their container'),
    );
    errors.clear();

    // Horizontal
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          children: <Widget>[
            ListView(
              scrollDirection: Axis.horizontal,
              shrinkWrap: true,
              children: const <Widget>[SizedBox.square(dimension: 500)],
            ),
          ],
        ),
      ),
    );

    expect(errors, isNotEmpty);
    expect(errors.first.exception, isFlutterError);
    error = errors.first.exception as FlutterError;
    expect(
      error.toString(),
      contains('Viewports expand in the cross axis to fill their container'),
    );
    errors.clear();

    // No children
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          scrollDirection: Axis.horizontal,
          children: <Widget>[ListView(shrinkWrap: true)],
        ),
      ),
    );

    expect(errors, isNotEmpty);
    expect(errors.first.exception, isFlutterError);
    error = errors.first.exception as FlutterError;
    expect(
      error.toString(),
      contains('Viewports expand in the cross axis to fill their container'),
    );
    errors.clear();
  });

  testWidgets('RenderViewport maxLayoutCycles depends on the number of children', (
    WidgetTester tester,
  ) async {
    Future<void> expectFlutterError({required Widget widget, required WidgetTester tester}) async {
      final List<FlutterErrorDetails> errors = <FlutterErrorDetails>[];
      final FlutterExceptionHandler? oldHandler = FlutterError.onError;
      FlutterError.onError = (FlutterErrorDetails error) => errors.add(error);
      try {
        await tester.pumpWidget(widget);
      } finally {
        FlutterError.onError = oldHandler;
      }
      expect(errors, isNotEmpty);
      expect(errors.first.exception, isFlutterError);
    }

    Widget buildWidget({required int sliverCount, required int correctionsCount}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: List<Widget>.generate(
            sliverCount,
            (_) => _ScrollOffsetCorrectionSliver(correctionsCount: correctionsCount),
          ),
        ),
      );
    }

    // 5 correction per child will pass.
    await tester.pumpWidget(buildWidget(sliverCount: 30, correctionsCount: 5));

    // 15 correction per child will throw exception.
    await expectFlutterError(
      widget: buildWidget(sliverCount: 1, correctionsCount: 15),
      tester: tester,
    );
  });
}

// Simple sliver that applies N scroll offset corrections.
class _RenderScrollOffsetCorrectionSliver extends RenderSliver {
  int _correctionCount = 0;
  @override
  void performLayout() {
    if (_correctionCount > 0) {
      --_correctionCount;
      geometry = const SliverGeometry(scrollOffsetCorrection: 1.0);
      return;
    }
    const double extent = 5;
    final double paintedChildSize = calculatePaintOffset(constraints, from: 0.0, to: extent);
    final double cacheExtent = calculateCacheOffset(constraints, from: 0.0, to: extent);

    geometry = SliverGeometry(
      scrollExtent: extent,
      paintExtent: paintedChildSize,
      maxPaintExtent: extent,
      cacheExtent: cacheExtent,
    );
  }
}

class _ScrollOffsetCorrectionSliver extends SingleChildRenderObjectWidget {
  const _ScrollOffsetCorrectionSliver({required this.correctionsCount});
  final int correctionsCount;

  @override
  _RenderScrollOffsetCorrectionSliver createRenderObject(BuildContext context) {
    final _RenderScrollOffsetCorrectionSliver sliver = _RenderScrollOffsetCorrectionSliver();
    sliver._correctionCount = correctionsCount;
    return sliver;
  }

  @override
  void updateRenderObject(
    BuildContext context,
    covariant _RenderScrollOffsetCorrectionSliver renderObject,
  ) {
    super.updateRenderObject(context, renderObject);
    renderObject.markNeedsLayout();
    renderObject._correctionCount = correctionsCount;
  }
}
