// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('LayoutBuilder parent size', (WidgetTester tester) async {
    late Size layoutBuilderSize;
    final Key childKey = UniqueKey();
    final Key parentKey = UniqueKey();

    await tester.pumpWidget(
      Center(
        child: ConstrainedBox(
          constraints: const BoxConstraints(maxWidth: 100.0, maxHeight: 200.0),
          child: LayoutBuilder(
            key: parentKey,
            builder: (BuildContext context, BoxConstraints constraints) {
              layoutBuilderSize = constraints.biggest;
              return SizedBox(
                key: childKey,
                width: layoutBuilderSize.width / 2.0,
                height: layoutBuilderSize.height / 2.0,
              );
            },
          ),
        ),
      ),
    );

    expect(layoutBuilderSize, const Size(100.0, 200.0));
    final RenderBox parentBox = tester.renderObject(find.byKey(parentKey));
    expect(parentBox.size, equals(const Size(50.0, 100.0)));
    final RenderBox childBox = tester.renderObject(find.byKey(childKey));
    expect(childBox.size, equals(const Size(50.0, 100.0)));
  });

  testWidgets('SliverLayoutBuilder parent geometry', (WidgetTester tester) async {
    late SliverConstraints parentConstraints1;
    late SliverConstraints parentConstraints2;
    final Key childKey1 = UniqueKey();
    final Key parentKey1 = UniqueKey();
    final Key childKey2 = UniqueKey();
    final Key parentKey2 = UniqueKey();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[
            SliverLayoutBuilder(
              key: parentKey1,
              builder: (BuildContext context, SliverConstraints constraint) {
                parentConstraints1 = constraint;
                return SliverPadding(
                  key: childKey1,
                  padding: const EdgeInsets.fromLTRB(1, 2, 3, 4),
                );
              },
            ),
            SliverLayoutBuilder(
              key: parentKey2,
              builder: (BuildContext context, SliverConstraints constraint) {
                parentConstraints2 = constraint;
                return SliverPadding(
                  key: childKey2,
                  padding: const EdgeInsets.fromLTRB(5, 7, 11, 13),
                );
              },
            ),
          ],
        ),
      ),
    );

    expect(parentConstraints1.crossAxisExtent, 800);
    expect(parentConstraints1.remainingPaintExtent, 600);

    expect(parentConstraints2.crossAxisExtent, 800);
    expect(parentConstraints2.remainingPaintExtent, 600 - 2 - 4);
    final RenderSliver parentSliver1 = tester.renderObject(find.byKey(parentKey1));
    final RenderSliver parentSliver2 = tester.renderObject(find.byKey(parentKey2));

    // scrollExtent == top + bottom.
    expect(parentSliver1.geometry!.scrollExtent, 2 + 4);
    expect(parentSliver2.geometry!.scrollExtent, 7 + 13);

    final RenderSliver childSliver1 = tester.renderObject(find.byKey(childKey1));
    final RenderSliver childSliver2 = tester.renderObject(find.byKey(childKey2));
    expect(childSliver1.geometry, parentSliver1.geometry);
    expect(childSliver2.geometry, parentSliver2.geometry);
  });

  testWidgets('LayoutBuilder stateful child', (WidgetTester tester) async {
    late Size layoutBuilderSize;
    late StateSetter setState;
    final Key childKey = UniqueKey();
    final Key parentKey = UniqueKey();
    double childWidth = 10.0;
    double childHeight = 20.0;

    await tester.pumpWidget(
      Center(
        child: LayoutBuilder(
          key: parentKey,
          builder: (BuildContext context, BoxConstraints constraints) {
            layoutBuilderSize = constraints.biggest;
            return StatefulBuilder(
              builder: (BuildContext context, StateSetter setter) {
                setState = setter;
                return SizedBox(key: childKey, width: childWidth, height: childHeight);
              },
            );
          },
        ),
      ),
    );

    expect(layoutBuilderSize, equals(const Size(800.0, 600.0)));
    RenderBox parentBox = tester.renderObject(find.byKey(parentKey));
    expect(parentBox.size, equals(const Size(10.0, 20.0)));
    RenderBox childBox = tester.renderObject(find.byKey(childKey));
    expect(childBox.size, equals(const Size(10.0, 20.0)));

    setState(() {
      childWidth = 100.0;
      childHeight = 200.0;
    });
    await tester.pump();
    parentBox = tester.renderObject(find.byKey(parentKey));
    expect(parentBox.size, equals(const Size(100.0, 200.0)));
    childBox = tester.renderObject(find.byKey(childKey));
    expect(childBox.size, equals(const Size(100.0, 200.0)));
  });

  testWidgets('SliverLayoutBuilder stateful descendants', (WidgetTester tester) async {
    late StateSetter setState;
    double childWidth = 10.0;
    double childHeight = 20.0;
    final Key parentKey = UniqueKey();
    final Key childKey = UniqueKey();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[
            SliverLayoutBuilder(
              key: parentKey,
              builder: (BuildContext context, SliverConstraints constraint) {
                return SliverToBoxAdapter(
                  child: StatefulBuilder(
                    builder: (BuildContext context, StateSetter setter) {
                      setState = setter;
                      return SizedBox(key: childKey, width: childWidth, height: childHeight);
                    },
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );

    RenderBox childBox = tester.renderObject(find.byKey(childKey));
    RenderSliver parentSliver = tester.renderObject(find.byKey(parentKey));
    expect(childBox.size.width, 800);
    expect(childBox.size.height, childHeight);
    expect(parentSliver.geometry!.scrollExtent, childHeight);
    expect(parentSliver.geometry!.paintExtent, childHeight);

    setState(() {
      childWidth = 100.0;
      childHeight = 200.0;
    });

    await tester.pump();
    childBox = tester.renderObject(find.byKey(childKey));
    parentSliver = tester.renderObject(find.byKey(parentKey));
    expect(childBox.size.width, 800);
    expect(childBox.size.height, childHeight);
    expect(parentSliver.geometry!.scrollExtent, childHeight);
    expect(parentSliver.geometry!.paintExtent, childHeight);

    // Make child wider and higher than the viewport.
    setState(() {
      childWidth = 900.0;
      childHeight = 900.0;
    });

    await tester.pump();
    childBox = tester.renderObject(find.byKey(childKey));
    parentSliver = tester.renderObject(find.byKey(parentKey));
    expect(childBox.size.width, 800);
    expect(childBox.size.height, childHeight);
    expect(parentSliver.geometry!.scrollExtent, childHeight);
    expect(parentSliver.geometry!.paintExtent, 600);
  });

  testWidgets('LayoutBuilder stateful parent', (WidgetTester tester) async {
    late Size layoutBuilderSize;
    late StateSetter setState;
    final Key childKey = UniqueKey();
    double childWidth = 10.0;
    double childHeight = 20.0;

    await tester.pumpWidget(
      Center(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setter) {
            setState = setter;
            return SizedBox(
              width: childWidth,
              height: childHeight,
              child: LayoutBuilder(
                builder: (BuildContext context, BoxConstraints constraints) {
                  layoutBuilderSize = constraints.biggest;
                  return SizedBox(
                    key: childKey,
                    width: layoutBuilderSize.width,
                    height: layoutBuilderSize.height,
                  );
                },
              ),
            );
          },
        ),
      ),
    );

    expect(layoutBuilderSize, equals(const Size(10.0, 20.0)));
    RenderBox box = tester.renderObject(find.byKey(childKey));
    expect(box.size, equals(const Size(10.0, 20.0)));

    setState(() {
      childWidth = 100.0;
      childHeight = 200.0;
    });
    await tester.pump();
    box = tester.renderObject(find.byKey(childKey));
    expect(box.size, equals(const Size(100.0, 200.0)));
  });

  testWidgets('LayoutBuilder and Inherited -- do not rebuild when not using inherited', (
    WidgetTester tester,
  ) async {
    int built = 0;
    final Widget target = LayoutBuilder(
      builder: (BuildContext context, BoxConstraints constraints) {
        built += 1;
        return Container();
      },
    );
    expect(built, 0);

    await tester.pumpWidget(
      MediaQuery(data: const MediaQueryData(size: Size(400.0, 300.0)), child: target),
    );
    expect(built, 1);

    await tester.pumpWidget(
      MediaQuery(data: const MediaQueryData(size: Size(300.0, 400.0)), child: target),
    );
    expect(built, 1);
  });

  testWidgets('LayoutBuilder and Inherited -- do rebuild when using inherited', (
    WidgetTester tester,
  ) async {
    int built = 0;
    final Widget target = LayoutBuilder(
      builder: (BuildContext context, BoxConstraints constraints) {
        built += 1;
        MediaQuery.of(context);
        return Container();
      },
    );
    expect(built, 0);

    await tester.pumpWidget(
      MediaQuery(data: const MediaQueryData(size: Size(400.0, 300.0)), child: target),
    );
    expect(built, 1);

    await tester.pumpWidget(
      MediaQuery(data: const MediaQueryData(size: Size(300.0, 400.0)), child: target),
    );
    expect(built, 2);
  });

  testWidgets('LayoutBuilder rebuilds once in the same frame', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/146379.
    int built = 0;
    final Widget target = LayoutBuilder(
      builder: (BuildContext context, BoxConstraints constraints) {
        return Builder(
          builder: (BuildContext context) {
            built += 1;
            MediaQuery.of(context);
            return const Placeholder();
          },
        );
      },
    );
    expect(built, 0);

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(size: Size(400.0, 300.0)),
        child: Center(child: SizedBox(width: 400.0, child: target)),
      ),
    );
    expect(built, 1);

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(size: Size(300.0, 400.0)),
        child: Center(child: SizedBox(width: 300.0, child: target)),
      ),
    );
    expect(built, 2);
  });

  testWidgets('LayoutBuilder does not dirty the render tree during the idle phase', (
    WidgetTester tester,
  ) async {
    RenderObject? dirtyRenderObject;
    void visitSubtree(RenderObject node) {
      assert(dirtyRenderObject == null);
      if (node.debugNeedsLayout) {
        dirtyRenderObject = node;
        return;
      }
      node.visitChildren(visitSubtree);
    }

    final Widget target = LayoutBuilder(
      builder: (BuildContext context, BoxConstraints constraints) => const Placeholder(),
    );
    await tester.pumpWidget(target);
    final RenderObject renderObject = tester.renderObject(find.byWidget(target));
    visitSubtree(renderObject);
    expect(dirtyRenderObject, isNull);

    tester.element(find.byType(Placeholder)).markNeedsBuild();
    visitSubtree(renderObject);
    expect(dirtyRenderObject, isNull);
  });

  testWidgets('LayoutBuilder can change size without rebuild', (WidgetTester tester) async {
    int built = 0;
    final Widget target = LayoutBuilder(
      builder: (BuildContext context, BoxConstraints constraints) {
        return Builder(
          builder: (BuildContext context) {
            built += 1;
            return const Text('A');
          },
        );
      },
    );
    expect(built, 0);

    await tester.pumpWidget(
      Center(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: DefaultTextStyle(style: const TextStyle(fontSize: 10), child: target),
        ),
      ),
    );
    expect(built, 1);
    expect(tester.getSize(find.byWidget(target)), const Size(10, 10));

    await tester.pumpWidget(
      Center(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: DefaultTextStyle(style: const TextStyle(fontSize: 100), child: target),
        ),
      ),
    );
    expect(built, 1);
    expect(tester.getSize(find.byWidget(target)), const Size(100, 100));
  });

  testWidgets('SliverLayoutBuilder and Inherited -- do not rebuild when not using inherited', (
    WidgetTester tester,
  ) async {
    int built = 0;
    final Widget target = Directionality(
      textDirection: TextDirection.ltr,
      child: CustomScrollView(
        slivers: <Widget>[
          SliverLayoutBuilder(
            builder: (BuildContext context, SliverConstraints constraint) {
              built++;
              return SliverToBoxAdapter(child: Container());
            },
          ),
        ],
      ),
    );

    expect(built, 0);

    await tester.pumpWidget(
      MediaQuery(data: const MediaQueryData(size: Size(400.0, 300.0)), child: target),
    );
    expect(built, 1);

    await tester.pumpWidget(
      MediaQuery(data: const MediaQueryData(size: Size(300.0, 400.0)), child: target),
    );
    expect(built, 1);
  });

  testWidgets('SliverLayoutBuilder and Inherited -- do rebuild when not using inherited', (
    WidgetTester tester,
  ) async {
    int built = 0;
    final Widget target = Directionality(
      textDirection: TextDirection.ltr,
      child: CustomScrollView(
        slivers: <Widget>[
          SliverLayoutBuilder(
            builder: (BuildContext context, SliverConstraints constraint) {
              built++;
              MediaQuery.of(context);
              return SliverToBoxAdapter(child: Container());
            },
          ),
        ],
      ),
    );

    expect(built, 0);

    await tester.pumpWidget(
      MediaQuery(data: const MediaQueryData(size: Size(400.0, 300.0)), child: target),
    );
    expect(built, 1);

    await tester.pumpWidget(
      MediaQuery(data: const MediaQueryData(size: Size(300.0, 400.0)), child: target),
    );
    expect(built, 2);
  });

  testWidgets('nested SliverLayoutBuilder', (WidgetTester tester) async {
    late SliverConstraints parentConstraints1;
    late SliverConstraints parentConstraints2;
    final Key childKey = UniqueKey();
    final Key parentKey1 = UniqueKey();
    final Key parentKey2 = UniqueKey();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[
            SliverLayoutBuilder(
              key: parentKey1,
              builder: (BuildContext context, SliverConstraints constraint) {
                parentConstraints1 = constraint;
                return SliverLayoutBuilder(
                  key: parentKey2,
                  builder: (BuildContext context, SliverConstraints constraint) {
                    parentConstraints2 = constraint;
                    return SliverPadding(
                      key: childKey,
                      padding: const EdgeInsets.fromLTRB(1, 2, 3, 4),
                    );
                  },
                );
              },
            ),
          ],
        ),
      ),
    );

    expect(parentConstraints1, parentConstraints2);

    expect(parentConstraints1.crossAxisExtent, 800);
    expect(parentConstraints1.remainingPaintExtent, 600);

    final RenderSliver parentSliver1 = tester.renderObject(find.byKey(parentKey1));
    final RenderSliver parentSliver2 = tester.renderObject(find.byKey(parentKey2));
    // scrollExtent == top + bottom.
    expect(parentSliver1.geometry!.scrollExtent, 2 + 4);

    final RenderSliver childSliver = tester.renderObject(find.byKey(childKey));
    expect(childSliver.geometry, parentSliver1.geometry);
    expect(parentSliver1.geometry, parentSliver2.geometry);
  });

  testWidgets('localToGlobal works with SliverLayoutBuilder', (WidgetTester tester) async {
    final Key childKey1 = UniqueKey();
    final Key childKey2 = UniqueKey();
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          controller: scrollController,
          slivers: <Widget>[
            const SliverToBoxAdapter(child: SizedBox(height: 300)),
            SliverLayoutBuilder(
              builder:
                  (BuildContext context, SliverConstraints constraint) =>
                      SliverToBoxAdapter(child: SizedBox(key: childKey1, height: 200)),
            ),
            SliverToBoxAdapter(child: SizedBox(key: childKey2, height: 100)),
          ],
        ),
      ),
    );

    final RenderBox renderChild1 = tester.renderObject(find.byKey(childKey1));
    final RenderBox renderChild2 = tester.renderObject(find.byKey(childKey2));

    // Test with scrollController.scrollOffset = 0.
    expect(renderChild1.localToGlobal(const Offset(100, 100)), const Offset(100, 300.0 + 100));

    expect(
      renderChild2.localToGlobal(const Offset(100, 100)),
      const Offset(100, 300.0 + 200 + 100),
    );

    scrollController.jumpTo(100);
    await tester.pump();
    expect(
      renderChild1.localToGlobal(const Offset(100, 100)),
      // -100 because the scroll offset is now 100.
      const Offset(100, 300.0 + 100 - 100),
    );

    expect(
      renderChild2.localToGlobal(const Offset(100, 100)),
      // -100 because the scroll offset is now 100.
      const Offset(100, 300.0 + 100 + 200 - 100),
    );
  });

  testWidgets('hitTest works within SliverLayoutBuilder', (WidgetTester tester) async {
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);
    List<int> hitCounts = <int>[0, 0, 0];

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Padding(
          padding: const EdgeInsets.all(50),
          child: CustomScrollView(
            controller: scrollController,
            slivers: <Widget>[
              SliverToBoxAdapter(
                child: SizedBox(height: 200, child: GestureDetector(onTap: () => hitCounts[0]++)),
              ),
              SliverLayoutBuilder(
                builder:
                    (BuildContext context, SliverConstraints constraint) => SliverToBoxAdapter(
                      child: SizedBox(
                        height: 200,
                        child: GestureDetector(onTap: () => hitCounts[1]++),
                      ),
                    ),
              ),
              SliverToBoxAdapter(
                child: SizedBox(height: 200, child: GestureDetector(onTap: () => hitCounts[2]++)),
              ),
            ],
          ),
        ),
      ),
    );

    // Tap item 1.
    await tester.tapAt(const Offset(300, 50.0 + 100));
    await tester.pump();
    expect(hitCounts, const <int>[1, 0, 0]);

    // Tap item 2.
    await tester.tapAt(const Offset(300, 50.0 + 100 + 200));
    await tester.pump();
    expect(hitCounts, const <int>[1, 1, 0]);

    // Tap item 3. Shift the touch point up to ensure the touch lands within the viewport.
    await tester.tapAt(const Offset(300, 50.0 + 200 + 200 + 10));
    await tester.pump();
    expect(hitCounts, const <int>[1, 1, 1]);

    // Scrolling doesn't break it.
    hitCounts = <int>[0, 0, 0];
    scrollController.jumpTo(100);
    await tester.pump();

    // Tap item 1.
    await tester.tapAt(const Offset(300, 50.0 + 100 - 100));
    await tester.pump();
    expect(hitCounts, const <int>[1, 0, 0]);

    // Tap item 2.
    await tester.tapAt(const Offset(300, 50.0 + 100 + 200 - 100));
    await tester.pump();
    expect(hitCounts, const <int>[1, 1, 0]);

    // Tap item 3.
    await tester.tapAt(const Offset(300, 50.0 + 100 + 200 + 200 - 100));
    await tester.pump();
    expect(hitCounts, const <int>[1, 1, 1]);

    // Tapping outside of the viewport shouldn't do anything.
    await tester.tapAt(const Offset(300, 1));
    await tester.pump();
    expect(hitCounts, const <int>[1, 1, 1]);

    await tester.tapAt(const Offset(300, 599));
    await tester.pump();
    expect(hitCounts, const <int>[1, 1, 1]);

    await tester.tapAt(const Offset(1, 100));
    await tester.pump();
    expect(hitCounts, const <int>[1, 1, 1]);

    await tester.tapAt(const Offset(799, 100));
    await tester.pump();
    expect(hitCounts, const <int>[1, 1, 1]);

    // Tap the no-content area in the viewport shouldn't do anything
    hitCounts = <int>[0, 0, 0];
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          controller: scrollController,
          slivers: <Widget>[
            SliverToBoxAdapter(
              child: SizedBox(height: 100, child: GestureDetector(onTap: () => hitCounts[0]++)),
            ),
            SliverLayoutBuilder(
              builder:
                  (BuildContext context, SliverConstraints constraint) => SliverToBoxAdapter(
                    child: SizedBox(
                      height: 100,
                      child: GestureDetector(onTap: () => hitCounts[1]++),
                    ),
                  ),
            ),
            SliverToBoxAdapter(
              child: SizedBox(height: 100, child: GestureDetector(onTap: () => hitCounts[2]++)),
            ),
          ],
        ),
      ),
    );

    await tester.tapAt(const Offset(300, 301));
    await tester.pump();
    expect(hitCounts, const <int>[0, 0, 0]);
  });

  testWidgets(
    'LayoutBuilder does not call builder when layout happens but layout constraints do not change',
    (WidgetTester tester) async {
      int builderInvocationCount = 0;

      Future<void> pumpTestWidget(Size size) async {
        await tester.pumpWidget(
          // Center is used to give the SizedBox the power to determine constraints for LayoutBuilder
          Center(
            child: SizedBox.fromSize(
              size: size,
              child: LayoutBuilder(
                builder: (BuildContext context, BoxConstraints constraints) {
                  builderInvocationCount += 1;
                  return const _LayoutSpy();
                },
              ),
            ),
          ),
        );
      }

      await pumpTestWidget(const Size(10, 10));

      final _RenderLayoutSpy spy = tester.renderObject(find.byType(_LayoutSpy));

      // The child is laid out once the first time.
      expect(spy.performLayoutCount, 1);
      expect(spy.performResizeCount, 1);

      // The initial `pumpWidget` will trigger `performRebuild`, asking for
      // builder invocation.
      expect(builderInvocationCount, 1);

      // Invalidate the layout without changing the constraints.
      tester.renderObject(find.byType(LayoutBuilder)).markNeedsLayout();

      // The second pump will not go through the `performRebuild` or `update`, and
      // only judge the need for builder invocation based on constraints, which
      // didn't change, so we don't expect any counters to go up.
      await tester.pump();
      expect(builderInvocationCount, 1);
      expect(spy.performLayoutCount, 1);
      expect(spy.performResizeCount, 1);

      // Cause the `update` to be called (but not `performRebuild`), triggering
      // builder invocation.
      await pumpTestWidget(const Size(10, 10));
      expect(builderInvocationCount, 2);

      // The spy does not invalidate its layout on widget update, so no
      // layout-related methods should be called.
      expect(spy.performLayoutCount, 1);
      expect(spy.performResizeCount, 1);

      // Have the child request layout and verify that the child gets laid out
      // despite layout constraints remaining constant.
      spy.markNeedsLayout();
      await tester.pump();

      // Builder is not invoked. This was a layout-only pump with the same parent
      // constraints.
      expect(builderInvocationCount, 2);

      // Expect performLayout to be called.
      expect(spy.performLayoutCount, 2);

      // performResize should not be called because the spy sets sizedByParent,
      // and the constraints did not change.
      expect(spy.performResizeCount, 1);

      // Change the parent size, triggering constraint change.
      await pumpTestWidget(const Size(20, 20));

      // We should see everything invoked once.
      expect(builderInvocationCount, 3);
      expect(spy.performLayoutCount, 3);
      expect(spy.performResizeCount, 2);
    },
  );

  testWidgets(
    'LayoutBuilder descendant widget can access [RenderBox.size] when rebuilding during layout',
    (WidgetTester tester) async {
      Size? childSize;
      int buildCount = 0;

      Future<void> pumpTestWidget(Size size) async {
        await tester.pumpWidget(
          // Center is used to give the SizedBox the power to determine constraints for LayoutBuilder
          Center(
            child: SizedBox.fromSize(
              size: size,
              child: LayoutBuilder(
                builder: (BuildContext context, BoxConstraints constraints) {
                  buildCount++;
                  if (buildCount > 1) {
                    final _RenderLayoutSpy spy = tester.renderObject(find.byType(_LayoutSpy));
                    childSize = spy.size;
                  }
                  return const ColoredBox(color: Color(0xffffffff), child: _LayoutSpy());
                },
              ),
            ),
          ),
        );
      }

      await pumpTestWidget(const Size(10.0, 10.0));
      expect(childSize, isNull);
      await pumpTestWidget(const Size(10.0, 10.0));
      expect(childSize, const Size(10.0, 10.0));
    },
  );

  testWidgets('LayoutBuilder will only invoke builder if updateShouldRebuild returns true', (
    WidgetTester tester,
  ) async {
    int buildCount = 0;
    int paintCount = 0;
    Offset? mostRecentOffset;
    void handleChildWasPainted(Offset extraOffset) {
      paintCount++;
      mostRecentOffset = extraOffset;
    }

    Future<void> pumpWidget(String text, double offsetPercentage) async {
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100,
              height: 100,
              child: _SmartLayoutBuilder(
                text: text,
                offsetPercentage: offsetPercentage,
                onChildWasPainted: handleChildWasPainted,
                builder: (BuildContext context, BoxConstraints constraints) {
                  buildCount++;
                  return Text(text);
                },
              ),
            ),
          ),
        ),
      );
    }

    await pumpWidget('aaa', 0.2);
    expect(find.text('aaa'), findsOneWidget);
    expect(buildCount, 1);
    expect(paintCount, 1);
    expect(mostRecentOffset, const Offset(20, 20));
    await pumpWidget('aaa', 0.4);
    expect(find.text('aaa'), findsOneWidget);
    expect(buildCount, 1);
    expect(paintCount, 2);
    expect(mostRecentOffset, const Offset(40, 40));
    await pumpWidget('bbb', 0.6);
    expect(find.text('aaa'), findsNothing);
    expect(find.text('bbb'), findsOneWidget);
    expect(buildCount, 2);
    expect(paintCount, 3);
    expect(mostRecentOffset, const Offset(60, 60));
  });

  testWidgets(
    'LayoutBuilder in a subtree that skips layout does not throw during the initial treewalk',
    (WidgetTester tester) async {
      final OverlayEntry overlayEntry1 = OverlayEntry(
        maintainState: true,
        builder:
            (BuildContext context) => LayoutBuilder(
              builder: (BuildContext context, BoxConstraints constraints) => const Placeholder(),
            ),
      );
      // OverlayEntry2 obstructs OverlayEntry1 and forces it to skip layout.
      final OverlayEntry overlayEntry2 = OverlayEntry(
        opaque: true,
        canSizeOverlay: true,
        builder: (BuildContext context) => Container(),
      );
      addTearDown(
        () =>
            overlayEntry1
              ..remove()
              ..dispose(),
      );
      addTearDown(
        () =>
            overlayEntry2
              ..remove()
              ..dispose(),
      );
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          // The UnconstrainedBox makes sure the OverlayEntries are not relayout boundaries.
          child: UnconstrainedBox(
            child: Overlay(initialEntries: <OverlayEntry>[overlayEntry1, overlayEntry2]),
          ),
        ),
      );

      WidgetsBinding.instance.buildOwner!.reassemble(WidgetsBinding.instance.rootElement!);
      await tester.pump();
      WidgetsBinding.instance.buildOwner!.reassemble(WidgetsBinding.instance.rootElement!);
      await tester.pump();
      expect(tester.takeException(), isNull);
    },
  );
}

class _SmartLayoutBuilder extends ConstrainedLayoutBuilder<BoxConstraints> {
  const _SmartLayoutBuilder({
    required this.text,
    required this.offsetPercentage,
    required this.onChildWasPainted,
    required super.builder,
  });

  final String text;
  final double offsetPercentage;
  final _OnChildWasPaintedCallback onChildWasPainted;

  @override
  bool updateShouldRebuild(_SmartLayoutBuilder oldWidget) {
    // Because this is a private widget and thus local to this file, we know
    // that only the [text] property affects the builder; the other properties
    // only affect painting.
    return text != oldWidget.text;
  }

  @override
  RenderObject createRenderObject(BuildContext context) {
    return _RenderSmartLayoutBuilder(
      offsetPercentage: offsetPercentage,
      onChildWasPainted: onChildWasPainted,
    );
  }

  @override
  void updateRenderObject(BuildContext context, _RenderSmartLayoutBuilder renderObject) {
    renderObject
      ..offsetPercentage = offsetPercentage
      ..onChildWasPainted = onChildWasPainted;
  }
}

typedef _OnChildWasPaintedCallback = void Function(Offset extraOffset);

class _RenderSmartLayoutBuilder extends RenderProxyBox
    with RenderConstrainedLayoutBuilder<BoxConstraints, RenderBox> {
  _RenderSmartLayoutBuilder({required double offsetPercentage, required this.onChildWasPainted})
    : _offsetPercentage = offsetPercentage;

  double _offsetPercentage;
  double get offsetPercentage => _offsetPercentage;
  set offsetPercentage(double value) {
    if (value != _offsetPercentage) {
      _offsetPercentage = value;
      markNeedsPaint();
    }
  }

  _OnChildWasPaintedCallback onChildWasPainted;

  @override
  bool get sizedByParent => true;

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    return constraints.biggest;
  }

  @override
  void performLayout() {
    rebuildIfNecessary();
    child?.layout(constraints);
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (child != null) {
      final Offset extraOffset = Offset(
        size.width * offsetPercentage,
        size.height * offsetPercentage,
      );
      context.paintChild(child!, offset + extraOffset);
      onChildWasPainted(extraOffset);
    }
  }
}

class _LayoutSpy extends LeafRenderObjectWidget {
  const _LayoutSpy();

  @override
  LeafRenderObjectElement createElement() => _LayoutSpyElement(this);

  @override
  RenderObject createRenderObject(BuildContext context) => _RenderLayoutSpy();
}

class _LayoutSpyElement extends LeafRenderObjectElement {
  _LayoutSpyElement(super.widget);
}

class _RenderLayoutSpy extends RenderBox {
  int performLayoutCount = 0;
  int performResizeCount = 0;

  @override
  bool get sizedByParent => true;

  @override
  void performResize() {
    performResizeCount += 1;
    size = constraints.biggest;
  }

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    return constraints.biggest;
  }

  @override
  void performLayout() {
    performLayoutCount += 1;
  }
}
