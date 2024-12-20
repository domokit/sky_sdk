// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart' show DragStartBehavior;
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  SemanticsTester semantics;

  setUp(() {
    debugResetSemanticsIdCounter();
  });

  testWidgets('scrollable exposes the correct semantic actions', (WidgetTester tester) async {
    semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(children: List<Widget>.generate(80, (int i) => Text('$i'))),
      ),
    );

    expect(
      semantics,
      includesNodeWith(
        actions: <SemanticsAction>[SemanticsAction.scrollUp, SemanticsAction.scrollToOffset],
      ),
    );

    await flingUp(tester);
    expect(
      semantics,
      includesNodeWith(
        actions: <SemanticsAction>[
          SemanticsAction.scrollUp,
          SemanticsAction.scrollDown,
          SemanticsAction.scrollToOffset,
        ],
      ),
    );

    await flingDown(tester, repetitions: 2);
    expect(
      semantics,
      includesNodeWith(
        actions: <SemanticsAction>[SemanticsAction.scrollUp, SemanticsAction.scrollToOffset],
      ),
    );

    await flingUp(tester, repetitions: 5);
    expect(
      semantics,
      includesNodeWith(
        actions: <SemanticsAction>[SemanticsAction.scrollDown, SemanticsAction.scrollToOffset],
      ),
    );

    await flingDown(tester);
    expect(
      semantics,
      includesNodeWith(
        actions: <SemanticsAction>[
          SemanticsAction.scrollUp,
          SemanticsAction.scrollDown,
          SemanticsAction.scrollToOffset,
        ],
      ),
    );

    semantics.dispose();
  });

  testWidgets('Vertical scrollable responds to scrollToOffset', (WidgetTester tester) async {
    semantics = SemanticsTester(tester);
    final ScrollController controller = ScrollController();
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          controller: controller,
          children: List<Widget>.generate(60, (int i) => Text('$i')),
        ),
      ),
    );
    final SemanticsOwner semanticsOwner = tester.binding.pipelineOwner.semanticsOwner!;
    final int scrollableId =
        semantics
            .nodesWith(
              actions: <SemanticsAction>[SemanticsAction.scrollUp, SemanticsAction.scrollToOffset],
            )
            .single
            .id;

    assert(controller.offset == 0);
    semanticsOwner.performAction(
      scrollableId,
      SemanticsAction.scrollToOffset,
      Float64List.fromList(<double>[123.0, 456.0]),
    );
    expect(controller.offset, 456.0);
    controller.dispose();
    semantics.dispose();
  });

  testWidgets('Horizontal scrollable responds to scrollToOffset', (WidgetTester tester) async {
    semantics = SemanticsTester(tester);
    final ScrollController controller = ScrollController();
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          controller: controller,
          scrollDirection: Axis.horizontal,
          children: List<Widget>.generate(60, (int i) => Text('$i')),
        ),
      ),
    );
    final SemanticsOwner semanticsOwner = tester.binding.pipelineOwner.semanticsOwner!;
    final int scrollableId =
        semantics
            .nodesWith(
              actions: <SemanticsAction>[
                SemanticsAction.scrollLeft,
                SemanticsAction.scrollToOffset,
              ],
            )
            .single
            .id;

    assert(controller.offset == 0);
    semanticsOwner.performAction(
      scrollableId,
      SemanticsAction.scrollToOffset,
      Float64List.fromList(<double>[123.0, 456.0]),
    );
    expect(controller.offset, 123.0);
    controller.dispose();
    semantics.dispose();
  });

  testWidgets('Unscrollable scrollable does not respond to scrollToOffset', (
    WidgetTester tester,
  ) async {
    semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(children: List<Widget>.generate(3, (int i) => Text('$i'))),
      ),
    );
    expect(
      semantics.nodesWith(
        actions: <SemanticsAction>[SemanticsAction.scrollUp, SemanticsAction.scrollToOffset],
      ),
      isEmpty,
    );
    semantics.dispose();
  });

  testWidgets('scrollToOffset respects implicit scrolling configuration', (
    WidgetTester tester,
  ) async {
    semantics = SemanticsTester(tester);
    final ScrollPhysics physics = _NoImplicitScrollingScrollPhysics();
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          physics: physics,
          children: List<Widget>.generate(60, (int i) => Text('$i')),
        ),
      ),
    );
    expect(
      semantics.nodesWith(
        actions: <SemanticsAction>[SemanticsAction.scrollUp, SemanticsAction.scrollToOffset],
      ),
      isEmpty,
    );
    semantics.dispose();
  });

  testWidgets('showOnScreen works in scrollable', (WidgetTester tester) async {
    semantics = SemanticsTester(tester); // enables semantics tree generation

    const double kItemHeight = 40.0;

    final List<Widget> containers = List<Widget>.generate(
      80,
      (int i) => MergeSemantics(
        child: SizedBox(
          height: kItemHeight,
          child: Text('container $i', textDirection: TextDirection.ltr),
        ),
      ),
    );

    final ScrollController scrollController = ScrollController(
      initialScrollOffset: kItemHeight / 2,
    );
    addTearDown(scrollController.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(controller: scrollController, children: containers),
      ),
    );

    expect(scrollController.offset, kItemHeight / 2);

    final int firstContainerId =
        tester.renderObject(find.byWidget(containers.first)).debugSemantics!.id;
    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      firstContainerId,
      SemanticsAction.showOnScreen,
    );
    await tester.pump();
    await tester.pump(const Duration(seconds: 5));

    expect(scrollController.offset, 0.0);

    semantics.dispose();
  });

  testWidgets('showOnScreen works with pinned app bar and sliver list', (
    WidgetTester tester,
  ) async {
    semantics = SemanticsTester(tester); // enables semantics tree generation

    const double kItemHeight = 100.0;
    const double kExpandedAppBarHeight = 56.0;

    final List<Widget> containers = List<Widget>.generate(
      80,
      (int i) => MergeSemantics(child: SizedBox(height: kItemHeight, child: Text('container $i'))),
    );

    final ScrollController scrollController = ScrollController(
      initialScrollOffset: kItemHeight / 2,
    );
    addTearDown(scrollController.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Localizations(
          locale: const Locale('en', 'us'),
          delegates: const <LocalizationsDelegate<dynamic>>[
            DefaultWidgetsLocalizations.delegate,
            DefaultMaterialLocalizations.delegate,
          ],
          child: MediaQuery(
            data: const MediaQueryData(),
            child: Scrollable(
              controller: scrollController,
              viewportBuilder: (BuildContext context, ViewportOffset offset) {
                return Viewport(
                  offset: offset,
                  slivers: <Widget>[
                    const SliverAppBar(
                      pinned: true,
                      expandedHeight: kExpandedAppBarHeight,
                      flexibleSpace: FlexibleSpaceBar(title: Text('App Bar')),
                    ),
                    SliverList(delegate: SliverChildListDelegate(containers)),
                  ],
                );
              },
            ),
          ),
        ),
      ),
    );

    expect(scrollController.offset, kItemHeight / 2);

    final int firstContainerId =
        tester.renderObject(find.byWidget(containers.first)).debugSemantics!.id;
    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      firstContainerId,
      SemanticsAction.showOnScreen,
    );
    await tester.pump();
    await tester.pump(const Duration(seconds: 5));
    expect(tester.getTopLeft(find.byWidget(containers.first)).dy, kExpandedAppBarHeight);

    semantics.dispose();
  });

  testWidgets('showOnScreen works with pinned app bar and individual slivers', (
    WidgetTester tester,
  ) async {
    semantics = SemanticsTester(tester); // enables semantics tree generation

    const double kItemHeight = 100.0;
    const double kExpandedAppBarHeight = 256.0;

    final List<Widget> children = <Widget>[];
    final List<Widget> slivers = List<Widget>.generate(30, (int i) {
      final Widget child = MergeSemantics(child: SizedBox(height: 72.0, child: Text('Item $i')));
      children.add(child);
      return SliverToBoxAdapter(child: child);
    });

    final ScrollController scrollController = ScrollController(
      initialScrollOffset: 2.5 * kItemHeight,
    );
    addTearDown(scrollController.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(),
          child: Localizations(
            locale: const Locale('en', 'us'),
            delegates: const <LocalizationsDelegate<dynamic>>[
              DefaultWidgetsLocalizations.delegate,
              DefaultMaterialLocalizations.delegate,
            ],
            child: Scrollable(
              controller: scrollController,
              viewportBuilder: (BuildContext context, ViewportOffset offset) {
                return Viewport(
                  offset: offset,
                  slivers: <Widget>[
                    const SliverAppBar(
                      pinned: true,
                      expandedHeight: kExpandedAppBarHeight,
                      flexibleSpace: FlexibleSpaceBar(title: Text('App Bar')),
                    ),
                    ...slivers,
                  ],
                );
              },
            ),
          ),
        ),
      ),
    );

    expect(scrollController.offset, 2.5 * kItemHeight);

    final int id0 = tester.renderObject(find.byWidget(children[0])).debugSemantics!.id;
    tester.binding.pipelineOwner.semanticsOwner!.performAction(id0, SemanticsAction.showOnScreen);
    await tester.pump();
    await tester.pump(const Duration(seconds: 5));
    expect(tester.getTopLeft(find.byWidget(children[0])).dy, kToolbarHeight);

    semantics.dispose();
  });

  testWidgets('correct scrollProgress', (WidgetTester tester) async {
    semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(children: List<Widget>.generate(80, (int i) => Text('$i'))),
      ),
    );

    expect(
      semantics,
      includesNodeWith(
        scrollExtentMin: 0.0,
        scrollPosition: 0.0,
        scrollExtentMax: 520.0,
        actions: <SemanticsAction>[SemanticsAction.scrollUp, SemanticsAction.scrollToOffset],
      ),
    );

    await flingUp(tester);

    expect(
      semantics,
      includesNodeWith(
        scrollExtentMin: 0.0,
        scrollPosition: 394.3,
        scrollExtentMax: 520.0,
        actions: <SemanticsAction>[
          SemanticsAction.scrollUp,
          SemanticsAction.scrollDown,
          SemanticsAction.scrollToOffset,
        ],
      ),
    );

    await flingUp(tester);

    expect(
      semantics,
      includesNodeWith(
        scrollExtentMin: 0.0,
        scrollPosition: 520.0,
        scrollExtentMax: 520.0,
        actions: <SemanticsAction>[SemanticsAction.scrollDown, SemanticsAction.scrollToOffset],
      ),
    );

    semantics.dispose();
  });

  testWidgets('correct scrollProgress for unbound', (WidgetTester tester) async {
    semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView.builder(
          dragStartBehavior: DragStartBehavior.down,
          itemExtent: 20.0,
          itemBuilder: (BuildContext context, int index) {
            return Text('entry $index');
          },
        ),
      ),
    );

    expect(
      semantics,
      includesNodeWith(
        scrollExtentMin: 0.0,
        scrollPosition: 0.0,
        scrollExtentMax: double.infinity,
        actions: <SemanticsAction>[SemanticsAction.scrollUp, SemanticsAction.scrollToOffset],
      ),
    );

    await flingUp(tester);

    expect(
      semantics,
      includesNodeWith(
        scrollExtentMin: 0.0,
        scrollPosition: 394.3,
        scrollExtentMax: double.infinity,
        actions: <SemanticsAction>[
          SemanticsAction.scrollUp,
          SemanticsAction.scrollDown,
          SemanticsAction.scrollToOffset,
        ],
      ),
    );

    await flingUp(tester);

    expect(
      semantics,
      includesNodeWith(
        scrollExtentMin: 0.0,
        scrollPosition: 788.6,
        scrollExtentMax: double.infinity,
        actions: <SemanticsAction>[
          SemanticsAction.scrollUp,
          SemanticsAction.scrollDown,
          SemanticsAction.scrollToOffset,
        ],
      ),
    );

    semantics.dispose();
  });

  testWidgets('Semantics tree is populated mid-scroll', (WidgetTester tester) async {
    semantics = SemanticsTester(tester);

    final List<Widget> children = List<Widget>.generate(
      80,
      (int i) => SizedBox(height: 40.0, child: Text('Item $i')),
    );
    await tester.pumpWidget(
      Directionality(textDirection: TextDirection.ltr, child: ListView(children: children)),
    );

    final TestGesture gesture = await tester.startGesture(tester.getCenter(find.byType(ListView)));
    await gesture.moveBy(const Offset(0.0, -40.0));
    await tester.pump();

    expect(semantics, includesNodeWith(label: 'Item 1'));
    expect(semantics, includesNodeWith(label: 'Item 2'));
    expect(semantics, includesNodeWith(label: 'Item 3'));

    semantics.dispose();
  });

  testWidgets('Can toggle semantics on, off, on without crash', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          children: List<Widget>.generate(40, (int i) {
            return SizedBox(height: 400.0, child: Text('item $i'));
          }),
        ),
      ),
    );

    final TestSemantics expectedSemantics = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics.rootChild(
          children: <TestSemantics>[
            TestSemantics(
              flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
              actions: <SemanticsAction>[SemanticsAction.scrollUp, SemanticsAction.scrollToOffset],
              children: <TestSemantics>[
                TestSemantics(label: r'item 0', textDirection: TextDirection.ltr),
                TestSemantics(label: r'item 1', textDirection: TextDirection.ltr),
                TestSemantics(flags: <SemanticsFlag>[SemanticsFlag.isHidden], label: r'item 2'),
              ],
            ),
          ],
        ),
      ],
    );

    // Start with semantics off.
    expect(tester.binding.pipelineOwner.semanticsOwner, isNull);

    // Semantics on
    semantics = SemanticsTester(tester);
    await tester.pumpAndSettle();
    expect(tester.binding.pipelineOwner.semanticsOwner, isNotNull);
    expect(
      semantics,
      hasSemantics(expectedSemantics, ignoreId: true, ignoreRect: true, ignoreTransform: true),
    );

    // Semantics off
    semantics.dispose();
    await tester.pumpAndSettle();
    expect(tester.binding.pipelineOwner.semanticsOwner, isNull);

    // Semantics on
    semantics = SemanticsTester(tester);
    await tester.pumpAndSettle();
    expect(tester.binding.pipelineOwner.semanticsOwner, isNotNull);
    expect(
      semantics,
      hasSemantics(expectedSemantics, ignoreId: true, ignoreRect: true, ignoreTransform: true),
    );

    semantics.dispose();
  }, semanticsEnabled: false);

  group('showOnScreen', () {
    const double kItemHeight = 100.0;

    late List<Widget> children;
    late ScrollController scrollController;
    late Widget widgetUnderTest;

    setUp(() {
      children = List<Widget>.generate(10, (int i) {
        return MergeSemantics(child: SizedBox(height: kItemHeight, child: Text('container $i')));
      });

      scrollController = ScrollController(initialScrollOffset: kItemHeight / 2);

      widgetUnderTest = Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 2 * kItemHeight,
            child: ListView(controller: scrollController, children: children),
          ),
        ),
      );
    });

    testWidgets('brings item above leading edge to leading edge', (WidgetTester tester) async {
      semantics = SemanticsTester(tester); // enables semantics tree generation

      await tester.pumpWidget(widgetUnderTest);

      expect(scrollController.offset, kItemHeight / 2);

      final int firstContainerId =
          tester.renderObject(find.byWidget(children.first)).debugSemantics!.id;
      tester.binding.pipelineOwner.semanticsOwner!.performAction(
        firstContainerId,
        SemanticsAction.showOnScreen,
      );
      await tester.pumpAndSettle();

      expect(scrollController.offset, 0.0);

      semantics.dispose();
    });

    testWidgets('brings item below trailing edge to trailing edge', (WidgetTester tester) async {
      semantics = SemanticsTester(tester); // enables semantics tree generation

      await tester.pumpWidget(widgetUnderTest);

      expect(scrollController.offset, kItemHeight / 2);

      final int firstContainerId =
          tester.renderObject(find.byWidget(children[2])).debugSemantics!.id;
      tester.binding.pipelineOwner.semanticsOwner!.performAction(
        firstContainerId,
        SemanticsAction.showOnScreen,
      );
      await tester.pumpAndSettle();

      expect(scrollController.offset, kItemHeight);

      semantics.dispose();
    });

    testWidgets('does not change position of items already fully on-screen', (
      WidgetTester tester,
    ) async {
      semantics = SemanticsTester(tester); // enables semantics tree generation

      await tester.pumpWidget(widgetUnderTest);

      expect(scrollController.offset, kItemHeight / 2);

      final int firstContainerId =
          tester.renderObject(find.byWidget(children[1])).debugSemantics!.id;
      tester.binding.pipelineOwner.semanticsOwner!.performAction(
        firstContainerId,
        SemanticsAction.showOnScreen,
      );
      await tester.pumpAndSettle();

      expect(scrollController.offset, kItemHeight / 2);

      semantics.dispose();
    });
  });

  group('showOnScreen with negative children', () {
    const double kItemHeight = 100.0;

    late List<Widget> children;
    late ScrollController scrollController;
    late Widget widgetUnderTest;

    setUp(() {
      final Key center = GlobalKey();

      children = List<Widget>.generate(10, (int i) {
        return SliverToBoxAdapter(
          key: i == 5 ? center : null,
          child: MergeSemantics(
            key: ValueKey<int>(i),
            child: SizedBox(height: kItemHeight, child: Text('container $i')),
          ),
        );
      });

      scrollController = ScrollController(initialScrollOffset: -2.5 * kItemHeight);

      // 'container 0' is at offset -500
      // 'container 1' is at offset -400
      // 'container 2' is at offset -300
      // 'container 3' is at offset -200
      // 'container 4' is at offset -100
      // 'container 5' is at offset 0

      widgetUnderTest = Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 2 * kItemHeight,
            child: Scrollable(
              controller: scrollController,
              viewportBuilder: (BuildContext context, ViewportOffset offset) {
                return Viewport(
                  cacheExtent: 0.0,
                  offset: offset,
                  center: center,
                  slivers: children,
                );
              },
            ),
          ),
        ),
      );
    });

    tearDown(() {
      scrollController.dispose();
    });

    testWidgets('brings item above leading edge to leading edge', (WidgetTester tester) async {
      semantics = SemanticsTester(tester); // enables semantics tree generation

      await tester.pumpWidget(widgetUnderTest);

      expect(scrollController.offset, -250.0);

      final int firstContainerId =
          tester.renderObject(find.byKey(const ValueKey<int>(2))).debugSemantics!.id;
      tester.binding.pipelineOwner.semanticsOwner!.performAction(
        firstContainerId,
        SemanticsAction.showOnScreen,
      );
      await tester.pumpAndSettle();

      expect(scrollController.offset, -300.0);

      semantics.dispose();
    });

    testWidgets('brings item below trailing edge to trailing edge', (WidgetTester tester) async {
      semantics = SemanticsTester(tester); // enables semantics tree generation

      await tester.pumpWidget(widgetUnderTest);

      expect(scrollController.offset, -250.0);

      final int firstContainerId =
          tester.renderObject(find.byKey(const ValueKey<int>(4))).debugSemantics!.id;
      tester.binding.pipelineOwner.semanticsOwner!.performAction(
        firstContainerId,
        SemanticsAction.showOnScreen,
      );
      await tester.pumpAndSettle();

      expect(scrollController.offset, -200.0);

      semantics.dispose();
    });

    testWidgets('does not change position of items already fully on-screen', (
      WidgetTester tester,
    ) async {
      semantics = SemanticsTester(tester); // enables semantics tree generation

      await tester.pumpWidget(widgetUnderTest);

      expect(scrollController.offset, -250.0);

      final int firstContainerId =
          tester.renderObject(find.byKey(const ValueKey<int>(3))).debugSemantics!.id;
      tester.binding.pipelineOwner.semanticsOwner!.performAction(
        firstContainerId,
        SemanticsAction.showOnScreen,
      );
      await tester.pumpAndSettle();

      expect(scrollController.offset, -250.0);

      semantics.dispose();
    });
  });

  testWidgets(
    'transform of inner node from useTwoPaneSemantics scrolls correctly with nested scrollables',
    (WidgetTester tester) async {
      semantics = SemanticsTester(tester); // enables semantics tree generation

      // Context: https://github.com/flutter/flutter/issues/61631
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: SingleChildScrollView(
            child: ListView(
              shrinkWrap: true,
              children: <Widget>[for (int i = 0; i < 50; ++i) Text('$i')],
            ),
          ),
        ),
      );

      final SemanticsNode rootScrollNode =
          semantics
              .nodesWith(
                actions: <SemanticsAction>[
                  SemanticsAction.scrollUp,
                  SemanticsAction.scrollToOffset,
                ],
              )
              .single;
      final SemanticsNode innerListPane =
          semantics.nodesWith(ancestor: rootScrollNode, scrollExtentMax: 0).single;
      final SemanticsNode outerListPane = innerListPane.parent!;
      final List<SemanticsNode> hiddenNodes =
          semantics.nodesWith(flags: <SemanticsFlag>[SemanticsFlag.isHidden]).toList();

      // This test is only valid if some children are offscreen.
      // Increase the number of Text children if this assert fails.
      assert(hiddenNodes.length >= 3);

      // Scroll to end -> beginning -> middle to test both directions.
      final List<SemanticsNode> targetNodes = <SemanticsNode>[
        hiddenNodes.last,
        hiddenNodes.first,
        hiddenNodes[hiddenNodes.length ~/ 2],
      ];

      expect(nodeGlobalRect(innerListPane), nodeGlobalRect(outerListPane));

      for (final SemanticsNode node in targetNodes) {
        tester.binding.pipelineOwner.semanticsOwner!.performAction(
          node.id,
          SemanticsAction.showOnScreen,
        );
        await tester.pumpAndSettle();

        expect(nodeGlobalRect(innerListPane), nodeGlobalRect(outerListPane));
      }

      semantics.dispose();
    },
  );
}

Future<void> flingUp(WidgetTester tester, {int repetitions = 1}) =>
    fling(tester, const Offset(0.0, -200.0), repetitions);

Future<void> flingDown(WidgetTester tester, {int repetitions = 1}) =>
    fling(tester, const Offset(0.0, 200.0), repetitions);

Future<void> fling(WidgetTester tester, Offset offset, int repetitions) async {
  while (repetitions-- > 0) {
    await tester.fling(find.byType(ListView), offset, 1000.0);
    await tester.pump();
    await tester.pump(const Duration(seconds: 5));
  }
}

Rect nodeGlobalRect(SemanticsNode node) {
  Matrix4 globalTransform = node.transform ?? Matrix4.identity();
  for (SemanticsNode? parent = node.parent; parent != null; parent = parent.parent) {
    if (parent.transform != null) {
      globalTransform = parent.transform!.multiplied(globalTransform);
    }
  }
  return MatrixUtils.transformRect(globalTransform, node.rect);
}

class _NoImplicitScrollingScrollPhysics extends ScrollPhysics {
  @override
  bool get allowImplicitScrolling => false;

  @override
  ScrollPhysics applyTo(ScrollPhysics? ancestor) => this;
}
