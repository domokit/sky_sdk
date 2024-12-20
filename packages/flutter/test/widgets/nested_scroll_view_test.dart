// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart' show DragStartBehavior;
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import '../rendering/rendering_tester.dart' show TestClipPaintingContext;

class _CustomPhysics extends ClampingScrollPhysics {
  const _CustomPhysics({super.parent});

  @override
  _CustomPhysics applyTo(ScrollPhysics? ancestor) {
    return _CustomPhysics(parent: buildParent(ancestor));
  }

  @override
  Simulation createBallisticSimulation(ScrollMetrics position, double dragVelocity) {
    return ScrollSpringSimulation(spring, 1000.0, 1000.0, 1000.0);
  }
}

Widget buildTest({
  ScrollController? controller,
  String title = 'TTTTTTTT',
  Key? key,
  bool expanded = true,
}) {
  return MaterialApp(
    home: Scaffold(
      drawerDragStartBehavior: DragStartBehavior.down,
      body: DefaultTabController(
        length: 4,
        child: NestedScrollView(
          key: key,
          dragStartBehavior: DragStartBehavior.down,
          controller: controller,
          headerSliverBuilder: (BuildContext context, bool innerBoxIsScrolled) {
            return <Widget>[
              SliverAppBar(
                title: Text(title),
                pinned: true,
                expandedHeight: expanded ? 200.0 : 0.0,
                forceElevated: innerBoxIsScrolled,
                bottom: const TabBar(
                  tabs: <Tab>[Tab(text: 'AA'), Tab(text: 'BB'), Tab(text: 'CC'), Tab(text: 'DD')],
                ),
              ),
            ];
          },
          body: TabBarView(
            children: <Widget>[
              ListView(
                children: const <Widget>[
                  SizedBox(height: 300.0, child: Text('aaa1')),
                  SizedBox(height: 200.0, child: Text('aaa2')),
                  SizedBox(height: 100.0, child: Text('aaa3')),
                  SizedBox(height: 50.0, child: Text('aaa4')),
                ],
              ),
              ListView(
                dragStartBehavior: DragStartBehavior.down,
                children: const <Widget>[SizedBox(height: 100.0, child: Text('bbb1'))],
              ),
              const Center(child: Text('ccc1')),
              ListView(
                dragStartBehavior: DragStartBehavior.down,
                children: const <Widget>[SizedBox(height: 10000.0, child: Text('ddd1'))],
              ),
            ],
          ),
        ),
      ),
    ),
  );
}

void main() {
  testWidgets('ScrollDirection test', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/107101
    final List<ScrollDirection> receivedResult = <ScrollDirection>[];
    const List<ScrollDirection> expectedReverseResult = <ScrollDirection>[
      ScrollDirection.reverse,
      ScrollDirection.idle,
    ];
    const List<ScrollDirection> expectedForwardResult = <ScrollDirection>[
      ScrollDirection.forward,
      ScrollDirection.idle,
    ];

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: NotificationListener<UserScrollNotification>(
            onNotification: (UserScrollNotification notification) {
              if (notification.depth != 1) {
                return true;
              }
              receivedResult.add(notification.direction);
              return true;
            },
            child: NestedScrollView(
              headerSliverBuilder:
                  (BuildContext context, bool innerBoxIsScrolled) => <Widget>[
                    const SliverAppBar(expandedHeight: 250.0, pinned: true),
                  ],
              body: ListView.builder(
                padding: const EdgeInsets.all(8),
                itemCount: 30,
                itemBuilder: (BuildContext context, int index) {
                  return SizedBox(height: 50, child: Center(child: Text('Item $index')));
                },
              ),
            ),
          ),
        ),
      ),
    );

    // Fling down to trigger ballistic activity
    await tester.fling(find.text('Item 3'), const Offset(0.0, -250.0), 10000.0);
    await tester.pumpAndSettle();

    expect(receivedResult, expectedReverseResult);

    receivedResult.clear();

    // Drag forward, without ballistic activity
    await tester.drag(find.text('Item 29'), const Offset(0.0, 20.0));
    await tester.pump();

    expect(receivedResult, expectedForwardResult);
  });

  testWidgets('NestedScrollView respects clipBehavior', (WidgetTester tester) async {
    Widget build(NestedScrollView nestedScrollView) {
      return Localizations(
        locale: const Locale('en', 'US'),
        delegates: const <LocalizationsDelegate<dynamic>>[
          DefaultMaterialLocalizations.delegate,
          DefaultWidgetsLocalizations.delegate,
        ],
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(data: const MediaQueryData(), child: nestedScrollView),
        ),
      );
    }

    await tester.pumpWidget(
      build(
        NestedScrollView(
          headerSliverBuilder:
              (BuildContext context, bool innerBoxIsScrolled) => <Widget>[const SliverAppBar()],
          body: Container(height: 2000.0),
        ),
      ),
    );

    // 1st, check that the render object has received the default clip behavior.
    final RenderNestedScrollViewViewport renderObject =
        tester.allRenderObjects.whereType<RenderNestedScrollViewViewport>().first;
    expect(renderObject.clipBehavior, equals(Clip.hardEdge));

    // 2nd, check that the painting context has received the default clip behavior.
    final TestClipPaintingContext context = TestClipPaintingContext();
    renderObject.paint(context, Offset.zero);
    expect(context.clipBehavior, equals(Clip.hardEdge));

    // 3rd, pump a new widget to check that the render object can update its clip behavior.
    await tester.pumpWidget(
      build(
        NestedScrollView(
          headerSliverBuilder:
              (BuildContext context, bool innerBoxIsScrolled) => <Widget>[const SliverAppBar()],
          body: Container(height: 2000.0),
          clipBehavior: Clip.antiAlias,
        ),
      ),
    );
    expect(renderObject.clipBehavior, equals(Clip.antiAlias));

    // 4th, check that a non-default clip behavior can be sent to the painting context.
    renderObject.paint(context, Offset.zero);
    expect(context.clipBehavior, equals(Clip.antiAlias));
  });

  testWidgets('NestedScrollView always scrolls outer scrollable first', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/136199
    final Key innerKey = UniqueKey();
    final GlobalKey<NestedScrollViewState> outerKey = GlobalKey();

    final ScrollController outerController = ScrollController();
    addTearDown(outerController.dispose);

    Widget build() {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Scaffold(
          body: NestedScrollView(
            key: outerKey,
            controller: outerController,
            physics: const BouncingScrollPhysics(),
            headerSliverBuilder:
                (BuildContext context, bool innerBoxIsScrolled) => <Widget>[
                  SliverToBoxAdapter(child: Container(color: Colors.green, height: 300)),
                  SliverOverlapAbsorber(
                    handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                    sliver: SliverToBoxAdapter(child: Container(color: Colors.blue, height: 64)),
                  ),
                ],
            body: SingleChildScrollView(
              key: innerKey,
              physics: const BouncingScrollPhysics(),
              child: Container(
                decoration: const BoxDecoration(
                  gradient: LinearGradient(
                    begin: Alignment.topCenter,
                    end: Alignment.bottomCenter,
                    colors: <ui.Color>[Colors.black, Colors.blue],
                    stops: <double>[0, 1],
                  ),
                ),
                height: 800,
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(build());

    final ScrollController outer = outerKey.currentState!.outerController;
    final ScrollController inner = outerKey.currentState!.innerController;

    // Assert the initial positions
    expect(outer.offset, 0.0);
    expect(inner.offset, 0.0);

    outerController.addListener(() {
      fail('Outer controller should not be scrolling'); // This should never be called
    });

    await tester.drag(
      find.byKey(innerKey),
      const Offset(0, 2000),
    ); // Over-scroll the inner Scrollable to the bottom

    // Using a precise value to make addition/subtraction possible later in the test
    // Which better conveys the intent of the test
    // The value is not equal to 2000 due to BouncingScrollPhysics of the inner Scrollable
    const double endPosition = -1974.0862087158384;
    const Duration nextFrame = Duration(microseconds: 16666);

    // Assert positions after over-scrolling
    expect(outer.offset, 0.0);
    expect(inner.offset, endPosition);

    await tester.fling(
      find.byKey(innerKey),
      const Offset(0, -600),
      2000,
    ); // Fling the inner Scrollable to the top

    // Assert positions after fling
    expect(outer.offset, 0.0);
    expect(inner.offset, endPosition + 600);

    await tester.pump(nextFrame);

    // Assert positions after pump
    expect(outer.offset, 0.0);
    expect(inner.offset, endPosition + 600);

    double currentOffset = inner.offset;
    int maxNumberOfSteps = 100;

    while (inner.offset < 0) {
      maxNumberOfSteps--;
      if (maxNumberOfSteps <= 0) {
        fail('Scrolling did not settle in an expected number of steps.');
      }
      await tester.pump(nextFrame);
      expect(inner.offset, greaterThanOrEqualTo(currentOffset));
      expect(outer.offset, 0.0);

      currentOffset = inner.offset;
    }

    // Assert positions returned to/stayed at 0.0
    expect(outer.offset, 0.0);
    expect(inner.offset, 0.0);

    await tester.pumpAndSettle();

    // Assert values settle at 0.0
    expect(outer.offset, 0.0);
    expect(inner.offset, 0.0);
  });

  testWidgets('NestedScrollView allows taps on children while over-scrolled to the top', (
    WidgetTester tester,
  ) async {
    final Key innerKey = UniqueKey();
    final GlobalKey<NestedScrollViewState> outerKey = GlobalKey();

    final ScrollController outerController = ScrollController();
    addTearDown(outerController.dispose);

    const Duration frame = Duration(milliseconds: 16);
    bool tapped = false;

    Widget build() {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Scaffold(
          body: NestedScrollView(
            key: outerKey,
            controller: outerController,
            physics: const BouncingScrollPhysics(),
            headerSliverBuilder:
                (BuildContext context, bool innerBoxIsScrolled) => <Widget>[
                  SliverToBoxAdapter(child: Container(color: Colors.green, height: 300)),
                  SliverOverlapAbsorber(
                    handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                    sliver: SliverToBoxAdapter(child: Container(color: Colors.blue, height: 64)),
                  ),
                ],
            body: ListView.builder(
              key: innerKey,
              physics: const BouncingScrollPhysics(),
              itemCount: 15,
              itemBuilder: (BuildContext context, int index) {
                return ListTile(
                  title: Text('Item $index'),
                  onTap: () {
                    tapped = true;
                  },
                );
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(build());

    final ScrollController outer = outerKey.currentState!.outerController;
    final ScrollController inner = outerKey.currentState!.innerController;

    // Assert the initial positions
    expect(outer.offset, 0.0);
    expect(inner.offset, 0.0);

    // Over-scroll the inner Scrollable to the top
    await tester.fling(find.byKey(innerKey), const Offset(0, 200), 2000);

    for (int i = 0; i < 5; i++) {
      await tester.pump(frame);
    }

    // Ensure the inner Scrollable is over-scrolled
    expect(inner.offset, lessThan(0.0));

    // Tap on the first item in the ListView
    await tester.tap(find.text('Item 0'));
    expect(tapped, isTrue);
    tapped = false;

    await tester.pump(frame);

    await tester.tap(find.text('Item 1'));
    expect(tapped, isTrue);
    tapped = false;

    await tester.pumpAndSettle();

    await tester.tap(find.text('Item 0'));
    expect(tapped, isTrue);
    tapped = false;
  });

  testWidgets('NestedScrollView absorbs touch to stop scrolling when not at the edge', (
    WidgetTester tester,
  ) async {
    final Key innerKey = UniqueKey();
    final GlobalKey<NestedScrollViewState> outerKey = GlobalKey();

    final ScrollController outerController = ScrollController();
    addTearDown(outerController.dispose);

    const Duration frame = Duration(milliseconds: 16);
    bool tapped = false;

    Widget build() {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Scaffold(
          body: NestedScrollView(
            key: outerKey,
            controller: outerController,
            physics: const BouncingScrollPhysics(),
            headerSliverBuilder:
                (BuildContext context, bool innerBoxIsScrolled) => <Widget>[
                  SliverToBoxAdapter(child: Container(color: Colors.green, height: 300)),
                  SliverOverlapAbsorber(
                    handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                    sliver: SliverToBoxAdapter(child: Container(color: Colors.blue, height: 64)),
                  ),
                ],
            body: ListView.builder(
              key: innerKey,
              physics: const BouncingScrollPhysics(),
              itemExtent: 56,
              itemCount: 15,
              itemBuilder: (BuildContext context, int index) {
                return ListTile(
                  title: Text('Item $index'),
                  onTap: () {
                    tapped = true;
                  },
                );
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(build());

    final ScrollController outer = outerKey.currentState!.outerController;
    final ScrollController inner = outerKey.currentState!.innerController;

    // Assert the initial positions
    expect(outer.offset, 0.0);
    expect(inner.offset, 0.0);

    // Fling to somewhere in the middle of the outer Scrollable
    await tester.fling(find.byKey(innerKey), const Offset(0, -200), 2000);

    for (int i = 0; i < 3; i++) {
      await tester.pump(frame);
    }

    // Ensure we are not at the edge
    expect(outer.offset, greaterThan(0.0));
    expect(outer.offset, lessThan(outer.position.maxScrollExtent));
    final double offset = outer.offset;

    // Tap on the first item in the ListView
    await tester.tap(find.text('Item 2'), warnIfMissed: false);
    expect(tapped, isFalse);

    await tester.pump(frame);

    // Ensure the outer Scrollable is not moving
    expect(offset, equals(outer.offset));

    await tester.tap(find.text('Item 2'));
    expect(tapped, isTrue);
    tapped = false;

    await tester.pumpAndSettle();

    await tester.tap(find.text('Item 2'));
    expect(tapped, isTrue);
    tapped = false;

    // Fling the scrollable further
    await tester.fling(find.byKey(innerKey), const Offset(0, -200), 2000);

    for (int i = 0; i < 3; i++) {
      await tester.pump(frame);
    }

    // Ensure the outer Scrollable is at edge
    expect(outer.offset, equals(outer.position.maxScrollExtent));
    // Ensure the inner Scrollable is not over-scrolled yet
    expect(inner.offset, lessThan(inner.position.maxScrollExtent));

    final double innerOffset = inner.offset;

    // Tap on an item near the end of the ListView
    await tester.tap(find.text('Item 10'), warnIfMissed: false);
    expect(tapped, isFalse);

    await tester.pump(frame);

    // Ensure the inner Scrollable is not moving
    expect(innerOffset, equals(inner.offset));

    // Tapping on an item should register the tap normally, as the scrollable is idle
    await tester.tap(find.text('Item 10'));
    expect(tapped, isTrue);
  });

  testWidgets('NestedScrollView when over-scrolled at the end allows touches on children', (
    WidgetTester tester,
  ) async {
    final Key innerKey = UniqueKey();
    final GlobalKey<NestedScrollViewState> outerKey = GlobalKey();

    final ScrollController outerController = ScrollController();
    addTearDown(outerController.dispose);

    const Duration frame = Duration(milliseconds: 16);
    bool tapped = false;

    Widget build() {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Scaffold(
          body: NestedScrollView(
            key: outerKey,
            controller: outerController,
            physics: const BouncingScrollPhysics(),
            headerSliverBuilder:
                (BuildContext context, bool innerBoxIsScrolled) => <Widget>[
                  SliverToBoxAdapter(child: Container(color: Colors.green, height: 300)),
                  SliverOverlapAbsorber(
                    handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                    sliver: SliverToBoxAdapter(child: Container(color: Colors.blue, height: 64)),
                  ),
                ],
            body: ListView.builder(
              key: innerKey,
              physics: const BouncingScrollPhysics(),
              itemExtent: 56,
              itemCount: 15,
              itemBuilder: (BuildContext context, int index) {
                return ListTile(
                  title: Text('Item $index'),
                  onTap: () {
                    tapped = true;
                  },
                );
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(build());

    final ScrollController outer = outerKey.currentState!.outerController;
    final ScrollController inner = outerKey.currentState!.innerController;

    // Assert the initial positions
    expect(outer.offset, 0.0);
    expect(inner.offset, 0.0);

    // Fling to somewhere in the middle of the outer Scrollable
    await tester.fling(find.byKey(innerKey), const Offset(0, -2000), 2000);

    for (int i = 0; i < 10; i++) {
      await tester.pump(frame);
    }

    // Ensure the outer Scrollable is at edge
    expect(outer.offset, equals(outer.position.maxScrollExtent));
    // Ensure the inner Scrollable is over-scrolled
    expect(inner.offset, greaterThan(inner.position.maxScrollExtent));

    // Tap on an item near the end of the ListView
    await tester.tap(find.text('Item 14'));
    expect(tapped, isTrue);
    tapped = false;

    double settleOffset = inner.offset;

    for (int i = 0; i < 5; i++) {
      await tester.pump(frame);
      await tester.pump(frame); // Pump a second frame to ensure the Scrollable has a chance to move

      await tester.tap(find.text('Item 14'));
      expect(tapped, isTrue);
      tapped = false;
      // Ensure the inner Scrollable is settling
      expect(settleOffset, greaterThan(inner.offset));
      settleOffset = inner.offset;
    }

    await tester.pumpAndSettle();

    await tester.tap(find.text('Item 14'));
    expect(tapped, isTrue);
  });

  testWidgets(
    'NestedScrollView overscroll and release and hold',
    (WidgetTester tester) async {
      await tester.pumpWidget(buildTest());
      expect(find.text('aaa2'), findsOneWidget);
      await tester.pump(const Duration(milliseconds: 250));
      final Offset point1 = tester.getCenter(find.text('aaa1'));
      if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS) {
        await tester.dragFrom(point1, const Offset(0.0, 400.0));
      } else {
        await tester.dragFrom(point1, const Offset(0.0, 200.0));
      }
      await tester.pump();
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 200.0);
      await tester.flingFrom(point1, const Offset(0.0, -80.0), 50000.0);
      await tester.pump(const Duration(milliseconds: 20));
      final Offset point2 = tester.getCenter(find.text('aaa1'));
      expect(point2.dy, greaterThan(point1.dy));
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 200.0);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'NestedScrollView overscroll and release and hold',
    (WidgetTester tester) async {
      await tester.pumpWidget(buildTest());
      expect(find.text('aaa2'), findsOneWidget);
      await tester.pump(const Duration(milliseconds: 250));
      final Offset point = tester.getCenter(find.text('aaa1'));
      if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS) {
        await tester.flingFrom(point, const Offset(0.0, 200.0), 15000.0);
      } else {
        await tester.flingFrom(point, const Offset(0.0, 200.0), 5000.0);
      }
      await tester.pump(const Duration(milliseconds: 10));
      await tester.pump(const Duration(milliseconds: 10));
      await tester.pump(const Duration(milliseconds: 10));
      expect(find.text('aaa2'), findsNothing);
      final TestGesture gesture1 = await tester.startGesture(point);
      await tester.pump(const Duration(milliseconds: 5000));
      expect(find.text('aaa2'), findsNothing);
      await gesture1.moveBy(const Offset(0.0, 50.0));
      await tester.pump(const Duration(milliseconds: 10));
      await tester.pump(const Duration(milliseconds: 10));
      expect(find.text('aaa2'), findsNothing);
      await tester.pump(const Duration(milliseconds: 1000));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'NestedScrollView overscroll and release',
    (WidgetTester tester) async {
      await tester.pumpWidget(buildTest());
      expect(find.text('aaa2'), findsOneWidget);
      await tester.pump(const Duration(milliseconds: 500));
      final TestGesture gesture1 = await tester.startGesture(tester.getCenter(find.text('aaa1')));
      await gesture1.moveBy(const Offset(0.0, 200.0));
      await tester.pumpAndSettle();
      expect(find.text('aaa2'), findsNothing);
      await tester.pump(const Duration(seconds: 1));
      await gesture1.up();
      await tester.pumpAndSettle();
      expect(find.text('aaa2'), findsOneWidget);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('NestedScrollView', (WidgetTester tester) async {
    await tester.pumpWidget(buildTest());
    expect(find.text('aaa2'), findsOneWidget);
    expect(find.text('aaa3'), findsNothing);
    expect(find.text('bbb1'), findsNothing);
    await tester.pump(const Duration(milliseconds: 250));
    expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 200.0);

    await tester.drag(find.text('AA'), const Offset(0.0, -20.0));
    await tester.pump(const Duration(milliseconds: 250));
    expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 180.0);

    await tester.drag(find.text('AA'), const Offset(0.0, -20.0));
    await tester.pump(const Duration(milliseconds: 250));
    expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 160.0);

    await tester.drag(find.text('AA'), const Offset(0.0, -20.0));
    await tester.pump(const Duration(milliseconds: 250));
    expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 140.0);

    expect(find.text('aaa4'), findsNothing);
    await tester.pump(const Duration(milliseconds: 250));
    await tester.fling(find.text('AA'), const Offset(0.0, -50.0), 10000.0);
    await tester.pumpAndSettle(const Duration(milliseconds: 250));
    expect(find.text('aaa4'), findsOneWidget);

    final double minHeight = tester.renderObject<RenderBox>(find.byType(AppBar)).size.height;
    expect(minHeight, lessThan(140.0));

    await tester.pump(const Duration(milliseconds: 250));
    await tester.tap(find.text('BB'));
    await tester.pumpAndSettle(const Duration(milliseconds: 250));
    expect(find.text('aaa4'), findsNothing);
    expect(find.text('bbb1'), findsOneWidget);

    await tester.pump(const Duration(milliseconds: 250));
    await tester.tap(find.text('CC'));
    await tester.pumpAndSettle(const Duration(milliseconds: 250));
    expect(find.text('bbb1'), findsNothing);
    expect(find.text('ccc1'), findsOneWidget);
    expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, minHeight);

    await tester.pump(const Duration(milliseconds: 250));
    await tester.fling(find.text('AA'), const Offset(0.0, 50.0), 10000.0);
    await tester.pumpAndSettle(const Duration(milliseconds: 250));
    expect(find.text('ccc1'), findsOneWidget);
    expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 200.0);
  });

  testWidgets('NestedScrollView with a ScrollController', (WidgetTester tester) async {
    final ScrollController controller = ScrollController(initialScrollOffset: 50.0);
    addTearDown(controller.dispose);

    late double scrollOffset;
    controller.addListener(() {
      scrollOffset = controller.offset;
    });

    await tester.pumpWidget(buildTest(controller: controller));
    expect(controller.position.minScrollExtent, 0.0);
    expect(controller.position.pixels, 50.0);
    expect(controller.position.maxScrollExtent, 200.0);

    // The appbar's expandedHeight - initialScrollOffset = 150.
    expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 150.0);

    // Fully expand the appbar by scrolling (no animation) to 0.0.
    controller.jumpTo(0.0);
    await tester.pumpAndSettle();
    expect(scrollOffset, 0.0);
    expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 200.0);

    // Scroll back to 50.0 animating over 100ms.
    controller.animateTo(50.0, duration: const Duration(milliseconds: 100), curve: Curves.linear);
    await tester.pump();
    await tester.pump();
    expect(scrollOffset, 0.0);
    expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 200.0);
    await tester.pump(const Duration(milliseconds: 50)); // 50ms - halfway to scroll offset = 50.0.
    expect(scrollOffset, 25.0);
    expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 175.0);
    await tester.pump(
      const Duration(milliseconds: 50),
    ); // 100ms - all the way to scroll offset = 50.0.
    expect(scrollOffset, 50.0);
    expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 150.0);

    // Scroll to the end, (we're not scrolling to the end of the list that contains aaa1,
    // just to the end of the outer scrollview). Verify that the first item in each tab
    // is still visible.
    controller.jumpTo(controller.position.maxScrollExtent);
    await tester.pumpAndSettle();
    expect(scrollOffset, 200.0);
    expect(find.text('aaa1'), findsOneWidget);

    await tester.tap(find.text('BB'));
    await tester.pumpAndSettle();
    expect(find.text('bbb1'), findsOneWidget);

    await tester.tap(find.text('CC'));
    await tester.pumpAndSettle();
    expect(find.text('ccc1'), findsOneWidget);

    await tester.tap(find.text('DD'));
    await tester.pumpAndSettle();
    expect(find.text('ddd1'), findsOneWidget);
  });

  testWidgets('Three NestedScrollViews with one ScrollController', (WidgetTester tester) async {
    final TrackingScrollController controller = TrackingScrollController();
    addTearDown(controller.dispose);
    expect(controller.mostRecentlyUpdatedPosition, isNull);
    expect(controller.initialScrollOffset, 0.0);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: PageView(
          children: <Widget>[
            buildTest(controller: controller, title: 'Page0'),
            buildTest(controller: controller, title: 'Page1'),
            buildTest(controller: controller, title: 'Page2'),
          ],
        ),
      ),
    );

    // Initially Page0 is visible and Page0's appbar is fully expanded (height = 200.0).
    expect(find.text('Page0'), findsOneWidget);
    expect(find.text('Page1'), findsNothing);
    expect(find.text('Page2'), findsNothing);
    expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 200.0);

    // A scroll collapses Page0's appbar to 150.0.
    controller.jumpTo(50.0);
    await tester.pumpAndSettle();
    expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 150.0);

    // Fling to Page1. Page1's appbar height is the same as the appbar for Page0.
    await tester.fling(find.text('Page0'), const Offset(-100.0, 0.0), 10000.0);
    await tester.pumpAndSettle();
    expect(find.text('Page0'), findsNothing);
    expect(find.text('Page1'), findsOneWidget);
    expect(find.text('Page2'), findsNothing);
    expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 150.0);

    // Expand Page1's appbar and then fling to Page2. Page2's appbar appears
    // fully expanded.
    controller.jumpTo(0.0);
    await tester.pumpAndSettle();
    expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 200.0);
    await tester.fling(find.text('Page1'), const Offset(-100.0, 0.0), 10000.0);
    await tester.pumpAndSettle();
    expect(find.text('Page0'), findsNothing);
    expect(find.text('Page1'), findsNothing);
    expect(find.text('Page2'), findsOneWidget);
    expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 200.0);
  });

  testWidgets('NestedScrollViews with custom physics', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Localizations(
          locale: const Locale('en', 'US'),
          delegates: const <LocalizationsDelegate<dynamic>>[
            DefaultMaterialLocalizations.delegate,
            DefaultWidgetsLocalizations.delegate,
          ],
          child: MediaQuery(
            data: const MediaQueryData(),
            child: NestedScrollView(
              physics: const _CustomPhysics(),
              headerSliverBuilder: (BuildContext context, bool innerBoxIsScrolled) {
                return <Widget>[const SliverAppBar(floating: true, title: Text('AA'))];
              },
              body: Container(),
            ),
          ),
        ),
      ),
    );
    expect(find.text('AA'), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 500));
    final Offset point1 = tester.getCenter(find.text('AA'));
    await tester.dragFrom(point1, const Offset(0.0, 200.0));
    await tester.pump(const Duration(milliseconds: 20));
    final Offset point2 = tester.getCenter(find.text('AA', skipOffstage: false));
    expect(point1.dy, greaterThan(point2.dy));
  });

  testWidgets('NestedScrollViews respect NeverScrollableScrollPhysics', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/113753
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Localizations(
          locale: const Locale('en', 'US'),
          delegates: const <LocalizationsDelegate<dynamic>>[
            DefaultMaterialLocalizations.delegate,
            DefaultWidgetsLocalizations.delegate,
          ],
          child: MediaQuery(
            data: const MediaQueryData(),
            child: NestedScrollView(
              physics: const NeverScrollableScrollPhysics(),
              headerSliverBuilder: (BuildContext context, bool innerBoxIsScrolled) {
                return <Widget>[const SliverAppBar(floating: true, title: Text('AA'))];
              },
              body: Container(),
            ),
          ),
        ),
      ),
    );

    expect(find.text('AA'), findsOneWidget);
    final Offset point1 = tester.getCenter(find.text('AA'));

    await tester.dragFrom(point1, const Offset(0.0, -200.0));
    await tester.pump();

    final Offset point2 = tester.getCenter(find.text('AA', skipOffstage: false));
    expect(point1, point2);
  });

  testWidgets('NestedScrollView and internal scrolling', (WidgetTester tester) async {
    debugDisableShadows = false;
    const List<String> tabs = <String>['Hello', 'World'];
    int buildCount = 0;
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child:
          // THE FOLLOWING SECTION IS FROM THE NestedScrollView DOCUMENTATION
          // (EXCEPT FOR THE CHANGES TO THE buildCount COUNTER)
          DefaultTabController(
            length: tabs.length, // This is the number of tabs.
            child: NestedScrollView(
              dragStartBehavior: DragStartBehavior.down,
              headerSliverBuilder: (BuildContext context, bool innerBoxIsScrolled) {
                buildCount += 1; // THIS LINE IS NOT IN THE ORIGINAL -- ADDED FOR TEST
                // These are the slivers that show up in the "outer" scroll view.
                return <Widget>[
                  SliverOverlapAbsorber(
                    // This widget takes the overlapping behavior of the
                    // SliverAppBar, and redirects it to the SliverOverlapInjector
                    // below. If it is missing, then it is possible for the nested
                    // "inner" scroll view below to end up under the SliverAppBar
                    // even when the inner scroll view thinks it has not been
                    // scrolled. This is not necessary if the
                    // "headerSliverBuilder" only builds widgets that do not
                    // overlap the next sliver.
                    handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                    sliver: SliverAppBar(
                      title: const Text('Books'), // This is the title in the app bar.
                      pinned: true,
                      expandedHeight: 150.0,
                      // The "forceElevated" property causes the SliverAppBar to
                      // show a shadow. The "innerBoxIsScrolled" parameter is true
                      // when the inner scroll view is scrolled beyond its "zero"
                      // point, i.e. when it appears to be scrolled below the
                      // SliverAppBar. Without this, there are cases where the
                      // shadow would appear or not appear inappropriately,
                      // because the SliverAppBar is not actually aware of the
                      // precise position of the inner scroll views.
                      forceElevated: innerBoxIsScrolled,
                      bottom: TabBar(
                        // These are the widgets to put in each tab in the tab
                        // bar.
                        tabs: tabs.map<Widget>((String name) => Tab(text: name)).toList(),
                        dragStartBehavior: DragStartBehavior.down,
                      ),
                    ),
                  ),
                ];
              },
              body: TabBarView(
                dragStartBehavior: DragStartBehavior.down,
                // These are the contents of the tab views, below the tabs.
                children:
                    tabs.map<Widget>((String name) {
                      return SafeArea(
                        top: false,
                        bottom: false,
                        child: Builder(
                          // This Builder is needed to provide a BuildContext that is
                          // "inside" the NestedScrollView, so that
                          // sliverOverlapAbsorberHandleFor() can find the
                          // NestedScrollView.
                          builder: (BuildContext context) {
                            return CustomScrollView(
                              // The "controller" and "primary" members should be left
                              // unset, so that the NestedScrollView can control this
                              // inner scroll view.
                              // If the "controller" property is set, then this scroll
                              // view will not be associated with the
                              // NestedScrollView. The PageStorageKey should be unique
                              // to this ScrollView; it allows the list to remember
                              // its scroll position when the tab view is not on the
                              // screen.
                              key: PageStorageKey<String>(name),
                              dragStartBehavior: DragStartBehavior.down,
                              slivers: <Widget>[
                                SliverOverlapInjector(
                                  // This is the flip side of the
                                  // SliverOverlapAbsorber above.
                                  handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                                ),
                                SliverPadding(
                                  padding: const EdgeInsets.all(8.0),
                                  // In this example, the inner scroll view has
                                  // fixed-height list items, hence the use of
                                  // SliverFixedExtentList. However, one could use any
                                  // sliver widget here, e.g. SliverList or
                                  // SliverGrid.
                                  sliver: SliverFixedExtentList(
                                    // The items in this example are fixed to 48
                                    // pixels high. This matches the Material Design
                                    // spec for ListTile widgets.
                                    itemExtent: 48.0,
                                    delegate: SliverChildBuilderDelegate(
                                      (BuildContext context, int index) {
                                        // This builder is called for each child.
                                        // In this example, we just number each list
                                        // item.
                                        return ListTile(title: Text('Item $index'));
                                      },
                                      // The childCount of the
                                      // SliverChildBuilderDelegate specifies how many
                                      // children this inner list has. In this
                                      // example, each tab has a list of exactly 30
                                      // items, but this is arbitrary.
                                      childCount: 30,
                                    ),
                                  ),
                                ),
                              ],
                            );
                          },
                        ),
                      );
                    }).toList(),
              ),
            ),
          ),
          // END
        ),
      ),
    );

    Object? dfsFindPhysicalLayer(RenderObject object) {
      expect(object, isNotNull);
      if (object is RenderPhysicalModel || object is RenderPhysicalShape) {
        return object;
      }
      final List<RenderObject> children = <RenderObject>[];
      object.visitChildren(children.add);
      for (final RenderObject child in children) {
        final Object? result = dfsFindPhysicalLayer(child);
        if (result != null) {
          return result;
        }
      }
      return null;
    }

    final RenderObject nestedScrollViewLayer =
        find.byType(NestedScrollView).evaluate().first.renderObject!;
    void checkPhysicalLayer({required double elevation}) {
      final dynamic physicalModel = dfsFindPhysicalLayer(nestedScrollViewLayer);
      expect(physicalModel, isNotNull);
      // ignore: avoid_dynamic_calls
      expect(physicalModel.elevation, equals(elevation));
    }

    int expectedBuildCount = 0;
    expectedBuildCount += 1;
    expect(buildCount, expectedBuildCount);
    expect(find.text('Item 2'), findsOneWidget);
    expect(find.text('Item 18'), findsNothing);
    checkPhysicalLayer(elevation: 0);
    // scroll down
    final TestGesture gesture0 = await tester.startGesture(tester.getCenter(find.text('Item 2')));
    await gesture0.moveBy(
      const Offset(0.0, -120.0),
    ); // tiny bit more than the pinned app bar height (56px * 2)
    await tester.pump();
    expect(buildCount, expectedBuildCount);
    expect(find.text('Item 2'), findsOneWidget);
    expect(find.text('Item 18'), findsNothing);
    await gesture0.up();
    await tester.pump(const Duration(milliseconds: 1)); // start shadow animation
    expectedBuildCount += 1;
    expect(buildCount, expectedBuildCount);
    await tester.pump(const Duration(milliseconds: 1)); // during shadow animation
    expect(buildCount, expectedBuildCount);
    checkPhysicalLayer(elevation: 0.00018262863159179688);
    await tester.pump(const Duration(seconds: 1)); // end shadow animation
    expect(buildCount, expectedBuildCount);
    checkPhysicalLayer(elevation: 4);
    // scroll down
    final TestGesture gesture1 = await tester.startGesture(tester.getCenter(find.text('Item 2')));
    await gesture1.moveBy(const Offset(0.0, -800.0));
    await tester.pump();
    expect(buildCount, expectedBuildCount);
    checkPhysicalLayer(elevation: 4);
    expect(find.text('Item 2'), findsNothing);
    expect(find.text('Item 18'), findsOneWidget);
    await gesture1.up();
    await tester.pump(const Duration(seconds: 1));
    expect(buildCount, expectedBuildCount);
    checkPhysicalLayer(elevation: 4);
    // swipe left to bring in tap on the right
    final TestGesture gesture2 = await tester.startGesture(
      tester.getCenter(find.byType(NestedScrollView)),
    );
    await gesture2.moveBy(const Offset(-400.0, 0.0));
    await tester.pump();
    expect(buildCount, expectedBuildCount);
    expect(find.text('Item 18'), findsOneWidget);
    expect(find.text('Item 2'), findsOneWidget);
    expect(find.text('Item 0'), findsOneWidget);
    expect(
      tester.getTopLeft(find.ancestor(of: find.text('Item 0'), matching: find.byType(ListTile))).dy,
      tester.getBottomLeft(find.byType(AppBar)).dy + 8.0,
    );
    checkPhysicalLayer(elevation: 4);
    await gesture2.up();
    await tester.pump(); // start sideways scroll
    await tester.pump(
      const Duration(seconds: 1),
    ); // end sideways scroll, triggers shadow going away
    expect(buildCount, expectedBuildCount);
    await tester.pump(const Duration(seconds: 1)); // start shadow going away
    expectedBuildCount += 1;
    expect(buildCount, expectedBuildCount);
    await tester.pump(const Duration(seconds: 1)); // end shadow going away
    expect(buildCount, expectedBuildCount);
    expect(find.text('Item 18'), findsNothing);
    expect(find.text('Item 2'), findsOneWidget);
    checkPhysicalLayer(elevation: 0);
    await tester.pump(const Duration(seconds: 1)); // just checking we don't rebuild...
    expect(buildCount, expectedBuildCount);
    // peek left to see it's still in the right place
    final TestGesture gesture3 = await tester.startGesture(
      tester.getCenter(find.byType(NestedScrollView)),
    );
    await gesture3.moveBy(const Offset(400.0, 0.0));
    await tester.pump(); // bring the left page into view
    expect(buildCount, expectedBuildCount);
    await tester.pump(); // shadow comes back starting here
    expectedBuildCount += 1;
    expect(buildCount, expectedBuildCount);
    expect(find.text('Item 18'), findsOneWidget);
    expect(find.text('Item 2'), findsOneWidget);
    checkPhysicalLayer(elevation: 0);
    await tester.pump(const Duration(seconds: 1)); // shadow finishes coming back
    expect(buildCount, expectedBuildCount);
    checkPhysicalLayer(elevation: 4);
    await gesture3.moveBy(const Offset(-400.0, 0.0));
    await gesture3.up();
    await tester.pump(); // left tab view goes away
    expect(buildCount, expectedBuildCount);
    await tester.pump(); // shadow goes away starting here
    expectedBuildCount += 1;
    expect(buildCount, expectedBuildCount);
    checkPhysicalLayer(elevation: 4);
    await tester.pump(const Duration(seconds: 1)); // shadow finishes going away
    expect(buildCount, expectedBuildCount);
    checkPhysicalLayer(elevation: 0);
    // scroll back up
    final TestGesture gesture4 = await tester.startGesture(
      tester.getCenter(find.byType(NestedScrollView)),
    );
    await gesture4.moveBy(const Offset(0.0, 200.0)); // expands the appbar again
    await tester.pump();
    expect(buildCount, expectedBuildCount);
    expect(find.text('Item 2'), findsOneWidget);
    expect(find.text('Item 18'), findsNothing);
    checkPhysicalLayer(elevation: 0);
    await gesture4.up();
    await tester.pump(const Duration(seconds: 1));
    expect(buildCount, expectedBuildCount);
    checkPhysicalLayer(elevation: 0);
    // peek left to see it's now back at zero
    final TestGesture gesture5 = await tester.startGesture(
      tester.getCenter(find.byType(NestedScrollView)),
    );
    await gesture5.moveBy(const Offset(400.0, 0.0));
    await tester.pump(); // bring the left page into view
    await tester.pump(); // shadow would come back starting here, but there's no shadow to show
    expect(buildCount, expectedBuildCount);
    expect(find.text('Item 18'), findsNothing);
    expect(find.text('Item 2'), findsNWidgets(2));
    checkPhysicalLayer(elevation: 0);
    await tester.pump(const Duration(seconds: 1)); // shadow would be finished coming back
    checkPhysicalLayer(elevation: 0);
    await gesture5.up();
    await tester.pump(); // right tab view goes away
    await tester.pumpAndSettle();
    expect(buildCount, expectedBuildCount);
    checkPhysicalLayer(elevation: 0);
    debugDisableShadows = true;
  });

  testWidgets(
    'NestedScrollView and bouncing',
    (WidgetTester tester) async {
      // This verifies that overscroll bouncing works correctly on iOS. For
      // example, this checks that if you pull to overscroll, friction is applied;
      // it also makes sure that if you scroll back the other way, the scroll
      // positions of the inner and outer list don't have a discontinuity.
      const Key key1 = ValueKey<int>(1);
      const Key key2 = ValueKey<int>(2);
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: DefaultTabController(
              length: 1,
              child: NestedScrollView(
                dragStartBehavior: DragStartBehavior.down,
                headerSliverBuilder: (BuildContext context, bool innerBoxIsScrolled) {
                  return <Widget>[
                    const SliverPersistentHeader(
                      delegate: TestHeader(key: key1, minExtent: 100.0, maxExtent: 100.0),
                    ),
                  ];
                },
                body: const SingleChildScrollView(
                  dragStartBehavior: DragStartBehavior.down,
                  child: SizedBox(height: 1000.0, child: Placeholder(key: key2)),
                ),
              ),
            ),
          ),
        ),
      );
      expect(tester.getRect(find.byKey(key1)), const Rect.fromLTWH(0.0, 0.0, 800.0, 100.0));
      expect(tester.getRect(find.byKey(key2)), const Rect.fromLTWH(0.0, 100.0, 800.0, 1000.0));
      final TestGesture gesture = await tester.startGesture(const Offset(10.0, 10.0));
      await gesture.moveBy(const Offset(0.0, -10.0)); // scroll up
      await tester.pump();
      expect(tester.getRect(find.byKey(key1)), const Rect.fromLTWH(0.0, -10.0, 800.0, 100.0));
      expect(tester.getRect(find.byKey(key2)), const Rect.fromLTWH(0.0, 90.0, 800.0, 1000.0));
      await gesture.moveBy(const Offset(0.0, 10.0)); // scroll back to origin
      await tester.pump();
      expect(tester.getRect(find.byKey(key1)), const Rect.fromLTWH(0.0, 0.0, 800.0, 100.0));
      expect(tester.getRect(find.byKey(key2)), const Rect.fromLTWH(0.0, 100.0, 800.0, 1000.0));
      await gesture.moveBy(const Offset(0.0, 10.0)); // overscroll
      await gesture.moveBy(const Offset(0.0, 10.0)); // overscroll
      await gesture.moveBy(const Offset(0.0, 10.0)); // overscroll
      await tester.pump();
      expect(tester.getRect(find.byKey(key1)), const Rect.fromLTWH(0.0, 0.0, 800.0, 100.0));
      expect(tester.getRect(find.byKey(key2)).top, greaterThan(100.0));
      expect(tester.getRect(find.byKey(key2)).top, lessThan(130.0));
      await gesture.moveBy(const Offset(0.0, -1.0)); // scroll back a little
      await tester.pump();
      expect(tester.getRect(find.byKey(key1)), const Rect.fromLTWH(0.0, 0.0, 800.0, 100.0));
      expect(tester.getRect(find.byKey(key2)).top, greaterThan(100.0));
      expect(tester.getRect(find.byKey(key2)).top, lessThan(129.0));
      await gesture.moveBy(const Offset(0.0, -10.0)); // scroll back a lot
      await tester.pump();
      expect(tester.getRect(find.byKey(key1)), const Rect.fromLTWH(0.0, 0.0, 800.0, 100.0));
      await gesture.moveBy(const Offset(0.0, 20.0)); // overscroll again
      await tester.pump();
      expect(tester.getRect(find.byKey(key1)), const Rect.fromLTWH(0.0, 0.0, 800.0, 100.0));
      await gesture.up();
      debugDefaultTargetPlatformOverride = null;
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  group('NestedScrollViewState exposes inner and outer controllers', () {
    testWidgets('Scrolling by less than the outer extent does not scroll the inner body', (
      WidgetTester tester,
    ) async {
      final GlobalKey<NestedScrollViewState> globalKey = GlobalKey();
      await tester.pumpWidget(buildTest(key: globalKey, expanded: false));

      double appBarHeight = tester.renderObject<RenderBox>(find.byType(AppBar)).size.height;
      expect(appBarHeight, 104.0);
      final double scrollExtent = appBarHeight - 50.0;
      expect(globalKey.currentState!.outerController.offset, 0.0);
      expect(globalKey.currentState!.innerController.offset, 0.0);

      // The scroll gesture should occur in the inner body, so the whole
      // scroll view is scrolled.
      final TestGesture gesture = await tester.startGesture(Offset(0.0, appBarHeight + 1.0));
      await gesture.moveBy(Offset(0.0, -scrollExtent));
      await tester.pump();

      appBarHeight = tester.renderObject<RenderBox>(find.byType(AppBar)).size.height;
      // This is not an expanded AppBar.
      expect(appBarHeight, 104.0);
      // The outer scroll controller should show an offset of the applied
      // scrollExtent.
      expect(globalKey.currentState!.outerController.offset, 54.0);
      // the inner scroll controller should not have scrolled.
      expect(globalKey.currentState!.innerController.offset, 0.0);
    });

    testWidgets('Scrolling by exactly the outer extent does not scroll the inner body', (
      WidgetTester tester,
    ) async {
      final GlobalKey<NestedScrollViewState> globalKey = GlobalKey();
      await tester.pumpWidget(buildTest(key: globalKey, expanded: false));

      double appBarHeight = tester.renderObject<RenderBox>(find.byType(AppBar)).size.height;
      expect(appBarHeight, 104.0);
      final double scrollExtent = appBarHeight;
      expect(globalKey.currentState!.outerController.offset, 0.0);
      expect(globalKey.currentState!.innerController.offset, 0.0);

      // The scroll gesture should occur in the inner body, so the whole
      // scroll view is scrolled.
      final TestGesture gesture = await tester.startGesture(Offset(0.0, appBarHeight + 1.0));
      await gesture.moveBy(Offset(0.0, -scrollExtent));
      await tester.pump();

      appBarHeight = tester.renderObject<RenderBox>(find.byType(AppBar)).size.height;
      // This is not an expanded AppBar.
      expect(appBarHeight, 104.0);
      // The outer scroll controller should show an offset of the applied
      // scrollExtent.
      expect(globalKey.currentState!.outerController.offset, 104.0);
      // the inner scroll controller should not have scrolled.
      expect(globalKey.currentState!.innerController.offset, 0.0);
    });

    testWidgets('Scrolling by greater than the outer extent scrolls the inner body', (
      WidgetTester tester,
    ) async {
      final GlobalKey<NestedScrollViewState> globalKey = GlobalKey();
      await tester.pumpWidget(buildTest(key: globalKey, expanded: false));

      double appBarHeight = tester.renderObject<RenderBox>(find.byType(AppBar)).size.height;
      expect(appBarHeight, 104.0);
      final double scrollExtent = appBarHeight + 50.0;
      expect(globalKey.currentState!.outerController.offset, 0.0);
      expect(globalKey.currentState!.innerController.offset, 0.0);

      // The scroll gesture should occur in the inner body, so the whole
      // scroll view is scrolled.
      final TestGesture gesture = await tester.startGesture(Offset(0.0, appBarHeight + 1.0));
      await gesture.moveBy(Offset(0.0, -scrollExtent));
      await tester.pump();

      appBarHeight = tester.renderObject<RenderBox>(find.byType(AppBar)).size.height;
      // This is not an expanded AppBar.
      expect(appBarHeight, 104.0);
      // The outer scroll controller should show an offset of the applied
      // scrollExtent.
      expect(globalKey.currentState!.outerController.offset, appBarHeight);
      // the inner scroll controller should have scrolled equivalent to the
      // difference between the applied scrollExtent and the outer extent.
      expect(globalKey.currentState!.innerController.offset, scrollExtent - appBarHeight);
    });

    testWidgets('Inertia-cancel event does not modify either position.', (
      WidgetTester tester,
    ) async {
      final GlobalKey<NestedScrollViewState> globalKey = GlobalKey();
      await tester.pumpWidget(buildTest(key: globalKey, expanded: false));

      double appBarHeight = tester.renderObject<RenderBox>(find.byType(AppBar)).size.height;
      expect(appBarHeight, 104.0);
      final double scrollExtent = appBarHeight + 50.0;
      expect(globalKey.currentState!.outerController.offset, 0.0);
      expect(globalKey.currentState!.innerController.offset, 0.0);

      // The scroll gesture should occur in the inner body, so the whole
      // scroll view is scrolled.
      final TestGesture gesture = await tester.startGesture(Offset(0.0, appBarHeight + 1.0));
      await gesture.moveBy(Offset(0.0, -scrollExtent));
      await tester.pump();

      appBarHeight = tester.renderObject<RenderBox>(find.byType(AppBar)).size.height;
      // This is not an expanded AppBar.
      expect(appBarHeight, 104.0);
      // The outer scroll controller should show an offset of the applied
      // scrollExtent.
      expect(globalKey.currentState!.outerController.offset, appBarHeight);
      // the inner scroll controller should have scrolled equivalent to the
      // difference between the applied scrollExtent and the outer extent.
      expect(globalKey.currentState!.innerController.offset, scrollExtent - appBarHeight);

      final TestPointer testPointer = TestPointer(3, ui.PointerDeviceKind.trackpad);
      await tester.sendEventToBinding(
        testPointer.addPointer(location: Offset(0.0, appBarHeight + 1.0)),
      );
      await tester.sendEventToBinding(testPointer.scrollInertiaCancel());
      // ensure no change.
      expect(globalKey.currentState!.outerController.offset, appBarHeight);
      expect(globalKey.currentState!.innerController.offset, scrollExtent - appBarHeight);
    });

    testWidgets('scrolling by less than the expanded outer extent does not scroll the inner body', (
      WidgetTester tester,
    ) async {
      final GlobalKey<NestedScrollViewState> globalKey = GlobalKey();
      await tester.pumpWidget(buildTest(key: globalKey));

      double appBarHeight = tester.renderObject<RenderBox>(find.byType(AppBar)).size.height;
      expect(appBarHeight, 200.0);
      final double scrollExtent = appBarHeight - 50.0;
      expect(globalKey.currentState!.outerController.offset, 0.0);
      expect(globalKey.currentState!.innerController.offset, 0.0);

      // The scroll gesture should occur in the inner body, so the whole
      // scroll view is scrolled.
      final TestGesture gesture = await tester.startGesture(Offset(0.0, appBarHeight + 1.0));
      await gesture.moveBy(Offset(0.0, -scrollExtent));
      await tester.pump();

      appBarHeight = tester.renderObject<RenderBox>(find.byType(AppBar)).size.height;
      // This is an expanding AppBar.
      expect(appBarHeight, 104.0);
      // The outer scroll controller should show an offset of the applied
      // scrollExtent.
      expect(globalKey.currentState!.outerController.offset, 150.0);
      // the inner scroll controller should not have scrolled.
      expect(globalKey.currentState!.innerController.offset, 0.0);
    });

    testWidgets('scrolling by exactly the expanded outer extent does not scroll the inner body', (
      WidgetTester tester,
    ) async {
      final GlobalKey<NestedScrollViewState> globalKey = GlobalKey();
      await tester.pumpWidget(buildTest(key: globalKey));

      double appBarHeight = tester.renderObject<RenderBox>(find.byType(AppBar)).size.height;
      expect(appBarHeight, 200.0);
      final double scrollExtent = appBarHeight;
      expect(globalKey.currentState!.outerController.offset, 0.0);
      expect(globalKey.currentState!.innerController.offset, 0.0);

      // The scroll gesture should occur in the inner body, so the whole
      // scroll view is scrolled.
      final TestGesture gesture = await tester.startGesture(Offset(0.0, appBarHeight + 1.0));
      await gesture.moveBy(Offset(0.0, -scrollExtent));
      await tester.pump();

      appBarHeight = tester.renderObject<RenderBox>(find.byType(AppBar)).size.height;
      // This is an expanding AppBar.
      expect(appBarHeight, 104.0);
      // The outer scroll controller should show an offset of the applied
      // scrollExtent.
      expect(globalKey.currentState!.outerController.offset, 200.0);
      // the inner scroll controller should not have scrolled.
      expect(globalKey.currentState!.innerController.offset, 0.0);
    });

    testWidgets('scrolling by greater than the expanded outer extent scrolls the inner body', (
      WidgetTester tester,
    ) async {
      final GlobalKey<NestedScrollViewState> globalKey = GlobalKey();
      await tester.pumpWidget(buildTest(key: globalKey));

      double appBarHeight = tester.renderObject<RenderBox>(find.byType(AppBar)).size.height;
      expect(appBarHeight, 200.0);
      final double scrollExtent = appBarHeight + 50.0;
      expect(globalKey.currentState!.outerController.offset, 0.0);
      expect(globalKey.currentState!.innerController.offset, 0.0);

      // The scroll gesture should occur in the inner body, so the whole
      // scroll view is scrolled.
      final TestGesture gesture = await tester.startGesture(Offset(0.0, appBarHeight + 1.0));
      await gesture.moveBy(Offset(0.0, -scrollExtent));
      await tester.pump();

      appBarHeight = tester.renderObject<RenderBox>(find.byType(AppBar)).size.height;
      // This is an expanding AppBar.
      expect(appBarHeight, 104.0);
      // The outer scroll controller should show an offset of the applied
      // scrollExtent.
      expect(globalKey.currentState!.outerController.offset, 200.0);
      // the inner scroll controller should have scrolled equivalent to the
      // difference between the applied scrollExtent and the outer extent.
      expect(globalKey.currentState!.innerController.offset, 50.0);
    });

    testWidgets(
      'NestedScrollViewState.outerController should correspond to NestedScrollView.controller',
      (WidgetTester tester) async {
        final GlobalKey<NestedScrollViewState> globalKey = GlobalKey();
        final ScrollController scrollController = ScrollController();
        addTearDown(scrollController.dispose);

        await tester.pumpWidget(buildTest(controller: scrollController, key: globalKey));

        // Scroll to compare offsets between controllers.
        final TestGesture gesture = await tester.startGesture(const Offset(0.0, 100.0));
        await gesture.moveBy(const Offset(0.0, -100.0));
        await tester.pump();

        expect(scrollController.offset, globalKey.currentState!.outerController.offset);
        expect(
          tester.widget<NestedScrollView>(find.byType(NestedScrollView)).controller!.offset,
          globalKey.currentState!.outerController.offset,
        );
      },
    );

    group('manipulating controllers when', () {
      testWidgets('outer: not scrolled, inner: not scrolled', (WidgetTester tester) async {
        final GlobalKey<NestedScrollViewState> globalKey1 = GlobalKey();
        await tester.pumpWidget(buildTest(key: globalKey1, expanded: false));
        expect(globalKey1.currentState!.outerController.position.pixels, 0.0);
        expect(globalKey1.currentState!.innerController.position.pixels, 0.0);
        final double appBarHeight = tester.renderObject<RenderBox>(find.byType(AppBar)).size.height;

        // Manipulating Inner
        globalKey1.currentState!.innerController.jumpTo(100.0);
        expect(globalKey1.currentState!.innerController.position.pixels, 100.0);
        expect(globalKey1.currentState!.outerController.position.pixels, appBarHeight);
        globalKey1.currentState!.innerController.jumpTo(0.0);
        expect(globalKey1.currentState!.innerController.position.pixels, 0.0);
        expect(globalKey1.currentState!.outerController.position.pixels, appBarHeight);

        // Reset
        final GlobalKey<NestedScrollViewState> globalKey2 = GlobalKey();
        await tester.pumpWidget(buildTest(key: globalKey2, expanded: false));
        expect(globalKey2.currentState!.outerController.position.pixels, 0.0);
        expect(globalKey2.currentState!.innerController.position.pixels, 0.0);

        // Manipulating Outer
        globalKey2.currentState!.outerController.jumpTo(100.0);
        expect(globalKey2.currentState!.innerController.position.pixels, 0.0);
        expect(globalKey2.currentState!.outerController.position.pixels, 100.0);
        globalKey2.currentState!.outerController.jumpTo(0.0);
        expect(globalKey2.currentState!.innerController.position.pixels, 0.0);
        expect(globalKey2.currentState!.outerController.position.pixels, 0.0);
      });

      testWidgets('outer: not scrolled, inner: scrolled', (WidgetTester tester) async {
        final GlobalKey<NestedScrollViewState> globalKey1 = GlobalKey();
        await tester.pumpWidget(buildTest(key: globalKey1, expanded: false));
        expect(globalKey1.currentState!.outerController.position.pixels, 0.0);
        globalKey1.currentState!.innerController.position.setPixels(10.0);
        expect(globalKey1.currentState!.innerController.position.pixels, 10.0);
        final double appBarHeight = tester.renderObject<RenderBox>(find.byType(AppBar)).size.height;

        // Manipulating Inner
        globalKey1.currentState!.innerController.jumpTo(100.0);
        expect(globalKey1.currentState!.innerController.position.pixels, 100.0);
        expect(globalKey1.currentState!.outerController.position.pixels, appBarHeight);
        globalKey1.currentState!.innerController.jumpTo(0.0);
        expect(globalKey1.currentState!.innerController.position.pixels, 0.0);
        expect(globalKey1.currentState!.outerController.position.pixels, appBarHeight);

        // Reset
        final GlobalKey<NestedScrollViewState> globalKey2 = GlobalKey();
        await tester.pumpWidget(buildTest(key: globalKey2, expanded: false));
        expect(globalKey2.currentState!.outerController.position.pixels, 0.0);
        globalKey2.currentState!.innerController.position.setPixels(10.0);
        expect(globalKey2.currentState!.innerController.position.pixels, 10.0);

        // Manipulating Outer
        globalKey2.currentState!.outerController.jumpTo(100.0);
        expect(globalKey2.currentState!.innerController.position.pixels, 0.0);
        expect(globalKey2.currentState!.outerController.position.pixels, 100.0);
        globalKey2.currentState!.outerController.jumpTo(0.0);
        expect(globalKey2.currentState!.innerController.position.pixels, 0.0);
        expect(globalKey2.currentState!.outerController.position.pixels, 0.0);
      });

      testWidgets('outer: scrolled, inner: not scrolled', (WidgetTester tester) async {
        final GlobalKey<NestedScrollViewState> globalKey1 = GlobalKey();
        await tester.pumpWidget(buildTest(key: globalKey1, expanded: false));
        expect(globalKey1.currentState!.innerController.position.pixels, 0.0);
        globalKey1.currentState!.outerController.position.setPixels(10.0);
        expect(globalKey1.currentState!.outerController.position.pixels, 10.0);
        final double appBarHeight = tester.renderObject<RenderBox>(find.byType(AppBar)).size.height;

        // Manipulating Inner
        globalKey1.currentState!.innerController.jumpTo(100.0);
        expect(globalKey1.currentState!.innerController.position.pixels, 100.0);
        expect(globalKey1.currentState!.outerController.position.pixels, appBarHeight);
        globalKey1.currentState!.innerController.jumpTo(0.0);
        expect(globalKey1.currentState!.innerController.position.pixels, 0.0);
        expect(globalKey1.currentState!.outerController.position.pixels, appBarHeight);

        // Reset
        final GlobalKey<NestedScrollViewState> globalKey2 = GlobalKey();
        await tester.pumpWidget(buildTest(key: globalKey2, expanded: false));
        expect(globalKey2.currentState!.innerController.position.pixels, 0.0);
        globalKey2.currentState!.outerController.position.setPixels(10.0);
        expect(globalKey2.currentState!.outerController.position.pixels, 10.0);

        // Manipulating Outer
        globalKey2.currentState!.outerController.jumpTo(100.0);
        expect(globalKey2.currentState!.innerController.position.pixels, 0.0);
        expect(globalKey2.currentState!.outerController.position.pixels, 100.0);
        globalKey2.currentState!.outerController.jumpTo(0.0);
        expect(globalKey2.currentState!.innerController.position.pixels, 0.0);
        expect(globalKey2.currentState!.outerController.position.pixels, 0.0);
      });

      testWidgets('outer: scrolled, inner: scrolled', (WidgetTester tester) async {
        final GlobalKey<NestedScrollViewState> globalKey1 = GlobalKey();
        await tester.pumpWidget(buildTest(key: globalKey1, expanded: false));
        globalKey1.currentState!.innerController.position.setPixels(10.0);
        expect(globalKey1.currentState!.innerController.position.pixels, 10.0);
        globalKey1.currentState!.outerController.position.setPixels(10.0);
        expect(globalKey1.currentState!.outerController.position.pixels, 10.0);
        final double appBarHeight = tester.renderObject<RenderBox>(find.byType(AppBar)).size.height;

        // Manipulating Inner
        globalKey1.currentState!.innerController.jumpTo(100.0);
        expect(globalKey1.currentState!.innerController.position.pixels, 100.0);
        expect(globalKey1.currentState!.outerController.position.pixels, appBarHeight);
        globalKey1.currentState!.innerController.jumpTo(0.0);
        expect(globalKey1.currentState!.innerController.position.pixels, 0.0);
        expect(globalKey1.currentState!.outerController.position.pixels, appBarHeight);

        // Reset
        final GlobalKey<NestedScrollViewState> globalKey2 = GlobalKey();
        await tester.pumpWidget(buildTest(key: globalKey2, expanded: false));
        globalKey2.currentState!.innerController.position.setPixels(10.0);
        expect(globalKey2.currentState!.innerController.position.pixels, 10.0);
        globalKey2.currentState!.outerController.position.setPixels(10.0);
        expect(globalKey2.currentState!.outerController.position.pixels, 10.0);

        // Manipulating Outer
        globalKey2.currentState!.outerController.jumpTo(100.0);
        expect(globalKey2.currentState!.innerController.position.pixels, 0.0);
        expect(globalKey2.currentState!.outerController.position.pixels, 100.0);
        globalKey2.currentState!.outerController.jumpTo(0.0);
        expect(globalKey2.currentState!.innerController.position.pixels, 0.0);
        expect(globalKey2.currentState!.outerController.position.pixels, 0.0);
      });
    });
  });

  // Regression test for https://github.com/flutter/flutter/issues/39963.
  testWidgets('NestedScrollView with SliverOverlapAbsorber in or out of the first screen', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const _TestLayoutExtentIsNegative(1));
    await tester.pumpWidget(const _TestLayoutExtentIsNegative(10));
  });

  group('NestedScrollView can float outer sliver with inner scroll view:', () {
    Widget buildFloatTest({
      GlobalKey? appBarKey,
      GlobalKey? nestedKey,
      ScrollController? controller,
      bool floating = false,
      bool pinned = false,
      bool snap = false,
      bool nestedFloat = false,
      bool expanded = false,
    }) {
      return MaterialApp(
        home: Scaffold(
          body: NestedScrollView(
            key: nestedKey,
            controller: controller,
            floatHeaderSlivers: nestedFloat,
            headerSliverBuilder: (BuildContext context, bool innerBoxIsScrolled) {
              return <Widget>[
                SliverOverlapAbsorber(
                  handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                  sliver: SliverAppBar(
                    key: appBarKey,
                    title: const Text('Test Title'),
                    floating: floating,
                    pinned: pinned,
                    snap: snap,
                    expandedHeight: expanded ? 200.0 : 0.0,
                  ),
                ),
              ];
            },
            body: Builder(
              builder: (BuildContext context) {
                return CustomScrollView(
                  slivers: <Widget>[
                    SliverOverlapInjector(
                      handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                    ),
                    SliverFixedExtentList(
                      itemExtent: 50.0,
                      delegate: SliverChildBuilderDelegate(
                        (BuildContext context, int index) => ListTile(title: Text('Item $index')),
                        childCount: 30,
                      ),
                    ),
                  ],
                );
              },
            ),
          ),
        ),
      );
    }

    double verifyGeometry({
      required GlobalKey key,
      required double paintExtent,
      bool extentGreaterThan = false,
      bool extentLessThan = false,
      required bool visible,
    }) {
      final RenderSliver target = key.currentContext!.findRenderObject()! as RenderSliver;
      final SliverGeometry geometry = target.geometry!;
      expect(target.parent, isA<RenderSliverOverlapAbsorber>());
      expect(geometry.visible, visible);
      if (extentGreaterThan) {
        expect(geometry.paintExtent, greaterThan(paintExtent));
      } else if (extentLessThan) {
        expect(geometry.paintExtent, lessThan(paintExtent));
      } else {
        expect(geometry.paintExtent, paintExtent);
      }
      return geometry.paintExtent;
    }

    testWidgets('float', (WidgetTester tester) async {
      final GlobalKey appBarKey = GlobalKey();
      await tester.pumpWidget(
        buildFloatTest(floating: true, nestedFloat: true, appBarKey: appBarKey),
      );
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 56.0, visible: true);

      // Scroll away the outer scroll view and some of the inner scroll view.
      // We will not scroll back the same amount to indicate that we are
      // floating in before reaching the top of the inner scrollable.
      final Offset point1 = tester.getCenter(find.text('Item 5'));
      await tester.dragFrom(point1, const Offset(0.0, -300.0));
      await tester.pump();
      expect(find.text('Test Title'), findsNothing);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: appBarKey, paintExtent: 0.0, visible: false);

      // The outer scrollable should float back in, inner should not change
      await tester.dragFrom(point1, const Offset(0.0, 50.0));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 50.0, visible: true);

      // Float the rest of the way in.
      await tester.dragFrom(point1, const Offset(0.0, 150.0));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 56.0, visible: true);
    });

    testWidgets('float expanded', (WidgetTester tester) async {
      final GlobalKey appBarKey = GlobalKey();
      await tester.pumpWidget(
        buildFloatTest(floating: true, nestedFloat: true, expanded: true, appBarKey: appBarKey),
      );
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 200.0);
      verifyGeometry(key: appBarKey, paintExtent: 200.0, visible: true);

      // Scroll away the outer scroll view and some of the inner scroll view.
      // We will not scroll back the same amount to indicate that we are
      // floating in before reaching the top of the inner scrollable.
      final Offset point1 = tester.getCenter(find.text('Item 5'));
      await tester.dragFrom(point1, const Offset(0.0, -300.0));
      await tester.pump();
      expect(find.text('Test Title'), findsNothing);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: appBarKey, paintExtent: 0.0, visible: false);

      // The outer scrollable should float back in, inner should not change
      // On initial float in, the app bar is collapsed.
      await tester.dragFrom(point1, const Offset(0.0, 50.0));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 50.0, visible: true);

      // The inner scrollable should receive leftover delta after the outer has
      // been scrolled back in fully.
      await tester.dragFrom(point1, const Offset(0.0, 200.0));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 200.0);
      verifyGeometry(key: appBarKey, paintExtent: 200.0, visible: true);
    });

    testWidgets('float with pointer signal', (WidgetTester tester) async {
      final GlobalKey appBarKey = GlobalKey();
      await tester.pumpWidget(
        buildFloatTest(floating: true, nestedFloat: true, appBarKey: appBarKey),
      );

      final Offset scrollEventLocation = tester.getCenter(find.byType(NestedScrollView));
      final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
      // Create a hover event so that |testPointer| has a location when generating the scroll.
      testPointer.hover(scrollEventLocation);

      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 56.0, visible: true);

      // Scroll away the outer scroll view and some of the inner scroll view.
      // We will not scroll back the same amount to indicate that we are
      // floating in before reaching the top of the inner scrollable.
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 300.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsNothing);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: appBarKey, paintExtent: 0.0, visible: false);

      // The outer scrollable should float back in, inner should not change
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -50.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 50.0, visible: true);

      // Float the rest of the way in.
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -150.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 56.0, visible: true);
    });

    testWidgets('snap with pointer signal', (WidgetTester tester) async {
      final GlobalKey appBarKey = GlobalKey();
      await tester.pumpWidget(buildFloatTest(floating: true, snap: true, appBarKey: appBarKey));

      final Offset scrollEventLocation = tester.getCenter(find.byType(NestedScrollView));
      final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
      // Create a hover event so that |testPointer| has a location when generating the scroll.
      testPointer.hover(scrollEventLocation);

      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 56.0, visible: true);

      // Scroll away the outer scroll view and some of the inner scroll view.
      // We will not scroll back the same amount to indicate that we are
      // snapping in before reaching the top of the inner scrollable.
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 300.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsNothing);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: appBarKey, paintExtent: 0.0, visible: false);

      // The snap animation should be triggered to expand the app bar
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -30.0)));
      await tester.pumpAndSettle();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 56.0, visible: true);

      // Scroll away a bit more to trigger the snap close animation.
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 30.0)));
      await tester.pumpAndSettle();
      expect(find.text('Test Title'), findsNothing);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(find.byType(AppBar), findsNothing);
      verifyGeometry(key: appBarKey, paintExtent: 0.0, visible: false);
    });

    testWidgets('float expanded with pointer signal', (WidgetTester tester) async {
      final GlobalKey appBarKey = GlobalKey();
      await tester.pumpWidget(
        buildFloatTest(floating: true, nestedFloat: true, expanded: true, appBarKey: appBarKey),
      );

      final Offset scrollEventLocation = tester.getCenter(find.byType(NestedScrollView));
      final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
      // Create a hover event so that |testPointer| has a location when generating the scroll.
      testPointer.hover(scrollEventLocation);

      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 200.0);
      verifyGeometry(key: appBarKey, paintExtent: 200.0, visible: true);

      // Scroll away the outer scroll view and some of the inner scroll view.
      // We will not scroll back the same amount to indicate that we are
      // floating in before reaching the top of the inner scrollable.
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 300.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsNothing);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: appBarKey, paintExtent: 0.0, visible: false);

      // The outer scrollable should float back in, inner should not change
      // On initial float in, the app bar is collapsed.
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -50.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 50.0, visible: true);

      // The inner scrollable should receive leftover delta after the outer has
      // been scrolled back in fully.
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -200.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 200.0);
      verifyGeometry(key: appBarKey, paintExtent: 200.0, visible: true);
    });

    testWidgets('only snap', (WidgetTester tester) async {
      final GlobalKey appBarKey = GlobalKey();
      final GlobalKey<NestedScrollViewState> nestedKey = GlobalKey();
      await tester.pumpWidget(
        buildFloatTest(floating: true, snap: true, appBarKey: appBarKey, nestedKey: nestedKey),
      );
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 56.0, visible: true);

      // Scroll down the list, the app bar should scroll away and no longer be
      // visible.
      final Offset point1 = tester.getCenter(find.text('Item 5'));
      await tester.dragFrom(point1, const Offset(0.0, -300.0));
      await tester.pump();
      expect(find.text('Test Title'), findsNothing);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: appBarKey, paintExtent: 0.0, visible: false);
      // The outer scroll view should be at its full extent, here the size of
      // the app bar.
      expect(nestedKey.currentState!.outerController.offset, 56.0);

      // Animate In

      // Drag the scrollable up and down. The app bar should not snap open, nor
      // should it float in.
      final TestGesture animateInGesture = await tester.startGesture(point1);
      await animateInGesture.moveBy(const Offset(0.0, 100.0)); // Should not float in
      await tester.pump();
      expect(find.text('Test Title'), findsNothing);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: appBarKey, paintExtent: 0.0, visible: false);
      expect(nestedKey.currentState!.outerController.offset, 56.0);

      await animateInGesture.moveBy(const Offset(0.0, -50.0)); // No float out
      await tester.pump();
      expect(find.text('Test Title'), findsNothing);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: appBarKey, paintExtent: 0.0, visible: false);
      expect(nestedKey.currentState!.outerController.offset, 56.0);

      // Trigger the snap open animation: drag down and release
      await animateInGesture.moveBy(const Offset(0.0, 10.0));
      await animateInGesture.up();

      // Now verify that the appbar is animating open
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      double lastExtent = verifyGeometry(
        key: appBarKey,
        paintExtent: 10.0, // >10.0 since 0.0 + 10.0
        extentGreaterThan: true,
        visible: true,
      );
      // The outer scroll offset should remain unchanged.
      expect(nestedKey.currentState!.outerController.offset, 56.0);

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(
        key: appBarKey,
        paintExtent: lastExtent,
        extentGreaterThan: true,
        visible: true,
      );
      expect(nestedKey.currentState!.outerController.offset, 56.0);

      // The animation finishes when the appbar is full height.
      await tester.pumpAndSettle();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: appBarKey, paintExtent: 56.0, visible: true);
      expect(nestedKey.currentState!.outerController.offset, 56.0);

      // Animate Out

      // Trigger the snap close animation: drag up and release
      final TestGesture animateOutGesture = await tester.startGesture(point1);
      await animateOutGesture.moveBy(const Offset(0.0, -10.0));
      await animateOutGesture.up();

      // Now verify that the appbar is animating closed
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      lastExtent = verifyGeometry(
        key: appBarKey,
        paintExtent: 46.0, // <46.0 since 56.0 - 10.0
        extentLessThan: true,
        visible: true,
      );
      expect(nestedKey.currentState!.outerController.offset, 56.0);

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: appBarKey, paintExtent: lastExtent, extentLessThan: true, visible: true);
      expect(nestedKey.currentState!.outerController.offset, 56.0);

      // The animation finishes when the appbar is no longer in view.
      await tester.pumpAndSettle();
      expect(find.text('Test Title'), findsNothing);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: appBarKey, paintExtent: 0.0, visible: false);
      expect(nestedKey.currentState!.outerController.offset, 56.0);
    });

    testWidgets('only snap expanded', (WidgetTester tester) async {
      final GlobalKey appBarKey = GlobalKey();
      final GlobalKey<NestedScrollViewState> nestedKey = GlobalKey();
      await tester.pumpWidget(
        buildFloatTest(
          floating: true,
          snap: true,
          expanded: true,
          appBarKey: appBarKey,
          nestedKey: nestedKey,
        ),
      );
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 200.0);
      verifyGeometry(key: appBarKey, paintExtent: 200.0, visible: true);

      // Scroll down the list, the app bar should scroll away and no longer be
      // visible.
      final Offset point1 = tester.getCenter(find.text('Item 5'));
      await tester.dragFrom(point1, const Offset(0.0, -400.0));
      await tester.pump();
      expect(find.text('Test Title'), findsNothing);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: appBarKey, paintExtent: 0.0, visible: false);
      // The outer scroll view should be at its full extent, here the size of
      // the app bar.
      expect(nestedKey.currentState!.outerController.offset, 200.0);

      // Animate In

      // Drag the scrollable up and down. The app bar should not snap open, nor
      // should it float in.
      final TestGesture animateInGesture = await tester.startGesture(point1);
      await animateInGesture.moveBy(const Offset(0.0, 100.0)); // Should not float in
      await tester.pump();
      expect(find.text('Test Title'), findsNothing);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: appBarKey, paintExtent: 0.0, visible: false);
      expect(nestedKey.currentState!.outerController.offset, 200.0);

      await animateInGesture.moveBy(const Offset(0.0, -50.0)); // No float out
      await tester.pump();
      expect(find.text('Test Title'), findsNothing);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: appBarKey, paintExtent: 0.0, visible: false);
      expect(nestedKey.currentState!.outerController.offset, 200.0);

      // Trigger the snap open animation: drag down and release
      await animateInGesture.moveBy(const Offset(0.0, 10.0));
      await animateInGesture.up();

      // Now verify that the appbar is animating open
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      double lastExtent = verifyGeometry(
        key: appBarKey,
        paintExtent: 10.0, // >10.0 since 0.0 + 10.0
        extentGreaterThan: true,
        visible: true,
      );
      // The outer scroll offset should remain unchanged.
      expect(nestedKey.currentState!.outerController.offset, 200.0);

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(
        key: appBarKey,
        paintExtent: lastExtent,
        extentGreaterThan: true,
        visible: true,
      );
      expect(nestedKey.currentState!.outerController.offset, 200.0);

      // The animation finishes when the appbar is full height.
      await tester.pumpAndSettle();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: appBarKey, paintExtent: 200.0, visible: true);
      expect(nestedKey.currentState!.outerController.offset, 200.0);

      // Animate Out

      // Trigger the snap close animation: drag up and release
      final TestGesture animateOutGesture = await tester.startGesture(point1);
      await animateOutGesture.moveBy(const Offset(0.0, -10.0));
      await animateOutGesture.up();

      // Now verify that the appbar is animating closed
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      lastExtent = verifyGeometry(
        key: appBarKey,
        paintExtent: 190.0, // <190.0 since 200.0 - 10.0
        extentLessThan: true,
        visible: true,
      );
      expect(nestedKey.currentState!.outerController.offset, 200.0);

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: appBarKey, paintExtent: lastExtent, extentLessThan: true, visible: true);
      expect(nestedKey.currentState!.outerController.offset, 200.0);

      // The animation finishes when the appbar is no longer in view.
      await tester.pumpAndSettle();
      expect(find.text('Test Title'), findsNothing);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      verifyGeometry(key: appBarKey, paintExtent: 0.0, visible: false);
      expect(nestedKey.currentState!.outerController.offset, 200.0);
    });

    testWidgets('float pinned', (WidgetTester tester) async {
      // This configuration should have the same behavior of a pinned app bar.
      // No floating should happen, and the app bar should persist.
      final GlobalKey appBarKey = GlobalKey();
      await tester.pumpWidget(
        buildFloatTest(floating: true, pinned: true, nestedFloat: true, appBarKey: appBarKey),
      );
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 56.0, visible: true);

      // Scroll away the outer scroll view and some of the inner scroll view.
      final Offset point1 = tester.getCenter(find.text('Item 5'));
      await tester.dragFrom(point1, const Offset(0.0, -300.0));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 56.0, visible: true);

      await tester.dragFrom(point1, const Offset(0.0, 50.0));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 56.0, visible: true);

      await tester.dragFrom(point1, const Offset(0.0, 150.0));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 56.0, visible: true);
    });

    testWidgets('float pinned expanded', (WidgetTester tester) async {
      // Only the expanded portion (flexible space) of the app bar should float
      // in and out.
      final GlobalKey appBarKey = GlobalKey();
      await tester.pumpWidget(
        buildFloatTest(
          floating: true,
          pinned: true,
          expanded: true,
          nestedFloat: true,
          appBarKey: appBarKey,
        ),
      );
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 200.0);
      verifyGeometry(key: appBarKey, paintExtent: 200.0, visible: true);

      // Scroll away the outer scroll view and some of the inner scroll view.
      // The expanded portion of the app bar should collapse.
      final Offset point1 = tester.getCenter(find.text('Item 5'));
      await tester.dragFrom(point1, const Offset(0.0, -300.0));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 56.0, visible: true);

      // Scroll back some, the app bar should expand.
      await tester.dragFrom(point1, const Offset(0.0, 50.0));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(
        tester.renderObject<RenderBox>(find.byType(AppBar)).size.height,
        106.0, // 56.0 + 50.0
      );
      verifyGeometry(key: appBarKey, paintExtent: 106.0, visible: true);

      // Finish scrolling the rest of the way in.
      await tester.dragFrom(point1, const Offset(0.0, 150.0));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 200.0);
      verifyGeometry(key: appBarKey, paintExtent: 200.0, visible: true);
    });

    testWidgets('float pinned with pointer signal', (WidgetTester tester) async {
      // This configuration should have the same behavior of a pinned app bar.
      // No floating should happen, and the app bar should persist.
      final GlobalKey appBarKey = GlobalKey();
      await tester.pumpWidget(
        buildFloatTest(floating: true, pinned: true, nestedFloat: true, appBarKey: appBarKey),
      );

      final Offset scrollEventLocation = tester.getCenter(find.byType(NestedScrollView));
      final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
      // Create a hover event so that |testPointer| has a location when generating the scroll.
      testPointer.hover(scrollEventLocation);

      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 56.0, visible: true);

      // Scroll away the outer scroll view and some of the inner scroll view.
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 300.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 56.0, visible: true);

      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -50.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 56.0, visible: true);

      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -150.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 56.0, visible: true);
    });

    testWidgets('float pinned expanded with pointer signal', (WidgetTester tester) async {
      // Only the expanded portion (flexible space) of the app bar should float
      // in and out.
      final GlobalKey appBarKey = GlobalKey();
      await tester.pumpWidget(
        buildFloatTest(
          floating: true,
          pinned: true,
          expanded: true,
          nestedFloat: true,
          appBarKey: appBarKey,
        ),
      );

      final Offset scrollEventLocation = tester.getCenter(find.byType(NestedScrollView));
      final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
      // Create a hover event so that |testPointer| has a location when generating the scroll.
      testPointer.hover(scrollEventLocation);

      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 200.0);
      verifyGeometry(key: appBarKey, paintExtent: 200.0, visible: true);

      // Scroll away the outer scroll view and some of the inner scroll view.
      // The expanded portion of the app bar should collapse.
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 300.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 56.0);
      verifyGeometry(key: appBarKey, paintExtent: 56.0, visible: true);

      // Scroll back some, the app bar should expand.
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -50.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 5'), findsOneWidget);
      expect(
        tester.renderObject<RenderBox>(find.byType(AppBar)).size.height,
        106.0, // 56.0 + 50.0
      );
      verifyGeometry(key: appBarKey, paintExtent: 106.0, visible: true);

      // Finish scrolling the rest of the way in.
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -150.0)));
      await tester.pump();
      expect(find.text('Test Title'), findsOneWidget);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 5'), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 200.0);
      verifyGeometry(key: appBarKey, paintExtent: 200.0, visible: true);
    });
  });

  group('Correctly handles 0 velocity inner ballistic scroll activity:', () {
    // Regression tests for https://github.com/flutter/flutter/issues/17096
    Widget buildBallisticTest(ScrollController controller) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: NestedScrollView(
            controller: controller,
            headerSliverBuilder: (BuildContext context, bool innerBoxIsScrolled) {
              return <Widget>[const SliverAppBar(pinned: true, expandedHeight: 200.0)];
            },
            body: ListView.builder(
              itemCount: 50,
              itemBuilder: (BuildContext context, int index) {
                return Padding(padding: const EdgeInsets.all(8.0), child: Text('Item $index'));
              },
            ),
          ),
        ),
      );
    }

    testWidgets(
      'overscroll, hold for 0 velocity, and release',
      (WidgetTester tester) async {
        // Dragging into an overscroll and holding so that when released, the
        // ballistic scroll activity has a 0 velocity.
        final ScrollController controller = ScrollController();
        addTearDown(controller.dispose);
        await tester.pumpWidget(buildBallisticTest(controller));
        // Last item of the inner scroll view.
        expect(find.text('Item 49'), findsNothing);

        // Scroll to bottom
        await tester.fling(find.text('Item 3'), const Offset(0.0, -50.0), 10000.0);
        await tester.pumpAndSettle();

        // End of list
        expect(find.text('Item 49'), findsOneWidget);
        expect(tester.getCenter(find.text('Item 49')).dy, equals(585.0));

        // Overscroll, dragging like this will release with 0 velocity.
        await tester.drag(find.text('Item 49'), const Offset(0.0, -50.0));
        await tester.pump();
        // If handled correctly, the last item should still be visible and
        // progressing back down to the bottom edge, instead of jumping further
        // up the list and out of view.
        expect(find.text('Item 49'), findsOneWidget);
        await tester.pumpAndSettle();
        expect(tester.getCenter(find.text('Item 49')).dy, equals(585.0));
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
    );

    testWidgets(
      'overscroll, release, and tap',
      (WidgetTester tester) async {
        // Tapping while an inner ballistic scroll activity is in progress will
        // trigger a secondary ballistic scroll activity with a 0 velocity.
        final ScrollController controller = ScrollController();
        addTearDown(controller.dispose);
        await tester.pumpWidget(buildBallisticTest(controller));
        // Last item of the inner scroll view.
        expect(find.text('Item 49'), findsNothing);

        // Scroll to bottom
        await tester.fling(find.text('Item 3'), const Offset(0.0, -50.0), 10000.0);
        await tester.pumpAndSettle();

        // End of list
        expect(find.text('Item 49'), findsOneWidget);
        expect(tester.getCenter(find.text('Item 49')).dy, equals(585.0));

        // Fling again to trigger first ballistic activity.
        await tester.fling(find.text('Item 48'), const Offset(0.0, -50.0), 10000.0);
        await tester.pump();

        // Tap after releasing the overscroll to trigger secondary inner ballistic
        // scroll activity with 0 velocity.
        await tester.tap(find.text('Item 49'));
        await tester.pumpAndSettle();

        // If handled correctly, the ballistic scroll activity should finish
        // closing out the overscrolled area, with the last item visible at the
        // bottom.
        expect(find.text('Item 49'), findsOneWidget);
        expect(tester.getCenter(find.text('Item 49')).dy, equals(585.0));
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
    );
  });

  // Regression test for https://github.com/flutter/flutter/issues/63978
  testWidgets(
    'Inner _NestedScrollPosition.applyClampedDragUpdate correctly calculates range when in overscroll',
    (WidgetTester tester) async {
      final GlobalKey<NestedScrollViewState> nestedScrollView = GlobalKey();
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: NestedScrollView(
              key: nestedScrollView,
              headerSliverBuilder: (BuildContext context, bool boxIsScrolled) {
                return <Widget>[const SliverAppBar(expandedHeight: 200, title: Text('Test'))];
              },
              body: ListView.builder(
                itemExtent: 100.0,
                itemBuilder:
                    (BuildContext context, int index) => Container(
                      padding: const EdgeInsets.all(10.0),
                      child: Material(
                        color: index.isEven ? Colors.cyan : Colors.deepOrange,
                        child: Center(child: Text(index.toString())),
                      ),
                    ),
              ),
            ),
          ),
        ),
      );

      expect(nestedScrollView.currentState!.outerController.position.pixels, 0.0);
      expect(nestedScrollView.currentState!.innerController.position.pixels, 0.0);
      expect(nestedScrollView.currentState!.outerController.position.maxScrollExtent, 200.0);
      final Offset point = tester.getCenter(find.text('1'));
      // Drag slightly into overscroll in the inner position.
      final TestGesture gesture = await tester.startGesture(point);
      await gesture.moveBy(const Offset(0.0, 5.0));
      await tester.pump();
      expect(nestedScrollView.currentState!.outerController.position.pixels, 0.0);
      expect(nestedScrollView.currentState!.innerController.position.pixels, -5.0);
      // Move by a much larger delta than the amount of over scroll, in a very
      // short period of time.
      await gesture.moveBy(const Offset(0.0, -500.0));
      await tester.pump();
      // The overscrolled inner position should have closed, then passed the
      // correct remaining delta to the outer position, and finally any remainder
      // back to the inner position.
      expect(
        nestedScrollView.currentState!.outerController.position.pixels,
        nestedScrollView.currentState!.outerController.position.maxScrollExtent,
      );
      expect(nestedScrollView.currentState!.innerController.position.pixels, 295.0);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('Scroll pointer signal should not cause overscroll.', (WidgetTester tester) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(buildTest(controller: controller));

    final Offset scrollEventLocation = tester.getCenter(find.byType(NestedScrollView));
    final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
    // Create a hover event so that |testPointer| has a location when generating the scroll.
    testPointer.hover(scrollEventLocation);

    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));
    expect(controller.offset, 20);

    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -40.0)));
    expect(controller.offset, 0);

    await tester.tap(find.text('DD'));
    await tester.pumpAndSettle();

    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 1000000.0)));
    expect(find.text('ddd1'), findsOneWidget);
  });

  testWidgets('NestedScrollView basic scroll with pointer signal', (WidgetTester tester) async {
    await tester.pumpWidget(buildTest());
    expect(find.text('aaa2'), findsOneWidget);
    expect(find.text('aaa3'), findsNothing);
    expect(find.text('bbb1'), findsNothing);
    await tester.pump(const Duration(milliseconds: 250));
    expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 200.0);

    // Regression test for https://github.com/flutter/flutter/issues/55362
    final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
    // The offset is the responsibility of innerPosition.
    testPointer.hover(const Offset(0, 201));

    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));
    await tester.pump(const Duration(milliseconds: 250));
    expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 180.0);

    testPointer.hover(const Offset(0, 179));
    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));
    await tester.pump(const Duration(milliseconds: 250));
    expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 160.0);

    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));
    await tester.pump(const Duration(milliseconds: 250));
    expect(tester.renderObject<RenderBox>(find.byType(AppBar)).size.height, 140.0);
  });

  // Related to https://github.com/flutter/flutter/issues/64266
  testWidgets(
    'Holding scroll and Scroll pointer signal will update ScrollDirection.forward / ScrollDirection.reverse',
    (WidgetTester tester) async {
      ScrollDirection? lastUserScrollingDirection;

      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);
      await tester.pumpWidget(buildTest(controller: controller));

      controller.addListener(() {
        if (controller.position.userScrollDirection != ScrollDirection.idle) {
          lastUserScrollingDirection = controller.position.userScrollDirection;
        }
      });

      await tester.drag(find.byType(NestedScrollView), const Offset(0.0, -20.0), touchSlopY: 0.0);

      expect(lastUserScrollingDirection, ScrollDirection.reverse);

      final Offset scrollEventLocation = tester.getCenter(find.byType(NestedScrollView));
      final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
      // Create a hover event so that |testPointer| has a location when generating the scroll.
      testPointer.hover(scrollEventLocation);
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));

      expect(lastUserScrollingDirection, ScrollDirection.reverse);

      await tester.drag(find.byType(NestedScrollView), const Offset(0.0, 20.0), touchSlopY: 0.0);

      expect(lastUserScrollingDirection, ScrollDirection.forward);

      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -20.0)));

      expect(lastUserScrollingDirection, ScrollDirection.forward);
    },
  );

  // Regression test for https://github.com/flutter/flutter/issues/72257
  testWidgets('NestedScrollView works well when rebuilding during scheduleWarmUpFrame', (
    WidgetTester tester,
  ) async {
    bool? isScrolled;
    final Widget myApp = MaterialApp(
      home: Scaffold(
        body: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Focus(
              onFocusChange: (_) => setState(() {}),
              child: NestedScrollView(
                headerSliverBuilder: (BuildContext context, bool boxIsScrolled) {
                  isScrolled = boxIsScrolled;
                  return <Widget>[const SliverAppBar(expandedHeight: 200, title: Text('Test'))];
                },
                body: CustomScrollView(
                  slivers: <Widget>[
                    SliverList(
                      delegate: SliverChildBuilderDelegate((BuildContext context, int index) {
                        return const Text('');
                      }, childCount: 10),
                    ),
                  ],
                ),
              ),
            );
          },
        ),
      ),
    );

    await tester.pumpWidget(myApp, duration: Duration.zero, phase: EnginePhase.build);
    expect(isScrolled, false);
    expect(tester.takeException(), isNull);
  });

  // Regression test of https://github.com/flutter/flutter/issues/74372
  testWidgets('ScrollPosition can be accessed during `_updatePosition()`', (
    WidgetTester tester,
  ) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);
    late ScrollPosition position;

    Widget buildFrame({ScrollPhysics? physics}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Localizations(
          locale: const Locale('en', 'US'),
          delegates: const <LocalizationsDelegate<dynamic>>[
            DefaultMaterialLocalizations.delegate,
            DefaultWidgetsLocalizations.delegate,
          ],
          child: MediaQuery(
            data: const MediaQueryData(),
            child: NestedScrollView(
              controller: controller,
              physics: physics,
              headerSliverBuilder: (BuildContext context, bool innerBoxIsScrolled) {
                return <Widget>[
                  Builder(
                    builder: (BuildContext context) {
                      position = controller.position;
                      return const SliverAppBar(floating: true, title: Text('AA'));
                    },
                  ),
                ];
              },
              body: Container(),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame());
    expect(position.pixels, 0.0);

    //Trigger `_updatePosition()`.
    await tester.pumpWidget(buildFrame(physics: const _CustomPhysics()));
    expect(position.pixels, 0.0);
  });

  testWidgets("NestedScrollView doesn't crash due to precision error", (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/63825

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: NestedScrollView(
            floatHeaderSlivers: true,
            headerSliverBuilder:
                (BuildContext context, bool innerBoxIsScrolled) => <Widget>[
                  const SliverAppBar(expandedHeight: 250.0),
                ],
            body: CustomScrollView(
              physics: const BouncingScrollPhysics(),
              slivers: <Widget>[
                SliverPadding(
                  padding: const EdgeInsets.all(8.0),
                  sliver: SliverFixedExtentList(
                    itemExtent: 48.0,
                    delegate: SliverChildBuilderDelegate((BuildContext context, int index) {
                      return ListTile(title: Text('Item $index'));
                    }, childCount: 30),
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    // Scroll to bottom
    await tester.fling(find.text('Item 3'), const Offset(0.0, -250.0), 10000.0);
    await tester.pumpAndSettle();

    // Fling down for AppBar to show
    await tester.drag(find.text('Item 29'), const Offset(0.0, 250 - 133.7981622869321));

    // Fling up to trigger ballistic activity
    await tester.fling(find.text('Item 25'), const Offset(0.0, -50.0), 4000.0);
    await tester.pumpAndSettle();
  });

  testWidgets('NestedScrollViewCoordinator.pointerScroll dispatches correct scroll notifications', (
    WidgetTester tester,
  ) async {
    int scrollEnded = 0;
    int scrollStarted = 0;
    bool isScrolled = false;

    await tester.pumpWidget(
      MaterialApp(
        home: NotificationListener<ScrollNotification>(
          onNotification: (ScrollNotification notification) {
            if (notification is ScrollStartNotification) {
              scrollStarted += 1;
            } else if (notification is ScrollEndNotification) {
              scrollEnded += 1;
            }
            return false;
          },
          child: Scaffold(
            body: NestedScrollView(
              headerSliverBuilder: (BuildContext context, bool innerBoxIsScrolled) {
                isScrolled = innerBoxIsScrolled;
                return <Widget>[const SliverAppBar(expandedHeight: 250.0)];
              },
              body: CustomScrollView(
                physics: const BouncingScrollPhysics(),
                slivers: <Widget>[
                  SliverPadding(
                    padding: const EdgeInsets.all(8.0),
                    sliver: SliverFixedExtentList(
                      itemExtent: 48.0,
                      delegate: SliverChildBuilderDelegate((BuildContext context, int index) {
                        return ListTile(title: Text('Item $index'));
                      }, childCount: 30),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    );

    final Offset scrollEventLocation = tester.getCenter(find.byType(NestedScrollView));
    final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
    // Create a hover event so that |testPointer| has a location when generating the scroll.
    testPointer.hover(scrollEventLocation);
    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 300.0)));
    await tester.pumpAndSettle();

    expect(isScrolled, isTrue);
    // There should have been a notification for each nested position (2).
    expect(scrollStarted, 2);
    expect(scrollEnded, 2);
  });

  testWidgets('SliverAppBar.medium collapses in NestedScrollView', (WidgetTester tester) async {
    final GlobalKey<NestedScrollViewState> nestedScrollView = GlobalKey();
    const double collapsedAppBarHeight = 64;
    const double expandedAppBarHeight = 112;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: NestedScrollView(
            key: nestedScrollView,
            headerSliverBuilder: (BuildContext context, bool innerBoxIsScrolled) {
              return <Widget>[
                SliverOverlapAbsorber(
                  handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                  sliver: const SliverAppBar.medium(title: Text('AppBar Title')),
                ),
              ];
            },
            body: Builder(
              builder: (BuildContext context) {
                return CustomScrollView(
                  slivers: <Widget>[
                    SliverOverlapInjector(
                      handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                    ),
                    SliverFixedExtentList(
                      itemExtent: 50.0,
                      delegate: SliverChildBuilderDelegate(
                        (BuildContext context, int index) => ListTile(title: Text('Item $index')),
                        childCount: 30,
                      ),
                    ),
                  ],
                );
              },
            ),
          ),
        ),
      ),
    );

    // There are two widgets for the title.
    final Finder expandedTitle = find.text('AppBar Title').first;
    final Finder expandedTitleClip =
        find.ancestor(of: expandedTitle, matching: find.byType(ClipRect)).first;

    // Default, fully expanded app bar.
    expect(nestedScrollView.currentState?.outerController.offset, 0);
    expect(nestedScrollView.currentState?.innerController.offset, 0);
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), expandedAppBarHeight);
    expect(tester.getSize(expandedTitleClip).height, expandedAppBarHeight - collapsedAppBarHeight);

    // Scroll the expanded app bar partially out of view.
    final Offset point1 = tester.getCenter(find.text('Item 5'));
    await tester.dragFrom(point1, const Offset(0.0, -45.0));
    await tester.pump();
    expect(nestedScrollView.currentState?.outerController.offset, 45.0);
    expect(nestedScrollView.currentState?.innerController.offset, 0.0);
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), expandedAppBarHeight - 45);
    expect(
      tester.getSize(expandedTitleClip).height,
      expandedAppBarHeight - collapsedAppBarHeight - 45,
    );

    // Scroll so that it is completely collapsed.
    await tester.dragFrom(point1, const Offset(0.0, -555.0));
    await tester.pump();
    expect(nestedScrollView.currentState?.outerController.offset, 48.0);
    expect(nestedScrollView.currentState?.innerController.offset, 552.0);
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), collapsedAppBarHeight);
    expect(tester.getSize(expandedTitleClip).height, 0);

    // Scroll back to fully expanded.
    await tester.dragFrom(point1, const Offset(0.0, 600.0));
    await tester.pump();
    expect(nestedScrollView.currentState?.outerController.offset, 0);
    expect(nestedScrollView.currentState?.innerController.offset, 0);
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), expandedAppBarHeight);
    expect(tester.getSize(expandedTitleClip).height, expandedAppBarHeight - collapsedAppBarHeight);
  });

  testWidgets('SliverAppBar.large collapses in NestedScrollView', (WidgetTester tester) async {
    final GlobalKey<NestedScrollViewState> nestedScrollView = GlobalKey();
    const double collapsedAppBarHeight = 64;
    const double expandedAppBarHeight = 152;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: NestedScrollView(
            key: nestedScrollView,
            headerSliverBuilder: (BuildContext context, bool innerBoxIsScrolled) {
              return <Widget>[
                SliverOverlapAbsorber(
                  handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                  sliver: SliverAppBar.large(
                    title: const Text('AppBar Title'),
                    forceElevated: innerBoxIsScrolled,
                  ),
                ),
              ];
            },
            body: Builder(
              builder: (BuildContext context) {
                return CustomScrollView(
                  slivers: <Widget>[
                    SliverOverlapInjector(
                      handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                    ),
                    SliverFixedExtentList(
                      itemExtent: 50.0,
                      delegate: SliverChildBuilderDelegate(
                        (BuildContext context, int index) => ListTile(title: Text('Item $index')),
                        childCount: 30,
                      ),
                    ),
                  ],
                );
              },
            ),
          ),
        ),
      ),
    );

    // There are two widgets for the title.
    final Finder expandedTitle = find.text('AppBar Title').first;
    final Finder expandedTitleClip =
        find.ancestor(of: expandedTitle, matching: find.byType(ClipRect)).first;

    // Default, fully expanded app bar.
    expect(nestedScrollView.currentState?.outerController.offset, 0);
    expect(nestedScrollView.currentState?.innerController.offset, 0);
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), expandedAppBarHeight);
    expect(tester.getSize(expandedTitleClip).height, expandedAppBarHeight - collapsedAppBarHeight);

    // Scroll the expanded app bar partially out of view.
    final Offset point1 = tester.getCenter(find.text('Item 5'));
    await tester.dragFrom(point1, const Offset(0.0, -45.0));
    await tester.pump();
    expect(nestedScrollView.currentState?.outerController.offset, 45.0);
    expect(nestedScrollView.currentState?.innerController.offset, 0);
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), expandedAppBarHeight - 45);
    expect(
      tester.getSize(expandedTitleClip).height,
      expandedAppBarHeight - collapsedAppBarHeight - 45,
    );

    // Scroll so that it is completely collapsed.
    await tester.dragFrom(point1, const Offset(0.0, -555.0));
    await tester.pump();
    expect(nestedScrollView.currentState?.outerController.offset, 88.0);
    expect(nestedScrollView.currentState?.innerController.offset, 512.0);
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), collapsedAppBarHeight);
    expect(tester.getSize(expandedTitleClip).height, 0);

    // Scroll back to fully expanded.
    await tester.dragFrom(point1, const Offset(0.0, 600.0));
    await tester.pump();
    expect(nestedScrollView.currentState?.outerController.offset, 0);
    expect(nestedScrollView.currentState?.innerController.offset, 0);
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), expandedAppBarHeight);
    expect(tester.getSize(expandedTitleClip).height, expandedAppBarHeight - collapsedAppBarHeight);
  });

  testWidgets('NestedScrollView does not crash when inner scrollable changes while scrolling', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/126454.
    Widget buildApp({required bool nested}) {
      final Widget innerScrollable = ListView(children: const <Widget>[SizedBox(height: 1000)]);
      return MaterialApp(
        home: Scaffold(
          body: NestedScrollView(
            headerSliverBuilder: (BuildContext context, bool innerBoxIsScrolled) {
              return <Widget>[
                SliverAppBar(
                  title: const Text('Books'),
                  pinned: true,
                  expandedHeight: 150.0,
                  forceElevated: innerBoxIsScrolled,
                ),
              ];
            },
            body: nested ? Container(child: innerScrollable) : innerScrollable,
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp(nested: false));

    // Start a scroll.
    final TestGesture scrollDrag = await tester.startGesture(
      tester.getCenter(find.byType(ListView)),
    );
    await tester.pump();
    await scrollDrag.moveBy(const Offset(0, 50));
    await tester.pump();

    // Restructuring inner scrollable while scroll is in progress shouldn't crash.
    await tester.pumpWidget(buildApp(nested: true));
  });

  testWidgets('SliverOverlapInjector asserts when there is no SliverOverlapAbsorber', (
    WidgetTester tester,
  ) async {
    Widget buildApp() {
      return MaterialApp(
        home: Scaffold(
          body: NestedScrollView(
            headerSliverBuilder: (BuildContext context, bool innerBoxIsScrolled) {
              return <Widget>[const SliverAppBar()];
            },
            body: Builder(
              builder: (BuildContext context) {
                return CustomScrollView(
                  slivers: <Widget>[
                    SliverOverlapInjector(
                      handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                    ),
                  ],
                );
              },
            ),
          ),
        ),
      );
    }

    final List<Object> exceptions = <Object>[];
    final FlutterExceptionHandler? oldHandler = FlutterError.onError;
    FlutterError.onError = (FlutterErrorDetails details) {
      exceptions.add(details.exception);
    };
    await tester.pumpWidget(buildApp());
    FlutterError.onError = oldHandler;
    expect(exceptions.length, 4);
    expect(exceptions[0], isAssertionError);
    expect(
      (exceptions[0] as AssertionError).message,
      contains('SliverOverlapInjector has found no absorbed extent to inject.'),
    );
  });

  group('NestedScrollView properly sets drag', () {
    Future<bool> canDrag(WidgetTester tester) async {
      await tester.drag(find.byType(CustomScrollView), const Offset(0.0, -20.0), pointer: 1);
      await tester.pumpAndSettle();
      final NestedScrollViewState nestedScrollView = tester.state<NestedScrollViewState>(
        find.byType(NestedScrollView),
      );
      return nestedScrollView.outerController.position.pixels > 0.0 ||
          nestedScrollView.innerController.position.pixels > 0.0;
    }

    Widget buildTest({
      // The body length is to test when the nested scroll view should or
      // should not be allowing drag.
      required _BodyLength bodyLength,
      Widget? header,
      bool applyOverlap = false,
    }) {
      return MaterialApp(
        home: Scaffold(
          body: NestedScrollView(
            headerSliverBuilder: (BuildContext context, _) {
              if (applyOverlap) {
                return <Widget>[
                  SliverOverlapAbsorber(
                    handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                    sliver: header,
                  ),
                ];
              }
              return header != null ? <Widget>[header] : <Widget>[];
            },
            body: Builder(
              builder: (BuildContext context) {
                return CustomScrollView(
                  slivers: <Widget>[
                    SliverList.builder(
                      itemCount: switch (bodyLength) {
                        _BodyLength.short => 10,
                        _BodyLength.long => 100,
                      },
                      itemBuilder: (_, int index) => Text('Item $index'),
                    ),
                  ],
                );
              },
            ),
          ),
        ),
      );
    }

    testWidgets('when headerSliverBuilder is empty', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/117316
      // Regression test for https://github.com/flutter/flutter/issues/46089
      // Short body / long body
      for (final _BodyLength bodyLength in _BodyLength.values) {
        await tester.pumpWidget(buildTest(bodyLength: bodyLength));
        await tester.pumpAndSettle();
        switch (bodyLength) {
          case _BodyLength.short:
            expect(await canDrag(tester), isFalse);
          case _BodyLength.long:
            expect(await canDrag(tester), isTrue);
        }
      }
    }, variant: TargetPlatformVariant.all());

    testWidgets('when headerSliverBuilder extent is 0', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/79077
      // Short body / long body
      for (final _BodyLength bodyLength in _BodyLength.values) {
        // SliverPersistentHeader
        await tester.pumpWidget(
          buildTest(
            bodyLength: bodyLength,
            header: const SliverPersistentHeader(
              delegate: TestHeader(minExtent: 0.0, maxExtent: 0.0),
            ),
          ),
        );
        await tester.pumpAndSettle();
        switch (bodyLength) {
          case _BodyLength.short:
            expect(await canDrag(tester), isFalse);
          case _BodyLength.long:
            expect(await canDrag(tester), isTrue);
        }

        // SliverPersistentHeader pinned
        await tester.pumpWidget(
          buildTest(
            bodyLength: bodyLength,
            header: const SliverPersistentHeader(
              pinned: true,
              delegate: TestHeader(minExtent: 0.0, maxExtent: 0.0),
            ),
          ),
        );
        await tester.pumpAndSettle();
        switch (bodyLength) {
          case _BodyLength.short:
            expect(await canDrag(tester), isFalse);
          case _BodyLength.long:
            expect(await canDrag(tester), isTrue);
        }

        // SliverPersistentHeader floating
        await tester.pumpWidget(
          buildTest(
            bodyLength: bodyLength,
            header: const SliverPersistentHeader(
              floating: true,
              delegate: TestHeader(minExtent: 0.0, maxExtent: 0.0),
            ),
          ),
        );
        await tester.pumpAndSettle();
        switch (bodyLength) {
          case _BodyLength.short:
            expect(await canDrag(tester), isFalse);
          case _BodyLength.long:
            expect(await canDrag(tester), isTrue);
        }

        // SliverPersistentHeader pinned+floating
        await tester.pumpWidget(
          buildTest(
            bodyLength: bodyLength,
            header: const SliverPersistentHeader(
              pinned: true,
              floating: true,
              delegate: TestHeader(minExtent: 0.0, maxExtent: 0.0),
            ),
          ),
        );
        await tester.pumpAndSettle();
        switch (bodyLength) {
          case _BodyLength.short:
            expect(await canDrag(tester), isFalse);
          case _BodyLength.long:
            expect(await canDrag(tester), isTrue);
        }

        // SliverPersistentHeader w/ overlap
        await tester.pumpWidget(
          buildTest(
            bodyLength: bodyLength,
            applyOverlap: true,
            header: const SliverPersistentHeader(
              delegate: TestHeader(minExtent: 0.0, maxExtent: 0.0),
            ),
          ),
        );
        await tester.pumpAndSettle();
        switch (bodyLength) {
          case _BodyLength.short:
            expect(await canDrag(tester), isFalse);
          case _BodyLength.long:
            expect(await canDrag(tester), isTrue);
        }

        // SliverPersistentHeader pinned w/ overlap
        await tester.pumpWidget(
          buildTest(
            bodyLength: bodyLength,
            applyOverlap: true,
            header: const SliverPersistentHeader(
              pinned: true,
              delegate: TestHeader(minExtent: 0.0, maxExtent: 0.0),
            ),
          ),
        );
        await tester.pumpAndSettle();
        switch (bodyLength) {
          case _BodyLength.short:
            expect(await canDrag(tester), isFalse);
          case _BodyLength.long:
            expect(await canDrag(tester), isTrue);
        }

        // SliverPersistentHeader floating w/ overlap
        await tester.pumpWidget(
          buildTest(
            bodyLength: bodyLength,
            applyOverlap: true,
            header: const SliverPersistentHeader(
              floating: true,
              delegate: TestHeader(minExtent: 0.0, maxExtent: 0.0),
            ),
          ),
        );
        await tester.pumpAndSettle();
        switch (bodyLength) {
          case _BodyLength.short:
            expect(await canDrag(tester), isFalse);
          case _BodyLength.long:
            expect(await canDrag(tester), isTrue);
        }

        // SliverPersistentHeader pinned+floating w/ overlap
        await tester.pumpWidget(
          buildTest(
            bodyLength: bodyLength,
            applyOverlap: true,
            header: const SliverPersistentHeader(
              floating: true,
              pinned: true,
              delegate: TestHeader(minExtent: 0.0, maxExtent: 0.0),
            ),
          ),
        );
        await tester.pumpAndSettle();
        switch (bodyLength) {
          case _BodyLength.short:
            expect(await canDrag(tester), isFalse);
          case _BodyLength.long:
            expect(await canDrag(tester), isTrue);
        }
      }
    }, variant: TargetPlatformVariant.all());

    testWidgets('With a pinned SliverAppBar', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/110956
      // Regression test for https://github.com/flutter/flutter/issues/127282
      // Regression test for https://github.com/flutter/flutter/issues/32563
      // Regression test for https://github.com/flutter/flutter/issues/79077
      // Short / long body
      for (final _BodyLength bodyLength in _BodyLength.values) {
        await tester.pumpWidget(
          buildTest(
            bodyLength: bodyLength,
            applyOverlap: true,
            header: const SliverAppBar(
              title: Text('Test'),
              pinned: true,
              bottom: PreferredSize(preferredSize: Size.square(25), child: SizedBox()),
            ),
          ),
        );
        await tester.pumpAndSettle();
        switch (bodyLength) {
          case _BodyLength.short:
            expect(await canDrag(tester), isFalse);
          case _BodyLength.long:
            expect(await canDrag(tester), isTrue);
        }
      }
    });
  });

  // Regression test for https://github.com/flutter/flutter/issues/40740.
  testWidgets('Maintains scroll position of inactive tab', (WidgetTester tester) async {
    const List<String> tabs = <String>['Featured', 'Popular', 'Latest'];
    final List<Widget> tabViews = <Widget>[
      for (final String name in tabs)
        SafeArea(
          top: false,
          bottom: false,
          child: Builder(
            builder:
                (BuildContext context) => CustomScrollView(
                  key: PageStorageKey<String>(name),
                  slivers: <Widget>[
                    SliverOverlapInjector(
                      handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                    ),
                    SliverPadding(
                      padding: const EdgeInsets.all(8.0),
                      sliver: SliverList(
                        delegate: SliverChildBuilderDelegate(childCount: 30, (
                          BuildContext context,
                          int index,
                        ) {
                          return ListTile(title: Text('Item $index'));
                        }),
                      ),
                    ),
                  ],
                ),
          ),
        ),
    ];

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DefaultTabController(
            length: tabs.length,
            child: NestedScrollView(
              headerSliverBuilder:
                  (BuildContext context, bool innerBoxIsScrolled) => <Widget>[
                    SliverOverlapAbsorber(
                      handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                      sliver: SliverSafeArea(
                        top: false,
                        sliver: SliverAppBar(
                          title: const Text('Tab Demo'),
                          floating: true,
                          pinned: true,
                          snap: true,
                          forceElevated: innerBoxIsScrolled,
                          bottom: TabBar(tabs: tabs.map((String name) => Tab(text: name)).toList()),
                        ),
                      ),
                    ),
                  ],
              body: TabBarView(children: tabViews),
            ),
          ),
        ),
      ),
    );

    final Finder finder = find.text('Item 14', skipOffstage: false);
    final Finder findAny =
        find.descendant(of: find.byType(SliverList), matching: find.byType(ListTile)).first;

    Future<void> scroll(VerticalDirection direction) async {
      switch (direction) {
        case VerticalDirection.down:
          while (!tester.any(finder)) {
            await tester.fling(find.byType(Scaffold), const Offset(0, -50), 20);
            await tester.pumpAndSettle();
          }
          await tester.ensureVisible(finder);
        case VerticalDirection.up:
          await tester.fling(find.byType(Scaffold), const Offset(0, 20), 20);
          await tester.pumpAndSettle();
      }
    }

    double getScrollPosition() {
      return Scrollable.of(tester.element(findAny)).position.pixels;
    }

    expect(getScrollPosition(), 0.0);

    // Scroll toward the bottom of Tab 1.
    await scroll(VerticalDirection.down);
    final double tab1Position = getScrollPosition();

    // Switch to second tab.
    await tester.tap(find.text('Popular'));
    await tester.pumpAndSettle();

    // Scroll toward the bottom of the second tab.
    await scroll(VerticalDirection.down);
    final double tab2Position = getScrollPosition();

    // Scroll up a bit in the second tab.
    await scroll(VerticalDirection.up);
    expect(getScrollPosition(), lessThan(tab2Position));

    // Switch back to the first tab.
    await tester.tap(find.text('Featured'));
    await tester.pumpAndSettle();
    expect(getScrollPosition(), tab1Position);
  });

  testWidgets('$SliverOverlapAbsorberHandle dispatches creation in constructor', (
    WidgetTester widgetTester,
  ) async {
    await expectLater(
      await memoryEvents(
        () => SliverOverlapAbsorberHandle().dispose(),
        SliverOverlapAbsorberHandle,
      ),
      areCreateAndDispose,
    );
  });
}

double appBarHeight(WidgetTester tester) =>
    tester.getSize(find.byType(AppBar, skipOffstage: false)).height;

enum _BodyLength { short, long }

class TestHeader extends SliverPersistentHeaderDelegate {
  const TestHeader({this.key, required this.minExtent, required this.maxExtent});
  final Key? key;
  @override
  final double minExtent;
  @override
  final double maxExtent;
  @override
  Widget build(BuildContext context, double shrinkOffset, bool overlapsContent) {
    return Placeholder(key: key);
  }

  @override
  bool shouldRebuild(TestHeader oldDelegate) => false;
}

class _TestLayoutExtentIsNegative extends StatelessWidget {
  const _TestLayoutExtentIsNegative(this.widgetCountBeforeSliverOverlapAbsorber);
  final int widgetCountBeforeSliverOverlapAbsorber;
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('Test')),
        body: NestedScrollView(
          headerSliverBuilder: (BuildContext context, bool innerBoxIsScrolled) {
            return <Widget>[
              ...List<Widget>.generate(widgetCountBeforeSliverOverlapAbsorber, (_) {
                return SliverToBoxAdapter(
                  child: Container(
                    color: Colors.red,
                    height: 200,
                    margin: const EdgeInsets.all(20),
                  ),
                );
              }),
              SliverOverlapAbsorber(
                handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                sliver: SliverAppBar(
                  pinned: true,
                  forceElevated: innerBoxIsScrolled,
                  backgroundColor: Colors.blue[300],
                  title: const SizedBox(height: 50, child: Center(child: Text('Sticky Header'))),
                ),
              ),
            ];
          },
          body: Container(
            height: 2000,
            margin: const EdgeInsets.only(top: 50),
            child: ListView(
              children: List<Widget>.generate(3, (_) {
                return Container(
                  color: Colors.green[200],
                  height: 200,
                  margin: const EdgeInsets.all(20),
                );
              }),
            ),
          ),
        ),
      ),
    );
  }
}
