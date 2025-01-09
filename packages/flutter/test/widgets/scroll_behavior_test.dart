// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

late GestureVelocityTrackerBuilder lastCreatedBuilder;

class TestScrollBehavior extends ScrollBehavior {
  const TestScrollBehavior(this.flag);

  final bool flag;

  @override
  ScrollPhysics getScrollPhysics(BuildContext context) {
    return flag ? const ClampingScrollPhysics() : const BouncingScrollPhysics();
  }

  @override
  bool shouldNotify(TestScrollBehavior old) => flag != old.flag;

  @override
  GestureVelocityTrackerBuilder velocityTrackerBuilder(BuildContext context) {
    lastCreatedBuilder =
        flag
            ? (PointerEvent ev) => VelocityTracker.withKind(ev.kind)
            : (PointerEvent ev) => IOSScrollViewFlingVelocityTracker(ev.kind);
    return lastCreatedBuilder;
  }
}

void main() {
  testWidgets(
    'Assert in buildScrollbar that controller != null when using it',
    (WidgetTester tester) async {
      const ScrollBehavior defaultBehavior = ScrollBehavior();
      late BuildContext capturedContext;

      await tester.pumpWidget(
        ScrollConfiguration(
          // Avoid the default ones here.
          behavior: const ScrollBehavior().copyWith(scrollbars: false),
          child: SingleChildScrollView(
            child: Builder(
              builder: (BuildContext context) {
                capturedContext = context;
                return Container(height: 1000.0);
              },
            ),
          ),
        ),
      );

      const ScrollableDetails details = ScrollableDetails(direction: AxisDirection.down);
      final Widget child = Container();

      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.iOS:
          // Does not throw if we aren't using it.
          defaultBehavior.buildScrollbar(capturedContext, child, details);
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          expect(
            () {
              defaultBehavior.buildScrollbar(capturedContext, child, details);
            },
            throwsA(
              isA<AssertionError>().having(
                (AssertionError error) => error.toString(),
                'description',
                contains('details.controller != null'),
              ),
            ),
          );
      }
    },
    variant: TargetPlatformVariant.all(),
  );

  // Regression test for https://github.com/flutter/flutter/issues/89681
  testWidgets('_WrappedScrollBehavior shouldNotify test', (WidgetTester tester) async {
    final ScrollBehavior behavior1 = const ScrollBehavior().copyWith();
    final ScrollBehavior behavior2 = const ScrollBehavior().copyWith();

    expect(behavior1.shouldNotify(behavior2), false);
  });

  testWidgets('Inherited ScrollConfiguration changed', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey(debugLabel: 'scrollable');
    TestScrollBehavior? behavior;
    late ScrollPositionWithSingleContext position;

    final Widget scrollView = SingleChildScrollView(
      key: key,
      child: Builder(
        builder: (BuildContext context) {
          behavior = ScrollConfiguration.of(context) as TestScrollBehavior;
          position = Scrollable.of(context).position as ScrollPositionWithSingleContext;
          return Container(height: 1000.0);
        },
      ),
    );

    await tester.pumpWidget(
      ScrollConfiguration(behavior: const TestScrollBehavior(true), child: scrollView),
    );

    expect(behavior, isNotNull);
    expect(behavior!.flag, isTrue);
    expect(position.physics, isA<ClampingScrollPhysics>());
    expect(lastCreatedBuilder(const PointerDownEvent()), isA<VelocityTracker>());
    ScrollMetrics metrics = position.copyWith();
    expect(metrics.extentAfter, equals(400.0));
    expect(metrics.viewportDimension, equals(600.0));

    // Same Scrollable, different ScrollConfiguration
    await tester.pumpWidget(
      ScrollConfiguration(behavior: const TestScrollBehavior(false), child: scrollView),
    );

    expect(behavior, isNotNull);
    expect(behavior!.flag, isFalse);
    expect(position.physics, isA<BouncingScrollPhysics>());
    expect(lastCreatedBuilder(const PointerDownEvent()), isA<IOSScrollViewFlingVelocityTracker>());
    // Regression test for https://github.com/flutter/flutter/issues/5856
    metrics = position.copyWith();
    expect(metrics.extentAfter, equals(400.0));
    expect(metrics.viewportDimension, equals(600.0));
  });

  testWidgets(
    'ScrollBehavior default android overscroll indicator',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: ScrollConfiguration(
            behavior: const ScrollBehavior(),
            child: ListView(
              children: const <Widget>[
                SizedBox(height: 1000.0, width: 1000.0, child: Text('Test')),
              ],
            ),
          ),
        ),
      );

      expect(find.byType(StretchingOverscrollIndicator), findsNothing);
      expect(find.byType(GlowingOverscrollIndicator), findsOneWidget);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.android),
  );

  testWidgets('ScrollBehavior multitouchDragStrategy test - 1', (WidgetTester tester) async {
    const ScrollBehavior behavior1 = ScrollBehavior();
    final ScrollBehavior behavior2 = const ScrollBehavior().copyWith(
      multitouchDragStrategy: MultitouchDragStrategy.sumAllPointers,
    );
    final ScrollController controller = ScrollController();
    addTearDown(() => controller.dispose());

    Widget buildFrame(ScrollBehavior behavior) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: ScrollConfiguration(
          behavior: behavior,
          child: ListView(
            controller: controller,
            children: const <Widget>[
              SizedBox(height: 1000.0, width: 1000.0, child: Text('I Love Flutter!')),
            ],
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(behavior1));

    expect(controller.position.pixels, 0.0);

    final Offset listLocation = tester.getCenter(find.byType(ListView));

    final TestGesture gesture1 = await tester.createGesture(pointer: 1);
    await gesture1.down(listLocation);
    await tester.pump();

    final TestGesture gesture2 = await tester.createGesture(pointer: 2);
    await gesture2.down(listLocation);
    await tester.pump();

    await gesture1.moveBy(const Offset(0, -50));
    await tester.pump();

    await gesture2.moveBy(const Offset(0, -50));
    await tester.pump();

    // The default multitouchDragStrategy is 'latestPointer' or 'averageBoundaryPointers,
    // the received delta should be 50.0.
    expect(controller.position.pixels, 50.0);

    // Change to sumAllPointers.
    await tester.pumpWidget(buildFrame(behavior2));

    await gesture1.moveBy(const Offset(0, -50));
    await tester.pump();

    await gesture2.moveBy(const Offset(0, -50));
    await tester.pump();

    // All active pointers be tracked.
    expect(controller.position.pixels, 50.0 + 50.0 + 50.0);
  }, variant: TargetPlatformVariant.all());

  testWidgets(
    'ScrollBehavior multitouchDragStrategy test (non-Apple platforms) - 2',
    (WidgetTester tester) async {
      const ScrollBehavior behavior1 = ScrollBehavior();
      final ScrollBehavior behavior2 = const ScrollBehavior().copyWith(
        multitouchDragStrategy: MultitouchDragStrategy.averageBoundaryPointers,
      );
      final ScrollController controller = ScrollController();
      late BuildContext capturedContext;
      addTearDown(() => controller.dispose());

      Widget buildFrame(ScrollBehavior behavior) {
        return Directionality(
          textDirection: TextDirection.ltr,
          child: ScrollConfiguration(
            behavior: behavior,
            child: Builder(
              builder: (BuildContext context) {
                capturedContext = context;
                return ListView(
                  controller: controller,
                  children: const <Widget>[
                    SizedBox(height: 1000.0, width: 1000.0, child: Text('I Love Flutter!')),
                  ],
                );
              },
            ),
          ),
        );
      }

      await tester.pumpWidget(buildFrame(behavior1));

      expect(controller.position.pixels, 0.0);

      final Offset listLocation = tester.getCenter(find.byType(ListView));

      final TestGesture gesture1 = await tester.createGesture(pointer: 1);
      await gesture1.down(listLocation);
      await tester.pump();

      final TestGesture gesture2 = await tester.createGesture(pointer: 2);
      await gesture2.down(listLocation);
      await tester.pump();

      await gesture1.moveBy(const Offset(0, -50));
      await tester.pump();

      await gesture2.moveBy(const Offset(0, -40));
      await tester.pump();

      // The default multitouchDragStrategy is latestPointer.
      // Only the latest active pointer be tracked.
      final ScrollBehavior scrollBehavior = ScrollConfiguration.of(capturedContext);
      expect(
        scrollBehavior.getMultitouchDragStrategy(capturedContext),
        MultitouchDragStrategy.latestPointer,
      );
      expect(controller.position.pixels, 40.0);

      // Change to averageBoundaryPointers.
      await tester.pumpWidget(buildFrame(behavior2));

      await gesture1.moveBy(const Offset(0, -70));
      await tester.pump();

      await gesture2.moveBy(const Offset(0, -60));
      await tester.pump();

      expect(controller.position.pixels, 40.0 + 70.0);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.linux,
      TargetPlatform.fuchsia,
      TargetPlatform.windows,
    }),
  );

  testWidgets(
    'ScrollBehavior multitouchDragStrategy test (Apple platforms) - 3',
    (WidgetTester tester) async {
      const ScrollBehavior behavior1 = ScrollBehavior();
      final ScrollBehavior behavior2 = const ScrollBehavior().copyWith(
        multitouchDragStrategy: MultitouchDragStrategy.latestPointer,
      );
      final ScrollController controller = ScrollController();
      late BuildContext capturedContext;
      addTearDown(() => controller.dispose());

      Widget buildFrame(ScrollBehavior behavior) {
        return Directionality(
          textDirection: TextDirection.ltr,
          child: ScrollConfiguration(
            behavior: behavior,
            child: Builder(
              builder: (BuildContext context) {
                capturedContext = context;
                return ListView(
                  controller: controller,
                  children: const <Widget>[
                    SizedBox(height: 1000.0, width: 1000.0, child: Text('I Love Flutter!')),
                  ],
                );
              },
            ),
          ),
        );
      }

      await tester.pumpWidget(buildFrame(behavior1));

      expect(controller.position.pixels, 0.0);

      final Offset listLocation = tester.getCenter(find.byType(ListView));

      final TestGesture gesture1 = await tester.createGesture(pointer: 1);
      await gesture1.down(listLocation);
      await tester.pump();

      final TestGesture gesture2 = await tester.createGesture(pointer: 2);
      await gesture2.down(listLocation);
      await tester.pump();

      await gesture1.moveBy(const Offset(0, -40));
      await tester.pump();

      await gesture2.moveBy(const Offset(0, -50));
      await tester.pump();

      // The default multitouchDragStrategy is averageBoundaryPointers.
      final ScrollBehavior scrollBehavior = ScrollConfiguration.of(capturedContext);
      expect(
        scrollBehavior.getMultitouchDragStrategy(capturedContext),
        MultitouchDragStrategy.averageBoundaryPointers,
      );
      expect(controller.position.pixels, 50.0);

      // Change to latestPointer.
      await tester.pumpWidget(buildFrame(behavior2));

      await gesture1.moveBy(const Offset(0, -50));
      await tester.pump();

      await gesture2.moveBy(const Offset(0, -40));
      await tester.pump();

      expect(controller.position.pixels, 50.0 + 40.0);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  group('ScrollBehavior configuration is maintained over multiple copies', () {
    testWidgets('dragDevices', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/91673
      const ScrollBehavior defaultBehavior = ScrollBehavior();
      expect(defaultBehavior.dragDevices, <PointerDeviceKind>{
        PointerDeviceKind.touch,
        PointerDeviceKind.stylus,
        PointerDeviceKind.invertedStylus,
        PointerDeviceKind.trackpad,
        PointerDeviceKind.unknown,
      });

      // Use copyWith to modify drag devices
      final ScrollBehavior onceCopiedBehavior = defaultBehavior.copyWith(
        dragDevices: PointerDeviceKind.values.toSet(),
      );
      expect(onceCopiedBehavior.dragDevices, PointerDeviceKind.values.toSet());

      // Copy again. The previously modified drag devices should carry over.
      final ScrollBehavior twiceCopiedBehavior = onceCopiedBehavior.copyWith();
      expect(twiceCopiedBehavior.dragDevices, PointerDeviceKind.values.toSet());
    });

    testWidgets('physics', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/91673
      late ScrollPhysics defaultPhysics;
      late ScrollPhysics onceCopiedPhysics;
      late ScrollPhysics twiceCopiedPhysics;

      await tester.pumpWidget(
        ScrollConfiguration(
          // Default ScrollBehavior
          behavior: const ScrollBehavior(),
          child: Builder(
            builder: (BuildContext context) {
              final ScrollBehavior defaultBehavior = ScrollConfiguration.of(context);
              // Copy once to change physics
              defaultPhysics = defaultBehavior.getScrollPhysics(context);
              return ScrollConfiguration(
                behavior: defaultBehavior.copyWith(physics: const BouncingScrollPhysics()),
                child: Builder(
                  builder: (BuildContext context) {
                    final ScrollBehavior onceCopiedBehavior = ScrollConfiguration.of(context);
                    onceCopiedPhysics = onceCopiedBehavior.getScrollPhysics(context);
                    return ScrollConfiguration(
                      // Copy again, physics should follow
                      behavior: onceCopiedBehavior.copyWith(),
                      child: Builder(
                        builder: (BuildContext context) {
                          twiceCopiedPhysics = ScrollConfiguration.of(
                            context,
                          ).getScrollPhysics(context);
                          return SingleChildScrollView(child: Container(height: 1000));
                        },
                      ),
                    );
                  },
                ),
              );
            },
          ),
        ),
      );

      expect(defaultPhysics, const ClampingScrollPhysics(parent: RangeMaintainingScrollPhysics()));
      expect(onceCopiedPhysics, const BouncingScrollPhysics());
      expect(twiceCopiedPhysics, const BouncingScrollPhysics());
    });

    testWidgets('platform', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/91673
      late TargetPlatform defaultPlatform;
      late TargetPlatform onceCopiedPlatform;
      late TargetPlatform twiceCopiedPlatform;

      await tester.pumpWidget(
        ScrollConfiguration(
          // Default ScrollBehavior
          behavior: const ScrollBehavior(),
          child: Builder(
            builder: (BuildContext context) {
              final ScrollBehavior defaultBehavior = ScrollConfiguration.of(context);
              // Copy once to change physics
              defaultPlatform = defaultBehavior.getPlatform(context);
              return ScrollConfiguration(
                behavior: defaultBehavior.copyWith(platform: TargetPlatform.fuchsia),
                child: Builder(
                  builder: (BuildContext context) {
                    final ScrollBehavior onceCopiedBehavior = ScrollConfiguration.of(context);
                    onceCopiedPlatform = onceCopiedBehavior.getPlatform(context);
                    return ScrollConfiguration(
                      // Copy again, physics should follow
                      behavior: onceCopiedBehavior.copyWith(),
                      child: Builder(
                        builder: (BuildContext context) {
                          twiceCopiedPlatform = ScrollConfiguration.of(
                            context,
                          ).getPlatform(context);
                          return SingleChildScrollView(child: Container(height: 1000));
                        },
                      ),
                    );
                  },
                ),
              );
            },
          ),
        ),
      );

      expect(defaultPlatform, TargetPlatform.android);
      expect(onceCopiedPlatform, TargetPlatform.fuchsia);
      expect(twiceCopiedPlatform, TargetPlatform.fuchsia);
    });

    Widget wrap(ScrollBehavior behavior) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(size: Size(500, 500)),
          child: ScrollConfiguration(
            behavior: behavior,
            child: Builder(
              builder:
                  (BuildContext context) => SingleChildScrollView(child: Container(height: 1000)),
            ),
          ),
        ),
      );
    }

    testWidgets('scrollbar', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/91673
      const ScrollBehavior defaultBehavior = ScrollBehavior();
      await tester.pumpWidget(wrap(defaultBehavior));
      // Default adds a scrollbar
      expect(find.byType(RawScrollbar), findsOneWidget);
      final ScrollBehavior onceCopiedBehavior = defaultBehavior.copyWith(scrollbars: false);

      await tester.pumpWidget(wrap(onceCopiedBehavior));
      // Copy does not add scrollbar
      expect(find.byType(RawScrollbar), findsNothing);
      final ScrollBehavior twiceCopiedBehavior = onceCopiedBehavior.copyWith();

      await tester.pumpWidget(wrap(twiceCopiedBehavior));
      // Second copy maintains scrollbar setting
      expect(find.byType(RawScrollbar), findsNothing);

      // For default scrollbars
    }, variant: TargetPlatformVariant.desktop());

    testWidgets('overscroll', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/91673
      const ScrollBehavior defaultBehavior = ScrollBehavior();
      await tester.pumpWidget(wrap(defaultBehavior));
      // Default adds a glowing overscroll indicator
      expect(find.byType(GlowingOverscrollIndicator), findsOneWidget);
      final ScrollBehavior onceCopiedBehavior = defaultBehavior.copyWith(overscroll: false);

      await tester.pumpWidget(wrap(onceCopiedBehavior));
      // Copy does not add indicator
      expect(find.byType(GlowingOverscrollIndicator), findsNothing);
      final ScrollBehavior twiceCopiedBehavior = onceCopiedBehavior.copyWith();

      await tester.pumpWidget(wrap(twiceCopiedBehavior));
      // Second copy maintains overscroll setting
      expect(find.byType(GlowingOverscrollIndicator), findsNothing);

      // For default glowing indicator
    }, variant: TargetPlatformVariant.only(TargetPlatform.android));
  });
}
