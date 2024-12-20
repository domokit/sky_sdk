// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

void main() {
  Widget boilerplateWidget(
    VoidCallback? onButtonPressed, {
    DraggableScrollableController? controller,
    int itemCount = 100,
    double initialChildSize = .5,
    double maxChildSize = 1.0,
    double minChildSize = .25,
    bool snap = false,
    List<double>? snapSizes,
    Duration? snapAnimationDuration,
    double? itemExtent,
    Key? containerKey,
    Key? stackKey,
    NotificationListenerCallback<ScrollNotification>? onScrollNotification,
    NotificationListenerCallback<DraggableScrollableNotification>?
    onDraggableScrollableNotification,
    bool ignoreController = false,
    bool shouldCloseOnMinExtent = true,
  }) {
    return Directionality(
      textDirection: TextDirection.ltr,
      child: MediaQuery(
        data: const MediaQueryData(),
        child: Stack(
          key: stackKey,
          children: <Widget>[
            TextButton(onPressed: onButtonPressed, child: const Text('TapHere')),
            DraggableScrollableActuator(
              child: DraggableScrollableSheet(
                controller: controller,
                maxChildSize: maxChildSize,
                minChildSize: minChildSize,
                initialChildSize: initialChildSize,
                snap: snap,
                snapSizes: snapSizes,
                snapAnimationDuration: snapAnimationDuration,
                shouldCloseOnMinExtent: shouldCloseOnMinExtent,
                builder: (BuildContext context, ScrollController scrollController) {
                  return NotificationListener<ScrollNotification>(
                    onNotification: onScrollNotification,
                    child: NotificationListener<DraggableScrollableNotification>(
                      onNotification: onDraggableScrollableNotification,
                      child: ColoredBox(
                        key: containerKey,
                        color: const Color(0xFFABCDEF),
                        child: ListView.builder(
                          controller: ignoreController ? null : scrollController,
                          itemExtent: itemExtent,
                          itemCount: itemCount,
                          itemBuilder: (BuildContext context, int index) => Text('Item $index'),
                        ),
                      ),
                    ),
                  );
                },
              ),
            ),
          ],
        ),
      ),
    );
  }

  testWidgets('Do not crash when replacing scroll position during the drag', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/89681
    bool showScrollbars = false;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(),
          child: Align(
            alignment: Alignment.bottomCenter,
            child: DraggableScrollableSheet(
              initialChildSize: 0.7,
              minChildSize: 0.2,
              maxChildSize: 0.9,
              expand: false,
              builder: (BuildContext context, ScrollController scrollController) {
                showScrollbars = !showScrollbars;
                // Change the scroll behavior will trigger scroll position replace.
                final ScrollBehavior behavior = const ScrollBehavior().copyWith(
                  scrollbars: showScrollbars,
                );
                return ScrollConfiguration(
                  behavior: behavior,
                  child: ListView.separated(
                    physics: const BouncingScrollPhysics(),
                    controller: scrollController,
                    separatorBuilder: (_, __) => const Divider(),
                    itemCount: 100,
                    itemBuilder:
                        (_, int index) => SizedBox(
                          height: 100,
                          child: ColoredBox(
                            color: Colors.primaries[index % Colors.primaries.length],
                            child: Text('Item $index'),
                          ),
                        ),
                  ),
                );
              },
            ),
          ),
        ),
      ),
    );

    await tester.fling(find.text('Item 1'), const Offset(0, 200), 350);
    await tester.pumpAndSettle();

    // Go without throw.
  });

  testWidgets('Scrolls correct amount when maxChildSize < 1.0', (WidgetTester tester) async {
    const Key key = ValueKey<String>('container');
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        maxChildSize: .6,
        initialChildSize: .25,
        itemExtent: 25.0,
        containerKey: key,
      ),
    );

    expect(tester.getRect(find.byKey(key)), const Rect.fromLTRB(0.0, 450.0, 800.0, 600.0));
    await tester.drag(find.text('Item 5'), const Offset(0, -125));
    await tester.pumpAndSettle();
    expect(tester.getRect(find.byKey(key)), const Rect.fromLTRB(0.0, 325.0, 800.0, 600.0));
  });

  testWidgets('Scrolls correct amount when maxChildSize == 1.0', (WidgetTester tester) async {
    const Key key = ValueKey<String>('container');
    await tester.pumpWidget(
      boilerplateWidget(null, initialChildSize: .25, itemExtent: 25.0, containerKey: key),
    );

    expect(tester.getRect(find.byKey(key)), const Rect.fromLTRB(0.0, 450.0, 800.0, 600.0));
    await tester.drag(find.text('Item 5'), const Offset(0, -125));
    await tester.pumpAndSettle();
    expect(tester.getRect(find.byKey(key)), const Rect.fromLTRB(0.0, 325.0, 800.0, 600.0));
  });

  testWidgets(
    'Invalid snap targets throw assertion errors.',
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
    (WidgetTester tester) async {
      await tester.pumpWidget(boilerplateWidget(null, maxChildSize: .8, snapSizes: <double>[.9]));
      expect(tester.takeException(), isAssertionError);

      await tester.pumpWidget(boilerplateWidget(null, snapSizes: <double>[.1]));
      expect(tester.takeException(), isAssertionError);

      await tester.pumpWidget(boilerplateWidget(null, snapSizes: <double>[.6, .6, .9]));
      expect(tester.takeException(), isAssertionError);
    },
  );

  group('Scroll Physics', () {
    testWidgets('Can be dragged up without covering its container', (WidgetTester tester) async {
      int taps = 0;
      await tester.pumpWidget(boilerplateWidget(() => taps++));

      expect(find.text('TapHere'), findsOneWidget);
      await tester.tap(find.text('TapHere'));
      expect(taps, 1);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 21'), findsOneWidget);
      expect(find.text('Item 31'), findsNothing);

      await tester.drag(find.text('Item 1'), const Offset(0, -200));
      await tester.pumpAndSettle();
      expect(find.text('TapHere'), findsOneWidget);
      await tester.tap(find.text('TapHere'));
      expect(taps, 2);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 21'), findsOneWidget);
      expect(find.text('Item 31'), findsOneWidget);
    }, variant: TargetPlatformVariant.all());

    testWidgets('Can be dragged down when not full height', (WidgetTester tester) async {
      await tester.pumpWidget(boilerplateWidget(null));
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 21'), findsOneWidget);
      expect(find.text('Item 36'), findsNothing);

      await tester.drag(find.text('Item 1'), const Offset(0, 325));
      await tester.pumpAndSettle();
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 21'), findsNothing);
      expect(find.text('Item 36'), findsNothing);
    }, variant: TargetPlatformVariant.all());

    testWidgets(
      'Can be dragged down when list is shorter than full height',
      (WidgetTester tester) async {
        await tester.pumpWidget(boilerplateWidget(null, itemCount: 30, initialChildSize: .25));

        expect(find.text('Item 1').hitTestable(), findsOneWidget);
        expect(find.text('Item 29').hitTestable(), findsNothing);

        await tester.drag(find.text('Item 1'), const Offset(0, -325));
        await tester.pumpAndSettle();
        expect(find.text('Item 1').hitTestable(), findsOneWidget);
        expect(find.text('Item 29').hitTestable(), findsOneWidget);

        await tester.drag(find.text('Item 1'), const Offset(0, 325));
        await tester.pumpAndSettle();
        expect(find.text('Item 1').hitTestable(), findsOneWidget);
        expect(find.text('Item 29').hitTestable(), findsNothing);
      },
      variant: TargetPlatformVariant.all(),
    );

    testWidgets(
      'Can be dragged up and cover its container and scroll in single motion, and then dragged back down',
      (WidgetTester tester) async {
        int taps = 0;
        await tester.pumpWidget(boilerplateWidget(() => taps++));

        expect(find.text('TapHere'), findsOneWidget);
        await tester.tap(find.text('TapHere'));
        expect(taps, 1);
        expect(find.text('Item 1'), findsOneWidget);
        expect(find.text('Item 21'), findsOneWidget);
        expect(find.text('Item 36'), findsNothing);

        await tester.drag(find.text('Item 1'), const Offset(0, -325));
        await tester.pumpAndSettle();
        expect(find.text('TapHere'), findsOneWidget);
        await tester.tap(find.text('TapHere'), warnIfMissed: false);
        expect(taps, 1);
        expect(find.text('Item 1'), findsOneWidget);
        expect(find.text('Item 21'), findsOneWidget);
        expect(find.text('Item 36'), findsOneWidget);

        await tester.dragFrom(const Offset(20, 20), const Offset(0, 325));
        await tester.pumpAndSettle();
        await tester.tap(find.text('TapHere'));
        expect(taps, 2);
        expect(find.text('Item 1'), findsOneWidget);
        expect(find.text('Item 18'), findsOneWidget);
        expect(find.text('Item 36'), findsNothing);
      },
      variant: TargetPlatformVariant.all(),
    );

    testWidgets('Can be flung up gently', (WidgetTester tester) async {
      int taps = 0;
      await tester.pumpWidget(boilerplateWidget(() => taps++));

      expect(find.text('TapHere'), findsOneWidget);
      await tester.tap(find.text('TapHere'));
      expect(taps, 1);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 21'), findsOneWidget);
      expect(find.text('Item 36'), findsNothing);
      expect(find.text('Item 70'), findsNothing);

      await tester.fling(find.text('Item 1'), const Offset(0, -200), 350);
      await tester.pumpAndSettle();
      expect(find.text('TapHere'), findsOneWidget);
      await tester.tap(find.text('TapHere'));
      expect(taps, 2);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 21'), findsOneWidget);
      expect(find.text('Item 36'), findsOneWidget);
      expect(find.text('Item 70'), findsNothing);
    }, variant: TargetPlatformVariant.all());

    testWidgets('Can be flung up', (WidgetTester tester) async {
      int taps = 0;
      await tester.pumpWidget(boilerplateWidget(() => taps++));

      expect(find.text('TapHere'), findsOneWidget);
      await tester.tap(find.text('TapHere'));
      expect(taps, 1);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 21'), findsOneWidget);
      expect(find.text('Item 70'), findsNothing);

      await tester.fling(find.text('Item 1'), const Offset(0, -200), 2000);
      await tester.pumpAndSettle();
      expect(find.text('TapHere'), findsOneWidget);
      await tester.tap(find.text('TapHere'), warnIfMissed: false);
      expect(taps, 1);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 21'), findsNothing);
      if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS) {
        expect(find.text('Item 40'), findsOneWidget);
      } else {
        expect(find.text('Item 70'), findsOneWidget);
      }
    }, variant: TargetPlatformVariant.all());

    testWidgets('Can be flung down when not full height', (WidgetTester tester) async {
      await tester.pumpWidget(boilerplateWidget(null));
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 21'), findsOneWidget);
      expect(find.text('Item 36'), findsNothing);

      await tester.fling(find.text('Item 1'), const Offset(0, 325), 2000);
      await tester.pumpAndSettle();
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 21'), findsNothing);
      expect(find.text('Item 36'), findsNothing);
    }, variant: TargetPlatformVariant.all());

    testWidgets('Can be flung up and then back down', (WidgetTester tester) async {
      int taps = 0;
      await tester.pumpWidget(boilerplateWidget(() => taps++));

      expect(find.text('TapHere'), findsOneWidget);
      await tester.tap(find.text('TapHere'));
      expect(taps, 1);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 21'), findsOneWidget);
      expect(find.text('Item 70'), findsNothing);

      await tester.fling(find.text('Item 1'), const Offset(0, -200), 2000);
      await tester.pumpAndSettle();
      expect(find.text('TapHere'), findsOneWidget);
      await tester.tap(find.text('TapHere'), warnIfMissed: false);
      expect(taps, 1);
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 21'), findsNothing);
      if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS) {
        expect(find.text('Item 40'), findsOneWidget);
        await tester.fling(find.text('Item 40'), const Offset(0, 200), 2000);
      } else {
        expect(find.text('Item 70'), findsOneWidget);
        await tester.fling(find.text('Item 70'), const Offset(0, 200), 2000);
      }
      await tester.pumpAndSettle();
      expect(find.text('TapHere'), findsOneWidget);
      await tester.tap(find.text('TapHere'), warnIfMissed: false);
      expect(taps, 1);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 21'), findsOneWidget);
      expect(find.text('Item 70'), findsNothing);

      await tester.fling(find.text('Item 1'), const Offset(0, 200), 2000);
      await tester.pumpAndSettle();
      expect(find.text('TapHere'), findsOneWidget);
      await tester.tap(find.text('TapHere'));
      expect(taps, 2);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 21'), findsNothing);
      expect(find.text('Item 70'), findsNothing);
    }, variant: TargetPlatformVariant.all());

    testWidgets('Ballistic animation on fling can be interrupted', (WidgetTester tester) async {
      int taps = 0;
      await tester.pumpWidget(boilerplateWidget(() => taps++));

      expect(find.text('TapHere'), findsOneWidget);
      await tester.tap(find.text('TapHere'));
      expect(taps, 1);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 31'), findsNothing);
      expect(find.text('Item 70'), findsNothing);

      await tester.fling(find.text('Item 1'), const Offset(0, -200), 2000);
      // Don't pump and settle because we want to interrupt the ballistic scrolling animation.
      expect(find.text('TapHere'), findsOneWidget);
      await tester.tap(find.text('TapHere'), warnIfMissed: false);
      expect(taps, 2);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 31'), findsOneWidget);
      expect(find.text('Item 70'), findsNothing);

      // Use `dragFrom` here because calling `drag` on a list item without
      // first calling `pumpAndSettle` fails with a hit test error.
      await tester.dragFrom(const Offset(0, 200), const Offset(0, 200));
      await tester.pumpAndSettle();

      // Verify that the ballistic animation has canceled and the sheet has
      // returned to it's original position.
      await tester.tap(find.text('TapHere'));
      expect(taps, 3);
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 31'), findsNothing);
      expect(find.text('Item 70'), findsNothing);
    }, variant: TargetPlatformVariant.all());

    testWidgets('Ballistic animation on fling should not leak Ticker', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/101061
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: const MediaQueryData(),
            child: Align(
              alignment: Alignment.bottomCenter,
              child: DraggableScrollableSheet(
                initialChildSize: 0.8,
                minChildSize: 0.2,
                maxChildSize: 0.9,
                expand: false,
                builder: (_, ScrollController scrollController) {
                  return ListView.separated(
                    physics: const BouncingScrollPhysics(),
                    controller: scrollController,
                    separatorBuilder: (_, __) => const Divider(),
                    itemCount: 100,
                    itemBuilder:
                        (_, int index) => SizedBox(
                          height: 100,
                          child: ColoredBox(
                            color: Colors.primaries[index % Colors.primaries.length],
                            child: Text('Item $index'),
                          ),
                        ),
                  );
                },
              ),
            ),
          ),
        ),
      );

      await tester.flingFrom(tester.getCenter(find.text('Item 1')), const Offset(0, 50), 10000);

      // Pumps several times to let the DraggableScrollableSheet react to scroll position changes.
      const int numberOfPumpsBeforeError = 22;
      for (int i = 0; i < numberOfPumpsBeforeError; i++) {
        await tester.pump(const Duration(milliseconds: 10));
      }

      // Dispose the DraggableScrollableSheet
      await tester.pumpWidget(const SizedBox.shrink());

      // When a Ticker leaks an exception is thrown
      expect(tester.takeException(), isNull);
    });
  });

  testWidgets('Does not snap away from initial child on build', (WidgetTester tester) async {
    const Key containerKey = ValueKey<String>('container');
    const Key stackKey = ValueKey<String>('stack');
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        snap: true,
        initialChildSize: .7,
        containerKey: containerKey,
        stackKey: stackKey,
      ),
    );
    await tester.pumpAndSettle();
    final double screenHeight = tester.getSize(find.byKey(stackKey)).height;

    // The sheet should not have snapped.
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.7, precisionErrorTolerance),
    );
  }, variant: TargetPlatformVariant.all());

  for (final bool useActuator in <bool>[false, true]) {
    testWidgets(
      'Does not snap away from initial child on ${useActuator ? 'actuator' : 'controller'}.reset()',
      (WidgetTester tester) async {
        const Key containerKey = ValueKey<String>('container');
        const Key stackKey = ValueKey<String>('stack');
        final DraggableScrollableController controller = DraggableScrollableController();
        addTearDown(controller.dispose);
        await tester.pumpWidget(
          boilerplateWidget(
            null,
            controller: controller,
            snap: true,
            containerKey: containerKey,
            stackKey: stackKey,
          ),
        );
        await tester.pumpAndSettle();
        final double screenHeight = tester.getSize(find.byKey(stackKey)).height;

        await tester.drag(find.text('Item 1'), Offset(0, -.4 * screenHeight));
        await tester.pumpAndSettle();
        expect(
          tester.getSize(find.byKey(containerKey)).height / screenHeight,
          closeTo(1.0, precisionErrorTolerance),
        );

        if (useActuator) {
          DraggableScrollableActuator.reset(tester.element(find.byKey(containerKey)));
        } else {
          controller.reset();
        }
        await tester.pumpAndSettle();

        // The sheet should have reset without snapping away from initial child.
        expect(
          tester.getSize(find.byKey(containerKey)).height / screenHeight,
          closeTo(.5, precisionErrorTolerance),
        );
      },
    );
  }

  for (final Duration? snapAnimationDuration in <Duration?>[null, const Duration(seconds: 2)]) {
    testWidgets('Zero velocity drag snaps to nearest snap target with '
        'snapAnimationDuration: $snapAnimationDuration', (WidgetTester tester) async {
      const Key stackKey = ValueKey<String>('stack');
      const Key containerKey = ValueKey<String>('container');
      await tester.pumpWidget(
        boilerplateWidget(
          null,
          snap: true,
          stackKey: stackKey,
          containerKey: containerKey,
          snapSizes: <double>[.25, .5, .75, 1.0],
          snapAnimationDuration: snapAnimationDuration,
        ),
      );
      await tester.pumpAndSettle();
      final double screenHeight = tester.getSize(find.byKey(stackKey)).height;

      // We are dragging up, but we'll snap down because we're closer to .75 than 1.
      await tester.drag(find.text('Item 1'), Offset(0, -.35 * screenHeight));
      await tester.pumpAndSettle();
      expect(
        tester.getSize(find.byKey(containerKey)).height / screenHeight,
        closeTo(.75, precisionErrorTolerance),
      );

      // Drag up and snap up.
      await tester.drag(find.text('Item 1'), Offset(0, -.2 * screenHeight));
      await tester.pumpAndSettle();
      expect(
        tester.getSize(find.byKey(containerKey)).height / screenHeight,
        closeTo(1.0, precisionErrorTolerance),
      );

      // Drag down and snap up.
      await tester.drag(find.text('Item 1'), Offset(0, .1 * screenHeight));
      await tester.pumpAndSettle();
      expect(
        tester.getSize(find.byKey(containerKey)).height / screenHeight,
        closeTo(1.0, precisionErrorTolerance),
      );

      // Drag down and snap down.
      await tester.drag(find.text('Item 1'), Offset(0, .45 * screenHeight));
      await tester.pumpAndSettle();
      expect(
        tester.getSize(find.byKey(containerKey)).height / screenHeight,
        closeTo(.5, precisionErrorTolerance),
      );

      // Fling up with negligible velocity and snap down.
      await tester.fling(find.text('Item 1'), Offset(0, .1 * screenHeight), 1);
      await tester.pumpAndSettle();
      expect(
        tester.getSize(find.byKey(containerKey)).height / screenHeight,
        closeTo(.5, precisionErrorTolerance),
      );
    }, variant: TargetPlatformVariant.all());
  }

  for (final List<double>? snapSizes in <List<double>?>[null, <double>[]]) {
    testWidgets('Setting snapSizes to $snapSizes resolves to min and max', (
      WidgetTester tester,
    ) async {
      const Key stackKey = ValueKey<String>('stack');
      const Key containerKey = ValueKey<String>('container');
      await tester.pumpWidget(
        boilerplateWidget(
          null,
          snap: true,
          stackKey: stackKey,
          containerKey: containerKey,
          snapSizes: snapSizes,
        ),
      );
      await tester.pumpAndSettle();
      final double screenHeight = tester.getSize(find.byKey(stackKey)).height;

      await tester.drag(find.text('Item 1'), Offset(0, -.4 * screenHeight));
      await tester.pumpAndSettle();
      expect(
        tester.getSize(find.byKey(containerKey)).height / screenHeight,
        closeTo(1.0, precisionErrorTolerance),
      );

      await tester.drag(find.text('Item 1'), Offset(0, .7 * screenHeight));
      await tester.pumpAndSettle();
      expect(
        tester.getSize(find.byKey(containerKey)).height / screenHeight,
        closeTo(.25, precisionErrorTolerance),
      );
    }, variant: TargetPlatformVariant.all());
  }

  testWidgets('Min and max are implicitly added to snapSizes', (WidgetTester tester) async {
    const Key stackKey = ValueKey<String>('stack');
    const Key containerKey = ValueKey<String>('container');
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        snap: true,
        stackKey: stackKey,
        containerKey: containerKey,
        snapSizes: <double>[.5],
      ),
    );
    await tester.pumpAndSettle();
    final double screenHeight = tester.getSize(find.byKey(stackKey)).height;

    await tester.drag(find.text('Item 1'), Offset(0, -.4 * screenHeight));
    await tester.pumpAndSettle();
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(1.0, precisionErrorTolerance),
    );

    await tester.drag(find.text('Item 1'), Offset(0, .7 * screenHeight));
    await tester.pumpAndSettle();
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.25, precisionErrorTolerance),
    );
  }, variant: TargetPlatformVariant.all());

  testWidgets('Changes to widget parameters are propagated', (WidgetTester tester) async {
    const Key stackKey = ValueKey<String>('stack');
    const Key containerKey = ValueKey<String>('container');
    await tester.pumpWidget(
      boilerplateWidget(null, stackKey: stackKey, containerKey: containerKey),
    );
    await tester.pumpAndSettle();
    final double screenHeight = tester.getSize(find.byKey(stackKey)).height;
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.5, precisionErrorTolerance),
    );

    // Pump the same widget but with a new initial child size.
    await tester.pumpWidget(
      boilerplateWidget(null, stackKey: stackKey, containerKey: containerKey, initialChildSize: .6),
    );
    await tester.pumpAndSettle();

    // We jump to the new initial size because the sheet hasn't changed yet.
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.6, precisionErrorTolerance),
    );

    // Pump the same widget but with a new max child size.
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        stackKey: stackKey,
        containerKey: containerKey,
        initialChildSize: .6,
        maxChildSize: .9,
      ),
    );
    await tester.pumpAndSettle();

    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.6, precisionErrorTolerance),
    );

    await tester.drag(find.text('Item 1'), Offset(0, -.6 * screenHeight));
    await tester.pumpAndSettle();
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.9, precisionErrorTolerance),
    );

    // Pump the same widget but with a new max child size and initial size.
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        stackKey: stackKey,
        containerKey: containerKey,
        maxChildSize: .8,
        initialChildSize: .7,
      ),
    );
    await tester.pumpAndSettle();

    // The max child size has been reduced, we should be rebuilt at the new
    // max of .8. We changed the initial size again, but the sheet has already
    // been changed so the new initial is ignored.
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.8, precisionErrorTolerance),
    );

    await tester.drag(find.text('Item 1'), Offset(0, .2 * screenHeight));

    // Pump the same widget but with snapping enabled.
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        snap: true,
        stackKey: stackKey,
        containerKey: containerKey,
        maxChildSize: .8,
        snapSizes: <double>[.5],
      ),
    );
    await tester.pumpAndSettle();

    // Sheet snaps immediately on a change to snap.
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.5, precisionErrorTolerance),
    );

    final List<double> snapSizes = <double>[.6];

    // Change the snap sizes.
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        snap: true,
        stackKey: stackKey,
        containerKey: containerKey,
        maxChildSize: .8,
        snapSizes: snapSizes,
      ),
    );
    await tester.pumpAndSettle();

    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.6, precisionErrorTolerance),
    );
  }, variant: TargetPlatformVariant.all());

  testWidgets('Fling snaps in direction of momentum', (WidgetTester tester) async {
    const Key stackKey = ValueKey<String>('stack');
    const Key containerKey = ValueKey<String>('container');
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        snap: true,
        stackKey: stackKey,
        containerKey: containerKey,
        snapSizes: <double>[.5, .75],
      ),
    );
    await tester.pumpAndSettle();
    final double screenHeight = tester.getSize(find.byKey(stackKey)).height;

    await tester.fling(find.text('Item 1'), Offset(0, -.1 * screenHeight), 1000);
    await tester.pumpAndSettle();
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.75, precisionErrorTolerance),
    );

    await tester.fling(find.text('Item 1'), Offset(0, .3 * screenHeight), 1000);
    await tester.pumpAndSettle();
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.25, precisionErrorTolerance),
    );
  }, variant: TargetPlatformVariant.all());

  testWidgets(
    "Changing parameters with an un-listened controller doesn't throw",
    (WidgetTester tester) async {
      await tester.pumpWidget(
        boilerplateWidget(
          null,
          snap: true,
          // Will prevent the sheet's child from listening to the controller.
          ignoreController: true,
        ),
      );
      await tester.pumpAndSettle();
      await tester.pumpWidget(boilerplateWidget(null, snap: true));
      await tester.pumpAndSettle();
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'Transitioning between scrollable children sharing a scroll controller will not throw',
    (WidgetTester tester) async {
      int s = 0;
      await tester.pumpWidget(
        MaterialApp(
          home: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Scaffold(
                body: DraggableScrollableSheet(
                  initialChildSize: 0.25,
                  snap: true,
                  snapSizes: const <double>[0.25, 0.5, 1.0],
                  builder: (BuildContext context, ScrollController scrollController) {
                    return PrimaryScrollController(
                      controller: scrollController,
                      child: AnimatedSwitcher(
                        duration: const Duration(milliseconds: 500),
                        child:
                            s.isEven
                                ? ListView(
                                  children: <Widget>[
                                    ElevatedButton(
                                      onPressed: () => setState(() => ++s),
                                      child: const Text('Switch to 2'),
                                    ),
                                    Container(height: 400, color: Colors.blue),
                                  ],
                                )
                                : SingleChildScrollView(
                                  child: Column(
                                    children: <Widget>[
                                      ElevatedButton(
                                        onPressed: () => setState(() => ++s),
                                        child: const Text('Switch to 1'),
                                      ),
                                      Container(height: 400, color: Colors.blue),
                                    ],
                                  ),
                                ),
                      ),
                    );
                  },
                ),
              );
            },
          ),
        ),
      );

      // Trigger the AnimatedSwitcher between ListViews
      await tester.tap(find.text('Switch to 2'));
      await tester.pump();
      // Completes without throwing
    },
  );

  testWidgets('ScrollNotification correctly dispatched when flung without covering its container', (
    WidgetTester tester,
  ) async {
    final List<Type> notificationTypes = <Type>[];
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        onScrollNotification: (ScrollNotification notification) {
          notificationTypes.add(notification.runtimeType);
          return false;
        },
      ),
    );

    await tester.fling(find.text('Item 1'), const Offset(0, -200), 200);
    await tester.pumpAndSettle();

    // TODO(itome): Make sure UserScrollNotification and ScrollUpdateNotification are called correctly.
    final List<Type> types = <Type>[ScrollStartNotification, ScrollEndNotification];
    expect(notificationTypes, equals(types));
  });

  testWidgets('ScrollNotification correctly dispatched when flung with contents scroll', (
    WidgetTester tester,
  ) async {
    final List<Type> notificationTypes = <Type>[];
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        onScrollNotification: (ScrollNotification notification) {
          notificationTypes.add(notification.runtimeType);
          return false;
        },
      ),
    );

    await tester.flingFrom(const Offset(0, 325), const Offset(0, -325), 200);
    await tester.pumpAndSettle();

    final List<Type> types = <Type>[
      ScrollStartNotification,
      UserScrollNotification,
      ...List<Type>.filled(5, ScrollUpdateNotification),
      ScrollEndNotification,
      UserScrollNotification,
    ];
    expect(notificationTypes, types);
  });

  testWidgets(
    'Emits DraggableScrollableNotification with shouldCloseOnMinExtent set to non-default value',
    (WidgetTester tester) async {
      DraggableScrollableNotification? receivedNotification;
      await tester.pumpWidget(
        boilerplateWidget(
          null,
          shouldCloseOnMinExtent: false,
          onDraggableScrollableNotification: (DraggableScrollableNotification notification) {
            receivedNotification = notification;
            return false;
          },
        ),
      );

      await tester.flingFrom(const Offset(0, 325), const Offset(0, -325), 200);
      await tester.pumpAndSettle();
      expect(receivedNotification!.shouldCloseOnMinExtent, isFalse);
    },
  );

  testWidgets('Do not crash when remove the tree during animation.', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/89214
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        onScrollNotification: (ScrollNotification notification) {
          return false;
        },
      ),
    );

    await tester.flingFrom(const Offset(0, 325), const Offset(0, 325), 200);

    // The animation is running.

    await tester.pumpWidget(const SizedBox.shrink());

    expect(tester.takeException(), isNull);
  });

  for (final bool shouldAnimate in <bool>[true, false]) {
    testWidgets('Can ${shouldAnimate ? 'animate' : 'jump'} to arbitrary positions', (
      WidgetTester tester,
    ) async {
      const Key stackKey = ValueKey<String>('stack');
      const Key containerKey = ValueKey<String>('container');
      final DraggableScrollableController controller = DraggableScrollableController();
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        boilerplateWidget(
          null,
          controller: controller,
          stackKey: stackKey,
          containerKey: containerKey,
        ),
      );
      await tester.pumpAndSettle();
      final double screenHeight = tester.getSize(find.byKey(stackKey)).height;
      // Use a local helper to animate so we can share code across a jumpTo test
      // and an animateTo test.
      void goTo(double size) =>
          shouldAnimate
              ? controller.animateTo(
                size,
                duration: const Duration(milliseconds: 200),
                curve: Curves.linear,
              )
              : controller.jumpTo(size);
      // If we're animating, pump will call four times, two of which are for the
      // animation duration.
      final int expectedPumpCount = shouldAnimate ? 4 : 2;

      goTo(.6);
      expect(await tester.pumpAndSettle(), expectedPumpCount);
      expect(
        tester.getSize(find.byKey(containerKey)).height / screenHeight,
        closeTo(.6, precisionErrorTolerance),
      );
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 20'), findsOneWidget);
      expect(find.text('Item 70'), findsNothing);

      goTo(.4);
      expect(await tester.pumpAndSettle(), expectedPumpCount);
      expect(
        tester.getSize(find.byKey(containerKey)).height / screenHeight,
        closeTo(.4, precisionErrorTolerance),
      );
      expect(find.text('Item 1'), findsOneWidget);
      expect(find.text('Item 20'), findsNothing);
      expect(find.text('Item 70'), findsNothing);

      await tester.fling(find.text('Item 1'), Offset(0, -screenHeight), 100);
      await tester.pumpAndSettle();
      expect(
        tester.getSize(find.byKey(containerKey)).height / screenHeight,
        closeTo(1, precisionErrorTolerance),
      );
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 20'), findsOneWidget);
      expect(find.text('Item 70'), findsNothing);

      // Programmatic control does not affect the inner scrollable's position.
      goTo(.8);
      expect(await tester.pumpAndSettle(), expectedPumpCount);
      expect(
        tester.getSize(find.byKey(containerKey)).height / screenHeight,
        closeTo(.8, precisionErrorTolerance),
      );
      expect(find.text('Item 1'), findsNothing);
      expect(find.text('Item 20'), findsOneWidget);
      expect(find.text('Item 70'), findsNothing);

      // Attempting to move to a size too big or too small instead moves to the
      // min or max child size.
      goTo(.5);
      await tester.pumpAndSettle();
      goTo(0);
      expect(await tester.pumpAndSettle(), expectedPumpCount);
      expect(
        tester.getSize(find.byKey(containerKey)).height / screenHeight,
        closeTo(.25, precisionErrorTolerance),
      );
    });
  }

  testWidgets('Can animateTo with a nonlinear curve', (WidgetTester tester) async {
    const Key stackKey = ValueKey<String>('stack');
    const Key containerKey = ValueKey<String>('container');
    final DraggableScrollableController controller = DraggableScrollableController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        controller: controller,
        stackKey: stackKey,
        containerKey: containerKey,
      ),
    );
    await tester.pumpAndSettle();
    final double screenHeight = tester.getSize(find.byKey(stackKey)).height;

    controller.animateTo(.6, curve: Curves.linear, duration: const Duration(milliseconds: 100));
    // We need to call one pump first to get the animation to start.
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.55, precisionErrorTolerance),
    );
    await tester.pumpAndSettle();
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.6, precisionErrorTolerance),
    );

    controller.animateTo(
      .7,
      curve: const Interval(.5, 1),
      duration: const Duration(milliseconds: 100),
    );
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));
    // The curve should result in the sheet not moving for the first 50 ms.
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.6, precisionErrorTolerance),
    );
    await tester.pump(const Duration(milliseconds: 25));
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.65, precisionErrorTolerance),
    );
    await tester.pumpAndSettle();
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.7, precisionErrorTolerance),
    );
  });

  testWidgets('Can animateTo with a Curves.easeInOutBack curve begin min-size', (
    WidgetTester tester,
  ) async {
    const Key stackKey = ValueKey<String>('stack');
    const Key containerKey = ValueKey<String>('container');
    final DraggableScrollableController controller = DraggableScrollableController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        initialChildSize: 0.25,
        controller: controller,
        stackKey: stackKey,
        containerKey: containerKey,
      ),
    );
    await tester.pumpAndSettle();
    final double screenHeight = tester.getSize(find.byKey(stackKey)).height;

    controller.animateTo(
      .6,
      curve: Curves.easeInOutBack,
      duration: const Duration(milliseconds: 500),
    );

    await tester.pumpAndSettle();
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.6, precisionErrorTolerance),
    );
  });

  testWidgets('Can reuse a controller after the old controller is disposed', (
    WidgetTester tester,
  ) async {
    const Key stackKey = ValueKey<String>('stack');
    const Key containerKey = ValueKey<String>('container');
    final DraggableScrollableController controller = DraggableScrollableController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        controller: controller,
        stackKey: stackKey,
        containerKey: containerKey,
      ),
    );
    await tester.pumpAndSettle();

    // Pump a new sheet with the same controller. This will dispose of the old sheet first.
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        controller: controller,
        stackKey: stackKey,
        containerKey: containerKey,
      ),
    );
    await tester.pumpAndSettle();
    final double screenHeight = tester.getSize(find.byKey(stackKey)).height;

    controller.jumpTo(.6);
    await tester.pumpAndSettle();
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.6, precisionErrorTolerance),
    );
  });

  testWidgets('animateTo interrupts other animations', (WidgetTester tester) async {
    const Key stackKey = ValueKey<String>('stack');
    const Key containerKey = ValueKey<String>('container');
    final DraggableScrollableController controller = DraggableScrollableController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: boilerplateWidget(
          null,
          controller: controller,
          stackKey: stackKey,
          containerKey: containerKey,
        ),
      ),
    );
    await tester.pumpAndSettle();
    final double screenHeight = tester.getSize(find.byKey(stackKey)).height;

    await tester.flingFrom(Offset(0, .5 * screenHeight), Offset(0, -.5 * screenHeight), 2000);
    // Wait until `flinFrom` finished dragging, but before the scrollable goes ballistic.
    await tester.pump(const Duration(seconds: 1));

    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(1, precisionErrorTolerance),
    );
    expect(find.text('Item 1'), findsOneWidget);

    controller.animateTo(.9, duration: const Duration(milliseconds: 200), curve: Curves.linear);
    await tester.pumpAndSettle();
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.9, precisionErrorTolerance),
    );
    // The ballistic animation should have been canceled so item 1 should still be visible.
    expect(find.text('Item 1'), findsOneWidget);
  });

  testWidgets('Other animations interrupt animateTo', (WidgetTester tester) async {
    const Key stackKey = ValueKey<String>('stack');
    const Key containerKey = ValueKey<String>('container');
    final DraggableScrollableController controller = DraggableScrollableController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: boilerplateWidget(
          null,
          controller: controller,
          stackKey: stackKey,
          containerKey: containerKey,
        ),
      ),
    );
    await tester.pumpAndSettle();
    final double screenHeight = tester.getSize(find.byKey(stackKey)).height;

    controller.animateTo(1, duration: const Duration(milliseconds: 200), curve: Curves.linear);
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.75, precisionErrorTolerance),
    );

    // Interrupt animation and drag downward.
    await tester.drag(find.text('Item 1'), Offset(0, .1 * screenHeight));
    await tester.pumpAndSettle();
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.65, precisionErrorTolerance),
    );
  });

  testWidgets('animateTo can be interrupted by other animateTo or jumpTo', (
    WidgetTester tester,
  ) async {
    const Key stackKey = ValueKey<String>('stack');
    const Key containerKey = ValueKey<String>('container');
    final DraggableScrollableController controller = DraggableScrollableController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: boilerplateWidget(
          null,
          controller: controller,
          stackKey: stackKey,
          containerKey: containerKey,
        ),
      ),
    );
    await tester.pumpAndSettle();
    final double screenHeight = tester.getSize(find.byKey(stackKey)).height;

    controller.animateTo(1, duration: const Duration(milliseconds: 200), curve: Curves.linear);
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.75, precisionErrorTolerance),
    );

    // Interrupt animation with a new `animateTo`.
    controller.animateTo(.25, duration: const Duration(milliseconds: 200), curve: Curves.linear);
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.5, precisionErrorTolerance),
    );

    // Interrupt animation with a jump.
    controller.jumpTo(.6);
    await tester.pumpAndSettle();
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.6, precisionErrorTolerance),
    );
  });

  testWidgets('Can get size and pixels', (WidgetTester tester) async {
    const Key stackKey = ValueKey<String>('stack');
    const Key containerKey = ValueKey<String>('container');
    final DraggableScrollableController controller = DraggableScrollableController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        controller: controller,
        stackKey: stackKey,
        containerKey: containerKey,
      ),
    );
    await tester.pumpAndSettle();
    final double screenHeight = tester.getSize(find.byKey(stackKey)).height;

    expect(controller.sizeToPixels(.25), .25 * screenHeight);
    expect(controller.pixelsToSize(.25 * screenHeight), .25);

    controller.animateTo(.6, duration: const Duration(milliseconds: 200), curve: Curves.linear);
    await tester.pumpAndSettle();
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.6, precisionErrorTolerance),
    );
    expect(controller.size, closeTo(.6, precisionErrorTolerance));
    expect(controller.pixels, closeTo(.6 * screenHeight, precisionErrorTolerance));

    await tester.drag(find.text('Item 5'), Offset(0, .2 * screenHeight));
    expect(controller.size, closeTo(.4, precisionErrorTolerance));
    expect(controller.pixels, closeTo(.4 * screenHeight, precisionErrorTolerance));
  });

  testWidgets(
    'Cannot attach a controller to multiple sheets',
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
    (WidgetTester tester) async {
      final DraggableScrollableController controller = DraggableScrollableController();
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Stack(
            children: <Widget>[
              boilerplateWidget(null, controller: controller),
              boilerplateWidget(null, controller: controller),
            ],
          ),
        ),
        phase: EnginePhase.build,
      );
      expect(tester.takeException(), isAssertionError);
    },
  );

  testWidgets('Can listen for changes in sheet size', (WidgetTester tester) async {
    const Key stackKey = ValueKey<String>('stack');
    const Key containerKey = ValueKey<String>('container');
    final List<double> loggedSizes = <double>[];
    final DraggableScrollableController controller = DraggableScrollableController();
    addTearDown(controller.dispose);
    controller.addListener(() {
      loggedSizes.add(controller.size);
    });
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        controller: controller,
        stackKey: stackKey,
        containerKey: containerKey,
      ),
    );
    await tester.pumpAndSettle();
    final double screenHeight = tester.getSize(find.byKey(stackKey)).height;

    // The initial size shouldn't be logged because no change has occurred yet.
    expect(loggedSizes.isEmpty, true);

    await tester.drag(find.text('Item 1'), Offset(0, .1 * screenHeight), touchSlopY: 0);
    await tester.pumpAndSettle();
    expect(loggedSizes, <double>[.4].map((double v) => closeTo(v, precisionErrorTolerance)));
    loggedSizes.clear();

    await tester.timedDrag(
      find.text('Item 1'),
      Offset(0, -.1 * screenHeight),
      const Duration(seconds: 1),
      frequency: 2,
    );
    await tester.pumpAndSettle();
    expect(loggedSizes, <double>[.45, .5].map((double v) => closeTo(v, precisionErrorTolerance)));
    loggedSizes.clear();

    controller.jumpTo(.6);
    await tester.pumpAndSettle();
    expect(loggedSizes, <double>[.6].map((double v) => closeTo(v, precisionErrorTolerance)));
    loggedSizes.clear();

    controller.animateTo(1, duration: const Duration(milliseconds: 400), curve: Curves.linear);
    await tester.pumpAndSettle();
    expect(
      loggedSizes,
      <double>[.7, .8, .9, 1].map((double v) => closeTo(v, precisionErrorTolerance)),
    );
    loggedSizes.clear();

    DraggableScrollableActuator.reset(tester.element(find.byKey(containerKey)));
    await tester.pumpAndSettle();
    expect(loggedSizes, <double>[.5].map((double v) => closeTo(v, precisionErrorTolerance)));
    loggedSizes.clear();
  });

  testWidgets('Listener does not fire on parameter change and persists after change', (
    WidgetTester tester,
  ) async {
    const Key stackKey = ValueKey<String>('stack');
    const Key containerKey = ValueKey<String>('container');
    final List<double> loggedSizes = <double>[];
    final DraggableScrollableController controller = DraggableScrollableController();
    addTearDown(controller.dispose);
    controller.addListener(() {
      loggedSizes.add(controller.size);
    });
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        controller: controller,
        stackKey: stackKey,
        containerKey: containerKey,
      ),
    );
    await tester.pumpAndSettle();
    final double screenHeight = tester.getSize(find.byKey(stackKey)).height;

    expect(loggedSizes.isEmpty, true);

    await tester.drag(find.text('Item 1'), Offset(0, .1 * screenHeight), touchSlopY: 0);
    await tester.pumpAndSettle();
    expect(loggedSizes, <double>[.4].map((double v) => closeTo(v, precisionErrorTolerance)));
    loggedSizes.clear();

    // Update a parameter without forcing a change in the current size.
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        minChildSize: .1,
        controller: controller,
        stackKey: stackKey,
        containerKey: containerKey,
      ),
    );
    expect(loggedSizes.isEmpty, true);

    await tester.drag(find.text('Item 1'), Offset(0, .1 * screenHeight), touchSlopY: 0);
    await tester.pumpAndSettle();
    expect(loggedSizes, <double>[.3].map((double v) => closeTo(v, precisionErrorTolerance)));
    loggedSizes.clear();
  });

  testWidgets('Listener fires if a parameter change forces a change in size', (
    WidgetTester tester,
  ) async {
    const Key stackKey = ValueKey<String>('stack');
    const Key containerKey = ValueKey<String>('container');
    final List<double> loggedSizes = <double>[];
    final DraggableScrollableController controller = DraggableScrollableController();
    addTearDown(controller.dispose);
    controller.addListener(() {
      loggedSizes.add(controller.size);
    });
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        controller: controller,
        stackKey: stackKey,
        containerKey: containerKey,
      ),
    );
    await tester.pumpAndSettle();
    final double screenHeight = tester.getSize(find.byKey(stackKey)).height;

    expect(loggedSizes.isEmpty, true);

    // Set a new `initialChildSize` which will trigger a size change because we
    // haven't moved away initial size yet.
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        initialChildSize: .6,
        controller: controller,
        stackKey: stackKey,
        containerKey: containerKey,
      ),
    );
    expect(loggedSizes, <double>[.6].map((double v) => closeTo(v, precisionErrorTolerance)));
    loggedSizes.clear();

    // Move away from initial child size.
    await tester.drag(find.text('Item 1'), Offset(0, .3 * screenHeight), touchSlopY: 0);
    await tester.pumpAndSettle();
    expect(loggedSizes, <double>[.3].map((double v) => closeTo(v, precisionErrorTolerance)));
    loggedSizes.clear();

    // Set a `minChildSize` greater than the current size.
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        minChildSize: .4,
        controller: controller,
        stackKey: stackKey,
        containerKey: containerKey,
      ),
    );
    expect(loggedSizes, <double>[.4].map((double v) => closeTo(v, precisionErrorTolerance)));
    loggedSizes.clear();
  });

  testWidgets('Invalid controller interactions throw assertion errors', (
    WidgetTester tester,
  ) async {
    final DraggableScrollableController controller = DraggableScrollableController();
    addTearDown(controller.dispose);
    // Can't use a controller before attaching it.
    expect(() => controller.jumpTo(.1), throwsAssertionError);

    expect(() => controller.pixels, throwsAssertionError);
    expect(() => controller.size, throwsAssertionError);
    expect(() => controller.pixelsToSize(0), throwsAssertionError);
    expect(() => controller.sizeToPixels(0), throwsAssertionError);

    await tester.pumpWidget(boilerplateWidget(null, controller: controller));

    // Can't jump or animate to invalid sizes.
    expect(() => controller.jumpTo(-1), throwsAssertionError);
    expect(() => controller.jumpTo(1.1), throwsAssertionError);
    expect(
      () =>
          controller.animateTo(-1, duration: const Duration(milliseconds: 1), curve: Curves.linear),
      throwsAssertionError,
    );
    expect(
      () => controller.animateTo(
        1.1,
        duration: const Duration(milliseconds: 1),
        curve: Curves.linear,
      ),
      throwsAssertionError,
    );

    // Can't use animateTo with a zero duration.
    expect(
      () => controller.animateTo(.5, duration: Duration.zero, curve: Curves.linear),
      throwsAssertionError,
    );
  });

  testWidgets('DraggableScrollableController must be attached before using any of its parameters', (
    WidgetTester tester,
  ) async {
    final DraggableScrollableController controller = DraggableScrollableController();
    addTearDown(controller.dispose);
    expect(controller.isAttached, false);
    expect(() => controller.size, throwsAssertionError);
    final Widget boilerplate = boilerplateWidget(null, minChildSize: 0.4, controller: controller);
    await tester.pumpWidget(boilerplate);
    expect(controller.isAttached, true);
    expect(controller.size, isNotNull);
  });

  testWidgets('DraggableScrollableController.animateTo after detach', (WidgetTester tester) async {
    final DraggableScrollableController controller = DraggableScrollableController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(boilerplateWidget(() {}, controller: controller));

    controller.animateTo(0.0, curve: Curves.linear, duration: const Duration(milliseconds: 200));
    await tester.pump();

    // Dispose the DraggableScrollableSheet
    await tester.pumpWidget(const SizedBox.shrink());
    // Controller should be detached and no exception should be thrown
    expect(controller.isAttached, false);
    expect(tester.takeException(), isNull);
  });

  testWidgets('DraggableScrollableSheet should not reset programmatic drag on rebuild', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/101114
    const Key stackKey = ValueKey<String>('stack');
    const Key containerKey = ValueKey<String>('container');
    final DraggableScrollableController controller = DraggableScrollableController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        controller: controller,
        stackKey: stackKey,
        containerKey: containerKey,
      ),
    );
    await tester.pumpAndSettle();
    final double screenHeight = tester.getSize(find.byKey(stackKey)).height;

    controller.jumpTo(.6);
    await tester.pumpAndSettle();
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.6, precisionErrorTolerance),
    );

    // Force an arbitrary rebuild by pushing a new widget.
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        controller: controller,
        stackKey: stackKey,
        containerKey: containerKey,
      ),
    );
    // Sheet remains at .6.
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.6, precisionErrorTolerance),
    );

    controller.reset();
    await tester.pumpAndSettle();
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.5, precisionErrorTolerance),
    );

    controller.animateTo(.6, curve: Curves.linear, duration: const Duration(milliseconds: 200));
    await tester.pumpAndSettle();
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.6, precisionErrorTolerance),
    );

    // Force an arbitrary rebuild by pushing a new widget.
    await tester.pumpWidget(
      boilerplateWidget(
        null,
        controller: controller,
        stackKey: stackKey,
        containerKey: containerKey,
      ),
    );
    // Sheet remains at .6.
    expect(
      tester.getSize(find.byKey(containerKey)).height / screenHeight,
      closeTo(.6, precisionErrorTolerance),
    );
  });

  testWidgets('DraggableScrollableSheet should respect NeverScrollableScrollPhysics', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/121021
    final DraggableScrollableController controller = DraggableScrollableController();
    addTearDown(controller.dispose);
    Widget buildFrame(ScrollPhysics? physics) {
      return MaterialApp(
        home: Scaffold(
          body: DraggableScrollableSheet(
            controller: controller,
            initialChildSize: 0.25,
            builder: (BuildContext context, ScrollController scrollController) {
              return ListView(
                physics: physics,
                controller: scrollController,
                children: <Widget>[
                  const Text('Drag me!'),
                  Container(height: 10000.0, color: Colors.blue),
                ],
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(const NeverScrollableScrollPhysics()));

    final double initPixels = controller.pixels;

    await tester.drag(find.text('Drag me!'), const Offset(0, -300));
    await tester.pumpAndSettle();

    //Should not allow user scrolling.
    expect(controller.pixels, initPixels);

    await tester.pumpWidget(buildFrame(null));

    await tester.drag(find.text('Drag me!'), const Offset(0, -300.0));
    await tester.pumpAndSettle();

    //Allow user scrolling.
    expect(controller.pixels, initPixels + 300.0);
  });

  testWidgets('DraggableScrollableSheet should not rebuild every frame while dragging', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/67219
    int buildCount = 0;
    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder:
              (BuildContext context, StateSetter setState) => Scaffold(
                body: DraggableScrollableSheet(
                  initialChildSize: 0.25,
                  snap: true,
                  snapSizes: const <double>[0.25, 0.5, 1.0],
                  builder: (BuildContext context, ScrollController scrollController) {
                    buildCount++;
                    return ListView(
                      controller: scrollController,
                      children: <Widget>[
                        const Text('Drag me!'),
                        ElevatedButton(
                          onPressed: () => setState(() {}),
                          child: const Text('Rebuild'),
                        ),
                        Container(height: 10000, color: Colors.blue),
                      ],
                    );
                  },
                ),
              ),
        ),
      ),
    );

    expect(buildCount, 1);

    await tester.fling(find.text('Drag me!'), const Offset(0, -300), 300);
    await tester.pumpAndSettle();

    // No need to rebuild the scrollable sheet, as only position has changed.
    expect(buildCount, 1);

    await tester.tap(find.text('Rebuild'));
    await tester.pump();

    // DraggableScrollableSheet has rebuilt, so expect the builder to be called.
    expect(buildCount, 2);
  });

  testWidgets('DraggableScrollableSheet controller can be changed', (WidgetTester tester) async {
    final DraggableScrollableController controller1 = DraggableScrollableController();
    addTearDown(controller1.dispose);
    final DraggableScrollableController controller2 = DraggableScrollableController();
    addTearDown(controller2.dispose);
    final List<double> loggedSizes = <double>[];

    DraggableScrollableController controller = controller1;
    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder:
              (BuildContext context, StateSetter setState) => Scaffold(
                body: DraggableScrollableSheet(
                  initialChildSize: 0.25,
                  snap: true,
                  snapSizes: const <double>[0.25, 0.5, 1.0],
                  controller: controller,
                  builder: (BuildContext context, ScrollController scrollController) {
                    return ListView(
                      controller: scrollController,
                      children: <Widget>[
                        ElevatedButton(
                          onPressed:
                              () => setState(() {
                                controller = controller2;
                              }),
                          child: const Text('Switch controller'),
                        ),
                        Container(height: 10000, color: Colors.blue),
                      ],
                    );
                  },
                ),
              ),
        ),
      ),
    );
    expect(controller1.isAttached, true);
    expect(controller2.isAttached, false);

    controller1.addListener(() {
      loggedSizes.add(controller1.size);
    });
    controller1.jumpTo(0.5);
    expect(loggedSizes, <double>[0.5].map((double v) => closeTo(v, precisionErrorTolerance)));
    loggedSizes.clear();

    await tester.tap(find.text('Switch controller'));
    await tester.pump();

    expect(controller1.isAttached, false);
    expect(controller2.isAttached, true);

    controller2.addListener(() {
      loggedSizes.add(controller2.size);
    });
    controller2.jumpTo(1.0);
    expect(loggedSizes, <double>[1.0].map((double v) => closeTo(v, precisionErrorTolerance)));
  });

  testWidgets('DraggableScrollableSheet controller can be changed while animating', (
    WidgetTester tester,
  ) async {
    final DraggableScrollableController controller1 = DraggableScrollableController();
    addTearDown(controller1.dispose);
    final DraggableScrollableController controller2 = DraggableScrollableController();
    addTearDown(controller2.dispose);

    DraggableScrollableController controller = controller1;
    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder:
              (BuildContext context, StateSetter setState) => Scaffold(
                body: DraggableScrollableSheet(
                  initialChildSize: 0.25,
                  snap: true,
                  snapSizes: const <double>[0.25, 0.5, 1.0],
                  controller: controller,
                  builder: (BuildContext context, ScrollController scrollController) {
                    return ListView(
                      controller: scrollController,
                      children: <Widget>[
                        ElevatedButton(
                          onPressed:
                              () => setState(() {
                                controller = controller2;
                              }),
                          child: const Text('Switch controller'),
                        ),
                        Container(height: 10000, color: Colors.blue),
                      ],
                    );
                  },
                ),
              ),
        ),
      ),
    );
    expect(controller1.isAttached, true);
    expect(controller2.isAttached, false);

    controller1.animateTo(0.5, curve: Curves.linear, duration: const Duration(milliseconds: 200));
    await tester.pump();

    await tester.tap(find.text('Switch controller'));
    await tester.pump();

    expect(controller1.isAttached, false);
    expect(controller2.isAttached, true);

    controller2.animateTo(1.0, curve: Curves.linear, duration: const Duration(milliseconds: 200));
    await tester.pump();

    await tester.pumpWidget(const SizedBox.shrink());
    expect(controller1.isAttached, false);
    expect(controller2.isAttached, false);
  });

  testWidgets('$DraggableScrollableController dispatches creation in constructor.', (
    WidgetTester widgetTester,
  ) async {
    await expectLater(
      await memoryEvents(
        () async => DraggableScrollableController().dispose(),
        DraggableScrollableController,
      ),
      areCreateAndDispose,
    );
  });

  testWidgets('DraggableScrollableSheet respects shouldCloseOnMinExtent', (
    WidgetTester tester,
  ) async {
    final DraggableScrollableController controller = DraggableScrollableController();
    DraggableScrollableNotification? receivedNotification;

    Future<void> pumpWidgetAndFling() async {
      await tester.pumpWidget(
        boilerplateWidget(
          null,
          controller: controller,
          shouldCloseOnMinExtent: false,
          onDraggableScrollableNotification: (DraggableScrollableNotification notification) {
            receivedNotification = notification;
            return false;
          },
        ),
      );

      await tester.flingFrom(const Offset(0, 325), const Offset(0, -325), 200);
      await tester.pumpAndSettle();
    }

    await pumpWidgetAndFling();
    expect(receivedNotification!.shouldCloseOnMinExtent, isFalse);

    receivedNotification = null;
    controller.jumpTo(0.5);

    // Construct the widget a second time, to ensure didUpdateWidget is called.
    await pumpWidgetAndFling();
    expect(receivedNotification!.shouldCloseOnMinExtent, isFalse);
    controller.dispose();
  });
}
