// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

Future<void> pumpTest(
  WidgetTester tester,
  TargetPlatform? platform, {
  bool scrollable = true,
  bool reverse = false,
  Set<LogicalKeyboardKey>? axisModifier,
  Axis scrollDirection = Axis.vertical,
  ScrollController? controller,
  bool enableMouseDrag = true,
}) async {
  await tester.pumpWidget(
    MaterialApp(
      scrollBehavior: const NoScrollbarBehavior().copyWith(
        dragDevices:
            enableMouseDrag ? <ui.PointerDeviceKind>{...ui.PointerDeviceKind.values} : null,
        pointerAxisModifiers: axisModifier,
      ),
      theme: ThemeData(platform: platform),
      home: CustomScrollView(
        controller: controller,
        reverse: reverse,
        scrollDirection: scrollDirection,
        physics: scrollable ? null : const NeverScrollableScrollPhysics(),
        slivers: <Widget>[
          SliverToBoxAdapter(
            child: SizedBox(
              height: scrollDirection == Axis.vertical ? 2000.0 : null,
              width: scrollDirection == Axis.horizontal ? 2000.0 : null,
            ),
          ),
        ],
      ),
    ),
  );
  await tester.pump(const Duration(seconds: 5)); // to let the theme animate
}

class NoScrollbarBehavior extends MaterialScrollBehavior {
  const NoScrollbarBehavior();

  @override
  Widget buildScrollbar(BuildContext context, Widget child, ScrollableDetails details) => child;
}

// Pump a nested scrollable. The outer scrollable contains a sliver of a
// 300-pixel-long scrollable followed by a 2000-pixel-long content.
Future<void> pumpDoubleScrollableTest(WidgetTester tester, TargetPlatform platform) async {
  await tester.pumpWidget(
    MaterialApp(
      theme: ThemeData(platform: platform),
      home: const CustomScrollView(
        slivers: <Widget>[
          SliverToBoxAdapter(
            child: SizedBox(
              height: 300,
              child: CustomScrollView(
                slivers: <Widget>[SliverToBoxAdapter(child: SizedBox(height: 2000.0))],
              ),
            ),
          ),
          SliverToBoxAdapter(child: SizedBox(height: 2000.0)),
        ],
      ),
    ),
  );
  await tester.pump(const Duration(seconds: 5)); // to let the theme animate
}

const double dragOffset = 200.0;

double getScrollOffset(WidgetTester tester, {bool last = true}) {
  Finder viewportFinder = find.byType(Viewport);
  if (last) {
    viewportFinder = viewportFinder.last;
  }
  final RenderViewport viewport = tester.renderObject(viewportFinder);
  return viewport.offset.pixels;
}

double getScrollVelocity(WidgetTester tester) {
  final RenderViewport viewport = tester.renderObject(find.byType(Viewport));
  final ScrollPosition position = viewport.offset as ScrollPosition;
  return position.activity!.velocity;
}

void resetScrollOffset(WidgetTester tester) {
  final RenderViewport viewport = tester.renderObject(find.byType(Viewport));
  final ScrollPosition position = viewport.offset as ScrollPosition;
  position.jumpTo(0.0);
}

void main() {
  testWidgets('hitTestBehavior is respected', (WidgetTester tester) async {
    HitTestBehavior? getBehavior(Type of) {
      final RawGestureDetector widget = tester.widget(
        find.descendant(of: find.byType(of), matching: find.byType(RawGestureDetector)),
      );
      return widget.behavior;
    }

    await tester.pumpWidget(
      const MaterialApp(home: SingleChildScrollView(hitTestBehavior: HitTestBehavior.translucent)),
    );
    expect(getBehavior(SingleChildScrollView), HitTestBehavior.translucent);

    await tester.pumpWidget(
      const MaterialApp(home: CustomScrollView(hitTestBehavior: HitTestBehavior.translucent)),
    );
    expect(getBehavior(CustomScrollView), HitTestBehavior.translucent);

    await tester.pumpWidget(
      MaterialApp(home: ListView(hitTestBehavior: HitTestBehavior.translucent)),
    );
    expect(getBehavior(ListView), HitTestBehavior.translucent);

    await tester.pumpWidget(
      MaterialApp(
        home: GridView.extent(maxCrossAxisExtent: 1, hitTestBehavior: HitTestBehavior.translucent),
      ),
    );
    expect(getBehavior(GridView), HitTestBehavior.translucent);

    await tester.pumpWidget(
      MaterialApp(home: PageView(hitTestBehavior: HitTestBehavior.translucent)),
    );
    expect(getBehavior(PageView), HitTestBehavior.translucent);

    await tester.pumpWidget(
      MaterialApp(
        home: ListWheelScrollView(
          itemExtent: 10,
          hitTestBehavior: HitTestBehavior.translucent,
          children: const <Widget>[],
        ),
      ),
    );
    expect(getBehavior(ListWheelScrollView), HitTestBehavior.translucent);
  });

  testWidgets('hitTestBehavior.translucent lets widgets underneath catch the hit', (
    WidgetTester tester,
  ) async {
    final Key key = UniqueKey();
    bool tapped = false;
    await tester.pumpWidget(
      MaterialApp(
        home: Stack(
          children: <Widget>[
            Positioned.fill(
              child: GestureDetector(
                behavior: HitTestBehavior.opaque,
                onTap: () => tapped = true,
                child: SizedBox(key: key, height: 300),
              ),
            ),
            const SingleChildScrollView(hitTestBehavior: HitTestBehavior.translucent),
          ],
        ),
      ),
    );
    await tester.tapAt(tester.getCenter(find.byKey(key)));
    expect(tapped, isTrue);
  });

  testWidgets('Flings on different platforms', (WidgetTester tester) async {
    await pumpTest(tester, TargetPlatform.android);
    await tester.fling(find.byType(Scrollable), const Offset(0.0, -dragOffset), 1000.0);
    expect(getScrollOffset(tester), dragOffset);
    await tester.pump(); // trigger fling
    expect(getScrollOffset(tester), dragOffset);
    await tester.pump(const Duration(seconds: 5));
    final double androidResult = getScrollOffset(tester);

    resetScrollOffset(tester);

    await pumpTest(tester, TargetPlatform.iOS);
    await tester.fling(find.byType(Scrollable), const Offset(0.0, -dragOffset), 1000.0);
    // Scroll starts ease into the scroll on iOS.
    expect(getScrollOffset(tester), moreOrLessEquals(197.16666666666669));
    await tester.pump(); // trigger fling
    expect(getScrollOffset(tester), moreOrLessEquals(197.16666666666669));
    await tester.pump(const Duration(seconds: 5));
    final double iOSResult = getScrollOffset(tester);

    resetScrollOffset(tester);

    await pumpTest(tester, TargetPlatform.macOS);
    await tester.fling(find.byType(Scrollable), const Offset(0.0, -dragOffset), 1000.0);
    // Scroll starts ease into the scroll on iOS.
    expect(getScrollOffset(tester), moreOrLessEquals(197.16666666666669));
    await tester.pump(); // trigger fling
    expect(getScrollOffset(tester), moreOrLessEquals(197.16666666666669));
    await tester.pump(const Duration(seconds: 5));
    final double macOSResult = getScrollOffset(tester);

    expect(macOSResult, lessThan(androidResult)); // macOS is slipperier than Android
    expect(androidResult, lessThan(iOSResult)); // iOS is slipperier than Android
    expect(macOSResult, lessThan(iOSResult)); // iOS is slipperier than macOS
  });

  testWidgets(
    'Holding scroll',
    (WidgetTester tester) async {
      await pumpTest(tester, debugDefaultTargetPlatformOverride);
      await tester.drag(find.byType(Scrollable), const Offset(0.0, 200.0), touchSlopY: 0.0);
      expect(getScrollOffset(tester), -200.0);
      await tester.pump(); // trigger ballistic
      await tester.pump(const Duration(milliseconds: 10));
      expect(getScrollOffset(tester), greaterThan(-200.0));
      expect(getScrollOffset(tester), lessThan(0.0));
      final double heldPosition = getScrollOffset(tester);
      // Hold and let go while in overscroll.
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byType(Scrollable), warnIfMissed: true),
      );
      expect(await tester.pumpAndSettle(), 1);
      expect(getScrollOffset(tester), heldPosition);
      await gesture.up();
      // Once the hold is let go, it should still snap back to origin.
      expect(await tester.pumpAndSettle(const Duration(minutes: 1)), 3);
      expect(getScrollOffset(tester), 0.0);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'Repeated flings builds momentum',
    (WidgetTester tester) async {
      await pumpTest(tester, debugDefaultTargetPlatformOverride);
      await tester.fling(find.byType(Scrollable), const Offset(0.0, -dragOffset), 1000.0);
      await tester.pump(); // trigger fling
      await tester.pump(const Duration(milliseconds: 10));
      // Repeat the exact same motion.
      await tester.fling(find.byType(Scrollable), const Offset(0.0, -dragOffset), 1000.0);
      await tester.pump();
      // On iOS, the velocity will be larger than the velocity of the last fling by a
      // non-trivial amount.
      expect(getScrollVelocity(tester), greaterThan(1100.0));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('Repeated flings do not build momentum on Android', (WidgetTester tester) async {
    await pumpTest(tester, TargetPlatform.android);
    await tester.fling(find.byType(Scrollable), const Offset(0.0, -dragOffset), 1000.0);
    await tester.pump(); // trigger fling
    await tester.pump(const Duration(milliseconds: 10));
    // Repeat the exact same motion.
    await tester.fling(find.byType(Scrollable), const Offset(0.0, -dragOffset), 1000.0);
    await tester.pump();
    // On Android, there is no momentum build. The final velocity is the same as the
    // velocity of the last fling.
    expect(getScrollVelocity(tester), moreOrLessEquals(1000.0));
  });

  testWidgets(
    'A slower final fling does not apply carried momentum',
    (WidgetTester tester) async {
      await pumpTest(tester, debugDefaultTargetPlatformOverride);
      await tester.fling(find.byType(Scrollable), const Offset(0.0, -dragOffset), 1000.0);
      await tester.pump(); // trigger fling
      await tester.pump(const Duration(milliseconds: 10));
      // Repeat the exact same motion to build momentum.
      await tester.fling(find.byType(Scrollable), const Offset(0.0, -dragOffset), 1000.0);
      await tester.pump(); // trigger the second fling
      await tester.pump(const Duration(milliseconds: 10));
      // Make a final fling that is much slower.
      await tester.fling(find.byType(Scrollable), const Offset(0.0, -dragOffset), 200.0);
      await tester.pump(); // trigger the third fling
      await tester.pump(const Duration(milliseconds: 10));
      // expect that there is no carried velocity
      expect(getScrollVelocity(tester), lessThan(200.0));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'No iOS/macOS momentum build with flings in opposite directions',
    (WidgetTester tester) async {
      await pumpTest(tester, debugDefaultTargetPlatformOverride);
      await tester.fling(find.byType(Scrollable), const Offset(0.0, -dragOffset), 1000.0);
      await tester.pump(); // trigger fling
      await tester.pump(const Duration(milliseconds: 10));
      // Repeat the exact same motion in the opposite direction.
      await tester.fling(find.byType(Scrollable), const Offset(0.0, dragOffset), 1000.0);
      await tester.pump();
      // The only applied velocity to the scrollable is the second fling that was in the
      // opposite direction.
      expect(getScrollVelocity(tester), -1000.0);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'No iOS/macOS momentum kept on hold gestures',
    (WidgetTester tester) async {
      await pumpTest(tester, debugDefaultTargetPlatformOverride);
      await tester.fling(find.byType(Scrollable), const Offset(0.0, -dragOffset), 1000.0);
      await tester.pump(); // trigger fling
      await tester.pump(const Duration(milliseconds: 10));
      expect(getScrollVelocity(tester), greaterThan(0.0));
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byType(Scrollable), warnIfMissed: true),
      );
      await tester.pump(const Duration(milliseconds: 40));
      await gesture.up();
      // After a hold longer than 2 frames, previous velocity is lost.
      expect(getScrollVelocity(tester), 0.0);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('Drags creeping unaffected on Android', (WidgetTester tester) async {
    await pumpTest(tester, TargetPlatform.android);
    final TestGesture gesture = await tester.startGesture(
      tester.getCenter(find.byType(Scrollable), warnIfMissed: true),
    );
    await gesture.moveBy(const Offset(0.0, -0.5));
    expect(getScrollOffset(tester), 0.5);
    await gesture.moveBy(const Offset(0.0, -0.5), timeStamp: const Duration(milliseconds: 10));
    expect(getScrollOffset(tester), 1.0);
    await gesture.moveBy(const Offset(0.0, -0.5), timeStamp: const Duration(milliseconds: 20));
    expect(getScrollOffset(tester), 1.5);
  });

  testWidgets(
    'Drags creeping must break threshold on iOS/macOS',
    (WidgetTester tester) async {
      await pumpTest(tester, debugDefaultTargetPlatformOverride);
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byType(Scrollable), warnIfMissed: true),
      );
      await gesture.moveBy(const Offset(0.0, -0.5));
      expect(getScrollOffset(tester), 0.0);
      await gesture.moveBy(const Offset(0.0, -0.5), timeStamp: const Duration(milliseconds: 10));
      expect(getScrollOffset(tester), 0.0);
      await gesture.moveBy(const Offset(0.0, -0.5), timeStamp: const Duration(milliseconds: 20));
      expect(getScrollOffset(tester), 0.0);
      await gesture.moveBy(const Offset(0.0, -1.0), timeStamp: const Duration(milliseconds: 30));
      // Now -2.5 in total.
      expect(getScrollOffset(tester), 0.0);
      await gesture.moveBy(const Offset(0.0, -1.0), timeStamp: const Duration(milliseconds: 40));
      // Now -3.5, just reached threshold.
      expect(getScrollOffset(tester), 0.0);
      await gesture.moveBy(const Offset(0.0, -0.5), timeStamp: const Duration(milliseconds: 50));
      // -0.5 over threshold transferred.
      expect(getScrollOffset(tester), 0.5);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'Big drag over threshold magnitude preserved on iOS/macOS',
    (WidgetTester tester) async {
      await pumpTest(tester, debugDefaultTargetPlatformOverride);
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byType(Scrollable), warnIfMissed: true),
      );
      await gesture.moveBy(const Offset(0.0, -30.0));
      // No offset lost from threshold.
      expect(getScrollOffset(tester), 30.0);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'Slow threshold breaks are attenuated on iOS/macOS',
    (WidgetTester tester) async {
      await pumpTest(tester, debugDefaultTargetPlatformOverride);
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byType(Scrollable), warnIfMissed: true),
      );
      // This is a typical 'hesitant' iOS scroll start.
      await gesture.moveBy(const Offset(0.0, -10.0));
      expect(getScrollOffset(tester), moreOrLessEquals(1.1666666666666667));
      await gesture.moveBy(const Offset(0.0, -10.0), timeStamp: const Duration(milliseconds: 20));
      // Subsequent motions unaffected.
      expect(getScrollOffset(tester), moreOrLessEquals(11.16666666666666673));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'Small continuing motion preserved on iOS/macOS',
    (WidgetTester tester) async {
      await pumpTest(tester, debugDefaultTargetPlatformOverride);
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byType(Scrollable), warnIfMissed: true),
      );
      await gesture.moveBy(const Offset(0.0, -30.0)); // Break threshold.
      expect(getScrollOffset(tester), 30.0);
      await gesture.moveBy(const Offset(0.0, -0.5), timeStamp: const Duration(milliseconds: 20));
      expect(getScrollOffset(tester), 30.5);
      await gesture.moveBy(const Offset(0.0, -0.5), timeStamp: const Duration(milliseconds: 40));
      expect(getScrollOffset(tester), 31.0);
      await gesture.moveBy(const Offset(0.0, -0.5), timeStamp: const Duration(milliseconds: 60));
      expect(getScrollOffset(tester), 31.5);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'Motion stop resets threshold on iOS/macOS',
    (WidgetTester tester) async {
      await pumpTest(tester, debugDefaultTargetPlatformOverride);
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byType(Scrollable), warnIfMissed: true),
      );
      await gesture.moveBy(const Offset(0.0, -30.0)); // Break threshold.
      expect(getScrollOffset(tester), 30.0);
      await gesture.moveBy(const Offset(0.0, -0.5), timeStamp: const Duration(milliseconds: 20));
      expect(getScrollOffset(tester), 30.5);
      await gesture.moveBy(Offset.zero, timeStamp: const Duration(milliseconds: 21));
      // Stationary too long, threshold reset.
      await gesture.moveBy(Offset.zero, timeStamp: const Duration(milliseconds: 120));
      await gesture.moveBy(const Offset(0.0, -1.0), timeStamp: const Duration(milliseconds: 140));
      expect(getScrollOffset(tester), 30.5);
      await gesture.moveBy(const Offset(0.0, -1.0), timeStamp: const Duration(milliseconds: 150));
      expect(getScrollOffset(tester), 30.5);
      await gesture.moveBy(const Offset(0.0, -1.0), timeStamp: const Duration(milliseconds: 160));
      expect(getScrollOffset(tester), 30.5);
      await gesture.moveBy(const Offset(0.0, -1.0), timeStamp: const Duration(milliseconds: 170));
      // New threshold broken.
      expect(getScrollOffset(tester), 31.5);
      await gesture.moveBy(const Offset(0.0, -1.0), timeStamp: const Duration(milliseconds: 180));
      expect(getScrollOffset(tester), 32.5);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('Scroll pointer signals are handled on Fuchsia', (WidgetTester tester) async {
    await pumpTest(tester, TargetPlatform.fuchsia);
    final Offset scrollEventLocation = tester.getCenter(find.byType(Viewport));
    final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
    // Create a hover event so that |testPointer| has a location when generating the scroll.
    testPointer.hover(scrollEventLocation);
    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));
    expect(getScrollOffset(tester), 20.0);
    // Pointer signals should not cause overscroll.
    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -30.0)));
    expect(getScrollOffset(tester), 0.0);
  });

  testWidgets('Scroll pointer signals are handled when there is competition', (
    WidgetTester tester,
  ) async {
    // This is a regression test. When there are multiple scrollables listening
    // to the same event, for example when scrollables are nested, there used
    // to be exceptions at scrolling events.

    await pumpDoubleScrollableTest(tester, TargetPlatform.fuchsia);
    final Offset scrollEventLocation = tester.getCenter(find.byType(Viewport).last);
    final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
    // Create a hover event so that |testPointer| has a location when generating the scroll.
    testPointer.hover(scrollEventLocation);
    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));
    expect(getScrollOffset(tester), 20.0);
    // Pointer signals should not cause overscroll.
    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -30.0)));
    expect(getScrollOffset(tester), 0.0);
  });

  testWidgets('Scroll pointer signals are ignored when scrolling is disabled', (
    WidgetTester tester,
  ) async {
    await pumpTest(tester, TargetPlatform.fuchsia, scrollable: false);
    final Offset scrollEventLocation = tester.getCenter(find.byType(Viewport));
    final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
    // Create a hover event so that |testPointer| has a location when generating the scroll.
    testPointer.hover(scrollEventLocation);
    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));
    expect(getScrollOffset(tester), 0.0);
  });

  testWidgets(
    'Engine is notified of ignored pointer signals (no scroll physics)',
    (WidgetTester tester) async {
      await pumpTest(tester, debugDefaultTargetPlatformOverride, scrollable: false);
      final Offset scrollEventLocation = tester.getCenter(find.byType(Viewport));
      final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
      // Create a hover event so that |testPointer| has a location when generating the scroll.
      testPointer.hover(scrollEventLocation);

      bool allowedPlatformDefault = false;
      await tester.sendEventToBinding(
        testPointer.scroll(
          const Offset(0.0, 20.0),
          onRespond: ({required bool allowPlatformDefault}) {
            allowedPlatformDefault = allowPlatformDefault;
          },
        ),
      );

      expect(
        allowedPlatformDefault,
        isTrue,
        reason: 'Engine should be notified of ignored scroll pointer signals.',
      );
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'Engine is notified of rejected scroll events (wrong direction)',
    (WidgetTester tester) async {
      await pumpTest(tester, debugDefaultTargetPlatformOverride, scrollDirection: Axis.horizontal);

      final Offset scrollEventLocation = tester.getCenter(find.byType(Viewport));
      final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
      // Create a hover event so that |testPointer| has a location when generating the scroll.
      testPointer.hover(scrollEventLocation);

      // Horizontal input is accepted
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      await tester.sendEventToBinding(
        testPointer.scroll(
          const Offset(0.0, 10.0),
          onRespond: ({required bool allowPlatformDefault}) {
            fail('The engine should not be notified when the scroll is accepted.');
          },
        ),
      );
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
      await tester.pump();

      // Vertical input not accepted
      bool allowedPlatformDefault = false;
      await tester.sendEventToBinding(
        testPointer.scroll(
          const Offset(0.0, 20.0),
          onRespond: ({required bool allowPlatformDefault}) {
            allowedPlatformDefault = allowPlatformDefault;
          },
        ),
      );
      expect(
        allowedPlatformDefault,
        isTrue,
        reason: 'Engine should be notified when scroll is rejected by the scrollable.',
      );
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'Holding scroll and Scroll pointer signal will update ScrollDirection.forward / ScrollDirection.reverse',
    (WidgetTester tester) async {
      ScrollDirection? lastUserScrollingDirection;

      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);

      await pumpTest(tester, TargetPlatform.fuchsia, controller: controller);

      controller.addListener(() {
        if (controller.position.userScrollDirection != ScrollDirection.idle) {
          lastUserScrollingDirection = controller.position.userScrollDirection;
        }
      });

      await tester.drag(find.byType(Scrollable), const Offset(0.0, -20.0), touchSlopY: 0.0);

      expect(lastUserScrollingDirection, ScrollDirection.reverse);

      final Offset scrollEventLocation = tester.getCenter(find.byType(Viewport));
      final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
      // Create a hover event so that |testPointer| has a location when generating the scroll.
      testPointer.hover(scrollEventLocation);
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));

      expect(lastUserScrollingDirection, ScrollDirection.reverse);

      await tester.drag(find.byType(Scrollable), const Offset(0.0, 20.0), touchSlopY: 0.0);

      expect(lastUserScrollingDirection, ScrollDirection.forward);

      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -20.0)));

      expect(lastUserScrollingDirection, ScrollDirection.forward);
    },
  );

  testWidgets('Scrolls in correct direction when scroll axis is reversed', (
    WidgetTester tester,
  ) async {
    await pumpTest(tester, TargetPlatform.fuchsia, reverse: true);

    final Offset scrollEventLocation = tester.getCenter(find.byType(Viewport));
    final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
    // Create a hover event so that |testPointer| has a location when generating the scroll.
    testPointer.hover(scrollEventLocation);
    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -20.0)));

    expect(getScrollOffset(tester), 20.0);
  });

  testWidgets('Scrolls horizontally when shift is pressed by default', (WidgetTester tester) async {
    await pumpTest(tester, debugDefaultTargetPlatformOverride, scrollDirection: Axis.horizontal);

    final Offset scrollEventLocation = tester.getCenter(find.byType(Viewport));
    final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
    // Create a hover event so that |testPointer| has a location when generating the scroll.
    testPointer.hover(scrollEventLocation);
    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));
    // Vertical input not accepted
    expect(getScrollOffset(tester), 0.0);

    await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));
    // Vertical input flipped to horizontal and accepted.
    expect(getScrollOffset(tester), 20.0);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
    await tester.pump();

    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));
    // Vertical input not accepted
    expect(getScrollOffset(tester), 20.0);
  }, variant: TargetPlatformVariant.all());

  testWidgets('Scroll axis is not flipped for trackpad', (WidgetTester tester) async {
    await pumpTest(tester, debugDefaultTargetPlatformOverride, scrollDirection: Axis.horizontal);

    final Offset scrollEventLocation = tester.getCenter(find.byType(Viewport));
    final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.trackpad);
    // Create a hover event so that |testPointer| has a location when generating the scroll.
    testPointer.hover(scrollEventLocation);
    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));
    // Vertical input not accepted
    expect(getScrollOffset(tester), 0.0);

    await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));
    // Vertical input not flipped.
    expect(getScrollOffset(tester), 0.0);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
    await tester.pump();

    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));
    // Vertical input not accepted
    expect(getScrollOffset(tester), 0.0);
  }, variant: TargetPlatformVariant.all());

  testWidgets('Scrolls horizontally when custom key is pressed', (WidgetTester tester) async {
    await pumpTest(
      tester,
      debugDefaultTargetPlatformOverride,
      scrollDirection: Axis.horizontal,
      axisModifier: <LogicalKeyboardKey>{LogicalKeyboardKey.altLeft},
    );

    final Offset scrollEventLocation = tester.getCenter(find.byType(Viewport));
    final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
    // Create a hover event so that |testPointer| has a location when generating the scroll.
    testPointer.hover(scrollEventLocation);
    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));
    // Vertical input not accepted
    expect(getScrollOffset(tester), 0.0);

    await tester.sendKeyDownEvent(LogicalKeyboardKey.altLeft);
    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));
    // Vertical input flipped to horizontal and accepted.
    expect(getScrollOffset(tester), 20.0);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.altLeft);
    await tester.pump();

    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));
    // Vertical input not accepted
    expect(getScrollOffset(tester), 20.0);
  }, variant: TargetPlatformVariant.all());

  testWidgets(
    'Still scrolls horizontally when other keys are pressed at the same time',
    (WidgetTester tester) async {
      await pumpTest(
        tester,
        debugDefaultTargetPlatformOverride,
        scrollDirection: Axis.horizontal,
        axisModifier: <LogicalKeyboardKey>{LogicalKeyboardKey.altLeft},
      );

      final Offset scrollEventLocation = tester.getCenter(find.byType(Viewport));
      final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
      // Create a hover event so that |testPointer| has a location when generating the scroll.
      testPointer.hover(scrollEventLocation);
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));
      // Vertical input not accepted
      expect(getScrollOffset(tester), 0.0);

      await tester.sendKeyDownEvent(LogicalKeyboardKey.altLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.space);
      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));
      // Vertical flipped & accepted.
      expect(getScrollOffset(tester), 20.0);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.altLeft);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.space);
      await tester.pump();

      await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));
      // Vertical input not accepted
      expect(getScrollOffset(tester), 20.0);
    },
    variant: TargetPlatformVariant.all(),
  );

  group('setCanDrag to false with active drag gesture: ', () {
    Future<void> pumpTestWidget(WidgetTester tester, {required bool canDrag}) {
      return tester.pumpWidget(
        MaterialApp(
          home: CustomScrollView(
            physics:
                canDrag
                    ? const AlwaysScrollableScrollPhysics()
                    : const NeverScrollableScrollPhysics(),
            slivers: <Widget>[
              SliverToBoxAdapter(
                child: SizedBox(height: 2000, child: GestureDetector(onTap: () {})),
              ),
            ],
          ),
        ),
      );
    }

    testWidgets('Hold does not disable user interaction', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/66816.
      await pumpTestWidget(tester, canDrag: true);
      final RenderIgnorePointer renderIgnorePointer = tester.renderObject<RenderIgnorePointer>(
        find.descendant(of: find.byType(CustomScrollView), matching: find.byType(IgnorePointer)),
      );

      expect(renderIgnorePointer.ignoring, false);

      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byType(Viewport)),
      );
      expect(renderIgnorePointer.ignoring, false);

      await pumpTestWidget(tester, canDrag: false);
      expect(renderIgnorePointer.ignoring, false);

      await gesture.up();
      expect(renderIgnorePointer.ignoring, false);
    });

    testWidgets('Drag disables user interaction when recognized', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/66816.
      await pumpTestWidget(tester, canDrag: true);
      final RenderIgnorePointer renderIgnorePointer = tester.renderObject<RenderIgnorePointer>(
        find.descendant(of: find.byType(CustomScrollView), matching: find.byType(IgnorePointer)),
      );
      expect(renderIgnorePointer.ignoring, false);

      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byType(Viewport)),
      );
      expect(renderIgnorePointer.ignoring, false);

      await gesture.moveBy(const Offset(0, -100));
      // Starts ignoring when the drag is recognized.
      expect(renderIgnorePointer.ignoring, true);

      await pumpTestWidget(tester, canDrag: false);
      expect(renderIgnorePointer.ignoring, false);

      await gesture.up();
      expect(renderIgnorePointer.ignoring, false);
    });

    testWidgets('Ballistic disables user interaction until it stops', (WidgetTester tester) async {
      await pumpTestWidget(tester, canDrag: true);
      final RenderIgnorePointer renderIgnorePointer = tester.renderObject<RenderIgnorePointer>(
        find.descendant(of: find.byType(CustomScrollView), matching: find.byType(IgnorePointer)),
      );
      expect(renderIgnorePointer.ignoring, false);

      // Starts ignoring when the drag is recognized.
      await tester.fling(find.byType(Scrollable), const Offset(0, -100), 1000);
      expect(renderIgnorePointer.ignoring, true);
      await tester.pump();

      // When the activity ends we should stop ignoring pointers.
      await tester.pumpAndSettle();
      expect(renderIgnorePointer.ignoring, false);
    });
  });

  testWidgets('Can recommendDeferredLoadingForContext - animation', (WidgetTester tester) async {
    final List<String> widgetTracker = <String>[];
    int cheapWidgets = 0;
    int expensiveWidgets = 0;
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView.builder(
          controller: controller,
          itemBuilder: (BuildContext context, int index) {
            if (Scrollable.recommendDeferredLoadingForContext(context)) {
              cheapWidgets += 1;
              widgetTracker.add('cheap');
              return const SizedBox(height: 50.0);
            }
            widgetTracker.add('expensive');
            expensiveWidgets += 1;
            return const SizedBox(height: 50.0);
          },
        ),
      ),
    );

    await tester.pumpAndSettle();

    expect(expensiveWidgets, 17);
    expect(cheapWidgets, 0);

    // The position value here is different from the maximum velocity we will
    // reach, which is controlled by a combination of curve, duration, and
    // position.
    // This is just meant to be a pretty good simulation. A linear curve
    // with these same parameters will never back off on the velocity enough
    // to reset here.
    controller.animateTo(5000, duration: const Duration(seconds: 2), curve: Curves.linear);

    expect(expensiveWidgets, 17);
    expect(widgetTracker.every((String type) => type == 'expensive'), true);
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));
    expect(expensiveWidgets, 17);
    expect(cheapWidgets, 25);
    expect(widgetTracker.skip(17).every((String type) => type == 'cheap'), true);

    await tester.pumpAndSettle();

    expect(expensiveWidgets, 22);
    expect(cheapWidgets, 95);
    expect(widgetTracker.skip(17).skip(25).take(70).every((String type) => type == 'cheap'), true);
    expect(
      widgetTracker.skip(17).skip(25).skip(70).every((String type) => type == 'expensive'),
      true,
    );
  });

  testWidgets('Can recommendDeferredLoadingForContext - ballistics', (WidgetTester tester) async {
    int cheapWidgets = 0;
    int expensiveWidgets = 0;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView.builder(
          itemBuilder: (BuildContext context, int index) {
            if (Scrollable.recommendDeferredLoadingForContext(context)) {
              cheapWidgets += 1;
              return const SizedBox(height: 50.0);
            }
            expensiveWidgets += 1;
            return SizedBox(key: ValueKey<String>('Box $index'), height: 50.0);
          },
        ),
      ),
    );

    await tester.pumpAndSettle();
    expect(find.byKey(const ValueKey<String>('Box 0')), findsOneWidget);
    expect(find.byKey(const ValueKey<String>('Box 52')), findsNothing);

    expect(expensiveWidgets, 17);
    expect(cheapWidgets, 0);

    // Getting the tester to simulate a life-like fling is difficult.
    // Instead, just manually drive the activity with a ballistic simulation as
    // if the user has flung the list.
    Scrollable.of(
      find.byType(SizedBox).evaluate().first,
    ).position.activity!.delegate.goBallistic(4000);

    await tester.pumpAndSettle();
    expect(find.byKey(const ValueKey<String>('Box 0')), findsNothing);
    expect(find.byKey(const ValueKey<String>('Box 52')), findsOneWidget);

    expect(expensiveWidgets, 40);
    expect(cheapWidgets, 21);
  });

  testWidgets('Can recommendDeferredLoadingForContext - override heuristic', (
    WidgetTester tester,
  ) async {
    int cheapWidgets = 0;
    int expensiveWidgets = 0;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView.builder(
          physics: SuperPessimisticScrollPhysics(),
          itemBuilder: (BuildContext context, int index) {
            if (Scrollable.recommendDeferredLoadingForContext(context)) {
              cheapWidgets += 1;
              return SizedBox(key: ValueKey<String>('Cheap box $index'), height: 50.0);
            }
            expensiveWidgets += 1;
            return SizedBox(key: ValueKey<String>('Box $index'), height: 50.0);
          },
        ),
      ),
    );
    await tester.pumpAndSettle();

    final ScrollPosition position = Scrollable.of(find.byType(SizedBox).evaluate().first).position;
    final SuperPessimisticScrollPhysics physics = position.physics as SuperPessimisticScrollPhysics;

    expect(find.byKey(const ValueKey<String>('Box 0')), findsOneWidget);
    expect(find.byKey(const ValueKey<String>('Cheap box 52')), findsNothing);

    expect(physics.count, 17);
    expect(expensiveWidgets, 17);
    expect(cheapWidgets, 0);

    // Getting the tester to simulate a life-like fling is difficult.
    // Instead, just manually drive the activity with a ballistic simulation as
    // if the user has flung the list.
    position.activity!.delegate.goBallistic(4000);

    await tester.pumpAndSettle();

    expect(find.byKey(const ValueKey<String>('Box 0')), findsNothing);
    expect(find.byKey(const ValueKey<String>('Cheap box 52')), findsOneWidget);

    expect(expensiveWidgets, 17);
    expect(cheapWidgets, 44);
    expect(physics.count, 44 + 17);
  });

  testWidgets(
    'Can recommendDeferredLoadingForContext - override heuristic and always return true',
    (WidgetTester tester) async {
      int cheapWidgets = 0;
      int expensiveWidgets = 0;
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: ListView.builder(
            physics: const ExtraSuperPessimisticScrollPhysics(),
            itemBuilder: (BuildContext context, int index) {
              if (Scrollable.recommendDeferredLoadingForContext(context)) {
                cheapWidgets += 1;
                return SizedBox(key: ValueKey<String>('Cheap box $index'), height: 50.0);
              }
              expensiveWidgets += 1;
              return SizedBox(key: ValueKey<String>('Box $index'), height: 50.0);
            },
          ),
        ),
      );
      await tester.pumpAndSettle();

      final ScrollPosition position =
          Scrollable.of(find.byType(SizedBox).evaluate().first).position;

      expect(find.byKey(const ValueKey<String>('Cheap box 0')), findsOneWidget);
      expect(find.byKey(const ValueKey<String>('Cheap box 52')), findsNothing);

      expect(expensiveWidgets, 0);
      expect(cheapWidgets, 17);

      // Getting the tester to simulate a life-like fling is difficult.
      // Instead, just manually drive the activity with a ballistic simulation as
      // if the user has flung the list.
      position.activity!.delegate.goBallistic(4000);

      await tester.pumpAndSettle();

      expect(find.byKey(const ValueKey<String>('Cheap box 0')), findsNothing);
      expect(find.byKey(const ValueKey<String>('Cheap box 52')), findsOneWidget);

      expect(expensiveWidgets, 0);
      expect(cheapWidgets, 61);
    },
  );

  testWidgets('ensureVisible does not move PageViews', (WidgetTester tester) async {
    final PageController controller = PageController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: PageView(
          controller: controller,
          children: List<ListView>.generate(3, (int pageIndex) {
            return ListView(
              key: Key('list_$pageIndex'),
              children: List<Widget>.generate(100, (int listIndex) {
                return Row(
                  children: <Widget>[
                    Container(
                      key: Key('${pageIndex}_${listIndex}_0'),
                      color: Colors.red,
                      width: 200,
                      height: 10,
                    ),
                    Container(
                      key: Key('${pageIndex}_${listIndex}_1'),
                      color: Colors.blue,
                      width: 200,
                      height: 10,
                    ),
                    Container(
                      key: Key('${pageIndex}_${listIndex}_2'),
                      color: Colors.green,
                      width: 200,
                      height: 10,
                    ),
                  ],
                );
              }),
            );
          }),
        ),
      ),
    );

    final Finder targetMidRightPage0 = find.byKey(const Key('0_25_2'));
    final Finder targetMidRightPage1 = find.byKey(const Key('1_25_2'));
    final Finder targetMidLeftPage1 = find.byKey(const Key('1_25_0'));

    expect(find.byKey(const Key('list_0')), findsOneWidget);
    expect(find.byKey(const Key('list_1')), findsNothing);
    expect(targetMidRightPage0, findsOneWidget);
    expect(targetMidRightPage1, findsNothing);
    expect(targetMidLeftPage1, findsNothing);

    await tester.ensureVisible(targetMidRightPage0);
    await tester.pumpAndSettle();
    expect(targetMidRightPage0, findsOneWidget);
    expect(targetMidRightPage1, findsNothing);
    expect(targetMidLeftPage1, findsNothing);

    controller.jumpToPage(1);
    await tester.pumpAndSettle();

    expect(find.byKey(const Key('list_0')), findsNothing);
    expect(find.byKey(const Key('list_1')), findsOneWidget);
    await tester.ensureVisible(targetMidRightPage1);
    await tester.pumpAndSettle();

    expect(targetMidRightPage0, findsNothing);
    expect(targetMidRightPage1, findsOneWidget);
    expect(targetMidLeftPage1, findsOneWidget);

    await tester.ensureVisible(targetMidLeftPage1);
    await tester.pumpAndSettle();

    expect(targetMidRightPage0, findsNothing);
    expect(targetMidRightPage1, findsOneWidget);
    expect(targetMidLeftPage1, findsOneWidget);
  });

  testWidgets('ensureVisible does not move TabViews', (WidgetTester tester) async {
    final TickerProvider vsync = TestTickerProvider();
    final TabController controller = TabController(length: 3, vsync: vsync);
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: TabBarView(
          controller: controller,
          children: List<ListView>.generate(3, (int pageIndex) {
            return ListView(
              key: Key('list_$pageIndex'),
              children: List<Widget>.generate(100, (int listIndex) {
                return Row(
                  children: <Widget>[
                    Container(
                      key: Key('${pageIndex}_${listIndex}_0'),
                      color: Colors.red,
                      width: 200,
                      height: 10,
                    ),
                    Container(
                      key: Key('${pageIndex}_${listIndex}_1'),
                      color: Colors.blue,
                      width: 200,
                      height: 10,
                    ),
                    Container(
                      key: Key('${pageIndex}_${listIndex}_2'),
                      color: Colors.green,
                      width: 200,
                      height: 10,
                    ),
                  ],
                );
              }),
            );
          }),
        ),
      ),
    );

    final Finder targetMidRightPage0 = find.byKey(const Key('0_25_2'));
    final Finder targetMidRightPage1 = find.byKey(const Key('1_25_2'));
    final Finder targetMidLeftPage1 = find.byKey(const Key('1_25_0'));

    expect(find.byKey(const Key('list_0')), findsOneWidget);
    expect(find.byKey(const Key('list_1')), findsNothing);
    expect(targetMidRightPage0, findsOneWidget);
    expect(targetMidRightPage1, findsNothing);
    expect(targetMidLeftPage1, findsNothing);

    await tester.ensureVisible(targetMidRightPage0);
    await tester.pumpAndSettle();
    expect(targetMidRightPage0, findsOneWidget);
    expect(targetMidRightPage1, findsNothing);
    expect(targetMidLeftPage1, findsNothing);

    controller.index = 1;
    await tester.pumpAndSettle();

    expect(find.byKey(const Key('list_0')), findsNothing);
    expect(find.byKey(const Key('list_1')), findsOneWidget);
    await tester.ensureVisible(targetMidRightPage1);
    await tester.pumpAndSettle();

    expect(targetMidRightPage0, findsNothing);
    expect(targetMidRightPage1, findsOneWidget);
    expect(targetMidLeftPage1, findsOneWidget);

    await tester.ensureVisible(targetMidLeftPage1);
    await tester.pumpAndSettle();

    expect(targetMidRightPage0, findsNothing);
    expect(targetMidRightPage1, findsOneWidget);
    expect(targetMidLeftPage1, findsOneWidget);
  });

  testWidgets('PointerScroll on nested NeverScrollable ListView goes to outer Scrollable.', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/70948
    final ScrollController outerController = ScrollController();
    addTearDown(outerController.dispose);
    final ScrollController innerController = ScrollController();
    addTearDown(innerController.dispose);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: SingleChildScrollView(
            controller: outerController,
            child: Row(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                Column(
                  children: <Widget>[
                    for (int i = 0; i < 100; i++) Text('SingleChildScrollView $i'),
                  ],
                ),
                SizedBox(
                  height: 3000,
                  width: 400,
                  child: ListView.builder(
                    controller: innerController,
                    physics: const NeverScrollableScrollPhysics(),
                    itemCount: 100,
                    itemBuilder: (BuildContext context, int index) {
                      return Text('Nested NeverScrollable ListView $index');
                    },
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
    expect(outerController.position.pixels, 0.0);
    expect(innerController.position.pixels, 0.0);
    final Offset outerScrollable = tester.getCenter(find.text('SingleChildScrollView 3'));
    final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
    // Hover over the outer scroll view and create a pointer scroll.
    testPointer.hover(outerScrollable);
    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 20.0)));
    await tester.pump(const Duration(milliseconds: 250));
    expect(outerController.position.pixels, 20.0);
    expect(innerController.position.pixels, 0.0);

    final Offset innerScrollable = tester.getCenter(
      find.text('Nested NeverScrollable ListView 20'),
    );
    // Hover over the inner scroll view and create a pointer scroll.
    // This inner scroll view is not scrollable, and so the outer should scroll.
    testPointer.hover(innerScrollable);
    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, -20.0)));
    await tester.pump(const Duration(milliseconds: 250));
    expect(outerController.position.pixels, 0.0);
    expect(innerController.position.pixels, 0.0);
  });

  // Regression test for https://github.com/flutter/flutter/issues/71949
  testWidgets('Zero offset pointer scroll should not trigger an assertion.', (
    WidgetTester tester,
  ) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    Widget build(double height) {
      return MaterialApp(
        home: Scaffold(
          body: SizedBox(
            width: double.infinity,
            height: height,
            child: SingleChildScrollView(
              controller: controller,
              child: const SizedBox(width: double.infinity, height: 300.0),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(build(200.0));
    expect(controller.position.pixels, 0.0);

    controller.jumpTo(100.0);
    expect(controller.position.pixels, 100.0);

    // Make the outer constraints larger that the scrollable widget is no longer able to scroll.
    await tester.pumpWidget(build(300.0));
    expect(controller.position.pixels, 0.0);
    expect(controller.position.maxScrollExtent, 0.0);

    // Hover over the scroll view and create a zero offset pointer scroll.
    final Offset scrollable = tester.getCenter(find.byType(SingleChildScrollView));
    final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
    testPointer.hover(scrollable);
    await tester.sendEventToBinding(testPointer.scroll(Offset.zero));

    expect(tester.takeException(), null);
  });

  testWidgets(
    'Accepts drag with unknown device kind by default',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/90912.
      await tester.pumpWidget(
        const MaterialApp(
          home: CustomScrollView(
            slivers: <Widget>[SliverToBoxAdapter(child: SizedBox(height: 2000.0))],
          ),
        ),
      );
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byType(Scrollable), warnIfMissed: true),
        kind: ui.PointerDeviceKind.unknown,
      );
      expect(getScrollOffset(tester), 0.0);
      await gesture.moveBy(const Offset(0.0, -200));
      await tester.pump();
      await tester.pumpAndSettle();

      expect(getScrollOffset(tester), 200);

      await gesture.moveBy(const Offset(0.0, 200));
      await tester.pump();
      await tester.pumpAndSettle();

      expect(getScrollOffset(tester), 0.0);

      await gesture.removePointer();
      await tester.pump();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
      TargetPlatform.android,
    }),
  );

  testWidgets(
    'Does not scroll with mouse pointer drag when behavior is configured to ignore them',
    (WidgetTester tester) async {
      await pumpTest(tester, debugDefaultTargetPlatformOverride, enableMouseDrag: false);
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byType(Scrollable), warnIfMissed: true),
        kind: ui.PointerDeviceKind.mouse,
      );

      await gesture.moveBy(const Offset(0.0, -200));
      await tester.pump();
      await tester.pumpAndSettle();

      expect(getScrollOffset(tester), 0.0);

      await gesture.moveBy(const Offset(0.0, 200));
      await tester.pump();
      await tester.pumpAndSettle();

      expect(getScrollOffset(tester), 0.0);

      await gesture.removePointer();
      await tester.pump();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
      TargetPlatform.android,
    }),
  );

  testWidgets("Support updating 'ScrollBehavior.dragDevices' at runtime", (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/111716
    Widget buildFrame(Set<ui.PointerDeviceKind>? dragDevices) {
      return MaterialApp(
        scrollBehavior: const NoScrollbarBehavior().copyWith(dragDevices: dragDevices),
        home: ListView.builder(
          itemCount: 1000,
          itemBuilder: (BuildContext context, int index) {
            return Text('Item $index');
          },
        ),
      );
    }

    await tester.pumpWidget(buildFrame(<ui.PointerDeviceKind>{ui.PointerDeviceKind.mouse}));
    await tester.drag(
      find.byType(Scrollable),
      const Offset(0.0, -100.0),
      kind: ui.PointerDeviceKind.mouse,
    );

    // Matching device should allow user scrolling.
    expect(getScrollOffset(tester), 100.0);

    await tester.pumpWidget(buildFrame(<ui.PointerDeviceKind>{ui.PointerDeviceKind.stylus}));
    await tester.drag(
      find.byType(Scrollable),
      const Offset(0.0, -100.0),
      kind: ui.PointerDeviceKind.mouse,
    );

    // Non-matching device should not allow user scrolling.
    expect(getScrollOffset(tester), 100.0);

    await tester.drag(
      find.byType(Scrollable),
      const Offset(0.0, -100.0),
      kind: ui.PointerDeviceKind.stylus,
    );

    // Matching device should allow user scrolling.
    expect(getScrollOffset(tester), 200.0);
  });

  testWidgets(
    'Does scroll with mouse pointer drag when behavior is not configured to ignore them',
    (WidgetTester tester) async {
      await pumpTest(tester, debugDefaultTargetPlatformOverride);
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byType(Scrollable), warnIfMissed: true),
        kind: ui.PointerDeviceKind.mouse,
      );

      await gesture.moveBy(const Offset(0.0, -200));
      await tester.pump();
      await tester.pumpAndSettle();

      expect(getScrollOffset(tester), 200.0);

      await gesture.moveBy(const Offset(0.0, 200));
      await tester.pump();
      await tester.pumpAndSettle();

      expect(getScrollOffset(tester), 0.0);

      await gesture.removePointer();
      await tester.pump();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
      TargetPlatform.android,
    }),
  );

  testWidgets('Updated content dimensions correctly reflect in semantics', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/40419.
    final SemanticsHandle handle = tester.ensureSemantics();
    final UniqueKey listView = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        home: TickerMode(
          enabled: true,
          child: ListView.builder(
            key: listView,
            itemCount: 100,
            itemBuilder: (BuildContext context, int index) {
              return Text('Item $index');
            },
          ),
        ),
      ),
    );

    SemanticsNode scrollableNode = tester.getSemantics(
      find.descendant(of: find.byKey(listView), matching: find.byType(RawGestureDetector)),
    );
    SemanticsNode? syntheticScrollableNode;
    scrollableNode.visitChildren((SemanticsNode node) {
      syntheticScrollableNode = node;
      return true;
    });
    expect(syntheticScrollableNode!.hasFlag(ui.SemanticsFlag.hasImplicitScrolling), isTrue);
    // Disabled the ticker mode to trigger didChangeDependencies on Scrollable.
    // This can happen when a route is push or pop from top.
    // It will reconstruct the scroll position and apply content dimensions.
    await tester.pumpWidget(
      MaterialApp(
        home: TickerMode(
          enabled: false,
          child: ListView.builder(
            key: listView,
            itemCount: 100,
            itemBuilder: (BuildContext context, int index) {
              return Text('Item $index');
            },
          ),
        ),
      ),
    );
    await tester.pump();
    // The correct workflow will be the following:
    // 1. _RenderScrollSemantics receives a new scroll position without content
    //    dimensions and creates a SemanticsNode without implicit scroll.
    // 2. The content dimensions are applied to the scroll position during the
    //    layout phase, and the scroll position marks the semantics node of
    //    _RenderScrollSemantics dirty.
    // 3. The _RenderScrollSemantics rebuilds its semantics node with implicit
    //    scroll.
    scrollableNode = tester.getSemantics(
      find.descendant(of: find.byKey(listView), matching: find.byType(RawGestureDetector)),
    );
    syntheticScrollableNode = null;
    scrollableNode.visitChildren((SemanticsNode node) {
      syntheticScrollableNode = node;
      return true;
    });
    expect(syntheticScrollableNode!.hasFlag(ui.SemanticsFlag.hasImplicitScrolling), isTrue);
    handle.dispose();
  });

  testWidgets('Two panel semantics is added to the sibling nodes of direct children', (
    WidgetTester tester,
  ) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    final UniqueKey key = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: ListView(
            key: key,
            children: const <Widget>[
              TextField(autofocus: true, decoration: InputDecoration(prefixText: 'prefix')),
            ],
          ),
        ),
      ),
    );
    // Wait for focus.
    await tester.pumpAndSettle();

    final SemanticsNode scrollableNode = tester.getSemantics(find.byKey(key));
    SemanticsNode? intermediateNode;
    scrollableNode.visitChildren((SemanticsNode node) {
      intermediateNode = node;
      return true;
    });
    SemanticsNode? syntheticScrollableNode;
    intermediateNode!.visitChildren((SemanticsNode node) {
      syntheticScrollableNode = node;
      return true;
    });
    expect(syntheticScrollableNode!.hasFlag(ui.SemanticsFlag.hasImplicitScrolling), isTrue);

    int numberOfChild = 0;
    syntheticScrollableNode!.visitChildren((SemanticsNode node) {
      expect(node.isTagged(RenderViewport.useTwoPaneSemantics), isTrue);
      numberOfChild += 1;
      return true;
    });
    expect(numberOfChild, 2);

    handle.dispose();
  });

  testWidgets('Scroll inertia cancel event', (WidgetTester tester) async {
    await pumpTest(tester, null);
    await tester.fling(find.byType(Scrollable), const Offset(0.0, -dragOffset), 1000.0);
    expect(getScrollOffset(tester), dragOffset);
    await tester.pump(); // trigger fling
    expect(getScrollOffset(tester), dragOffset);
    await tester.pump(const Duration(milliseconds: 200));
    final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
    await tester.sendEventToBinding(testPointer.hover(tester.getCenter(find.byType(Scrollable))));
    await tester.sendEventToBinding(testPointer.scrollInertiaCancel()); // Cancel partway through.
    await tester.pump();
    expect(getScrollOffset(tester), closeTo(344.0642, 0.0001));
    await tester.pump(const Duration(milliseconds: 4800));
    expect(getScrollOffset(tester), closeTo(344.0642, 0.0001));
  });

  testWidgets('Swapping viewports in a scrollable does not crash', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final GlobalKey key = GlobalKey();
    final GlobalKey key1 = GlobalKey();
    Widget buildScrollable(bool withViewPort) {
      return Scrollable(
        key: key,
        viewportBuilder: (BuildContext context, ViewportOffset position) {
          if (withViewPort) {
            final ViewportOffset offset = ViewportOffset.zero();
            addTearDown(() => offset.dispose());
            return Viewport(
              slivers: <Widget>[
                SliverToBoxAdapter(
                  child: Semantics(key: key1, container: true, child: const Text('text1')),
                ),
              ],
              offset: offset,
            );
          }
          return Semantics(key: key1, container: true, child: const Text('text1'));
        },
      );
    }

    // This should cache the inner node in Scrollable with the children text1.
    await tester.pumpWidget(MaterialApp(home: buildScrollable(true)));
    expect(semantics, includesNodeWith(tags: <SemanticsTag>{RenderViewport.useTwoPaneSemantics}));
    // This does not use two panel, this should clear cached inner node.
    await tester.pumpWidget(MaterialApp(home: buildScrollable(false)));
    expect(
      semantics,
      isNot(includesNodeWith(tags: <SemanticsTag>{RenderViewport.useTwoPaneSemantics})),
    );
    // If the inner node was cleared in the previous step, this should not crash.
    await tester.pumpWidget(MaterialApp(home: buildScrollable(true)));
    expect(semantics, includesNodeWith(tags: <SemanticsTag>{RenderViewport.useTwoPaneSemantics}));
    expect(tester.takeException(), isNull);
    semantics.dispose();
  });

  testWidgets('deltaToScrollOrigin getter', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: CustomScrollView(
          slivers: <Widget>[SliverToBoxAdapter(child: SizedBox(height: 2000.0))],
        ),
      ),
    );
    final TestGesture gesture = await tester.startGesture(
      tester.getCenter(find.byType(Scrollable), warnIfMissed: true),
      kind: ui.PointerDeviceKind.unknown,
    );
    expect(getScrollOffset(tester), 0.0);
    await gesture.moveBy(const Offset(0.0, -200));
    await tester.pump();
    await tester.pumpAndSettle();

    expect(getScrollOffset(tester), 200);
    final ScrollableState scrollable = tester.state(find.byType(Scrollable));
    expect(scrollable.deltaToScrollOrigin, const Offset(0.0, 200));
  });

  testWidgets('resolvedPhysics getter', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.light().copyWith(platform: TargetPlatform.android),
        home: const CustomScrollView(
          physics: AlwaysScrollableScrollPhysics(),
          slivers: <Widget>[SliverToBoxAdapter(child: SizedBox(height: 2000.0))],
        ),
      ),
    );
    final TestGesture gesture = await tester.startGesture(
      tester.getCenter(find.byType(Scrollable), warnIfMissed: true),
      kind: ui.PointerDeviceKind.unknown,
    );
    expect(getScrollOffset(tester), 0.0);
    await gesture.moveBy(const Offset(0.0, -200));
    await tester.pump();
    await tester.pumpAndSettle();

    expect(getScrollOffset(tester), 200);
    final ScrollableState scrollable = tester.state(find.byType(Scrollable));
    String types(ScrollPhysics? value) =>
        value!.parent == null
            ? '${value.runtimeType}'
            : '${value.runtimeType} ${types(value.parent)}';

    expect(
      types(scrollable.resolvedPhysics),
      'AlwaysScrollableScrollPhysics ClampingScrollPhysics RangeMaintainingScrollPhysics',
    );
  });

  testWidgets('dragDevices change updates widget', (WidgetTester tester) async {
    bool enable = false;

    await tester.pumpWidget(
      Builder(
        builder: (BuildContext context) {
          return StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return MaterialApp(
                home: Scaffold(
                  body: Scrollable(
                    scrollBehavior: const MaterialScrollBehavior().copyWith(
                      dragDevices: <ui.PointerDeviceKind>{if (enable) ui.PointerDeviceKind.mouse},
                    ),
                    viewportBuilder:
                        (BuildContext context, ViewportOffset position) => Viewport(
                          offset: position,
                          slivers: const <Widget>[
                            SliverToBoxAdapter(child: SizedBox(height: 2000.0)),
                          ],
                        ),
                  ),
                  floatingActionButton: FloatingActionButton(
                    onPressed: () {
                      setState(() {
                        enable = !enable;
                      });
                    },
                  ),
                ),
              );
            },
          );
        },
      ),
    );

    // Gesture should not work.
    TestGesture gesture = await tester.startGesture(
      tester.getCenter(find.byType(Scrollable), warnIfMissed: true),
      kind: ui.PointerDeviceKind.mouse,
    );
    expect(getScrollOffset(tester), 0.0);
    await gesture.moveBy(const Offset(0.0, -200));
    await tester.pumpAndSettle();
    expect(getScrollOffset(tester), 0.0);

    // Change state to include mouse pointer device.
    await tester.tap(find.byType(FloatingActionButton));
    await tester.pump();

    // Gesture should work after state change.
    gesture = await tester.startGesture(
      tester.getCenter(find.byType(Scrollable), warnIfMissed: true),
      kind: ui.PointerDeviceKind.mouse,
    );
    expect(getScrollOffset(tester), 0.0);
    await gesture.moveBy(const Offset(0.0, -200));
    await tester.pumpAndSettle();
    expect(getScrollOffset(tester), 200);
  });

  testWidgets('dragDevices change updates widget when oldWidget scrollBehavior is null', (
    WidgetTester tester,
  ) async {
    ScrollBehavior? scrollBehavior;

    await tester.pumpWidget(
      Builder(
        builder: (BuildContext context) {
          return StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return MaterialApp(
                home: Scaffold(
                  body: Scrollable(
                    physics: const ScrollPhysics(),
                    scrollBehavior: scrollBehavior,
                    viewportBuilder:
                        (BuildContext context, ViewportOffset position) => Viewport(
                          offset: position,
                          slivers: const <Widget>[
                            SliverToBoxAdapter(child: SizedBox(height: 2000.0)),
                          ],
                        ),
                  ),
                  floatingActionButton: FloatingActionButton(
                    onPressed: () {
                      setState(() {
                        scrollBehavior = const MaterialScrollBehavior().copyWith(
                          dragDevices: <ui.PointerDeviceKind>{ui.PointerDeviceKind.mouse},
                        );
                      });
                    },
                  ),
                ),
              );
            },
          );
        },
      ),
    );

    // Gesture should not work.
    TestGesture gesture = await tester.startGesture(
      tester.getCenter(find.byType(Scrollable), warnIfMissed: true),
      kind: ui.PointerDeviceKind.mouse,
    );
    expect(getScrollOffset(tester), 0.0);
    await gesture.moveBy(const Offset(0.0, -200));
    await tester.pumpAndSettle();
    expect(getScrollOffset(tester), 0.0);

    // Change state to include mouse pointer device.
    await tester.tap(find.byType(FloatingActionButton));
    await tester.pump();

    // Gesture should work after state change.
    gesture = await tester.startGesture(
      tester.getCenter(find.byType(Scrollable), warnIfMissed: true),
      kind: ui.PointerDeviceKind.mouse,
    );
    expect(getScrollOffset(tester), 0.0);
    await gesture.moveBy(const Offset(0.0, -200));
    await tester.pumpAndSettle();
    expect(getScrollOffset(tester), 200);
  });
}

// ignore: must_be_immutable
class SuperPessimisticScrollPhysics extends ScrollPhysics {
  SuperPessimisticScrollPhysics({super.parent});

  int count = 0;

  @override
  bool recommendDeferredLoading(double velocity, ScrollMetrics metrics, BuildContext context) {
    count++;
    return velocity > 1;
  }

  @override
  ScrollPhysics applyTo(ScrollPhysics? ancestor) {
    return SuperPessimisticScrollPhysics(parent: buildParent(ancestor));
  }
}

class ExtraSuperPessimisticScrollPhysics extends ScrollPhysics {
  const ExtraSuperPessimisticScrollPhysics({super.parent});

  @override
  bool recommendDeferredLoading(double velocity, ScrollMetrics metrics, BuildContext context) {
    return true;
  }

  @override
  ScrollPhysics applyTo(ScrollPhysics? ancestor) {
    return ExtraSuperPessimisticScrollPhysics(parent: buildParent(ancestor));
  }
}

class TestTickerProvider extends TickerProvider {
  @override
  Ticker createTicker(TickerCallback onTick) {
    return Ticker(onTick);
  }
}
