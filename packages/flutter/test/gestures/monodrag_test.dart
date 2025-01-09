// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'gesture_tester.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  test('acceptGesture tolerates a null lastPendingEventTimestamp', () {
    // Regression test for https://github.com/flutter/flutter/issues/112403
    // and b/249091367
    final DragGestureRecognizer recognizer = VerticalDragGestureRecognizer();
    const PointerDownEvent event = PointerDownEvent(timeStamp: Duration(days: 10));

    expect(recognizer.debugLastPendingEventTimestamp, null);

    recognizer.addAllowedPointer(event);
    expect(recognizer.debugLastPendingEventTimestamp, event.timeStamp);

    // Normal case: acceptGesture called and we have a last timestamp set.
    recognizer.acceptGesture(event.pointer);
    expect(recognizer.debugLastPendingEventTimestamp, null);

    // Reject the gesture to reset state and allow accepting it again.
    recognizer.rejectGesture(event.pointer);
    expect(recognizer.debugLastPendingEventTimestamp, null);

    // Not entirely clear how this can happen, but the bugs mentioned above show
    // we can end up in this state empirically.
    recognizer.acceptGesture(event.pointer);
    expect(recognizer.debugLastPendingEventTimestamp, null);
  });

  testGesture(
    'do not crash on up event for a pending pointer after winning arena for another pointer',
    (GestureTester tester) {
      // Regression test for https://github.com/flutter/flutter/issues/75061.

      final VerticalDragGestureRecognizer v = VerticalDragGestureRecognizer()..onStart = (_) {};
      addTearDown(v.dispose);
      final HorizontalDragGestureRecognizer h = HorizontalDragGestureRecognizer()..onStart = (_) {};
      addTearDown(h.dispose);

      const PointerDownEvent down90 = PointerDownEvent(pointer: 90, position: Offset(10.0, 10.0));

      const PointerUpEvent up90 = PointerUpEvent(pointer: 90, position: Offset(10.0, 10.0));

      const PointerDownEvent down91 = PointerDownEvent(pointer: 91, position: Offset(20.0, 20.0));

      const PointerUpEvent up91 = PointerUpEvent(pointer: 91, position: Offset(20.0, 20.0));

      v.addPointer(down90);
      GestureBinding.instance.gestureArena.close(90);
      h.addPointer(down91);
      v.addPointer(down91);
      GestureBinding.instance.gestureArena.close(91);
      tester.async.flushMicrotasks();

      GestureBinding.instance.handleEvent(up90, HitTestEntry(MockHitTestTarget()));
      GestureBinding.instance.handleEvent(up91, HitTestEntry(MockHitTestTarget()));
    },
  );

  testGesture(
    'DragGestureRecognizer should not dispatch drag callbacks when it wins the arena if onlyAcceptDragOnThreshold is true and the threshold has not been met',
    (GestureTester tester) {
      final VerticalDragGestureRecognizer verticalDrag = VerticalDragGestureRecognizer();
      final List<String> dragCallbacks = <String>[];
      verticalDrag
        ..onlyAcceptDragOnThreshold = true
        ..onStart = (DragStartDetails details) {
          dragCallbacks.add('onStart');
        }
        ..onUpdate = (DragUpdateDetails details) {
          dragCallbacks.add('onUpdate');
        }
        ..onEnd = (DragEndDetails details) {
          dragCallbacks.add('onEnd');
        };

      const PointerDownEvent down1 = PointerDownEvent(pointer: 6, position: Offset(10.0, 10.0));

      const PointerUpEvent up1 = PointerUpEvent(pointer: 6, position: Offset(10.0, 10.0));

      verticalDrag.addPointer(down1);
      tester.closeArena(down1.pointer);
      tester.route(down1);
      tester.route(up1);
      expect(dragCallbacks.isEmpty, true);
      verticalDrag.dispose();
      dragCallbacks.clear();
    },
  );

  testGesture(
    'DragGestureRecognizer should dispatch drag callbacks when it wins the arena if onlyAcceptDragOnThreshold is false and the threshold has not been met',
    (GestureTester tester) {
      final VerticalDragGestureRecognizer verticalDrag = VerticalDragGestureRecognizer();
      final List<String> dragCallbacks = <String>[];
      verticalDrag
        ..onlyAcceptDragOnThreshold = false
        ..onStart = (DragStartDetails details) {
          dragCallbacks.add('onStart');
        }
        ..onUpdate = (DragUpdateDetails details) {
          dragCallbacks.add('onUpdate');
        }
        ..onEnd = (DragEndDetails details) {
          dragCallbacks.add('onEnd');
        };

      const PointerDownEvent down1 = PointerDownEvent(pointer: 6, position: Offset(10.0, 10.0));

      const PointerUpEvent up1 = PointerUpEvent(pointer: 6, position: Offset(10.0, 10.0));

      verticalDrag.addPointer(down1);
      tester.closeArena(down1.pointer);
      tester.route(down1);
      tester.route(up1);
      expect(dragCallbacks.isEmpty, false);
      expect(dragCallbacks, <String>['onStart', 'onEnd']);
      verticalDrag.dispose();
      dragCallbacks.clear();
    },
  );

  testWidgets('DragGestureRecognizer can be subclassed to beat a CustomScrollView in the arena', (
    WidgetTester tester,
  ) async {
    final GlobalKey tapTargetKey = GlobalKey();
    bool wasPanStartCalled = false;

    // Pump a tree with panable widget inside a CustomScrollView. The CustomScrollView
    // has a more aggressive drag recognizer that will typically beat other drag
    // recognizers in the arena. This pan recognizer uses a smaller threshold to
    // accept the gesture, that should make it win the arena.
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            slivers: <Widget>[
              SliverToBoxAdapter(
                child: RawGestureDetector(
                  behavior: HitTestBehavior.translucent,
                  gestures: <Type, GestureRecognizerFactory>{
                    _EagerPanGestureRecognizer: GestureRecognizerFactoryWithHandlers<
                      _EagerPanGestureRecognizer
                    >(() => _EagerPanGestureRecognizer(), (_EagerPanGestureRecognizer recognizer) {
                      recognizer.onStart = (DragStartDetails details) => wasPanStartCalled = true;
                    }),
                  },
                  child: SizedBox(key: tapTargetKey, width: 100, height: 100),
                ),
              ),
            ],
          ),
        ),
      ),
    );

    // Tap down on the tap target inside the gesture recognizer.
    final TestGesture gesture = await tester.startGesture(
      tester.getCenter(find.byKey(tapTargetKey)),
    );
    await tester.pump();

    // Move the pointer predominantly on the x-axis, with a y-axis movement that
    // is sufficient bigger so that both the CustomScrollScrollView and the
    // pan gesture recognizer want to accept the gesture.
    await gesture.moveBy(const Offset(30, kTouchSlop + 1));
    await tester.pump();

    // Ensure our gesture recognizer won the arena.
    expect(wasPanStartCalled, isTrue);
  });

  group('Recognizers on different button filters:', () {
    final List<String> recognized = <String>[];
    late HorizontalDragGestureRecognizer primaryRecognizer;
    late HorizontalDragGestureRecognizer secondaryRecognizer;
    setUp(() {
      primaryRecognizer = HorizontalDragGestureRecognizer(
          allowedButtonsFilter: (int buttons) => kPrimaryButton == buttons,
        )
        ..onStart = (DragStartDetails details) {
          recognized.add('onStartPrimary');
        };
      secondaryRecognizer = HorizontalDragGestureRecognizer(
          allowedButtonsFilter: (int buttons) => kSecondaryButton == buttons,
        )
        ..onStart = (DragStartDetails details) {
          recognized.add('onStartSecondary');
        };
    });

    tearDown(() {
      recognized.clear();
      primaryRecognizer.dispose();
      secondaryRecognizer.dispose();
    });

    testGesture('Primary button works', (GestureTester tester) {
      const PointerDownEvent down1 = PointerDownEvent(pointer: 6, position: Offset(10.0, 10.0));

      primaryRecognizer.addPointer(down1);
      secondaryRecognizer.addPointer(down1);
      tester.closeArena(down1.pointer);
      tester.route(down1);
      expect(recognized, <String>['onStartPrimary']);
    });

    testGesture('Secondary button works', (GestureTester tester) {
      const PointerDownEvent down1 = PointerDownEvent(
        pointer: 6,
        position: Offset(10.0, 10.0),
        buttons: kSecondaryMouseButton,
      );

      primaryRecognizer.addPointer(down1);
      secondaryRecognizer.addPointer(down1);
      tester.closeArena(down1.pointer);
      tester.route(down1);
      expect(recognized, <String>['onStartSecondary']);
    });
  });
}

class MockHitTestTarget implements HitTestTarget {
  @override
  void handleEvent(PointerEvent event, HitTestEntry entry) {}
}

/// A [PanGestureRecognizer] that tries to beat [VerticalDragGestureRecognizer] in the arena.
///
/// Typically, [VerticalDragGestureRecognizer] wins because it has a smaller threshold to
/// accept the gesture. This recognizer uses the same threshold that [VerticalDragGestureRecognizer]
/// uses.
class _EagerPanGestureRecognizer extends PanGestureRecognizer {
  @override
  bool hasSufficientGlobalDistanceToAccept(
    PointerDeviceKind pointerDeviceKind,
    double? deviceTouchSlop,
  ) {
    return globalDistanceMoved.abs() > computeHitSlop(pointerDeviceKind, gestureSettings);
  }

  @override
  bool isFlingGesture(VelocityEstimate estimate, PointerDeviceKind kind) {
    final double minVelocity = minFlingVelocity ?? kMinFlingVelocity;
    final double minDistance = minFlingDistance ?? computeHitSlop(kind, gestureSettings);
    return estimate.pixelsPerSecond.distanceSquared > minVelocity * minVelocity &&
        estimate.offset.distanceSquared > minDistance * minDistance;
  }

  @override
  DragEndDetails? considerFling(VelocityEstimate estimate, PointerDeviceKind kind) {
    if (!isFlingGesture(estimate, kind)) {
      return null;
    }
    final double maxVelocity = maxFlingVelocity ?? kMaxFlingVelocity;
    final double dy = clampDouble(estimate.pixelsPerSecond.dy, -maxVelocity, maxVelocity);
    return DragEndDetails(
      velocity: Velocity(pixelsPerSecond: Offset(0, dy)),
      primaryVelocity: dy,
      globalPosition: lastPosition.global,
      localPosition: lastPosition.local,
    );
  }
}
