// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import 'gesture_tester.dart';

class TestDrag extends Drag {}

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  testGesture('MultiDrag: moving before delay rejects', (GestureTester tester) {
    final DelayedMultiDragGestureRecognizer drag = DelayedMultiDragGestureRecognizer();

    bool didStartDrag = false;
    drag.onStart = (Offset position) {
      didStartDrag = true;
      return TestDrag();
    };

    final TestPointer pointer = TestPointer(5);
    final PointerDownEvent down = pointer.down(const Offset(10.0, 10.0));
    drag.addPointer(down);
    tester.closeArena(5);
    expect(didStartDrag, isFalse);
    tester.async.flushMicrotasks();
    expect(didStartDrag, isFalse);
    tester.route(
      pointer.move(const Offset(20.0, 60.0)),
    ); // move more than touch slop before delay expires
    expect(didStartDrag, isFalse);
    tester.async.elapse(kLongPressTimeout * 2); // expire delay
    expect(didStartDrag, isFalse);
    tester.route(pointer.move(const Offset(30.0, 120.0))); // move some more after delay expires
    expect(didStartDrag, isFalse);
    drag.dispose();
  });

  testGesture('MultiDrag: delay triggers', (GestureTester tester) {
    final DelayedMultiDragGestureRecognizer drag = DelayedMultiDragGestureRecognizer();

    bool didStartDrag = false;
    drag.onStart = (Offset position) {
      didStartDrag = true;
      return TestDrag();
    };

    final TestPointer pointer = TestPointer(5);
    final PointerDownEvent down = pointer.down(const Offset(10.0, 10.0));
    drag.addPointer(down);
    tester.closeArena(5);
    expect(didStartDrag, isFalse);
    tester.async.flushMicrotasks();
    expect(didStartDrag, isFalse);
    tester.route(
      pointer.move(const Offset(20.0, 20.0)),
    ); // move less than touch slop before delay expires
    expect(didStartDrag, isFalse);
    tester.async.elapse(kLongPressTimeout * 2); // expire delay
    expect(didStartDrag, isTrue);
    tester.route(
      pointer.move(const Offset(30.0, 70.0)),
    ); // move more than touch slop after delay expires
    expect(didStartDrag, isTrue);
    drag.dispose();
  });

  testGesture('MultiDrag: can filter based on device kind', (GestureTester tester) {
    final DelayedMultiDragGestureRecognizer drag = DelayedMultiDragGestureRecognizer(
      supportedDevices: <PointerDeviceKind>{PointerDeviceKind.touch},
    );

    bool didStartDrag = false;
    drag.onStart = (Offset position) {
      didStartDrag = true;
      return TestDrag();
    };

    final TestPointer mousePointer = TestPointer(5, PointerDeviceKind.mouse);
    final PointerDownEvent down = mousePointer.down(const Offset(10.0, 10.0));
    drag.addPointer(down);
    tester.closeArena(5);
    expect(didStartDrag, isFalse);
    tester.async.flushMicrotasks();
    expect(didStartDrag, isFalse);
    tester.route(
      mousePointer.move(const Offset(20.0, 20.0)),
    ); // move less than touch slop before delay expires
    expect(didStartDrag, isFalse);
    tester.async.elapse(kLongPressTimeout * 2); // expire delay
    // Still false because it shouldn't recognize mouse events.
    expect(didStartDrag, isFalse);
    tester.route(
      mousePointer.move(const Offset(30.0, 70.0)),
    ); // move more than touch slop after delay expires
    // And still false.
    expect(didStartDrag, isFalse);
    drag.dispose();
  });

  test('allowedButtonsFilter should work the same when null or not specified', () {
    // Regression test for https://github.com/flutter/flutter/pull/122227

    final ImmediateMultiDragGestureRecognizer recognizer1 = ImmediateMultiDragGestureRecognizer();
    final ImmediateMultiDragGestureRecognizer recognizer2 = ImmediateMultiDragGestureRecognizer(
      allowedButtonsFilter: null, // ignore: avoid_redundant_argument_values
    );

    // We want to test _allowedButtonsFilter, which is called in this method.
    const PointerDownEvent allowedPointer = PointerDownEvent(timeStamp: Duration(days: 10));
    // ignore: invalid_use_of_protected_member
    expect(recognizer1.isPointerAllowed(allowedPointer), true);
    // ignore: invalid_use_of_protected_member
    expect(recognizer2.isPointerAllowed(allowedPointer), true);

    const PointerDownEvent rejectedPointer = PointerDownEvent(
      timeStamp: Duration(days: 10),
      buttons: kMiddleMouseButton,
    );
    // ignore: invalid_use_of_protected_member
    expect(recognizer1.isPointerAllowed(rejectedPointer), false);
    // ignore: invalid_use_of_protected_member
    expect(recognizer2.isPointerAllowed(rejectedPointer), false);
  });

  test('$MultiDragPointerState dispatches memory events', () async {
    await expectLater(
      await memoryEvents(
        () => _MultiDragPointerState(Offset.zero, PointerDeviceKind.touch, null).dispose(),
        _MultiDragPointerState,
      ),
      areCreateAndDispose,
    );
  });
}

class _MultiDragPointerState extends MultiDragPointerState {
  _MultiDragPointerState(super.initialPosition, super.kind, super.gestureSettings);

  @override
  void accepted(GestureMultiDragStartCallback starter) {}

  @override
  void dispose() {
    super.dispose();
  }
}
