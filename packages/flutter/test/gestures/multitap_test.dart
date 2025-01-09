// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';

import 'gesture_tester.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  testGesture('Should recognize pan', (GestureTester tester) {
    final MultiTapGestureRecognizer tap = MultiTapGestureRecognizer(
      longTapDelay: kLongPressTimeout,
    );

    final List<String> log = <String>[];

    tap.onTapDown = (int pointer, TapDownDetails details) {
      log.add('tap-down $pointer');
    };
    tap.onTapUp = (int pointer, TapUpDetails details) {
      log.add('tap-up $pointer');
    };
    tap.onTap = (int pointer) {
      log.add('tap $pointer');
    };
    tap.onLongTapDown = (int pointer, TapDownDetails details) {
      log.add('long-tap-down $pointer');
    };
    tap.onTapCancel = (int pointer) {
      log.add('tap-cancel $pointer');
    };

    final TestPointer pointer5 = TestPointer(5);
    final PointerDownEvent down5 = pointer5.down(const Offset(10.0, 10.0));
    tap.addPointer(down5);
    tester.closeArena(5);
    expect(log, <String>['tap-down 5']);
    log.clear();
    tester.route(down5);
    expect(log, isEmpty);

    final TestPointer pointer6 = TestPointer(6);
    final PointerDownEvent down6 = pointer6.down(const Offset(15.0, 15.0));
    tap.addPointer(down6);
    tester.closeArena(6);
    expect(log, <String>['tap-down 6']);
    log.clear();
    tester.route(down6);
    expect(log, isEmpty);

    tester.route(pointer5.move(const Offset(11.0, 12.0)));
    expect(log, isEmpty);

    tester.route(pointer6.move(const Offset(14.0, 13.0)));
    expect(log, isEmpty);

    tester.route(pointer5.up());
    expect(log, <String>['tap-up 5', 'tap 5']);
    log.clear();

    tester.async.elapse(kLongPressTimeout + kPressTimeout);
    expect(log, <String>['long-tap-down 6']);
    log.clear();

    tester.route(
      pointer6.move(const Offset(40.0, 30.0)),
    ); // move more than kTouchSlop from 15.0,15.0
    expect(log, <String>['tap-cancel 6']);
    log.clear();

    tester.route(pointer6.up());
    expect(log, isEmpty);

    tap.dispose();
  });

  testGesture('Can filter based on device kind', (GestureTester tester) {
    final MultiTapGestureRecognizer tap = MultiTapGestureRecognizer(
      longTapDelay: kLongPressTimeout,
      supportedDevices: <PointerDeviceKind>{PointerDeviceKind.touch},
    );

    final List<String> log = <String>[];

    tap.onTapDown = (int pointer, TapDownDetails details) {
      log.add('tap-down $pointer');
    };
    tap.onTapUp = (int pointer, TapUpDetails details) {
      log.add('tap-up $pointer');
    };
    tap.onTap = (int pointer) {
      log.add('tap $pointer');
    };
    tap.onLongTapDown = (int pointer, TapDownDetails details) {
      log.add('long-tap-down $pointer');
    };
    tap.onTapCancel = (int pointer) {
      log.add('tap-cancel $pointer');
    };

    final TestPointer touchPointer5 = TestPointer(5);
    final PointerDownEvent down5 = touchPointer5.down(const Offset(10.0, 10.0));
    tap.addPointer(down5);
    tester.closeArena(5);
    expect(log, <String>['tap-down 5']);
    log.clear();
    tester.route(down5);
    expect(log, isEmpty);

    final TestPointer mousePointer6 = TestPointer(6, PointerDeviceKind.mouse);
    final PointerDownEvent down6 = mousePointer6.down(const Offset(20.0, 20.0));
    tap.addPointer(down6);
    tester.closeArena(6);
    // Mouse down should be ignored by the recognizer.
    expect(log, isEmpty);

    final TestPointer touchPointer7 = TestPointer(7);
    final PointerDownEvent down7 = touchPointer7.down(const Offset(15.0, 15.0));
    tap.addPointer(down7);
    tester.closeArena(7);
    expect(log, <String>['tap-down 7']);
    log.clear();
    tester.route(down7);
    expect(log, isEmpty);

    tester.route(touchPointer5.move(const Offset(11.0, 12.0)));
    expect(log, isEmpty);

    // Move within the [kTouchSlop] range.
    tester.route(mousePointer6.move(const Offset(21.0, 18.0)));
    // Move beyond the slop range.
    tester.route(mousePointer6.move(const Offset(50.0, 40.0)));
    // Neither triggers any event because they originate from a mouse.
    expect(log, isEmpty);

    tester.route(touchPointer7.move(const Offset(14.0, 13.0)));
    expect(log, isEmpty);

    tester.route(touchPointer5.up());
    expect(log, <String>['tap-up 5', 'tap 5']);
    log.clear();

    // Mouse up should be ignored.
    tester.route(mousePointer6.up());
    expect(log, isEmpty);

    tester.async.elapse(kLongPressTimeout + kPressTimeout);
    // Only the touch pointer (7) triggers a long-tap, not the mouse pointer (6).
    expect(log, <String>['long-tap-down 7']);
    log.clear();

    tester.route(
      touchPointer7.move(const Offset(40.0, 30.0)),
    ); // move more than kTouchSlop from 15.0,15.0
    expect(log, <String>['tap-cancel 7']);
    log.clear();

    tap.dispose();
  });
}
