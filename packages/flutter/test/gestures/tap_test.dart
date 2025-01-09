// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';

import 'gesture_tester.dart';

class TestGestureArenaMember extends GestureArenaMember {
  @override
  void acceptGesture(int key) {}

  @override
  void rejectGesture(int key) {}
}

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  // Down/up pair 1: normal tap sequence
  const PointerDownEvent down1 = PointerDownEvent(pointer: 1, position: Offset(10.0, 10.0));

  const PointerUpEvent up1 = PointerUpEvent(pointer: 1, position: Offset(11.0, 9.0));

  // Down/up pair 2: normal tap sequence far away from pair 1
  const PointerDownEvent down2 = PointerDownEvent(pointer: 2, position: Offset(30.0, 30.0));

  const PointerUpEvent up2 = PointerUpEvent(pointer: 2, position: Offset(31.0, 29.0));

  // Down/move/up sequence 3: intervening motion, more than kTouchSlop. (~21px)
  const PointerDownEvent down3 = PointerDownEvent(pointer: 3, position: Offset(10.0, 10.0));

  const PointerMoveEvent move3 = PointerMoveEvent(pointer: 3, position: Offset(25.0, 25.0));

  const PointerUpEvent up3 = PointerUpEvent(pointer: 3, position: Offset(25.0, 25.0));

  // Down/move/up sequence 4: intervening motion, less than kTouchSlop. (~17px)
  const PointerDownEvent down4 = PointerDownEvent(pointer: 4, position: Offset(10.0, 10.0));

  const PointerMoveEvent move4 = PointerMoveEvent(pointer: 4, position: Offset(22.0, 22.0));

  const PointerUpEvent up4 = PointerUpEvent(pointer: 4, position: Offset(22.0, 22.0));

  // Down/up sequence 5: tap sequence with secondary button
  const PointerDownEvent down5 = PointerDownEvent(
    pointer: 5,
    position: Offset(20.0, 20.0),
    buttons: kSecondaryButton,
  );

  const PointerUpEvent up5 = PointerUpEvent(pointer: 5, position: Offset(20.0, 20.0));

  // Down/up sequence 6: tap sequence with tertiary button
  const PointerDownEvent down6 = PointerDownEvent(
    pointer: 6,
    position: Offset(20.0, 20.0),
    buttons: kTertiaryButton,
  );

  const PointerUpEvent up6 = PointerUpEvent(pointer: 6, position: Offset(20.0, 20.0));

  testGesture('Should recognize tap', (GestureTester tester) {
    final TapGestureRecognizer tap = TapGestureRecognizer();
    addTearDown(tap.dispose);

    bool tapRecognized = false;
    tap.onTap = () {
      tapRecognized = true;
    };

    tap.addPointer(down1);
    tester.closeArena(1);
    expect(tapRecognized, isFalse);
    tester.route(down1);
    expect(tapRecognized, isFalse);

    tester.route(up1);
    expect(tapRecognized, isTrue);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(tapRecognized, isTrue);

    tap.dispose();
  });

  testGesture('Should recognize tap for supported devices only', (GestureTester tester) {
    final TapGestureRecognizer tap = TapGestureRecognizer(
      supportedDevices: <PointerDeviceKind>{PointerDeviceKind.mouse, PointerDeviceKind.stylus},
    );

    bool tapRecognized = false;
    tap.onTap = () {
      tapRecognized = true;
    };
    const PointerDownEvent touchDown = PointerDownEvent(pointer: 1, position: Offset(10.0, 10.0));
    const PointerUpEvent touchUp = PointerUpEvent(pointer: 1, position: Offset(11.0, 9.0));

    tap.addPointer(touchDown);
    tester.closeArena(1);
    expect(tapRecognized, isFalse);
    tester.route(touchDown);
    expect(tapRecognized, isFalse);

    tester.route(touchUp);
    expect(tapRecognized, isFalse);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(tapRecognized, isFalse);

    const PointerDownEvent mouseDown = PointerDownEvent(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
      position: Offset(10.0, 10.0),
    );
    const PointerUpEvent mouseUp = PointerUpEvent(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
      position: Offset(11.0, 9.0),
    );

    tap.addPointer(mouseDown);
    tester.closeArena(1);
    expect(tapRecognized, isFalse);
    tester.route(mouseDown);
    expect(tapRecognized, isFalse);

    tester.route(mouseUp);
    expect(tapRecognized, isTrue);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(tapRecognized, isTrue);

    tapRecognized = false;

    const PointerDownEvent stylusDown = PointerDownEvent(
      kind: PointerDeviceKind.stylus,
      pointer: 1,
      position: Offset(10.0, 10.0),
    );
    const PointerUpEvent stylusUp = PointerUpEvent(
      kind: PointerDeviceKind.stylus,
      pointer: 1,
      position: Offset(11.0, 9.0),
    );

    tap.addPointer(stylusDown);
    tester.closeArena(1);
    expect(tapRecognized, isFalse);
    tester.route(stylusDown);
    expect(tapRecognized, isFalse);

    tester.route(stylusUp);
    expect(tapRecognized, isTrue);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(tapRecognized, isTrue);

    tap.dispose();
  });

  testGesture('Details contain the correct device kind', (GestureTester tester) {
    final TapGestureRecognizer tap = TapGestureRecognizer();

    TapDownDetails? lastDownDetails;
    TapUpDetails? lastUpDetails;

    tap.onTapDown = (TapDownDetails details) {
      lastDownDetails = details;
    };
    tap.onTapUp = (TapUpDetails details) {
      lastUpDetails = details;
    };

    const PointerDownEvent mouseDown = PointerDownEvent(pointer: 1, kind: PointerDeviceKind.mouse);
    const PointerUpEvent mouseUp = PointerUpEvent(pointer: 1, kind: PointerDeviceKind.mouse);

    tap.addPointer(mouseDown);
    tester.closeArena(1);
    tester.route(mouseDown);
    expect(lastDownDetails?.kind, PointerDeviceKind.mouse);

    tester.route(mouseUp);
    expect(lastUpDetails?.kind, PointerDeviceKind.mouse);

    tap.dispose();
  });

  testGesture('No duplicate tap events', (GestureTester tester) {
    final TapGestureRecognizer tap = TapGestureRecognizer();

    int tapsRecognized = 0;
    tap.onTap = () {
      tapsRecognized++;
    };

    tap.addPointer(down1);
    tester.closeArena(1);
    expect(tapsRecognized, 0);
    tester.route(down1);
    expect(tapsRecognized, 0);

    tester.route(up1);
    expect(tapsRecognized, 1);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(tapsRecognized, 1);

    tap.addPointer(down1);
    tester.closeArena(1);
    expect(tapsRecognized, 1);
    tester.route(down1);
    expect(tapsRecognized, 1);

    tester.route(up1);
    expect(tapsRecognized, 2);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(tapsRecognized, 2);

    tap.dispose();
  });

  testGesture('Should not recognize two overlapping taps (FIFO)', (GestureTester tester) {
    final TapGestureRecognizer tap = TapGestureRecognizer();

    int tapsRecognized = 0;
    tap.onTap = () {
      tapsRecognized++;
    };

    tap.addPointer(down1);
    tester.closeArena(1);
    expect(tapsRecognized, 0);
    tester.route(down1);
    expect(tapsRecognized, 0);

    tap.addPointer(down2);
    tester.closeArena(2);
    expect(tapsRecognized, 0);
    tester.route(down1);
    expect(tapsRecognized, 0);

    tester.route(up1);
    expect(tapsRecognized, 1);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(tapsRecognized, 1);

    tester.route(up2);
    expect(tapsRecognized, 1);
    GestureBinding.instance.gestureArena.sweep(2);
    expect(tapsRecognized, 1);

    tap.dispose();
  });

  testGesture('Should not recognize two overlapping taps (FILO)', (GestureTester tester) {
    final TapGestureRecognizer tap = TapGestureRecognizer();

    int tapsRecognized = 0;
    tap.onTap = () {
      tapsRecognized++;
    };

    tap.addPointer(down1);
    tester.closeArena(1);
    expect(tapsRecognized, 0);
    tester.route(down1);
    expect(tapsRecognized, 0);

    tap.addPointer(down2);
    tester.closeArena(2);
    expect(tapsRecognized, 0);
    tester.route(down1);
    expect(tapsRecognized, 0);

    tester.route(up2);
    expect(tapsRecognized, 0);
    GestureBinding.instance.gestureArena.sweep(2);
    expect(tapsRecognized, 0);

    tester.route(up1);
    expect(tapsRecognized, 1);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(tapsRecognized, 1);

    tap.dispose();
  });

  testGesture('Distance cancels tap', (GestureTester tester) {
    final TapGestureRecognizer tap = TapGestureRecognizer();

    bool tapRecognized = false;
    tap.onTap = () {
      tapRecognized = true;
    };
    bool tapCanceled = false;
    tap.onTapCancel = () {
      tapCanceled = true;
    };

    tap.addPointer(down3);
    tester.closeArena(3);
    expect(tapRecognized, isFalse);
    expect(tapCanceled, isFalse);
    tester.route(down3);
    expect(tapRecognized, isFalse);
    expect(tapCanceled, isFalse);

    tester.route(move3);
    expect(tapRecognized, isFalse);
    expect(tapCanceled, isTrue);
    tester.route(up3);
    expect(tapRecognized, isFalse);
    expect(tapCanceled, isTrue);
    GestureBinding.instance.gestureArena.sweep(3);
    expect(tapRecognized, isFalse);
    expect(tapCanceled, isTrue);

    tap.dispose();
  });

  testGesture('Short distance does not cancel tap', (GestureTester tester) {
    final TapGestureRecognizer tap = TapGestureRecognizer();

    bool tapRecognized = false;
    tap.onTap = () {
      tapRecognized = true;
    };
    bool tapCanceled = false;
    tap.onTapCancel = () {
      tapCanceled = true;
    };

    tap.addPointer(down4);
    tester.closeArena(4);
    expect(tapRecognized, isFalse);
    expect(tapCanceled, isFalse);
    tester.route(down4);
    expect(tapRecognized, isFalse);
    expect(tapCanceled, isFalse);

    tester.route(move4);
    expect(tapRecognized, isFalse);
    expect(tapCanceled, isFalse);
    tester.route(up4);
    expect(tapRecognized, isTrue);
    expect(tapCanceled, isFalse);
    GestureBinding.instance.gestureArena.sweep(4);
    expect(tapRecognized, isTrue);
    expect(tapCanceled, isFalse);

    tap.dispose();
  });

  testGesture('Timeout does not cancel tap', (GestureTester tester) {
    final TapGestureRecognizer tap = TapGestureRecognizer();

    bool tapRecognized = false;
    tap.onTap = () {
      tapRecognized = true;
    };

    tap.addPointer(down1);
    tester.closeArena(1);
    expect(tapRecognized, isFalse);
    tester.route(down1);
    expect(tapRecognized, isFalse);

    tester.async.elapse(const Duration(milliseconds: 500));
    expect(tapRecognized, isFalse);
    tester.route(up1);
    expect(tapRecognized, isTrue);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(tapRecognized, isTrue);

    tap.dispose();
  });

  testGesture('Should yield to other arena members', (GestureTester tester) {
    final TapGestureRecognizer tap = TapGestureRecognizer();

    bool tapRecognized = false;
    tap.onTap = () {
      tapRecognized = true;
    };

    tap.addPointer(down1);
    final TestGestureArenaMember member = TestGestureArenaMember();
    final GestureArenaEntry entry = GestureBinding.instance.gestureArena.add(1, member);
    GestureBinding.instance.gestureArena.hold(1);
    tester.closeArena(1);
    expect(tapRecognized, isFalse);
    tester.route(down1);
    expect(tapRecognized, isFalse);

    tester.route(up1);
    expect(tapRecognized, isFalse);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(tapRecognized, isFalse);

    entry.resolve(GestureDisposition.accepted);
    expect(tapRecognized, isFalse);

    tap.dispose();
  });

  testGesture('Should trigger on release of held arena', (GestureTester tester) {
    final TapGestureRecognizer tap = TapGestureRecognizer();

    bool tapRecognized = false;
    tap.onTap = () {
      tapRecognized = true;
    };

    tap.addPointer(down1);
    final TestGestureArenaMember member = TestGestureArenaMember();
    final GestureArenaEntry entry = GestureBinding.instance.gestureArena.add(1, member);
    GestureBinding.instance.gestureArena.hold(1);
    tester.closeArena(1);
    expect(tapRecognized, isFalse);
    tester.route(down1);
    expect(tapRecognized, isFalse);

    tester.route(up1);
    expect(tapRecognized, isFalse);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(tapRecognized, isFalse);

    entry.resolve(GestureDisposition.rejected);
    tester.async.flushMicrotasks();
    expect(tapRecognized, isTrue);

    tap.dispose();
  });

  testGesture('Should log exceptions from callbacks', (GestureTester tester) {
    final TapGestureRecognizer tap = TapGestureRecognizer();

    tap.onTap = () {
      throw Exception(test);
    };

    final FlutterExceptionHandler? previousErrorHandler = FlutterError.onError;
    bool gotError = false;
    FlutterError.onError = (FlutterErrorDetails details) {
      gotError = true;
    };

    tap.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);
    expect(gotError, isFalse);

    tester.route(up1);
    expect(gotError, isTrue);

    FlutterError.onError = previousErrorHandler;
    tap.dispose();
  });

  testGesture('onTapCancel should show reason in the proper format', (GestureTester tester) {
    final TapGestureRecognizer tap = TapGestureRecognizer();

    tap.onTapCancel = () {
      throw Exception(test);
    };

    final FlutterExceptionHandler? previousErrorHandler = FlutterError.onError;
    bool gotError = false;
    FlutterError.onError = (FlutterErrorDetails details) {
      expect(details.toString().contains('"spontaneous onTapCancel"'), isTrue);
      gotError = true;
    };

    const int pointer = 1;
    tap.addPointer(const PointerDownEvent(pointer: pointer));
    tester.closeArena(pointer);
    tester.async.elapse(const Duration(milliseconds: 500));
    tester.route(const PointerCancelEvent(pointer: pointer));

    expect(gotError, isTrue);

    FlutterError.onError = previousErrorHandler;
    tap.dispose();
  });

  testGesture('No duplicate tap events', (GestureTester tester) {
    final TapGestureRecognizer tapA = TapGestureRecognizer();
    final TapGestureRecognizer tapB = TapGestureRecognizer();

    final List<String> log = <String>[];
    tapA.onTapDown = (TapDownDetails details) {
      log.add('tapA onTapDown');
    };
    tapA.onTapUp = (TapUpDetails details) {
      log.add('tapA onTapUp');
    };
    tapA.onTap = () {
      log.add('tapA onTap');
    };
    tapA.onTapCancel = () {
      log.add('tapA onTapCancel');
    };
    tapB.onTapDown = (TapDownDetails details) {
      log.add('tapB onTapDown');
    };
    tapB.onTapUp = (TapUpDetails details) {
      log.add('tapB onTapUp');
    };
    tapB.onTap = () {
      log.add('tapB onTap');
    };
    tapB.onTapCancel = () {
      log.add('tapB onTapCancel');
    };

    log.add('start');
    tapA.addPointer(down1);
    log.add('added 1 to A');
    tapB.addPointer(down1);
    log.add('added 1 to B');
    tester.closeArena(1);
    log.add('closed 1');
    tester.route(down1);
    log.add('routed 1 down');
    tester.route(up1);
    log.add('routed 1 up');
    GestureBinding.instance.gestureArena.sweep(1);
    log.add('swept 1');
    tapA.addPointer(down2);
    log.add('down 2 to A');
    tapB.addPointer(down2);
    log.add('down 2 to B');
    tester.closeArena(2);
    log.add('closed 2');
    tester.route(down2);
    log.add('routed 2 down');
    tester.route(up2);
    log.add('routed 2 up');
    GestureBinding.instance.gestureArena.sweep(2);
    log.add('swept 2');
    tapA.dispose();
    log.add('disposed A');
    tapB.dispose();
    log.add('disposed B');

    expect(log, <String>[
      'start',
      'added 1 to A',
      'added 1 to B',
      'closed 1',
      'routed 1 down',
      'routed 1 up',
      'tapA onTapDown',
      'tapA onTapUp',
      'tapA onTap',
      'swept 1',
      'down 2 to A',
      'down 2 to B',
      'closed 2',
      'routed 2 down',
      'routed 2 up',
      'tapA onTapDown',
      'tapA onTapUp',
      'tapA onTap',
      'swept 2',
      'disposed A',
      'disposed B',
    ]);
  });

  testGesture('PointerCancelEvent cancels tap', (GestureTester tester) {
    const PointerDownEvent down = PointerDownEvent(pointer: 5, position: Offset(10.0, 10.0));
    const PointerCancelEvent cancel = PointerCancelEvent(pointer: 5, position: Offset(10.0, 10.0));

    final TapGestureRecognizer tap = TapGestureRecognizer();

    final List<String> recognized = <String>[];
    tap.onTapDown = (_) {
      recognized.add('down');
    };
    tap.onTapUp = (_) {
      recognized.add('up');
    };
    tap.onTap = () {
      recognized.add('tap');
    };
    tap.onTapCancel = () {
      recognized.add('cancel');
    };

    tap.addPointer(down);
    tester.closeArena(5);
    tester.async.elapse(const Duration(milliseconds: 5000));
    expect(recognized, <String>['down']);
    tester.route(cancel);
    expect(recognized, <String>['down', 'cancel']);

    tap.dispose();
  });

  testGesture('PointerCancelEvent after exceeding deadline cancels tap', (GestureTester tester) {
    const PointerDownEvent down = PointerDownEvent(pointer: 5, position: Offset(10.0, 10.0));
    const PointerCancelEvent cancel = PointerCancelEvent(pointer: 5, position: Offset(10.0, 10.0));

    final TapGestureRecognizer tap = TapGestureRecognizer();
    final HorizontalDragGestureRecognizer drag =
        HorizontalDragGestureRecognizer()..onStart = (_) {}; // Need a callback to compete
    addTearDown(drag.dispose);

    final List<String> recognized = <String>[];
    tap.onTapDown = (_) {
      recognized.add('down');
    };
    tap.onTapUp = (_) {
      recognized.add('up');
    };
    tap.onTap = () {
      recognized.add('tap');
    };
    tap.onTapCancel = () {
      recognized.add('cancel');
    };

    tap.addPointer(down);
    drag.addPointer(down);
    tester.closeArena(5);
    tester.route(down);
    expect(recognized, <String>[]);
    tester.async.elapse(const Duration(milliseconds: 1000));
    expect(recognized, <String>['down']);
    tester.route(cancel);
    expect(recognized, <String>['down', 'cancel']);

    tap.dispose();
    drag.dispose();
  });

  testGesture('losing tap gesture recognizer does not send onTapCancel', (GestureTester tester) {
    final TapGestureRecognizer tap = TapGestureRecognizer();
    addTearDown(tap.dispose);
    final HorizontalDragGestureRecognizer drag = HorizontalDragGestureRecognizer();
    addTearDown(drag.dispose);

    final List<String> recognized = <String>[];
    tap.onTapDown = (_) {
      recognized.add('down');
    };
    tap.onTapUp = (_) {
      recognized.add('up');
    };
    tap.onTap = () {
      recognized.add('tap');
    };
    tap.onTapCancel = () {
      recognized.add('cancel');
    };

    tap.addPointer(down3);
    drag.addPointer(down3);
    tester.closeArena(3);
    tester.route(move3);
    GestureBinding.instance.gestureArena.sweep(3);
    expect(recognized, isEmpty);

    tap.dispose();
    drag.dispose();
  });

  testGesture('non-primary pointers does not trigger timeout', (GestureTester tester) {
    // Regression test for https://github.com/flutter/flutter/issues/43310
    // Pointer1 down, pointer2 down, then pointer 1 up, all within the timeout.
    // In this way, `BaseTapGestureRecognizer.didExceedDeadline` can be triggered
    // after its `_reset`.
    final TapGestureRecognizer tap = TapGestureRecognizer();
    addTearDown(tap.dispose);

    final List<String> recognized = <String>[];
    tap.onTapDown = (_) {
      recognized.add('down');
    };
    tap.onTapUp = (_) {
      recognized.add('up');
    };
    tap.onTap = () {
      recognized.add('tap');
    };
    tap.onTapCancel = () {
      recognized.add('cancel');
    };

    tap.addPointer(down1);
    tester.closeArena(down1.pointer);

    tap.addPointer(down2);
    tester.closeArena(down2.pointer);

    expect(recognized, isEmpty);

    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(down1.pointer);
    expect(recognized, <String>['down', 'up', 'tap']);
    recognized.clear();

    // If regression happens, the following step will throw error
    tester.async.elapse(const Duration(milliseconds: 200));
    expect(recognized, isEmpty);

    tester.route(up2);
    GestureBinding.instance.gestureArena.sweep(down2.pointer);
    expect(recognized, isEmpty);

    tap.dispose();
  });

  group('Enforce consistent-button restriction:', () {
    // Change buttons during down-up sequence 1
    const PointerMoveEvent move1lr = PointerMoveEvent(
      pointer: 1,
      position: Offset(10.0, 10.0),
      buttons: kPrimaryMouseButton | kSecondaryMouseButton,
    );
    const PointerMoveEvent move1r = PointerMoveEvent(
      pointer: 1,
      position: Offset(10.0, 10.0),
      buttons: kSecondaryMouseButton,
    );

    final List<String> recognized = <String>[];
    late TapGestureRecognizer tap;
    setUp(() {
      tap =
          TapGestureRecognizer()
            ..onTapDown = (TapDownDetails details) {
              recognized.add('down');
            }
            ..onTapUp = (TapUpDetails details) {
              recognized.add('up');
            }
            ..onTapCancel = () {
              recognized.add('cancel');
            };
      addTearDown(tap.dispose);
    });

    tearDown(() {
      tap.dispose();
      recognized.clear();
    });

    testGesture('changing buttons before TapDown should cancel gesture without sending cancel', (
      GestureTester tester,
    ) {
      tap.addPointer(down1);
      tester.closeArena(1);
      expect(recognized, <String>[]);

      tester.route(move1lr);
      expect(recognized, <String>[]);

      tester.route(move1r);
      expect(recognized, <String>[]);

      tester.route(up1);
      expect(recognized, <String>[]);

      tap.dispose();
    });

    testGesture('changing buttons before TapDown should not prevent the next tap', (
      GestureTester tester,
    ) {
      tap.addPointer(down1);
      tester.closeArena(1);

      tester.route(move1lr);
      tester.route(move1r);
      tester.route(up1);
      expect(recognized, <String>[]);

      tap.addPointer(down2);
      tester.closeArena(2);
      tester.async.elapse(const Duration(milliseconds: 1000));
      tester.route(up2);
      expect(recognized, <String>['down', 'up']);

      tap.dispose();
    });

    testGesture('changing buttons after TapDown should cancel gesture and send cancel', (
      GestureTester tester,
    ) {
      tap.addPointer(down1);
      tester.closeArena(1);
      expect(recognized, <String>[]);
      tester.async.elapse(const Duration(milliseconds: 1000));
      expect(recognized, <String>['down']);

      tester.route(move1lr);
      expect(recognized, <String>['down', 'cancel']);

      tester.route(move1r);
      expect(recognized, <String>['down', 'cancel']);

      tester.route(up1);
      expect(recognized, <String>['down', 'cancel']);

      tap.dispose();
    });

    testGesture('changing buttons after TapDown should not prevent the next tap', (
      GestureTester tester,
    ) {
      tap.addPointer(down1);
      tester.closeArena(1);
      tester.async.elapse(const Duration(milliseconds: 1000));

      tester.route(move1lr);
      tester.route(move1r);
      tester.route(up1);
      GestureBinding.instance.gestureArena.sweep(1);
      expect(recognized, <String>['down', 'cancel']);

      tap.addPointer(down2);
      tester.closeArena(2);
      tester.async.elapse(const Duration(milliseconds: 1000));
      tester.route(up2);
      GestureBinding.instance.gestureArena.sweep(2);
      expect(recognized, <String>['down', 'cancel', 'down', 'up']);

      tap.dispose();
    });
  });

  group('Recognizers listening on different buttons do not form competition:', () {
    // If a tap gesture has no competitors, a pointer down event triggers
    // onTapDown immediately; if there are competitors, onTapDown is triggered
    // after a timeout. The following tests make sure that tap recognizers
    // listening on different buttons do not form competition.

    final List<String> recognized = <String>[];
    late TapGestureRecognizer primary;
    late TapGestureRecognizer primary2;
    late TapGestureRecognizer secondary;
    late TapGestureRecognizer tertiary;
    setUp(() {
      primary =
          TapGestureRecognizer()
            ..onTapDown = (TapDownDetails details) {
              recognized.add('primaryDown');
            }
            ..onTapUp = (TapUpDetails details) {
              recognized.add('primaryUp');
            }
            ..onTapCancel = () {
              recognized.add('primaryCancel');
            };
      addTearDown(primary.dispose);
      primary2 =
          TapGestureRecognizer()
            ..onTapDown = (TapDownDetails details) {
              recognized.add('primary2Down');
            }
            ..onTapUp = (TapUpDetails details) {
              recognized.add('primary2Up');
            }
            ..onTapCancel = () {
              recognized.add('primary2Cancel');
            };
      addTearDown(primary2.dispose);
      secondary =
          TapGestureRecognizer()
            ..onSecondaryTapDown = (TapDownDetails details) {
              recognized.add('secondaryDown');
            }
            ..onSecondaryTapUp = (TapUpDetails details) {
              recognized.add('secondaryUp');
            }
            ..onSecondaryTapCancel = () {
              recognized.add('secondaryCancel');
            };
      addTearDown(secondary.dispose);
      tertiary =
          TapGestureRecognizer()
            ..onTertiaryTapDown = (TapDownDetails details) {
              recognized.add('tertiaryDown');
            }
            ..onTertiaryTapUp = (TapUpDetails details) {
              recognized.add('tertiaryUp');
            }
            ..onTertiaryTapCancel = () {
              recognized.add('tertiaryCancel');
            };
      addTearDown(tertiary.dispose);
    });

    tearDown(() {
      primary.dispose();
      primary2.dispose();
      secondary.dispose();
      tertiary.dispose();
      recognized.clear();
    });

    testGesture(
      'A primary tap recognizer does not form competition with a secondary tap recognizer',
      (GestureTester tester) {
        primary.addPointer(down1);
        secondary.addPointer(down1);
        tester.closeArena(1);

        tester.route(down1);
        expect(recognized, <String>['primaryDown']);
        recognized.clear();

        tester.route(up1);
        expect(recognized, <String>['primaryUp']);
      },
    );

    testGesture(
      'A primary tap recognizer does not form competition with a tertiary tap recognizer',
      (GestureTester tester) {
        primary.addPointer(down1);
        tertiary.addPointer(down1);
        tester.closeArena(1);

        tester.route(down1);
        expect(recognized, <String>['primaryDown']);
        recognized.clear();

        tester.route(up1);
        expect(recognized, <String>['primaryUp']);
      },
    );

    testGesture('A primary tap recognizer forms competition with another primary tap recognizer', (
      GestureTester tester,
    ) {
      primary.addPointer(down1);
      primary2.addPointer(down1);
      tester.closeArena(1);

      tester.route(down1);
      expect(recognized, <String>[]);

      tester.async.elapse(const Duration(milliseconds: 500));
      expect(recognized, <String>['primaryDown', 'primary2Down']);
    });
  });

  group('Gestures of different buttons trigger correct callbacks:', () {
    final List<String> recognized = <String>[];
    late TapGestureRecognizer tap;
    const PointerCancelEvent cancel1 = PointerCancelEvent(pointer: 1);
    const PointerCancelEvent cancel5 = PointerCancelEvent(pointer: 5);
    const PointerCancelEvent cancel6 = PointerCancelEvent(pointer: 6);

    setUp(() {
      tap =
          TapGestureRecognizer()
            ..onTapDown = (TapDownDetails details) {
              recognized.add('primaryDown');
            }
            ..onTap = () {
              recognized.add('primary');
            }
            ..onTapUp = (TapUpDetails details) {
              recognized.add('primaryUp');
            }
            ..onTapCancel = () {
              recognized.add('primaryCancel');
            }
            ..onSecondaryTapDown = (TapDownDetails details) {
              recognized.add('secondaryDown');
            }
            ..onSecondaryTapUp = (TapUpDetails details) {
              recognized.add('secondaryUp');
            }
            ..onSecondaryTapCancel = () {
              recognized.add('secondaryCancel');
            }
            ..onTertiaryTapDown = (TapDownDetails details) {
              recognized.add('tertiaryDown');
            }
            ..onTertiaryTapUp = (TapUpDetails details) {
              recognized.add('tertiaryUp');
            }
            ..onTertiaryTapCancel = () {
              recognized.add('tertiaryCancel');
            };
    });

    tearDown(() {
      recognized.clear();
      tap.dispose();
    });

    testGesture('A primary tap should trigger primary callbacks', (GestureTester tester) {
      tap.addPointer(down1);
      tester.closeArena(down1.pointer);
      expect(recognized, <String>[]);
      tester.async.elapse(const Duration(milliseconds: 500));
      expect(recognized, <String>['primaryDown']);
      recognized.clear();

      tester.route(up1);
      expect(recognized, <String>['primaryUp', 'primary']);
      GestureBinding.instance.gestureArena.sweep(down1.pointer);
    });

    testGesture('A primary tap cancel trigger primary callbacks', (GestureTester tester) {
      tap.addPointer(down1);
      tester.closeArena(down1.pointer);
      expect(recognized, <String>[]);
      tester.async.elapse(const Duration(milliseconds: 500));
      expect(recognized, <String>['primaryDown']);
      recognized.clear();

      tester.route(cancel1);
      expect(recognized, <String>['primaryCancel']);
      GestureBinding.instance.gestureArena.sweep(down1.pointer);
    });

    testGesture('A secondary tap should trigger secondary callbacks', (GestureTester tester) {
      tap.addPointer(down5);
      tester.closeArena(down5.pointer);
      expect(recognized, <String>[]);
      tester.async.elapse(const Duration(milliseconds: 500));
      expect(recognized, <String>['secondaryDown']);
      recognized.clear();

      tester.route(up5);
      GestureBinding.instance.gestureArena.sweep(down5.pointer);
      expect(recognized, <String>['secondaryUp']);
    });

    testGesture('A tertiary tap should trigger tertiary callbacks', (GestureTester tester) {
      tap.addPointer(down6);
      tester.closeArena(down6.pointer);
      expect(recognized, <String>[]);
      tester.async.elapse(const Duration(milliseconds: 500));
      expect(recognized, <String>['tertiaryDown']);
      recognized.clear();

      tester.route(up6);
      GestureBinding.instance.gestureArena.sweep(down6.pointer);
      expect(recognized, <String>['tertiaryUp']);
    });

    testGesture('A secondary tap cancel should trigger secondary callbacks', (
      GestureTester tester,
    ) {
      tap.addPointer(down5);
      tester.closeArena(down5.pointer);
      expect(recognized, <String>[]);
      tester.async.elapse(const Duration(milliseconds: 500));
      expect(recognized, <String>['secondaryDown']);
      recognized.clear();

      tester.route(cancel5);
      GestureBinding.instance.gestureArena.sweep(down5.pointer);
      expect(recognized, <String>['secondaryCancel']);
    });

    testGesture('A tertiary tap cancel should trigger tertiary callbacks', (GestureTester tester) {
      tap.addPointer(down6);
      tester.closeArena(down6.pointer);
      expect(recognized, <String>[]);
      tester.async.elapse(const Duration(milliseconds: 500));
      expect(recognized, <String>['tertiaryDown']);
      recognized.clear();

      tester.route(cancel6);
      GestureBinding.instance.gestureArena.sweep(down6.pointer);
      expect(recognized, <String>['tertiaryCancel']);
    });
  });

  testGesture('A second tap after rejection is ignored', (GestureTester tester) {
    bool didTap = false;

    final TapGestureRecognizer tap =
        TapGestureRecognizer()
          ..onTap = () {
            didTap = true;
          };
    addTearDown(tap.dispose);

    // Add drag recognizer for competition
    final HorizontalDragGestureRecognizer drag =
        HorizontalDragGestureRecognizer()..onStart = (_) {};
    addTearDown(drag.dispose);

    final TestPointer pointer1 = TestPointer();

    final PointerDownEvent down = pointer1.down(Offset.zero);
    drag.addPointer(down);
    tap.addPointer(down);

    tester.closeArena(1);

    // One-finger moves, canceling the tap
    tester.route(down);
    tester.route(pointer1.move(const Offset(50.0, 0)));

    // Add another finger
    final TestPointer pointer2 = TestPointer(2);
    final PointerDownEvent down2 = pointer2.down(const Offset(10.0, 20.0));
    drag.addPointer(down2);
    tap.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);

    expect(didTap, isFalse);
  });
}
