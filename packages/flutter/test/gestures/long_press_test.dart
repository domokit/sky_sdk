// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';

import 'gesture_tester.dart';

// Down/move/up pair 1: normal tap sequence
const PointerDownEvent down = PointerDownEvent(pointer: 5, position: Offset(10, 10));

const PointerUpEvent up = PointerUpEvent(pointer: 5, position: Offset(11, 9));

const PointerMoveEvent move = PointerMoveEvent(pointer: 5, position: Offset(100, 200));

// Down/up pair 2: normal tap sequence far away from pair 1
const PointerDownEvent down2 = PointerDownEvent(pointer: 6, position: Offset(10, 10));

const PointerUpEvent up2 = PointerUpEvent(pointer: 6, position: Offset(11, 9));

// Down/up pair 3: tap sequence with secondary button
const PointerDownEvent down3 = PointerDownEvent(
  pointer: 7,
  position: Offset(30, 30),
  buttons: kSecondaryButton,
);

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  group('Long press', () {
    late LongPressGestureRecognizer gesture;
    late List<String> recognized;

    void setUpHandlers() {
      gesture
        ..onLongPressDown = (LongPressDownDetails details) {
          recognized.add('down');
        }
        ..onLongPressCancel = () {
          recognized.add('cancel');
        }
        ..onLongPress = () {
          recognized.add('start');
        }
        ..onLongPressMoveUpdate = (LongPressMoveUpdateDetails details) {
          recognized.add('move');
        }
        ..onLongPressUp = () {
          recognized.add('end');
        };
    }

    setUp(() {
      recognized = <String>[];
      gesture = LongPressGestureRecognizer();
      setUpHandlers();
    });

    testGesture('Should recognize long press', (GestureTester tester) {
      gesture.addPointer(down);
      tester.closeArena(5);
      expect(recognized, const <String>[]);
      tester.route(down);
      expect(recognized, const <String>['down']);
      tester.async.elapse(const Duration(milliseconds: 300));
      expect(recognized, const <String>['down']);
      tester.async.elapse(const Duration(milliseconds: 700));
      expect(recognized, const <String>['down', 'start']);
      gesture.dispose();
      expect(recognized, const <String>['down', 'start']);
    });

    testGesture('Should recognize long press with altered duration', (GestureTester tester) {
      gesture = LongPressGestureRecognizer(duration: const Duration(milliseconds: 100));
      setUpHandlers();
      gesture.addPointer(down);
      tester.closeArena(5);
      expect(recognized, const <String>[]);
      tester.route(down);
      expect(recognized, const <String>['down']);
      tester.async.elapse(const Duration(milliseconds: 50));
      expect(recognized, const <String>['down']);
      tester.async.elapse(const Duration(milliseconds: 50));
      expect(recognized, const <String>['down', 'start']);
      gesture.dispose();
      expect(recognized, const <String>['down', 'start']);
    });

    testGesture('Up cancels long press', (GestureTester tester) {
      gesture.addPointer(down);
      tester.closeArena(5);
      expect(recognized, const <String>[]);
      tester.route(down);
      expect(recognized, const <String>['down']);
      tester.async.elapse(const Duration(milliseconds: 300));
      expect(recognized, const <String>['down']);
      tester.route(up);
      expect(recognized, const <String>['down', 'cancel']);
      tester.async.elapse(const Duration(seconds: 1));
      gesture.dispose();
      expect(recognized, const <String>['down', 'cancel']);
    });

    testGesture('Moving before accept cancels', (GestureTester tester) {
      gesture.addPointer(down);
      tester.closeArena(5);
      expect(recognized, const <String>[]);
      tester.route(down);
      expect(recognized, const <String>['down']);
      tester.async.elapse(const Duration(milliseconds: 300));
      expect(recognized, const <String>['down']);
      tester.route(move);
      expect(recognized, const <String>['down', 'cancel']);
      tester.async.elapse(const Duration(seconds: 1));
      tester.route(up);
      tester.async.elapse(const Duration(milliseconds: 300));
      expect(recognized, const <String>['down', 'cancel']);
      gesture.dispose();
      expect(recognized, const <String>['down', 'cancel']);
    });

    testGesture('Moving after accept is ok', (GestureTester tester) {
      gesture.addPointer(down);
      tester.closeArena(5);
      expect(recognized, const <String>[]);
      tester.route(down);
      expect(recognized, const <String>['down']);
      tester.async.elapse(const Duration(seconds: 1));
      expect(recognized, const <String>['down', 'start']);
      tester.route(move);
      expect(recognized, const <String>['down', 'start', 'move']);
      tester.route(up);
      expect(recognized, const <String>['down', 'start', 'move', 'end']);
      tester.async.elapse(const Duration(milliseconds: 300));
      expect(recognized, const <String>['down', 'start', 'move', 'end']);
      gesture.dispose();
      expect(recognized, const <String>['down', 'start', 'move', 'end']);
    });

    testGesture('Should recognize both tap down and long press', (GestureTester tester) {
      final TapGestureRecognizer tap = TapGestureRecognizer();
      tap.onTapDown = (_) {
        recognized.add('tap_down');
      };

      tap.addPointer(down);
      gesture.addPointer(down);
      tester.closeArena(5);
      expect(recognized, const <String>[]);
      tester.route(down);
      expect(recognized, const <String>['down']);
      tester.async.elapse(const Duration(milliseconds: 300));
      expect(recognized, const <String>['down', 'tap_down']);
      tester.async.elapse(const Duration(milliseconds: 700));
      expect(recognized, const <String>['down', 'tap_down', 'start']);
      tap.dispose();
      gesture.dispose();
      expect(recognized, const <String>['down', 'tap_down', 'start']);
    });

    testGesture('Drag start delayed by microtask', (GestureTester tester) {
      final HorizontalDragGestureRecognizer drag = HorizontalDragGestureRecognizer();
      bool isDangerousStack = false;
      drag.onStart = (DragStartDetails details) {
        expect(isDangerousStack, isFalse);
        recognized.add('drag_start');
      };

      drag.addPointer(down);
      gesture.addPointer(down);
      tester.closeArena(5);
      expect(recognized, const <String>[]);
      tester.route(down);
      expect(recognized, const <String>['down']);
      tester.async.elapse(const Duration(milliseconds: 300));
      expect(recognized, const <String>['down']);
      isDangerousStack = true;
      gesture.dispose();
      isDangerousStack = false;
      expect(recognized, const <String>['down', 'cancel']);
      tester.async.flushMicrotasks();
      expect(recognized, const <String>['down', 'cancel', 'drag_start']);
      drag.dispose();
    });

    testGesture('Should recognize long press up', (GestureTester tester) {
      gesture.addPointer(down);
      tester.closeArena(5);
      expect(recognized, const <String>[]);
      tester.route(down); // kLongPressTimeout = 500;
      expect(recognized, const <String>['down']);
      tester.async.elapse(const Duration(milliseconds: 300));
      expect(recognized, const <String>['down']);
      tester.async.elapse(const Duration(milliseconds: 700));
      expect(recognized, const <String>['down', 'start']);
      tester.route(up);
      expect(recognized, const <String>['down', 'start', 'end']);
      gesture.dispose();
      expect(recognized, const <String>['down', 'start', 'end']);
    });

    testGesture('Should not recognize long press with more than one buttons', (
      GestureTester tester,
    ) {
      gesture.addPointer(
        const PointerDownEvent(
          pointer: 5,
          kind: PointerDeviceKind.mouse,
          buttons: kSecondaryMouseButton | kTertiaryButton,
          position: Offset(10, 10),
        ),
      );
      tester.closeArena(5);
      expect(recognized, const <String>[]);
      tester.route(down);
      expect(recognized, const <String>[]);
      tester.async.elapse(const Duration(milliseconds: 1000));
      expect(recognized, const <String>[]);
      tester.route(up);
      expect(recognized, const <String>[]);
      gesture.dispose();
      expect(recognized, const <String>[]);
    });

    testGesture('Should cancel long press when buttons change before acceptance', (
      GestureTester tester,
    ) {
      gesture.addPointer(down);
      tester.closeArena(5);
      expect(recognized, const <String>[]);
      tester.route(down);
      expect(recognized, const <String>['down']);
      tester.async.elapse(const Duration(milliseconds: 300));
      expect(recognized, const <String>['down']);
      tester.route(
        const PointerMoveEvent(
          pointer: 5,
          kind: PointerDeviceKind.mouse,
          buttons: kTertiaryButton,
          position: Offset(10, 10),
        ),
      );
      expect(recognized, const <String>['down', 'cancel']);
      tester.async.elapse(const Duration(milliseconds: 700));
      expect(recognized, const <String>['down', 'cancel']);
      tester.route(up);
      expect(recognized, const <String>['down', 'cancel']);
      gesture.dispose();
      expect(recognized, const <String>['down', 'cancel']);
    });

    testGesture('non-allowed pointer does not inadvertently reset the recognizer', (
      GestureTester tester,
    ) {
      gesture = LongPressGestureRecognizer(
        supportedDevices: <PointerDeviceKind>{PointerDeviceKind.touch},
      );
      setUpHandlers();

      // Accept a long-press gesture
      gesture.addPointer(down);
      tester.closeArena(5);
      tester.route(down);
      tester.async.elapse(const Duration(milliseconds: 500));
      expect(recognized, const <String>['down', 'start']);

      // Add a non-allowed pointer (doesn't match the kind filter)
      gesture.addPointer(
        const PointerDownEvent(
          pointer: 101,
          kind: PointerDeviceKind.mouse,
          position: Offset(10, 10),
        ),
      );
      expect(recognized, const <String>['down', 'start']);

      // Moving the primary pointer should result in a normal event
      tester.route(const PointerMoveEvent(pointer: 5, position: Offset(15, 15)));
      expect(recognized, const <String>['down', 'start', 'move']);
      gesture.dispose();
    });
  });

  group('long press drag', () {
    late LongPressGestureRecognizer gesture;
    Offset? longPressDragUpdate;
    late List<String> recognized;

    void setUpHandlers() {
      gesture
        ..onLongPressDown = (LongPressDownDetails details) {
          recognized.add('down');
        }
        ..onLongPressCancel = () {
          recognized.add('cancel');
        }
        ..onLongPress = () {
          recognized.add('start');
        }
        ..onLongPressMoveUpdate = (LongPressMoveUpdateDetails details) {
          recognized.add('move');
          longPressDragUpdate = details.globalPosition;
        }
        ..onLongPressUp = () {
          recognized.add('end');
        };
    }

    setUp(() {
      gesture = LongPressGestureRecognizer();
      setUpHandlers();
      recognized = <String>[];
    });

    testGesture('Should recognize long press down', (GestureTester tester) {
      gesture.addPointer(down);
      tester.closeArena(5);
      expect(recognized, const <String>[]);
      tester.route(down);
      expect(recognized, const <String>['down']);
      tester.async.elapse(const Duration(milliseconds: 300));
      expect(recognized, const <String>['down']);
      tester.async.elapse(const Duration(milliseconds: 700));
      expect(recognized, const <String>['down', 'start']);
      gesture.dispose();
      expect(recognized, const <String>['down', 'start']);
    });

    testGesture('Short up cancels long press', (GestureTester tester) {
      gesture.addPointer(down);
      tester.closeArena(5);
      expect(recognized, const <String>[]);
      tester.route(down);
      expect(recognized, const <String>['down']);
      tester.async.elapse(const Duration(milliseconds: 300));
      expect(recognized, const <String>['down']);
      tester.route(up);
      expect(recognized, const <String>['down', 'cancel']);
      tester.async.elapse(const Duration(seconds: 1));
      expect(recognized, const <String>['down', 'cancel']);
      gesture.dispose();
      expect(recognized, const <String>['down', 'cancel']);
    });

    testGesture('Moving before accept cancels', (GestureTester tester) {
      gesture.addPointer(down);
      tester.closeArena(5);
      expect(recognized, const <String>[]);
      tester.route(down);
      expect(recognized, const <String>['down']);
      tester.async.elapse(const Duration(milliseconds: 300));
      expect(recognized, const <String>['down']);
      tester.route(move);
      expect(recognized, const <String>['down', 'cancel']);
      tester.async.elapse(const Duration(seconds: 1));
      tester.route(up);
      tester.async.elapse(const Duration(milliseconds: 300));
      expect(recognized, const <String>['down', 'cancel']);
      gesture.dispose();
      expect(recognized, const <String>['down', 'cancel']);
    });

    testGesture('Moving after accept does not cancel', (GestureTester tester) {
      gesture.addPointer(down);
      tester.closeArena(5);
      expect(recognized, const <String>[]);
      tester.route(down);
      expect(recognized, const <String>['down']);
      tester.async.elapse(const Duration(seconds: 1));
      expect(recognized, const <String>['down', 'start']);
      tester.route(move);
      expect(recognized, const <String>['down', 'start', 'move']);
      expect(longPressDragUpdate, const Offset(100, 200));
      tester.route(up);
      tester.async.elapse(const Duration(milliseconds: 300));
      expect(recognized, const <String>['down', 'start', 'move', 'end']);
      gesture.dispose();
      expect(recognized, const <String>['down', 'start', 'move', 'end']);
    });
  });

  group('Enforce consistent-button restriction:', () {
    // In sequence between `down` and `up` but with buttons changed
    const PointerMoveEvent moveR = PointerMoveEvent(
      pointer: 5,
      buttons: kSecondaryButton,
      position: Offset(10, 10),
    );

    late LongPressGestureRecognizer gesture;
    final List<String> recognized = <String>[];

    setUp(() {
      gesture =
          LongPressGestureRecognizer()
            ..onLongPressDown = (LongPressDownDetails details) {
              recognized.add('down');
            }
            ..onLongPressCancel = () {
              recognized.add('cancel');
            }
            ..onLongPressStart = (LongPressStartDetails details) {
              recognized.add('start');
            }
            ..onLongPressEnd = (LongPressEndDetails details) {
              recognized.add('end');
            };
    });

    tearDown(() {
      gesture.dispose();
      recognized.clear();
    });

    testGesture('Should cancel long press when buttons change before acceptance', (
      GestureTester tester,
    ) {
      // First press
      gesture.addPointer(down);
      tester.closeArena(down.pointer);
      tester.route(down);
      tester.async.elapse(const Duration(milliseconds: 300));
      tester.route(moveR);
      expect(recognized, const <String>['down', 'cancel']);
      tester.async.elapse(const Duration(milliseconds: 700));
      tester.route(up);
      expect(recognized, const <String>['down', 'cancel']);
    });

    testGesture('Buttons change before acceptance should not prevent the next long press', (
      GestureTester tester,
    ) {
      // First press
      gesture.addPointer(down);
      tester.closeArena(down.pointer);
      tester.route(down);
      expect(recognized, <String>['down']);
      tester.async.elapse(const Duration(milliseconds: 300));
      tester.route(moveR);
      expect(recognized, <String>['down', 'cancel']);
      tester.async.elapse(const Duration(milliseconds: 700));
      tester.route(up);
      recognized.clear();

      // Second press
      gesture.addPointer(down2);
      tester.closeArena(down2.pointer);
      tester.route(down2);
      expect(recognized, <String>['down']);
      tester.async.elapse(const Duration(milliseconds: 1000));
      expect(recognized, <String>['down', 'start']);
      recognized.clear();

      tester.route(up2);
      expect(recognized, <String>['end']);
    });

    testGesture('Should not cancel long press when buttons change after acceptance', (
      GestureTester tester,
    ) {
      // First press
      gesture.addPointer(down);
      tester.closeArena(down.pointer);
      tester.route(down);
      tester.async.elapse(const Duration(milliseconds: 1000));
      expect(recognized, <String>['down', 'start']);
      recognized.clear();

      tester.route(moveR);
      expect(recognized, <String>[]);
      tester.route(up);
      expect(recognized, <String>['end']);
    });

    testGesture('Buttons change after acceptance should not prevent the next long press', (
      GestureTester tester,
    ) {
      // First press
      gesture.addPointer(down);
      tester.closeArena(down.pointer);
      tester.route(down);
      tester.async.elapse(const Duration(milliseconds: 1000));
      tester.route(moveR);
      tester.route(up);
      recognized.clear();

      // Second press
      gesture.addPointer(down2);
      tester.closeArena(down2.pointer);
      tester.route(down2);
      tester.async.elapse(const Duration(milliseconds: 1000));
      expect(recognized, <String>['down', 'start']);
      recognized.clear();

      tester.route(up2);
      expect(recognized, <String>['end']);
    });
  });

  testGesture('Can filter long press based on device kind', (GestureTester tester) {
    final LongPressGestureRecognizer mouseLongPress = LongPressGestureRecognizer(
      supportedDevices: <PointerDeviceKind>{PointerDeviceKind.mouse},
    );

    bool mouseLongPressDown = false;
    mouseLongPress.onLongPress = () {
      mouseLongPressDown = true;
    };

    const PointerDownEvent mouseDown = PointerDownEvent(
      pointer: 5,
      position: Offset(10, 10),
      kind: PointerDeviceKind.mouse,
    );
    const PointerDownEvent touchDown = PointerDownEvent(pointer: 5, position: Offset(10, 10));

    // Touch events shouldn't be recognized.
    mouseLongPress.addPointer(touchDown);
    tester.closeArena(5);
    expect(mouseLongPressDown, isFalse);
    tester.route(touchDown);
    expect(mouseLongPressDown, isFalse);
    tester.async.elapse(const Duration(seconds: 2));
    expect(mouseLongPressDown, isFalse);

    // Mouse events are still recognized.
    mouseLongPress.addPointer(mouseDown);
    tester.closeArena(5);
    expect(mouseLongPressDown, isFalse);
    tester.route(mouseDown);
    expect(mouseLongPressDown, isFalse);
    tester.async.elapse(const Duration(seconds: 2));
    expect(mouseLongPressDown, isTrue);

    mouseLongPress.dispose();
  });

  group('Recognizers listening on different buttons do not form competition:', () {
    // This test is assisted by tap recognizers. If a tap gesture has
    // no competing recognizers, a pointer down event triggers its onTapDown
    // immediately; if there are competitors, onTapDown is triggered after a
    // timeout.
    // The following tests make sure that long press recognizers do not form
    // competition with a tap gesture recognizer listening on a different button.

    final List<String> recognized = <String>[];
    late TapGestureRecognizer tapPrimary;
    late TapGestureRecognizer tapSecondary;
    late LongPressGestureRecognizer longPress;
    setUp(() {
      tapPrimary =
          TapGestureRecognizer()
            ..onTapDown = (TapDownDetails details) {
              recognized.add('tapPrimary');
            };
      tapSecondary =
          TapGestureRecognizer()
            ..onSecondaryTapDown = (TapDownDetails details) {
              recognized.add('tapSecondary');
            };
      longPress =
          LongPressGestureRecognizer()
            ..onLongPressStart = (_) {
              recognized.add('longPress');
            };
    });

    tearDown(() {
      recognized.clear();
      tapPrimary.dispose();
      tapSecondary.dispose();
      longPress.dispose();
    });

    testGesture(
      'A primary long press recognizer does not form competition with a secondary tap recognizer',
      (GestureTester tester) {
        longPress.addPointer(down3);
        tapSecondary.addPointer(down3);
        tester.closeArena(down3.pointer);

        tester.route(down3);
        expect(recognized, <String>['tapSecondary']);
      },
    );

    testGesture('A primary long press recognizer forms competition with a primary tap recognizer', (
      GestureTester tester,
    ) {
      longPress.addPointer(down);
      tapPrimary.addPointer(down);
      tester.closeArena(down.pointer);

      tester.route(down);
      expect(recognized, <String>[]);

      tester.route(up);
      expect(recognized, <String>['tapPrimary']);
    });
  });

  testGesture('A secondary long press should not trigger primary', (GestureTester tester) {
    final List<String> recognized = <String>[];
    final LongPressGestureRecognizer longPress =
        LongPressGestureRecognizer()
          ..onLongPressStart = (LongPressStartDetails details) {
            recognized.add('primaryStart');
          }
          ..onLongPress = () {
            recognized.add('primary');
          }
          ..onLongPressMoveUpdate = (LongPressMoveUpdateDetails details) {
            recognized.add('primaryUpdate');
          }
          ..onLongPressEnd = (LongPressEndDetails details) {
            recognized.add('primaryEnd');
          }
          ..onLongPressUp = () {
            recognized.add('primaryUp');
          };

    const PointerDownEvent down2 = PointerDownEvent(
      pointer: 2,
      buttons: kSecondaryButton,
      position: Offset(30.0, 30.0),
    );

    const PointerMoveEvent move2 = PointerMoveEvent(
      pointer: 2,
      buttons: kSecondaryButton,
      position: Offset(100, 200),
    );

    const PointerUpEvent up2 = PointerUpEvent(pointer: 2, position: Offset(100, 201));

    longPress.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);
    tester.async.elapse(const Duration(milliseconds: 700));
    tester.route(move2);
    tester.route(up2);
    expect(recognized, <String>[]);
    longPress.dispose();
    recognized.clear();
  });

  testGesture('A tertiary long press should not trigger primary or secondary', (
    GestureTester tester,
  ) {
    final List<String> recognized = <String>[];
    final LongPressGestureRecognizer longPress =
        LongPressGestureRecognizer()
          ..onLongPressStart = (LongPressStartDetails details) {
            recognized.add('primaryStart');
          }
          ..onLongPress = () {
            recognized.add('primary');
          }
          ..onLongPressMoveUpdate = (LongPressMoveUpdateDetails details) {
            recognized.add('primaryUpdate');
          }
          ..onLongPressEnd = (LongPressEndDetails details) {
            recognized.add('primaryEnd');
          }
          ..onLongPressUp = () {
            recognized.add('primaryUp');
          }
          ..onSecondaryLongPressStart = (LongPressStartDetails details) {
            recognized.add('secondaryStart');
          }
          ..onSecondaryLongPress = () {
            recognized.add('secondary');
          }
          ..onSecondaryLongPressMoveUpdate = (LongPressMoveUpdateDetails details) {
            recognized.add('secondaryUpdate');
          }
          ..onSecondaryLongPressEnd = (LongPressEndDetails details) {
            recognized.add('secondaryEnd');
          }
          ..onSecondaryLongPressUp = () {
            recognized.add('secondaryUp');
          };

    const PointerDownEvent down2 = PointerDownEvent(
      pointer: 2,
      buttons: kTertiaryButton,
      position: Offset(30.0, 30.0),
    );

    const PointerMoveEvent move2 = PointerMoveEvent(
      pointer: 2,
      buttons: kTertiaryButton,
      position: Offset(100, 200),
    );

    const PointerUpEvent up2 = PointerUpEvent(pointer: 2, position: Offset(100, 201));

    longPress.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);
    tester.async.elapse(const Duration(milliseconds: 700));
    tester.route(move2);
    tester.route(up2);
    expect(recognized, <String>[]);
    longPress.dispose();
    recognized.clear();
  });

  testGesture('Switching buttons mid-stream does not fail to send "end" event', (
    GestureTester tester,
  ) {
    final List<String> recognized = <String>[];
    final LongPressGestureRecognizer longPress =
        LongPressGestureRecognizer()
          ..onLongPressStart = (LongPressStartDetails details) {
            recognized.add('primaryStart');
          }
          ..onLongPressEnd = (LongPressEndDetails details) {
            recognized.add('primaryEnd');
          };

    const PointerDownEvent down4 = PointerDownEvent(pointer: 8, position: Offset(10, 10));

    const PointerMoveEvent move4 = PointerMoveEvent(
      pointer: 8,
      position: Offset(100, 200),
      buttons: kPrimaryButton | kSecondaryButton,
    );

    const PointerUpEvent up4 = PointerUpEvent(
      pointer: 8,
      position: Offset(100, 200),
      buttons: kSecondaryButton,
    );

    longPress.addPointer(down4);
    tester.closeArena(4);
    tester.route(down4);
    tester.async.elapse(const Duration(milliseconds: 1000));
    recognized.add('two seconds later...');
    tester.route(move4);
    tester.async.elapse(const Duration(milliseconds: 1000));
    recognized.add('two more seconds later...');
    tester.route(up4);
    tester.async.elapse(const Duration(milliseconds: 1000));
    expect(recognized, <String>[
      'primaryStart',
      'two seconds later...',
      'two more seconds later...',
      'primaryEnd',
    ]);
    longPress.dispose();
  });

  testGesture(
    'Switching buttons mid-stream does not fail to send "end" event (alternative sequence)',
    (GestureTester tester) {
      // This reproduces sequences seen on macOS.
      final List<String> recognized = <String>[];
      final LongPressGestureRecognizer longPress =
          LongPressGestureRecognizer()
            ..onLongPressStart = (LongPressStartDetails details) {
              recognized.add('primaryStart');
            }
            ..onLongPressEnd = (LongPressEndDetails details) {
              recognized.add('primaryEnd');
            };

      const PointerDownEvent down5 = PointerDownEvent(pointer: 9, position: Offset(10, 10));

      const PointerMoveEvent move5a = PointerMoveEvent(
        pointer: 9,
        position: Offset(100, 200),
        buttons: 3, // add 2
      );

      const PointerMoveEvent move5b = PointerMoveEvent(
        pointer: 9,
        position: Offset(100, 200),
        buttons: 2, // remove 1
      );

      const PointerUpEvent up5 = PointerUpEvent(pointer: 9, position: Offset(100, 200));

      longPress.addPointer(down5);
      tester.closeArena(4);
      tester.route(down5);
      tester.async.elapse(const Duration(milliseconds: 1000));
      recognized.add('two seconds later...');
      tester.route(move5a);
      tester.async.elapse(const Duration(milliseconds: 1000));
      recognized.add('two more seconds later...');
      tester.route(move5b);
      tester.async.elapse(const Duration(milliseconds: 1000));
      recognized.add('two more seconds later still...');
      tester.route(up5);
      tester.async.elapse(const Duration(milliseconds: 1000));
      expect(recognized, <String>[
        'primaryStart',
        'two seconds later...',
        'two more seconds later...',
        'two more seconds later still...',
        'primaryEnd',
      ]);
      longPress.dispose();
    },
  );
}
