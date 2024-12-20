// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';

import '../gestures/gesture_tester.dart';

// Anything longer than [kDoubleTapTimeout] will reset the consecutive tap count.
final Duration kConsecutiveTapDelay = kDoubleTapTimeout ~/ 2;

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  late List<String> events;
  late BaseTapAndDragGestureRecognizer tapAndDrag;

  void setUpTapAndPanGestureRecognizer({
    bool eagerVictoryOnDrag = true, // This is the default for [BaseTapAndDragGestureRecognizer].
  }) {
    tapAndDrag =
        TapAndPanGestureRecognizer()
          ..dragStartBehavior = DragStartBehavior.down
          ..eagerVictoryOnDrag = eagerVictoryOnDrag
          ..maxConsecutiveTap = 3
          ..onTapDown = (TapDragDownDetails details) {
            events.add('down#${details.consecutiveTapCount}');
          }
          ..onTapUp = (TapDragUpDetails details) {
            events.add('up#${details.consecutiveTapCount}');
          }
          ..onDragStart = (TapDragStartDetails details) {
            events.add('panstart#${details.consecutiveTapCount}');
          }
          ..onDragUpdate = (TapDragUpdateDetails details) {
            events.add('panupdate#${details.consecutiveTapCount}');
          }
          ..onDragEnd = (TapDragEndDetails details) {
            events.add('panend#${details.consecutiveTapCount}');
          }
          ..onCancel = () {
            events.add('cancel');
          };
    addTearDown(tapAndDrag.dispose);
  }

  void setUpTapAndHorizontalDragGestureRecognizer({
    bool eagerVictoryOnDrag = true, // This is the default for [BaseTapAndDragGestureRecognizer].
  }) {
    tapAndDrag =
        TapAndHorizontalDragGestureRecognizer()
          ..dragStartBehavior = DragStartBehavior.down
          ..eagerVictoryOnDrag = eagerVictoryOnDrag
          ..maxConsecutiveTap = 3
          ..onTapDown = (TapDragDownDetails details) {
            events.add('down#${details.consecutiveTapCount}');
          }
          ..onTapUp = (TapDragUpDetails details) {
            events.add('up#${details.consecutiveTapCount}');
          }
          ..onDragStart = (TapDragStartDetails details) {
            events.add('horizontaldragstart#${details.consecutiveTapCount}');
          }
          ..onDragUpdate = (TapDragUpdateDetails details) {
            events.add('horizontaldragupdate#${details.consecutiveTapCount}');
          }
          ..onDragEnd = (TapDragEndDetails details) {
            events.add('horizontaldragend#${details.consecutiveTapCount}');
          }
          ..onCancel = () {
            events.add('cancel');
          };
    addTearDown(tapAndDrag.dispose);
  }

  setUp(() {
    events = <String>[];
  });

  // Down/up pair 1: normal tap sequence
  const PointerDownEvent down1 = PointerDownEvent(pointer: 1, position: Offset(10.0, 10.0));

  const PointerUpEvent up1 = PointerUpEvent(pointer: 1, position: Offset(11.0, 9.0));

  const PointerCancelEvent cancel1 = PointerCancelEvent(pointer: 1);

  // Down/up pair 2: normal tap sequence close to pair 1
  const PointerDownEvent down2 = PointerDownEvent(pointer: 2, position: Offset(12.0, 12.0));

  const PointerUpEvent up2 = PointerUpEvent(pointer: 2, position: Offset(13.0, 11.0));

  // Down/up pair 3: normal tap sequence close to pair 1
  const PointerDownEvent down3 = PointerDownEvent(pointer: 3, position: Offset(12.0, 12.0));

  const PointerUpEvent up3 = PointerUpEvent(pointer: 3, position: Offset(13.0, 11.0));

  // Down/up pair 4: normal tap sequence far away from pair 1
  const PointerDownEvent down4 = PointerDownEvent(pointer: 4, position: Offset(130.0, 130.0));

  const PointerUpEvent up4 = PointerUpEvent(pointer: 4, position: Offset(131.0, 129.0));

  // Down/move/up sequence 5: intervening motion
  const PointerDownEvent down5 = PointerDownEvent(pointer: 5, position: Offset(10.0, 10.0));

  const PointerMoveEvent move5 = PointerMoveEvent(pointer: 5, position: Offset(25.0, 25.0));

  const PointerUpEvent up5 = PointerUpEvent(pointer: 5, position: Offset(25.0, 25.0));

  // Mouse Down/move/up sequence 6: intervening motion - kPrecisePointerPanSlop
  const PointerDownEvent down6 = PointerDownEvent(
    kind: PointerDeviceKind.mouse,
    pointer: 6,
    position: Offset(10.0, 10.0),
  );

  const PointerMoveEvent move6 = PointerMoveEvent(
    kind: PointerDeviceKind.mouse,
    pointer: 6,
    position: Offset(15.0, 15.0),
    delta: Offset(5.0, 5.0),
  );

  const PointerUpEvent up6 = PointerUpEvent(
    kind: PointerDeviceKind.mouse,
    pointer: 6,
    position: Offset(15.0, 15.0),
  );

  testGesture('Recognizes consecutive taps', (GestureTester tester) {
    setUpTapAndPanGestureRecognizer();

    tapAndDrag.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);
    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(events, <String>['down#1', 'up#1']);

    events.clear();
    tester.async.elapse(kConsecutiveTapDelay);
    tapAndDrag.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);
    tester.route(up2);
    GestureBinding.instance.gestureArena.sweep(2);
    expect(events, <String>['down#2', 'up#2']);

    events.clear();
    tester.async.elapse(kConsecutiveTapDelay);
    tapAndDrag.addPointer(down3);
    tester.closeArena(3);
    tester.route(down3);
    tester.route(up3);
    GestureBinding.instance.gestureArena.sweep(3);
    expect(events, <String>['down#3', 'up#3']);
  });

  testGesture('Resets if times out in between taps', (GestureTester tester) {
    setUpTapAndPanGestureRecognizer();

    tapAndDrag.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);
    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(events, <String>['down#1', 'up#1']);

    events.clear();
    tester.async.elapse(const Duration(milliseconds: 1000));
    tapAndDrag.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);
    tester.route(up2);
    GestureBinding.instance.gestureArena.sweep(2);
    expect(events, <String>['down#1', 'up#1']);
  });

  testGesture('Resets if taps are far apart', (GestureTester tester) {
    setUpTapAndPanGestureRecognizer();

    tapAndDrag.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);
    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(events, <String>['down#1', 'up#1']);

    events.clear();
    tester.async.elapse(const Duration(milliseconds: 100));
    tapAndDrag.addPointer(down4);
    tester.closeArena(4);
    tester.route(down4);
    tester.route(up4);
    GestureBinding.instance.gestureArena.sweep(4);
    expect(events, <String>['down#1', 'up#1']);
  });

  testGesture('Resets if consecutiveTapCount reaches maxConsecutiveTap', (GestureTester tester) {
    setUpTapAndPanGestureRecognizer();

    // First tap.
    tapAndDrag.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);
    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(events, <String>['down#1', 'up#1']);

    // Second tap.
    events.clear();
    tapAndDrag.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);
    tester.route(up2);
    GestureBinding.instance.gestureArena.sweep(2);
    expect(events, <String>['down#2', 'up#2']);

    // Third tap.
    events.clear();
    tapAndDrag.addPointer(down3);
    tester.closeArena(3);
    tester.route(down3);
    tester.route(up3);
    GestureBinding.instance.gestureArena.sweep(3);
    expect(events, <String>['down#3', 'up#3']);

    // Fourth tap. Here we arrived at the `maxConsecutiveTap` for `consecutiveTapCount`
    // so our count should reset and our new count should be `1`.
    events.clear();
    tapAndDrag.addPointer(down3);
    tester.closeArena(3);
    tester.route(down3);
    tester.route(up3);
    GestureBinding.instance.gestureArena.sweep(3);
    expect(events, <String>['down#1', 'up#1']);
  });

  testGesture('Should recognize drag', (GestureTester tester) {
    setUpTapAndPanGestureRecognizer();

    final TestPointer pointer = TestPointer(5);
    final PointerDownEvent down = pointer.down(const Offset(10.0, 10.0));
    tapAndDrag.addPointer(down);
    tester.closeArena(5);
    tester.route(down);
    tester.route(pointer.move(const Offset(40.0, 45.0)));
    tester.route(pointer.up());
    GestureBinding.instance.gestureArena.sweep(5);
    expect(events, <String>['down#1', 'panstart#1', 'panupdate#1', 'panend#1']);
  });

  testGesture('Recognizes consecutive taps + drag', (GestureTester tester) {
    setUpTapAndPanGestureRecognizer();

    final TestPointer pointer = TestPointer(5);
    final PointerDownEvent downA = pointer.down(const Offset(10.0, 10.0));
    tapAndDrag.addPointer(downA);
    tester.closeArena(5);
    tester.route(downA);
    tester.route(pointer.up());
    GestureBinding.instance.gestureArena.sweep(5);

    tester.async.elapse(kConsecutiveTapDelay);

    final PointerDownEvent downB = pointer.down(const Offset(10.0, 10.0));
    tapAndDrag.addPointer(downB);
    tester.closeArena(5);
    tester.route(downB);
    tester.route(pointer.up());
    GestureBinding.instance.gestureArena.sweep(5);

    tester.async.elapse(kConsecutiveTapDelay);

    final PointerDownEvent downC = pointer.down(const Offset(10.0, 10.0));
    tapAndDrag.addPointer(downC);
    tester.closeArena(5);
    tester.route(downC);
    tester.route(pointer.move(const Offset(40.0, 45.0)));
    tester.route(pointer.up());
    expect(events, <String>[
      'down#1',
      'up#1',
      'down#2',
      'up#2',
      'down#3',
      'panstart#3',
      'panupdate#3',
      'panend#3',
    ]);
  });

  testGesture('Recognizer rejects pointer that is not the primary one (FIFO) - before acceptance', (
    GestureTester tester,
  ) {
    setUpTapAndPanGestureRecognizer();

    tapAndDrag.addPointer(down1);
    tapAndDrag.addPointer(down2);
    tester.closeArena(1);
    tester.route(down1);

    tester.closeArena(2);
    tester.route(down2);

    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);

    tester.route(up2);
    GestureBinding.instance.gestureArena.sweep(2);
    expect(events, <String>['down#1', 'up#1']);
  });

  testGesture('Calls tap up when the recognizer accepts before handleEvent is called', (
    GestureTester tester,
  ) {
    setUpTapAndPanGestureRecognizer();

    tapAndDrag.addPointer(down1);
    tester.closeArena(1);
    GestureBinding.instance.gestureArena.sweep(1);
    tester.route(down1);
    tester.route(up1);
    expect(events, <String>['down#1', 'up#1']);
  });

  testGesture('Recognizer rejects pointer that is not the primary one (FILO) - before acceptance', (
    GestureTester tester,
  ) {
    setUpTapAndPanGestureRecognizer();

    tapAndDrag.addPointer(down1);
    tapAndDrag.addPointer(down2);
    tester.closeArena(1);
    tester.route(down1);

    tester.closeArena(2);
    tester.route(down2);

    tester.route(up2);
    GestureBinding.instance.gestureArena.sweep(2);

    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(events, <String>['down#1', 'up#1']);
  });

  testGesture('Recognizer rejects pointer that is not the primary one (FIFO) - after acceptance', (
    GestureTester tester,
  ) {
    setUpTapAndPanGestureRecognizer();

    tapAndDrag.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);

    tapAndDrag.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);

    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);

    tester.route(up2);
    GestureBinding.instance.gestureArena.sweep(2);

    expect(events, <String>['down#1', 'up#1']);
  });

  testGesture('Recognizer rejects pointer that is not the primary one (FILO) - after acceptance', (
    GestureTester tester,
  ) {
    setUpTapAndPanGestureRecognizer();

    tapAndDrag.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);

    tapAndDrag.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);

    tester.route(up2);
    GestureBinding.instance.gestureArena.sweep(2);

    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(events, <String>['down#1', 'up#1']);
  });

  testGesture('Recognizer detects tap gesture when pointer does not move past tap tolerance', (
    GestureTester tester,
  ) {
    setUpTapAndPanGestureRecognizer();

    // In this test the tap has not travelled past the tap tolerance defined by
    // [kDoubleTapTouchSlop]. It is expected for the recognizer to detect a tap
    // and fire drag cancel.
    tapAndDrag.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);
    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(events, <String>['down#1', 'up#1']);
  });

  testGesture(
    'Recognizer detects drag gesture when pointer moves past tap tolerance but not the drag minimum',
    (GestureTester tester) {
      setUpTapAndPanGestureRecognizer();

      // In this test, the pointer has moved past the tap tolerance but it has
      // not reached the distance travelled to be considered a drag gesture. In
      // this case it is expected for the recognizer to detect a drag and fire tap cancel.
      tapAndDrag.addPointer(down5);
      tester.closeArena(5);
      tester.route(down5);
      tester.route(move5);
      tester.route(up5);
      GestureBinding.instance.gestureArena.sweep(5);
      expect(events, <String>['down#1', 'panstart#1', 'panend#1']);
    },
  );

  testGesture('Beats TapGestureRecognizer when mouse pointer moves past kPrecisePointerPanSlop', (
    GestureTester tester,
  ) {
    setUpTapAndPanGestureRecognizer();

    // This is a regression test for https://github.com/flutter/flutter/issues/122141.
    final TapGestureRecognizer taps =
        TapGestureRecognizer()
          ..onTapDown = (TapDownDetails details) {
            events.add('tapdown');
          }
          ..onTapUp = (TapUpDetails details) {
            events.add('tapup');
          }
          ..onTapCancel = () {
            events.add('tapscancel');
          };
    addTearDown(taps.dispose);

    tapAndDrag.addPointer(down6);
    taps.addPointer(down6);
    tester.closeArena(6);
    tester.route(down6);
    tester.route(move6);
    tester.route(up6);
    GestureBinding.instance.gestureArena.sweep(6);

    expect(events, <String>['down#1', 'panstart#1', 'panupdate#1', 'panend#1']);
  });

  testGesture(
    'Recognizer declares self-victory in a non-empty arena when pointer travels minimum distance to be considered a drag',
    (GestureTester tester) {
      setUpTapAndPanGestureRecognizer();

      final PanGestureRecognizer pans =
          PanGestureRecognizer()
            ..onStart = (DragStartDetails details) {
              events.add('panstart');
            }
            ..onUpdate = (DragUpdateDetails details) {
              events.add('panupdate');
            }
            ..onEnd = (DragEndDetails details) {
              events.add('panend');
            }
            ..onCancel = () {
              events.add('pancancel');
            };
      addTearDown(pans.dispose);

      final TestPointer pointer = TestPointer(5);
      final PointerDownEvent downB = pointer.down(const Offset(10.0, 10.0));
      // When competing against another [DragGestureRecognizer], the recognizer
      // that first in the arena will win after sweep is called.
      tapAndDrag.addPointer(downB);
      pans.addPointer(downB);
      tester.closeArena(5);
      tester.route(downB);
      tester.route(pointer.move(const Offset(40.0, 45.0)));
      tester.route(pointer.up());
      expect(events, <String>['pancancel', 'down#1', 'panstart#1', 'panupdate#1', 'panend#1']);
    },
  );

  testGesture(
    'TapAndHorizontalDragGestureRecognizer accepts drag on a pan when the arena has already been won by the primary pointer',
    (GestureTester tester) {
      setUpTapAndHorizontalDragGestureRecognizer();

      final TestPointer pointer = TestPointer(5);
      final PointerDownEvent downB = pointer.down(const Offset(10.0, 10.0));

      tapAndDrag.addPointer(downB);
      tester.closeArena(5);
      tester.route(downB);
      tester.route(pointer.move(const Offset(25.0, 45.0)));
      tester.route(pointer.up());
      expect(events, <String>[
        'down#1',
        'horizontaldragstart#1',
        'horizontaldragupdate#1',
        'horizontaldragend#1',
      ]);
    },
  );

  testGesture(
    'TapAndHorizontalDragGestureRecognizer loses to VerticalDragGestureRecognizer on a vertical drag',
    (GestureTester tester) {
      setUpTapAndHorizontalDragGestureRecognizer();

      final VerticalDragGestureRecognizer verticalDrag =
          VerticalDragGestureRecognizer()
            ..onStart = (DragStartDetails details) {
              events.add('verticalstart');
            }
            ..onUpdate = (DragUpdateDetails details) {
              events.add('verticalupdate');
            }
            ..onEnd = (DragEndDetails details) {
              events.add('verticalend');
            }
            ..onCancel = () {
              events.add('verticalcancel');
            };
      addTearDown(verticalDrag.dispose);

      final TestPointer pointer = TestPointer(5);
      final PointerDownEvent downB = pointer.down(const Offset(10.0, 10.0));

      tapAndDrag.addPointer(downB);
      verticalDrag.addPointer(downB);
      tester.closeArena(5);
      tester.route(downB);
      tester.route(pointer.move(const Offset(10.0, 45.0)));
      tester.route(pointer.move(const Offset(10.0, 100.0)));
      tester.route(pointer.up());
      expect(events, <String>['verticalstart', 'verticalupdate', 'verticalend']);
    },
  );

  testGesture(
    'TapAndPanGestureRecognizer loses to VerticalDragGestureRecognizer on a vertical drag',
    (GestureTester tester) {
      setUpTapAndPanGestureRecognizer();

      final VerticalDragGestureRecognizer verticalDrag =
          VerticalDragGestureRecognizer()
            ..onStart = (DragStartDetails details) {
              events.add('verticalstart');
            }
            ..onUpdate = (DragUpdateDetails details) {
              events.add('verticalupdate');
            }
            ..onEnd = (DragEndDetails details) {
              events.add('verticalend');
            }
            ..onCancel = () {
              events.add('verticalcancel');
            };
      addTearDown(verticalDrag.dispose);

      final TestPointer pointer = TestPointer(5);
      final PointerDownEvent downB = pointer.down(const Offset(10.0, 10.0));

      tapAndDrag.addPointer(downB);
      verticalDrag.addPointer(downB);
      tester.closeArena(5);
      tester.route(downB);
      tester.route(pointer.move(const Offset(10.0, 45.0)));
      tester.route(pointer.move(const Offset(10.0, 100.0)));
      tester.route(pointer.up());
      expect(events, <String>['verticalstart', 'verticalupdate', 'verticalend']);
    },
  );

  testGesture(
    'TapAndHorizontalDragGestureRecognizer beats VerticalDragGestureRecognizer on a horizontal drag',
    (GestureTester tester) {
      setUpTapAndHorizontalDragGestureRecognizer();

      final VerticalDragGestureRecognizer verticalDrag =
          VerticalDragGestureRecognizer()
            ..onStart = (DragStartDetails details) {
              events.add('verticalstart');
            }
            ..onUpdate = (DragUpdateDetails details) {
              events.add('verticalupdate');
            }
            ..onEnd = (DragEndDetails details) {
              events.add('verticalend');
            }
            ..onCancel = () {
              events.add('verticalcancel');
            };
      addTearDown(verticalDrag.dispose);

      final TestPointer pointer = TestPointer(5);
      final PointerDownEvent downB = pointer.down(const Offset(10.0, 10.0));

      tapAndDrag.addPointer(downB);
      verticalDrag.addPointer(downB);
      tester.closeArena(5);
      tester.route(downB);
      tester.route(pointer.move(const Offset(45.0, 10.0)));
      tester.route(pointer.up());
      expect(events, <String>[
        'verticalcancel',
        'down#1',
        'horizontaldragstart#1',
        'horizontaldragupdate#1',
        'horizontaldragend#1',
      ]);
    },
  );

  testGesture(
    'TapAndPanGestureRecognizer beats VerticalDragGestureRecognizer on a horizontal pan',
    (GestureTester tester) {
      setUpTapAndPanGestureRecognizer();

      final VerticalDragGestureRecognizer verticalDrag =
          VerticalDragGestureRecognizer()
            ..onStart = (DragStartDetails details) {
              events.add('verticalstart');
            }
            ..onUpdate = (DragUpdateDetails details) {
              events.add('verticalupdate');
            }
            ..onEnd = (DragEndDetails details) {
              events.add('verticalend');
            }
            ..onCancel = () {
              events.add('verticalcancel');
            };
      addTearDown(verticalDrag.dispose);

      final TestPointer pointer = TestPointer(5);
      final PointerDownEvent downB = pointer.down(const Offset(10.0, 10.0));

      tapAndDrag.addPointer(downB);
      verticalDrag.addPointer(downB);
      tester.closeArena(5);
      tester.route(downB);
      tester.route(pointer.move(const Offset(45.0, 25.0)));
      tester.route(pointer.up());
      expect(events, <String>['verticalcancel', 'down#1', 'panstart#1', 'panupdate#1', 'panend#1']);
    },
  );

  testGesture(
    'Recognizer loses when competing against a DragGestureRecognizer for a drag when eagerVictoryOnDrag is disabled',
    (GestureTester tester) {
      setUpTapAndPanGestureRecognizer(eagerVictoryOnDrag: false);
      final PanGestureRecognizer pans =
          PanGestureRecognizer()
            ..onStart = (DragStartDetails details) {
              events.add('panstart');
            }
            ..onUpdate = (DragUpdateDetails details) {
              events.add('panupdate');
            }
            ..onEnd = (DragEndDetails details) {
              events.add('panend');
            }
            ..onCancel = () {
              events.add('pancancel');
            };
      addTearDown(pans.dispose);

      final TestPointer pointer = TestPointer(5);
      final PointerDownEvent downB = pointer.down(const Offset(10.0, 10.0));
      // When competing against another [DragGestureRecognizer], the [TapAndPanGestureRecognizer]
      // will only win when it is the last recognizer in the arena.
      tapAndDrag.addPointer(downB);
      pans.addPointer(downB);
      tester.closeArena(5);
      tester.route(downB);
      tester.route(pointer.move(const Offset(40.0, 45.0)));
      tester.route(pointer.up());
      expect(events, <String>['panstart', 'panend']);
    },
  );

  testGesture('Drag state is properly reset after losing GestureArena', (GestureTester tester) {
    setUpTapAndHorizontalDragGestureRecognizer(eagerVictoryOnDrag: false);
    final HorizontalDragGestureRecognizer horizontalDrag =
        HorizontalDragGestureRecognizer()
          ..onStart = (DragStartDetails details) {
            events.add('basichorizontalstart');
          }
          ..onUpdate = (DragUpdateDetails details) {
            events.add('basichorizontalupdate');
          }
          ..onEnd = (DragEndDetails details) {
            events.add('basichorizontalend');
          }
          ..onCancel = () {
            events.add('basichorizontalcancel');
          };
    addTearDown(horizontalDrag.dispose);

    final LongPressGestureRecognizer longpress =
        LongPressGestureRecognizer()
          ..onLongPressStart = (LongPressStartDetails details) {
            events.add('longpressstart');
          }
          ..onLongPressMoveUpdate = (LongPressMoveUpdateDetails details) {
            events.add('longpressmoveupdate');
          }
          ..onLongPressEnd = (LongPressEndDetails details) {
            events.add('longpressend');
          }
          ..onLongPressCancel = () {
            events.add('longpresscancel');
          };
    addTearDown(longpress.dispose);

    FlutterErrorDetails? errorDetails;
    final FlutterExceptionHandler? oldHandler = FlutterError.onError;
    FlutterError.onError = (FlutterErrorDetails details) {
      errorDetails = details;
    };

    final TestPointer pointer = TestPointer(5);
    final PointerDownEvent downB = pointer.down(const Offset(10.0, 10.0));
    // When competing against another [DragGestureRecognizer], the [TapAndPanGestureRecognizer]
    // will only win when it is the last recognizer in the arena.
    tapAndDrag.addPointer(downB);
    horizontalDrag.addPointer(downB);
    longpress.addPointer(downB);
    tester.closeArena(5);
    tester.route(downB);
    tester.route(pointer.move(const Offset(28.1, 10.0)));
    tester.route(pointer.up());
    expect(events, <String>['basichorizontalstart', 'basichorizontalend']);

    final PointerDownEvent downC = pointer.down(const Offset(10.0, 10.0));
    tapAndDrag.addPointer(downC);
    horizontalDrag.addPointer(downC);
    longpress.addPointer(downC);
    tester.closeArena(5);
    tester.route(downC);
    tester.route(pointer.up());
    FlutterError.onError = oldHandler;
    expect(errorDetails, isNull);
  });

  testGesture('Beats LongPressGestureRecognizer on a consecutive tap greater than one', (
    GestureTester tester,
  ) {
    setUpTapAndPanGestureRecognizer();

    final LongPressGestureRecognizer longpress =
        LongPressGestureRecognizer()
          ..onLongPressStart = (LongPressStartDetails details) {
            events.add('longpressstart');
          }
          ..onLongPressMoveUpdate = (LongPressMoveUpdateDetails details) {
            events.add('longpressmoveupdate');
          }
          ..onLongPressEnd = (LongPressEndDetails details) {
            events.add('longpressend');
          }
          ..onLongPressCancel = () {
            events.add('longpresscancel');
          };
    addTearDown(longpress.dispose);

    final TestPointer pointer = TestPointer(5);
    final PointerDownEvent downA = pointer.down(const Offset(10.0, 10.0));
    tapAndDrag.addPointer(downA);
    longpress.addPointer(downA);
    tester.closeArena(5);
    tester.route(downA);
    tester.route(pointer.up());
    GestureBinding.instance.gestureArena.sweep(5);

    tester.async.elapse(kConsecutiveTapDelay);

    final PointerDownEvent downB = pointer.down(const Offset(10.0, 10.0));
    tapAndDrag.addPointer(downB);
    longpress.addPointer(downB);
    tester.closeArena(5);
    tester.route(downB);

    tester.async.elapse(const Duration(milliseconds: 500));

    tester.route(pointer.move(const Offset(40.0, 45.0)));
    tester.route(pointer.up());
    expect(events, <String>[
      'longpresscancel',
      'down#1',
      'up#1',
      'down#2',
      'panstart#2',
      'panupdate#2',
      'panend#2',
    ]);
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/129161.
  testGesture(
    'Beats TapGestureRecognizer and DoubleTapGestureRecognizer when the pointer has not moved and this recognizer is the first in the arena',
    (GestureTester tester) {
      setUpTapAndPanGestureRecognizer();

      final TapGestureRecognizer taps =
          TapGestureRecognizer()
            ..onTapDown = (TapDownDetails details) {
              events.add('tapdown');
            }
            ..onTapUp = (TapUpDetails details) {
              events.add('tapup');
            }
            ..onTapCancel = () {
              events.add('tapscancel');
            };
      addTearDown(taps.dispose);

      final DoubleTapGestureRecognizer doubleTaps =
          DoubleTapGestureRecognizer()
            ..onDoubleTapDown = (TapDownDetails details) {
              events.add('doubletapdown');
            }
            ..onDoubleTap = () {
              events.add('doubletapup');
            }
            ..onDoubleTapCancel = () {
              events.add('doubletapcancel');
            };
      addTearDown(doubleTaps.dispose);

      tapAndDrag.addPointer(down1);
      taps.addPointer(down1);
      doubleTaps.addPointer(down1);
      tester.closeArena(1);
      tester.route(down1);
      tester.route(up1);
      GestureBinding.instance.gestureArena.sweep(1);
      // Wait for GestureArena to resolve itself.
      tester.async.elapse(kDoubleTapTimeout);
      expect(events, <String>['down#1', 'up#1']);
    },
  );

  testGesture(
    'Beats TapGestureRecognizer when the pointer has not moved and this recognizer is the first in the arena',
    (GestureTester tester) {
      setUpTapAndPanGestureRecognizer();

      final TapGestureRecognizer taps =
          TapGestureRecognizer()
            ..onTapDown = (TapDownDetails details) {
              events.add('tapdown');
            }
            ..onTapUp = (TapUpDetails details) {
              events.add('tapup');
            }
            ..onTapCancel = () {
              events.add('tapscancel');
            };
      addTearDown(taps.dispose);
      tapAndDrag.addPointer(down1);
      taps.addPointer(down1);
      tester.closeArena(1);
      tester.route(down1);
      tester.route(up1);
      GestureBinding.instance.gestureArena.sweep(1);
      expect(events, <String>['down#1', 'up#1']);
    },
  );

  testGesture('Beats TapGestureRecognizer when the pointer has exceeded the slop tolerance', (
    GestureTester tester,
  ) {
    setUpTapAndPanGestureRecognizer();

    final TapGestureRecognizer taps =
        TapGestureRecognizer()
          ..onTapDown = (TapDownDetails details) {
            events.add('tapdown');
          }
          ..onTapUp = (TapUpDetails details) {
            events.add('tapup');
          }
          ..onTapCancel = () {
            events.add('tapscancel');
          };
    addTearDown(taps.dispose);

    tapAndDrag.addPointer(down5);
    taps.addPointer(down5);
    tester.closeArena(5);
    tester.route(down5);
    tester.route(move5);
    tester.route(up5);
    GestureBinding.instance.gestureArena.sweep(5);
    expect(events, <String>['down#1', 'panstart#1', 'panend#1']);

    events.clear();
    tester.async.elapse(const Duration(milliseconds: 1000));
    taps.addPointer(down1);
    tapAndDrag.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);
    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(events, <String>['tapdown', 'tapup']);
  });

  testGesture(
    'Ties with PanGestureRecognizer when pointer has not met sufficient global distance to be a drag',
    (GestureTester tester) {
      setUpTapAndPanGestureRecognizer();

      final PanGestureRecognizer pans =
          PanGestureRecognizer()
            ..onStart = (DragStartDetails details) {
              events.add('panstart');
            }
            ..onUpdate = (DragUpdateDetails details) {
              events.add('panupdate');
            }
            ..onEnd = (DragEndDetails details) {
              events.add('panend');
            }
            ..onCancel = () {
              events.add('pancancel');
            };
      addTearDown(pans.dispose);

      tapAndDrag.addPointer(down5);
      pans.addPointer(down5);
      tester.closeArena(5);
      tester.route(down5);
      tester.route(move5);
      tester.route(up5);
      GestureBinding.instance.gestureArena.sweep(5);
      expect(events, <String>['pancancel']);
    },
  );

  testGesture('Defaults to drag when pointer dragged past slop tolerance', (GestureTester tester) {
    setUpTapAndPanGestureRecognizer();

    tapAndDrag.addPointer(down5);
    tester.closeArena(5);
    tester.route(down5);
    tester.route(move5);
    tester.route(up5);
    GestureBinding.instance.gestureArena.sweep(5);
    expect(events, <String>['down#1', 'panstart#1', 'panend#1']);

    events.clear();
    tester.async.elapse(const Duration(milliseconds: 1000));
    tapAndDrag.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);
    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(events, <String>['down#1', 'up#1']);
  });

  testGesture('Fires cancel and resets for PointerCancelEvent', (GestureTester tester) {
    setUpTapAndPanGestureRecognizer();

    tapAndDrag.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);
    tester.route(cancel1);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(events, <String>['down#1', 'cancel']);

    events.clear();
    tester.async.elapse(const Duration(milliseconds: 100));
    tapAndDrag.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);
    tester.route(up2);
    GestureBinding.instance.gestureArena.sweep(2);
    expect(events, <String>['down#1', 'up#1']);
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/102084.
  testGesture('Does not call onDragEnd if not provided', (GestureTester tester) {
    tapAndDrag =
        TapAndDragGestureRecognizer()
          ..dragStartBehavior = DragStartBehavior.down
          ..maxConsecutiveTap = 3
          ..onTapDown = (TapDragDownDetails details) {
            events.add('down#${details.consecutiveTapCount}');
          };
    addTearDown(tapAndDrag.dispose);

    FlutterErrorDetails? errorDetails;
    final FlutterExceptionHandler? oldHandler = FlutterError.onError;
    FlutterError.onError = (FlutterErrorDetails details) {
      errorDetails = details;
    };
    addTearDown(() {
      FlutterError.onError = oldHandler;
    });

    tapAndDrag.addPointer(down5);
    tester.closeArena(5);
    tester.route(down5);
    tester.route(move5);
    tester.route(up5);
    GestureBinding.instance.gestureArena.sweep(5);
    expect(events, <String>['down#1']);

    expect(errorDetails, isNull);

    events.clear();
    tester.async.elapse(const Duration(milliseconds: 1000));
    tapAndDrag.addPointer(down1);
    tester.closeArena(1);
    tester.route(down1);
    tester.route(up1);
    GestureBinding.instance.gestureArena.sweep(1);
    expect(events, <String>['down#1']);
  });
}
