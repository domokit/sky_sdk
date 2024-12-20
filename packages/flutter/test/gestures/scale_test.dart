// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';

import 'gesture_tester.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  testGesture('Should recognize scale gestures', (GestureTester tester) {
    final ScaleGestureRecognizer scale = ScaleGestureRecognizer();
    final TapGestureRecognizer tap = TapGestureRecognizer();

    bool didStartScale = false;
    Offset? updatedFocalPoint;
    int? updatedPointerCount;
    scale.onStart = (ScaleStartDetails details) {
      didStartScale = true;
      updatedFocalPoint = details.focalPoint;
      updatedPointerCount = details.pointerCount;
    };

    double? updatedScale;
    double? updatedHorizontalScale;
    double? updatedVerticalScale;
    Offset? updatedDelta;
    scale.onUpdate = (ScaleUpdateDetails details) {
      updatedScale = details.scale;
      updatedHorizontalScale = details.horizontalScale;
      updatedVerticalScale = details.verticalScale;
      updatedFocalPoint = details.focalPoint;
      updatedDelta = details.focalPointDelta;
      updatedPointerCount = details.pointerCount;
    };

    bool didEndScale = false;
    scale.onEnd = (ScaleEndDetails details) {
      didEndScale = true;
    };

    bool didTap = false;
    tap.onTap = () {
      didTap = true;
    };

    final TestPointer pointer1 = TestPointer();

    final PointerDownEvent down = pointer1.down(Offset.zero);
    scale.addPointer(down);
    tap.addPointer(down);

    tester.closeArena(1);
    expect(didStartScale, isFalse);
    expect(updatedScale, isNull);
    expect(updatedFocalPoint, isNull);
    expect(updatedDelta, isNull);
    expect(updatedPointerCount, isNull);
    expect(didEndScale, isFalse);
    expect(didTap, isFalse);

    // One-finger panning
    tester.route(down);
    expect(didStartScale, isFalse);
    expect(updatedScale, isNull);
    expect(updatedFocalPoint, isNull);
    expect(updatedDelta, isNull);
    expect(updatedPointerCount, isNull);
    expect(didEndScale, isFalse);
    expect(didTap, isFalse);

    tester.route(pointer1.move(const Offset(20.0, 30.0)));
    expect(didStartScale, isTrue);
    didStartScale = false;
    expect(updatedFocalPoint, const Offset(20.0, 30.0));
    updatedFocalPoint = null;
    expect(updatedScale, 1.0);
    updatedScale = null;
    expect(updatedDelta, const Offset(20.0, 30.0));
    updatedDelta = null;
    expect(updatedPointerCount, 1);
    updatedPointerCount = null;
    expect(didEndScale, isFalse);
    expect(didTap, isFalse);

    // Two-finger scaling
    final TestPointer pointer2 = TestPointer(2);
    final PointerDownEvent down2 = pointer2.down(const Offset(10.0, 20.0));
    scale.addPointer(down2);
    tap.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);

    expect(didEndScale, isTrue);
    didEndScale = false;
    expect(updatedScale, isNull);
    expect(updatedFocalPoint, isNull);
    expect(updatedDelta, isNull);
    expect(updatedPointerCount, isNull);
    expect(didStartScale, isFalse);

    // Zoom in
    tester.route(pointer2.move(const Offset(0.0, 10.0)));
    expect(didStartScale, isTrue);
    didStartScale = false;
    expect(updatedFocalPoint, const Offset(10.0, 20.0));
    updatedFocalPoint = null;
    expect(updatedScale, 2.0);
    expect(updatedHorizontalScale, 2.0);
    expect(updatedVerticalScale, 2.0);
    expect(updatedDelta, const Offset(-5.0, -5.0));
    expect(updatedPointerCount, 2);
    updatedScale = null;
    updatedHorizontalScale = null;
    updatedVerticalScale = null;
    updatedDelta = null;
    updatedPointerCount = null;
    expect(didEndScale, isFalse);
    expect(didTap, isFalse);

    // Zoom out
    tester.route(pointer2.move(const Offset(15.0, 25.0)));
    expect(updatedFocalPoint, const Offset(17.5, 27.5));
    expect(updatedScale, 0.5);
    expect(updatedHorizontalScale, 0.5);
    expect(updatedVerticalScale, 0.5);
    expect(updatedDelta, const Offset(7.5, 7.5));
    expect(updatedPointerCount, 2);
    expect(didTap, isFalse);

    // Horizontal scaling
    tester.route(pointer2.move(const Offset(0.0, 20.0)));
    expect(updatedHorizontalScale, 2.0);
    expect(updatedVerticalScale, 1.0);
    expect(updatedPointerCount, 2);

    // Vertical scaling
    tester.route(pointer2.move(const Offset(10.0, 10.0)));
    expect(updatedHorizontalScale, 1.0);
    expect(updatedVerticalScale, 2.0);
    expect(updatedDelta, const Offset(5.0, -5.0));
    expect(updatedPointerCount, 2);
    tester.route(pointer2.move(const Offset(15.0, 25.0)));
    updatedFocalPoint = null;
    updatedScale = null;
    updatedDelta = null;
    updatedPointerCount = null;

    // Three-finger scaling
    final TestPointer pointer3 = TestPointer(3);
    final PointerDownEvent down3 = pointer3.down(const Offset(25.0, 35.0));
    scale.addPointer(down3);
    tap.addPointer(down3);
    tester.closeArena(3);
    tester.route(down3);

    expect(didEndScale, isTrue);
    didEndScale = false;
    expect(updatedScale, isNull);
    expect(updatedFocalPoint, isNull);
    expect(updatedDelta, isNull);
    expect(updatedPointerCount, isNull);
    expect(didStartScale, isFalse);

    // Zoom in
    tester.route(pointer3.move(const Offset(55.0, 65.0)));
    expect(didStartScale, isTrue);
    didStartScale = false;
    expect(updatedFocalPoint, const Offset(30.0, 40.0));
    updatedFocalPoint = null;
    expect(updatedScale, 5.0);
    updatedScale = null;
    expect(updatedDelta, const Offset(10.0, 10.0));
    updatedDelta = null;
    expect(updatedPointerCount, 3);
    updatedPointerCount = null;
    expect(didEndScale, isFalse);
    expect(didTap, isFalse);

    // Return to original positions but with different fingers
    tester.route(pointer1.move(const Offset(25.0, 35.0)));
    tester.route(pointer2.move(const Offset(20.0, 30.0)));
    tester.route(pointer3.move(const Offset(15.0, 25.0)));
    expect(didStartScale, isFalse);
    expect(updatedFocalPoint, const Offset(20.0, 30.0));
    updatedFocalPoint = null;
    expect(updatedScale, 1.0);
    updatedScale = null;
    expect(updatedDelta!.dx, closeTo(-13.3, 0.1));
    expect(updatedDelta!.dy, closeTo(-13.3, 0.1));
    updatedDelta = null;
    expect(updatedPointerCount, 3);
    updatedPointerCount = null;
    expect(didEndScale, isFalse);
    expect(didTap, isFalse);

    tester.route(pointer1.up());
    expect(didStartScale, isFalse);
    expect(updatedFocalPoint, isNull);
    expect(updatedScale, isNull);
    expect(updatedDelta, isNull);
    expect(didEndScale, isTrue);
    didEndScale = false;
    expect(didTap, isFalse);

    // Continue scaling with two fingers
    tester.route(pointer3.move(const Offset(10.0, 20.0)));
    expect(didStartScale, isTrue);
    didStartScale = false;
    expect(updatedFocalPoint, const Offset(15.0, 25.0));
    updatedFocalPoint = null;
    expect(updatedScale, 2.0);
    updatedScale = null;
    expect(updatedDelta, const Offset(-2.5, -2.5));
    updatedDelta = null;
    expect(updatedPointerCount, 2);
    updatedPointerCount = null;

    // Continue rotating with two fingers
    tester.route(pointer3.move(const Offset(30.0, 40.0)));
    expect(updatedFocalPoint, const Offset(25.0, 35.0));
    updatedFocalPoint = null;
    expect(updatedScale, 2.0);
    updatedScale = null;
    expect(updatedDelta, const Offset(10.0, 10.0));
    updatedDelta = null;
    tester.route(pointer3.move(const Offset(10.0, 20.0)));
    expect(updatedFocalPoint, const Offset(15.0, 25.0));
    updatedFocalPoint = null;
    expect(updatedScale, 2.0);
    updatedScale = null;
    expect(updatedDelta, const Offset(-10.0, -10.0));
    updatedDelta = null;
    expect(updatedPointerCount, 2);
    updatedPointerCount = null;

    tester.route(pointer2.up());
    expect(didStartScale, isFalse);
    expect(updatedFocalPoint, isNull);
    expect(updatedScale, isNull);
    expect(updatedDelta, isNull);
    expect(didEndScale, isTrue);
    didEndScale = false;
    expect(didTap, isFalse);

    // Continue panning with one finger
    tester.route(pointer3.move(Offset.zero));
    expect(didStartScale, isTrue);
    didStartScale = false;
    expect(updatedFocalPoint, Offset.zero);
    updatedFocalPoint = null;
    expect(updatedScale, 1.0);
    updatedScale = null;
    expect(updatedDelta, const Offset(-10.0, -20.0));
    updatedDelta = null;
    expect(updatedPointerCount, 1);
    updatedPointerCount = null;

    // We are done
    tester.route(pointer3.up());
    expect(didStartScale, isFalse);
    expect(updatedFocalPoint, isNull);
    expect(updatedScale, isNull);
    expect(updatedDelta, isNull);
    expect(updatedPointerCount, isNull);
    expect(didEndScale, isTrue);
    didEndScale = false;
    expect(didTap, isFalse);

    scale.dispose();
    tap.dispose();
  });

  testGesture('Rejects scale gestures from unallowed device kinds', (GestureTester tester) {
    final ScaleGestureRecognizer scale = ScaleGestureRecognizer(
      supportedDevices: <PointerDeviceKind>{PointerDeviceKind.touch},
    );

    bool didStartScale = false;
    scale.onStart = (ScaleStartDetails details) {
      didStartScale = true;
    };

    double? updatedScale;
    scale.onUpdate = (ScaleUpdateDetails details) {
      updatedScale = details.scale;
    };

    final TestPointer mousePointer = TestPointer(1, PointerDeviceKind.mouse);

    final PointerDownEvent down = mousePointer.down(Offset.zero);
    scale.addPointer(down);
    tester.closeArena(1);

    // One-finger panning
    tester.route(down);
    expect(didStartScale, isFalse);
    expect(updatedScale, isNull);

    // Using a mouse, the scale gesture shouldn't even start.
    tester.route(mousePointer.move(const Offset(20.0, 30.0)));
    expect(didStartScale, isFalse);
    expect(updatedScale, isNull);

    scale.dispose();
  });

  testGesture(
    'Scale gestures starting from allowed device kinds cannot be ended from unallowed devices',
    (GestureTester tester) {
      final ScaleGestureRecognizer scale = ScaleGestureRecognizer(
        supportedDevices: <PointerDeviceKind>{PointerDeviceKind.touch},
      );

      bool didStartScale = false;
      Offset? updatedFocalPoint;
      scale.onStart = (ScaleStartDetails details) {
        didStartScale = true;
        updatedFocalPoint = details.focalPoint;
      };

      double? updatedScale;
      scale.onUpdate = (ScaleUpdateDetails details) {
        updatedScale = details.scale;
        updatedFocalPoint = details.focalPoint;
      };

      bool didEndScale = false;
      scale.onEnd = (ScaleEndDetails details) {
        didEndScale = true;
      };

      final TestPointer touchPointer = TestPointer();

      final PointerDownEvent down = touchPointer.down(Offset.zero);
      scale.addPointer(down);
      tester.closeArena(1);

      // One-finger panning
      tester.route(down);
      expect(didStartScale, isTrue);
      didStartScale = false;
      expect(updatedScale, isNull);
      expect(updatedFocalPoint, Offset.zero);
      expect(didEndScale, isFalse);

      // The gesture can start using one touch finger.
      tester.route(touchPointer.move(const Offset(20.0, 30.0)));
      expect(updatedFocalPoint, const Offset(20.0, 30.0));
      updatedFocalPoint = null;
      expect(updatedScale, 1.0);
      updatedScale = null;
      expect(didEndScale, isFalse);

      // Two-finger scaling
      final TestPointer mousePointer = TestPointer(2, PointerDeviceKind.mouse);
      final PointerDownEvent down2 = mousePointer.down(const Offset(10.0, 20.0));
      scale.addPointer(down2);
      tester.closeArena(2);
      tester.route(down2);

      // Mouse-generated events are ignored.
      expect(didEndScale, isFalse);
      expect(updatedScale, isNull);
      expect(didStartScale, isFalse);

      // Zoom in using a mouse doesn't work either.
      tester.route(mousePointer.move(const Offset(0.0, 10.0)));
      expect(updatedScale, isNull);
      expect(didEndScale, isFalse);

      scale.dispose();
    },
  );

  testGesture('Scale gesture competes with drag', (GestureTester tester) {
    final ScaleGestureRecognizer scale = ScaleGestureRecognizer();
    final HorizontalDragGestureRecognizer drag = HorizontalDragGestureRecognizer();

    final List<String> log = <String>[];

    scale.onStart = (ScaleStartDetails details) {
      log.add('scale-start');
    };
    scale.onUpdate = (ScaleUpdateDetails details) {
      log.add('scale-update');
    };
    scale.onEnd = (ScaleEndDetails details) {
      log.add('scale-end');
    };

    drag.onStart = (DragStartDetails details) {
      log.add('drag-start');
    };
    drag.onEnd = (DragEndDetails details) {
      log.add('drag-end');
    };

    final TestPointer pointer1 = TestPointer();

    final PointerDownEvent down = pointer1.down(const Offset(10.0, 10.0));
    scale.addPointer(down);
    drag.addPointer(down);

    tester.closeArena(1);
    expect(log, isEmpty);

    // Vertical moves are scales.
    tester.route(down);
    expect(log, isEmpty);

    // Scale will win if focal point delta exceeds 18.0*2.

    tester.route(pointer1.move(const Offset(10.0, 50.0))); // Delta of 40.0 exceeds 18.0*2.
    expect(log, equals(<String>['scale-start', 'scale-update']));
    log.clear();

    final TestPointer pointer2 = TestPointer(2);
    final PointerDownEvent down2 = pointer2.down(const Offset(10.0, 20.0));
    scale.addPointer(down2);
    drag.addPointer(down2);

    tester.closeArena(2);
    expect(log, isEmpty);

    // Second pointer joins scale even though it moves horizontally.
    tester.route(down2);
    expect(log, <String>['scale-end']);
    log.clear();

    tester.route(pointer2.move(const Offset(30.0, 20.0)));
    expect(log, equals(<String>['scale-start', 'scale-update']));
    log.clear();

    tester.route(pointer1.up());
    expect(log, equals(<String>['scale-end']));
    log.clear();

    tester.route(pointer2.up());
    expect(log, isEmpty);
    log.clear();

    // Horizontal moves are either drags or scales, depending on which wins first.
    // TODO(ianh): https://github.com/flutter/flutter/issues/11384
    // In this case, we move fast, so that the scale wins. If we moved slowly,
    // the horizontal drag would win, since it was added first.
    final TestPointer pointer3 = TestPointer(3);
    final PointerDownEvent down3 = pointer3.down(const Offset(30.0, 30.0));
    scale.addPointer(down3);
    drag.addPointer(down3);
    tester.closeArena(3);
    tester.route(down3);

    expect(log, isEmpty);

    tester.route(pointer3.move(const Offset(100.0, 30.0)));
    expect(log, equals(<String>['scale-start', 'scale-update']));
    log.clear();

    tester.route(pointer3.up());
    expect(log, equals(<String>['scale-end']));
    log.clear();

    scale.dispose();
    drag.dispose();
  });

  testGesture('Should recognize rotation gestures', (GestureTester tester) {
    final ScaleGestureRecognizer scale = ScaleGestureRecognizer();
    final TapGestureRecognizer tap = TapGestureRecognizer();

    bool didStartScale = false;
    Offset? updatedFocalPoint;
    int? updatedPointerCount;
    scale.onStart = (ScaleStartDetails details) {
      didStartScale = true;
      updatedFocalPoint = details.focalPoint;
      updatedPointerCount = details.pointerCount;
    };

    double? updatedRotation;
    Offset? updatedDelta;
    scale.onUpdate = (ScaleUpdateDetails details) {
      updatedRotation = details.rotation;
      updatedFocalPoint = details.focalPoint;
      updatedDelta = details.focalPointDelta;
      updatedPointerCount = details.pointerCount;
    };

    bool didEndScale = false;
    scale.onEnd = (ScaleEndDetails details) {
      didEndScale = true;
    };

    bool didTap = false;
    tap.onTap = () {
      didTap = true;
    };

    final TestPointer pointer1 = TestPointer();

    final PointerDownEvent down = pointer1.down(Offset.zero);
    scale.addPointer(down);
    tap.addPointer(down);

    tester.closeArena(1);
    expect(didStartScale, isFalse);
    expect(updatedRotation, isNull);
    expect(updatedFocalPoint, isNull);
    expect(updatedDelta, isNull);
    expect(didEndScale, isFalse);
    expect(didTap, isFalse);

    tester.route(down);
    tester.route(pointer1.move(const Offset(20.0, 30.0)));
    expect(didStartScale, isTrue);
    didStartScale = false;

    expect(updatedFocalPoint, const Offset(20.0, 30.0));
    updatedFocalPoint = null;
    expect(updatedDelta, const Offset(20.0, 30.0));
    updatedDelta = null;
    expect(updatedRotation, 0.0);
    updatedRotation = null;
    expect(updatedPointerCount, 1);
    updatedPointerCount = null;
    expect(didEndScale, isFalse);
    expect(didTap, isFalse);

    // Two-finger scaling
    final TestPointer pointer2 = TestPointer(2);
    final PointerDownEvent down2 = pointer2.down(const Offset(30.0, 40.0));
    scale.addPointer(down2);
    tap.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);

    expect(didEndScale, isTrue);
    didEndScale = false;
    expect(updatedFocalPoint, isNull);
    expect(updatedDelta, isNull);
    expect(updatedRotation, isNull);
    expect(updatedPointerCount, isNull);
    expect(didStartScale, isFalse);

    // Zoom in
    tester.route(pointer2.move(const Offset(40.0, 50.0)));
    expect(didStartScale, isTrue);
    didStartScale = false;
    expect(updatedFocalPoint, const Offset(30.0, 40.0));
    updatedFocalPoint = null;
    expect(updatedDelta, const Offset(5.0, 5.0));
    updatedDelta = null;
    expect(updatedRotation, 0.0);
    updatedRotation = null;
    expect(updatedPointerCount, 2);
    updatedPointerCount = null;
    expect(didEndScale, isFalse);
    expect(didTap, isFalse);

    // Rotation
    tester.route(pointer2.move(const Offset(0.0, 10.0)));
    expect(updatedFocalPoint, const Offset(10.0, 20.0));
    updatedFocalPoint = null;
    expect(updatedDelta, const Offset(-20.0, -20.0));
    updatedDelta = null;
    expect(updatedRotation, math.pi);
    updatedRotation = null;
    expect(updatedPointerCount, 2);
    updatedPointerCount = null;
    expect(didEndScale, isFalse);
    expect(didTap, isFalse);

    // Three-finger scaling
    final TestPointer pointer3 = TestPointer(3);
    final PointerDownEvent down3 = pointer3.down(const Offset(25.0, 35.0));
    scale.addPointer(down3);
    tap.addPointer(down3);
    tester.closeArena(3);
    tester.route(down3);

    expect(didEndScale, isTrue);
    didEndScale = false;
    expect(updatedFocalPoint, isNull);
    expect(updatedDelta, isNull);
    expect(updatedRotation, isNull);
    expect(updatedPointerCount, isNull);
    expect(didStartScale, isFalse);

    // Zoom in
    tester.route(pointer3.move(const Offset(55.0, 65.0)));
    expect(didStartScale, isTrue);
    didStartScale = false;
    expect(updatedFocalPoint, const Offset(25.0, 35.0));
    updatedFocalPoint = null;
    expect(updatedDelta, const Offset(10.0, 10.0));
    updatedDelta = null;
    expect(updatedRotation, 0.0);
    updatedRotation = null;
    expect(updatedPointerCount, 3);
    updatedPointerCount = null;
    expect(didEndScale, isFalse);
    expect(didTap, isFalse);

    // Return to original positions but with different fingers
    tester.route(pointer1.move(const Offset(25.0, 35.0)));
    tester.route(pointer2.move(const Offset(20.0, 30.0)));
    tester.route(pointer3.move(const Offset(15.0, 25.0)));
    expect(didStartScale, isFalse);
    expect(updatedFocalPoint, const Offset(20.0, 30.0));
    updatedFocalPoint = null;
    expect(updatedDelta!.dx, closeTo(-13.3, 0.1));
    expect(updatedDelta!.dy, closeTo(-13.3, 0.1));
    updatedDelta = null;
    expect(updatedRotation, 0.0);
    updatedRotation = null;
    expect(updatedPointerCount, 3);
    updatedPointerCount = null;
    expect(didEndScale, isFalse);
    expect(didTap, isFalse);

    tester.route(pointer1.up());
    expect(didStartScale, isFalse);
    expect(updatedFocalPoint, isNull);
    expect(updatedDelta, isNull);
    expect(updatedRotation, isNull);
    expect(didEndScale, isTrue);
    didEndScale = false;
    expect(didTap, isFalse);

    // Continue scaling with two fingers
    tester.route(pointer3.move(const Offset(10.0, 20.0)));
    expect(didStartScale, isTrue);
    didStartScale = false;
    expect(updatedFocalPoint, const Offset(15.0, 25.0));
    updatedFocalPoint = null;
    expect(updatedDelta, const Offset(-2.5, -2.5));
    updatedDelta = null;
    expect(updatedRotation, 0.0);
    updatedRotation = null;
    expect(updatedPointerCount, 2);
    updatedPointerCount = null;

    // Continue rotating with two fingers
    tester.route(pointer3.move(const Offset(30.0, 40.0)));
    expect(updatedFocalPoint, const Offset(25.0, 35.0));
    updatedFocalPoint = null;
    expect(updatedDelta, const Offset(10.0, 10.0));
    updatedDelta = null;
    expect(updatedRotation, -math.pi);
    updatedRotation = null;
    expect(updatedPointerCount, 2);
    updatedPointerCount = null;
    tester.route(pointer3.move(const Offset(10.0, 20.0)));
    expect(updatedFocalPoint, const Offset(15.0, 25.0));
    updatedFocalPoint = null;
    expect(updatedDelta, const Offset(-10.0, -10.0));
    updatedDelta = null;
    expect(updatedRotation, 0.0);
    updatedRotation = null;
    expect(updatedPointerCount, 2);
    updatedPointerCount = null;

    tester.route(pointer2.up());
    expect(didStartScale, isFalse);
    expect(updatedFocalPoint, isNull);
    expect(updatedDelta, isNull);
    expect(updatedRotation, isNull);
    expect(updatedPointerCount, isNull);
    expect(didEndScale, isTrue);
    didEndScale = false;
    expect(didTap, isFalse);

    // We are done
    tester.route(pointer3.up());
    expect(didStartScale, isFalse);
    expect(updatedFocalPoint, isNull);
    expect(updatedDelta, isNull);
    expect(updatedRotation, isNull);
    expect(updatedPointerCount, isNull);
    expect(didEndScale, isFalse);
    didEndScale = false;
    expect(didTap, isFalse);

    scale.dispose();
    tap.dispose();
  });

  // Regressing test for https://github.com/flutter/flutter/issues/78941
  testGesture('First rotation test', (GestureTester tester) {
    final ScaleGestureRecognizer scale = ScaleGestureRecognizer();
    addTearDown(scale.dispose);

    double? updatedRotation;
    scale.onUpdate = (ScaleUpdateDetails details) {
      updatedRotation = details.rotation;
    };

    final TestPointer pointer1 = TestPointer();
    final PointerDownEvent down = pointer1.down(Offset.zero);
    scale.addPointer(down);
    tester.closeArena(1);
    tester.route(down);

    final TestPointer pointer2 = TestPointer(2);
    final PointerDownEvent down2 = pointer2.down(const Offset(10.0, 10.0));
    scale.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);

    expect(updatedRotation, isNull);

    // Rotation 45°.
    tester.route(pointer2.move(const Offset(0.0, 10.0)));
    expect(updatedRotation, math.pi / 4.0);
  });

  testGesture('Scale gestures pointer count test', (GestureTester tester) {
    final ScaleGestureRecognizer scale = ScaleGestureRecognizer();

    int pointerCountOfStart = 0;
    scale.onStart = (ScaleStartDetails details) => pointerCountOfStart = details.pointerCount;

    int pointerCountOfUpdate = 0;
    scale.onUpdate = (ScaleUpdateDetails details) => pointerCountOfUpdate = details.pointerCount;

    int pointerCountOfEnd = 0;
    scale.onEnd = (ScaleEndDetails details) => pointerCountOfEnd = details.pointerCount;

    final TestPointer pointer1 = TestPointer();
    final PointerDownEvent down = pointer1.down(Offset.zero);
    scale.addPointer(down);
    tester.closeArena(1);

    // One-finger panning
    tester.route(down);
    // One pointer in contact with the screen now.
    expect(pointerCountOfStart, 1);
    tester.route(pointer1.move(const Offset(20.0, 30.0)));
    expect(pointerCountOfUpdate, 1);

    // Two-finger scaling
    final TestPointer pointer2 = TestPointer(2);
    final PointerDownEvent down2 = pointer2.down(const Offset(10.0, 20.0));
    scale.addPointer(down2);
    tester.closeArena(2);
    tester.route(down2);
    // Two pointers in contact with the screen now.
    expect(pointerCountOfEnd, 2); // Additional pointer down will trigger an end event.

    tester.route(pointer2.move(const Offset(0.0, 10.0)));
    expect(pointerCountOfStart, 2); // The new pointer move will trigger a start event.
    expect(pointerCountOfUpdate, 2);

    tester.route(pointer1.up());
    // One pointer in contact with the screen now.
    expect(pointerCountOfEnd, 1);

    tester.route(pointer2.move(const Offset(0.0, 10.0)));
    expect(pointerCountOfStart, 1);
    expect(pointerCountOfUpdate, 1);

    tester.route(pointer2.up());
    // No pointer in contact with the screen now.
    expect(pointerCountOfEnd, 0);

    scale.dispose();
  });

  testGesture('Should recognize scale gestures from pointer pan/zoom events', (
    GestureTester tester,
  ) {
    final ScaleGestureRecognizer scale = ScaleGestureRecognizer();
    addTearDown(scale.dispose);
    final HorizontalDragGestureRecognizer drag = HorizontalDragGestureRecognizer();
    addTearDown(drag.dispose);

    bool didStartScale = false;
    Offset? updatedFocalPoint;
    int? updatedPointerCount;
    scale.onStart = (ScaleStartDetails details) {
      didStartScale = true;
      updatedFocalPoint = details.focalPoint;
      updatedPointerCount = details.pointerCount;
    };

    double? updatedScale;
    double? updatedHorizontalScale;
    double? updatedVerticalScale;
    Offset? updatedDelta;
    scale.onUpdate = (ScaleUpdateDetails details) {
      updatedScale = details.scale;
      updatedHorizontalScale = details.horizontalScale;
      updatedVerticalScale = details.verticalScale;
      updatedFocalPoint = details.focalPoint;
      updatedDelta = details.focalPointDelta;
      updatedPointerCount = details.pointerCount;
    };

    bool didEndScale = false;
    scale.onEnd = (ScaleEndDetails details) {
      didEndScale = true;
    };

    final TestPointer pointer1 = TestPointer(2, PointerDeviceKind.trackpad);

    final PointerPanZoomStartEvent start = pointer1.panZoomStart(Offset.zero);
    scale.addPointerPanZoom(start);
    drag.addPointerPanZoom(start);

    tester.closeArena(2);
    expect(didStartScale, isFalse);
    expect(updatedScale, isNull);
    expect(updatedFocalPoint, isNull);
    expect(updatedDelta, isNull);
    expect(updatedPointerCount, isNull);
    expect(didEndScale, isFalse);

    // Panning.
    tester.route(start);
    expect(didStartScale, isFalse);
    expect(updatedScale, isNull);
    expect(updatedFocalPoint, isNull);
    expect(updatedDelta, isNull);
    expect(updatedPointerCount, isNull);
    expect(didEndScale, isFalse);

    tester.route(pointer1.panZoomUpdate(Offset.zero, pan: const Offset(20.0, 30.0)));
    expect(didStartScale, isTrue);
    didStartScale = false;
    expect(updatedFocalPoint, const Offset(20.0, 30.0));
    updatedFocalPoint = null;
    expect(updatedScale, 1.0);
    updatedScale = null;
    expect(updatedDelta, const Offset(20.0, 30.0));
    updatedDelta = null;
    expect(updatedPointerCount, 2);
    updatedPointerCount = null;
    expect(didEndScale, isFalse);

    // Zoom in.
    tester.route(pointer1.panZoomUpdate(Offset.zero, pan: const Offset(20.0, 30.0), scale: 2.0));
    expect(updatedFocalPoint, const Offset(20.0, 30.0));
    updatedFocalPoint = null;
    expect(updatedScale, 2.0);
    expect(updatedHorizontalScale, 2.0);
    expect(updatedVerticalScale, 2.0);
    expect(updatedDelta, Offset.zero);
    expect(updatedPointerCount, 2);
    updatedScale = null;
    updatedHorizontalScale = null;
    updatedVerticalScale = null;
    updatedDelta = null;
    updatedPointerCount = null;
    expect(didEndScale, isFalse);

    // Zoom out.
    tester.route(pointer1.panZoomUpdate(Offset.zero, pan: const Offset(20.0, 30.0)));
    expect(updatedFocalPoint, const Offset(20.0, 30.0));
    updatedFocalPoint = null;
    expect(updatedScale, 1.0);
    expect(updatedHorizontalScale, 1.0);
    expect(updatedVerticalScale, 1.0);
    expect(updatedDelta, Offset.zero);
    expect(updatedPointerCount, 2);
    updatedScale = null;
    updatedHorizontalScale = null;
    updatedVerticalScale = null;
    updatedDelta = null;
    updatedPointerCount = null;
    expect(didEndScale, isFalse);

    // We are done.
    tester.route(pointer1.panZoomEnd());
    expect(didStartScale, isFalse);
    expect(updatedFocalPoint, isNull);
    expect(updatedScale, isNull);
    expect(updatedDelta, isNull);
    expect(updatedPointerCount, isNull);
    expect(didEndScale, isTrue);
    didEndScale = false;

    scale.dispose();
  });

  testGesture('Pointer pan/zooms should work alongside touches', (GestureTester tester) {
    final ScaleGestureRecognizer scale = ScaleGestureRecognizer();
    addTearDown(scale.dispose);
    final HorizontalDragGestureRecognizer drag = HorizontalDragGestureRecognizer();
    addTearDown(drag.dispose);

    bool didStartScale = false;
    Offset? updatedFocalPoint;
    int? updatedPointerCount;
    scale.onStart = (ScaleStartDetails details) {
      didStartScale = true;
      updatedFocalPoint = details.focalPoint;
      updatedPointerCount = details.pointerCount;
    };

    double? updatedScale;
    double? updatedHorizontalScale;
    double? updatedVerticalScale;
    Offset? updatedDelta;
    double? updatedRotation;
    scale.onUpdate = (ScaleUpdateDetails details) {
      updatedScale = details.scale;
      updatedHorizontalScale = details.horizontalScale;
      updatedVerticalScale = details.verticalScale;
      updatedFocalPoint = details.focalPoint;
      updatedDelta = details.focalPointDelta;
      updatedRotation = details.rotation;
      updatedPointerCount = details.pointerCount;
    };

    bool didEndScale = false;
    scale.onEnd = (ScaleEndDetails details) {
      didEndScale = true;
    };

    final TestPointer touchPointer1 = TestPointer(2);
    final TestPointer touchPointer2 = TestPointer(3);
    final TestPointer panZoomPointer = TestPointer(4, PointerDeviceKind.trackpad);

    final PointerPanZoomStartEvent panZoomStart = panZoomPointer.panZoomStart(Offset.zero);
    scale.addPointerPanZoom(panZoomStart);
    drag.addPointerPanZoom(panZoomStart);

    tester.closeArena(4);
    expect(didStartScale, isFalse);
    expect(updatedScale, isNull);
    expect(updatedFocalPoint, isNull);
    expect(updatedDelta, isNull);
    expect(updatedPointerCount, isNull);
    expect(didEndScale, isFalse);

    // Panning starting with trackpad.
    tester.route(panZoomStart);
    expect(didStartScale, isFalse);
    expect(updatedScale, isNull);
    expect(updatedFocalPoint, isNull);
    expect(updatedDelta, isNull);
    expect(updatedPointerCount, isNull);
    expect(didEndScale, isFalse);

    tester.route(panZoomPointer.panZoomUpdate(Offset.zero, pan: const Offset(40, 40)));
    expect(didStartScale, isTrue);
    didStartScale = false;
    expect(updatedFocalPoint, const Offset(40.0, 40.0));
    updatedFocalPoint = null;
    expect(updatedScale, 1.0);
    updatedScale = null;
    expect(updatedDelta, const Offset(40.0, 40.0));
    updatedDelta = null;
    expect(updatedPointerCount, 2);
    updatedPointerCount = null;
    expect(didEndScale, isFalse);

    // Add a touch pointer.
    final PointerDownEvent touchStart1 = touchPointer1.down(const Offset(40, 40));
    scale.addPointer(touchStart1);
    drag.addPointer(touchStart1);
    tester.closeArena(2);
    tester.route(touchStart1);
    expect(didEndScale, isTrue);
    didEndScale = false;

    tester.route(touchPointer1.move(const Offset(10, 10)));
    expect(didStartScale, isTrue);
    didStartScale = false;
    expect(updatedFocalPoint, const Offset(25, 25));
    updatedFocalPoint = null;
    // 1 down pointer + pointer pan/zoom should not scale, only pan.
    expect(updatedScale, 1.0);
    updatedScale = null;
    expect(updatedDelta, const Offset(-15, -15));
    updatedDelta = null;
    expect(updatedPointerCount, 3);
    updatedPointerCount = null;
    expect(didEndScale, isFalse);

    // Add a second touch pointer.
    final PointerDownEvent touchStart2 = touchPointer2.down(const Offset(10, 40));
    scale.addPointer(touchStart2);
    drag.addPointer(touchStart2);
    tester.closeArena(3);
    tester.route(touchStart2);
    expect(didEndScale, isTrue);
    didEndScale = false;

    // Move the second pointer to cause pan, zoom, and rotation.
    tester.route(touchPointer2.move(const Offset(40, 40)));
    expect(didStartScale, isTrue);
    didStartScale = false;
    expect(updatedFocalPoint, const Offset(30, 30));
    updatedFocalPoint = null;
    expect(updatedScale, math.sqrt(2));
    updatedScale = null;
    expect(updatedHorizontalScale, 1.0);
    updatedHorizontalScale = null;
    expect(updatedVerticalScale, 1.0);
    updatedVerticalScale = null;
    expect(updatedDelta, const Offset(10, 0));
    updatedDelta = null;
    expect(updatedRotation, -math.pi / 4);
    updatedRotation = null;
    expect(updatedPointerCount, 4);
    updatedPointerCount = null;
    expect(didEndScale, isFalse);

    // Change the scale and angle of the pan/zoom to test combining.
    // Scale should be multiplied together.
    // Rotation angle should be added together.
    tester.route(
      panZoomPointer.panZoomUpdate(
        Offset.zero,
        pan: const Offset(40, 40),
        scale: math.sqrt(2),
        rotation: math.pi / 3,
      ),
    );
    expect(didStartScale, isFalse);
    expect(updatedFocalPoint, const Offset(30, 30));
    updatedFocalPoint = null;
    expect(updatedScale, closeTo(2, 0.0001));
    updatedScale = null;
    expect(updatedHorizontalScale, math.sqrt(2));
    updatedHorizontalScale = null;
    expect(updatedVerticalScale, math.sqrt(2));
    updatedVerticalScale = null;
    expect(updatedDelta, Offset.zero);
    updatedDelta = null;
    expect(updatedRotation, closeTo(math.pi / 12, 0.0001));
    updatedRotation = null;
    expect(updatedPointerCount, 4);
    updatedPointerCount = null;
    expect(didEndScale, isFalse);

    // Move the pan/zoom origin to test combining.
    tester.route(
      panZoomPointer.panZoomUpdate(
        const Offset(15, 15),
        pan: const Offset(55, 55),
        scale: math.sqrt(2),
        rotation: math.pi / 3,
      ),
    );
    expect(didStartScale, isFalse);
    expect(updatedFocalPoint, const Offset(40, 40));
    updatedFocalPoint = null;
    expect(updatedScale, closeTo(2, 0.0001));
    updatedScale = null;
    expect(updatedDelta, const Offset(10, 10));
    updatedDelta = null;
    expect(updatedRotation, closeTo(math.pi / 12, 0.0001));
    updatedRotation = null;
    expect(updatedPointerCount, 4);
    updatedPointerCount = null;
    expect(didEndScale, isFalse);

    // We are done.
    tester.route(panZoomPointer.panZoomEnd());
    expect(updatedFocalPoint, isNull);
    expect(didEndScale, isTrue);
    didEndScale = false;
    expect(updatedScale, isNull);
    expect(updatedDelta, isNull);
    expect(updatedPointerCount, isNull);
    expect(didStartScale, isFalse);
    tester.route(touchPointer1.up());
    expect(updatedFocalPoint, isNull);
    expect(didEndScale, isFalse);
    expect(updatedScale, isNull);
    expect(updatedDelta, isNull);
    expect(updatedPointerCount, isNull);
    expect(didStartScale, isFalse);
    tester.route(touchPointer2.up());
    expect(didEndScale, isFalse);
    expect(updatedFocalPoint, isNull);
    expect(updatedScale, isNull);
    expect(updatedDelta, isNull);
    expect(updatedPointerCount, isNull);
    expect(didStartScale, isFalse);

    scale.dispose();
  });

  testGesture('Scale gesture competes with drag for trackpad gesture', (GestureTester tester) {
    final ScaleGestureRecognizer scale = ScaleGestureRecognizer();
    final HorizontalDragGestureRecognizer drag = HorizontalDragGestureRecognizer();

    final List<String> log = <String>[];

    scale.onStart = (ScaleStartDetails details) {
      log.add('scale-start');
    };
    scale.onUpdate = (ScaleUpdateDetails details) {
      log.add('scale-update');
    };
    scale.onEnd = (ScaleEndDetails details) {
      log.add('scale-end');
    };

    drag.onStart = (DragStartDetails details) {
      log.add('drag-start');
    };
    drag.onEnd = (DragEndDetails details) {
      log.add('drag-end');
    };

    final TestPointer pointer1 = TestPointer(2, PointerDeviceKind.trackpad);

    final PointerPanZoomStartEvent down = pointer1.panZoomStart(const Offset(10.0, 10.0));
    scale.addPointerPanZoom(down);
    drag.addPointerPanZoom(down);

    tester.closeArena(2);
    expect(log, isEmpty);

    // Vertical moves are scales.
    tester.route(down);
    expect(log, isEmpty);

    // Scale will win if focal point delta exceeds 18.0*2.

    tester.route(
      pointer1.panZoomUpdate(const Offset(10.0, 10.0), pan: const Offset(10.0, 40.0)),
    ); // delta of 40.0 exceeds 18.0*2.
    expect(log, equals(<String>['scale-start', 'scale-update']));
    log.clear();

    final TestPointer pointer2 = TestPointer(3, PointerDeviceKind.trackpad);
    final PointerPanZoomStartEvent down2 = pointer2.panZoomStart(const Offset(10.0, 20.0));
    scale.addPointerPanZoom(down2);
    drag.addPointerPanZoom(down2);

    tester.closeArena(3);
    expect(log, isEmpty);

    // Second pointer joins scale even though it moves horizontally.
    tester.route(down2);
    expect(log, <String>['scale-end']);
    log.clear();

    tester.route(pointer2.panZoomUpdate(const Offset(10.0, 20.0), pan: const Offset(20.0, 0.0)));
    expect(log, equals(<String>['scale-start', 'scale-update']));
    log.clear();

    tester.route(pointer1.panZoomEnd());
    expect(log, equals(<String>['scale-end']));
    log.clear();

    tester.route(pointer2.panZoomEnd());
    expect(log, isEmpty);
    log.clear();

    // Horizontal moves are either drags or scales, depending on which wins first.
    // TODO(ianh): https://github.com/flutter/flutter/issues/11384
    // In this case, we move fast, so that the scale wins. If we moved slowly,
    // the horizontal drag would win, since it was added first.
    final TestPointer pointer3 = TestPointer(4, PointerDeviceKind.trackpad);
    final PointerPanZoomStartEvent down3 = pointer3.panZoomStart(const Offset(30.0, 30.0));
    scale.addPointerPanZoom(down3);
    drag.addPointerPanZoom(down3);
    tester.closeArena(4);
    tester.route(down3);

    expect(log, isEmpty);

    tester.route(pointer3.panZoomUpdate(const Offset(30.0, 30.0), pan: const Offset(70.0, 0.0)));
    expect(log, equals(<String>['scale-start', 'scale-update']));
    log.clear();

    tester.route(pointer3.panZoomEnd());
    expect(log, equals(<String>['scale-end']));
    log.clear();

    scale.dispose();
    drag.dispose();
  });

  testGesture('Scale gesture from pan/zoom events properly handles DragStartBehavior.start', (
    GestureTester tester,
  ) {
    final ScaleGestureRecognizer scale = ScaleGestureRecognizer(
      dragStartBehavior: DragStartBehavior.start,
    );
    addTearDown(scale.dispose);
    final HorizontalDragGestureRecognizer drag = HorizontalDragGestureRecognizer();
    addTearDown(drag.dispose);

    bool didStartScale = false;
    Offset? updatedFocalPoint;
    scale.onStart = (ScaleStartDetails details) {
      didStartScale = true;
      updatedFocalPoint = details.focalPoint;
    };

    double? updatedScale;
    double? updatedHorizontalScale;
    double? updatedVerticalScale;
    double? updatedRotation;
    Offset? updatedDelta;
    scale.onUpdate = (ScaleUpdateDetails details) {
      updatedScale = details.scale;
      updatedHorizontalScale = details.horizontalScale;
      updatedVerticalScale = details.verticalScale;
      updatedFocalPoint = details.focalPoint;
      updatedRotation = details.rotation;
      updatedDelta = details.focalPointDelta;
    };

    bool didEndScale = false;
    scale.onEnd = (ScaleEndDetails details) {
      didEndScale = true;
    };

    final TestPointer pointer1 = TestPointer(2, PointerDeviceKind.trackpad);

    final PointerPanZoomStartEvent start = pointer1.panZoomStart(Offset.zero);
    scale.addPointerPanZoom(start);
    drag.addPointerPanZoom(start);

    tester.closeArena(2);
    expect(didStartScale, isFalse);
    expect(updatedScale, isNull);
    expect(updatedFocalPoint, isNull);
    expect(updatedDelta, isNull);
    expect(didEndScale, isFalse);

    tester.route(start);
    expect(didStartScale, isFalse);
    expect(updatedScale, isNull);
    expect(updatedFocalPoint, isNull);
    expect(updatedDelta, isNull);
    expect(didEndScale, isFalse);

    // Zoom enough to win the gesture.
    tester.route(pointer1.panZoomUpdate(Offset.zero, scale: 1.1, rotation: 1));
    expect(didStartScale, isTrue);
    didStartScale = false;
    expect(updatedFocalPoint, Offset.zero);
    updatedFocalPoint = null;
    expect(updatedScale, 1.0);
    updatedScale = null;
    expect(updatedDelta, Offset.zero);
    updatedDelta = null;
    expect(didEndScale, isFalse);

    // Zoom in - should be relative to 1.1.
    tester.route(pointer1.panZoomUpdate(Offset.zero, scale: 1.21, rotation: 1.5));
    expect(updatedFocalPoint, Offset.zero);
    updatedFocalPoint = null;
    expect(updatedScale, closeTo(1.1, 0.0001));
    expect(updatedHorizontalScale, closeTo(1.1, 0.0001));
    expect(updatedVerticalScale, closeTo(1.1, 0.0001));
    expect(updatedRotation, 0.5);
    expect(updatedDelta, Offset.zero);
    updatedScale = null;
    updatedHorizontalScale = null;
    updatedVerticalScale = null;
    updatedRotation = null;
    updatedDelta = null;
    expect(didEndScale, isFalse);

    // Zoom out - should be relative to 1.1.
    tester.route(pointer1.panZoomUpdate(Offset.zero, scale: 0.99, rotation: 1.0));
    expect(updatedFocalPoint, Offset.zero);
    updatedFocalPoint = null;
    expect(updatedScale, closeTo(0.9, 0.0001));
    expect(updatedHorizontalScale, closeTo(0.9, 0.0001));
    expect(updatedVerticalScale, closeTo(0.9, 0.0001));
    expect(updatedRotation, 0.0);
    expect(updatedDelta, Offset.zero);
    updatedScale = null;
    updatedHorizontalScale = null;
    updatedVerticalScale = null;
    updatedDelta = null;
    expect(didEndScale, isFalse);

    // We are done.
    tester.route(pointer1.panZoomEnd());
    expect(didStartScale, isFalse);
    expect(updatedFocalPoint, isNull);
    expect(updatedScale, isNull);
    expect(updatedDelta, isNull);
    expect(didEndScale, isTrue);
    didEndScale = false;
  });

  testGesture('scale trackpadScrollCausesScale', (GestureTester tester) {
    final ScaleGestureRecognizer scale = ScaleGestureRecognizer(
      dragStartBehavior: DragStartBehavior.start,
      trackpadScrollCausesScale: true,
    );

    bool didStartScale = false;
    Offset? updatedFocalPoint;
    int? updatedPointerCount;
    scale.onStart = (ScaleStartDetails details) {
      didStartScale = true;
      updatedFocalPoint = details.focalPoint;
      updatedPointerCount = details.pointerCount;
    };

    double? updatedScale;
    Offset? updatedDelta;
    scale.onUpdate = (ScaleUpdateDetails details) {
      updatedScale = details.scale;
      updatedFocalPoint = details.focalPoint;
      updatedDelta = details.focalPointDelta;
      updatedPointerCount = details.pointerCount;
    };

    bool didEndScale = false;
    scale.onEnd = (ScaleEndDetails details) {
      didEndScale = true;
    };

    final TestPointer pointer1 = TestPointer(2, PointerDeviceKind.trackpad);

    final PointerPanZoomStartEvent start = pointer1.panZoomStart(Offset.zero);
    scale.addPointerPanZoom(start);

    tester.closeArena(2);
    expect(didStartScale, isFalse);
    expect(updatedScale, isNull);
    expect(updatedFocalPoint, isNull);
    expect(updatedDelta, isNull);
    expect(updatedPointerCount, isNull);
    expect(didEndScale, isFalse);

    tester.route(start);
    expect(didStartScale, isTrue);
    didStartScale = false;
    expect(updatedScale, isNull);
    expect(updatedFocalPoint, Offset.zero);
    updatedFocalPoint = null;
    expect(updatedDelta, isNull);
    expect(updatedPointerCount, 2);
    updatedPointerCount = null;
    expect(didEndScale, isFalse);

    // Zoom in by scrolling up.
    tester.route(pointer1.panZoomUpdate(Offset.zero, pan: const Offset(0, -200)));
    expect(didStartScale, isFalse);
    expect(updatedFocalPoint, Offset.zero);
    updatedFocalPoint = null;
    expect(updatedScale, math.e);
    updatedScale = null;
    expect(updatedDelta, Offset.zero);
    updatedDelta = null;
    expect(updatedPointerCount, 2);
    updatedPointerCount = null;
    expect(didEndScale, isFalse);

    // A horizontal scroll should do nothing.
    tester.route(pointer1.panZoomUpdate(Offset.zero, pan: const Offset(200, -200)));
    expect(didStartScale, isFalse);
    expect(updatedFocalPoint, Offset.zero);
    updatedFocalPoint = null;
    expect(updatedScale, math.e);
    updatedScale = null;
    expect(updatedDelta, Offset.zero);
    updatedDelta = null;
    expect(updatedPointerCount, 2);
    updatedPointerCount = null;
    expect(didEndScale, isFalse);

    // End.
    tester.route(pointer1.panZoomEnd());
    expect(didStartScale, isFalse);
    expect(updatedFocalPoint, isNull);
    expect(updatedScale, isNull);
    expect(updatedDelta, isNull);
    expect(updatedPointerCount, isNull);
    expect(didEndScale, isTrue);
    didEndScale = false;

    // Try with a different trackpadScrollToScaleFactor
    scale.trackpadScrollToScaleFactor = const Offset(1 / 125, 0);

    final PointerPanZoomStartEvent start2 = pointer1.panZoomStart(Offset.zero);
    scale.addPointerPanZoom(start2);

    tester.closeArena(2);
    expect(didStartScale, isFalse);
    expect(updatedScale, isNull);
    expect(updatedFocalPoint, isNull);
    expect(updatedDelta, isNull);
    expect(updatedPointerCount, isNull);
    expect(didEndScale, isFalse);

    tester.route(start2);
    expect(didStartScale, isTrue);
    didStartScale = false;
    expect(updatedScale, isNull);
    expect(updatedFocalPoint, Offset.zero);
    updatedFocalPoint = null;
    expect(updatedDelta, isNull);
    expect(updatedPointerCount, 2);
    updatedPointerCount = null;
    expect(didEndScale, isFalse);

    // Zoom in by scrolling left.
    tester.route(pointer1.panZoomUpdate(Offset.zero, pan: const Offset(125, 0)));
    expect(didStartScale, isFalse);
    didStartScale = false;
    expect(updatedFocalPoint, Offset.zero);
    updatedFocalPoint = null;
    expect(updatedScale, math.e);
    updatedScale = null;
    expect(updatedDelta, Offset.zero);
    updatedDelta = null;
    expect(updatedPointerCount, 2);
    updatedPointerCount = null;
    expect(didEndScale, isFalse);

    // A vertical scroll should do nothing.
    tester.route(pointer1.panZoomUpdate(Offset.zero, pan: const Offset(125, 125)));
    expect(didStartScale, isFalse);
    expect(updatedFocalPoint, Offset.zero);
    updatedFocalPoint = null;
    expect(updatedScale, math.e);
    updatedScale = null;
    expect(updatedDelta, Offset.zero);
    updatedDelta = null;
    expect(updatedPointerCount, 2);
    updatedPointerCount = null;
    expect(didEndScale, isFalse);

    // End.
    tester.route(pointer1.panZoomEnd());
    expect(didStartScale, isFalse);
    expect(updatedFocalPoint, isNull);
    expect(updatedScale, isNull);
    expect(updatedDelta, isNull);
    expect(updatedPointerCount, isNull);
    expect(didEndScale, isTrue);
    didEndScale = false;

    scale.dispose();
  });

  testGesture('scale ending velocity', (GestureTester tester) {
    final ScaleGestureRecognizer scale = ScaleGestureRecognizer(
      dragStartBehavior: DragStartBehavior.start,
      trackpadScrollCausesScale: true,
    );

    bool didStartScale = false;
    Offset? updatedFocalPoint;
    scale.onStart = (ScaleStartDetails details) {
      didStartScale = true;
      updatedFocalPoint = details.focalPoint;
    };

    bool didEndScale = false;
    double? scaleEndVelocity;
    scale.onEnd = (ScaleEndDetails details) {
      didEndScale = true;
      scaleEndVelocity = details.scaleVelocity;
    };

    final TestPointer pointer1 = TestPointer(2, PointerDeviceKind.trackpad);

    final PointerPanZoomStartEvent start = pointer1.panZoomStart(Offset.zero);
    scale.addPointerPanZoom(start);

    tester.closeArena(2);
    expect(didStartScale, isFalse);
    expect(updatedFocalPoint, isNull);
    expect(didEndScale, isFalse);

    tester.route(start);
    expect(didStartScale, isTrue);
    didStartScale = false;
    expect(updatedFocalPoint, Offset.zero);
    updatedFocalPoint = null;
    expect(didEndScale, isFalse);

    // Zoom in by scrolling up.
    for (int i = 0; i < 100; i++) {
      tester.route(
        pointer1.panZoomUpdate(
          Offset.zero,
          pan: Offset(0, i * -10),
          timeStamp: Duration(milliseconds: i * 25),
        ),
      );
    }

    // End.
    tester.route(pointer1.panZoomEnd(timeStamp: const Duration(milliseconds: 2500)));
    expect(didStartScale, isFalse);
    expect(updatedFocalPoint, isNull);
    expect(didEndScale, isTrue);
    didEndScale = false;
    expect(scaleEndVelocity, moreOrLessEquals(281.41454098027765));

    scale.dispose();
  });

  testGesture(
    'ScaleStartDetails and ScaleUpdateDetails callbacks should contain their event.timestamp',
    (GestureTester tester) {
      final ScaleGestureRecognizer scale = ScaleGestureRecognizer();
      final TapGestureRecognizer tap = TapGestureRecognizer();

      bool didStartScale = false;
      Offset? updatedFocalPoint;
      Duration? initialSourceTimestamp;
      scale.onStart = (ScaleStartDetails details) {
        didStartScale = true;
        updatedFocalPoint = details.focalPoint;
        initialSourceTimestamp = details.sourceTimeStamp;
      };

      double? updatedScale;
      double? updatedHorizontalScale;
      double? updatedVerticalScale;
      Offset? updatedDelta;
      Duration? updatedSourceTimestamp;
      scale.onUpdate = (ScaleUpdateDetails details) {
        updatedScale = details.scale;
        updatedHorizontalScale = details.horizontalScale;
        updatedVerticalScale = details.verticalScale;
        updatedFocalPoint = details.focalPoint;
        updatedDelta = details.focalPointDelta;
        updatedSourceTimestamp = details.sourceTimeStamp;
      };

      bool didEndScale = false;
      scale.onEnd = (ScaleEndDetails details) {
        didEndScale = true;
      };

      bool didTap = false;
      tap.onTap = () {
        didTap = true;
      };

      final TestPointer pointer1 = TestPointer();

      final PointerDownEvent down = pointer1.down(
        Offset.zero,
        timeStamp: const Duration(milliseconds: 10),
      );
      scale.addPointer(down);
      tap.addPointer(down);

      tester.closeArena(1);
      expect(didStartScale, isFalse);
      expect(updatedScale, isNull);
      expect(updatedFocalPoint, isNull);
      expect(updatedDelta, isNull);
      expect(updatedSourceTimestamp, isNull);
      expect(didEndScale, isFalse);
      expect(didTap, isFalse);
      expect(initialSourceTimestamp, isNull);

      // One-finger panning.
      tester.route(down);
      expect(didStartScale, isFalse);
      expect(updatedScale, isNull);
      expect(updatedFocalPoint, isNull);
      expect(updatedDelta, isNull);
      expect(didEndScale, isFalse);
      expect(didTap, isFalse);
      expect(initialSourceTimestamp, isNull);

      tester.route(
        pointer1.move(const Offset(20.0, 30.0), timeStamp: const Duration(milliseconds: 20)),
      );
      expect(didStartScale, isTrue);
      didStartScale = false;
      expect(updatedFocalPoint, const Offset(20.0, 30.0));
      updatedFocalPoint = null;
      expect(updatedScale, 1.0);
      updatedScale = null;
      expect(updatedDelta, const Offset(20.0, 30.0));
      updatedDelta = null;
      expect(updatedSourceTimestamp, const Duration(milliseconds: 20));
      updatedSourceTimestamp = null;
      expect(initialSourceTimestamp, const Duration(milliseconds: 10));
      initialSourceTimestamp = null;
      expect(didEndScale, isFalse);
      expect(didTap, isFalse);
      expect(scale.pointerCount, 1);

      // Two-finger scaling.
      final TestPointer pointer2 = TestPointer(2);
      final PointerDownEvent down2 = pointer2.down(
        const Offset(10.0, 20.0),
        timeStamp: const Duration(milliseconds: 30),
      );
      scale.addPointer(down2);
      tap.addPointer(down2);
      tester.closeArena(2);
      tester.route(down2);
      expect(scale.pointerCount, 2);

      expect(didEndScale, isTrue);
      didEndScale = false;
      expect(updatedScale, isNull);
      expect(updatedFocalPoint, isNull);
      expect(updatedDelta, isNull);
      expect(updatedSourceTimestamp, isNull);
      expect(didStartScale, isFalse);
      expect(initialSourceTimestamp, isNull);

      // Zoom in.
      tester.route(
        pointer2.move(const Offset(0.0, 10.0), timeStamp: const Duration(milliseconds: 40)),
      );
      expect(didStartScale, isTrue);
      didStartScale = false;
      expect(updatedFocalPoint, const Offset(10.0, 20.0));
      updatedFocalPoint = null;
      expect(updatedScale, 2.0);
      expect(updatedHorizontalScale, 2.0);
      expect(updatedVerticalScale, 2.0);
      expect(updatedDelta, const Offset(-5.0, -5.0));
      expect(updatedSourceTimestamp, const Duration(milliseconds: 40));
      expect(initialSourceTimestamp, const Duration(milliseconds: 40));
      updatedScale = null;
      updatedHorizontalScale = null;
      updatedVerticalScale = null;
      updatedDelta = null;
      updatedSourceTimestamp = null;
      initialSourceTimestamp = null;
      expect(didEndScale, isFalse);
      expect(didTap, isFalse);

      // Zoom out.
      tester.route(
        pointer2.move(const Offset(15.0, 25.0), timeStamp: const Duration(milliseconds: 50)),
      );
      expect(updatedFocalPoint, const Offset(17.5, 27.5));
      expect(updatedScale, 0.5);
      expect(updatedHorizontalScale, 0.5);
      expect(updatedVerticalScale, 0.5);
      expect(updatedDelta, const Offset(7.5, 7.5));
      expect(updatedSourceTimestamp, const Duration(milliseconds: 50));
      expect(didTap, isFalse);
      expect(initialSourceTimestamp, isNull);

      // Horizontal scaling.
      tester.route(
        pointer2.move(const Offset(0.0, 20.0), timeStamp: const Duration(milliseconds: 60)),
      );
      expect(updatedHorizontalScale, 2.0);
      expect(updatedVerticalScale, 1.0);
      expect(updatedSourceTimestamp, const Duration(milliseconds: 60));
      expect(initialSourceTimestamp, isNull);

      // Vertical scaling.
      tester.route(
        pointer2.move(const Offset(10.0, 10.0), timeStamp: const Duration(milliseconds: 70)),
      );
      expect(updatedHorizontalScale, 1.0);
      expect(updatedVerticalScale, 2.0);
      expect(updatedDelta, const Offset(5.0, -5.0));
      expect(updatedSourceTimestamp, const Duration(milliseconds: 70));
      expect(initialSourceTimestamp, isNull);
      tester.route(pointer2.move(const Offset(15.0, 25.0)));
      updatedFocalPoint = null;
      updatedScale = null;
      updatedDelta = null;
      updatedSourceTimestamp = null;

      // Three-finger scaling.
      final TestPointer pointer3 = TestPointer(3);
      final PointerDownEvent down3 = pointer3.down(
        const Offset(25.0, 35.0),
        timeStamp: const Duration(milliseconds: 80),
      );
      scale.addPointer(down3);
      tap.addPointer(down3);
      tester.closeArena(3);
      tester.route(down3);

      expect(didEndScale, isTrue);
      didEndScale = false;
      expect(updatedScale, isNull);
      expect(updatedFocalPoint, isNull);
      expect(updatedDelta, isNull);
      expect(didStartScale, isFalse);
      expect(initialSourceTimestamp, isNull);

      // Zoom in.
      tester.route(
        pointer3.move(const Offset(55.0, 65.0), timeStamp: const Duration(milliseconds: 90)),
      );
      expect(didStartScale, isTrue);
      didStartScale = false;
      expect(updatedFocalPoint, const Offset(30.0, 40.0));
      updatedFocalPoint = null;
      expect(updatedScale, 5.0);
      updatedScale = null;
      expect(updatedDelta, const Offset(10.0, 10.0));
      updatedDelta = null;
      expect(updatedSourceTimestamp, const Duration(milliseconds: 90));
      updatedSourceTimestamp = null;
      expect(initialSourceTimestamp, const Duration(milliseconds: 90));
      initialSourceTimestamp = null;
      expect(didEndScale, isFalse);
      expect(didTap, isFalse);

      // Return to original positions but with different fingers.
      tester.route(
        pointer1.move(const Offset(25.0, 35.0), timeStamp: const Duration(milliseconds: 100)),
      );
      tester.route(
        pointer2.move(const Offset(20.0, 30.0), timeStamp: const Duration(milliseconds: 110)),
      );
      tester.route(
        pointer3.move(const Offset(15.0, 25.0), timeStamp: const Duration(milliseconds: 120)),
      );
      expect(didStartScale, isFalse);
      expect(updatedFocalPoint, const Offset(20.0, 30.0));
      updatedFocalPoint = null;
      expect(updatedScale, 1.0);
      updatedScale = null;
      expect(updatedDelta!.dx, closeTo(-13.3, 0.1));
      expect(updatedDelta!.dy, closeTo(-13.3, 0.1));
      updatedDelta = null;
      expect(didEndScale, isFalse);
      expect(didTap, isFalse);
      expect(updatedSourceTimestamp, const Duration(milliseconds: 120));
      updatedSourceTimestamp = null;
      expect(initialSourceTimestamp, isNull);
      tester.route(pointer1.up());
      expect(didStartScale, isFalse);
      expect(updatedFocalPoint, isNull);
      expect(updatedScale, isNull);
      expect(updatedDelta, isNull);
      expect(didEndScale, isTrue);
      expect(updatedSourceTimestamp, isNull);
      expect(initialSourceTimestamp, isNull);
      didEndScale = false;
      expect(didTap, isFalse);

      // Continue scaling with two fingers.
      tester.route(
        pointer3.move(const Offset(10.0, 20.0), timeStamp: const Duration(milliseconds: 130)),
      );
      expect(didStartScale, isTrue);
      didStartScale = false;
      expect(updatedFocalPoint, const Offset(15.0, 25.0));
      updatedFocalPoint = null;
      expect(updatedScale, 2.0);
      updatedScale = null;
      expect(updatedDelta, const Offset(-2.5, -2.5));
      updatedDelta = null;
      expect(updatedSourceTimestamp, const Duration(milliseconds: 130));
      updatedSourceTimestamp = null;
      expect(initialSourceTimestamp, const Duration(milliseconds: 130));
      initialSourceTimestamp = null;

      // Continue rotating with two fingers.
      tester.route(
        pointer3.move(const Offset(30.0, 40.0), timeStamp: const Duration(milliseconds: 140)),
      );
      expect(updatedFocalPoint, const Offset(25.0, 35.0));
      updatedFocalPoint = null;
      expect(updatedScale, 2.0);
      updatedScale = null;
      expect(updatedDelta, const Offset(10.0, 10.0));
      updatedDelta = null;
      expect(updatedSourceTimestamp, const Duration(milliseconds: 140));
      updatedSourceTimestamp = null;
      expect(initialSourceTimestamp, isNull);
      tester.route(
        pointer3.move(const Offset(10.0, 20.0), timeStamp: const Duration(milliseconds: 140)),
      );
      expect(updatedFocalPoint, const Offset(15.0, 25.0));
      updatedFocalPoint = null;
      expect(updatedScale, 2.0);
      updatedScale = null;
      expect(updatedDelta, const Offset(-10.0, -10.0));
      updatedDelta = null;
      expect(updatedSourceTimestamp, const Duration(milliseconds: 140));
      updatedSourceTimestamp = null;
      expect(initialSourceTimestamp, isNull);

      tester.route(pointer2.up());
      expect(didStartScale, isFalse);
      expect(updatedFocalPoint, isNull);
      expect(updatedScale, isNull);
      expect(updatedDelta, isNull);
      expect(updatedSourceTimestamp, isNull);
      expect(initialSourceTimestamp, isNull);
      expect(didEndScale, isTrue);
      didEndScale = false;
      expect(didTap, isFalse);

      // Continue panning with one finger.
      tester.route(pointer3.move(Offset.zero, timeStamp: const Duration(milliseconds: 150)));
      expect(didStartScale, isTrue);
      didStartScale = false;
      expect(updatedFocalPoint, Offset.zero);
      updatedFocalPoint = null;
      expect(updatedScale, 1.0);
      updatedScale = null;
      expect(updatedDelta, const Offset(-10.0, -20.0));
      updatedDelta = null;
      expect(updatedSourceTimestamp, const Duration(milliseconds: 150));
      updatedSourceTimestamp = null;
      expect(initialSourceTimestamp, const Duration(milliseconds: 150));
      initialSourceTimestamp = null;

      // We are done.
      tester.route(pointer3.up());
      expect(didStartScale, isFalse);
      expect(updatedFocalPoint, isNull);
      expect(updatedScale, isNull);
      expect(updatedDelta, isNull);
      expect(didEndScale, isTrue);
      expect(updatedSourceTimestamp, isNull);
      expect(initialSourceTimestamp, isNull);
      didEndScale = false;
      expect(didTap, isFalse);

      scale.dispose();
      tap.dispose();
    },
  );
}
