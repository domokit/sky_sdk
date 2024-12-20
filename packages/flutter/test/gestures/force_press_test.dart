// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'gesture_tester.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  testGesture('A force press can be recognized', (GestureTester tester) {
    // Device specific constants that represent those from the iPhone X
    const double pressureMin = 0;
    const double pressureMax = 6.66;

    int started = 0;
    int peaked = 0;
    int updated = 0;
    int ended = 0;

    Offset? startGlobalPosition;

    void onStart(ForcePressDetails details) {
      startGlobalPosition = details.globalPosition;
      started += 1;
    }

    final ForcePressGestureRecognizer force = ForcePressGestureRecognizer();
    addTearDown(force.dispose);

    force.onStart = onStart;
    force.onPeak = (ForcePressDetails details) => peaked += 1;
    force.onUpdate = (ForcePressDetails details) => updated += 1;
    force.onEnd = (ForcePressDetails details) => ended += 1;

    const int pointerValue = 1;
    final TestPointer pointer = TestPointer();
    const PointerDownEvent down = PointerDownEvent(
      pointer: pointerValue,
      position: Offset(10.0, 10.0),
      pressure: 0,
      pressureMin: pressureMin,
      pressureMax: pressureMax,
    );
    pointer.setDownInfo(down, const Offset(10.0, 10.0));
    force.addPointer(down);
    tester.closeArena(pointerValue);

    expect(started, 0);
    expect(peaked, 0);
    expect(updated, 0);
    expect(ended, 0);

    // Pressure fed into the test environment simulates the values received directly from the device.
    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 2.5,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );

    // We have not hit the start pressure, so no events should be true.
    expect(started, 0);
    expect(peaked, 0);
    expect(updated, 0);
    expect(ended, 0);

    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 2.8,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );

    // We have just hit the start pressure so just the start event should be triggered and one update call should have occurred.
    expect(started, 1);
    expect(peaked, 0);
    expect(updated, 1);
    expect(ended, 0);

    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 3.3,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );
    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 4.0,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );
    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 5.0,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );
    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );

    // We have exceeded the start pressure so update should be greater than 0.
    expect(started, 1);
    expect(updated, 5);
    expect(peaked, 0);
    expect(ended, 0);
    expect(startGlobalPosition, const Offset(10.0, 10.0));

    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 6.0,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );

    // We have exceeded the peak pressure so peak pressure should be true.
    expect(started, 1);
    expect(updated, 6);
    expect(peaked, 1);
    expect(ended, 0);

    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 3.3,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );
    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 4.0,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );
    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 5.0,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );
    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );

    // Update is still called.
    expect(started, 1);
    expect(updated, 10);
    expect(peaked, 1);
    expect(ended, 0);

    tester.route(pointer.up());

    // We have ended the gesture so ended should be true.
    expect(started, 1);
    expect(updated, 10);
    expect(peaked, 1);
    expect(ended, 1);
  });

  testGesture('Invalid pressure ranges capabilities are not recognized', (GestureTester tester) {
    void testGestureWithMaxPressure(double pressureMax) {
      int started = 0;
      int peaked = 0;
      int updated = 0;
      int ended = 0;

      final ForcePressGestureRecognizer force = ForcePressGestureRecognizer();
      addTearDown(force.dispose);

      force.onStart = (ForcePressDetails details) => started += 1;
      force.onPeak = (ForcePressDetails details) => peaked += 1;
      force.onUpdate = (ForcePressDetails details) => updated += 1;
      force.onEnd = (ForcePressDetails details) => ended += 1;

      const int pointerValue = 1;
      final TestPointer pointer = TestPointer();
      final PointerDownEvent down = PointerDownEvent(
        pointer: pointerValue,
        position: const Offset(10.0, 10.0),
        pressure: 0,
        pressureMin: 0,
        pressureMax: pressureMax,
      );
      pointer.setDownInfo(down, const Offset(10.0, 10.0));
      force.addPointer(down);
      tester.closeArena(pointerValue);

      expect(started, 0);
      expect(peaked, 0);
      expect(updated, 0);
      expect(ended, 0);

      // Pressure fed into the test environment simulates the values received directly from the device.
      tester.route(
        PointerMoveEvent(
          pointer: pointerValue,
          position: const Offset(10.0, 10.0),
          pressure: 10,
          pressureMin: 0,
          pressureMax: pressureMax,
        ),
      );

      // Regardless of current pressure, this recognizer shouldn't participate or
      // trigger any callbacks.
      expect(started, 0);
      expect(peaked, 0);
      expect(updated, 0);
      expect(ended, 0);

      tester.route(pointer.up());

      // There should still be no callbacks.
      expect(started, 0);
      expect(updated, 0);
      expect(peaked, 0);
      expect(ended, 0);
    }

    testGestureWithMaxPressure(0);
    testGestureWithMaxPressure(1);
    testGestureWithMaxPressure(-1);
    testGestureWithMaxPressure(0.5);
  });

  testGesture('If minimum pressure is not reached, start and end callbacks are not called', (
    GestureTester tester,
  ) {
    // Device specific constants that represent those from the iPhone X
    const double pressureMin = 0;
    const double pressureMax = 6.66;

    int started = 0;
    int peaked = 0;
    int updated = 0;
    int ended = 0;

    final ForcePressGestureRecognizer force = ForcePressGestureRecognizer();
    addTearDown(force.dispose);

    force.onStart = (_) => started += 1;
    force.onPeak = (_) => peaked += 1;
    force.onUpdate = (_) => updated += 1;
    force.onEnd = (_) => ended += 1;

    const int pointerValue = 1;
    final TestPointer pointer = TestPointer();
    const PointerDownEvent down = PointerDownEvent(
      pointer: pointerValue,
      position: Offset(10.0, 10.0),
      pressure: 0,
      pressureMin: pressureMin,
      pressureMax: pressureMax,
    );
    pointer.setDownInfo(down, const Offset(10.0, 10.0));
    force.addPointer(down);
    tester.closeArena(1);

    expect(started, 0);
    expect(peaked, 0);
    expect(updated, 0);
    expect(ended, 0);

    // Pressure fed into the test environment simulates the values received directly from the device.
    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 2.5,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );

    // We have not hit the start pressure, so no events should be true.
    expect(started, 0);
    expect(peaked, 0);
    expect(updated, 0);
    expect(ended, 0);

    tester.route(pointer.up());

    expect(started, 0);
    expect(peaked, 0);
    expect(updated, 0);
    expect(ended, 0);
  });

  testGesture('Should recognize drag and not force touch if there is a drag recognizer', (
    GestureTester tester,
  ) {
    final PanGestureRecognizer drag = PanGestureRecognizer();
    addTearDown(drag.dispose);

    // Device specific constants that represent those from the iPhone X
    const double pressureMin = 0;
    const double pressureMax = 6.66;

    int started = 0;
    int peaked = 0;
    int updated = 0;
    int ended = 0;

    final ForcePressGestureRecognizer force = ForcePressGestureRecognizer();
    addTearDown(force.dispose);

    force.onStart = (_) => started += 1;
    force.onPeak = (_) => peaked += 1;
    force.onUpdate = (_) => updated += 1;
    force.onEnd = (_) => ended += 1;

    int didStartPan = 0;
    drag.onStart = (_) => didStartPan += 1;

    const int pointerValue = 1;
    final TestPointer pointer = TestPointer();
    const PointerDownEvent down = PointerDownEvent(
      pointer: pointerValue,
      position: Offset(10.0, 10.0),
      pressureMin: pressureMin,
      pressureMax: pressureMax,
    );
    pointer.setDownInfo(down, const Offset(10.0, 10.0));
    force.addPointer(down);
    drag.addPointer(down);
    tester.closeArena(1);

    expect(started, 0);
    expect(peaked, 0);
    expect(updated, 0);
    expect(ended, 0);
    expect(didStartPan, 0);

    tester.route(
      pointer.move(const Offset(30.0, 30.0)),
    ); // moved 20 horizontally and 20 vertically which is 28 total

    expect(started, 0);
    expect(peaked, 0);
    expect(updated, 0);
    expect(ended, 0);
    expect(didStartPan, 1);

    // Pressure fed into the test environment simulates the values received directly from the device.
    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 2.5,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );

    // We have not hit the start pressure, so no events should be true.
    expect(started, 0);
    expect(peaked, 0);
    expect(updated, 0);
    expect(ended, 0);
    expect(didStartPan, 1);

    // We don't expect any events from the force press recognizer.
    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 4.0,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );

    expect(started, 0);
    expect(peaked, 0);
    expect(updated, 0);
    expect(ended, 0);
    expect(didStartPan, 1);

    tester.route(pointer.up());

    expect(started, 0);
    expect(peaked, 0);
    expect(updated, 0);
    expect(ended, 0);
    expect(didStartPan, 1);
  });

  testGesture('Should not call ended on pointer up if the gesture was never accepted', (
    GestureTester tester,
  ) {
    final PanGestureRecognizer drag = PanGestureRecognizer();
    addTearDown(drag.dispose);

    // Device specific constants that represent those from the iPhone X
    const double pressureMin = 0;
    const double pressureMax = 6.66;

    int started = 0;
    int peaked = 0;
    int updated = 0;
    int ended = 0;

    final ForcePressGestureRecognizer force = ForcePressGestureRecognizer();
    addTearDown(force.dispose);

    force.onStart = (_) => started += 1;
    force.onPeak = (_) => peaked += 1;
    force.onUpdate = (_) => updated += 1;
    force.onEnd = (_) => ended += 1;

    int didStartPan = 0;
    drag.onStart = (_) => didStartPan += 1;

    const int pointerValue = 1;
    final TestPointer pointer = TestPointer();
    const PointerDownEvent down = PointerDownEvent(
      pointer: pointerValue,
      position: Offset(10.0, 10.0),
      pressureMin: pressureMin,
      pressureMax: pressureMax,
    );
    pointer.setDownInfo(down, const Offset(10.0, 10.0));
    force.addPointer(down);
    drag.addPointer(down);
    tester.closeArena(1);

    expect(started, 0);
    expect(peaked, 0);
    expect(updated, 0);
    expect(ended, 0);
    expect(didStartPan, 0);

    tester.route(pointer.up());

    expect(started, 0);
    expect(peaked, 0);
    expect(updated, 0);
    expect(ended, 0);
    expect(didStartPan, 0);
  });

  testGesture('Should call start only once if there is a competing gesture recognizer', (
    GestureTester tester,
  ) {
    final PanGestureRecognizer drag = PanGestureRecognizer();
    addTearDown(drag.dispose);

    // Device specific constants that represent those from the iPhone X
    const double pressureMin = 0;
    const double pressureMax = 6.66;

    int started = 0;
    int peaked = 0;
    int updated = 0;
    int ended = 0;

    final ForcePressGestureRecognizer force = ForcePressGestureRecognizer();
    addTearDown(force.dispose);

    force.onStart = (_) => started += 1;
    force.onPeak = (_) => peaked += 1;
    force.onUpdate = (_) => updated += 1;
    force.onEnd = (_) => ended += 1;

    int didStartPan = 0;
    drag.onStart = (_) => didStartPan += 1;

    const int pointerValue = 1;
    final TestPointer pointer = TestPointer();
    const PointerDownEvent down = PointerDownEvent(
      pointer: pointerValue,
      position: Offset(10.0, 10.0),
      pressureMin: pressureMin,
      pressureMax: pressureMax,
    );
    pointer.setDownInfo(down, const Offset(10.0, 10.0));
    force.addPointer(down);
    drag.addPointer(down);
    tester.closeArena(1);

    expect(started, 0);
    expect(peaked, 0);
    expect(updated, 0);
    expect(ended, 0);
    expect(didStartPan, 0);

    // Pressure fed into the test environment simulates the values received directly from the device.
    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 3.0,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );

    // We have not hit the start pressure, so no events should be true.
    expect(started, 1);
    expect(peaked, 0);
    expect(updated, 1);
    expect(ended, 0);
    expect(didStartPan, 0);

    tester.route(pointer.up());

    expect(started, 1);
    expect(peaked, 0);
    expect(updated, 1);
    expect(ended, 1);
    expect(didStartPan, 0);
  });

  testGesture('A force press can be recognized with a custom interpolation function', (
    GestureTester tester,
  ) {
    // Device specific constants that represent those from the iPhone X
    const double pressureMin = 0;
    const double pressureMax = 6.66;

    int started = 0;
    int peaked = 0;
    int updated = 0;
    int ended = 0;

    Offset? startGlobalPosition;

    void onStart(ForcePressDetails details) {
      startGlobalPosition = details.globalPosition;
      started += 1;
    }

    double interpolateWithEasing(double min, double max, double t) {
      final double lerp = (t - min) / (max - min);
      return Curves.easeIn.transform(lerp);
    }

    final ForcePressGestureRecognizer force = ForcePressGestureRecognizer(
      interpolation: interpolateWithEasing,
    );
    addTearDown(force.dispose);

    force.onStart = onStart;
    force.onPeak = (ForcePressDetails details) => peaked += 1;
    force.onUpdate = (ForcePressDetails details) => updated += 1;
    force.onEnd = (ForcePressDetails details) => ended += 1;

    const int pointerValue = 1;
    final TestPointer pointer = TestPointer();
    const PointerDownEvent down = PointerDownEvent(
      pointer: pointerValue,
      position: Offset(10.0, 10.0),
      pressure: 0,
      pressureMin: pressureMin,
      pressureMax: pressureMax,
    );
    pointer.setDownInfo(down, const Offset(10.0, 10.0));
    force.addPointer(down);
    tester.closeArena(pointerValue);

    expect(started, 0);
    expect(peaked, 0);
    expect(updated, 0);
    expect(ended, 0);

    // Pressure fed into the test environment simulates the values received directly from the device.
    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 2.5,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );

    // We have not hit the start pressure, so no events should be true.
    expect(started, 0);
    expect(peaked, 0);
    expect(updated, 0);
    expect(ended, 0);

    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 2.8,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );

    // We have just hit the start pressure so just the start event should be triggered and one update call should have occurred.
    expect(started, 0);
    expect(peaked, 0);
    expect(updated, 0);
    expect(ended, 0);

    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 3.3,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );
    expect(started, 0);

    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 4.0,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );
    expect(started, 1);

    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 5.0,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );
    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );

    // We have exceeded the start pressure so update should be greater than 0.
    expect(started, 1);
    expect(updated, 3);
    expect(peaked, 0);
    expect(ended, 0);
    expect(startGlobalPosition, const Offset(10.0, 10.0));

    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 6.0,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );

    // We have exceeded the peak pressure so peak pressure should be true.
    expect(started, 1);
    expect(updated, 4);
    expect(peaked, 0);
    expect(ended, 0);

    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 3.3,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );
    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 4.0,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );
    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 6.5,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );
    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );

    // Update is still called.
    expect(started, 1);
    expect(updated, 8);
    expect(peaked, 1);
    expect(ended, 0);

    tester.route(pointer.up());

    // We have ended the gesture so ended should be true.
    expect(started, 1);
    expect(updated, 8);
    expect(peaked, 1);
    expect(ended, 1);
  });

  testGesture('A pressure outside of the device reported min and max pressure will not give an error', (
    GestureTester tester,
  ) {
    // Device specific constants that represent those from the iPhone X
    const double pressureMin = 0;
    const double pressureMax = 6.66;

    int started = 0;
    int peaked = 0;
    int updated = 0;
    int ended = 0;

    final ForcePressGestureRecognizer force = ForcePressGestureRecognizer();
    addTearDown(force.dispose);

    force.onStart = (_) => started += 1;
    force.onPeak = (_) => peaked += 1;
    force.onUpdate = (_) => updated += 1;
    force.onEnd = (_) => ended += 1;

    const int pointerValue = 1;
    final TestPointer pointer = TestPointer();
    const PointerDownEvent down = PointerDownEvent(
      pointer: pointerValue,
      position: Offset(10.0, 10.0),
      pressure: 0,
      pressureMin: pressureMin,
      pressureMax: pressureMax,
    );
    pointer.setDownInfo(down, const Offset(10.0, 10.0));
    force.addPointer(down);
    tester.closeArena(1);

    expect(started, 0);
    expect(peaked, 0);
    expect(updated, 0);
    expect(ended, 0);

    // Pressure fed into the test environment simulates the values received directly from the device.
    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 2.5,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );

    // We have not hit the start pressure, so no events should be true.
    expect(started, 0);
    expect(peaked, 0);
    expect(updated, 0);
    expect(ended, 0);

    // If the case where the pressure is greater than the max pressure were not handled correctly, this move event would throw an error.
    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 8.0,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );
    tester.route(pointer.up());

    expect(started, 1);
    expect(peaked, 1);
    expect(updated, 1);
    expect(ended, 1);
  });

  testGesture('A pressure of NAN will not give an error', (GestureTester tester) {
    // Device specific constants that represent those from the iPhone X
    const double pressureMin = 0;
    const double pressureMax = 6.66;

    int started = 0;
    int peaked = 0;
    int updated = 0;
    int ended = 0;

    final ForcePressGestureRecognizer force = ForcePressGestureRecognizer();
    addTearDown(force.dispose);

    force.onStart = (_) => started += 1;
    force.onPeak = (_) => peaked += 1;
    force.onUpdate = (_) => updated += 1;
    force.onEnd = (_) => ended += 1;

    const int pointerValue = 1;
    final TestPointer pointer = TestPointer();
    const PointerDownEvent down = PointerDownEvent(
      pointer: pointerValue,
      position: Offset(10.0, 10.0),
      pressure: 0,
      pressureMin: pressureMin,
      pressureMax: pressureMax,
    );
    pointer.setDownInfo(down, const Offset(10.0, 10.0));
    force.addPointer(down);
    tester.closeArena(1);

    expect(started, 0);
    expect(peaked, 0);
    expect(updated, 0);
    expect(ended, 0);

    // Pressure fed into the test environment simulates the values received directly from the device.
    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 2.5,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );

    // We have not hit the start pressure, so no events should be true.
    expect(started, 0);
    expect(peaked, 0);
    expect(updated, 0);
    expect(ended, 0);

    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 6.0,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );
    expect(peaked, 1);

    tester.route(
      const PointerMoveEvent(
        pointer: pointerValue,
        position: Offset(10.0, 10.0),
        pressure: 0.0 / 0.0,
        pressureMin: pressureMin,
        pressureMax: pressureMax,
      ),
    );
    tester.route(pointer.up());

    expect(started, 1);
    expect(peaked, 1);
    expect(updated, 1);
    expect(ended, 1);
  });
}
