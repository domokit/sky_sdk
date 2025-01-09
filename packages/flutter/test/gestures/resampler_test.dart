// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  PointerEvent createSimulatedPointerAddedEvent(int timeStampUs, double x, double y) {
    return PointerAddedEvent(
      timeStamp: Duration(microseconds: timeStampUs),
      position: Offset(x, y),
    );
  }

  PointerEvent createSimulatedPointerRemovedEvent(int timeStampUs, double x, double y) {
    return PointerRemovedEvent(
      timeStamp: Duration(microseconds: timeStampUs),
      position: Offset(x, y),
    );
  }

  PointerEvent createSimulatedPointerDownEvent(int timeStampUs, double x, double y) {
    return PointerDownEvent(timeStamp: Duration(microseconds: timeStampUs), position: Offset(x, y));
  }

  PointerEvent createSimulatedPointerMoveEvent(
    int timeStampUs,
    double x,
    double y,
    double deltaX,
    double deltaY,
  ) {
    return PointerMoveEvent(
      timeStamp: Duration(microseconds: timeStampUs),
      position: Offset(x, y),
      delta: Offset(deltaX, deltaY),
    );
  }

  PointerEvent createSimulatedPointerHoverEvent(
    int timeStampUs,
    double x,
    double y,
    double deltaX,
    double deltaY,
  ) {
    return PointerHoverEvent(
      timeStamp: Duration(microseconds: timeStampUs),
      position: Offset(x, y),
      delta: Offset(deltaX, deltaY),
    );
  }

  PointerEvent createSimulatedPointerUpEvent(int timeStampUs, double x, double y) {
    return PointerUpEvent(timeStamp: Duration(microseconds: timeStampUs), position: Offset(x, y));
  }

  test('basic', () {
    final PointerEventResampler resampler = PointerEventResampler();
    final PointerEvent event0 = createSimulatedPointerAddedEvent(1000, 0.0, 50.0);
    final PointerEvent event1 = createSimulatedPointerHoverEvent(2000, 10.0, 40.0, 10.0, -10.0);
    final PointerEvent event2 = createSimulatedPointerDownEvent(2000, 10.0, 40.0);
    final PointerEvent event3 = createSimulatedPointerMoveEvent(3000, 20.0, 30.0, 10.0, -10.0);
    final PointerEvent event4 = createSimulatedPointerMoveEvent(4000, 30.0, 20.0, 10.0, -10.0);
    final PointerEvent event5 = createSimulatedPointerUpEvent(4000, 30.0, 20.0);
    final PointerEvent event6 = createSimulatedPointerHoverEvent(5000, 40.0, 10.0, 10.0, -10.0);
    final PointerEvent event7 = createSimulatedPointerHoverEvent(6000, 50.0, 0.0, 10.0, -10.0);
    final PointerEvent event8 = createSimulatedPointerRemovedEvent(6000, 50.0, 0.0);

    resampler
      ..addEvent(event0)
      ..addEvent(event1)
      ..addEvent(event2)
      ..addEvent(event3)
      ..addEvent(event4)
      ..addEvent(event5)
      ..addEvent(event6)
      ..addEvent(event7)
      ..addEvent(event8);

    final List<PointerEvent> result = <PointerEvent>[];

    resampler.sample(const Duration(microseconds: 500), Duration.zero, result.add);

    // No pointer event should have been returned yet.
    expect(result.isEmpty, true);

    resampler.sample(const Duration(microseconds: 1500), Duration.zero, result.add);

    // Add pointer event should have been returned.
    expect(result.length, 1);
    expect(result[0].timeStamp, const Duration(microseconds: 1500));
    expect(result[0] is PointerAddedEvent, true);
    expect(result[0].position.dx, 5.0);
    expect(result[0].position.dy, 45.0);

    resampler.sample(const Duration(microseconds: 2500), Duration.zero, result.add);

    // Hover and down pointer events should have been returned.
    expect(result.length, 3);
    expect(result[1].timeStamp, const Duration(microseconds: 2500));
    expect(result[1] is PointerHoverEvent, true);
    expect(result[1].position.dx, 15.0);
    expect(result[1].position.dy, 35.0);
    expect(result[1].delta.dx, 10.0);
    expect(result[1].delta.dy, -10.0);
    expect(result[2].timeStamp, const Duration(microseconds: 2500));
    expect(result[2] is PointerDownEvent, true);
    expect(result[2].position.dx, 15.0);
    expect(result[2].position.dy, 35.0);

    resampler.sample(const Duration(microseconds: 3500), Duration.zero, result.add);

    // Move pointer event should have been returned.
    expect(result.length, 4);
    expect(result[3].timeStamp, const Duration(microseconds: 3500));
    expect(result[3] is PointerMoveEvent, true);
    expect(result[3].position.dx, 25.0);
    expect(result[3].position.dy, 25.0);
    expect(result[3].delta.dx, 10.0);
    expect(result[3].delta.dy, -10.0);

    resampler.sample(const Duration(microseconds: 4500), Duration.zero, result.add);

    // Move and up pointer events should have been returned.
    expect(result.length, 6);
    expect(result[4].timeStamp, const Duration(microseconds: 4500));
    expect(result[4] is PointerMoveEvent, true);
    expect(result[4].position.dx, 35.0);
    expect(result[4].position.dy, 15.0);
    expect(result[4].delta.dx, 10.0);
    expect(result[4].delta.dy, -10.0);
    // buttons field needs to be a valid value
    expect(result[4].buttons, kPrimaryButton);
    expect(result[5].timeStamp, const Duration(microseconds: 4500));
    expect(result[5] is PointerUpEvent, true);
    expect(result[5].position.dx, 35.0);
    expect(result[5].position.dy, 15.0);

    resampler.sample(const Duration(microseconds: 5500), Duration.zero, result.add);

    // Hover pointer event should have been returned.
    expect(result.length, 7);
    expect(result[6].timeStamp, const Duration(microseconds: 5500));
    expect(result[6] is PointerHoverEvent, true);
    expect(result[6].position.dx, 45.0);
    expect(result[6].position.dy, 5.0);
    expect(result[6].delta.dx, 10.0);
    expect(result[6].delta.dy, -10.0);

    resampler.sample(const Duration(microseconds: 6500), Duration.zero, result.add);

    // Hover and removed pointer events should have been returned.
    expect(result.length, 9);
    expect(result[7].timeStamp, const Duration(microseconds: 6500));
    expect(result[7] is PointerHoverEvent, true);
    expect(result[7].position.dx, 50.0);
    expect(result[7].position.dy, 0.0);
    expect(result[7].delta.dx, 5.0);
    expect(result[7].delta.dy, -5.0);
    expect(result[8].timeStamp, const Duration(microseconds: 6500));
    expect(result[8] is PointerRemovedEvent, true);
    expect(result[8].position.dx, 50.0);
    expect(result[8].position.dy, 0.0);

    resampler.sample(const Duration(microseconds: 7500), Duration.zero, result.add);

    // No pointer event should have been returned.
    expect(result.length, 9);
  });

  test('stream', () {
    final PointerEventResampler resampler = PointerEventResampler();
    final PointerEvent event0 = createSimulatedPointerAddedEvent(1000, 0.0, 50.0);
    final PointerEvent event1 = createSimulatedPointerHoverEvent(2000, 10.0, 40.0, 10.0, -10.0);
    final PointerEvent event2 = createSimulatedPointerDownEvent(2000, 10.0, 40.0);
    final PointerEvent event3 = createSimulatedPointerMoveEvent(3000, 20.0, 30.0, 10.0, -10.0);
    final PointerEvent event4 = createSimulatedPointerMoveEvent(4000, 30.0, 20.0, 10.0, -10.0);
    final PointerEvent event5 = createSimulatedPointerUpEvent(4000, 30.0, 20.0);
    final PointerEvent event6 = createSimulatedPointerHoverEvent(5000, 40.0, 10.0, 10.0, -10.0);
    final PointerEvent event7 = createSimulatedPointerHoverEvent(6000, 50.0, 0.0, 10.0, -10.0);
    final PointerEvent event8 = createSimulatedPointerRemovedEvent(6000, 50.0, 0.0);

    resampler.addEvent(event0);

    //
    // Initial sample time a 0.5 ms.
    //

    final List<PointerEvent> result = <PointerEvent>[];

    resampler.sample(const Duration(microseconds: 500), Duration.zero, result.add);

    // No pointer event should have been returned yet.
    expect(result.isEmpty, true);

    resampler
      ..addEvent(event1)
      ..addEvent(event2);

    resampler.sample(const Duration(microseconds: 500), Duration.zero, result.add);

    // No pointer event should have been returned yet.
    expect(result.isEmpty, true);

    //
    // Advance sample time to 1.5 ms.
    //

    resampler.sample(const Duration(microseconds: 1500), Duration.zero, result.add);

    // Added pointer event should have been returned.
    expect(result.length, 1);
    expect(result[0].timeStamp, const Duration(microseconds: 1500));
    expect(result[0] is PointerAddedEvent, true);
    expect(result[0].position.dx, 5.0);
    expect(result[0].position.dy, 45.0);

    resampler.addEvent(event3);

    resampler.sample(const Duration(microseconds: 1500), Duration.zero, result.add);

    // No more pointer events should have been returned.
    expect(result.length, 1);

    //
    // Advance sample time to 2.5 ms.
    //

    resampler.sample(const Duration(microseconds: 2500), Duration.zero, result.add);

    // Hover and down pointer events should have been returned.
    expect(result.length, 3);
    expect(result[1].timeStamp, const Duration(microseconds: 2500));
    expect(result[1] is PointerHoverEvent, true);
    expect(result[1].position.dx, 15.0);
    expect(result[1].position.dy, 35.0);
    expect(result[1].delta.dx, 10.0);
    expect(result[1].delta.dy, -10.0);
    expect(result[2].timeStamp, const Duration(microseconds: 2500));
    expect(result[2] is PointerDownEvent, true);
    expect(result[2].position.dx, 15.0);
    expect(result[2].position.dy, 35.0);

    resampler
      ..addEvent(event4)
      ..addEvent(event5);

    resampler.sample(const Duration(microseconds: 2500), Duration.zero, result.add);

    // No more pointer events should have been returned.
    expect(result.length, 3);

    //
    // Advance sample time to 3.5 ms.
    //

    resampler.sample(const Duration(microseconds: 3500), Duration.zero, result.add);

    // Move pointer event should have been returned.
    expect(result.length, 4);
    expect(result[3].timeStamp, const Duration(microseconds: 3500));
    expect(result[3] is PointerMoveEvent, true);
    expect(result[3].position.dx, 25.0);
    expect(result[3].position.dy, 25.0);
    expect(result[3].delta.dx, 10.0);
    expect(result[3].delta.dy, -10.0);

    resampler.addEvent(event6);

    resampler.sample(const Duration(microseconds: 3500), Duration.zero, result.add);

    // No more pointer events should have been returned.
    expect(result.length, 4);

    //
    // Advance sample time to 4.5 ms.
    //

    resampler.sample(const Duration(microseconds: 4500), Duration.zero, result.add);

    // Move and up pointer events should have been returned.
    expect(result.length, 6);
    expect(result[4].timeStamp, const Duration(microseconds: 4500));
    expect(result[4] is PointerMoveEvent, true);
    expect(result[4].position.dx, 35.0);
    expect(result[4].position.dy, 15.0);
    expect(result[4].delta.dx, 10.0);
    expect(result[4].delta.dy, -10.0);
    expect(result[5].timeStamp, const Duration(microseconds: 4500));
    expect(result[5] is PointerUpEvent, true);
    expect(result[5].position.dx, 35.0);
    expect(result[5].position.dy, 15.0);

    resampler
      ..addEvent(event7)
      ..addEvent(event8);

    resampler.sample(const Duration(microseconds: 4500), Duration.zero, result.add);

    // No more pointer events should have been returned.
    expect(result.length, 6);

    //
    // Advance sample time to 5.5 ms.
    //

    resampler.sample(const Duration(microseconds: 5500), Duration.zero, result.add);

    // Hover pointer event should have been returned.
    expect(result.length, 7);
    expect(result[6].timeStamp, const Duration(microseconds: 5500));
    expect(result[6] is PointerHoverEvent, true);
    expect(result[6].position.dx, 45.0);
    expect(result[6].position.dy, 5.0);
    expect(result[6].delta.dx, 10.0);
    expect(result[6].delta.dy, -10.0);

    //
    // Advance sample time to 6.5 ms.
    //

    resampler.sample(const Duration(microseconds: 6500), Duration.zero, result.add);

    // Hover and removed pointer event should have been returned.
    expect(result.length, 9);
    expect(result[7].timeStamp, const Duration(microseconds: 6500));
    expect(result[7] is PointerHoverEvent, true);
    expect(result[7].position.dx, 50.0);
    expect(result[7].position.dy, 0.0);
    expect(result[7].delta.dx, 5.0);
    expect(result[7].delta.dy, -5.0);
    expect(result[8].timeStamp, const Duration(microseconds: 6500));
    expect(result[8] is PointerRemovedEvent, true);
    expect(result[8].position.dx, 50.0);
    expect(result[8].position.dy, 0.0);

    //
    // Advance sample time to 7.5 ms.
    //

    resampler.sample(const Duration(microseconds: 7500), Duration.zero, result.add);

    // No pointer events should have been returned.
    expect(result.length, 9);
  });

  test('quick tap', () {
    final PointerEventResampler resampler = PointerEventResampler();
    final PointerEvent event0 = createSimulatedPointerAddedEvent(1000, 0.0, 0.0);
    final PointerEvent event1 = createSimulatedPointerDownEvent(1000, 0.0, 0.0);
    final PointerEvent event2 = createSimulatedPointerUpEvent(1000, 0.0, 0.0);
    final PointerEvent event3 = createSimulatedPointerRemovedEvent(1000, 0.0, 0.0);

    resampler
      ..addEvent(event0)
      ..addEvent(event1)
      ..addEvent(event2)
      ..addEvent(event3);

    final List<PointerEvent> result = <PointerEvent>[];

    resampler.sample(const Duration(microseconds: 1500), Duration.zero, result.add);

    // All pointer events should have been returned.
    expect(result.length, 4);
    expect(result[0].timeStamp, const Duration(microseconds: 1500));
    expect(result[0] is PointerAddedEvent, true);
    expect(result[0].position.dx, 0.0);
    expect(result[0].position.dy, 0.0);
    expect(result[1].timeStamp, const Duration(microseconds: 1500));
    expect(result[1] is PointerDownEvent, true);
    expect(result[1].position.dx, 0.0);
    expect(result[1].position.dy, 0.0);
    expect(result[2].timeStamp, const Duration(microseconds: 1500));
    expect(result[2] is PointerUpEvent, true);
    expect(result[2].position.dx, 0.0);
    expect(result[2].position.dy, 0.0);
    expect(result[3].timeStamp, const Duration(microseconds: 1500));
    expect(result[3] is PointerRemovedEvent, true);
    expect(result[3].position.dx, 0.0);
    expect(result[3].position.dy, 0.0);
  });

  test('advance slowly', () {
    final PointerEventResampler resampler = PointerEventResampler();
    final PointerEvent event0 = createSimulatedPointerAddedEvent(1000, 0.0, 0.0);
    final PointerEvent event1 = createSimulatedPointerDownEvent(1000, 0.0, 0.0);
    final PointerEvent event2 = createSimulatedPointerMoveEvent(2000, 10.0, 0.0, 10.0, 0.0);
    final PointerEvent event3 = createSimulatedPointerMoveEvent(3000, 20.0, 0.0, 10.0, 0.0);
    final PointerEvent event4 = createSimulatedPointerUpEvent(3000, 20.0, 0.0);
    final PointerEvent event5 = createSimulatedPointerRemovedEvent(3000, 20.0, 0.0);

    resampler
      ..addEvent(event0)
      ..addEvent(event1)
      ..addEvent(event2)
      ..addEvent(event3)
      ..addEvent(event4)
      ..addEvent(event5);

    final List<PointerEvent> result = <PointerEvent>[];

    resampler.sample(const Duration(microseconds: 1500), Duration.zero, result.add);

    // Added and down pointer events should have been returned.
    expect(result.length, 2);
    expect(result[0].timeStamp, const Duration(microseconds: 1500));
    expect(result[0] is PointerAddedEvent, true);
    expect(result[0].position.dx, 5.0);
    expect(result[0].position.dy, 0.0);
    expect(result[1].timeStamp, const Duration(microseconds: 1500));
    expect(result[1] is PointerDownEvent, true);
    expect(result[1].position.dx, 5.0);
    expect(result[1].position.dy, 0.0);

    resampler.sample(const Duration(microseconds: 1500), Duration.zero, result.add);

    // No pointer events should have been returned.
    expect(result.length, 2);

    resampler.sample(const Duration(microseconds: 1750), Duration.zero, result.add);

    // Move pointer event should have been returned.
    expect(result.length, 3);
    expect(result[2].timeStamp, const Duration(microseconds: 1750));
    expect(result[2] is PointerMoveEvent, true);
    expect(result[2].position.dx, 7.5);
    expect(result[2].position.dy, 0.0);
    expect(result[2].delta.dx, 2.5);
    expect(result[2].delta.dy, 0.0);

    resampler.sample(const Duration(microseconds: 2000), Duration.zero, result.add);

    // Another move pointer event should have been returned.
    expect(result.length, 4);
    expect(result[3].timeStamp, const Duration(microseconds: 2000));
    expect(result[3] is PointerMoveEvent, true);
    expect(result[3].position.dx, 10.0);
    expect(result[3].position.dy, 0.0);
    expect(result[3].delta.dx, 2.5);
    expect(result[3].delta.dy, 0.0);

    resampler.sample(const Duration(microseconds: 3000), Duration.zero, result.add);

    // Move, up and removed pointer events should have been returned.
    expect(result.length, 7);
    expect(result[4].timeStamp, const Duration(microseconds: 3000));
    expect(result[4] is PointerMoveEvent, true);
    expect(result[4].position.dx, 20.0);
    expect(result[4].position.dy, 0.0);
    expect(result[4].delta.dx, 10.0);
    expect(result[4].delta.dy, 0.0);
    expect(result[5].timeStamp, const Duration(microseconds: 3000));
    expect(result[5] is PointerUpEvent, true);
    expect(result[5].position.dx, 20.0);
    expect(result[5].position.dy, 0.0);
    expect(result[6].timeStamp, const Duration(microseconds: 3000));
    expect(result[6] is PointerRemovedEvent, true);
    expect(result[6].position.dx, 20.0);
    expect(result[6].position.dy, 0.0);
  });

  test('advance fast', () {
    final PointerEventResampler resampler = PointerEventResampler();
    final PointerEvent event0 = createSimulatedPointerAddedEvent(1000, 0.0, 0.0);
    final PointerEvent event1 = createSimulatedPointerDownEvent(1000, 0.0, 0.0);
    final PointerEvent event2 = createSimulatedPointerMoveEvent(2000, 5.0, 0.0, 5.0, 0.0);
    final PointerEvent event3 = createSimulatedPointerMoveEvent(3000, 20.0, 0.0, 15.0, 0.0);
    final PointerEvent event4 = createSimulatedPointerUpEvent(4000, 30.0, 0.0);
    final PointerEvent event5 = createSimulatedPointerRemovedEvent(4000, 30.0, 0.0);

    resampler
      ..addEvent(event0)
      ..addEvent(event1)
      ..addEvent(event2)
      ..addEvent(event3)
      ..addEvent(event4)
      ..addEvent(event5);

    final List<PointerEvent> result = <PointerEvent>[];

    resampler.sample(const Duration(microseconds: 2500), Duration.zero, result.add);

    // Added and down pointer events should have been returned.
    expect(result.length, 2);
    expect(result[0].timeStamp, const Duration(microseconds: 2500));
    expect(result[0] is PointerAddedEvent, true);
    expect(result[0].position.dx, 12.5);
    expect(result[0].position.dy, 0.0);
    expect(result[1].timeStamp, const Duration(microseconds: 2500));
    expect(result[1] is PointerDownEvent, true);
    expect(result[1].position.dx, 12.5);
    expect(result[1].position.dy, 0.0);

    resampler.sample(const Duration(microseconds: 5500), Duration.zero, result.add);

    // Move, up and removed pointer events should have been returned.
    expect(result.length, 5);
    expect(result[2].timeStamp, const Duration(microseconds: 5500));
    expect(result[2] is PointerMoveEvent, true);
    expect(result[2].position.dx, 30.0);
    expect(result[2].position.dy, 0.0);
    expect(result[2].delta.dx, 17.5);
    expect(result[2].delta.dy, 0.0);
    expect(result[3].timeStamp, const Duration(microseconds: 5500));
    expect(result[3] is PointerUpEvent, true);
    expect(result[3].position.dx, 30.0);
    expect(result[3].position.dy, 0.0);
    expect(result[4].timeStamp, const Duration(microseconds: 5500));
    expect(result[4] is PointerRemovedEvent, true);
    expect(result[4].position.dx, 30.0);
    expect(result[4].position.dy, 0.0);

    resampler.sample(const Duration(microseconds: 6500), Duration.zero, result.add);

    // No pointer events should have been returned.
    expect(result.length, 5);
  });

  test('skip', () {
    final PointerEventResampler resampler = PointerEventResampler();
    final PointerEvent event0 = createSimulatedPointerAddedEvent(1000, 0.0, 0.0);
    final PointerEvent event1 = createSimulatedPointerDownEvent(1000, 0.0, 0.0);
    final PointerEvent event2 = createSimulatedPointerMoveEvent(2000, 10.0, 0.0, 10.0, 0.0);
    final PointerEvent event3 = createSimulatedPointerUpEvent(3000, 10.0, 0.0);
    final PointerEvent event4 = createSimulatedPointerHoverEvent(4000, 20.0, 0.0, 10.0, 0.0);
    final PointerEvent event5 = createSimulatedPointerDownEvent(4000, 20.0, 0.0);
    final PointerEvent event6 = createSimulatedPointerMoveEvent(5000, 30.0, 0.0, 10.0, 0.0);
    final PointerEvent event7 = createSimulatedPointerUpEvent(5000, 30.0, 0.0);
    final PointerEvent event8 = createSimulatedPointerRemovedEvent(5000, 30.0, 0.0);

    resampler
      ..addEvent(event0)
      ..addEvent(event1)
      ..addEvent(event2)
      ..addEvent(event3)
      ..addEvent(event4)
      ..addEvent(event5)
      ..addEvent(event6)
      ..addEvent(event7)
      ..addEvent(event8);

    final List<PointerEvent> result = <PointerEvent>[];

    resampler.sample(const Duration(microseconds: 1500), Duration.zero, result.add);

    // Added and down pointer events should have been returned.
    expect(result.length, 2);
    expect(result[0].timeStamp, const Duration(microseconds: 1500));
    expect(result[0] is PointerAddedEvent, true);
    expect(result[0].position.dx, 5.0);
    expect(result[0].position.dy, 0.0);
    expect(result[1].timeStamp, const Duration(microseconds: 1500));
    expect(result[1] is PointerDownEvent, true);
    expect(result[1].position.dx, 5.0);
    expect(result[1].position.dy, 0.0);

    resampler.sample(const Duration(microseconds: 5500), Duration.zero, result.add);

    // All remaining pointer events should have been returned.
    expect(result.length, 7);
    expect(result[2].timeStamp, const Duration(microseconds: 5500));
    expect(result[2] is PointerMoveEvent, true);
    expect(result[2].position.dx, 30.0);
    expect(result[2].position.dy, 0.0);
    expect(result[2].delta.dx, 25.0);
    expect(result[2].delta.dy, 0.0);
    expect(result[3].timeStamp, const Duration(microseconds: 5500));
    expect(result[3] is PointerUpEvent, true);
    expect(result[3].position.dx, 30.0);
    expect(result[3].position.dy, 0.0);
    expect(result[4].timeStamp, const Duration(microseconds: 5500));
    expect(result[4] is PointerDownEvent, true);
    expect(result[4].position.dx, 30.0);
    expect(result[4].position.dy, 0.0);
    expect(result[5].timeStamp, const Duration(microseconds: 5500));
    expect(result[5] is PointerUpEvent, true);
    expect(result[5].position.dx, 30.0);
    expect(result[5].position.dy, 0.0);
    expect(result[6].timeStamp, const Duration(microseconds: 5500));
    expect(result[6] is PointerRemovedEvent, true);
    expect(result[6].position.dx, 30.0);
    expect(result[6].position.dy, 0.0);

    resampler.sample(const Duration(microseconds: 6500), Duration.zero, result.add);

    // No pointer events should have been returned.
    expect(result.length, 7);
  });

  test('skip all', () {
    final PointerEventResampler resampler = PointerEventResampler();
    final PointerEvent event0 = createSimulatedPointerAddedEvent(1000, 0.0, 0.0);
    final PointerEvent event1 = createSimulatedPointerDownEvent(1000, 0.0, 0.0);
    final PointerEvent event2 = createSimulatedPointerMoveEvent(4000, 30.0, 0.0, 30.0, 0.0);
    final PointerEvent event3 = createSimulatedPointerUpEvent(4000, 30.0, 0.0);
    final PointerEvent event4 = createSimulatedPointerRemovedEvent(4000, 30.0, 0.0);

    resampler
      ..addEvent(event0)
      ..addEvent(event1)
      ..addEvent(event2)
      ..addEvent(event3)
      ..addEvent(event4);

    final List<PointerEvent> result = <PointerEvent>[];

    resampler.sample(const Duration(microseconds: 500), Duration.zero, result.add);

    // No pointer events should have been returned.
    expect(result.isEmpty, true);

    resampler.sample(const Duration(microseconds: 5500), Duration.zero, result.add);

    // All remaining pointer events should have been returned.
    expect(result.length, 4);
    expect(result[0].timeStamp, const Duration(microseconds: 5500));
    expect(result[0] is PointerAddedEvent, true);
    expect(result[0].position.dx, 30.0);
    expect(result[0].position.dy, 0.0);
    expect(result[1].timeStamp, const Duration(microseconds: 5500));
    expect(result[1] is PointerDownEvent, true);
    expect(result[1].position.dx, 30.0);
    expect(result[1].position.dy, 0.0);
    expect(result[2].timeStamp, const Duration(microseconds: 5500));
    expect(result[2] is PointerUpEvent, true);
    expect(result[2].position.dx, 30.0);
    expect(result[2].position.dy, 0.0);
    expect(result[3].timeStamp, const Duration(microseconds: 5500));
    expect(result[3] is PointerRemovedEvent, true);
    expect(result[3].position.dx, 30.0);
    expect(result[3].position.dy, 0.0);

    resampler.sample(const Duration(microseconds: 6500), Duration.zero, result.add);

    // No pointer events should have been returned.
    expect(result.length, 4);
  });

  test('stop', () {
    final PointerEventResampler resampler = PointerEventResampler();
    final PointerEvent event0 = createSimulatedPointerAddedEvent(1000, 0.0, 0.0);
    final PointerEvent event1 = createSimulatedPointerDownEvent(2000, 0.0, 0.0);
    final PointerEvent event2 = createSimulatedPointerMoveEvent(3000, 10.0, 0.0, 10.0, 0.0);
    final PointerEvent event3 = createSimulatedPointerMoveEvent(4000, 20.0, 0.0, 10.0, 0.0);
    final PointerEvent event4 = createSimulatedPointerUpEvent(4000, 20.0, 0.0);
    final PointerEvent event5 = createSimulatedPointerRemovedEvent(5000, 20.0, 0.0);

    resampler
      ..addEvent(event0)
      ..addEvent(event1)
      ..addEvent(event2)
      ..addEvent(event3)
      ..addEvent(event4)
      ..addEvent(event5);

    final List<PointerEvent> result = <PointerEvent>[];

    resampler.sample(const Duration(microseconds: 500), Duration.zero, result.add);

    // No pointer events should have been returned.
    expect(result.isEmpty, true);

    resampler.stop(result.add);

    // All pointer events should have been returned with original
    // time stamps and positions.
    expect(result.length, 6);
    expect(result[0].timeStamp, const Duration(microseconds: 1000));
    expect(result[0] is PointerAddedEvent, true);
    expect(result[0].position.dx, 0.0);
    expect(result[0].position.dy, 0.0);
    expect(result[1].timeStamp, const Duration(microseconds: 2000));
    expect(result[1] is PointerDownEvent, true);
    expect(result[1].position.dx, 0.0);
    expect(result[1].position.dy, 0.0);
    expect(result[2].timeStamp, const Duration(microseconds: 3000));
    expect(result[2] is PointerMoveEvent, true);
    expect(result[2].position.dx, 10.0);
    expect(result[2].position.dy, 0.0);
    expect(result[2].delta.dx, 10.0);
    expect(result[2].delta.dy, 0.0);
    expect(result[3].timeStamp, const Duration(microseconds: 4000));
    expect(result[3] is PointerMoveEvent, true);
    expect(result[3].position.dx, 20.0);
    expect(result[3].position.dy, 0.0);
    expect(result[3].delta.dx, 10.0);
    expect(result[3].delta.dy, 0.0);
    expect(result[4].timeStamp, const Duration(microseconds: 4000));
    expect(result[4] is PointerUpEvent, true);
    expect(result[4].position.dx, 20.0);
    expect(result[4].position.dy, 0.0);
    expect(result[5].timeStamp, const Duration(microseconds: 5000));
    expect(result[5] is PointerRemovedEvent, true);
    expect(result[5].position.dx, 20.0);
    expect(result[5].position.dy, 0.0);

    resampler.sample(const Duration(microseconds: 10000), Duration.zero, result.add);

    // No pointer events should have been returned.
    expect(result.length, 6);
  });

  test('synthetic move', () {
    final PointerEventResampler resampler = PointerEventResampler();
    final PointerEvent event0 = createSimulatedPointerAddedEvent(1000, 0.0, 0.0);
    final PointerEvent event1 = createSimulatedPointerDownEvent(2000, 0.0, 0.0);
    final PointerEvent event2 = createSimulatedPointerMoveEvent(3000, 10.0, 0.0, 10.0, 0.0);
    final PointerEvent event3 = createSimulatedPointerUpEvent(4000, 10.0, 0.0);
    final PointerEvent event4 = createSimulatedPointerRemovedEvent(5000, 10.0, 0.0);

    resampler
      ..addEvent(event0)
      ..addEvent(event1)
      ..addEvent(event2)
      ..addEvent(event3)
      ..addEvent(event4);

    final List<PointerEvent> result = <PointerEvent>[];

    resampler.sample(const Duration(microseconds: 500), Duration.zero, result.add);

    // No pointer events should have been returned.
    expect(result.isEmpty, true);

    resampler.sample(const Duration(microseconds: 2000), Duration.zero, result.add);

    // Added and down pointer events should have been returned.
    expect(result.length, 2);
    expect(result[0].timeStamp, const Duration(microseconds: 2000));
    expect(result[0] is PointerAddedEvent, true);
    expect(result[0].position.dx, 0.0);
    expect(result[0].position.dy, 0.0);
    expect(result[1].timeStamp, const Duration(microseconds: 2000));
    expect(result[1] is PointerDownEvent, true);
    expect(result[1].position.dx, 0.0);
    expect(result[1].position.dy, 0.0);

    resampler.sample(const Duration(microseconds: 5000), Duration.zero, result.add);

    // All remaining pointer events and a synthetic move event should
    // have been returned.
    expect(result.length, 5);
    expect(result[2].timeStamp, const Duration(microseconds: 5000));
    expect(result[2] is PointerMoveEvent, true);
    expect(result[2].position.dx, 10.0);
    expect(result[2].position.dy, 0.0);
    expect(result[2].delta.dx, 10.0);
    expect(result[2].delta.dy, 0.0);
    expect(result[3].timeStamp, const Duration(microseconds: 5000));
    expect(result[3] is PointerUpEvent, true);
    expect(result[3].position.dx, 10.0);
    expect(result[3].position.dy, 0.0);
    expect(result[4].timeStamp, const Duration(microseconds: 5000));
    expect(result[4] is PointerRemovedEvent, true);
    expect(result[4].position.dx, 10.0);
    expect(result[4].position.dy, 0.0);

    resampler.sample(const Duration(microseconds: 10000), Duration.zero, result.add);

    // No pointer events should have been returned.
    expect(result.length, 5);
  });

  test('next sample time', () {
    final PointerEventResampler resampler = PointerEventResampler();
    final PointerEvent event0 = createSimulatedPointerAddedEvent(1000, 0.0, 0.0);
    final PointerEvent event1 = createSimulatedPointerDownEvent(1000, 0.0, 0.0);
    final PointerEvent event2 = createSimulatedPointerMoveEvent(2000, 10.0, 0.0, 10.0, 0.0);
    final PointerEvent event3 = createSimulatedPointerMoveEvent(3000, 20.0, 0.0, 10.0, 0.0);
    final PointerEvent event4 = createSimulatedPointerUpEvent(3000, 20.0, 0.0);
    final PointerEvent event5 = createSimulatedPointerHoverEvent(4000, 30.0, 0.0, 10.0, 0.0);
    final PointerEvent event6 = createSimulatedPointerRemovedEvent(4000, 30.0, 0.0);

    resampler
      ..addEvent(event0)
      ..addEvent(event1)
      ..addEvent(event2)
      ..addEvent(event3)
      ..addEvent(event4)
      ..addEvent(event5)
      ..addEvent(event6);

    final List<PointerEvent> result = <PointerEvent>[];

    Duration sampleTime = const Duration(microseconds: 500);
    Duration nextSampleTime = const Duration(microseconds: 1500);
    resampler.sample(sampleTime, nextSampleTime, result.add);

    // No pointer events should have been returned.
    expect(result.isEmpty, true);

    sampleTime = nextSampleTime;
    nextSampleTime = const Duration(microseconds: 2500);
    resampler.sample(sampleTime, nextSampleTime, result.add);

    // Added and down pointer events should have been returned.
    expect(result.length, 2);
    expect(result[0].timeStamp, const Duration(microseconds: 1500));
    expect(result[0] is PointerAddedEvent, true);
    expect(result[0].position.dx, 5.0);
    expect(result[0].position.dy, 0.0);
    expect(result[1].timeStamp, const Duration(microseconds: 1500));
    expect(result[1] is PointerDownEvent, true);
    expect(result[1].position.dx, 5.0);
    expect(result[1].position.dy, 0.0);

    sampleTime = nextSampleTime;
    nextSampleTime = const Duration(microseconds: 3500);
    resampler.sample(sampleTime, nextSampleTime, result.add);

    // Move and up pointer events should have been returned.
    expect(result.length, 4);
    expect(result[2].timeStamp, const Duration(microseconds: 2500));
    expect(result[2] is PointerMoveEvent, true);
    expect(result[2].position.dx, 15.0);
    expect(result[2].position.dy, 0.0);
    expect(result[2].delta.dx, 10.0);
    expect(result[2].delta.dy, 0.0);
    expect(result[3].timeStamp, const Duration(microseconds: 2500));
    expect(result[3] is PointerUpEvent, true);
    expect(result[3].position.dx, 15.0);
    expect(result[3].position.dy, 0.0);

    sampleTime = nextSampleTime;
    nextSampleTime = const Duration(microseconds: 4500);
    resampler.sample(sampleTime, nextSampleTime, result.add);

    // All remaining pointer events should have been returned.
    expect(result.length, 6);
    expect(result[4].timeStamp, const Duration(microseconds: 3500));
    expect(result[4] is PointerHoverEvent, true);
    expect(result[4].position.dx, 25.0);
    expect(result[4].position.dy, 0.0);
    expect(result[4].delta.dx, 10.0);
    expect(result[4].delta.dy, 0.0);
    expect(result[5].timeStamp, const Duration(microseconds: 3500));
    expect(result[5] is PointerRemovedEvent, true);
    expect(result[5].position.dx, 25.0);
    expect(result[5].position.dy, 0.0);

    resampler.sample(const Duration(microseconds: 10000), Duration.zero, result.add);

    // No pointer events should have been returned.
    expect(result.length, 6);
  });
}
