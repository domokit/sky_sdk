// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart';

import 'gesture_tester.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  testGesture('toString control tests', (GestureTester tester) {
    expect(const PointerDownEvent(), hasOneLineDescription);
    expect(const PointerDownEvent().toStringFull(), hasOneLineDescription);
  });

  testGesture('nthMouseButton control tests', (GestureTester tester) {
    expect(nthMouseButton(2), kSecondaryMouseButton);
    expect(nthStylusButton(2), kSecondaryStylusButton);
  });

  testGesture('smallestButton tests', (GestureTester tester) {
    expect(smallestButton(0x0), equals(0x0));
    expect(smallestButton(0x1), equals(0x1));
    expect(smallestButton(0x200), equals(0x200));
    expect(smallestButton(0x220), equals(0x20));
  });

  testGesture('isSingleButton tests', (GestureTester tester) {
    expect(isSingleButton(0x0), isFalse);
    expect(isSingleButton(0x1), isTrue);
    expect(isSingleButton(0x200), isTrue);
    expect(isSingleButton(0x220), isFalse);
  });

  test('computed hit slop values are based on pointer device kind', () {
    expect(computeHitSlop(PointerDeviceKind.mouse, null), kPrecisePointerHitSlop);
    expect(computeHitSlop(PointerDeviceKind.stylus, null), kTouchSlop);
    expect(computeHitSlop(PointerDeviceKind.invertedStylus, null), kTouchSlop);
    expect(computeHitSlop(PointerDeviceKind.touch, null), kTouchSlop);
    expect(computeHitSlop(PointerDeviceKind.unknown, null), kTouchSlop);

    expect(computePanSlop(PointerDeviceKind.mouse, null), kPrecisePointerPanSlop);
    expect(computePanSlop(PointerDeviceKind.stylus, null), kPanSlop);
    expect(computePanSlop(PointerDeviceKind.invertedStylus, null), kPanSlop);
    expect(computePanSlop(PointerDeviceKind.touch, null), kPanSlop);
    expect(computePanSlop(PointerDeviceKind.unknown, null), kPanSlop);

    expect(computeScaleSlop(PointerDeviceKind.mouse), kPrecisePointerScaleSlop);
    expect(computeScaleSlop(PointerDeviceKind.stylus), kScaleSlop);
    expect(computeScaleSlop(PointerDeviceKind.invertedStylus), kScaleSlop);
    expect(computeScaleSlop(PointerDeviceKind.touch), kScaleSlop);
    expect(computeScaleSlop(PointerDeviceKind.unknown), kScaleSlop);
  });

  test('computed hit slop values defer to device value when pointer kind is touch', () {
    const DeviceGestureSettings settings = DeviceGestureSettings(touchSlop: 1);

    expect(computeHitSlop(PointerDeviceKind.mouse, settings), kPrecisePointerHitSlop);
    expect(computeHitSlop(PointerDeviceKind.stylus, settings), 1);
    expect(computeHitSlop(PointerDeviceKind.invertedStylus, settings), 1);
    expect(computeHitSlop(PointerDeviceKind.touch, settings), 1);
    expect(computeHitSlop(PointerDeviceKind.unknown, settings), 1);

    expect(computePanSlop(PointerDeviceKind.mouse, settings), kPrecisePointerPanSlop);
    // Pan slop is 2x touch slop
    expect(computePanSlop(PointerDeviceKind.stylus, settings), 2);
    expect(computePanSlop(PointerDeviceKind.invertedStylus, settings), 2);
    expect(computePanSlop(PointerDeviceKind.touch, settings), 2);
    expect(computePanSlop(PointerDeviceKind.unknown, settings), 2);
  });

  group('fromMouseEvent', () {
    const PointerEvent hover = PointerHoverEvent(
      timeStamp: Duration(days: 1),
      kind: PointerDeviceKind.unknown,
      device: 10,
      position: Offset(101.0, 202.0),
      buttons: 7,
      obscured: true,
      pressureMax: 2.1,
      pressureMin: 1.1,
      distance: 11,
      distanceMax: 110,
      size: 11,
      radiusMajor: 11,
      radiusMinor: 9,
      radiusMin: 1.1,
      radiusMax: 22,
      orientation: 1.1,
      tilt: 1.1,
      synthesized: true,
    );
    const PointerEvent move = PointerMoveEvent(
      timeStamp: Duration(days: 1),
      kind: PointerDeviceKind.unknown,
      device: 10,
      position: Offset(101.0, 202.0),
      buttons: 7,
      obscured: true,
      pressureMax: 2.1,
      pressureMin: 1.1,
      distanceMax: 110,
      size: 11,
      radiusMajor: 11,
      radiusMinor: 9,
      radiusMin: 1.1,
      radiusMax: 22,
      orientation: 1.1,
      tilt: 1.1,
      synthesized: true,
    );

    test('PointerEnterEvent.fromMouseEvent(hover)', () {
      final PointerEnterEvent event = PointerEnterEvent.fromMouseEvent(hover);
      const PointerEnterEvent empty = PointerEnterEvent();
      expect(event.timeStamp, hover.timeStamp);
      expect(event.pointer, empty.pointer);
      expect(event.kind, hover.kind);
      expect(event.device, hover.device);
      expect(event.position, hover.position);
      expect(event.buttons, hover.buttons);
      expect(event.down, empty.down);
      expect(event.obscured, hover.obscured);
      expect(event.pressure, empty.pressure);
      expect(event.pressureMin, hover.pressureMin);
      expect(event.pressureMax, hover.pressureMax);
      expect(event.distance, hover.distance);
      expect(event.distanceMax, hover.distanceMax);
      expect(event.distanceMax, hover.distanceMax);
      expect(event.size, hover.size);
      expect(event.radiusMajor, hover.radiusMajor);
      expect(event.radiusMinor, hover.radiusMinor);
      expect(event.radiusMin, hover.radiusMin);
      expect(event.radiusMax, hover.radiusMax);
      expect(event.orientation, hover.orientation);
      expect(event.tilt, hover.tilt);
      expect(event.synthesized, hover.synthesized);
    });

    test('PointerExitEvent.fromMouseEvent(hover)', () {
      final PointerExitEvent event = PointerExitEvent.fromMouseEvent(hover);
      const PointerExitEvent empty = PointerExitEvent();
      expect(event.timeStamp, hover.timeStamp);
      expect(event.pointer, empty.pointer);
      expect(event.kind, hover.kind);
      expect(event.device, hover.device);
      expect(event.position, hover.position);
      expect(event.buttons, hover.buttons);
      expect(event.down, empty.down);
      expect(event.obscured, hover.obscured);
      expect(event.pressure, empty.pressure);
      expect(event.pressureMin, hover.pressureMin);
      expect(event.pressureMax, hover.pressureMax);
      expect(event.distance, hover.distance);
      expect(event.distanceMax, hover.distanceMax);
      expect(event.distanceMax, hover.distanceMax);
      expect(event.size, hover.size);
      expect(event.radiusMajor, hover.radiusMajor);
      expect(event.radiusMinor, hover.radiusMinor);
      expect(event.radiusMin, hover.radiusMin);
      expect(event.radiusMax, hover.radiusMax);
      expect(event.orientation, hover.orientation);
      expect(event.tilt, hover.tilt);
      expect(event.synthesized, hover.synthesized);
    });

    test('PointerEnterEvent.fromMouseEvent(move)', () {
      final PointerEnterEvent event = PointerEnterEvent.fromMouseEvent(move);
      const PointerEnterEvent empty = PointerEnterEvent();
      expect(event.timeStamp, move.timeStamp);
      expect(event.pointer, empty.pointer);
      expect(event.kind, move.kind);
      expect(event.device, move.device);
      expect(event.position, move.position);
      expect(event.buttons, move.buttons);
      expect(event.down, move.down);
      expect(event.obscured, move.obscured);
      expect(event.pressure, empty.pressure);
      expect(event.pressureMin, move.pressureMin);
      expect(event.pressureMax, move.pressureMax);
      expect(event.distance, move.distance);
      expect(event.distanceMax, move.distanceMax);
      expect(event.distanceMax, move.distanceMax);
      expect(event.size, move.size);
      expect(event.radiusMajor, move.radiusMajor);
      expect(event.radiusMinor, move.radiusMinor);
      expect(event.radiusMin, move.radiusMin);
      expect(event.radiusMax, move.radiusMax);
      expect(event.orientation, move.orientation);
      expect(event.tilt, move.tilt);
      expect(event.synthesized, move.synthesized);
    });

    test('PointerExitEvent.fromMouseEvent(move)', () {
      final PointerExitEvent event = PointerExitEvent.fromMouseEvent(move);
      const PointerExitEvent empty = PointerExitEvent();
      expect(event.timeStamp, move.timeStamp);
      expect(event.pointer, empty.pointer);
      expect(event.kind, move.kind);
      expect(event.device, move.device);
      expect(event.position, move.position);
      expect(event.buttons, move.buttons);
      expect(event.down, move.down);
      expect(event.obscured, move.obscured);
      expect(event.pressure, empty.pressure);
      expect(event.pressureMin, move.pressureMin);
      expect(event.pressureMax, move.pressureMax);
      expect(event.distance, move.distance);
      expect(event.distanceMax, move.distanceMax);
      expect(event.distanceMax, move.distanceMax);
      expect(event.size, move.size);
      expect(event.radiusMajor, move.radiusMajor);
      expect(event.radiusMinor, move.radiusMinor);
      expect(event.radiusMin, move.radiusMin);
      expect(event.radiusMax, move.radiusMax);
      expect(event.orientation, move.orientation);
      expect(event.tilt, move.tilt);
      expect(event.synthesized, move.synthesized);
    });
  });

  group('Default values of PointerEvents:', () {
    // Some parameters are intentionally set to a non-trivial value.

    test('PointerDownEvent', () {
      const PointerDownEvent event = PointerDownEvent();
      expect(event.buttons, kPrimaryButton);
    });

    test('PointerMoveEvent', () {
      const PointerMoveEvent event = PointerMoveEvent();
      expect(event.buttons, kPrimaryButton);
    });
  });

  test('paintTransformToPointerEventTransform', () {
    Matrix4 original = Matrix4.identity();
    Matrix4 changed = PointerEvent.removePerspectiveTransform(original);
    expect(changed, original);

    original = Matrix4.identity()..scale(3.0);
    changed = PointerEvent.removePerspectiveTransform(original);
    expect(changed, isNot(original));
    original
      ..setColumn(2, Vector4(0, 0, 1, 0))
      ..setRow(2, Vector4(0, 0, 1, 0));
    expect(changed, original);
  });

  test('transformPosition', () {
    const Offset position = Offset(20, 30);
    expect(PointerEvent.transformPosition(null, position), position);
    expect(PointerEvent.transformPosition(Matrix4.identity(), position), position);
    final Matrix4 transform = Matrix4.translationValues(10, 20, 0);
    expect(
      PointerEvent.transformPosition(transform, position),
      const Offset(20.0 + 10.0, 30.0 + 20.0),
    );
  });

  test('transformDeltaViaPositions', () {
    Offset transformedDelta = PointerEvent.transformDeltaViaPositions(
      untransformedEndPosition: const Offset(20, 30),
      untransformedDelta: const Offset(5, 5),
      transform: Matrix4.identity()..scale(2.0, 2.0, 1.0),
    );
    expect(transformedDelta, const Offset(10.0, 10.0));

    transformedDelta = PointerEvent.transformDeltaViaPositions(
      untransformedEndPosition: const Offset(20, 30),
      transformedEndPosition: const Offset(40, 60),
      untransformedDelta: const Offset(5, 5),
      transform: Matrix4.identity()..scale(2.0, 2.0, 1.0),
    );
    expect(transformedDelta, const Offset(10.0, 10.0));

    transformedDelta = PointerEvent.transformDeltaViaPositions(
      untransformedEndPosition: const Offset(20, 30),
      transformedEndPosition: const Offset(40, 60),
      untransformedDelta: const Offset(5, 5),
      transform: null,
    );
    expect(transformedDelta, const Offset(5, 5));
  });

  test('transforming events', () {
    final Matrix4 transform = (Matrix4.identity()..scale(2.0, 2.0, 1.0)).multiplied(
      Matrix4.translationValues(10.0, 20.0, 0.0),
    );
    const Offset localPosition = Offset(60, 100);
    const Offset localDelta = Offset(10, 10);

    const PointerAddedEvent added = PointerAddedEvent(
      timeStamp: Duration(seconds: 2),
      kind: PointerDeviceKind.mouse,
      device: 1,
      position: Offset(20, 30),
      obscured: true,
      pressureMin: 10,
      pressureMax: 60,
      distance: 12,
      distanceMax: 24,
      radiusMin: 10,
      radiusMax: 50,
      orientation: 2,
      tilt: 4,
    );
    _expectTransformedEvent(original: added, transform: transform, localPosition: localPosition);

    const PointerCancelEvent cancel = PointerCancelEvent(
      timeStamp: Duration(seconds: 2),
      pointer: 45,
      kind: PointerDeviceKind.mouse,
      device: 1,
      position: Offset(20, 30),
      buttons: 4,
      obscured: true,
      pressureMin: 10,
      pressureMax: 60,
      distance: 12,
      distanceMax: 24,
      size: 10,
      radiusMajor: 33,
      radiusMinor: 44,
      radiusMin: 10,
      radiusMax: 50,
      orientation: 2,
      tilt: 4,
    );
    _expectTransformedEvent(original: cancel, transform: transform, localPosition: localPosition);

    const PointerDownEvent down = PointerDownEvent(
      timeStamp: Duration(seconds: 2),
      pointer: 45,
      kind: PointerDeviceKind.mouse,
      device: 1,
      position: Offset(20, 30),
      buttons: 4,
      obscured: true,
      pressure: 34,
      pressureMin: 10,
      pressureMax: 60,
      distanceMax: 24,
      size: 10,
      radiusMajor: 33,
      radiusMinor: 44,
      radiusMin: 10,
      radiusMax: 50,
      orientation: 2,
      tilt: 4,
    );
    _expectTransformedEvent(original: down, transform: transform, localPosition: localPosition);

    const PointerEnterEvent enter = PointerEnterEvent(
      timeStamp: Duration(seconds: 2),
      kind: PointerDeviceKind.mouse,
      device: 1,
      position: Offset(20, 30),
      delta: Offset(5, 5),
      buttons: 4,
      obscured: true,
      pressureMin: 10,
      pressureMax: 60,
      distance: 12,
      distanceMax: 24,
      size: 10,
      radiusMajor: 33,
      radiusMinor: 44,
      radiusMin: 10,
      radiusMax: 50,
      orientation: 2,
      tilt: 4,
      synthesized: true,
    );
    _expectTransformedEvent(
      original: enter,
      transform: transform,
      localPosition: localPosition,
      localDelta: localDelta,
    );

    const PointerExitEvent exit = PointerExitEvent(
      timeStamp: Duration(seconds: 2),
      kind: PointerDeviceKind.mouse,
      device: 1,
      position: Offset(20, 30),
      delta: Offset(5, 5),
      buttons: 4,
      obscured: true,
      pressureMin: 10,
      pressureMax: 60,
      distance: 12,
      distanceMax: 24,
      size: 10,
      radiusMajor: 33,
      radiusMinor: 44,
      radiusMin: 10,
      radiusMax: 50,
      orientation: 2,
      tilt: 4,
      synthesized: true,
    );
    _expectTransformedEvent(
      original: exit,
      transform: transform,
      localPosition: localPosition,
      localDelta: localDelta,
    );

    const PointerHoverEvent hover = PointerHoverEvent(
      timeStamp: Duration(seconds: 2),
      kind: PointerDeviceKind.mouse,
      device: 1,
      position: Offset(20, 30),
      delta: Offset(5, 5),
      buttons: 4,
      obscured: true,
      pressureMin: 10,
      pressureMax: 60,
      distance: 12,
      distanceMax: 24,
      size: 10,
      radiusMajor: 33,
      radiusMinor: 44,
      radiusMin: 10,
      radiusMax: 50,
      orientation: 2,
      tilt: 4,
      synthesized: true,
    );
    _expectTransformedEvent(
      original: hover,
      transform: transform,
      localPosition: localPosition,
      localDelta: localDelta,
    );

    const PointerMoveEvent move = PointerMoveEvent(
      timeStamp: Duration(seconds: 2),
      pointer: 45,
      kind: PointerDeviceKind.mouse,
      device: 1,
      position: Offset(20, 30),
      delta: Offset(5, 5),
      buttons: 4,
      obscured: true,
      pressure: 34,
      pressureMin: 10,
      pressureMax: 60,
      distanceMax: 24,
      size: 10,
      radiusMajor: 33,
      radiusMinor: 44,
      radiusMin: 10,
      radiusMax: 50,
      orientation: 2,
      tilt: 4,
      platformData: 10,
      synthesized: true,
    );
    _expectTransformedEvent(
      original: move,
      transform: transform,
      localPosition: localPosition,
      localDelta: localDelta,
    );

    const PointerRemovedEvent removed = PointerRemovedEvent(
      timeStamp: Duration(seconds: 2),
      kind: PointerDeviceKind.mouse,
      device: 1,
      position: Offset(20, 30),
      obscured: true,
      pressureMin: 10,
      pressureMax: 60,
      distanceMax: 24,
      radiusMin: 10,
      radiusMax: 50,
    );
    _expectTransformedEvent(original: removed, transform: transform, localPosition: localPosition);

    const PointerScrollEvent scroll = PointerScrollEvent(
      timeStamp: Duration(seconds: 2),
      device: 1,
      position: Offset(20, 30),
    );
    _expectTransformedEvent(original: scroll, transform: transform, localPosition: localPosition);

    const PointerPanZoomStartEvent panZoomStart = PointerPanZoomStartEvent(
      timeStamp: Duration(seconds: 2),
      device: 1,
      position: Offset(20, 30),
    );
    _expectTransformedEvent(
      original: panZoomStart,
      transform: transform,
      localPosition: localPosition,
    );

    const PointerPanZoomUpdateEvent panZoomUpdate = PointerPanZoomUpdateEvent(
      timeStamp: Duration(seconds: 2),
      device: 1,
      position: Offset(20, 30),
    );
    _expectTransformedEvent(
      original: panZoomUpdate,
      transform: transform,
      localPosition: localPosition,
    );

    const PointerPanZoomEndEvent panZoomEnd = PointerPanZoomEndEvent(
      timeStamp: Duration(seconds: 2),
      device: 1,
      position: Offset(20, 30),
    );
    _expectTransformedEvent(
      original: panZoomEnd,
      transform: transform,
      localPosition: localPosition,
    );

    const PointerUpEvent up = PointerUpEvent(
      timeStamp: Duration(seconds: 2),
      pointer: 45,
      kind: PointerDeviceKind.mouse,
      device: 1,
      position: Offset(20, 30),
      buttons: 4,
      obscured: true,
      pressure: 34,
      pressureMin: 10,
      pressureMax: 60,
      distance: 12,
      distanceMax: 24,
      size: 10,
      radiusMajor: 33,
      radiusMinor: 44,
      radiusMin: 10,
      radiusMax: 50,
      orientation: 2,
      tilt: 4,
    );
    _expectTransformedEvent(original: up, transform: transform, localPosition: localPosition);
  });

  group('copyWith', () {
    const PointerEvent added = PointerAddedEvent(
      timeStamp: Duration(days: 1),
      kind: PointerDeviceKind.unknown,
      device: 10,
      position: Offset(101.0, 202.0),
      obscured: true,
      pressureMax: 2.1,
      pressureMin: 1.1,
      distance: 11,
      distanceMax: 110,
      radiusMin: 1.1,
      radiusMax: 22,
      orientation: 1.1,
      tilt: 1.1,
    );
    const PointerEvent hover = PointerHoverEvent(
      timeStamp: Duration(days: 1),
      kind: PointerDeviceKind.unknown,
      device: 10,
      position: Offset(101.0, 202.0),
      buttons: 7,
      obscured: true,
      pressureMax: 2.1,
      pressureMin: 1.1,
      distance: 11,
      distanceMax: 110,
      size: 11,
      radiusMajor: 11,
      radiusMinor: 9,
      radiusMin: 1.1,
      radiusMax: 22,
      orientation: 1.1,
      tilt: 1.1,
      synthesized: true,
    );
    const PointerEvent down = PointerDownEvent(
      timeStamp: Duration(days: 1),
      pointer: 1,
      kind: PointerDeviceKind.unknown,
      device: 10,
      position: Offset(101.0, 202.0),
      buttons: 7,
      obscured: true,
      pressureMax: 2.1,
      pressureMin: 1.1,
      distanceMax: 110,
      size: 11,
      radiusMajor: 11,
      radiusMinor: 9,
      radiusMin: 1.1,
      radiusMax: 22,
      orientation: 1.1,
      tilt: 1.1,
    );
    const PointerEvent move = PointerMoveEvent(
      timeStamp: Duration(days: 1),
      pointer: 1,
      kind: PointerDeviceKind.unknown,
      device: 10,
      position: Offset(101.0, 202.0),
      delta: Offset(1.0, 2.0),
      buttons: 7,
      obscured: true,
      pressureMax: 2.1,
      pressureMin: 1.1,
      distanceMax: 110,
      size: 11,
      radiusMajor: 11,
      radiusMinor: 9,
      radiusMin: 1.1,
      radiusMax: 22,
      orientation: 1.1,
      tilt: 1.1,
      synthesized: true,
    );
    const PointerEvent up = PointerUpEvent(
      timeStamp: Duration(days: 1),
      pointer: 1,
      kind: PointerDeviceKind.unknown,
      device: 10,
      position: Offset(101.0, 202.0),
      buttons: 7,
      obscured: true,
      pressureMax: 2.1,
      pressureMin: 1.1,
      distanceMax: 110,
      size: 11,
      radiusMajor: 11,
      radiusMinor: 9,
      radiusMin: 1.1,
      radiusMax: 22,
      orientation: 1.1,
      tilt: 1.1,
    );
    const PointerEvent removed = PointerRemovedEvent(
      timeStamp: Duration(days: 1),
      kind: PointerDeviceKind.unknown,
      device: 10,
      position: Offset(101.0, 202.0),
      obscured: true,
      pressureMax: 2.1,
      pressureMin: 1.1,
      distanceMax: 110,
      radiusMin: 1.1,
      radiusMax: 22,
    );

    const Offset position = Offset.zero;
    const Duration timeStamp = Duration(days: 2);

    test('PointerAddedEvent.copyWith()', () {
      final PointerEvent event = added.copyWith(position: position, timeStamp: timeStamp);
      const PointerEvent empty = PointerAddedEvent();
      expect(event.timeStamp, timeStamp);
      expect(event.pointer, empty.pointer);
      expect(event.kind, added.kind);
      expect(event.device, added.device);
      expect(event.position, position);
      expect(event.delta, empty.delta);
      expect(event.buttons, added.buttons);
      expect(event.down, empty.down);
      expect(event.obscured, added.obscured);
      expect(event.pressure, empty.pressure);
      expect(event.pressureMin, added.pressureMin);
      expect(event.pressureMax, added.pressureMax);
      expect(event.distance, added.distance);
      expect(event.distanceMax, added.distanceMax);
      expect(event.distanceMax, added.distanceMax);
      expect(event.size, empty.size);
      expect(event.radiusMajor, empty.radiusMajor);
      expect(event.radiusMinor, empty.radiusMinor);
      expect(event.radiusMin, added.radiusMin);
      expect(event.radiusMax, added.radiusMax);
      expect(event.orientation, added.orientation);
      expect(event.tilt, added.tilt);
      expect(event.synthesized, empty.synthesized);
    });

    test('PointerHoverEvent.copyWith()', () {
      final PointerEvent event = hover.copyWith(position: position, timeStamp: timeStamp);
      const PointerEvent empty = PointerHoverEvent();
      expect(event.timeStamp, timeStamp);
      expect(event.pointer, empty.pointer);
      expect(event.kind, hover.kind);
      expect(event.device, hover.device);
      expect(event.position, position);
      expect(event.delta, empty.delta);
      expect(event.buttons, hover.buttons);
      expect(event.down, empty.down);
      expect(event.obscured, hover.obscured);
      expect(event.pressure, empty.pressure);
      expect(event.pressureMin, hover.pressureMin);
      expect(event.pressureMax, hover.pressureMax);
      expect(event.distance, hover.distance);
      expect(event.distanceMax, hover.distanceMax);
      expect(event.distanceMax, hover.distanceMax);
      expect(event.size, hover.size);
      expect(event.radiusMajor, hover.radiusMajor);
      expect(event.radiusMinor, hover.radiusMinor);
      expect(event.radiusMin, hover.radiusMin);
      expect(event.radiusMax, hover.radiusMax);
      expect(event.orientation, hover.orientation);
      expect(event.tilt, hover.tilt);
      expect(event.synthesized, hover.synthesized);
    });

    test('PointerDownEvent.copyWith()', () {
      final PointerEvent event = down.copyWith(position: position, timeStamp: timeStamp);
      const PointerEvent empty = PointerDownEvent();
      expect(event.timeStamp, timeStamp);
      expect(event.pointer, down.pointer);
      expect(event.kind, down.kind);
      expect(event.device, down.device);
      expect(event.position, position);
      expect(event.delta, empty.delta);
      expect(event.buttons, down.buttons);
      expect(event.down, empty.down);
      expect(event.obscured, down.obscured);
      expect(event.pressure, empty.pressure);
      expect(event.pressureMin, down.pressureMin);
      expect(event.pressureMax, down.pressureMax);
      expect(event.distance, down.distance);
      expect(event.distanceMax, down.distanceMax);
      expect(event.distanceMax, down.distanceMax);
      expect(event.size, down.size);
      expect(event.radiusMajor, down.radiusMajor);
      expect(event.radiusMinor, down.radiusMinor);
      expect(event.radiusMin, down.radiusMin);
      expect(event.radiusMax, down.radiusMax);
      expect(event.orientation, down.orientation);
      expect(event.tilt, down.tilt);
      expect(event.synthesized, empty.synthesized);
    });

    test('PointerMoveEvent.copyWith()', () {
      final PointerEvent event = move.copyWith(position: position, timeStamp: timeStamp);
      const PointerEvent empty = PointerMoveEvent();
      expect(event.timeStamp, timeStamp);
      expect(event.pointer, move.pointer);
      expect(event.kind, move.kind);
      expect(event.device, move.device);
      expect(event.position, position);
      expect(event.delta, move.delta);
      expect(event.buttons, move.buttons);
      expect(event.down, move.down);
      expect(event.obscured, move.obscured);
      expect(event.pressure, empty.pressure);
      expect(event.pressureMin, move.pressureMin);
      expect(event.pressureMax, move.pressureMax);
      expect(event.distance, move.distance);
      expect(event.distanceMax, move.distanceMax);
      expect(event.distanceMax, move.distanceMax);
      expect(event.size, move.size);
      expect(event.radiusMajor, move.radiusMajor);
      expect(event.radiusMinor, move.radiusMinor);
      expect(event.radiusMin, move.radiusMin);
      expect(event.radiusMax, move.radiusMax);
      expect(event.orientation, move.orientation);
      expect(event.tilt, move.tilt);
      expect(event.synthesized, move.synthesized);
    });

    test('PointerUpEvent.copyWith()', () {
      final PointerEvent event = up.copyWith(position: position, timeStamp: timeStamp);
      const PointerEvent empty = PointerUpEvent();
      expect(event.timeStamp, timeStamp);
      expect(event.pointer, up.pointer);
      expect(event.kind, up.kind);
      expect(event.device, up.device);
      expect(event.position, position);
      expect(event.delta, up.delta);
      expect(event.buttons, up.buttons);
      expect(event.down, empty.down);
      expect(event.obscured, up.obscured);
      expect(event.pressure, empty.pressure);
      expect(event.pressureMin, up.pressureMin);
      expect(event.pressureMax, up.pressureMax);
      expect(event.distance, up.distance);
      expect(event.distanceMax, up.distanceMax);
      expect(event.distanceMax, up.distanceMax);
      expect(event.size, up.size);
      expect(event.radiusMajor, up.radiusMajor);
      expect(event.radiusMinor, up.radiusMinor);
      expect(event.radiusMin, up.radiusMin);
      expect(event.radiusMax, up.radiusMax);
      expect(event.orientation, up.orientation);
      expect(event.tilt, up.tilt);
      expect(event.synthesized, empty.synthesized);
    });

    test('PointerRemovedEvent.copyWith()', () {
      final PointerEvent event = removed.copyWith(position: position, timeStamp: timeStamp);
      const PointerEvent empty = PointerRemovedEvent();
      expect(event.timeStamp, timeStamp);
      expect(event.pointer, empty.pointer);
      expect(event.kind, removed.kind);
      expect(event.device, removed.device);
      expect(event.position, position);
      expect(event.delta, empty.delta);
      expect(event.buttons, removed.buttons);
      expect(event.down, empty.down);
      expect(event.obscured, removed.obscured);
      expect(event.pressure, empty.pressure);
      expect(event.pressureMin, removed.pressureMin);
      expect(event.pressureMax, removed.pressureMax);
      expect(event.distance, empty.distance);
      expect(event.distanceMax, removed.distanceMax);
      expect(event.distanceMax, removed.distanceMax);
      expect(event.size, empty.size);
      expect(event.radiusMajor, empty.radiusMajor);
      expect(event.radiusMinor, empty.radiusMinor);
      expect(event.radiusMin, removed.radiusMin);
      expect(event.radiusMax, removed.radiusMax);
      expect(event.orientation, empty.orientation);
      expect(event.tilt, empty.tilt);
      expect(event.synthesized, empty.synthesized);
    });
  });

  test('Ensure certain event types are allowed', () {
    // Regression test for https://github.com/flutter/flutter/issues/107962
    expect(const PointerHoverEvent(kind: PointerDeviceKind.trackpad), isNotNull);
    // Regression test for https://github.com/flutter/flutter/issues/108176
    expect(const PointerScrollInertiaCancelEvent(kind: PointerDeviceKind.trackpad), isNotNull);

    expect(const PointerScrollEvent(kind: PointerDeviceKind.trackpad), isNotNull);
    // The test passes if it compiles.
  });

  test('Ensure certain event types are not allowed', () {
    expect(() => PointerDownEvent(kind: PointerDeviceKind.trackpad), throwsAssertionError);
    expect(() => PointerMoveEvent(kind: PointerDeviceKind.trackpad), throwsAssertionError);
    expect(() => PointerUpEvent(kind: PointerDeviceKind.trackpad), throwsAssertionError);
  });
}

void _expectTransformedEvent({
  required PointerEvent original,
  required Matrix4 transform,
  Offset? localDelta,
  Offset? localPosition,
}) {
  expect(original.position, original.localPosition);
  expect(original.delta, original.localDelta);
  expect(original.original, isNull);
  expect(original.transform, isNull);

  final PointerEvent transformed = original.transformed(transform);
  expect(transformed.original, same(original));
  expect(transformed.transform, transform);
  expect(transformed.localDelta, localDelta ?? original.localDelta);
  expect(transformed.localPosition, localPosition ?? original.localPosition);

  expect(transformed.buttons, original.buttons);
  expect(transformed.delta, original.delta);
  expect(transformed.device, original.device);
  expect(transformed.distance, original.distance);
  expect(transformed.distanceMax, original.distanceMax);
  expect(transformed.distanceMin, original.distanceMin);
  expect(transformed.down, original.down);
  expect(transformed.kind, original.kind);
  expect(transformed.obscured, original.obscured);
  expect(transformed.orientation, original.orientation);
  expect(transformed.platformData, original.platformData);
  expect(transformed.pointer, original.pointer);
  expect(transformed.position, original.position);
  expect(transformed.pressure, original.pressure);
  expect(transformed.pressureMax, original.pressureMax);
  expect(transformed.pressureMin, original.pressureMin);
  expect(transformed.radiusMajor, original.radiusMajor);
  expect(transformed.radiusMax, original.radiusMax);
  expect(transformed.radiusMin, original.radiusMin);
  expect(transformed.radiusMinor, original.radiusMinor);
  expect(transformed.size, original.size);
  expect(transformed.synthesized, original.synthesized);
  expect(transformed.tilt, original.tilt);
  expect(transformed.timeStamp, original.timeStamp);
}
