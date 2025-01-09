// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' show VoidCallback;

import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import 'gesture_tester.dart';

// Down/move/up pair 1: normal tap sequence
const PointerDownEvent down = PointerDownEvent(pointer: 5, position: Offset(10, 10));

const PointerMoveEvent move = PointerMoveEvent(pointer: 5, position: Offset(15, 15));

const PointerUpEvent up = PointerUpEvent(pointer: 5, position: Offset(15, 15));

// Down/move/up pair 2: tap sequence with a large move in the middle
const PointerDownEvent down2 = PointerDownEvent(pointer: 6, position: Offset(10, 10));

const PointerMoveEvent move2 = PointerMoveEvent(pointer: 6, position: Offset(100, 200));

const PointerUpEvent up2 = PointerUpEvent(pointer: 6, position: Offset(100, 200));

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  test('GestureRecognizer smoketest', () {
    final TestGestureRecognizer recognizer = TestGestureRecognizer(debugOwner: 0);
    expect(recognizer, hasAGoodToStringDeep);
  });

  test('GestureRecognizer dispatches memory events', () async {
    await expectLater(
      await memoryEvents(() => TestGestureRecognizer().dispose(), TestGestureRecognizer),
      areCreateAndDispose,
    );
  });

  test('OffsetPair', () {
    const OffsetPair offset1 = OffsetPair(local: Offset(10, 20), global: Offset(30, 40));

    expect(offset1.local, const Offset(10, 20));
    expect(offset1.global, const Offset(30, 40));

    const OffsetPair offset2 = OffsetPair(local: Offset(50, 60), global: Offset(70, 80));

    final OffsetPair sum = offset2 + offset1;
    expect(sum.local, const Offset(60, 80));
    expect(sum.global, const Offset(100, 120));

    final OffsetPair difference = offset2 - offset1;
    expect(difference.local, const Offset(40, 40));
    expect(difference.global, const Offset(40, 40));
  });

  group('PrimaryPointerGestureRecognizer', () {
    testGesture('cleans up state after winning arena', (GestureTester tester) {
      final List<String> resolutions = <String>[];
      final IndefiniteGestureRecognizer indefinite = IndefiniteGestureRecognizer();
      addTearDown(indefinite.dispose);
      final TestPrimaryPointerGestureRecognizer<PointerUpEvent> accepting =
          TestPrimaryPointerGestureRecognizer<PointerUpEvent>(
            GestureDisposition.accepted,
            onAcceptGesture: () => resolutions.add('accepted'),
            onRejectGesture: () => resolutions.add('rejected'),
          );
      addTearDown(accepting.dispose);
      expect(accepting.state, GestureRecognizerState.ready);
      expect(accepting.primaryPointer, isNull);
      expect(accepting.initialPosition, isNull);
      expect(resolutions, <String>[]);

      indefinite.addPointer(down);
      accepting.addPointer(down);
      expect(accepting.state, GestureRecognizerState.possible);
      expect(accepting.primaryPointer, 5);
      expect(accepting.initialPosition!.global, down.position);
      expect(accepting.initialPosition!.local, down.localPosition);
      expect(resolutions, <String>[]);

      tester.closeArena(5);
      tester.async.flushMicrotasks();
      tester.route(down);
      tester.route(up);
      expect(accepting.state, GestureRecognizerState.ready);
      expect(accepting.primaryPointer, 5);
      expect(accepting.initialPosition, isNull);
      expect(resolutions, <String>['accepted']);
    });

    testGesture('cleans up state after losing arena', (GestureTester tester) {
      final List<String> resolutions = <String>[];
      final IndefiniteGestureRecognizer indefinite = IndefiniteGestureRecognizer();
      addTearDown(indefinite.dispose);
      final TestPrimaryPointerGestureRecognizer<PointerMoveEvent> rejecting =
          TestPrimaryPointerGestureRecognizer<PointerMoveEvent>(
            GestureDisposition.rejected,
            onAcceptGesture: () => resolutions.add('accepted'),
            onRejectGesture: () => resolutions.add('rejected'),
          );
      addTearDown(rejecting.dispose);
      expect(rejecting.state, GestureRecognizerState.ready);
      expect(rejecting.primaryPointer, isNull);
      expect(rejecting.initialPosition, isNull);
      expect(resolutions, <String>[]);

      indefinite.addPointer(down);
      rejecting.addPointer(down);
      expect(rejecting.state, GestureRecognizerState.possible);
      expect(rejecting.primaryPointer, 5);
      expect(rejecting.initialPosition!.global, down.position);
      expect(rejecting.initialPosition!.local, down.localPosition);
      expect(resolutions, <String>[]);

      tester.closeArena(5);
      tester.async.flushMicrotasks();
      tester.route(down);
      tester.route(move);
      expect(rejecting.state, GestureRecognizerState.defunct);
      expect(rejecting.primaryPointer, 5);
      expect(rejecting.initialPosition!.global, down.position);
      expect(rejecting.initialPosition!.local, down.localPosition);
      expect(resolutions, <String>['rejected']);

      tester.route(up);
      expect(rejecting.state, GestureRecognizerState.ready);
      expect(rejecting.primaryPointer, 5);
      expect(rejecting.initialPosition, isNull);
      expect(resolutions, <String>['rejected']);
    });

    testGesture('works properly when recycled', (GestureTester tester) {
      final List<String> resolutions = <String>[];
      final IndefiniteGestureRecognizer indefinite = IndefiniteGestureRecognizer();
      addTearDown(indefinite.dispose);
      final TestPrimaryPointerGestureRecognizer<PointerUpEvent> accepting =
          TestPrimaryPointerGestureRecognizer<PointerUpEvent>(
            GestureDisposition.accepted,
            preAcceptSlopTolerance: 15,
            postAcceptSlopTolerance: 1000,
            onAcceptGesture: () => resolutions.add('accepted'),
            onRejectGesture: () => resolutions.add('rejected'),
          );
      addTearDown(accepting.dispose);

      // Send one complete pointer sequence
      indefinite.addPointer(down);
      accepting.addPointer(down);
      tester.closeArena(5);
      tester.async.flushMicrotasks();
      tester.route(down);
      tester.route(up);
      expect(resolutions, <String>['accepted']);
      resolutions.clear();

      // Send a follow-on sequence that breaks preAcceptSlopTolerance
      indefinite.addPointer(down2);
      accepting.addPointer(down2);
      tester.closeArena(6);
      tester.async.flushMicrotasks();
      tester.route(down2);
      tester.route(move2);
      expect(resolutions, <String>['rejected']);
      tester.route(up2);
      expect(resolutions, <String>['rejected']);
    });
  });
}

class TestGestureRecognizer extends GestureRecognizer {
  TestGestureRecognizer({super.debugOwner});

  @override
  String get debugDescription => 'debugDescription content';

  @override
  void addPointer(PointerDownEvent event) {}

  @override
  void acceptGesture(int pointer) {}

  @override
  void rejectGesture(int pointer) {}
}

/// Gesture recognizer that adds itself to the gesture arena but never
/// resolves itself.
class IndefiniteGestureRecognizer extends GestureRecognizer {
  @override
  void addAllowedPointer(PointerDownEvent event) {
    GestureBinding.instance.gestureArena.add(event.pointer, this);
  }

  @override
  void acceptGesture(int pointer) {}

  @override
  void rejectGesture(int pointer) {}

  @override
  String get debugDescription => 'Unresolving';
}

/// Gesture recognizer that resolves with [resolution] when it handles an event
/// on the primary pointer of type [T]
class TestPrimaryPointerGestureRecognizer<T extends PointerEvent>
    extends PrimaryPointerGestureRecognizer {
  TestPrimaryPointerGestureRecognizer(
    this.resolution, {
    this.onAcceptGesture,
    this.onRejectGesture,
    super.preAcceptSlopTolerance,
    super.postAcceptSlopTolerance,
  });

  final GestureDisposition resolution;
  final VoidCallback? onAcceptGesture;
  final VoidCallback? onRejectGesture;

  @override
  void acceptGesture(int pointer) {
    super.acceptGesture(pointer);
    onAcceptGesture?.call();
  }

  @override
  void rejectGesture(int pointer) {
    super.rejectGesture(pointer);
    onRejectGesture?.call();
  }

  @override
  void handlePrimaryPointer(PointerEvent event) {
    if (event is T) {
      resolve(resolution);
    }
  }

  @override
  String get debugDescription => 'TestPrimaryPointer';
}
