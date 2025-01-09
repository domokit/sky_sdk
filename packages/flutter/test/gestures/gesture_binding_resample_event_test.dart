// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:clock/clock.dart';
import 'package:fake_async/fake_async.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_test/flutter_test.dart';

typedef HandleEventCallback = void Function(PointerEvent event);

class TestResampleEventFlutterBinding extends BindingBase with GestureBinding, SchedulerBinding {
  HandleEventCallback? callback;
  FrameCallback? postFrameCallback;
  Duration? frameTime;

  @override
  void handleEvent(PointerEvent event, HitTestEntry entry) {
    super.handleEvent(event, entry);
    callback?.call(event);
  }

  @override
  Duration get currentSystemFrameTimeStamp {
    assert(frameTime != null);
    return frameTime!;
  }

  @override
  int addPostFrameCallback(FrameCallback callback, {String debugLabel = 'callback'}) {
    postFrameCallback = callback;
    return 0;
  }

  @override
  SamplingClock? get debugSamplingClock => TestSamplingClock();
}

class TestSamplingClock implements SamplingClock {
  @override
  DateTime now() => clock.now();

  @override
  Stopwatch stopwatch() => clock.stopwatch(); // flutter_ignore: stopwatch (see analyze.dart)
  // Ignore context: FakeAsync controls clock.stopwatch(), this is safe in tests.
}

typedef ResampleEventTest = void Function(FakeAsync async);

void testResampleEvent(String description, ResampleEventTest callback) {
  test(description, () {
    fakeAsync((FakeAsync async) {
      callback(async);
    }, initialTime: DateTime.utc(2015));
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/87067
  // Fake clock is not working with the web platform.
}

void main() {
  final TestResampleEventFlutterBinding binding = TestResampleEventFlutterBinding();
  testResampleEvent('Pointer event resampling', (FakeAsync async) {
    Duration currentTime() => Duration(milliseconds: clock.now().millisecondsSinceEpoch);
    final Duration epoch = currentTime();
    final ui.PointerDataPacket packet = ui.PointerDataPacket(
      data: <ui.PointerData>[
        ui.PointerData(change: ui.PointerChange.add, timeStamp: epoch),
        ui.PointerData(
          change: ui.PointerChange.down,
          timeStamp: epoch + const Duration(milliseconds: 10),
        ),
        ui.PointerData(
          change: ui.PointerChange.move,
          physicalX: 10.0,
          timeStamp: epoch + const Duration(milliseconds: 20),
        ),
        ui.PointerData(
          change: ui.PointerChange.move,
          physicalX: 20.0,
          timeStamp: epoch + const Duration(milliseconds: 30),
        ),
        ui.PointerData(
          change: ui.PointerChange.move,
          physicalX: 30.0,
          timeStamp: epoch + const Duration(milliseconds: 40),
        ),
        ui.PointerData(
          change: ui.PointerChange.move,
          physicalX: 40.0,
          timeStamp: epoch + const Duration(milliseconds: 50),
        ),
        ui.PointerData(
          change: ui.PointerChange.move,
          physicalX: 50.0,
          timeStamp: epoch + const Duration(milliseconds: 60),
        ),
        ui.PointerData(
          change: ui.PointerChange.up,
          physicalX: 50.0,
          timeStamp: epoch + const Duration(milliseconds: 70),
        ),
        ui.PointerData(
          change: ui.PointerChange.remove,
          physicalX: 50.0,
          timeStamp: epoch + const Duration(milliseconds: 70),
        ),
      ],
    );

    const Duration samplingOffset = Duration(milliseconds: -5);
    const Duration frameInterval = Duration(microseconds: 16667);

    GestureBinding.instance.resamplingEnabled = true;
    GestureBinding.instance.samplingOffset = samplingOffset;

    final List<PointerEvent> events = <PointerEvent>[];
    binding.callback = events.add;

    GestureBinding.instance.platformDispatcher.onPointerDataPacket?.call(packet);

    // No pointer events should have been dispatched yet.
    expect(events.length, 0);

    // Frame callback should have been requested.
    FrameCallback? callback = binding.postFrameCallback;
    binding.postFrameCallback = null;
    expect(callback, isNotNull);

    binding.frameTime = epoch + const Duration(milliseconds: 15);
    callback!(Duration.zero);

    // One pointer event should have been dispatched.
    expect(events.length, 1);
    expect(events[0], isA<PointerDownEvent>());
    expect(events[0].timeStamp, binding.frameTime! + samplingOffset);
    expect(events[0].position, Offset(0.0 / _devicePixelRatio, 0.0));

    // Second frame callback should have been requested.
    callback = binding.postFrameCallback;
    binding.postFrameCallback = null;
    expect(callback, isNotNull);

    final Duration frameTime = epoch + const Duration(milliseconds: 25);
    binding.frameTime = frameTime;
    callback!(Duration.zero);

    // Second pointer event should have been dispatched.
    expect(events.length, 2);
    expect(events[1], isA<PointerMoveEvent>());
    expect(events[1].timeStamp, binding.frameTime! + samplingOffset);
    expect(events[1].position, Offset(10.0 / _devicePixelRatio, 0.0));
    expect(events[1].delta, Offset(10.0 / _devicePixelRatio, 0.0));

    // Verify that resampling continues without a frame callback.
    async.elapse(frameInterval * 1.5);

    // Third pointer event should have been dispatched.
    expect(events.length, 3);
    expect(events[2], isA<PointerMoveEvent>());
    expect(events[2].timeStamp, frameTime + frameInterval + samplingOffset);

    async.elapse(frameInterval);

    // Remaining pointer events should have been dispatched.
    expect(events.length, 5);
    expect(events[3], isA<PointerMoveEvent>());
    expect(events[3].timeStamp, frameTime + frameInterval * 2 + samplingOffset);
    expect(events[4], isA<PointerUpEvent>());
    expect(events[4].timeStamp, frameTime + frameInterval * 2 + samplingOffset);

    async.elapse(frameInterval);

    // No more pointer events should have been dispatched.
    expect(events.length, 5);

    GestureBinding.instance.resamplingEnabled = false;
  });
}

double get _devicePixelRatio =>
    GestureBinding.instance.platformDispatcher.implicitView!.devicePixelRatio;
