// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This test is a use case of flutter/flutter#60796
// the test should be run as:
// flutter drive -t test/using_array.dart --driver test_driver/scrolling_test_e2e_test.dart

import 'package:complex_layout/main.dart' as app;
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';

/// Generates the [PointerEvent] to simulate a drag operation from
/// `center - totalMove/2` to `center + totalMove/2`.
Iterable<PointerEvent> dragInputEvents(
  final Duration epoch,
  final Offset center, {
  final Offset totalMove = const Offset(0, -400),
  final Duration totalTime = const Duration(milliseconds: 2000),
  final double frequency = 90,
}) sync* {
  final Offset startLocation = center - totalMove / 2;
  // The issue is about 120Hz input on 90Hz refresh rate device.
  // We test 90Hz input on 60Hz device here, which shows similar pattern.
  final int moveEventCount =
      totalTime.inMicroseconds * frequency ~/ const Duration(seconds: 1).inMicroseconds;
  final Offset movePerEvent = totalMove / moveEventCount.toDouble();
  yield PointerAddedEvent(timeStamp: epoch, position: startLocation);
  yield PointerDownEvent(timeStamp: epoch, position: startLocation, pointer: 1);
  for (int t = 0; t < moveEventCount + 1; t++) {
    final Offset position = startLocation + movePerEvent * t.toDouble();
    yield PointerMoveEvent(
      timeStamp: epoch + totalTime * t ~/ moveEventCount,
      position: position,
      delta: movePerEvent,
      pointer: 1,
    );
  }
  final Offset position = startLocation + totalMove;
  yield PointerUpEvent(timeStamp: epoch + totalTime, position: position, pointer: 1);
}

enum TestScenario { resampleOn90Hz, resampleOn59Hz, resampleOff90Hz, resampleOff59Hz }

class ResampleFlagVariant extends TestVariant<TestScenario> {
  ResampleFlagVariant(this.binding);
  final IntegrationTestWidgetsFlutterBinding binding;

  @override
  final Set<TestScenario> values = Set<TestScenario>.from(TestScenario.values);

  late TestScenario currentValue;

  bool get resample => switch (currentValue) {
    TestScenario.resampleOn90Hz || TestScenario.resampleOn59Hz => true,
    TestScenario.resampleOff90Hz || TestScenario.resampleOff59Hz => false,
  };

  double get frequency => switch (currentValue) {
    TestScenario.resampleOn90Hz || TestScenario.resampleOff90Hz => 90.0,
    TestScenario.resampleOn59Hz || TestScenario.resampleOff59Hz => 59.0,
  };

  Map<String, dynamic>? result;

  @override
  String describeValue(TestScenario value) {
    return switch (value) {
      TestScenario.resampleOn90Hz => 'resample on with 90Hz input',
      TestScenario.resampleOn59Hz => 'resample on with 59Hz input',
      TestScenario.resampleOff90Hz => 'resample off with 90Hz input',
      TestScenario.resampleOff59Hz => 'resample off with 59Hz input',
    };
  }

  @override
  Future<bool> setUp(TestScenario value) async {
    currentValue = value;
    final bool original = binding.resamplingEnabled;
    binding.resamplingEnabled = resample;
    return original;
  }

  @override
  Future<void> tearDown(TestScenario value, bool memento) async {
    binding.resamplingEnabled = memento;
    binding.reportData![describeValue(value)] = result;
  }
}

Future<void> main() async {
  final WidgetsBinding widgetsBinding = IntegrationTestWidgetsFlutterBinding.ensureInitialized();
  assert(widgetsBinding is IntegrationTestWidgetsFlutterBinding);
  final IntegrationTestWidgetsFlutterBinding binding =
      widgetsBinding as IntegrationTestWidgetsFlutterBinding;
  binding.framePolicy = LiveTestWidgetsFlutterBindingFramePolicy.benchmarkLive;
  binding.reportData ??= <String, dynamic>{};
  final ResampleFlagVariant variant = ResampleFlagVariant(binding);
  testWidgets(
    'Smoothness test',
    (WidgetTester tester) async {
      app.main();
      await tester.pumpAndSettle();
      final Finder scrollerFinder = find.byKey(const ValueKey<String>('complex-scroll'));
      final ListView scroller = tester.widget<ListView>(scrollerFinder);
      final ScrollController? controller = scroller.controller;
      final List<int> frameTimestamp = <int>[];
      final List<double> scrollOffset = <double>[];
      final List<Duration> delays = <Duration>[];
      binding.addPersistentFrameCallback((Duration timeStamp) {
        if (controller?.hasClients ?? false) {
          // This if is necessary because by the end of the test the widget tree
          // is destroyed.
          frameTimestamp.add(timeStamp.inMicroseconds);
          scrollOffset.add(controller!.offset);
        }
      });

      Duration now() => binding.currentSystemFrameTimeStamp;
      Future<void> scroll() async {
        // Extra 50ms to avoid timeouts.
        final Duration startTime = const Duration(milliseconds: 500) + now();
        for (final PointerEvent event in dragInputEvents(
          startTime,
          tester.getCenter(scrollerFinder),
          frequency: variant.frequency,
        )) {
          await tester.binding.delayed(event.timeStamp - now());
          // This now measures how accurate the above delayed is.
          final Duration delay = now() - event.timeStamp;
          if (delays.length < frameTimestamp.length) {
            while (delays.length < frameTimestamp.length - 1) {
              delays.add(Duration.zero);
            }
            delays.add(delay);
          } else if (delays.last < delay) {
            delays.last = delay;
          }
          tester.binding.handlePointerEventForSource(event, source: TestBindingEventSource.test);
        }
      }

      for (int n = 0; n < 5; n++) {
        await scroll();
      }
      variant.result = scrollSummary(scrollOffset, delays, frameTimestamp);
      await tester.pumpAndSettle();
      scrollOffset.clear();
      delays.clear();
      await tester.idle();
    },
    semanticsEnabled: false,
    variant: variant,
  );
}

/// Calculates the smoothness measure from `scrollOffset` and `delays` list.
///
/// Smoothness (`abs_jerk`) is measured by the absolute value of the discrete
/// 2nd derivative of the scroll offset.
///
/// It was experimented that jerk (3rd derivative of the position) is a good
/// measure the smoothness.
/// Here we are using 2nd derivative instead because the input is completely
/// linear and the expected acceleration should be strictly zero.
/// Observed acceleration is jumping from positive to negative within
/// adjacent frames, meaning mathematically the discrete 3-rd derivative
/// (`f[3] - 3*f[2] + 3*f[1] - f[0]`) is not a good approximation of jerk
/// (continuous 3-rd derivative), while discrete 2nd
/// derivative (`f[2] - 2*f[1] + f[0]`) on the other hand is a better measure
/// of how the scrolling deviate away from linear, and given the acceleration
/// should average to zero within two frames, it's also a good approximation
/// for jerk in terms of physics.
/// We use abs rather than square because square (2-norm) amplifies the
/// effect of the data point that's relatively large, but in this metric
/// we prefer smaller data point to have similar effect.
/// This is also why we count the number of data that's larger than a
/// threshold (and the result is tested not sensitive to this threshold),
/// which is effectively a 0-norm.
///
/// Frames that are too slow to build (longer than 40ms) or with input delay
/// longer than 16ms (1/60Hz) is filtered out to separate the janky due to slow
/// response.
///
/// The returned map has keys:
/// `average_abs_jerk`: average for the overall smoothness. The smaller this
/// number the more smooth the scrolling is.
/// `janky_count`: number of frames with `abs_jerk` larger than 0.5. The frames
/// that take longer than the frame budget to build are ignored, so increase of
/// this number itself may not represent a regression.
/// `dropped_frame_count`: number of frames that are built longer than 40ms and
///  are not used for smoothness measurement.
/// `frame_timestamp`: the list of the timestamp for each frame, in the time
/// order.
/// `scroll_offset`: the scroll offset for each frame. Its length is the same as
/// `frame_timestamp`.
/// `input_delay`: the list of maximum delay time of the input simulation during
/// a frame. Its length is the same as `frame_timestamp`
Map<String, dynamic> scrollSummary(
  List<double> scrollOffset,
  List<Duration> delays,
  List<int> frameTimestamp,
) {
  double jankyCount = 0;
  double absJerkAvg = 0;
  int lostFrame = 0;
  for (int i = 1; i < scrollOffset.length - 1; i += 1) {
    if (frameTimestamp[i + 1] - frameTimestamp[i - 1] > 40E3 ||
        (i >= delays.length || delays[i] > const Duration(milliseconds: 16))) {
      // filter data points from slow frame building or input simulation artifact
      lostFrame += 1;
      continue;
    }
    //
    final double absJerk = (scrollOffset[i - 1] + scrollOffset[i + 1] - 2 * scrollOffset[i]).abs();
    absJerkAvg += absJerk;
    if (absJerk > 0.5) {
      jankyCount += 1;
    }
  }
  // expect(lostFrame < 0.1 * frameTimestamp.length, true);
  absJerkAvg /= frameTimestamp.length - lostFrame;

  return <String, dynamic>{
    'janky_count': jankyCount,
    'average_abs_jerk': absJerkAvg,
    'dropped_frame_count': lostFrame,
    'frame_timestamp': List<int>.from(frameTimestamp),
    'scroll_offset': List<double>.from(scrollOffset),
    'input_delay': delays.map<int>((Duration data) => data.inMicroseconds).toList(),
  };
}
