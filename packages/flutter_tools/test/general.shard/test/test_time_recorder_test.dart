// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/test/test_time_recorder.dart';

import '../../src/common.dart';
import '../../src/fakes.dart';
import '../../src/logging_logger.dart';

void main() {
  testWithoutContext('Test phases prints correctly', () {
    const Duration zero = Duration.zero;
    const Duration combinedDuration = Duration(seconds: 42);
    const Duration wallClockDuration = Duration(seconds: 21);

    for (final TestTimePhases phase in TestTimePhases.values) {
      final TestTimeRecorder recorder = createRecorderWithTimesForPhase(
        phase,
        combinedDuration,
        wallClockDuration,
      );
      final Set<String> prints = recorder.getPrintAsListForTesting().toSet();

      // Expect one entry per phase.
      expect(prints, hasLength(TestTimePhases.values.length));

      // Expect this phase to have the specified times.
      expect(
        prints,
        contains(
          'Runtime for phase ${phase.name}: '
          'Wall-clock: $wallClockDuration; combined: $combinedDuration.',
        ),
      );

      // Expect all other phases to say 0.
      for (final TestTimePhases innerPhase in TestTimePhases.values) {
        if (phase == innerPhase) {
          continue;
        }
        expect(
          prints,
          contains(
            'Runtime for phase ${innerPhase.name}: '
            'Wall-clock: $zero; combined: $zero.',
          ),
        );
      }
    }
  });
}

TestTimeRecorder createRecorderWithTimesForPhase(
  TestTimePhases phase,
  Duration combinedDuration,
  Duration wallClockDuration,
) {
  final LoggingLogger logger = LoggingLogger();
  final TestTimeRecorder recorder = TestTimeRecorder(
    logger,
    stopwatchFactory: FakeStopwatchFactory(),
  );
  final FakeStopwatch combinedStopwatch = recorder.start(phase) as FakeStopwatch;
  final FakeStopwatch wallClockStopwatch =
      recorder.getPhaseWallClockStopwatchForTesting(phase) as FakeStopwatch;
  wallClockStopwatch.elapsed = wallClockDuration;
  combinedStopwatch.elapsed = combinedDuration;
  recorder.stop(phase, combinedStopwatch);
  return recorder;
}
