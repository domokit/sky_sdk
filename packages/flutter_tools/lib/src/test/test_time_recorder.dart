// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';

import '../base/logger.dart';

/// Utility class that can record time used in different phases of a test run.
class TestTimeRecorder {
  TestTimeRecorder(this.logger, {this.stopwatchFactory = const StopwatchFactory()})
    : _phaseRecords = List<TestTimeRecord>.generate(
        TestTimePhases.values.length,
        (_) => TestTimeRecord(stopwatchFactory),
      );

  final List<TestTimeRecord> _phaseRecords;
  final Logger logger;
  final StopwatchFactory stopwatchFactory;

  Stopwatch start(TestTimePhases phase) {
    return _phaseRecords[phase.index].start();
  }

  void stop(TestTimePhases phase, Stopwatch stopwatch) {
    _phaseRecords[phase.index].stop(stopwatch);
  }

  void print() {
    for (final TestTimePhases phase in TestTimePhases.values) {
      logger.printTrace(_getPrintStringForPhase(phase));
    }
  }

  @visibleForTesting
  List<String> getPrintAsListForTesting() {
    return TestTimePhases.values.map(_getPrintStringForPhase).toList();
  }

  @visibleForTesting
  Stopwatch getPhaseWallClockStopwatchForTesting(final TestTimePhases phase) {
    return _phaseRecords[phase.index]._wallClockRuntime;
  }

  String _getPrintStringForPhase(final TestTimePhases phase) {
    assert(_phaseRecords[phase.index].isDone());
    return 'Runtime for phase ${phase.name}: ${_phaseRecords[phase.index]}';
  }
}

/// Utility class that can record time used in a specific phase of a test run.
class TestTimeRecord {
  TestTimeRecord(this.stopwatchFactory) : _wallClockRuntime = stopwatchFactory.createStopwatch();

  final StopwatchFactory stopwatchFactory;
  Duration _combinedRuntime = Duration.zero;
  final Stopwatch _wallClockRuntime;
  int _currentlyRunningCount = 0;

  Stopwatch start() {
    final Stopwatch stopwatch = stopwatchFactory.createStopwatch()..start();
    if (_currentlyRunningCount == 0) {
      _wallClockRuntime.start();
    }
    _currentlyRunningCount++;
    return stopwatch;
  }

  void stop(Stopwatch stopwatch) {
    _currentlyRunningCount--;
    if (_currentlyRunningCount == 0) {
      _wallClockRuntime.stop();
    }
    _combinedRuntime = _combinedRuntime + stopwatch.elapsed;
    assert(_currentlyRunningCount >= 0);
  }

  @override
  String toString() {
    return 'Wall-clock: ${_wallClockRuntime.elapsed}; combined: $_combinedRuntime.';
  }

  bool isDone() {
    return _currentlyRunningCount == 0;
  }
}

enum TestTimePhases {
  /// Covers entire TestRunner run, i.e. from the test runner was asked to `runTests` until it is done.
  ///
  /// This implicitly includes all the other phases.
  TestRunner,

  /// Covers time spent compiling, including subsequent copying of files.
  Compile,

  /// Covers time spent running, i.e. from starting the test device until the test has finished.
  Run,

  /// Covers all time spent collecting coverage.
  CoverageTotal,

  /// Covers collecting the coverage, but not parsing nor adding to hit map.
  ///
  /// This is included in [CoverageTotal]
  CoverageCollect,

  /// Covers parsing the json from the coverage collected.
  ///
  /// This is included in [CoverageTotal]
  CoverageParseJson,

  /// Covers adding the parsed coverage to the hitmap.
  ///
  /// This is included in [CoverageTotal]
  CoverageAddHitmap,

  /// Covers time spent in `collectCoverageData`, mostly formatting and writing coverage data to file.
  CoverageDataCollect,

  /// Covers time spend in the Watchers `handleFinishedTest` call. This is probably collecting coverage.
  WatcherFinishedTest,
}
