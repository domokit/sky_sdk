// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';

import '../common.dart';
import 'data/velocity_tracker_data.dart';

const int _kNumIters = 10000;

class TrackerBenchmark {
  TrackerBenchmark({required this.name, required this.tracker});

  final VelocityTracker tracker;
  final String name;
}

Future<void> execute() async {
  assert(false, "Don't run benchmarks in debug mode! Use 'flutter run --release'.");
  final BenchmarkResultPrinter printer = BenchmarkResultPrinter();
  final List<TrackerBenchmark> benchmarks = <TrackerBenchmark>[
    TrackerBenchmark(
      name: 'velocity_tracker_iteration',
      tracker: VelocityTracker.withKind(PointerDeviceKind.touch),
    ),
    TrackerBenchmark(
      name: 'velocity_tracker_iteration_ios_fling',
      tracker: IOSScrollViewFlingVelocityTracker(PointerDeviceKind.touch),
    ),
  ];
  final Stopwatch watch = Stopwatch();

  await benchmarkWidgets((WidgetTester tester) async {
    for (final TrackerBenchmark benchmark in benchmarks) {
      print('${benchmark.name} benchmark...');
      final VelocityTracker tracker = benchmark.tracker;
      watch.reset();
      watch.start();
      for (int i = 0; i < _kNumIters; i += 1) {
        for (final PointerEvent event in velocityEventData) {
          if (event is PointerDownEvent || event is PointerMoveEvent) {
            tracker.addPosition(event.timeStamp, event.position);
          }
          if (event is PointerUpEvent) {
            tracker.getVelocity();
          }
        }
      }
      watch.stop();

      printer.addResult(
        description: 'Velocity tracker: ${tracker.runtimeType}',
        value: watch.elapsedMicroseconds / _kNumIters,
        unit: 'µs per iteration',
        name: benchmark.name,
      );
    }
  });

  printer.printToStdout();
}

//
//  Note that the benchmark is normally run by benchmark_collection.dart.
//
Future<void> main() async {
  return execute();
}
