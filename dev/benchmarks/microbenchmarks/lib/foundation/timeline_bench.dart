// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';

import '../common.dart';

const int _kNumIterations = 10000;

Future<void> execute() async {
  assert(false, "Don't run benchmarks in debug mode! Use 'flutter run --release'.");
  final BenchmarkResultPrinter printer = BenchmarkResultPrinter();

  final Stopwatch watch = Stopwatch();
  watch.start();
  for (int i = 0; i < _kNumIterations; i += 1) {
    FlutterTimeline.startSync('foo');
    FlutterTimeline.finishSync();
  }
  watch.stop();

  printer.addResult(
    description: 'timeline events without arguments',
    value: watch.elapsedMicroseconds.toDouble() / _kNumIterations,
    unit: 'us per iteration',
    name: 'timeline_without_arguments',
  );

  watch.reset();
  watch.start();
  for (int i = 0; i < _kNumIterations; i += 1) {
    FlutterTimeline.startSync(
      'foo',
      arguments: <String, dynamic>{
        'int': 1234,
        'double': 0.3,
        'list': <int>[1, 2, 3, 4],
        'map': <String, dynamic>{'map': true},
        'bool': false,
      },
    );
    FlutterTimeline.finishSync();
  }
  watch.stop();

  printer.addResult(
    description: 'timeline events with arguments',
    value: watch.elapsedMicroseconds.toDouble() / _kNumIterations,
    unit: 'us per iteration',
    name: 'timeline_with_arguments',
  );

  printer.printToStdout();
}
