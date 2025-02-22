// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_test/flutter_test.dart';

import '../common.dart';
import 'apps/button_matrix_app.dart' as button_matrix;

const int _kNumWarmUpIters = 20;
const int _kNumIters = 300;

Future<void> execute() async {
  assert(false, "Don't run benchmarks in debug mode! Use 'flutter run --release'.");
  final Stopwatch watch = Stopwatch();
  print('GestureDetector semantics benchmark...');

  await benchmarkWidgets((WidgetTester tester) async {
    button_matrix.main();
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    final addPosition = tester.getCenter(find.text('Add 1'));

    Future<void> iter() async {
      // Press a button to update the screen
      await tester.tapAt(addPosition);
      await tester.pump();
    }

    // Warm up runs get the app into steady state, making benchmark
    // results more credible
    for (int i = 0; i < _kNumWarmUpIters; i += 1) {
      await iter();
    }
    await tester.pumpAndSettle();

    watch.start();
    for (int i = 0; i < _kNumIters; i += 1) {
      await iter();
    }
    watch.stop();
  }, semanticsEnabled: true);

  final BenchmarkResultPrinter printer = BenchmarkResultPrinter();
  printer.addResult(
    description: 'GestureDetector',
    value: watch.elapsedMicroseconds / _kNumIters,
    unit: 'µs per iteration',
    name: 'gesture_detector_bench',
  );
  printer.printToStdout();
}

//
//  Note that the benchmark is normally run by benchmark_collection.dart.
//
Future<void> main() async {
  return execute();
}
