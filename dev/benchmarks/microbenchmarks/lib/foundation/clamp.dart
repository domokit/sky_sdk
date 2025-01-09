// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart' show clampDouble;

import '../common.dart';

const int _kBatchSize = 100000;
const int _kNumIterations = 1000;

Future<void> execute() async {
  assert(false, "Don't run benchmarks in debug mode! Use 'flutter run --release'.");
  final BenchmarkResultPrinter printer = BenchmarkResultPrinter();

  final Stopwatch watch = Stopwatch();
  {
    final List<double> clampDoubleValues = <double>[];
    for (int j = 0; j < _kNumIterations; ++j) {
      double tally = 0;
      watch.reset();
      watch.start();
      for (int i = 0; i < _kBatchSize; i += 1) {
        tally += clampDouble(-1.0, 0.0, 1.0);
        tally += clampDouble(2.0, 0.0, 1.0);
        tally += clampDouble(0.0, 0.0, 1.0);
        tally += clampDouble(double.nan, 0.0, 1.0);
      }
      watch.stop();
      clampDoubleValues.add(watch.elapsedMicroseconds.toDouble() / _kBatchSize);
      if (tally < 0.0) {
        print("This shouldn't happen.");
      }
    }

    printer.addResultStatistics(
      description: 'clamp - clampDouble',
      values: clampDoubleValues,
      unit: 'us per iteration',
      name: 'clamp_clampDouble',
    );
  }

  {
    final List<double> doubleClampValues = <double>[];

    for (int j = 0; j < _kNumIterations; ++j) {
      double tally = 0;
      watch.reset();
      watch.start();
      for (int i = 0; i < _kBatchSize; i += 1) {
        tally += -1.0.clamp(0.0, 1.0);
        tally += 2.0.clamp(0.0, 1.0);
        tally += 0.0.clamp(0.0, 1.0);
        tally += double.nan.clamp(0.0, 1.0);
      }
      watch.stop();
      doubleClampValues.add(watch.elapsedMicroseconds.toDouble() / _kBatchSize);
      if (tally < 0.0) {
        print("This shouldn't happen.");
      }
    }

    printer.addResultStatistics(
      description: 'clamp - Double.clamp',
      values: doubleClampValues,
      unit: 'us per iteration',
      name: 'clamp_Double_clamp',
    );
  }
  printer.printToStdout();
}
