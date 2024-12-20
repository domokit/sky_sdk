// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';

import '../common.dart';

const int _kNumIterations = 10;
const int _kNumWarmUp = 100;

class Data {
  Data(this.value);

  final int value;

  @override
  String toString() => 'Data($value)';
}

List<Data> test(int length) {
  return List<Data>.generate(length, (int index) => Data(index * index));
}

Future<void> execute() async {
  assert(false, "Don't run benchmarks in debug mode! Use 'flutter run --release'.");

  // Warm up lap
  for (int i = 0; i < _kNumWarmUp; i += 1) {
    await compute(test, 10);
  }

  final Stopwatch watch = Stopwatch();
  watch.start();
  for (int i = 0; i < _kNumIterations; i += 1) {
    await compute(test, 1000000);
  }
  final int elapsedMicroseconds = watch.elapsedMicroseconds;

  final BenchmarkResultPrinter printer = BenchmarkResultPrinter();
  const double scale = 1000.0 / _kNumIterations;
  printer.addResult(
    description: 'compute',
    value: elapsedMicroseconds * scale,
    unit: 'ns per iteration',
    name: 'compute_iteration',
  );
  printer.printToStdout();
}
