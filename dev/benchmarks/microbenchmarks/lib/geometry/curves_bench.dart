// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/animation.dart';

import '../common.dart';

const int _kNumIters = 10000;

void _testCurve(
  Curve curve, {
  required String name,
  required String description,
  required BenchmarkResultPrinter printer,
}) {
  final Stopwatch watch = Stopwatch();
  print('$description benchmark...');
  watch.start();
  for (int i = 0; i < _kNumIters; i += 1) {
    final double t = i / _kNumIters.toDouble();
    curve.transform(t);
  }
  watch.stop();

  printer.addResult(
    description: description,
    value: watch.elapsedMicroseconds / _kNumIters,
    unit: 'µs per iteration',
    name: name,
  );
}

void main() {
  assert(false, "Don't run benchmarks in debug mode! Use 'flutter run --release'.");
  final BenchmarkResultPrinter printer = BenchmarkResultPrinter();
  _testCurve(
    const Cubic(0.0, 0.25, 0.5, 1.0),
    name: 'cubic_animation_transform_iteration',
    description: 'Cubic animation transform',
    printer: printer,
  );

  final CatmullRomCurve catmullRomCurve = CatmullRomCurve(const <Offset>[
    Offset(0.09, 0.99),
    Offset(0.21, 0.01),
    Offset(0.28, 0.99),
    Offset(0.38, -0.00),
    Offset(0.43, 0.99),
    Offset(0.54, -0.01),
    Offset(0.59, 0.98),
    Offset(0.70, 0.04),
    Offset(0.78, 0.98),
    Offset(0.88, -0.00),
  ]);
  _testCurve(
    catmullRomCurve,
    name: 'catmullrom_transform_iteration',
    description: 'CatmullRomCurve animation transform',
    printer: printer,
  );

  printer.printToStdout();
}
