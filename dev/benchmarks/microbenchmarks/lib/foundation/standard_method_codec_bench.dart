// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';

import '../common.dart';

const int _kNumIterations = 100000;

Future<void> execute() async {
  assert(false, "Don't run benchmarks in debug mode! Use 'flutter run --release'.");
  final BenchmarkResultPrinter printer = BenchmarkResultPrinter();

  const StandardMethodCodec codec = StandardMethodCodec();
  final Stopwatch watch = Stopwatch();
  const String methodName = 'something';
  watch.start();
  for (int i = 0; i < _kNumIterations; i += 1) {
    codec.encodeMethodCall(const MethodCall(methodName));
  }
  watch.stop();

  printer.addResult(
    description: 'StandardMethodCodec null',
    value: watch.elapsedMicroseconds.toDouble() / _kNumIterations,
    unit: 'us per iteration',
    name: 'StandardMethodCodec_null',
  );

  watch.reset();
  watch.start();
  for (int i = 0; i < _kNumIterations; i += 1) {
    codec.encodeMethodCall(const MethodCall(methodName, 12345));
  }
  watch.stop();

  printer.addResult(
    description: 'StandardMethodCodec int',
    value: watch.elapsedMicroseconds.toDouble() / _kNumIterations,
    unit: 'us per iteration',
    name: 'StandardMethodCodec_int',
  );

  watch.reset();

  watch.start();
  for (int i = 0; i < _kNumIterations; i += 1) {
    codec.encodeMethodCall(const MethodCall(methodName, 'This is a performance test.'));
  }
  watch.stop();

  printer.addResult(
    description: 'StandardMethodCodec string',
    value: watch.elapsedMicroseconds.toDouble() / _kNumIterations,
    unit: 'us per iteration',
    name: 'StandardMethodCodec_string',
  );

  watch.reset();
  watch.start();
  for (int i = 0; i < _kNumIterations; i += 1) {
    codec.encodeMethodCall(
      const MethodCall(methodName, <Object>[1234, 'This is a performance test.', 1.25, true]),
    );
  }
  watch.stop();

  printer.addResult(
    description: 'StandardMethodCodec heterogenous list',
    value: watch.elapsedMicroseconds.toDouble() / _kNumIterations,
    unit: 'us per iteration',
    name: 'StandardMethodCodec_heterogenous_list',
  );

  watch.reset();
  watch.start();
  for (int i = 0; i < _kNumIterations; i += 1) {
    codec.encodeMethodCall(
      const MethodCall(methodName, <String, Object>{
        'integer': 1234,
        'string': 'This is a performance test.',
        'float': 1.25,
        'boolean': true,
      }),
    );
  }
  watch.stop();

  printer.addResult(
    description: 'StandardMethodCodec heterogenous map',
    value: watch.elapsedMicroseconds.toDouble() / _kNumIterations,
    unit: 'us per iteration',
    name: 'StandardMethodCodec_heterogenous_map',
  );

  watch.reset();

  printer.printToStdout();
}
