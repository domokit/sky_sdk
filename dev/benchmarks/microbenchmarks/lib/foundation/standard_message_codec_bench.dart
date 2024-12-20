// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';

import '../common.dart';

const int _kNumIterations = 100000;

Future<void> execute() async {
  assert(false, "Don't run benchmarks in debug mode! Use 'flutter run --release'.");
  final BenchmarkResultPrinter printer = BenchmarkResultPrinter();

  const StandardMessageCodec codec = StandardMessageCodec();
  final Stopwatch watch = Stopwatch();
  watch.start();
  for (int i = 0; i < _kNumIterations; i += 1) {
    codec.encodeMessage(null);
  }
  watch.stop();

  printer.addResult(
    description: 'StandardMessageCodec null',
    value: watch.elapsedMicroseconds.toDouble() / _kNumIterations,
    unit: 'us per iteration',
    name: 'StandardMessageCodec_null',
  );

  watch.reset();
  watch.start();
  for (int i = 0; i < _kNumIterations; i += 1) {
    codec.encodeMessage(12345);
  }
  watch.stop();

  printer.addResult(
    description: 'StandardMessageCodec int',
    value: watch.elapsedMicroseconds.toDouble() / _kNumIterations,
    unit: 'us per iteration',
    name: 'StandardMessageCodec_int',
  );

  watch.reset();

  watch.start();
  for (int i = 0; i < _kNumIterations; i += 1) {
    codec.encodeMessage('This is a performance test.');
  }
  watch.stop();

  printer.addResult(
    description: 'StandardMessageCodec string',
    value: watch.elapsedMicroseconds.toDouble() / _kNumIterations,
    unit: 'us per iteration',
    name: 'StandardMessageCodec_string',
  );

  watch.reset();
  watch.start();
  for (int i = 0; i < _kNumIterations; i += 1) {
    codec.encodeMessage(<Object>[1234, 'This is a performance test.', 1.25, true]);
  }
  watch.stop();

  printer.addResult(
    description: 'StandardMessageCodec heterogenous list',
    value: watch.elapsedMicroseconds.toDouble() / _kNumIterations,
    unit: 'us per iteration',
    name: 'StandardMessageCodec_heterogenous_list',
  );

  watch.reset();
  watch.start();
  for (int i = 0; i < _kNumIterations; i += 1) {
    codec.encodeMessage(<String, Object>{
      'integer': 1234,
      'string': 'This is a performance test.',
      'float': 1.25,
      'boolean': true,
    });
  }
  watch.stop();

  printer.addResult(
    description: 'StandardMessageCodec heterogenous map',
    value: watch.elapsedMicroseconds.toDouble() / _kNumIterations,
    unit: 'us per iteration',
    name: 'StandardMessageCodec_heterogenous_map',
  );

  watch.reset();

  watch.start();
  for (int i = 0; i < _kNumIterations; i += 1) {
    codec.encodeMessage('special chars >\u263A\u{1F602}<');
  }
  watch.stop();

  printer.addResult(
    description: 'StandardMessageCodec unicode',
    value: watch.elapsedMicroseconds.toDouble() / _kNumIterations,
    unit: 'us per iteration',
    name: 'StandardMessageCodec_unicode',
  );

  watch.reset();

  printer.printToStdout();
}
