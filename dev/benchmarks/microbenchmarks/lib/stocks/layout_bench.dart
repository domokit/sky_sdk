// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:stocks/main.dart' as stocks;
import 'package:stocks/stock_data.dart' as stock_data;

import '../common.dart';

const Duration kBenchmarkTime = Duration(seconds: 15);

Future<void> execute() async {
  assert(false, "Don't run benchmarks in debug mode! Use 'flutter run --release'.");
  stock_data.StockData.actuallyFetchData = false;

  // We control the framePolicy below to prevent us from scheduling frames in
  // the engine, so that the engine does not interfere with our timings.
  final LiveTestWidgetsFlutterBinding binding =
      TestWidgetsFlutterBinding.ensureInitialized() as LiveTestWidgetsFlutterBinding;

  final Stopwatch watch = Stopwatch();
  int iterations = 0;

  await benchmarkWidgets((WidgetTester tester) async {
    stocks.main();
    await tester.pump(); // Start startup animation
    await tester.pump(const Duration(seconds: 1)); // Complete startup animation
    await tester.tapAt(const Offset(20.0, 40.0)); // Open drawer
    await tester.pump(); // Start drawer animation
    await tester.pump(const Duration(seconds: 1)); // Complete drawer animation

    final TestViewConfiguration big = TestViewConfiguration.fromView(
      size: const Size(360.0, 640.0),
      view: tester.view,
    );
    final TestViewConfiguration small = TestViewConfiguration.fromView(
      size: const Size(355.0, 635.0),
      view: tester.view,
    );
    final RenderView renderView = WidgetsBinding.instance.renderViews.single;
    binding.framePolicy = LiveTestWidgetsFlutterBindingFramePolicy.benchmark;

    watch.start();
    while (watch.elapsed < kBenchmarkTime) {
      renderView.configuration = iterations.isEven ? big : small;
      await tester.pumpBenchmark(Duration(milliseconds: iterations * 16));
      iterations += 1;
    }
    watch.stop();
  });

  final BenchmarkResultPrinter printer = BenchmarkResultPrinter();
  printer.addResult(
    description: 'Stock layout',
    value: watch.elapsedMicroseconds / iterations,
    unit: 'µs per iteration',
    name: 'stock_layout_iteration',
  );
  printer.printToStdout();
}

//
//  Note that the benchmark is normally run by benchmark_collection.dart.
//
Future<void> main() async {
  return execute();
}
