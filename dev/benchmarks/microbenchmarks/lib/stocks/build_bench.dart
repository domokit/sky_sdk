// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:stocks/main.dart' as stocks;
import 'package:stocks/stock_data.dart' as stock_data;

import '../common.dart';

const Duration kBenchmarkTime = Duration(seconds: 15);

Future<List<double>> runBuildBenchmark() async {
  assert(false, "Don't run benchmarks in debug mode! Use 'flutter run --release'.");
  stock_data.StockData.actuallyFetchData = false;

  // We control the framePolicy below to prevent us from scheduling frames in
  // the engine, so that the engine does not interfere with our timings.
  final LiveTestWidgetsFlutterBinding binding =
      TestWidgetsFlutterBinding.ensureInitialized() as LiveTestWidgetsFlutterBinding;

  final Stopwatch watch = Stopwatch();
  int iterations = 0;
  final List<double> values = <double>[];

  await benchmarkWidgets((WidgetTester tester) async {
    stocks.main();
    await tester.pump(); // Start startup animation
    await tester.pump(const Duration(seconds: 1)); // Complete startup animation
    await tester.tapAt(const Offset(20.0, 40.0)); // Open drawer
    await tester.pump(); // Start drawer animation
    await tester.pumpAndSettle(const Duration(seconds: 1)); // Complete drawer animation

    final Element appState = tester.element(find.byType(stocks.StocksApp));
    binding.framePolicy = LiveTestWidgetsFlutterBindingFramePolicy.benchmark;

    Duration elapsed = Duration.zero;
    while (elapsed < kBenchmarkTime) {
      watch.reset();
      watch.start();
      appState.markNeedsBuild();
      // We don't use tester.pump() because we're trying to drive it in an
      // artificially high load to find out how much CPU each frame takes.
      // This differs from normal benchmarks which might look at how many
      // frames are missed, etc.
      // We use Timer.run to ensure there's a microtask flush in between
      // the two calls below.
      await tester.pumpBenchmark(Duration(milliseconds: iterations * 16));
      watch.stop();
      iterations += 1;
      elapsed += Duration(microseconds: watch.elapsedMicroseconds);
      values.add(watch.elapsedMicroseconds.toDouble());
    }
  });
  return values;
}

Future<void> execute() async {
  final BenchmarkResultPrinter printer = BenchmarkResultPrinter();
  printer.addResultStatistics(
    description: 'Stock build',
    values: await runBuildBenchmark(),
    unit: 'µs per iteration',
    name: 'stock_build_iteration',
  );
  printer.printToStdout();
}

//
//  Note that the benchmark is normally run by benchmark_collection.dart.
//
Future<void> main() async {
  return execute();
}
