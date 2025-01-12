// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/scheduler.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:stocks/main.dart' as stocks;
import 'package:stocks/stock_data.dart' as stock_data;

import '../benchmark_binding.dart';
import '../common.dart';

const Duration kBenchmarkTime = Duration(seconds: 15);

Future<void> execute(BenchmarkingBinding binding) async {
  assert(false, "Don't run benchmarks in debug mode! Use 'flutter run --release'.");
  stock_data.StockData.actuallyFetchData = false;

  final Stopwatch wallClockWatch = Stopwatch();

  int totalOpenFrameElapsedMicroseconds = 0;
  int totalOpenIterationCount = 0;
  int totalCloseFrameElapsedMicroseconds = 0;
  int totalCloseIterationCount = 0;
  int totalSubsequentFramesElapsedMicroseconds = 0;
  int totalSubsequentFramesIterationCount = 0;

  await benchmarkWidgets((WidgetTester tester) async {
    stocks.main();
    await tester.pump(); // Start startup animation
    await tester.pump(const Duration(seconds: 1)); // Complete startup animation

    bool drawerIsOpen = false;
    wallClockWatch.start();
    final drawerController = tester.getCenter(find.byType(DrawerController));
    final navigationMenu = tester.getCenter(find.byTooltip('Open navigation menu'));
    while (wallClockWatch.elapsed < kBenchmarkTime) {
      binding.drawFrameWatch.reset();
      if (drawerIsOpen) {
        await tester.tapAt(drawerController); // Close drawer
        await tester.pump();
        totalCloseIterationCount += 1;
        totalCloseFrameElapsedMicroseconds += binding.drawFrameWatch.elapsedMicroseconds;
      } else {
        await tester.tapAt(navigationMenu); // Open drawer
        await tester.pump();
        totalOpenIterationCount += 1;
        totalOpenFrameElapsedMicroseconds += binding.drawFrameWatch.elapsedMicroseconds;
      }
      drawerIsOpen = !drawerIsOpen;

      // Time how long each frame takes
      binding.drawFrameWatch.reset();
      while (SchedulerBinding.instance.hasScheduledFrame) {
        await tester.pump();
        totalSubsequentFramesIterationCount += 1;
      }
      totalSubsequentFramesElapsedMicroseconds += binding.drawFrameWatch.elapsedMicroseconds;
    }
  });

  final BenchmarkResultPrinter printer = BenchmarkResultPrinter();
  printer.addResult(
    description: 'Stock animation',
    value: wallClockWatch.elapsedMicroseconds / (1000 * 1000),
    unit: 's',
    name: 'stock_animation_total_run_time',
  );
  if (totalOpenIterationCount > 0) {
    printer.addResult(
      description: '  Opening first frame average time',
      value: totalOpenFrameElapsedMicroseconds / totalOpenIterationCount,
      unit: 'µs per frame ($totalOpenIterationCount frames)',
      name: 'stock_animation_open_first_frame_average',
    );
  }
  if (totalCloseIterationCount > 0) {
    printer.addResult(
      description: '  Closing first frame average time',
      value: totalCloseFrameElapsedMicroseconds / totalCloseIterationCount,
      unit: 'µs per frame ($totalCloseIterationCount frames)',
      name: 'stock_animation_close_first_frame_average',
    );
  }
  if (totalSubsequentFramesIterationCount > 0) {
    printer.addResult(
      description: '  Subsequent frames average time',
      value: totalSubsequentFramesElapsedMicroseconds / totalSubsequentFramesIterationCount,
      unit: 'µs per frame ($totalSubsequentFramesIterationCount frames)',
      name: 'stock_animation_subsequent_frame_average',
    );
  }
  printer.printToStdout();
}

//
//  Note that the benchmark is normally run by benchmark_collection.dart.
//
Future<void> main() async {
  return execute(BenchmarkingBinding());
}
