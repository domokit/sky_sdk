// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import '../common.dart';

const Duration kBenchmarkTime = Duration(seconds: 15);

// Use an Align to loosen the constraints.
final Widget intrinsicTextHeight = Directionality(
  textDirection: TextDirection.ltr,
  child: Align(child: IntrinsicHeight(child: Text('A' * 100))),
);

Future<void> execute() async {
  assert(false, "Don't run benchmarks in debug mode! Use 'flutter run --release'.");

  // We control the framePolicy below to prevent us from scheduling frames in
  // the engine, so that the engine does not interfere with our timings.
  final LiveTestWidgetsFlutterBinding binding =
      TestWidgetsFlutterBinding.ensureInitialized() as LiveTestWidgetsFlutterBinding;

  final Stopwatch watch = Stopwatch();
  int iterations = 0;

  await benchmarkWidgets((WidgetTester tester) async {
    runApp(intrinsicTextHeight);
    // Wait for the UI to stabilize.
    await tester.pumpAndSettle(const Duration(seconds: 1));

    final TestViewConfiguration big = TestViewConfiguration.fromView(
      size: const Size(360.0, 640.0),
      view: tester.view,
    );
    final TestViewConfiguration small = TestViewConfiguration.fromView(
      size: const Size(100.0, 640.0),
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
    description: 'Text intrinsic height',
    value: watch.elapsedMicroseconds / iterations,
    unit: 'µs per iteration',
    name: 'text_intrinsic_height_iteration',
  );
  printer.printToStdout();
}
