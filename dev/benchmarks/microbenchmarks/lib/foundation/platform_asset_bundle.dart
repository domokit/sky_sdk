// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart' show PlatformAssetBundle;
import 'package:flutter/widgets.dart';

import '../common.dart';

const int _kBatchSize = 100;
const int _kNumIterations = 100;

Future<void> execute() async {
  assert(false, "Don't run benchmarks in debug mode! Use 'flutter run --release'.");
  final BenchmarkResultPrinter printer = BenchmarkResultPrinter();
  WidgetsFlutterBinding.ensureInitialized();
  final Stopwatch watch = Stopwatch();
  final PlatformAssetBundle bundle = PlatformAssetBundle();

  final List<double> values = <double>[];
  for (int j = 0; j < _kNumIterations; ++j) {
    double tally = 0;
    watch.reset();
    watch.start();
    for (int i = 0; i < _kBatchSize; i += 1) {
      // We don't load images like this. PlatformAssetBundle is used for
      // other assets (like Rive animations). We are using an image because it's
      // conveniently sized and available for the test.
      tally +=
          (await bundle.load(
            'packages/flutter_gallery_assets/places/india_pondicherry_salt_farm.png',
          )).lengthInBytes;
    }
    watch.stop();
    values.add(watch.elapsedMicroseconds.toDouble() / _kBatchSize);
    if (tally < 0.0) {
      print("This shouldn't happen.");
    }
  }

  printer.addResultStatistics(
    description: 'PlatformAssetBundle.load 1MB',
    values: values,
    unit: 'us per iteration',
    name: 'PlatformAssetBundle_load_1MB',
  );

  printer.printToStdout();
}
