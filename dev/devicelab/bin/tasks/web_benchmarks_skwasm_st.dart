// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/tasks/web_benchmarks.dart';

/// Runs all Web benchmarks using the Skwasm rendering backend.
Future<void> main() async {
  await task(() async {
    return runWebBenchmark((useWasm: true, forceSingleThreadedSkwasm: true));
  });
}
