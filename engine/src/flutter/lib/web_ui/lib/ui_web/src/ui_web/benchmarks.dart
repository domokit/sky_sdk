// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/src/engine.dart';

/// Signature of the callback that receives a benchmark [value] labeled by
/// [name].
typedef BenchmarkValueCallback = void Function(String name, double value);

/// A callback for receiving benchmark values.
///
/// Each benchmark value is labeled by a `name` and has a double `value`.
set benchmarkValueCallback(BenchmarkValueCallback? callback) {
  engineBenchmarkValueCallback = callback;
}
