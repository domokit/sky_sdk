// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

part of dart.ui;

/// Same as [num.clamp] but optimized for a non-null [double].
///
/// This is faster because it avoids polymorphism, boxing, and special cases for
/// floating point numbers.
//
// See also: //dev/benchmarks/microbenchmarks/lib/foundation/clamp.dart
double clampDouble(double x, double min, double max) {
  assert(min <= max && !max.isNaN && !min.isNaN);
  if (x < min) {
    return min;
  }
  if (x > max) {
    return max;
  }
  if (x.isNaN) {
    return max;
  }
  return x;
}
