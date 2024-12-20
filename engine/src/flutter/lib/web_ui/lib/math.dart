// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

part of ui;

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
