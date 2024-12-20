// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

part of ui;

abstract class PathMetrics extends collection.IterableBase<PathMetric> {
  @override
  Iterator<PathMetric> get iterator;
}

abstract class PathMetricIterator implements Iterator<PathMetric> {
  @override
  PathMetric get current;

  @override
  bool moveNext();
}

abstract class PathMetric {
  double get length;
  int get contourIndex;
  Tangent? getTangentForOffset(double distance);
  Path extractPath(double start, double end, {bool startWithMoveTo = true});
  bool get isClosed;
}

class Tangent {
  const Tangent(this.position, this.vector);
  factory Tangent.fromAngle(Offset position, double angle) {
    return Tangent(position, Offset(math.cos(angle), math.sin(angle)));
  }
  final Offset position;
  final Offset vector;
  // flip the sign to be consistent with [Path.arcTo]'s `sweepAngle`
  double get angle => -math.atan2(vector.dy, vector.dx);
}
