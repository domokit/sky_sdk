// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'basic_types.dart';

/// Draw a line between two points, which cuts diagonally back and forth across
/// the line that connects the two points.
///
/// The line will cross the line `zigs - 1` times.
///
/// If `zigs` is 1, then this will draw two sides of a triangle from `start` to
/// `end`, with the third point being `width` away from the line, as measured
/// perpendicular to that line.
///
/// If `width` is positive, the first `zig` will be to the left of the `start`
/// point when facing the `end` point. To reverse the zigging polarity, provide
/// a negative `width`.
///
/// The line is drawn using the provided `paint` on the provided `canvas`.
void paintZigZag(Canvas canvas, Paint paint, Offset start, Offset end, int zigs, double width) {
  assert(zigs.isFinite);
  assert(zigs > 0);
  canvas.save();
  canvas.translate(start.dx, start.dy);
  end = end - start;
  canvas.rotate(math.atan2(end.dy, end.dx));
  final double length = end.distance;
  final double spacing = length / (zigs * 2.0);
  final Path path = Path()..moveTo(0.0, 0.0);
  for (int index = 0; index < zigs; index += 1) {
    final double x = (index * 2.0 + 1.0) * spacing;
    final double y = width * ((index % 2.0) * 2.0 - 1.0);
    path.lineTo(x, y);
  }
  path.lineTo(length, 0.0);
  canvas.drawPath(path, paint);
  canvas.restore();
}
