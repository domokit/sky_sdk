// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/rendering.dart';

import '../common.dart';

const int _kNumIterations = 10000000;
const int _kNumWarmUp = 100000;

Future<void> execute() async {
  assert(false, "Don't run benchmarks in debug mode! Use 'flutter run --release'.");
  print('MatrixUtils.transformRect and .transformPoint benchmark...');

  Matrix4 makePerspective(double radius, double angle, double perspective) {
    return MatrixUtils.createCylindricalProjectionTransform(
      radius: radius,
      angle: angle,
      perspective: perspective,
    );
  }

  final List<Matrix4> affineTransforms = <Matrix4>[
    Matrix4.identity()
      ..scale(1.2, 1.3, 1.0)
      ..rotateZ(0.1),
    Matrix4.identity()..translate(12.0, 13.0, 10.0),
    Matrix4.identity()
      ..scale(1.2, 1.3, 1.0)
      ..translate(12.0, 13.0, 10.0),
  ];
  final List<Matrix4> perspectiveTransforms = <Matrix4>[
    makePerspective(10.0, math.pi / 8.0, 0.3),
    makePerspective(8.0, math.pi / 8.0, 0.2),
    makePerspective(1.0, math.pi / 4.0, 0.1)..rotateX(0.1),
  ];
  final List<Rect> rectangles = <Rect>[
    const Rect.fromLTRB(1.1, 1.2, 1.5, 1.8),
    const Rect.fromLTRB(1.1, 1.2, 0.0, 1.0),
    const Rect.fromLTRB(1.1, 1.2, 1.3, 1.0),
    const Rect.fromLTRB(-1.1, -1.2, 0.0, 1.0),
    const Rect.fromLTRB(-1.1, -1.2, -1.5, -1.8),
  ];
  final List<Offset> offsets = <Offset>[
    const Offset(1.1, 1.2),
    const Offset(1.5, 1.8),
    Offset.zero,
    const Offset(-1.1, -1.2),
    const Offset(-1.5, -1.8),
  ];
  final int nAffine = affineTransforms.length;
  final int nPerspective = perspectiveTransforms.length;
  final int nRectangles = rectangles.length;
  final int nOffsets = offsets.length;

  // Warm up lap
  for (int i = 0; i < _kNumWarmUp; i += 1) {
    final Matrix4 transform = perspectiveTransforms[i % nPerspective];
    final Rect rect = rectangles[(i ~/ nPerspective) % nRectangles];
    final Offset offset = offsets[(i ~/ nPerspective) % nOffsets];
    MatrixUtils.transformRect(transform, rect);
    MatrixUtils.transformPoint(transform, offset);
  }
  for (int i = 0; i < _kNumWarmUp; i += 1) {
    final Matrix4 transform = affineTransforms[i % nAffine];
    final Rect rect = rectangles[(i ~/ nAffine) % nRectangles];
    final Offset offset = offsets[(i ~/ nAffine) % nOffsets];
    MatrixUtils.transformRect(transform, rect);
    MatrixUtils.transformPoint(transform, offset);
  }

  final Stopwatch watch = Stopwatch();
  watch.start();
  for (int i = 0; i < _kNumIterations; i += 1) {
    final Matrix4 transform = perspectiveTransforms[i % nPerspective];
    final Rect rect = rectangles[(i ~/ nPerspective) % nRectangles];
    MatrixUtils.transformRect(transform, rect);
  }
  watch.stop();
  final int rectMicrosecondsPerspective = watch.elapsedMicroseconds;

  watch.reset();
  watch.start();
  for (int i = 0; i < _kNumIterations; i += 1) {
    final Matrix4 transform = affineTransforms[i % nAffine];
    final Rect rect = rectangles[(i ~/ nAffine) % nRectangles];
    MatrixUtils.transformRect(transform, rect);
  }
  watch.stop();
  final int rectMicrosecondsAffine = watch.elapsedMicroseconds;

  watch.reset();
  watch.start();
  for (int i = 0; i < _kNumIterations; i += 1) {
    final Matrix4 transform = perspectiveTransforms[i % nPerspective];
    final Offset offset = offsets[(i ~/ nPerspective) % nOffsets];
    MatrixUtils.transformPoint(transform, offset);
  }
  watch.stop();
  final int pointMicrosecondsPerspective = watch.elapsedMicroseconds;

  watch.reset();
  watch.start();
  for (int i = 0; i < _kNumIterations; i += 1) {
    final Matrix4 transform = affineTransforms[i % nAffine];
    final Offset offset = offsets[(i ~/ nAffine) % nOffsets];
    MatrixUtils.transformPoint(transform, offset);
  }
  watch.stop();
  final int pointMicrosecondsAffine = watch.elapsedMicroseconds;

  final BenchmarkResultPrinter printer = BenchmarkResultPrinter();
  const double scale = 1000.0 / _kNumIterations;
  printer.addResult(
    description: 'MatrixUtils.transformRectPerspective',
    value: rectMicrosecondsPerspective * scale,
    unit: 'ns per iteration',
    name: 'MatrixUtils_persp_transformRect_iteration',
  );
  printer.addResult(
    description: 'MatrixUtils.transformRectAffine',
    value: rectMicrosecondsAffine * scale,
    unit: 'ns per iteration',
    name: 'MatrixUtils_affine_transformRect_iteration',
  );
  printer.addResult(
    description: 'MatrixUtils.transformPointPerspective',
    value: pointMicrosecondsPerspective * scale,
    unit: 'ns per iteration',
    name: 'MatrixUtils_persp_transformPoint_iteration',
  );
  printer.addResult(
    description: 'MatrixUtils.transformPointAffine',
    value: pointMicrosecondsAffine * scale,
    unit: 'ns per iteration',
    name: 'MatrixUtils_affine_transformPoint_iteration',
  );
  printer.printToStdout();
}

//
//  Note that the benchmark is normally run by benchmark_collection.dart.
//
Future<void> main() async {
  return execute();
}
