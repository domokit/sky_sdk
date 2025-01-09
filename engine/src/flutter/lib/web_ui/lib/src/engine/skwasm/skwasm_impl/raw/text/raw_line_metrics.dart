// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@DefaultAsset('skwasm')
library skwasm_impl;

import 'dart:ffi';

final class RawLineMetrics extends Opaque {}

typedef LineMetricsHandle = Pointer<RawLineMetrics>;

@Native<
  LineMetricsHandle Function(Bool, Double, Double, Double, Double, Double, Double, Double, Size)
>(symbol: 'lineMetrics_create', isLeaf: true)
external LineMetricsHandle lineMetricsCreate(
  bool hardBreak,
  double ascent,
  double descent,
  double unscaledAscent,
  double height,
  double width,
  double left,
  double baseline,
  int lineNumber,
);

@Native<Void Function(LineMetricsHandle)>(symbol: 'lineMetrics_dispose', isLeaf: true)
external void lineMetricsDispose(LineMetricsHandle handle);

@Native<Bool Function(LineMetricsHandle)>(symbol: 'lineMetrics_getHardBreak', isLeaf: true)
external bool lineMetricsGetHardBreak(LineMetricsHandle handle);

@Native<Float Function(LineMetricsHandle)>(symbol: 'lineMetrics_getAscent', isLeaf: true)
external double lineMetricsGetAscent(LineMetricsHandle handle);

@Native<Float Function(LineMetricsHandle)>(symbol: 'lineMetrics_getDescent', isLeaf: true)
external double lineMetricsGetDescent(LineMetricsHandle handle);

@Native<Float Function(LineMetricsHandle)>(symbol: 'lineMetrics_getUnscaledAscent', isLeaf: true)
external double lineMetricsGetUnscaledAscent(LineMetricsHandle handle);

@Native<Float Function(LineMetricsHandle)>(symbol: 'lineMetrics_getHeight', isLeaf: true)
external double lineMetricsGetHeight(LineMetricsHandle handle);

@Native<Float Function(LineMetricsHandle)>(symbol: 'lineMetrics_getWidth', isLeaf: true)
external double lineMetricsGetWidth(LineMetricsHandle handle);

@Native<Float Function(LineMetricsHandle)>(symbol: 'lineMetrics_getLeft', isLeaf: true)
external double lineMetricsGetLeft(LineMetricsHandle handle);

@Native<Float Function(LineMetricsHandle)>(symbol: 'lineMetrics_getBaseline', isLeaf: true)
external double lineMetricsGetBaseline(LineMetricsHandle handle);

@Native<Int Function(LineMetricsHandle)>(symbol: 'lineMetrics_getLineNumber', isLeaf: true)
external int lineMetricsGetLineNumber(LineMetricsHandle handle);

@Native<Size Function(LineMetricsHandle)>(symbol: 'lineMetrics_getStartIndex', isLeaf: true)
external int lineMetricsGetStartIndex(LineMetricsHandle handle);

@Native<Size Function(LineMetricsHandle)>(symbol: 'lineMetrics_getEndIndex', isLeaf: true)
external int lineMetricsGetEndIndex(LineMetricsHandle handle);
