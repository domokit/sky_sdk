// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@DefaultAsset('skwasm')
library skwasm_impl;

import 'dart:ffi';

import 'package:ui/src/engine/skwasm/skwasm_impl.dart';

final class RawContourMeasure extends Opaque {}

final class RawContourMeasureIter extends Opaque {}

typedef ContourMeasureHandle = Pointer<RawContourMeasure>;
typedef ContourMeasureIterHandle = Pointer<RawContourMeasureIter>;

@Native<ContourMeasureIterHandle Function(PathHandle, Bool, Float)>(
  symbol: 'contourMeasureIter_create',
)
external ContourMeasureIterHandle contourMeasureIterCreate(
  PathHandle path,
  bool forceClosed,
  double resScale,
);

@Native<ContourMeasureHandle Function(ContourMeasureIterHandle)>(symbol: 'contourMeasureIter_next')
external ContourMeasureHandle contourMeasureIterNext(ContourMeasureIterHandle handle);

@Native<Void Function(ContourMeasureIterHandle)>(symbol: 'contourMeasureIter_dispose')
external void contourMeasureIterDispose(ContourMeasureIterHandle handle);

@Native<Void Function(ContourMeasureHandle)>(symbol: 'contourMeasure_dispose')
external void contourMeasureDispose(ContourMeasureHandle handle);

@Native<Float Function(ContourMeasureHandle)>(symbol: 'contourMeasure_length')
external double contourMeasureLength(ContourMeasureHandle handle);

@Native<Bool Function(ContourMeasureHandle)>(symbol: 'contourMeasure_isClosed')
external bool contourMeasureIsClosed(ContourMeasureHandle handle);

@Native<Bool Function(ContourMeasureHandle, Float, RawPointArray, RawPointArray)>(
  symbol: 'contourMeasure_getPosTan',
)
external bool contourMeasureGetPosTan(
  ContourMeasureHandle handle,
  double distance,
  RawPointArray outPosition,
  RawPointArray outTangent,
);

@Native<PathHandle Function(ContourMeasureHandle, Float, Float, Bool)>(
  symbol: 'contourMeasure_getSegment',
)
external PathHandle contourMeasureGetSegment(
  ContourMeasureHandle handle,
  double start,
  double stop,
  bool startWithMoveTo,
);
