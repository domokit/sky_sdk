// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "export.h"
#include "helpers.h"

#include "third_party/skia/include/core/SkContourMeasure.h"
#include "third_party/skia/include/core/SkPath.h"

using namespace Skwasm;

SKWASM_EXPORT SkContourMeasureIter*
contourMeasureIter_create(SkPath* path, bool forceClosed, SkScalar resScale) {
  return new SkContourMeasureIter(*path, forceClosed, resScale);
}

SKWASM_EXPORT SkContourMeasure* contourMeasureIter_next(
    SkContourMeasureIter* iter) {
  auto next = iter->next();
  if (next) {
    next->ref();
  }
  return next.get();
}

SKWASM_EXPORT void contourMeasureIter_dispose(SkContourMeasureIter* iter) {
  delete iter;
}

SKWASM_EXPORT void contourMeasure_dispose(SkContourMeasure* measure) {
  measure->unref();
}

SKWASM_EXPORT SkScalar contourMeasure_length(SkContourMeasure* measure) {
  return measure->length();
}

SKWASM_EXPORT bool contourMeasure_isClosed(SkContourMeasure* measure) {
  return measure->isClosed();
}

SKWASM_EXPORT bool contourMeasure_getPosTan(SkContourMeasure* measure,
                                            SkScalar distance,
                                            SkPoint* outPosition,
                                            SkVector* outTangent) {
  return measure->getPosTan(distance, outPosition, outTangent);
}

SKWASM_EXPORT SkPath* contourMeasure_getSegment(SkContourMeasure* measure,
                                                SkScalar startD,
                                                SkScalar stopD,
                                                bool startWithMoveTo) {
  SkPath* outPath = new SkPath();
  if (!measure->getSegment(startD, stopD, outPath, startWithMoveTo)) {
    delete outPath;
    return nullptr;
  }
  return outPath;
}
