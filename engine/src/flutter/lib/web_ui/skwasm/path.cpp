// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "export.h"
#include "helpers.h"
#include "third_party/skia/include/core/SkPath.h"
#include "third_party/skia/include/core/SkString.h"
#include "third_party/skia/include/pathops/SkPathOps.h"
#include "third_party/skia/include/utils/SkParsePath.h"

using namespace Skwasm;

SKWASM_EXPORT SkPath* path_create() {
  return new SkPath();
}

SKWASM_EXPORT void path_dispose(SkPath* path) {
  delete path;
}

SKWASM_EXPORT SkPath* path_copy(SkPath* path) {
  return new SkPath(*path);
}

SKWASM_EXPORT void path_setFillType(SkPath* path, SkPathFillType fillType) {
  path->setFillType(fillType);
}

SKWASM_EXPORT SkPathFillType path_getFillType(SkPath* path) {
  return path->getFillType();
}

SKWASM_EXPORT void path_moveTo(SkPath* path, SkScalar x, SkScalar y) {
  path->moveTo(x, y);
}

SKWASM_EXPORT void path_relativeMoveTo(SkPath* path, SkScalar x, SkScalar y) {
  path->rMoveTo(x, y);
}

SKWASM_EXPORT void path_lineTo(SkPath* path, SkScalar x, SkScalar y) {
  path->lineTo(x, y);
}

SKWASM_EXPORT void path_relativeLineTo(SkPath* path, SkScalar x, SkScalar y) {
  path->rLineTo(x, y);
}

SKWASM_EXPORT void path_quadraticBezierTo(SkPath* path,
                                          SkScalar x1,
                                          SkScalar y1,
                                          SkScalar x2,
                                          SkScalar y2) {
  path->quadTo(x1, y1, x2, y2);
}

SKWASM_EXPORT void path_relativeQuadraticBezierTo(SkPath* path,
                                                  SkScalar x1,
                                                  SkScalar y1,
                                                  SkScalar x2,
                                                  SkScalar y2) {
  path->rQuadTo(x1, y1, x2, y2);
}

SKWASM_EXPORT void path_cubicTo(SkPath* path,
                                SkScalar x1,
                                SkScalar y1,
                                SkScalar x2,
                                SkScalar y2,
                                SkScalar x3,
                                SkScalar y3) {
  path->cubicTo(x1, y1, x2, y2, x3, y3);
}

SKWASM_EXPORT void path_relativeCubicTo(SkPath* path,
                                        SkScalar x1,
                                        SkScalar y1,
                                        SkScalar x2,
                                        SkScalar y2,
                                        SkScalar x3,
                                        SkScalar y3) {
  path->rCubicTo(x1, y1, x2, y2, x3, y3);
}

SKWASM_EXPORT void path_conicTo(SkPath* path,
                                SkScalar x1,
                                SkScalar y1,
                                SkScalar x2,
                                SkScalar y2,
                                SkScalar w) {
  path->conicTo(x1, y1, x2, y2, w);
}

SKWASM_EXPORT void path_relativeConicTo(SkPath* path,
                                        SkScalar x1,
                                        SkScalar y1,
                                        SkScalar x2,
                                        SkScalar y2,
                                        SkScalar w) {
  path->rConicTo(x1, y1, x2, y2, w);
}

SKWASM_EXPORT void path_arcToOval(SkPath* path,
                                  const SkRect* rect,
                                  SkScalar startAngle,
                                  SkScalar sweepAngle,
                                  bool forceMoveTo) {
  path->arcTo(*rect, startAngle, sweepAngle, forceMoveTo);
}

SKWASM_EXPORT void path_arcToRotated(SkPath* path,
                                     SkScalar rx,
                                     SkScalar ry,
                                     SkScalar xAxisRotate,
                                     SkPath::ArcSize arcSize,
                                     SkPathDirection pathDirection,
                                     SkScalar x,
                                     SkScalar y) {
  path->arcTo(rx, ry, xAxisRotate, arcSize, pathDirection, x, y);
}

SKWASM_EXPORT void path_relativeArcToRotated(SkPath* path,
                                             SkScalar rx,
                                             SkScalar ry,
                                             SkScalar xAxisRotate,
                                             SkPath::ArcSize arcSize,
                                             SkPathDirection pathDirection,
                                             SkScalar x,
                                             SkScalar y) {
  path->rArcTo(rx, ry, xAxisRotate, arcSize, pathDirection, x, y);
}

SKWASM_EXPORT void path_addRect(SkPath* path, const SkRect* rect) {
  path->addRect(*rect);
}

SKWASM_EXPORT void path_addOval(SkPath* path, const SkRect* oval) {
  path->addOval(*oval, SkPathDirection::kCW, 1);
}

SKWASM_EXPORT void path_addArc(SkPath* path,
                               const SkRect* oval,
                               SkScalar startAngle,
                               SkScalar sweepAngle) {
  path->addArc(*oval, startAngle, sweepAngle);
}

SKWASM_EXPORT void path_addPolygon(SkPath* path,
                                   const SkPoint* points,
                                   int count,
                                   bool close) {
  path->addPoly(points, count, close);
}

SKWASM_EXPORT void path_addRRect(SkPath* path, const SkScalar* rrectValues) {
  path->addRRect(createRRect(rrectValues), SkPathDirection::kCW);
}

SKWASM_EXPORT void path_addPath(SkPath* path,
                                const SkPath* other,
                                const SkScalar* matrix33,
                                SkPath::AddPathMode extendPath) {
  path->addPath(*other, createMatrix(matrix33), extendPath);
}

SKWASM_EXPORT void path_close(SkPath* path) {
  path->close();
}

SKWASM_EXPORT void path_reset(SkPath* path) {
  path->reset();
}

SKWASM_EXPORT bool path_contains(SkPath* path, SkScalar x, SkScalar y) {
  return path->contains(x, y);
}

SKWASM_EXPORT void path_transform(SkPath* path, const SkScalar* matrix33) {
  path->transform(createMatrix(matrix33));
}

SKWASM_EXPORT void path_getBounds(SkPath* path, SkRect* rect) {
  *rect = path->getBounds();
}

SKWASM_EXPORT SkPath* path_combine(SkPathOp operation,
                                   const SkPath* path1,
                                   const SkPath* path2) {
  SkPath* output = new SkPath();
  if (Op(*path1, *path2, operation, output)) {
    output->setFillType(path1->getFillType());
    return output;
  } else {
    delete output;
    return nullptr;
  }
}

SKWASM_EXPORT SkString* path_getSvgString(SkPath* path) {
  SkString* string = new SkString(SkParsePath::ToSVGString(*path));
  return string;
}
