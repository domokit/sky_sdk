// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "path_ops.h"

namespace flutter {
SkPath* CreatePath(SkPathFillType fill_type) {
  auto* path = new SkPath();
  path->setFillType(fill_type);
  return path;
}

void DestroyPath(SkPath* path) {
  delete path;
}

void MoveTo(SkPath* path, SkScalar x, SkScalar y) {
  path->moveTo(x, y);
}

void LineTo(SkPath* path, SkScalar x, SkScalar y) {
  path->lineTo(x, y);
}

void CubicTo(SkPath* path,
             SkScalar x1,
             SkScalar y1,
             SkScalar x2,
             SkScalar y2,
             SkScalar x3,
             SkScalar y3) {
  path->cubicTo(x1, y1, x2, y2, x3, y3);
}

void Close(SkPath* path) {
  path->close();
}

void Reset(SkPath* path) {
  path->reset();
}

void Op(SkPath* one, SkPath* two, SkPathOp op) {
  Op(*one, *two, op, one);
}

int GetFillType(SkPath* path) {
  return static_cast<int>(path->getFillType());
}

struct PathData* Data(SkPath* path) {
  int point_count = path->countPoints();
  int verb_count = path->countVerbs();

  auto data = new PathData();
  data->points = new float[point_count * 2];
  data->point_count = point_count * 2;
  data->verbs = new uint8_t[verb_count];
  data->verb_count = verb_count;

  path->getVerbs(data->verbs, verb_count);
  path->getPoints(reinterpret_cast<SkPoint*>(data->points), point_count);
  return data;
}

void DestroyData(PathData* data) {
  delete[] data->points;
  delete[] data->verbs;
  delete data;
}

}  // namespace flutter
