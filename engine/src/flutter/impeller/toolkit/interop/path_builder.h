// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_PATH_BUILDER_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_PATH_BUILDER_H_

#include "flutter/third_party/skia/include/core/SkPath.h"
#include "impeller/geometry/path.h"
#include "impeller/geometry/path_builder.h"
#include "impeller/toolkit/interop/impeller.h"
#include "impeller/toolkit/interop/object.h"
#include "impeller/toolkit/interop/path.h"

namespace impeller::interop {

class PathBuilder final
    : public Object<PathBuilder,
                    IMPELLER_INTERNAL_HANDLE_NAME(ImpellerPathBuilder)> {
 public:
  PathBuilder();

  ~PathBuilder();

  PathBuilder(const PathBuilder&) = delete;

  PathBuilder& operator=(const PathBuilder&) = delete;

  void MoveTo(const Point& point);

  void LineTo(const Point& location);

  void QuadraticCurveTo(const Point& control_point, const Point& end_point);

  void CubicCurveTo(const Point& control_point_1,
                    const Point& control_point_2,
                    const Point& end_point);

  void AddRect(const Rect& rect);

  void AddArc(const Rect& oval_bounds, Degrees start_angle, Degrees end_angle);

  void AddOval(const Rect& oval_bounds);

  void AddRoundedRect(const Rect& rect, const RoundingRadii& radii);

  void Close();

  ScopedObject<Path> TakePath(FillType fill);

  ScopedObject<Path> CopyPath(FillType fill);

 private:
  SkPath builder_;
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_PATH_BUILDER_H_
