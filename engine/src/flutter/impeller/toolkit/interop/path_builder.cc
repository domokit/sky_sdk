// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/interop/path_builder.h"

#include "impeller/toolkit/interop/formats.h"

namespace impeller::interop {

PathBuilder::PathBuilder() = default;

PathBuilder::~PathBuilder() = default;

void PathBuilder::MoveTo(const Point& point) {
  builder_.moveTo(ToSkiaType(point));
}

void PathBuilder::LineTo(const Point& location) {
  builder_.lineTo(ToSkiaType(location));
}

void PathBuilder::QuadraticCurveTo(const Point& control_point,
                                   const Point& end_point) {
  builder_.quadTo(ToSkiaType(control_point), ToSkiaType(end_point));
}

void PathBuilder::CubicCurveTo(const Point& control_point_1,
                               const Point& control_point_2,
                               const Point& end_point) {
  builder_.cubicTo(ToSkiaType(control_point_1),  //
                   ToSkiaType(control_point_2),  //
                   ToSkiaType(end_point)         //
  );
}

void PathBuilder::AddRect(const Rect& rect) {
  builder_.addRect(ToSkiaType(rect));
}

void PathBuilder::AddArc(const Rect& oval_bounds,
                         Degrees start_angle,
                         Degrees end_angle) {
  builder_.addArc(ToSkiaType(oval_bounds),                 //
                  start_angle.degrees,                     //
                  end_angle.degrees - start_angle.degrees  // sweep
  );
}

void PathBuilder::AddOval(const Rect& oval_bounds) {
  builder_.addOval(ToSkiaType(oval_bounds));
}

void PathBuilder::AddRoundedRect(const Rect& rect, const RoundingRadii& radii) {
  builder_.addRRect(ToSkiaType(rect, radii));
}

void PathBuilder::Close() {
  builder_.close();
}

ScopedObject<Path> PathBuilder::TakePath(FillType fill) {
  builder_.setFillType(ToSkiaType(fill));
  return Create<Path>(std::move(builder_));
}

ScopedObject<Path> PathBuilder::CopyPath(FillType fill) {
  builder_.setFillType(ToSkiaType(fill));
  return Create<Path>(builder_);
}

}  // namespace impeller::interop
