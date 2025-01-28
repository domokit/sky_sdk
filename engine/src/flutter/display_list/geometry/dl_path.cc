// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/geometry/dl_path.h"

#include "flutter/display_list/geometry/dl_geometry_types.h"
#include "flutter/impeller/geometry/path_builder.h"
#include "impeller/geometry/path.h"

namespace flutter {

using Path = impeller::Path;
using PathBuilder = impeller::PathBuilder;
using FillType = impeller::FillType;
using Convexity = impeller::Convexity;
using ComponentType = impeller::Path::ComponentType;

DlPath DlPath::MakeRect(const DlRect& rect) {
  return DlPath(SkPath::Rect(ToSkRect(rect)));
}

DlPath DlPath::MakeRectLTRB(DlScalar left,
                            DlScalar top,
                            DlScalar right,
                            DlScalar bottom) {
  return DlPath(SkPath().addRect(left, top, right, bottom));
}

DlPath DlPath::MakeRectXYWH(DlScalar x,
                            DlScalar y,
                            DlScalar width,
                            DlScalar height) {
  return DlPath(SkPath().addRect(SkRect::MakeXYWH(x, y, width, height)));
}

DlPath DlPath::MakeOval(const DlRect& bounds) {
  return DlPath(SkPath::Oval(ToSkRect(bounds)));
}

DlPath DlPath::MakeOvalLTRB(DlScalar left,
                            DlScalar top,
                            DlScalar right,
                            DlScalar bottom) {
  return DlPath(SkPath::Oval(SkRect::MakeLTRB(left, top, right, bottom)));
}

DlPath DlPath::MakeCircle(const DlPoint& center, DlScalar radius) {
  return DlPath(SkPath::Circle(center.x, center.y, radius));
}

DlPath DlPath::MakeRoundRect(const DlRoundRect& rrect) {
  return DlPath(SkPath::RRect(ToSkRRect(rrect)));
}

DlPath DlPath::MakeRoundRectXY(const DlRect& rect,
                               DlScalar x_radius,
                               DlScalar y_radius,
                               bool counter_clock_wise) {
  return DlPath(SkPath::RRect(
      ToSkRect(rect), x_radius, y_radius,
      counter_clock_wise ? SkPathDirection::kCCW : SkPathDirection::kCW));
}

DlPath DlPath::MakeLine(const DlPoint& a, const DlPoint& b) {
  return DlPath(SkPath::Line(ToSkPoint(a), ToSkPoint(b)));
}

DlPath DlPath::MakePoly(const DlPoint pts[],
                        int count,
                        bool close,
                        DlPathFillType fill_type) {
  return DlPath(
      SkPath::Polygon(ToSkPoints(pts), count, close, ToSkFillType(fill_type)));
}

DlPath DlPath::MakeArc(const DlRect& bounds,
                       DlDegrees start,
                       DlDegrees end,
                       bool use_center) {
  SkPath path;
  if (use_center) {
    path.moveTo(ToSkPoint(bounds.GetCenter()));
  }
  path.arcTo(ToSkRect(bounds), start.degrees, end.degrees, !use_center);
  if (use_center) {
    path.close();
  }
  return DlPath(path);
}

const SkPath& DlPath::GetSkPath() const {
  auto& sk_path = data_->sk_path;
  auto& path = data_->path;
  if (sk_path.has_value()) {
    return sk_path.value();
  }
  if (path.has_value()) {
    sk_path.emplace(ConvertToSkiaPath(path.value()));
    if (data_->render_count >= kMaxVolatileUses) {
      sk_path.value().setIsVolatile(false);
    }
    return sk_path.value();
  }
  sk_path.emplace();
  return sk_path.value();
}

Path DlPath::GetPath() const {
  auto& sk_path = data_->sk_path;
  auto& path = data_->path;
  if (path.has_value()) {
    return path.value();
  }
  if (sk_path.has_value()) {
    path.emplace(ConvertToImpellerPath(sk_path.value()));
    return path.value();
  }
  path.emplace();
  return path.value();
}

void DlPath::WillRenderSkPath() const {
  if (data_->render_count >= kMaxVolatileUses) {
    auto& sk_path = data_->sk_path;
    if (sk_path.has_value()) {
      sk_path.value().setIsVolatile(false);
    }
  } else {
    data_->render_count++;
  }
}

[[nodiscard]] DlPath DlPath::WithOffset(const DlPoint& offset) const {
  if (offset.IsZero()) {
    return *this;
  }
  if (!offset.IsFinite()) {
    return DlPath();
  }
  auto& path = data_->path;
  if (path.has_value()) {
    PathBuilder builder;
    builder.AddPath(path.value());
    builder.Shift(offset);
    return DlPath(builder.TakePath());
  }
  auto& sk_path = data_->sk_path;
  if (sk_path.has_value()) {
    SkPath path = sk_path.value();
    path = path.offset(offset.x, offset.y);
    return DlPath(path);
  }
  return *this;
}

[[nodiscard]] DlPath DlPath::WithFillType(DlPathFillType type) const {
  auto& path = data_->path;
  if (path.has_value()) {
    if (path.value().GetFillType() == type) {
      return *this;
    }
    PathBuilder builder;
    builder.AddPath(path.value());
    return DlPath(builder.TakePath(type));
  }
  auto& sk_path = data_->sk_path;
  if (sk_path.has_value()) {
    SkPathFillType sk_type = ToSkFillType(type);
    if (sk_path.value().getFillType() == sk_type) {
      return *this;
    }
    SkPath path = sk_path.value();
    path.setFillType(sk_type);
    return DlPath(path);
  }
  return *this;
}

bool DlPath::IsRect(DlRect* rect, bool* is_closed) const {
  return GetSkPath().isRect(ToSkRect(rect), is_closed);
}

bool DlPath::IsOval(DlRect* bounds) const {
  return GetSkPath().isOval(ToSkRect(bounds));
}

bool DlPath::IsRoundRect(DlRoundRect* rrect) const {
  SkRRect sk_rrect;
  bool ret = GetSkPath().isRRect(rrect ? &sk_rrect : nullptr);
  if (rrect) {
    *rrect = ToDlRoundRect(sk_rrect);
  }
  return ret;
}

bool DlPath::IsSkRect(SkRect* rect, bool* is_closed) const {
  return GetSkPath().isRect(rect, is_closed);
}

bool DlPath::IsSkOval(SkRect* bounds) const {
  return GetSkPath().isOval(bounds);
}

bool DlPath::IsSkRRect(SkRRect* rrect) const {
  return GetSkPath().isRRect(rrect);
}

bool DlPath::Contains(const DlPoint& point) const {
  return GetSkPath().contains(point.x, point.y);
}

SkRect DlPath::GetSkBounds() const {
  return GetSkPath().getBounds();
}

DlRect DlPath::GetBounds() const {
  auto& path = data_->path;
  if (path.has_value()) {
    return path.value().GetBoundingBox().value_or(DlRect());
  }
  return ToDlRect(GetSkPath().getBounds());
}

bool DlPath::operator==(const DlPath& other) const {
  return GetSkPath() == other.GetSkPath();
}

bool DlPath::IsConverted() const {
  return data_->path.has_value() && data_->sk_path.has_value();
}

bool DlPath::IsVolatile() const {
  return GetSkPath().isVolatile();
}

bool DlPath::IsConvex() const {
  if (data_->sk_path_original) {
    auto& sk_path = data_->sk_path;
    FML_DCHECK(sk_path.has_value());
    return sk_path.has_value() && sk_path->isConvex();
  } else {
    auto& path = data_->path;
    FML_DCHECK(path.has_value());
    return path.has_value() && path->IsConvex();
  }
}

DlPath DlPath::operator+(const DlPath& other) const {
  SkPath path = GetSkPath();
  path.addPath(other.GetSkPath());
  return DlPath(path);
}

SkPath DlPath::ConvertToSkiaPath(const Path& path, const DlPoint& shift) {
  SkPath sk_path;
  sk_path.setFillType(ToSkFillType(path.GetFillType()));
  bool subpath_needs_close = false;
  std::optional<DlPoint> pending_moveto;

  auto resolve_moveto = [&pending_moveto, &sk_path]() {
    if (pending_moveto.has_value()) {
      sk_path.moveTo(ToSkPoint(pending_moveto.value()));
      pending_moveto.reset();
    }
  };

  size_t count = path.GetComponentCount();
  for (size_t i = 0; i < count; i++) {
    switch (path.GetComponentTypeAtIndex(i)) {
      case ComponentType::kContour: {
        impeller::ContourComponent contour;
        path.GetContourComponentAtIndex(i, contour);
        if (subpath_needs_close) {
          sk_path.close();
        }
        pending_moveto = contour.destination;
        subpath_needs_close = contour.IsClosed();
        break;
      }
      case ComponentType::kLinear: {
        impeller::LinearPathComponent linear;
        path.GetLinearComponentAtIndex(i, linear);
        resolve_moveto();
        sk_path.lineTo(ToSkPoint(linear.p2));
        break;
      }
      case ComponentType::kQuadratic: {
        impeller::QuadraticPathComponent quadratic;
        path.GetQuadraticComponentAtIndex(i, quadratic);
        resolve_moveto();
        sk_path.quadTo(ToSkPoint(quadratic.cp), ToSkPoint(quadratic.p2));
        break;
      }
      case ComponentType::kCubic: {
        impeller::CubicPathComponent cubic;
        path.GetCubicComponentAtIndex(i, cubic);
        resolve_moveto();
        sk_path.cubicTo(ToSkPoint(cubic.cp1), ToSkPoint(cubic.cp2),
                        ToSkPoint(cubic.p2));
        break;
      }
    }
  }
  if (subpath_needs_close) {
    sk_path.close();
  }

  return sk_path;
}

Path DlPath::ConvertToImpellerPath(const SkPath& path, const DlPoint& shift) {
  if (path.isEmpty() || !shift.IsFinite()) {
    return Path{};
  }
  auto iterator = SkPath::Iter(path, false);

  struct PathData {
    union {
      SkPoint points[4];
    };
  };

  PathBuilder builder;
  PathData data;
  // Reserve a path size with some arbitrarily additional padding.
  builder.Reserve(path.countPoints() + 8, path.countVerbs() + 8);
  auto verb = SkPath::Verb::kDone_Verb;
  do {
    verb = iterator.next(data.points);
    switch (verb) {
      case SkPath::kMove_Verb:
        builder.MoveTo(ToDlPoint(data.points[0]));
        break;
      case SkPath::kLine_Verb:
        builder.LineTo(ToDlPoint(data.points[1]));
        break;
      case SkPath::kQuad_Verb:
        builder.QuadraticCurveTo(ToDlPoint(data.points[1]),
                                 ToDlPoint(data.points[2]));
        break;
      case SkPath::kConic_Verb: {
        constexpr auto kPow2 = 1;  // Only works for sweeps up to 90 degrees.
        constexpr auto kQuadCount = 1 + (2 * (1 << kPow2));
        SkPoint points[kQuadCount];
        const auto curve_count =
            SkPath::ConvertConicToQuads(data.points[0],          //
                                        data.points[1],          //
                                        data.points[2],          //
                                        iterator.conicWeight(),  //
                                        points,                  //
                                        kPow2                    //
            );

        for (int curve_index = 0, point_index = 0;  //
             curve_index < curve_count;             //
             curve_index++, point_index += 2        //
        ) {
          builder.QuadraticCurveTo(ToDlPoint(points[point_index + 1]),
                                   ToDlPoint(points[point_index + 2]));
        }
      } break;
      case SkPath::kCubic_Verb:
        builder.CubicCurveTo(ToDlPoint(data.points[1]),
                             ToDlPoint(data.points[2]),
                             ToDlPoint(data.points[3]));
        break;
      case SkPath::kClose_Verb:
        builder.Close();
        break;
      case SkPath::kDone_Verb:
        break;
    }
  } while (verb != SkPath::Verb::kDone_Verb);

  DlRect bounds = ToDlRect(path.getBounds());
  if (!shift.IsZero()) {
    builder.Shift(shift);
    bounds = bounds.Shift(shift);
  }

  builder.SetConvexity(path.isConvex() ? Convexity::kConvex
                                       : Convexity::kUnknown);
  builder.SetBounds(bounds);
  return builder.TakePath(ToDlFillType(path.getFillType()));
}

}  // namespace flutter
