// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/geometry/dl_path.h"

#include "flutter/display_list/geometry/dl_geometry_types.h"
#include "flutter/impeller/geometry/path_builder.h"
#include "impeller/geometry/path.h"

namespace flutter {

DlPath DlPath::MakeRect(DlRect rect) {
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

DlPath DlPath::MakeOval(DlRect bounds) {
  return DlPath(SkPath::Oval(ToSkRect(bounds)));
}

DlPath DlPath::MakeOvalLTRB(DlScalar left,
                            DlScalar top,
                            DlScalar right,
                            DlScalar bottom) {
  return DlPath(SkPath::Oval(SkRect::MakeLTRB(left, top, right, bottom)));
}

const SkPath& DlPath::GetSkPath() const {
  return data_->sk_path;
}

impeller::Path DlPath::GetPath() const {
  if (!data_->path.has_value()) {
    data_->path = ConvertToImpellerPath(data_->sk_path);
  }

  // Covered by check above.
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  return data_->path.value();
}

void DlPath::WillRenderSkPath() const {
  if (data_->render_count >= kMaxVolatileUses) {
    data_->sk_path.setIsVolatile(false);
  } else {
    data_->render_count++;
  }
}

bool DlPath::IsInverseFillType() const {
  return data_->sk_path.isInverseFillType();
}

bool DlPath::IsRect(DlRect* rect, bool* is_closed) const {
  return data_->sk_path.isRect(ToSkRect(rect), is_closed);
}

bool DlPath::IsOval(DlRect* bounds) const {
  return data_->sk_path.isOval(ToSkRect(bounds));
}

bool DlPath::IsSkRect(SkRect* rect, bool* is_closed) const {
  return data_->sk_path.isRect(rect, is_closed);
}

bool DlPath::IsSkOval(SkRect* bounds) const {
  return data_->sk_path.isOval(bounds);
}

bool DlPath::IsSkRRect(SkRRect* rrect) const {
  return data_->sk_path.isRRect(rrect);
}

SkRect DlPath::GetSkBounds() const {
  return data_->sk_path.getBounds();
}

DlRect DlPath::GetBounds() const {
  return ToDlRect(data_->sk_path.getBounds());
}

bool DlPath::operator==(const DlPath& other) const {
  return data_->sk_path == other.data_->sk_path;
}

bool DlPath::IsConverted() const {
  return data_->path.has_value();
}

bool DlPath::IsVolatile() const {
  return data_->sk_path.isVolatile();
}

DlPath DlPath::operator+(const DlPath& other) const {
  SkPath path = data_->sk_path;
  path.addPath(other.data_->sk_path);
  return DlPath(path);
}

using Path = impeller::Path;
using PathBuilder = impeller::PathBuilder;
using FillType = impeller::FillType;
using Convexity = impeller::Convexity;

Path DlPath::ConvertToImpellerPath(const SkPath& path, const DlPoint& shift) {
  if (path.isEmpty()) {
    return impeller::Path{};
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

  FillType fill_type;
  switch (path.getFillType()) {
    case SkPathFillType::kWinding:
      fill_type = FillType::kNonZero;
      break;
    case SkPathFillType::kEvenOdd:
      fill_type = FillType::kOdd;
      break;
    case SkPathFillType::kInverseWinding:
    case SkPathFillType::kInverseEvenOdd:
      // Flutter doesn't expose these path fill types. These are only visible
      // via the receiver interface. We should never get here.
      fill_type = FillType::kNonZero;
      break;
  }
  builder.SetConvexity(path.isConvex() ? Convexity::kConvex
                                       : Convexity::kUnknown);
  builder.Shift(shift);
  auto sk_bounds = path.getBounds().makeOutset(shift.x, shift.y);
  builder.SetBounds(ToDlRect(sk_bounds));
  return builder.TakePath(fill_type);
}

}  // namespace flutter
