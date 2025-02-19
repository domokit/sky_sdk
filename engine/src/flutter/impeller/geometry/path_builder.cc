// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "path_builder.h"

#include <array>
#include <cmath>

#include "impeller/geometry/path_component.h"
#include "impeller/geometry/round_superellipse_param.h"

namespace impeller {

namespace {

// Utility functions used to build a rounded superellipse.
class RoundSuperellipseBuilder {
 public:
  typedef std::function<
      void(const Point&, const Point&, const Point&, const Point&)>
      CubicAdder;

  // Create a builder.
  //
  // The resulting curves, which consists of cubic curves, are added by calling
  // `cubic_adder`.
  explicit RoundSuperellipseBuilder(CubicAdder cubic_adder)
      : cubic_adder_(std::move(cubic_adder)) {}

  // Draws an arc representing 1/4 of a rounded superellipse.
  //
  // If `reverse` is false, the resulting arc spans from 0 to pi/2, moving
  // clockwise starting from the positive Y-axis. Otherwise it moves from pi/2
  // to 0.
  void AddQuadrant(const RoundSuperellipseParam::Quadrant& param,
                   bool reverse) {
    auto transform =
        Matrix::MakeTranslateScale(param.signed_scale, param.offset);
    if (!reverse) {
      AddOctant(param.top, /*reverse=*/false, /*flip=*/false, transform);
      AddOctant(param.right, /*reverse=*/true, /*flip=*/true, transform);
    } else {
      AddOctant(param.right, /*reverse=*/false, /*flip=*/true, transform);
      AddOctant(param.top, /*reverse=*/true, /*flip=*/false, transform);
    }
  }

 private:
  std::array<Point, 4> SuperellipseArcPoints(
      const RoundSuperellipseParam::Octant& param) {
    Point start = {param.se_center.x, param.edge_mid.y};
    const Point& end = param.circle_start;
    constexpr Point start_tangent = {1, 0};
    Point circle_start_vector = param.circle_start - param.circle_center;
    Point end_tangent =
        Point{-circle_start_vector.y, circle_start_vector.x}.Normalize();

    std::array<Scalar, 2> factors = SuperellipseBezierFactors(param.se_n);

    return std::array<Point, 4>{
        start, start + start_tangent * factors[0] * param.se_a,
        end + end_tangent * factors[1] * param.se_a, end};
  };

  std::array<Point, 4> CircularArcPoints(
      const RoundSuperellipseParam::Octant& param) {
    Point start_vector = param.circle_start - param.circle_center;
    Point end_vector =
        start_vector.Rotate(Radians(-param.circle_max_angle.radians));
    Point circle_end = param.circle_center + end_vector;
    Point start_tangent = Point{start_vector.y, -start_vector.x}.Normalize();
    Point end_tangent = Point{-end_vector.y, end_vector.x}.Normalize();
    Scalar bezier_factor = std::tan(param.circle_max_angle.radians / 4) * 4 / 3;
    Scalar radius = start_vector.GetLength();

    return std::array<Point, 4>{
        param.circle_start,
        param.circle_start + start_tangent * bezier_factor * radius,
        circle_end + end_tangent * bezier_factor * radius, circle_end};
  };

  // Draws an arc representing 1/8 of a rounded superellipse.
  //
  // If `reverse` is false, the resulting arc spans from 0 to pi/4, moving
  // clockwise starting from the positive Y-axis. Otherwise it moves from pi/4
  // to 0.
  //
  // If `flip` is true, all points have their X and Y coordinates swapped,
  // effectively mirrowing each point by the y=x line.
  //
  // All points are transformed by `external_transform` after the optional
  // flipping before being used as control points for the cubic curves.
  void AddOctant(const RoundSuperellipseParam::Octant& param,
                 bool reverse,
                 bool flip,
                 const Matrix& external_transform) {
    Matrix transform =
        external_transform * Matrix::MakeTranslation(param.offset);
    if (flip) {
      transform = transform * kFlip;
    }

    auto circle_points = CircularArcPoints(param);
    auto se_points = SuperellipseArcPoints(param);

    if (!reverse) {
      cubic_adder_(transform * se_points[0], transform * se_points[1],
                   transform * se_points[2], transform * se_points[3]);
      cubic_adder_(transform * circle_points[0], transform * circle_points[1],
                   transform * circle_points[2], transform * circle_points[3]);
    } else {
      cubic_adder_(transform * circle_points[3], transform * circle_points[2],
                   transform * circle_points[1], transform * circle_points[0]);
      cubic_adder_(transform * se_points[3], transform * se_points[2],
                   transform * se_points[1], transform * se_points[0]);
    }
  };

  // Get the Bezier factor for the superellipse arc in a rounded superellipse.
  //
  // The result will be assigned to output, where [0] will be the factor for the
  // starting tangent and [1] for the ending tangent.
  //
  // These values are computed by brute-force searching for the minimal distance
  // on a rounded superellipse and are not for general purpose superellipses.
  std::array<Scalar, 2> SuperellipseBezierFactors(Scalar n) {
    constexpr Scalar kPrecomputedVariables[][2] = {
        /*n=2.000*/ {0.02927797, 0.05200645},
        /*n=2.050*/ {0.02927797, 0.05200645},
        /*n=2.100*/ {0.03288032, 0.06051731},
        /*n=2.150*/ {0.03719241, 0.06818433},
        /*n=2.200*/ {0.04009513, 0.07196947},
        /*n=2.250*/ {0.04504750, 0.07860258},
        /*n=2.300*/ {0.05038706, 0.08498836},
        /*n=2.350*/ {0.05580771, 0.09071105},
        /*n=2.400*/ {0.06002306, 0.09363976},
        /*n=2.450*/ {0.06630048, 0.09946086},
        /*n=2.500*/ {0.07200351, 0.10384857}};
    constexpr Scalar kNStepInverse = 20;  // = 1 / 0.05
    constexpr size_t kNumRecords =
        sizeof(kPrecomputedVariables) / sizeof(kPrecomputedVariables[0]);
    constexpr Scalar kMinN = 2.00f;

    Scalar steps =
        std::clamp<Scalar>((n - kMinN) * kNStepInverse, 0, kNumRecords - 1);
    size_t left = std::clamp<size_t>(static_cast<size_t>(std::floor(steps)), 0,
                                     kNumRecords - 2);
    Scalar frac = steps - left;

    return std::array<Scalar, 2>{(1 - frac) * kPrecomputedVariables[left][0] +
                                     frac * kPrecomputedVariables[left + 1][0],
                                 (1 - frac) * kPrecomputedVariables[left][1] +
                                     frac * kPrecomputedVariables[left + 1][1]};
  }

  CubicAdder cubic_adder_;

  // A matrix that swaps the coordinates of a point.
  // clang-format off
  static constexpr Matrix kFlip = Matrix(
    0.0f, 1.0f, 0.0f, 0.0f,
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f);
  // clang-format on
};

}  // namespace

PathBuilder::PathBuilder() {
  AddContourComponent({});
}

PathBuilder::~PathBuilder() = default;

Path PathBuilder::CopyPath(FillType fill) {
  prototype_.fill = fill;
  prototype_.single_contour =
      current_contour_location_ == 0u ||
      (contour_count_ == 2 &&
       prototype_.components.back() == Path::ComponentType::kContour);
  return Path(prototype_);
}

Path PathBuilder::TakePath(FillType fill) {
  prototype_.fill = fill;
  UpdateBounds();
  prototype_.single_contour =
      current_contour_location_ == 0u ||
      (contour_count_ == 2 &&
       prototype_.components.back() == Path::ComponentType::kContour);
  current_contour_location_ = 0u;
  contour_count_ = 1;
  return Path(std::move(prototype_));
}

void PathBuilder::Reserve(size_t point_size, size_t verb_size) {
  prototype_.points.reserve(point_size);
  prototype_.components.reserve(verb_size);
}

PathBuilder& PathBuilder::MoveTo(Point point, bool relative) {
  current_ = relative ? current_ + point : point;
  subpath_start_ = current_;
  AddContourComponent(current_);
  return *this;
}

PathBuilder& PathBuilder::Close() {
  // If the subpath start is the same as the current position, this
  // is an empty contour and inserting a line segment will just
  // confuse the tessellator.
  if (subpath_start_ != current_) {
    LineTo(subpath_start_);
  }
  SetContourClosed(true);
  AddContourComponent(current_);
  return *this;
}

PathBuilder& PathBuilder::LineTo(Point point, bool relative) {
  point = relative ? current_ + point : point;
  AddLinearComponent(current_, point);
  current_ = point;
  return *this;
}

PathBuilder& PathBuilder::HorizontalLineTo(Scalar x, bool relative) {
  Point endpoint =
      relative ? Point{current_.x + x, current_.y} : Point{x, current_.y};
  AddLinearComponent(current_, endpoint);
  current_ = endpoint;
  return *this;
}

PathBuilder& PathBuilder::VerticalLineTo(Scalar y, bool relative) {
  Point endpoint =
      relative ? Point{current_.x, current_.y + y} : Point{current_.x, y};
  AddLinearComponent(current_, endpoint);
  current_ = endpoint;
  return *this;
}

PathBuilder& PathBuilder::QuadraticCurveTo(Point controlPoint,
                                           Point point,
                                           bool relative) {
  point = relative ? current_ + point : point;
  controlPoint = relative ? current_ + controlPoint : controlPoint;
  AddQuadraticComponent(current_, controlPoint, point);
  current_ = point;
  return *this;
}

PathBuilder& PathBuilder::ConicCurveTo(Point controlPoint,
                                       Point point,
                                       Scalar weight,
                                       bool relative) {
  point = relative ? current_ + point : point;
  controlPoint = relative ? current_ + controlPoint : controlPoint;
  AddConicComponent(current_, controlPoint, point, weight);
  current_ = point;
  return *this;
}

PathBuilder& PathBuilder::SetConvexity(Convexity value) {
  prototype_.convexity = value;
  return *this;
}

PathBuilder& PathBuilder::CubicCurveTo(Point controlPoint1,
                                       Point controlPoint2,
                                       Point point,
                                       bool relative) {
  controlPoint1 = relative ? current_ + controlPoint1 : controlPoint1;
  controlPoint2 = relative ? current_ + controlPoint2 : controlPoint2;
  point = relative ? current_ + point : point;
  AddCubicComponent(current_, controlPoint1, controlPoint2, point);
  current_ = point;
  return *this;
}

PathBuilder& PathBuilder::AddQuadraticCurve(const Point& p1,
                                            const Point& cp,
                                            const Point& p2) {
  MoveTo(p1);
  AddQuadraticComponent(p1, cp, p2);
  return *this;
}

PathBuilder& PathBuilder::AddConicCurve(const Point& p1,
                                        const Point& cp,
                                        const Point& p2,
                                        Scalar weight) {
  MoveTo(p1);
  AddConicComponent(p1, cp, p2, weight);
  return *this;
}

PathBuilder& PathBuilder::AddCubicCurve(const Point& p1,
                                        const Point& cp1,
                                        const Point& cp2,
                                        const Point& p2) {
  MoveTo(p1);
  AddCubicComponent(p1, cp1, cp2, p2);
  return *this;
}

PathBuilder& PathBuilder::AddRect(const Rect& rect) {
  auto origin = rect.GetOrigin();
  auto size = rect.GetSize();

  auto tl = origin;
  auto bl = origin + Point{0.0, size.height};
  auto br = origin + size;
  auto tr = origin + Point{size.width, 0.0};

  MoveTo(tl);
  LineTo(tr);
  LineTo(br);
  LineTo(bl);
  Close();

  return *this;
}

PathBuilder& PathBuilder::AddCircle(const Point& c, Scalar r) {
  return AddOval(Rect::MakeXYWH(c.x - r, c.y - r, 2.0f * r, 2.0f * r));
}

PathBuilder& PathBuilder::AddRoundRect(RoundRect round_rect) {
  auto rect = round_rect.GetBounds();
  auto radii = round_rect.GetRadii();
  if (radii.AreAllCornersEmpty()) {
    return AddRect(rect);
  }

  auto rect_origin = rect.GetOrigin();
  auto rect_size = rect.GetSize();

  current_ = rect_origin + Point{radii.top_left.width, 0.0};

  MoveTo({rect_origin.x + radii.top_left.width, rect_origin.y});

  //----------------------------------------------------------------------------
  // Top line.
  //
  AddLinearComponentIfNeeded(
      {rect_origin.x + radii.top_left.width, rect_origin.y},
      {rect_origin.x + rect_size.width - radii.top_right.width, rect_origin.y});

  //----------------------------------------------------------------------------
  // Top right arc.
  //
  AddRoundedRectTopRight(rect, radii);

  //----------------------------------------------------------------------------
  // Right line.
  //
  AddLinearComponentIfNeeded(
      {rect_origin.x + rect_size.width, rect_origin.y + radii.top_right.height},
      {rect_origin.x + rect_size.width,
       rect_origin.y + rect_size.height - radii.bottom_right.height});

  //----------------------------------------------------------------------------
  // Bottom right arc.
  //
  AddRoundedRectBottomRight(rect, radii);

  //----------------------------------------------------------------------------
  // Bottom line.
  //
  AddLinearComponentIfNeeded(
      {rect_origin.x + rect_size.width - radii.bottom_right.width,
       rect_origin.y + rect_size.height},
      {rect_origin.x + radii.bottom_left.width,
       rect_origin.y + rect_size.height});

  //----------------------------------------------------------------------------
  // Bottom left arc.
  //
  AddRoundedRectBottomLeft(rect, radii);

  //----------------------------------------------------------------------------
  // Left line.
  //
  AddLinearComponentIfNeeded(
      {rect_origin.x,
       rect_origin.y + rect_size.height - radii.bottom_left.height},
      {rect_origin.x, rect_origin.y + radii.top_left.height});

  //----------------------------------------------------------------------------
  // Top left arc.
  //
  AddRoundedRectTopLeft(rect, radii);

  Close();

  return *this;
}

PathBuilder& PathBuilder::AddRoundSuperellipse(RoundSuperellipse rse) {
  if (rse.IsRect()) {
    return AddRect(rse.GetBounds());
  }

  RoundSuperellipseBuilder builder(
      [this](const Point& a, const Point& b, const Point& c, const Point& d) {
        AddCubicComponent(a, b, c, d);
      });

  auto param =
      RoundSuperellipseParam::MakeBoundsRadii(rse.GetBounds(), rse.GetRadii());
  Point start = param.top_right.offset +
                param.top_right.signed_scale *
                    (param.top_right.top.offset + param.top_right.top.edge_mid);
  MoveTo(start);

  if (param.all_corners_same) {
    auto* quadrant = &param.top_right;
    builder.AddQuadrant(*quadrant, /*reverse=*/false);
    quadrant->signed_scale.y *= -1;
    builder.AddQuadrant(*quadrant, /*reverse=*/true);
    quadrant->signed_scale.x *= -1;
    builder.AddQuadrant(*quadrant, /*reverse=*/false);
    quadrant->signed_scale.y *= -1;
    builder.AddQuadrant(*quadrant, /*reverse=*/true);
  } else {
    builder.AddQuadrant(param.top_right, /*reverse=*/false);
    builder.AddQuadrant(param.bottom_right, /*reverse=*/true);
    builder.AddQuadrant(param.bottom_left, /*reverse=*/false);
    builder.AddQuadrant(param.top_left, /*reverse=*/true);
  }

  LineTo(start);

  Close();

  return *this;
}

PathBuilder& PathBuilder::AddRoundedRectTopLeft(Rect rect,
                                                RoundingRadii radii) {
  const auto magic_top_left = radii.top_left * kArcApproximationMagic;
  const auto corner = rect.GetOrigin();
  AddCubicComponent(
      {corner.x, corner.y + radii.top_left.height},
      {corner.x, corner.y + radii.top_left.height - magic_top_left.height},
      {corner.x + radii.top_left.width - magic_top_left.width, corner.y},
      {corner.x + radii.top_left.width, corner.y});
  return *this;
}

PathBuilder& PathBuilder::AddRoundedRectTopRight(Rect rect,
                                                 RoundingRadii radii) {
  const auto magic_top_right = radii.top_right * kArcApproximationMagic;
  const auto corner = rect.GetOrigin() + Point{rect.GetWidth(), 0};
  AddCubicComponent(
      {corner.x - radii.top_right.width, corner.y},
      {corner.x - radii.top_right.width + magic_top_right.width, corner.y},
      {corner.x, corner.y + radii.top_right.height - magic_top_right.height},
      {corner.x, corner.y + radii.top_right.height});
  return *this;
}

PathBuilder& PathBuilder::AddRoundedRectBottomRight(Rect rect,
                                                    RoundingRadii radii) {
  const auto magic_bottom_right = radii.bottom_right * kArcApproximationMagic;
  const auto corner = rect.GetOrigin() + rect.GetSize();
  AddCubicComponent(
      {corner.x, corner.y - radii.bottom_right.height},
      {corner.x,
       corner.y - radii.bottom_right.height + magic_bottom_right.height},
      {corner.x - radii.bottom_right.width + magic_bottom_right.width,
       corner.y},
      {corner.x - radii.bottom_right.width, corner.y});
  return *this;
}

PathBuilder& PathBuilder::AddRoundedRectBottomLeft(Rect rect,
                                                   RoundingRadii radii) {
  const auto magic_bottom_left = radii.bottom_left * kArcApproximationMagic;
  const auto corner = rect.GetOrigin() + Point{0, rect.GetHeight()};
  AddCubicComponent(
      {corner.x + radii.bottom_left.width, corner.y},
      {corner.x + radii.bottom_left.width - magic_bottom_left.width, corner.y},
      {corner.x,
       corner.y - radii.bottom_left.height + magic_bottom_left.height},
      {corner.x, corner.y - radii.bottom_left.height});
  return *this;
}

void PathBuilder::AddContourComponent(const Point& destination,
                                      bool is_closed) {
  auto& components = prototype_.components;
  auto& points = prototype_.points;
  auto closed = is_closed ? Point{0, 0} : Point{1, 1};
  if (components.size() > 0 &&
      components.back() == Path::ComponentType::kContour) {
    // Never insert contiguous contours.
    points[current_contour_location_] = destination;
    points[current_contour_location_ + 1] = closed;
  } else {
    current_contour_location_ = points.size();
    points.push_back(destination);
    points.push_back(closed);
    components.push_back(Path::ComponentType::kContour);
    contour_count_ += 1;
  }
  prototype_.bounds.reset();
}

void PathBuilder::AddLinearComponentIfNeeded(const Point& p1, const Point& p2) {
  if (ScalarNearlyEqual(p1.x, p2.x, 1e-4f) &&
      ScalarNearlyEqual(p1.y, p2.y, 1e-4f)) {
    return;
  }
  AddLinearComponent(p1, p2);
}

void PathBuilder::AddLinearComponent(const Point& p1, const Point& p2) {
  auto& points = prototype_.points;
  points.push_back(p1);
  points.push_back(p2);
  prototype_.components.push_back(Path::ComponentType::kLinear);
  prototype_.bounds.reset();
}

void PathBuilder::AddQuadraticComponent(const Point& p1,
                                        const Point& cp,
                                        const Point& p2) {
  auto& points = prototype_.points;
  points.push_back(p1);
  points.push_back(cp);
  points.push_back(p2);
  prototype_.components.push_back(Path::ComponentType::kQuadratic);
  prototype_.bounds.reset();
}

void PathBuilder::AddConicComponent(const Point& p1,
                                    const Point& cp,
                                    const Point& p2,
                                    Scalar weight) {
  if (!std::isfinite(weight)) {
    AddLinearComponent(p1, cp);
    AddLinearComponent(cp, p2);
  } else if (weight <= 0) {
    AddLinearComponent(p1, p2);
  } else if (weight == 1) {
    AddQuadraticComponent(p1, cp, p2);
  } else {
    auto& points = prototype_.points;
    points.push_back(p1);
    points.push_back(cp);
    points.push_back(p2);
    points.emplace_back(weight, weight);
    prototype_.components.push_back(Path::ComponentType::kConic);
    prototype_.bounds.reset();
  }
}

void PathBuilder::AddCubicComponent(const Point& p1,
                                    const Point& cp1,
                                    const Point& cp2,
                                    const Point& p2) {
  auto& points = prototype_.points;
  points.push_back(p1);
  points.push_back(cp1);
  points.push_back(cp2);
  points.push_back(p2);
  prototype_.components.push_back(Path::ComponentType::kCubic);
  prototype_.bounds.reset();
}

void PathBuilder::SetContourClosed(bool is_closed) {
  prototype_.points[current_contour_location_ + 1] =
      is_closed ? Point{0, 0} : Point{1, 1};
}

PathBuilder& PathBuilder::AddArc(const Rect& oval_bounds,
                                 Radians start,
                                 Radians sweep,
                                 bool use_center) {
  if (sweep.radians < 0) {
    start.radians += sweep.radians;
    sweep.radians *= -1;
  }
  sweep.radians = std::min(k2Pi, sweep.radians);
  start.radians = std::fmod(start.radians, k2Pi);

  const Point center = oval_bounds.GetCenter();
  const Point radius = center - oval_bounds.GetOrigin();

  Vector2 p1_unit(std::cos(start.radians), std::sin(start.radians));

  if (use_center) {
    MoveTo(center);
    LineTo(center + p1_unit * radius);
  } else {
    MoveTo(center + p1_unit * radius);
  }

  while (sweep.radians > 0) {
    Vector2 p2_unit;
    Scalar quadrant_angle;
    if (sweep.radians < kPiOver2) {
      quadrant_angle = sweep.radians;
      p2_unit = Vector2(std::cos(start.radians + quadrant_angle),
                        std::sin(start.radians + quadrant_angle));
    } else {
      quadrant_angle = kPiOver2;
      p2_unit = Vector2(-p1_unit.y, p1_unit.x);
    }

    Vector2 arc_cp_lengths =
        (quadrant_angle / kPiOver2) * kArcApproximationMagic * radius;

    Point p1 = center + p1_unit * radius;
    Point p2 = center + p2_unit * radius;
    Point cp1 = p1 + Vector2(-p1_unit.y, p1_unit.x) * arc_cp_lengths;
    Point cp2 = p2 + Vector2(p2_unit.y, -p2_unit.x) * arc_cp_lengths;

    AddCubicComponent(p1, cp1, cp2, p2);
    current_ = p2;

    start.radians += quadrant_angle;
    sweep.radians -= quadrant_angle;
    p1_unit = p2_unit;
  }

  if (use_center) {
    Close();
  }

  return *this;
}

PathBuilder& PathBuilder::AddOval(const Rect& container) {
  const Point c = container.GetCenter();
  const Point r = c - container.GetOrigin();
  const Point m = r * kArcApproximationMagic;

  MoveTo({c.x, c.y - r.y});

  //----------------------------------------------------------------------------
  // Top right arc.
  //
  AddCubicComponent({c.x, c.y - r.y},        // p1
                    {c.x + m.x, c.y - r.y},  // cp1
                    {c.x + r.x, c.y - m.y},  // cp2
                    {c.x + r.x, c.y}         // p2
  );

  //----------------------------------------------------------------------------
  // Bottom right arc.
  //
  AddCubicComponent({c.x + r.x, c.y},        // p1
                    {c.x + r.x, c.y + m.y},  // cp1
                    {c.x + m.x, c.y + r.y},  // cp2
                    {c.x, c.y + r.y}         // p2
  );

  //----------------------------------------------------------------------------
  // Bottom left arc.
  //
  AddCubicComponent({c.x, c.y + r.y},        // p1
                    {c.x - m.x, c.y + r.y},  // cp1
                    {c.x - r.x, c.y + m.y},  // cp2
                    {c.x - r.x, c.y}         // p2
  );

  //----------------------------------------------------------------------------
  // Top left arc.
  //
  AddCubicComponent({c.x - r.x, c.y},        // p1
                    {c.x - r.x, c.y - m.y},  // cp1
                    {c.x - m.x, c.y - r.y},  // cp2
                    {c.x, c.y - r.y}         // p2
  );

  Close();

  return *this;
}

PathBuilder& PathBuilder::AddLine(const Point& p1, const Point& p2) {
  MoveTo(p1);
  AddLinearComponent(p1, p2);
  return *this;
}

PathBuilder& PathBuilder::AddPath(const Path& path) {
  auto& points = prototype_.points;
  auto& components = prototype_.components;
  size_t source_offset = points.size();

  points.insert(points.end(), path.data_->points.begin(),
                path.data_->points.end());
  components.insert(components.end(), path.data_->components.begin(),
                    path.data_->components.end());

  for (auto component : path.data_->components) {
    if (component == Path::ComponentType::kContour) {
      current_contour_location_ = source_offset;
      contour_count_ += 1;
    }
    source_offset += Path::VerbToOffset(component);
  }
  return *this;
}

PathBuilder& PathBuilder::Shift(Point offset) {
  auto& points = prototype_.points;
  size_t storage_offset = 0u;
  for (const auto& component : prototype_.components) {
    switch (component) {
      case Path::ComponentType::kLinear: {
        auto* linear =
            reinterpret_cast<LinearPathComponent*>(&points[storage_offset]);
        linear->p1 += offset;
        linear->p2 += offset;
        break;
      }
      case Path::ComponentType::kQuadratic: {
        auto* quad =
            reinterpret_cast<QuadraticPathComponent*>(&points[storage_offset]);
        quad->p1 += offset;
        quad->p2 += offset;
        quad->cp += offset;
      } break;
      case Path::ComponentType::kConic: {
        auto* conic =
            reinterpret_cast<ConicPathComponent*>(&points[storage_offset]);
        conic->p1 += offset;
        conic->p2 += offset;
        conic->cp += offset;
      } break;
      case Path::ComponentType::kCubic: {
        auto* cubic =
            reinterpret_cast<CubicPathComponent*>(&points[storage_offset]);
        cubic->p1 += offset;
        cubic->p2 += offset;
        cubic->cp1 += offset;
        cubic->cp2 += offset;
      } break;
      case Path::ComponentType::kContour:
        auto* contour =
            reinterpret_cast<ContourComponent*>(&points[storage_offset]);
        contour->destination += offset;
        break;
    }
    storage_offset += Path::VerbToOffset(component);
  }

  prototype_.bounds.reset();
  return *this;
}

PathBuilder& PathBuilder::SetBounds(Rect bounds) {
  prototype_.bounds = bounds;
  return *this;
}

void PathBuilder::UpdateBounds() {
  if (!prototype_.bounds.has_value()) {
    auto min_max = GetMinMaxCoveragePoints();
    if (!min_max.has_value()) {
      prototype_.bounds.reset();
      return;
    }
    auto min = min_max->first;
    auto max = min_max->second;
    const auto difference = max - min;
    prototype_.bounds =
        Rect::MakeXYWH(min.x, min.y, difference.x, difference.y);
  }
}

std::optional<std::pair<Point, Point>> PathBuilder::GetMinMaxCoveragePoints()
    const {
  auto& points = prototype_.points;

  if (points.empty()) {
    return std::nullopt;
  }

  std::optional<Point> min, max;

  auto clamp = [&min, &max](const Point& point) {
    if (min.has_value()) {
      min = min->Min(point);
    } else {
      min = point;
    }

    if (max.has_value()) {
      max = max->Max(point);
    } else {
      max = point;
    }
  };

  size_t storage_offset = 0u;
  for (const auto& component : prototype_.components) {
    switch (component) {
      case Path::ComponentType::kLinear: {
        auto* linear = reinterpret_cast<const LinearPathComponent*>(
            &points[storage_offset]);
        clamp(linear->p1);
        clamp(linear->p2);
        break;
      }
      case Path::ComponentType::kQuadratic:
        for (const auto& extrema :
             reinterpret_cast<const QuadraticPathComponent*>(
                 &points[storage_offset])
                 ->Extrema()) {
          clamp(extrema);
        }
        break;
      case Path::ComponentType::kConic:
        for (const auto& extrema : reinterpret_cast<const ConicPathComponent*>(
                                       &points[storage_offset])
                                       ->Extrema()) {
          clamp(extrema);
        }
        break;
      case Path::ComponentType::kCubic:
        for (const auto& extrema : reinterpret_cast<const CubicPathComponent*>(
                                       &points[storage_offset])
                                       ->Extrema()) {
          clamp(extrema);
        }
        break;
      case Path::ComponentType::kContour:
        break;
    }
    storage_offset += Path::VerbToOffset(component);
  }

  if (!min.has_value() || !max.has_value()) {
    return std::nullopt;
  }

  return std::make_pair(min.value(), max.value());
}

}  // namespace impeller
