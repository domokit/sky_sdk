// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/tessellator/tessellator.h"
#include <cstdint>
#include <cstring>

#include "impeller/core/device_buffer.h"
#include "impeller/geometry/path_component.h"

namespace impeller {

Tessellator::Tessellator()
    : point_buffer_(std::make_unique<std::vector<Point>>()),
      index_buffer_(std::make_unique<std::vector<uint16_t>>()),
      stroke_points_(kPointArenaSize) {
  point_buffer_->reserve(2048);
  index_buffer_->reserve(2048);
}

Tessellator::~Tessellator() = default;

std::vector<Point>& Tessellator::GetStrokePointCache() {
  return stroke_points_;
}

Path::Polyline Tessellator::CreateTempPolyline(const Path& path,
                                               Scalar tolerance) {
  FML_DCHECK(point_buffer_);
  point_buffer_->clear();
  auto polyline =
      path.CreatePolyline(tolerance, std::move(point_buffer_),
                          [this](Path::Polyline::PointBufferPtr point_buffer) {
                            point_buffer_ = std::move(point_buffer);
                          });
  return polyline;
}

VertexBuffer Tessellator::TessellateConvex(const Path& path,
                                           HostBuffer& host_buffer,
                                           Scalar tolerance,
                                           bool supports_primitive_restart,
                                           bool supports_triangle_fan) {
  if (supports_primitive_restart) {
    // Primitive Restart.
    const auto [point_count, contour_count] = path.CountStorage(tolerance);
    BufferView point_buffer = host_buffer.Emplace(
        nullptr, sizeof(Point) * point_count, alignof(Point));
    BufferView index_buffer = host_buffer.Emplace(
        nullptr, sizeof(uint16_t) * (point_count + contour_count),
        alignof(uint16_t));

    if (supports_triangle_fan) {
      FanVertexWriter writer(
          reinterpret_cast<Point*>(point_buffer.GetBuffer()->OnGetContents() +
                                   point_buffer.GetRange().offset),
          reinterpret_cast<uint16_t*>(
              index_buffer.GetBuffer()->OnGetContents() +
              index_buffer.GetRange().offset));
      path.WritePolyline(tolerance, writer);
      point_buffer.GetBuffer()->Flush(point_buffer.GetRange());
      index_buffer.GetBuffer()->Flush(index_buffer.GetRange());

      return VertexBuffer{
          .vertex_buffer = std::move(point_buffer),
          .index_buffer = std::move(index_buffer),
          .vertex_count = writer.GetIndexCount(),
          .index_type = IndexType::k16bit,
      };
    } else {
      StripVertexWriter writer(
          reinterpret_cast<Point*>(point_buffer.GetBuffer()->OnGetContents() +
                                   point_buffer.GetRange().offset),
          reinterpret_cast<uint16_t*>(
              index_buffer.GetBuffer()->OnGetContents() +
              index_buffer.GetRange().offset));
      path.WritePolyline(tolerance, writer);
      point_buffer.GetBuffer()->Flush(point_buffer.GetRange());
      index_buffer.GetBuffer()->Flush(index_buffer.GetRange());

      return VertexBuffer{
          .vertex_buffer = std::move(point_buffer),
          .index_buffer = std::move(index_buffer),
          .vertex_count = writer.GetIndexCount(),
          .index_type = IndexType::k16bit,
      };
    }
  }

  FML_DCHECK(point_buffer_);
  FML_DCHECK(index_buffer_);
  TessellateConvexInternal(path, *point_buffer_, *index_buffer_, tolerance);

  if (point_buffer_->empty()) {
    return VertexBuffer{
        .vertex_buffer = {},
        .index_buffer = {},
        .vertex_count = 0u,
        .index_type = IndexType::k16bit,
    };
  }

  BufferView vertex_buffer = host_buffer.Emplace(
      point_buffer_->data(), sizeof(Point) * point_buffer_->size(),
      alignof(Point));

  BufferView index_buffer = host_buffer.Emplace(
      index_buffer_->data(), sizeof(uint16_t) * index_buffer_->size(),
      alignof(uint16_t));

  return VertexBuffer{
      .vertex_buffer = std::move(vertex_buffer),
      .index_buffer = std::move(index_buffer),
      .vertex_count = index_buffer_->size(),
      .index_type = IndexType::k16bit,
  };
}

VertexBuffer Tessellator::GenerateLineStrip(const Path& path,
                                            HostBuffer& host_buffer,
                                            Scalar tolerance) {
  LineStripVertexWriter writer(stroke_points_);
  path.WritePolyline(tolerance, writer);

  const auto [arena_length, oversized_length] = writer.GetVertexCount();

  if (oversized_length == 0) {
    return VertexBuffer{
        .vertex_buffer =
            host_buffer.Emplace(stroke_points_.data(),
                                arena_length * sizeof(Point), alignof(Point)),
        .index_buffer = {},
        .vertex_count = arena_length,
        .index_type = IndexType::kNone,
    };
  }
  const std::vector<Point>& oversized_data = writer.GetOversizedBuffer();
  BufferView buffer_view = host_buffer.Emplace(
      /*buffer=*/nullptr,                                 //
      (arena_length + oversized_length) * sizeof(Point),  //
      alignof(Point)                                      //
  );
  memcpy(buffer_view.GetBuffer()->OnGetContents() +
             buffer_view.GetRange().offset,  //
         stroke_points_.data(),              //
         arena_length * sizeof(Point)        //
  );
  memcpy(buffer_view.GetBuffer()->OnGetContents() +
             buffer_view.GetRange().offset + arena_length * sizeof(Point),  //
         oversized_data.data(),                                             //
         oversized_data.size() * sizeof(Point)                              //
  );
  buffer_view.GetBuffer()->Flush(buffer_view.GetRange());

  return VertexBuffer{
      .vertex_buffer = buffer_view,
      .index_buffer = {},
      .vertex_count = arena_length + oversized_length,
      .index_type = IndexType::kNone,
  };
}

void Tessellator::TessellateConvexInternal(const Path& path,
                                           std::vector<Point>& point_buffer,
                                           std::vector<uint16_t>& index_buffer,
                                           Scalar tolerance) {
  point_buffer.clear();
  index_buffer.clear();

  GLESVertexWriter writer(point_buffer, index_buffer);

  path.WritePolyline(tolerance, writer);
}

static constexpr int kPrecomputedDivisionCount = 1024;
static int kPrecomputedDivisions[kPrecomputedDivisionCount] = {
    // clang-format off
     1,  2,  3,  4,  4,  4,  5,  5,  5,  6,  6,  6,  7,  7,  7,  7,
     8,  8,  8,  8,  8,  9,  9,  9,  9,  9,  9, 10, 10, 10, 10, 10,
    10, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 13,
    13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14,
    15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16,
    16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 18, 18,
    18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19, 19,
    19, 19, 19, 19, 19, 19, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20,
    20, 20, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21,
    22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 23, 23, 23,
    23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 24, 24, 24, 24,
    24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25,
    25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26,
    26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 27, 27, 27, 27,
    27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 28, 28, 28,
    28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 29,
    29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29,
    29, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30,
    30, 30, 30, 30, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31,
    31, 31, 31, 31, 31, 31, 31, 31, 32, 32, 32, 32, 32, 32, 32, 32,
    32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 33, 33, 33,
    33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33,
    33, 33, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34,
    34, 34, 34, 34, 34, 34, 34, 35, 35, 35, 35, 35, 35, 35, 35, 35,
    35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 36, 36,
    36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36,
    36, 36, 36, 36, 36, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37,
    37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 38, 38, 38, 38,
    38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38,
    38, 38, 38, 38, 38, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39,
    39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 41, 41, 41, 41, 41, 41, 41, 41, 41,
    41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41,
    41, 41, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42,
    42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 43, 43, 43, 43,
    43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43,
    43, 43, 43, 43, 43, 43, 43, 43, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 44, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45,
    45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45,
    45, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46,
    46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 47,
    47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47,
    47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 48, 48, 48,
    48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
    48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 49, 49, 49, 49,
    49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49,
    49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 50, 50, 50, 50, 50,
    50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50,
    50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 51, 51, 51, 51, 51,
    51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51,
    51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 52, 52, 52, 52,
    52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52,
    52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 53, 53, 53,
    53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53,
    53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 54,
    54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54,
    54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54,
    54, 54, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55,
    55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55,
    55, 55, 55, 55, 55, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56,
    56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56,
    56, 56, 56, 56, 56, 56, 56, 56, 56, 57, 57, 57, 57, 57, 57, 57,
    // clang-format on
};

static size_t ComputeQuadrantDivisions(Scalar pixel_radius) {
  if (pixel_radius <= 0.0) {
    return 1;
  }
  int radius_index = ceil(pixel_radius);
  if (radius_index < kPrecomputedDivisionCount) {
    return kPrecomputedDivisions[radius_index];
  }

  // For a circle with N divisions per quadrant, the maximum deviation of
  // the polgyon approximation from the true circle will be at the center
  // of the base of each triangular pie slice. We can compute that distance
  // by finding the midpoint of the line of the first slice and compare
  // its distance from the center of the circle to the radius. We will aim
  // to have the length of that bisector to be within |kCircleTolerance|
  // from the radius in pixels.
  //
  // Each vertex will appear at an angle of:
  //   theta(i) = (kPi / 2) * (i / N)  // for i in [0..N]
  // with each point falling at:
  //   point(i) = r * (cos(theta), sin(theta))
  // If we consider the unit circle to simplify the calculations below then
  // we need to scale the tolerance from its absolute quantity into a unit
  // circle fraction:
  //   k = tolerance / radius
  // Using this scaled tolerance below to avoid multiplying by the radius
  // throughout all of the math, we have:
  //   first point = (1, 0)   // theta(0) == 0
  //   theta = kPi / 2 / N    // theta(1)
  //   second point = (cos(theta), sin(theta)) = (c, s)
  //   midpoint = (first + second) * 0.5 = ((1 + c)/2, s/2)
  //   |midpoint| = sqrt((1 + c)*(1 + c)/4 + s*s/4)
  //     = sqrt((1 + c + c + c*c + s*s) / 4)
  //     = sqrt((1 + 2c + 1) / 4)
  //     = sqrt((2 + 2c) / 4)
  //     = sqrt((1 + c) / 2)
  //     = cos(theta / 2)     // using half-angle cosine formula
  //   error = 1 - |midpoint| = 1 - cos(theta / 2)
  //   cos(theta/2) = 1 - error
  //   theta/2 = acos(1 - error)
  //   kPi / 2 / N / 2 = acos(1 - error)
  //   kPi / 4 / acos(1 - error) = N
  // Since we need error <= k, we want divisions >= N, so we use:
  //   N = ceil(kPi / 4 / acos(1 - k))
  //
  // Math is confirmed in https://math.stackexchange.com/a/4132095
  // (keeping in mind that we are computing quarter circle divisions here)
  // which also points out a performance optimization that is accurate
  // to within an over-estimation of 1 division would be:
  //   N = ceil(kPi / 4 / sqrt(2 * k))
  // Since we have precomputed the divisions for radii up to 1024, we can
  // afford to be more accurate using the acos formula here for larger radii.
  double k = Tessellator::kCircleTolerance / pixel_radius;
  return ceil(kPiOver4 / std::acos(1 - k));
}

void Tessellator::Trigs::init(size_t divisions) {
  if (!trigs_.empty()) {
    return;
  }

  // Either not cached yet, or we are using the temp storage...
  trigs_.reserve(divisions + 1);

  double angle_scale = kPiOver2 / divisions;

  trigs_.emplace_back(1.0, 0.0);
  for (size_t i = 1; i < divisions; i++) {
    trigs_.emplace_back(Radians(i * angle_scale));
  }
  trigs_.emplace_back(0.0, 1.0);
}

Tessellator::Trigs Tessellator::GetTrigsForDivisions(size_t divisions) {
  return divisions < Tessellator::kCachedTrigCount
             ? Trigs(precomputed_trigs_[divisions], divisions)
             : Trigs(divisions);
}

using TessellatedVertexProc = Tessellator::TessellatedVertexProc;
using EllipticalVertexGenerator = Tessellator::EllipticalVertexGenerator;

EllipticalVertexGenerator::EllipticalVertexGenerator(
    EllipticalVertexGenerator::GeneratorProc& generator,
    Trigs&& trigs,
    PrimitiveType triangle_type,
    size_t vertices_per_trig,
    Data&& data)
    : impl_(generator),
      trigs_(std::move(trigs)),
      data_(data),
      vertices_per_trig_(vertices_per_trig) {}

EllipticalVertexGenerator Tessellator::FilledCircle(
    const Matrix& view_transform,
    const Point& center,
    Scalar radius) {
  size_t divisions =
      ComputeQuadrantDivisions(view_transform.GetMaxBasisLengthXY() * radius);
  return EllipticalVertexGenerator(Tessellator::GenerateFilledCircle,
                                   GetTrigsForDivisions(divisions),
                                   PrimitiveType::kTriangleStrip, 4,
                                   {
                                       .reference_centers = {center, center},
                                       .radii = {radius, radius},
                                       .half_width = -1.0f,
                                   });
}

EllipticalVertexGenerator Tessellator::StrokedCircle(
    const Matrix& view_transform,
    const Point& center,
    Scalar radius,
    Scalar half_width) {
  if (half_width > 0) {
    auto divisions = ComputeQuadrantDivisions(
        view_transform.GetMaxBasisLengthXY() * radius + half_width);
    return EllipticalVertexGenerator(Tessellator::GenerateStrokedCircle,
                                     GetTrigsForDivisions(divisions),
                                     PrimitiveType::kTriangleStrip, 8,
                                     {
                                         .reference_centers = {center, center},
                                         .radii = {radius, radius},
                                         .half_width = half_width,
                                     });
  } else {
    return FilledCircle(view_transform, center, radius);
  }
}

EllipticalVertexGenerator Tessellator::RoundCapLine(
    const Matrix& view_transform,
    const Point& p0,
    const Point& p1,
    Scalar radius) {
  auto along = p1 - p0;
  auto length = along.GetLength();
  if (length > kEhCloseEnough) {
    auto divisions =
        ComputeQuadrantDivisions(view_transform.GetMaxBasisLengthXY() * radius);
    return EllipticalVertexGenerator(Tessellator::GenerateRoundCapLine,
                                     GetTrigsForDivisions(divisions),
                                     PrimitiveType::kTriangleStrip, 4,
                                     {
                                         .reference_centers = {p0, p1},
                                         .radii = {radius, radius},
                                         .half_width = -1.0f,
                                     });
  } else {
    return FilledCircle(view_transform, p0, radius);
  }
}

EllipticalVertexGenerator Tessellator::FilledEllipse(
    const Matrix& view_transform,
    const Rect& bounds) {
  if (bounds.IsSquare()) {
    return FilledCircle(view_transform, bounds.GetCenter(),
                        bounds.GetWidth() * 0.5f);
  }
  auto max_radius = bounds.GetSize().MaxDimension();
  auto divisions = ComputeQuadrantDivisions(
      view_transform.GetMaxBasisLengthXY() * max_radius);
  auto center = bounds.GetCenter();
  return EllipticalVertexGenerator(Tessellator::GenerateFilledEllipse,
                                   GetTrigsForDivisions(divisions),
                                   PrimitiveType::kTriangleStrip, 4,
                                   {
                                       .reference_centers = {center, center},
                                       .radii = bounds.GetSize() * 0.5f,
                                       .half_width = -1.0f,
                                   });
}

EllipticalVertexGenerator Tessellator::FilledRoundRect(
    const Matrix& view_transform,
    const Rect& bounds,
    const Size& radii) {
  if (radii.width * 2 < bounds.GetWidth() ||
      radii.height * 2 < bounds.GetHeight()) {
    auto max_radius = radii.MaxDimension();
    auto divisions = ComputeQuadrantDivisions(
        view_transform.GetMaxBasisLengthXY() * max_radius);
    auto upper_left = bounds.GetLeftTop() + radii;
    auto lower_right = bounds.GetRightBottom() - radii;
    return EllipticalVertexGenerator(Tessellator::GenerateFilledRoundRect,
                                     GetTrigsForDivisions(divisions),
                                     PrimitiveType::kTriangleStrip, 4,
                                     {
                                         .reference_centers =
                                             {
                                                 upper_left,
                                                 lower_right,
                                             },
                                         .radii = radii,
                                         .half_width = -1.0f,
                                     });
  } else {
    return FilledEllipse(view_transform, bounds);
  }
}

void Tessellator::GenerateFilledCircle(
    const Trigs& trigs,
    const EllipticalVertexGenerator::Data& data,
    const TessellatedVertexProc& proc) {
  auto center = data.reference_centers[0];
  auto radius = data.radii.width;

  FML_DCHECK(center == data.reference_centers[1]);
  FML_DCHECK(radius == data.radii.height);
  FML_DCHECK(data.half_width < 0);

  // Quadrant 1 connecting with Quadrant 4:
  for (auto& trig : trigs) {
    auto offset = trig * radius;
    proc({center.x - offset.x, center.y + offset.y});
    proc({center.x - offset.x, center.y - offset.y});
  }

  // The second half of the circle should be iterated in reverse, but
  // we can instead iterate forward and swap the x/y values of the
  // offset as the angles should be symmetric and thus should generate
  // symmetrically reversed trig vectors.
  // Quadrant 2 connecting with Quadrant 2:
  for (auto& trig : trigs) {
    auto offset = trig * radius;
    proc({center.x + offset.y, center.y + offset.x});
    proc({center.x + offset.y, center.y - offset.x});
  }
}

void Tessellator::GenerateStrokedCircle(
    const Trigs& trigs,
    const EllipticalVertexGenerator::Data& data,
    const TessellatedVertexProc& proc) {
  auto center = data.reference_centers[0];

  FML_DCHECK(center == data.reference_centers[1]);
  FML_DCHECK(data.radii.IsSquare());
  FML_DCHECK(data.half_width > 0 && data.half_width < data.radii.width);

  auto outer_radius = data.radii.width + data.half_width;
  auto inner_radius = data.radii.width - data.half_width;

  // Zig-zag back and forth between points on the outer circle and the
  // inner circle. Both circles are evaluated at the same number of
  // quadrant divisions so the points for a given division should match
  // 1 for 1 other than their applied radius.

  // Quadrant 1:
  for (auto& trig : trigs) {
    auto outer = trig * outer_radius;
    auto inner = trig * inner_radius;
    proc({center.x - outer.x, center.y - outer.y});
    proc({center.x - inner.x, center.y - inner.y});
  }

  // The even quadrants of the circle should be iterated in reverse, but
  // we can instead iterate forward and swap the x/y values of the
  // offset as the angles should be symmetric and thus should generate
  // symmetrically reversed trig vectors.
  // Quadrant 2:
  for (auto& trig : trigs) {
    auto outer = trig * outer_radius;
    auto inner = trig * inner_radius;
    proc({center.x + outer.y, center.y - outer.x});
    proc({center.x + inner.y, center.y - inner.x});
  }

  // Quadrant 3:
  for (auto& trig : trigs) {
    auto outer = trig * outer_radius;
    auto inner = trig * inner_radius;
    proc({center.x + outer.x, center.y + outer.y});
    proc({center.x + inner.x, center.y + inner.y});
  }

  // Quadrant 4:
  for (auto& trig : trigs) {
    auto outer = trig * outer_radius;
    auto inner = trig * inner_radius;
    proc({center.x - outer.y, center.y + outer.x});
    proc({center.x - inner.y, center.y + inner.x});
  }
}

void Tessellator::GenerateRoundCapLine(
    const Trigs& trigs,
    const EllipticalVertexGenerator::Data& data,
    const TessellatedVertexProc& proc) {
  auto p0 = data.reference_centers[0];
  auto p1 = data.reference_centers[1];
  auto radius = data.radii.width;

  FML_DCHECK(radius == data.radii.height);
  FML_DCHECK(data.half_width < 0);

  auto along = p1 - p0;
  along *= radius / along.GetLength();
  auto across = Point(-along.y, along.x);

  for (auto& trig : trigs) {
    auto relative_along = along * trig.cos;
    auto relative_across = across * trig.sin;
    proc(p0 - relative_along + relative_across);
    proc(p0 - relative_along - relative_across);
  }

  // The second half of the round caps should be iterated in reverse, but
  // we can instead iterate forward and swap the sin/cos values as they
  // should be symmetric.
  for (auto& trig : trigs) {
    auto relative_along = along * trig.sin;
    auto relative_across = across * trig.cos;
    proc(p1 + relative_along + relative_across);
    proc(p1 + relative_along - relative_across);
  }
}

void Tessellator::GenerateFilledEllipse(
    const Trigs& trigs,
    const EllipticalVertexGenerator::Data& data,
    const TessellatedVertexProc& proc) {
  auto center = data.reference_centers[0];
  auto radii = data.radii;

  FML_DCHECK(center == data.reference_centers[1]);
  FML_DCHECK(data.half_width < 0);

  // Quadrant 1 connecting with Quadrant 4:
  for (auto& trig : trigs) {
    auto offset = trig * radii;
    proc({center.x - offset.x, center.y + offset.y});
    proc({center.x - offset.x, center.y - offset.y});
  }

  // The second half of the circle should be iterated in reverse, but
  // we can instead iterate forward and swap the x/y values of the
  // offset as the angles should be symmetric and thus should generate
  // symmetrically reversed trig vectors.
  // Quadrant 2 connecting with Quadrant 2:
  for (auto& trig : trigs) {
    auto offset = Point(trig.sin * radii.width, trig.cos * radii.height);
    proc({center.x + offset.x, center.y + offset.y});
    proc({center.x + offset.x, center.y - offset.y});
  }
}

void Tessellator::GenerateFilledRoundRect(
    const Trigs& trigs,
    const EllipticalVertexGenerator::Data& data,
    const TessellatedVertexProc& proc) {
  Scalar left = data.reference_centers[0].x;
  Scalar top = data.reference_centers[0].y;
  Scalar right = data.reference_centers[1].x;
  Scalar bottom = data.reference_centers[1].y;
  auto radii = data.radii;

  FML_DCHECK(data.half_width < 0);

  // Quadrant 1 connecting with Quadrant 4:
  for (auto& trig : trigs) {
    auto offset = trig * radii;
    proc({left - offset.x, bottom + offset.y});
    proc({left - offset.x, top - offset.y});
  }

  // The second half of the round rect should be iterated in reverse, but
  // we can instead iterate forward and swap the x/y values of the
  // offset as the angles should be symmetric and thus should generate
  // symmetrically reversed trig vectors.
  // Quadrant 2 connecting with Quadrant 2:
  for (auto& trig : trigs) {
    auto offset = Point(trig.sin * radii.width, trig.cos * radii.height);
    proc({right + offset.x, bottom + offset.y});
    proc({right + offset.x, top - offset.y});
  }
}

}  // namespace impeller
