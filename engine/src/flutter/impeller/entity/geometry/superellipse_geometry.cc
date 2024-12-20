// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <vector>

#include "flutter/impeller/entity/geometry/superellipse_geometry.h"

#include "impeller/geometry/constants.h"

namespace impeller {

SuperellipseGeometry::SuperellipseGeometry(const Point& center,
                                           Scalar radius,
                                           Scalar degree,
                                           Scalar alpha,
                                           Scalar beta)
    : center_(center),
      degree_(degree),
      radius_(radius),
      alpha_(alpha),
      beta_(beta) {}

SuperellipseGeometry::~SuperellipseGeometry() {}

GeometryResult SuperellipseGeometry::GetPositionBuffer(
    const ContentContext& renderer,
    const Entity& entity,
    RenderPass& pass) const {
  // https://math.stackexchange.com/questions/2573746/superellipse-parametric-equation
  Scalar a = alpha_;
  Scalar b = beta_;
  Scalar n = degree_;

  // TODO(jonahwilliams): determine parameter values based on scaling factor.
  Scalar step = kPi / 80;

  // Generate the points for the top left quadrant, and then mirror to the other
  // quadrants.
  std::vector<Point> points;
  points.reserve(41);
  for (int i = 0; i <= 40; i++) {
    Scalar t = i * step;
    Scalar x = a * pow(abs(cos(t)), 2 / n);
    Scalar y = b * pow(abs(sin(t)), 2 / n);
    points.emplace_back(x * radius_, y * radius_);
  }

  static constexpr Point reflection[4] = {{1, 1}, {-1, 1}, {-1, -1}, {1, -1}};

  // Reflect into the 4 quadrants and generate the tessellated mesh. The
  // iteration order is reversed so that the trianges are continuous from
  // quadrant to quadrant.
  std::vector<Point> geometry;
  geometry.reserve(1 + 4 * points.size());
  geometry.push_back(center_);
  for (auto i = 0u; i < points.size(); i++) {
    geometry.push_back(center_ + (reflection[0] * points[i]));
  }
  for (auto i = 0u; i < points.size(); i++) {
    geometry.push_back(center_ +
                       (reflection[1] * points[points.size() - i - 1]));
  }
  for (auto i = 0u; i < points.size(); i++) {
    geometry.push_back(center_ + (reflection[2] * points[i]));
  }
  for (auto i = 0u; i < points.size(); i++) {
    geometry.push_back(center_ +
                       (reflection[3] * points[points.size() - i - 1]));
  }

  std::vector<uint16_t> indices;
  indices.reserve(geometry.size() * 3);
  for (auto i = 2u; i < geometry.size(); i++) {
    indices.push_back(0);
    indices.push_back(i - 1);
    indices.push_back(i);
  }

  auto& host_buffer = renderer.GetTransientsBuffer();
  return GeometryResult{
      .type = PrimitiveType::kTriangle,
      .vertex_buffer =
          {
              .vertex_buffer = host_buffer.Emplace(
                  geometry.data(), geometry.size() * sizeof(Point),
                  alignof(Point)),
              .index_buffer = host_buffer.Emplace(
                  indices.data(), indices.size() * sizeof(uint16_t),
                  alignof(uint16_t)),
              .vertex_count = indices.size(),
              .index_type = IndexType::k16bit,
          },
      .transform = entity.GetShaderTransform(pass),
  };
}

std::optional<Rect> SuperellipseGeometry::GetCoverage(
    const Matrix& transform) const {
  return Rect::MakeOriginSize(center_ - Point(radius_, radius_),
                              Size(radius_ * 2, radius_ * 2));
}

bool SuperellipseGeometry::CoversArea(const Matrix& transform,
                                      const Rect& rect) const {
  return false;
}

bool SuperellipseGeometry::IsAxisAlignedRect() const {
  return false;
}

}  // namespace impeller
