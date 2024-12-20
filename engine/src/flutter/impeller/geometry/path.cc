// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/geometry/path.h"

#include <optional>
#include <utility>

#include "flutter/fml/logging.h"
#include "impeller/geometry/path_component.h"
#include "impeller/geometry/point.h"

namespace impeller {

Path::Path() : data_(new Data()) {}

Path::Path(Data data) : data_(std::make_shared<Data>(std::move(data))) {}

Path::~Path() = default;

std::tuple<size_t, size_t> Path::Polyline::GetContourPointBounds(
    size_t contour_index) const {
  if (contour_index >= contours.size()) {
    return {points->size(), points->size()};
  }
  const size_t start_index = contours.at(contour_index).start_index;
  const size_t end_index = (contour_index >= contours.size() - 1)
                               ? points->size()
                               : contours.at(contour_index + 1).start_index;
  return std::make_tuple(start_index, end_index);
}

size_t Path::GetComponentCount(std::optional<ComponentType> type) const {
  if (!type.has_value()) {
    return data_->components.size();
  }
  auto type_value = type.value();
  size_t count = 0u;
  for (const auto& component : data_->components) {
    if (component == type_value) {
      count++;
    }
  }
  return count;
}

FillType Path::GetFillType() const {
  return data_->fill;
}

bool Path::IsConvex() const {
  return data_->convexity == Convexity::kConvex;
}

bool Path::IsEmpty() const {
  return data_->points.empty() ||
         (data_->components.size() == 1 &&
          data_->components[0] == ComponentType::kContour);
}

bool Path::IsSingleContour() const {
  return data_->single_countour;
}

/// Determine required storage for points and indices.
std::pair<size_t, size_t> Path::CountStorage(Scalar scale) const {
  size_t points = 0;
  size_t contours = 0;

  auto& path_components = data_->components;
  auto& path_points = data_->points;

  size_t storage_offset = 0u;
  for (size_t component_i = 0; component_i < path_components.size();
       component_i++) {
    const auto& path_component = path_components[component_i];
    switch (path_component) {
      case ComponentType::kLinear: {
        points += 2;
        break;
      }
      case ComponentType::kQuadratic: {
        const QuadraticPathComponent* quad =
            reinterpret_cast<const QuadraticPathComponent*>(
                &path_points[storage_offset]);
        points += quad->CountLinearPathComponents(scale);
        break;
      }
      case ComponentType::kCubic: {
        const CubicPathComponent* cubic =
            reinterpret_cast<const CubicPathComponent*>(
                &path_points[storage_offset]);
        points += cubic->CountLinearPathComponents(scale);
        break;
      }
      case Path::ComponentType::kContour:
        contours++;
    }
    storage_offset += VerbToOffset(path_component);
  }
  return std::make_pair(points, contours);
}

void Path::WritePolyline(Scalar scale, VertexWriter& writer) const {
  auto& path_components = data_->components;
  auto& path_points = data_->points;
  bool started_contour = false;
  bool first_point = true;

  size_t storage_offset = 0u;
  for (size_t component_i = 0; component_i < path_components.size();
       component_i++) {
    const auto& path_component = path_components[component_i];
    switch (path_component) {
      case ComponentType::kLinear: {
        const LinearPathComponent* linear =
            reinterpret_cast<const LinearPathComponent*>(
                &path_points[storage_offset]);
        if (first_point) {
          writer.Write(linear->p1);
          first_point = false;
        }
        writer.Write(linear->p2);
        break;
      }
      case ComponentType::kQuadratic: {
        const QuadraticPathComponent* quad =
            reinterpret_cast<const QuadraticPathComponent*>(
                &path_points[storage_offset]);
        if (first_point) {
          writer.Write(quad->p1);
          first_point = false;
        }
        quad->ToLinearPathComponents(scale, writer);
        break;
      }
      case ComponentType::kCubic: {
        const CubicPathComponent* cubic =
            reinterpret_cast<const CubicPathComponent*>(
                &path_points[storage_offset]);
        if (first_point) {
          writer.Write(cubic->p1);
          first_point = false;
        }
        cubic->ToLinearPathComponents(scale, writer);
        break;
      }
      case Path::ComponentType::kContour:
        if (component_i == path_components.size() - 1) {
          // If the last component is a contour, that means it's an empty
          // contour, so skip it.
          continue;
        }
        // The contour component type is the first segment in a contour.
        // Since this should contain the destination (if closed), we
        // can start with this point. If there was already an open
        // contour, or we've reached the end of the verb list, we
        // also close the contour.
        if (started_contour) {
          writer.EndContour();
        }
        started_contour = true;
        first_point = true;
    }
    storage_offset += VerbToOffset(path_component);
  }
  if (started_contour) {
    writer.EndContour();
  }
}

bool Path::GetLinearComponentAtIndex(size_t index,
                                     LinearPathComponent& linear) const {
  auto& components = data_->components;
  if (index >= components.size() ||
      components[index] != ComponentType::kLinear) {
    return false;
  }

  size_t storage_offset = 0u;
  for (auto i = 0u; i < index; i++) {
    storage_offset += VerbToOffset(components[i]);
  }
  auto& points = data_->points;
  linear =
      LinearPathComponent(points[storage_offset], points[storage_offset + 1]);
  return true;
}

bool Path::GetQuadraticComponentAtIndex(
    size_t index,
    QuadraticPathComponent& quadratic) const {
  auto& components = data_->components;
  if (index >= components.size() ||
      components[index] != ComponentType::kQuadratic) {
    return false;
  }

  size_t storage_offset = 0u;
  for (auto i = 0u; i < index; i++) {
    storage_offset += VerbToOffset(components[i]);
  }
  auto& points = data_->points;

  quadratic =
      QuadraticPathComponent(points[storage_offset], points[storage_offset + 1],
                             points[storage_offset + 2]);
  return true;
}

bool Path::GetCubicComponentAtIndex(size_t index,
                                    CubicPathComponent& cubic) const {
  auto& components = data_->components;
  if (index >= components.size() ||
      components[index] != ComponentType::kCubic) {
    return false;
  }

  size_t storage_offset = 0u;
  for (auto i = 0u; i < index; i++) {
    storage_offset += VerbToOffset(components[i]);
  }
  auto& points = data_->points;

  cubic = CubicPathComponent(points[storage_offset], points[storage_offset + 1],
                             points[storage_offset + 2],
                             points[storage_offset + 3]);
  return true;
}

bool Path::GetContourComponentAtIndex(size_t index,
                                      ContourComponent& move) const {
  auto& components = data_->components;
  if (index >= components.size() ||
      components[index] != ComponentType::kContour) {
    return false;
  }

  size_t storage_offset = 0u;
  for (auto i = 0u; i < index; i++) {
    storage_offset += VerbToOffset(components[i]);
  }
  auto& points = data_->points;

  move = ContourComponent(points[storage_offset], points[storage_offset + 1]);
  return true;
}

Path::Polyline::Polyline(Path::Polyline::PointBufferPtr point_buffer,
                         Path::Polyline::ReclaimPointBufferCallback reclaim)
    : points(std::move(point_buffer)), reclaim_points_(std::move(reclaim)) {
  FML_DCHECK(points);
}

Path::Polyline::Polyline(Path::Polyline&& other) {
  points = std::move(other.points);
  reclaim_points_ = std::move(other.reclaim_points_);
  contours = std::move(other.contours);
}

Path::Polyline::~Polyline() {
  if (reclaim_points_) {
    points->clear();
    reclaim_points_(std::move(points));
  }
}

void Path::EndContour(
    size_t storage_offset,
    Polyline& polyline,
    size_t component_index,
    std::vector<PolylineContour::Component>& poly_components) const {
  auto& path_components = data_->components;
  auto& path_points = data_->points;
  // Whenever a contour has ended, extract the exact end direction from
  // the last component.
  if (polyline.contours.empty() || component_index == 0) {
    return;
  }

  auto& contour = polyline.contours.back();
  contour.end_direction = Vector2(0, 1);
  contour.components = poly_components;
  poly_components.clear();

  size_t previous_index = component_index - 1;
  storage_offset -= VerbToOffset(path_components[previous_index]);

  while (previous_index >= 0 && storage_offset >= 0) {
    const auto& path_component = path_components[previous_index];
    switch (path_component) {
      case ComponentType::kLinear: {
        auto* linear = reinterpret_cast<const LinearPathComponent*>(
            &path_points[storage_offset]);
        auto maybe_end = linear->GetEndDirection();
        if (maybe_end.has_value()) {
          contour.end_direction = maybe_end.value();
          return;
        }
        break;
      }
      case ComponentType::kQuadratic: {
        auto* quad = reinterpret_cast<const QuadraticPathComponent*>(
            &path_points[storage_offset]);
        auto maybe_end = quad->GetEndDirection();
        if (maybe_end.has_value()) {
          contour.end_direction = maybe_end.value();
          return;
        }
        break;
      }
      case ComponentType::kCubic: {
        auto* cubic = reinterpret_cast<const CubicPathComponent*>(
            &path_points[storage_offset]);
        auto maybe_end = cubic->GetEndDirection();
        if (maybe_end.has_value()) {
          contour.end_direction = maybe_end.value();
          return;
        }
        break;
      }
      case ComponentType::kContour: {
        // Hit previous contour, return.
        return;
      };
    }
    storage_offset -= VerbToOffset(path_component);
    previous_index--;
  }
};

Path::Polyline Path::CreatePolyline(
    Scalar scale,
    Path::Polyline::PointBufferPtr point_buffer,
    Path::Polyline::ReclaimPointBufferCallback reclaim) const {
  Polyline polyline(std::move(point_buffer), std::move(reclaim));

  auto& path_components = data_->components;
  auto& path_points = data_->points;
  std::optional<Vector2> start_direction;
  std::vector<PolylineContour::Component> poly_components;
  size_t storage_offset = 0u;
  size_t component_i = 0;

  for (; component_i < path_components.size(); component_i++) {
    auto path_component = path_components[component_i];
    switch (path_component) {
      case ComponentType::kLinear: {
        poly_components.push_back({
            .component_start_index = polyline.points->size() - 1,
            .is_curve = false,
        });
        auto* linear = reinterpret_cast<const LinearPathComponent*>(
            &path_points[storage_offset]);
        linear->AppendPolylinePoints(*polyline.points);
        if (!start_direction.has_value()) {
          start_direction = linear->GetStartDirection();
        }
        break;
      }
      case ComponentType::kQuadratic: {
        poly_components.push_back({
            .component_start_index = polyline.points->size() - 1,
            .is_curve = true,
        });
        auto* quad = reinterpret_cast<const QuadraticPathComponent*>(
            &path_points[storage_offset]);
        quad->AppendPolylinePoints(scale, *polyline.points);
        if (!start_direction.has_value()) {
          start_direction = quad->GetStartDirection();
        }
        break;
      }
      case ComponentType::kCubic: {
        poly_components.push_back({
            .component_start_index = polyline.points->size() - 1,
            .is_curve = true,
        });
        auto* cubic = reinterpret_cast<const CubicPathComponent*>(
            &path_points[storage_offset]);
        cubic->AppendPolylinePoints(scale, *polyline.points);
        if (!start_direction.has_value()) {
          start_direction = cubic->GetStartDirection();
        }
        break;
      }
      case ComponentType::kContour:
        if (component_i == path_components.size() - 1) {
          // If the last component is a contour, that means it's an empty
          // contour, so skip it.
          break;
        }
        if (!polyline.contours.empty()) {
          polyline.contours.back().start_direction =
              start_direction.value_or(Vector2(0, -1));
          start_direction = std::nullopt;
        }
        EndContour(storage_offset, polyline, component_i, poly_components);

        auto* contour = reinterpret_cast<const ContourComponent*>(
            &path_points[storage_offset]);
        polyline.contours.push_back(PolylineContour{
            .start_index = polyline.points->size(),  //
            .is_closed = contour->IsClosed(),        //
            .start_direction = Vector2(0, -1),       //
            .components = poly_components            //
        });

        polyline.points->push_back(contour->destination);
        break;
    }
    storage_offset += VerbToOffset(path_component);
  }

  // Subtract the last storage offset increment so that the storage lookup is
  // correct, including potentially an empty contour as well.
  if (component_i > 0 && path_components.back() == ComponentType::kContour) {
    storage_offset -= VerbToOffset(ComponentType::kContour);
    component_i--;
  }

  if (!polyline.contours.empty()) {
    polyline.contours.back().start_direction =
        start_direction.value_or(Vector2(0, -1));
  }
  EndContour(storage_offset, polyline, component_i, poly_components);
  return polyline;
}

std::optional<Rect> Path::GetBoundingBox() const {
  return data_->bounds;
}

std::optional<Rect> Path::GetTransformedBoundingBox(
    const Matrix& transform) const {
  auto bounds = GetBoundingBox();
  if (!bounds.has_value()) {
    return std::nullopt;
  }
  return bounds->TransformBounds(transform);
}

}  // namespace impeller
