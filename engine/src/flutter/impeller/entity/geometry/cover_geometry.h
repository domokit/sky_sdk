// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_GEOMETRY_COVER_GEOMETRY_H_
#define FLUTTER_IMPELLER_ENTITY_GEOMETRY_COVER_GEOMETRY_H_

#include "impeller/entity/geometry/geometry.h"

namespace impeller {

/// @brief A geometry that implements "drawPaint" like behavior by covering
///        the entire render pass area.
class CoverGeometry final : public Geometry {
 public:
  CoverGeometry();

  ~CoverGeometry() override = default;

  // |Geometry|
  bool CoversArea(const Matrix& transform, const Rect& rect) const override;

  bool CanApplyMaskFilter() const override;

 private:
  // |Geometry|
  GeometryResult GetPositionBuffer(const ContentContext& renderer,
                                   const Entity& entity,
                                   RenderPass& pass) const override;

  // |Geometry|
  std::optional<Rect> GetCoverage(const Matrix& transform) const override;

  CoverGeometry(const CoverGeometry&) = delete;

  CoverGeometry& operator=(const CoverGeometry&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_GEOMETRY_COVER_GEOMETRY_H_
