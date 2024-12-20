// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_GEOMETRY_VERTICES_GEOMETRY_H_
#define FLUTTER_IMPELLER_ENTITY_GEOMETRY_VERTICES_GEOMETRY_H_

#include "impeller/entity/geometry/geometry.h"

namespace impeller {

/// @brief A geometry that is created from a vertices object.
class VerticesGeometry : public Geometry {
 public:
  virtual GeometryResult GetPositionUVColorBuffer(
      Rect texture_coverage,
      Matrix effect_transform,
      const ContentContext& renderer,
      const Entity& entity,
      RenderPass& pass) const = 0;

  virtual bool HasVertexColors() const = 0;

  virtual bool HasTextureCoordinates() const = 0;

  virtual std::optional<Rect> GetTextureCoordinateCoverge() const = 0;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_GEOMETRY_VERTICES_GEOMETRY_H_
