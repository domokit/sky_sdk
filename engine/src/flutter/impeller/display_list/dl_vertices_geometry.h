// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_DISPLAY_LIST_DL_VERTICES_GEOMETRY_H_
#define FLUTTER_IMPELLER_DISPLAY_LIST_DL_VERTICES_GEOMETRY_H_

#include "flutter/display_list/dl_vertices.h"
#include "impeller/core/formats.h"
#include "impeller/entity/geometry/vertices_geometry.h"

namespace impeller {

/// @brief A geometry that is created from a DlVertices object.
class DlVerticesGeometry final : public VerticesGeometry {
 public:
  DlVerticesGeometry(const std::shared_ptr<const flutter::DlVertices>& vertices,
                     const ContentContext& renderer);

  ~DlVerticesGeometry() = default;

  GeometryResult GetPositionUVColorBuffer(Rect texture_coverage,
                                          Matrix effect_transform,
                                          const ContentContext& renderer,
                                          const Entity& entity,
                                          RenderPass& pass) const override;

  // |Geometry|
  GeometryResult GetPositionBuffer(const ContentContext& renderer,
                                   const Entity& entity,
                                   RenderPass& pass) const override;

  // |Geometry|
  std::optional<Rect> GetCoverage(const Matrix& transform) const override;

  bool HasVertexColors() const override;

  bool HasTextureCoordinates() const override;

  std::optional<Rect> GetTextureCoordinateCoverge() const override;

 private:
  PrimitiveType GetPrimitiveType() const;

  /// @brief Check if index normalization is required, returning whether or
  ///        not it was performed.
  ///
  /// If true, [indices_] should be used in place of the vertices object's
  /// indices.
  bool MaybePerformIndexNormalization(const ContentContext& renderer);

  const std::shared_ptr<const flutter::DlVertices> vertices_;
  std::vector<uint16_t> indices_;
  bool performed_normalization_ = false;
  Rect bounds_;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_DISPLAY_LIST_DL_VERTICES_GEOMETRY_H_
