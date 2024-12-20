// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_SNAPSHOT_H_
#define FLUTTER_IMPELLER_RENDERER_SNAPSHOT_H_

#include <functional>
#include <memory>
#include <vector>

#include "impeller/core/formats.h"
#include "impeller/core/sampler_descriptor.h"
#include "impeller/core/texture.h"
#include "impeller/geometry/matrix.h"
#include "impeller/geometry/rect.h"

namespace impeller {

class ContentContext;
class Entity;

/// Represents a texture and its intended draw transform/sampler configuration.
struct Snapshot {
  std::shared_ptr<Texture> texture;
  /// The transform that should be applied to this texture for rendering.
  Matrix transform;

  SamplerDescriptor sampler_descriptor =
      SamplerDescriptor("Default Snapshot Sampler",
                        MinMagFilter::kLinear,
                        MinMagFilter::kLinear,
                        MipFilter::kNearest);

  Scalar opacity = 1.0f;

  std::optional<Rect> GetCoverage() const;

  /// @brief  Get the transform that converts screen space coordinates to the UV
  ///         space of this snapshot.
  std::optional<Matrix> GetUVTransform() const;

  /// @brief  Map a coverage rect to this filter input's UV space.
  ///         Result order: Top left, top right, bottom left, bottom right.
  std::optional<std::array<Point, 4>> GetCoverageUVs(
      const Rect& coverage) const;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_SNAPSHOT_H_
