// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/snapshot.h"

#include <optional>

namespace impeller {

std::optional<Rect> Snapshot::GetCoverage() const {
  if (!texture) {
    return std::nullopt;
  }
  return Rect::MakeSize(texture->GetSize()).TransformBounds(transform);
}

std::optional<Matrix> Snapshot::GetUVTransform() const {
  if (!texture || texture->GetSize().IsEmpty()) {
    return std::nullopt;
  }
  return Matrix::MakeScale(1 / Vector2(texture->GetSize())) *
         transform.Invert();
}

std::optional<std::array<Point, 4>> Snapshot::GetCoverageUVs(
    const Rect& coverage) const {
  auto uv_transform = GetUVTransform();
  if (!uv_transform.has_value()) {
    return std::nullopt;
  }
  return coverage.GetTransformedPoints(uv_transform.value());
}

}  // namespace impeller
