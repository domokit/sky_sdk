// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/filters/local_matrix_filter_contents.h"

namespace impeller {

LocalMatrixFilterContents::LocalMatrixFilterContents() = default;

LocalMatrixFilterContents::~LocalMatrixFilterContents() = default;

void LocalMatrixFilterContents::SetMatrix(Matrix matrix) {
  matrix_ = matrix;
}

Matrix LocalMatrixFilterContents::GetLocalTransform(
    const Matrix& parent_transform) const {
  return matrix_;
}

std::optional<Rect> LocalMatrixFilterContents::GetFilterSourceCoverage(
    const Matrix& effect_transform,
    const Rect& output_limit) const {
  auto matrix = matrix_.Basis();
  if (!matrix.IsInvertible()) {
    return std::nullopt;
  }
  auto inverse = matrix.Invert();
  return output_limit.TransformBounds(inverse);
}

std::optional<Entity> LocalMatrixFilterContents::RenderFilter(
    const FilterInput::Vector& inputs,
    const ContentContext& renderer,
    const Entity& entity,
    const Matrix& effect_transform,
    const Rect& coverage,
    const std::optional<Rect>& coverage_hint) const {
  std::optional<Snapshot> snapshot =
      inputs[0]->GetSnapshot("LocalMatrix", renderer, entity);
  if (!snapshot.has_value()) {
    return std::nullopt;
  }
  return Entity::FromSnapshot(snapshot.value(), entity.GetBlendMode());
}

}  // namespace impeller
