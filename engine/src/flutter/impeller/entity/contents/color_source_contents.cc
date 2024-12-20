// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/color_source_contents.h"

#include "impeller/entity/entity.h"
#include "impeller/geometry/matrix.h"

namespace impeller {

ColorSourceContents::ColorSourceContents() = default;

ColorSourceContents::~ColorSourceContents() = default;

void ColorSourceContents::SetGeometry(const Geometry* geometry) {
  geometry_ = geometry;
}

const Geometry* ColorSourceContents::GetGeometry() const {
  return geometry_;
}

void ColorSourceContents::SetOpacityFactor(Scalar alpha) {
  opacity_ = alpha;
}

Scalar ColorSourceContents::GetOpacityFactor() const {
  return opacity_ * inherited_opacity_;
}

void ColorSourceContents::SetEffectTransform(Matrix matrix) {
  inverse_matrix_ = matrix.Invert();
}

const Matrix& ColorSourceContents::GetInverseEffectTransform() const {
  return inverse_matrix_;
}

bool ColorSourceContents::IsSolidColor() const {
  return false;
}

std::optional<Rect> ColorSourceContents::GetCoverage(
    const Entity& entity) const {
  return geometry_->GetCoverage(entity.GetTransform());
};

void ColorSourceContents::SetInheritedOpacity(Scalar opacity) {
  inherited_opacity_ = opacity;
}

bool ColorSourceContents::AppliesAlphaForStrokeCoverage(
    const Matrix& transform) const {
  return GetGeometry() && GetGeometry()->ComputeAlphaCoverage(transform) < 1.0;
}

}  // namespace impeller
