// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/filters/matrix_filter_contents.h"

namespace impeller {

MatrixFilterContents::MatrixFilterContents() = default;

MatrixFilterContents::~MatrixFilterContents() = default;

void MatrixFilterContents::SetMatrix(Matrix matrix) {
  matrix_ = matrix;
}

void MatrixFilterContents::SetRenderingMode(
    Entity::RenderingMode rendering_mode) {
  rendering_mode_ = rendering_mode;
  FilterContents::SetRenderingMode(rendering_mode);
}

void MatrixFilterContents::SetSamplerDescriptor(const SamplerDescriptor& desc) {
  sampler_descriptor_ = desc;
}

namespace {
Matrix CalculateSubpassTransform(const Matrix& snapshot_transform,
                                 const Matrix& effect_transform,
                                 const Matrix& matrix,
                                 Entity::RenderingMode rendering_mode) {
  if (rendering_mode ==
      Entity::RenderingMode::kSubpassAppendSnapshotTransform) {
    return snapshot_transform *  //
           effect_transform *    //
           matrix *              //
           effect_transform.Invert();
  } else {
    FML_DCHECK(rendering_mode ==
               Entity::RenderingMode::kSubpassPrependSnapshotTransform);
    return effect_transform *           //
           matrix *                     //
           effect_transform.Invert() *  //
           snapshot_transform;
  }
}
}  // namespace

std::optional<Entity> MatrixFilterContents::RenderFilter(
    const FilterInput::Vector& inputs,
    const ContentContext& renderer,
    const Entity& entity,
    const Matrix& effect_transform,
    const Rect& coverage,
    const std::optional<Rect>& coverage_hint) const {
  auto snapshot = inputs[0]->GetSnapshot("Matrix", renderer, entity);
  if (!snapshot.has_value()) {
    return std::nullopt;
  }

  if (rendering_mode_ ==
          Entity::RenderingMode::kSubpassPrependSnapshotTransform ||
      rendering_mode_ ==
          Entity::RenderingMode::kSubpassAppendSnapshotTransform) {
    // There are two special quirks with how Matrix filters behave when used as
    // subpass backdrop filters:
    //
    // 1. For subpass backdrop filters, the snapshot transform is always just a
    //    translation that positions the parent pass texture correctly relative
    //    to the subpass texture. However, this translation always needs to be
    //    applied in screen space.
    //
    //    Since we know the snapshot transform will always have an identity
    //    basis in this case, we safely reverse the order and apply the filter's
    //    matrix within the snapshot transform space.
    //
    // 2. The filter's matrix needs to be applied within the space defined by
    //    the scene's current transformation matrix (CTM). For example: If the
    //    CTM is scaled up, then translations applied by the matrix should be
    //    magnified accordingly.
    //
    //    To accomplish this, we sandwitch the filter's matrix within the CTM in
    //    both cases. But notice that for the subpass backdrop filter case, we
    //    use the "effect transform" instead of the Entity's transform!
    //
    //    That's because in the subpass backdrop filter case, the Entity's
    //    transform isn't actually the captured CTM of the scene like it usually
    //    is; instead, it's just a screen space translation that offsets the
    //    backdrop texture (as mentioned above). And so we sneak the subpass's
    //    captured CTM in through the effect transform.
    //
    snapshot->transform = CalculateSubpassTransform(
        snapshot->transform, effect_transform, matrix_, rendering_mode_);
  } else {
    snapshot->transform = entity.GetTransform() *           //
                          matrix_ *                         //
                          entity.GetTransform().Invert() *  //
                          snapshot->transform;
  }
  snapshot->sampler_descriptor = sampler_descriptor_;
  if (!snapshot.has_value()) {
    return std::nullopt;
  }
  return Entity::FromSnapshot(snapshot.value(), entity.GetBlendMode());
}

std::optional<Rect> MatrixFilterContents::GetFilterSourceCoverage(
    const Matrix& effect_transform,
    const Rect& output_limit) const {
  auto transform = effect_transform *          //
                   matrix_ *                   //
                   effect_transform.Invert();  //
  if (!transform.IsInvertible()) {
    return std::nullopt;
  }
  auto inverse = transform.Invert();
  return output_limit.TransformBounds(inverse);
}

std::optional<Rect> MatrixFilterContents::GetFilterCoverage(
    const FilterInput::Vector& inputs,
    const Entity& entity,
    const Matrix& effect_transform) const {
  if (inputs.empty()) {
    return std::nullopt;
  }

  std::optional<Rect> coverage = inputs[0]->GetCoverage(entity);
  if (!coverage.has_value()) {
    return std::nullopt;
  }

  Matrix input_transform = inputs[0]->GetTransform(entity);
  if (rendering_mode_ ==
          Entity::RenderingMode::kSubpassPrependSnapshotTransform ||
      rendering_mode_ ==
          Entity::RenderingMode::kSubpassAppendSnapshotTransform) {
    Rect coverage_bounds = coverage->TransformBounds(input_transform.Invert());
    Matrix transform = CalculateSubpassTransform(
        input_transform, effect_transform, matrix_, rendering_mode_);
    return coverage_bounds.TransformBounds(transform);
  } else {
    Matrix transform = input_transform *          //
                       matrix_ *                  //
                       input_transform.Invert();  //
    return coverage->TransformBounds(transform);
  }
}

}  // namespace impeller
