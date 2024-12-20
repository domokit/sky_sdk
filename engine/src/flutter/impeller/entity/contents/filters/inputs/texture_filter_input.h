// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_INPUTS_TEXTURE_FILTER_INPUT_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_INPUTS_TEXTURE_FILTER_INPUT_H_

#include "impeller/entity/contents/filters/inputs/filter_input.h"

#include "impeller/geometry/matrix.h"

namespace impeller {

class TextureFilterInput final : public FilterInput {
 public:
  ~TextureFilterInput() override;

  // |FilterInput|
  std::optional<Snapshot> GetSnapshot(std::string_view label,
                                      const ContentContext& renderer,
                                      const Entity& entity,
                                      std::optional<Rect> coverage_limit,
                                      int32_t mip_count) const override;

  // |FilterInput|
  std::optional<Rect> GetCoverage(const Entity& entity) const override;

  // |FilterInput|
  Matrix GetLocalTransform(const Entity& entity) const override;

 private:
  explicit TextureFilterInput(std::shared_ptr<Texture> texture,
                              Matrix local_transform = Matrix());

  std::shared_ptr<Texture> texture_;
  Matrix local_transform_;

  friend FilterInput;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_INPUTS_TEXTURE_FILTER_INPUT_H_
