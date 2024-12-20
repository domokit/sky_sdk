// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_INPUTS_PLACEHOLDER_FILTER_INPUT_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_INPUTS_PLACEHOLDER_FILTER_INPUT_H_

#include "impeller/entity/contents/filters/inputs/filter_input.h"

namespace impeller {

class PlaceholderFilterInput final : public FilterInput {
 public:
  explicit PlaceholderFilterInput(Rect coverage);

  ~PlaceholderFilterInput() override;

  // |FilterInput|
  std::optional<Snapshot> GetSnapshot(std::string_view label,
                                      const ContentContext& renderer,
                                      const Entity& entity,
                                      std::optional<Rect> coverage_limit,
                                      int32_t mip_count = 1) const override;

  // |FilterInput|
  std::optional<Rect> GetCoverage(const Entity& entity) const override;

 private:
  Rect coverage_rect_;

  friend FilterInput;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_INPUTS_PLACEHOLDER_FILTER_INPUT_H_
