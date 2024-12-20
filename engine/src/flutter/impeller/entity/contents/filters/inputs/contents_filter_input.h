// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_INPUTS_CONTENTS_FILTER_INPUT_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_INPUTS_CONTENTS_FILTER_INPUT_H_

#include "impeller/entity/contents/filters/inputs/filter_input.h"

namespace impeller {

class ContentsFilterInput final : public FilterInput {
 public:
  ~ContentsFilterInput() override;

  // |FilterInput|
  std::optional<Snapshot> GetSnapshot(std::string_view label,
                                      const ContentContext& renderer,
                                      const Entity& entity,
                                      std::optional<Rect> coverage_limit,
                                      int32_t mip_count) const override;

  // |FilterInput|
  std::optional<Rect> GetCoverage(const Entity& entity) const override;

 private:
  ContentsFilterInput(std::shared_ptr<Contents> contents, bool msaa_enabled);

  std::shared_ptr<Contents> contents_;
  mutable std::optional<Snapshot> snapshot_;
  bool msaa_enabled_;

  friend FilterInput;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_FILTERS_INPUTS_CONTENTS_FILTER_INPUT_H_
