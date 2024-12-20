// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/filters/inputs/placeholder_filter_input.h"

#include <optional>

namespace impeller {

PlaceholderFilterInput::PlaceholderFilterInput(Rect coverage_rect)
    : coverage_rect_(coverage_rect) {}

PlaceholderFilterInput::~PlaceholderFilterInput() = default;

std::optional<Snapshot> PlaceholderFilterInput::GetSnapshot(
    std::string_view label,
    const ContentContext& renderer,
    const Entity& entity,
    std::optional<Rect> coverage_limit,
    int32_t mip_count) const {
  return std::nullopt;
}

std::optional<Rect> PlaceholderFilterInput::GetCoverage(
    const Entity& entity) const {
  return coverage_rect_;
}

}  // namespace impeller
