// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/entity_pass_clip_stack.h"

#include "flutter/fml/logging.h"
#include "impeller/entity/contents/clip_contents.h"

namespace impeller {

EntityPassClipStack::EntityPassClipStack(const Rect& initial_coverage_rect) {
  subpass_state_.push_back(SubpassState{
      .clip_coverage =
          {
              {ClipCoverageLayer{
                  .coverage = initial_coverage_rect,
                  .clip_height = 0,
              }},
          },
  });
}

std::optional<Rect> EntityPassClipStack::CurrentClipCoverage() const {
  return subpass_state_.back().clip_coverage.back().coverage;
}

bool EntityPassClipStack::HasCoverage() const {
  return !subpass_state_.back().clip_coverage.empty();
}

void EntityPassClipStack::PushSubpass(std::optional<Rect> subpass_coverage,
                                      size_t clip_height) {
  subpass_state_.push_back(SubpassState{
      .clip_coverage =
          {
              ClipCoverageLayer{.coverage = subpass_coverage,
                                .clip_height = clip_height},
          },
  });
  next_replay_index_ = 0;
}

void EntityPassClipStack::PopSubpass() {
  subpass_state_.pop_back();
  next_replay_index_ = subpass_state_.back().rendered_clip_entities.size();
}

const std::vector<ClipCoverageLayer>
EntityPassClipStack::GetClipCoverageLayers() const {
  return subpass_state_.back().clip_coverage;
}

EntityPassClipStack::ClipStateResult EntityPassClipStack::RecordRestore(
    Point global_pass_position,
    size_t restore_height) {
  ClipStateResult result = {.should_render = false, .clip_did_change = false};
  auto& subpass_state = GetCurrentSubpassState();

  if (subpass_state.clip_coverage.back().clip_height <= restore_height) {
    // Drop clip restores that will do nothing.
    return result;
  }

  auto restoration_index =
      restore_height - subpass_state.clip_coverage.front().clip_height;
  FML_DCHECK(restoration_index < subpass_state.clip_coverage.size());

  // We only need to restore the area that covers the coverage of the
  // clip rect at target height + 1.
  std::optional<Rect> restore_coverage =
      (restoration_index + 1 < subpass_state.clip_coverage.size())
          ? subpass_state.clip_coverage[restoration_index + 1].coverage
          : std::nullopt;
  if (restore_coverage.has_value()) {
    // Make the coverage rectangle relative to the current pass.
    restore_coverage = restore_coverage->Shift(-global_pass_position);
  }

  subpass_state.clip_coverage.resize(restoration_index + 1);
  result.clip_did_change = true;

  if (subpass_state.clip_coverage.back().coverage.has_value()) {
    FML_DCHECK(next_replay_index_ <=
               subpass_state.rendered_clip_entities.size());
    // https://github.com/flutter/flutter/issues/162172
    // This code is slightly wrong and should be popping more than one clip
    // entry.
    if (!subpass_state.rendered_clip_entities.empty()) {
      subpass_state.rendered_clip_entities.pop_back();

      if (next_replay_index_ > subpass_state.rendered_clip_entities.size()) {
        next_replay_index_ = subpass_state.rendered_clip_entities.size();
      }
    }
  }
  return result;
}

EntityPassClipStack::ClipStateResult EntityPassClipStack::RecordClip(
    const ClipContents& clip_contents,
    Matrix transform,
    Point global_pass_position,
    uint32_t clip_depth,
    size_t clip_height_floor,
    bool is_aa) {
  ClipStateResult result = {.should_render = false, .clip_did_change = false};

  std::optional<Rect> maybe_clip_coverage = CurrentClipCoverage();
  // Running this append op won't impact the clip buffer because the
  // whole screen is already being clipped, so skip it.
  if (!maybe_clip_coverage.has_value()) {
    return result;
  }
  auto current_clip_coverage = maybe_clip_coverage.value();
  // Entity transforms are relative to the current pass position, so we need
  // to check clip coverage in the same space.
  current_clip_coverage = current_clip_coverage.Shift(-global_pass_position);

  ClipCoverage clip_coverage =
      clip_contents.GetClipCoverage(current_clip_coverage);
  if (clip_coverage.coverage.has_value()) {
    clip_coverage.coverage =
        clip_coverage.coverage->Shift(global_pass_position);
  }

  SubpassState& subpass_state = GetCurrentSubpassState();

  // Compute the previous clip height.
  size_t previous_clip_height = 0;
  if (!subpass_state.clip_coverage.empty()) {
    previous_clip_height = subpass_state.clip_coverage.back().clip_height;
  } else {
    // If there is no clip coverage, then the previous clip height is the
    // clip height floor.
    previous_clip_height = clip_height_floor;
  }

  // If the new clip coverage is bigger than the existing coverage for
  // intersect clips, we do not need to change the clip region.
  if (!clip_coverage.is_difference_or_non_square &&
      clip_coverage.coverage.has_value() &&
      clip_coverage.coverage.value().Contains(current_clip_coverage)) {
    subpass_state.clip_coverage.push_back(ClipCoverageLayer{
        .coverage = current_clip_coverage,       //
        .clip_height = previous_clip_height + 1  //
    });

    return result;
  }

  // If the clip is an axis aligned rect and either is_aa is false or
  // the clip is very nearly integral, then the depth write can be
  // skipped for intersect clips. Since we use 4x MSAA, anything within
  // < ~0.125 of an integral value in either axis can be treated as
  // approximately the same as an integral value.
  bool should_render = true;
  std::optional<Rect> coverage_value = clip_coverage.coverage;
  if (!clip_coverage.is_difference_or_non_square &&
      coverage_value.has_value()) {
    const Rect& coverage = coverage_value.value();
    constexpr Scalar threshold = 0.124;
    if (!is_aa ||
        (std::abs(std::round(coverage.GetLeft()) - coverage.GetLeft()) <=
             threshold &&
         std::abs(std::round(coverage.GetTop()) - coverage.GetTop()) <=
             threshold &&
         std::abs(std::round(coverage.GetRight()) - coverage.GetRight()) <=
             threshold &&
         std::abs(std::round(coverage.GetBottom()) - coverage.GetBottom()) <=
             threshold)) {
      coverage_value = Rect::Round(clip_coverage.coverage.value());
      should_render = false;
    }
  }

  subpass_state.clip_coverage.push_back(ClipCoverageLayer{
      .coverage = coverage_value,              //
      .clip_height = previous_clip_height + 1  //

  });
  result.clip_did_change = true;
  result.should_render = should_render;

  FML_DCHECK(subpass_state.clip_coverage.back().clip_height ==
             subpass_state.clip_coverage.front().clip_height +
                 subpass_state.clip_coverage.size() - 1);

  FML_DCHECK(next_replay_index_ == subpass_state.rendered_clip_entities.size())
      << "Not all clips have been replayed before appending new clip.";

  subpass_state.rendered_clip_entities.push_back(ReplayResult{
      .clip_contents = clip_contents,   //
      .transform = transform,           //
      .clip_coverage = coverage_value,  //
      .clip_depth = clip_depth          //
  });
  next_replay_index_++;

  return result;
}

EntityPassClipStack::SubpassState&
EntityPassClipStack::GetCurrentSubpassState() {
  return subpass_state_.back();
}

const std::vector<EntityPassClipStack::ReplayResult>&
EntityPassClipStack::GetReplayEntities() const {
  return subpass_state_.back().rendered_clip_entities;
}

}  // namespace impeller
