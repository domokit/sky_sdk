// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_CLIP_CONTENTS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_CLIP_CONTENTS_H_

#include "impeller/entity/contents/contents.h"
#include "impeller/entity/entity.h"
#include "impeller/entity/geometry/geometry.h"

namespace impeller {

struct ClipCoverage {
  // TODO(jonahwilliams): this should probably use the Entity::ClipOperation
  // enum, but that has transitive import errors.
  bool is_difference_or_non_square = false;

  /// @brief This coverage is the outer coverage of the clip.
  ///
  /// For example, if the clip is a circular clip, this is the rectangle that
  /// contains the circle and not the rectangle that is contained within the
  /// circle. This means that we cannot use the coverage alone to determine if
  /// a clip can be culled, and instead also use the somewhat hacky
  /// "is_difference_or_non_square" field.
  std::optional<Rect> coverage = std::nullopt;
};

class ClipContents {
 public:
  ClipContents(Rect coverage_rect, bool is_axis_aligned_rect);

  ~ClipContents();

  /// @brief Set the pre-tessellated clip geometry.
  void SetGeometry(GeometryResult geometry);

  void SetClipOperation(Entity::ClipOperation clip_op);

  //----------------------------------------------------------------------------
  /// @brief Given the current pass space bounding rectangle of the clip
  ///        buffer, return the expected clip coverage after this draw call.
  ///        This should only be implemented for contents that may write to the
  ///        clip buffer.
  ///
  ///        During rendering, coverage coordinates count pixels from the top
  ///        left corner of the framebuffer.
  ///
  ClipCoverage GetClipCoverage(
      const std::optional<Rect>& current_clip_coverage) const;

  bool Render(const ContentContext& renderer,
              RenderPass& pass,
              uint32_t clip_depth) const;

 private:
  // Pre-tessellated clip geometry.
  GeometryResult clip_geometry_;
  // Coverage rect of the tessellated geometry.
  Rect coverage_rect_;
  bool is_axis_aligned_rect_ = false;
  Entity::ClipOperation clip_op_ = Entity::ClipOperation::kIntersect;
};

/// @brief Render a restore clip.
///
/// This is is intended to be used for prevent overdraw mechanism. The clip
/// depth should be the depth of the entity that is currently being drawn, and
/// restore_coverage should be its coverage. If restore_coverage is
/// std::nullopt, the render pass coverage is used instead.
bool RenderClipRestore(const ContentContext& renderer,
                       RenderPass& pass,
                       uint32_t clip_depth,
                       std::optional<Rect> restore_coverage);

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_CLIP_CONTENTS_H_
