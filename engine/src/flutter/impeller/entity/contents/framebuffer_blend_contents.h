// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_FRAMEBUFFER_BLEND_CONTENTS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_FRAMEBUFFER_BLEND_CONTENTS_H_

#include <memory>

#include "impeller/entity/contents/color_source_contents.h"
#include "impeller/entity/entity.h"

namespace impeller {

enum class BlendSelectValues {
  kScreen = 0,
  kOverlay,
  kDarken,
  kLighten,
  kColorDodge,
  kColorBurn,
  kHardLight,
  kSoftLight,
  kDifference,
  kExclusion,
  kMultiply,
  kHue,
  kSaturation,
  kColor,
  kLuminosity,
};

class FramebufferBlendContents final : public ColorSourceContents {
 public:
  FramebufferBlendContents();

  ~FramebufferBlendContents() override;

  void SetBlendMode(BlendMode blend_mode);

  void SetChildContents(std::shared_ptr<Contents> child_contents);

 private:
  // |Contents|
  std::optional<Rect> GetCoverage(const Entity& entity) const override;

  // |Contents|
  bool Render(const ContentContext& renderer,
              const Entity& entity,
              RenderPass& pass) const override;

  BlendMode blend_mode_;
  std::shared_ptr<Contents> child_contents_;

  FramebufferBlendContents(const FramebufferBlendContents&) = delete;

  FramebufferBlendContents& operator=(const FramebufferBlendContents&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_FRAMEBUFFER_BLEND_CONTENTS_H_
