// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_TEXTURE_CONTENTS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_TEXTURE_CONTENTS_H_

#include <memory>

#include "impeller/core/sampler_descriptor.h"
#include "impeller/entity/contents/contents.h"

namespace impeller {

class Texture;

class TextureContents final : public Contents {
 public:
  TextureContents();

  ~TextureContents() override;

  /// @brief  A common case factory that marks the texture contents as having a
  ///         destination rectangle. In this situation, a subpass can be avoided
  ///         when image filters are applied.
  static std::shared_ptr<TextureContents> MakeRect(Rect destination);

  void SetLabel(std::string_view label);

  void SetDestinationRect(Rect rect);

  void SetTexture(std::shared_ptr<Texture> texture);

  std::shared_ptr<Texture> GetTexture() const;

  void SetSamplerDescriptor(const SamplerDescriptor& desc);

  const SamplerDescriptor& GetSamplerDescriptor() const;

  void SetSourceRect(const Rect& source_rect);

  const Rect& GetSourceRect() const;

  void SetStrictSourceRect(bool strict);

  bool GetStrictSourceRect() const;

  void SetOpacity(Scalar opacity);

  Scalar GetOpacity() const;

  void SetStencilEnabled(bool enabled);

  // |Contents|
  std::optional<Rect> GetCoverage(const Entity& entity) const override;

  // |Contents|
  std::optional<Snapshot> RenderToSnapshot(
      const ContentContext& renderer,
      const Entity& entity,
      std::optional<Rect> coverage_limit = std::nullopt,
      const std::optional<SamplerDescriptor>& sampler_descriptor = std::nullopt,
      bool msaa_enabled = true,
      int32_t mip_count = 1,
      std::string_view label = "Texture Snapshot") const override;

  // |Contents|
  bool Render(const ContentContext& renderer,
              const Entity& entity,
              RenderPass& pass) const override;

  // |Contents|
  void SetInheritedOpacity(Scalar opacity) override;

  void SetDeferApplyingOpacity(bool defer_applying_opacity);

 private:
  std::string label_;

  Rect destination_rect_;
  bool stencil_enabled_ = true;

  std::shared_ptr<Texture> texture_;
  SamplerDescriptor sampler_descriptor_ = {};
  Rect source_rect_;
  bool strict_source_rect_enabled_ = false;
  Scalar opacity_ = 1.0f;
  Scalar inherited_opacity_ = 1.0f;
  bool defer_applying_opacity_ = false;

  TextureContents(const TextureContents&) = delete;

  TextureContents& operator=(const TextureContents&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_TEXTURE_CONTENTS_H_
