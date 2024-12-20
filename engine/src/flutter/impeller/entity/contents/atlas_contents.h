// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_ATLAS_CONTENTS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_ATLAS_CONTENTS_H_

#include <memory>

#include "impeller/core/sampler_descriptor.h"
#include "impeller/entity/contents/contents.h"
#include "impeller/entity/entity.h"
#include "impeller/geometry/color.h"

namespace impeller {

// Interface wrapper to allow usage of DL pointer data without copying (or
// circular imports).
class AtlasGeometry {
 public:
  virtual bool ShouldUseBlend() const = 0;

  virtual bool ShouldSkip() const = 0;

  virtual VertexBuffer CreateSimpleVertexBuffer(
      HostBuffer& host_buffer) const = 0;

  virtual VertexBuffer CreateBlendVertexBuffer(
      HostBuffer& host_buffer) const = 0;

  virtual Rect ComputeBoundingBox() const = 0;

  virtual std::shared_ptr<Texture> GetAtlas() const = 0;

  virtual const SamplerDescriptor& GetSamplerDescriptor() const = 0;

  virtual BlendMode GetBlendMode() const = 0;
};

class AtlasContents final : public Contents {
 public:
  explicit AtlasContents();

  ~AtlasContents() override;

  void SetGeometry(AtlasGeometry* geometry);

  void SetAlpha(Scalar alpha);

  // |Contents|
  std::optional<Rect> GetCoverage(const Entity& entity) const override;

  // |Contents|
  bool Render(const ContentContext& renderer,
              const Entity& entity,
              RenderPass& pass) const override;

 private:
  AtlasGeometry* geometry_ = nullptr;
  Scalar alpha_ = 1.0;

  AtlasContents(const AtlasContents&) = delete;

  AtlasContents& operator=(const AtlasContents&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_ATLAS_CONTENTS_H_
