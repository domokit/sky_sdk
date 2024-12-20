// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_DISPLAY_LIST_DL_ATLAS_GEOMETRY_H_
#define FLUTTER_IMPELLER_DISPLAY_LIST_DL_ATLAS_GEOMETRY_H_

#include "display_list/dl_color.h"
#include "display_list/image/dl_image.h"
#include "impeller/core/sampler_descriptor.h"
#include "impeller/entity/contents/atlas_contents.h"
#include "impeller/geometry/color.h"
#include "include/core/SkRSXform.h"

namespace impeller {

/// @brief A wrapper around data provided by a drawAtlas call.
class DlAtlasGeometry : public AtlasGeometry {
 public:
  DlAtlasGeometry(const std::shared_ptr<Texture>& atlas,
                  const SkRSXform* xform,
                  const flutter::DlRect* tex,
                  const flutter::DlColor* colors,
                  size_t count,
                  BlendMode mode,
                  const SamplerDescriptor& sampling,
                  std::optional<Rect> cull_rect);

  ~DlAtlasGeometry();

  /// @brief Whether the blend shader should be used.
  bool ShouldUseBlend() const override;

  bool ShouldSkip() const override;

  VertexBuffer CreateSimpleVertexBuffer(HostBuffer& host_buffer) const override;

  VertexBuffer CreateBlendVertexBuffer(HostBuffer& host_buffer) const override;

  Rect ComputeBoundingBox() const override;

  std::shared_ptr<Texture> GetAtlas() const override;

  const SamplerDescriptor& GetSamplerDescriptor() const override;

  BlendMode GetBlendMode() const override;

 private:
  const std::shared_ptr<Texture> atlas_;
  const SkRSXform* xform_;
  const flutter::DlRect* tex_;
  const flutter::DlColor* colors_;
  size_t count_;
  BlendMode mode_;
  SamplerDescriptor sampling_;
  mutable std::optional<Rect> cull_rect_;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_DISPLAY_LIST_DL_ATLAS_GEOMETRY_H_
