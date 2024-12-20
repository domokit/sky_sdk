// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/display_list/dl_image_impeller.h"

#include "impeller/display_list/aiks_context.h"
#include "impeller/entity/contents/filters/filter_contents.h"

namespace impeller {

#if FML_OS_IOS_SIMULATOR
sk_sp<DlImageImpeller> DlImageImpeller::Make(std::shared_ptr<Texture> texture,
                                             OwningContext owning_context,
                                             bool is_fake_image) {
  if (!texture && !is_fake_image) {
    return nullptr;
  }
  return sk_sp<DlImageImpeller>(
      new DlImageImpeller(std::move(texture), owning_context, is_fake_image));
}
#else
sk_sp<DlImageImpeller> DlImageImpeller::Make(std::shared_ptr<Texture> texture,
                                             OwningContext owning_context) {
  if (!texture) {
    return nullptr;
  }
  return sk_sp<DlImageImpeller>(
      new DlImageImpeller(std::move(texture), owning_context));
}
#endif  // FML_OS_IOS_SIMULATOR

sk_sp<DlImageImpeller> DlImageImpeller::MakeFromYUVTextures(
    AiksContext* aiks_context,
    std::shared_ptr<Texture> y_texture,
    std::shared_ptr<Texture> uv_texture,
    YUVColorSpace yuv_color_space) {
  if (!aiks_context || !y_texture || !uv_texture) {
    return nullptr;
  }
  auto yuv_to_rgb_filter_contents = FilterContents::MakeYUVToRGBFilter(
      std::move(y_texture), std::move(uv_texture), yuv_color_space);
  impeller::Entity entity;
  entity.SetBlendMode(impeller::BlendMode::kSource);
  auto snapshot = yuv_to_rgb_filter_contents->RenderToSnapshot(
      aiks_context->GetContentContext(),  // renderer
      entity,                             // entity
      std::nullopt,                       // coverage_limit
      std::nullopt,                       // sampler_descriptor
      true,                               // msaa_enabled
      /*mip_count=*/1,
      "MakeYUVToRGBFilter Snapshot");  // label
  if (!snapshot.has_value()) {
    return nullptr;
  }
  return impeller::DlImageImpeller::Make(snapshot->texture);
}

DlImageImpeller::DlImageImpeller(std::shared_ptr<Texture> texture,
                                 OwningContext owning_context
#ifdef FML_OS_IOS_SIMULATOR
                                 ,
                                 bool is_fake_image
#endif  // FML_OS_IOS_SIMULATOR
                                 )
    : texture_(std::move(texture)),
      owning_context_(owning_context)
#ifdef FML_OS_IOS_SIMULATOR
      ,
      is_fake_image_(is_fake_image)
#endif  // #ifdef FML_OS_IOS_SIMULATOR
{
}

// |DlImage|
DlImageImpeller::~DlImageImpeller() = default;

// |DlImage|
sk_sp<SkImage> DlImageImpeller::skia_image() const {
  return nullptr;
};

// |DlImage|
std::shared_ptr<impeller::Texture> DlImageImpeller::impeller_texture() const {
  return texture_;
}

// |DlImage|
bool DlImageImpeller::isOpaque() const {
  // Impeller doesn't currently implement opaque alpha types.
  return false;
}

// |DlImage|
bool DlImageImpeller::isTextureBacked() const {
  // Impeller textures are always ... textures :/
  return true;
}

// |DlImage|
bool DlImageImpeller::isUIThreadSafe() const {
  // Impeller textures are always thread-safe
  return true;
}

// |DlImage|
SkISize DlImageImpeller::dimensions() const {
  const auto size = texture_ ? texture_->GetSize() : ISize{};
  return SkISize::Make(size.width, size.height);
}

// |DlImage|
size_t DlImageImpeller::GetApproximateByteSize() const {
  auto size = sizeof(*this);
  if (texture_) {
    size += texture_->GetTextureDescriptor().GetByteSizeOfBaseMipLevel();
  }
  return size;
}

}  // namespace impeller
