// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/render_target.h"

#include <sstream>

#include "impeller/base/strings.h"
#include "impeller/base/validation.h"
#include "impeller/core/allocator.h"
#include "impeller/core/formats.h"
#include "impeller/core/texture.h"
#include "impeller/core/texture_descriptor.h"
#include "impeller/renderer/context.h"

namespace impeller {

RenderTarget::RenderTarget() = default;

RenderTarget::~RenderTarget() = default;

bool RenderTarget::IsValid() const {
  // Validate that there is a color attachment at zero index.
  if (!HasColorAttachment(0u)) {
    VALIDATION_LOG
        << "Render target does not have color attachment at index 0.";
    return false;
  }

#ifndef NDEBUG
  // Validate that all attachments are of the same size.
  {
    std::optional<ISize> size;
    bool sizes_are_same = true;
    auto iterator = [&](const Attachment& attachment) -> bool {
      if (!size.has_value()) {
        size = attachment.texture->GetSize();
      }
      if (size != attachment.texture->GetSize()) {
        sizes_are_same = false;
        return false;
      }
      return true;
    };
    IterateAllAttachments(iterator);
    if (!sizes_are_same) {
      VALIDATION_LOG
          << "Sizes of all render target attachments are not the same.";
      return false;
    }
  }

  // Validate that all attachments are of the same type and sample counts.
  {
    std::optional<TextureType> texture_type;
    std::optional<SampleCount> sample_count;
    bool passes_type_validation = true;
    auto iterator = [&](const Attachment& attachment) -> bool {
      if (!texture_type.has_value() || !sample_count.has_value()) {
        texture_type = attachment.texture->GetTextureDescriptor().type;
        sample_count = attachment.texture->GetTextureDescriptor().sample_count;
      }

      if (texture_type != attachment.texture->GetTextureDescriptor().type) {
        passes_type_validation = false;
        VALIDATION_LOG << "Render target has incompatible texture types: "
                       << TextureTypeToString(texture_type.value()) << " != "
                       << TextureTypeToString(
                              attachment.texture->GetTextureDescriptor().type)
                       << " on target " << ToString();
        return false;
      }

      if (sample_count !=
          attachment.texture->GetTextureDescriptor().sample_count) {
        passes_type_validation = false;
        VALIDATION_LOG << "Render target (" << ToString()
                       << ") has incompatible sample counts.";

        return false;
      }

      return true;
    };
    IterateAllAttachments(iterator);
    if (!passes_type_validation) {
      return false;
    }
  }
#endif  // NDEBUG

  return true;
}

bool RenderTarget::IterateAllColorAttachments(
    const std::function<bool(size_t index, const ColorAttachment& attachment)>&
        iterator) const {
  if (color0_.has_value()) {
    if (!iterator(0, color0_.value())) {
      return false;
    }
  }
  for (const auto& [index, attachment] : colors_) {
    if (!iterator(index, attachment)) {
      return false;
    }
  }
  return true;
}

void RenderTarget::IterateAllAttachments(
    const std::function<bool(const Attachment& attachment)>& iterator) const {
  if (color0_.has_value()) {
    if (!iterator(color0_.value())) {
      return;
    }
  }
  for (const auto& color : colors_) {
    if (!iterator(color.second)) {
      return;
    }
  }

  if (depth_.has_value()) {
    if (!iterator(depth_.value())) {
      return;
    }
  }

  if (stencil_.has_value()) {
    if (!iterator(stencil_.value())) {
      return;
    }
  }
}

SampleCount RenderTarget::GetSampleCount() const {
  if (color0_.has_value()) {
    return color0_.value().texture->GetTextureDescriptor().sample_count;
  }
  return SampleCount::kCount1;
}

bool RenderTarget::HasColorAttachment(size_t index) const {
  if (index == 0u) {
    return color0_.has_value();
  }
  if (auto found = colors_.find(index); found != colors_.end()) {
    return true;
  }
  return false;
}

std::optional<ISize> RenderTarget::GetColorAttachmentSize(size_t index) const {
  if (index == 0u) {
    if (color0_.has_value()) {
      return color0_.value().texture->GetSize();
    }
    return std::nullopt;
  }
  auto found = colors_.find(index);

  if (found == colors_.end()) {
    return std::nullopt;
  }

  return found->second.texture->GetSize();
}

ISize RenderTarget::GetRenderTargetSize() const {
  auto size = GetColorAttachmentSize(0u);
  return size.has_value() ? size.value() : ISize{};
}

std::shared_ptr<Texture> RenderTarget::GetRenderTargetTexture() const {
  if (!color0_.has_value()) {
    return nullptr;
  }
  return color0_->resolve_texture ? color0_->resolve_texture : color0_->texture;
}

PixelFormat RenderTarget::GetRenderTargetPixelFormat() const {
  if (auto texture = GetRenderTargetTexture(); texture != nullptr) {
    return texture->GetTextureDescriptor().format;
  }

  return PixelFormat::kUnknown;
}

size_t RenderTarget::GetMaxColorAttacmentBindIndex() const {
  size_t max = 0;
  for (const auto& color : colors_) {
    max = std::max(color.first, max);
  }
  return max;
}

RenderTarget& RenderTarget::SetColorAttachment(
    const ColorAttachment& attachment,
    size_t index) {
  if (!attachment.IsValid()) {
    return *this;
  }
  if (index == 0u) {
    color0_ = attachment;
  } else {
    colors_[index] = attachment;
  }
  return *this;
}

RenderTarget& RenderTarget::SetDepthAttachment(
    std::optional<DepthAttachment> attachment) {
  if (!attachment.has_value()) {
    depth_ = std::nullopt;
  } else if (attachment->IsValid()) {
    depth_ = std::move(attachment);
  }
  return *this;
}

RenderTarget& RenderTarget::SetStencilAttachment(
    std::optional<StencilAttachment> attachment) {
  if (!attachment.has_value()) {
    stencil_ = std::nullopt;
  } else if (attachment->IsValid()) {
    stencil_ = std::move(attachment);
  }
  return *this;
}

ColorAttachment RenderTarget::GetColorAttachment(size_t index) const {
  if (index == 0) {
    if (color0_.has_value()) {
      return color0_.value();
    }
    return ColorAttachment{};
  }
  std::map<size_t, ColorAttachment>::const_iterator it = colors_.find(index);
  if (it != colors_.end()) {
    return it->second;
  }
  return ColorAttachment{};
}

const std::optional<DepthAttachment>& RenderTarget::GetDepthAttachment() const {
  return depth_;
}

const std::optional<StencilAttachment>& RenderTarget::GetStencilAttachment()
    const {
  return stencil_;
}

size_t RenderTarget::GetTotalAttachmentCount() const {
  size_t count = 0u;
  for (const auto& [_, color] : colors_) {
    if (color.texture) {
      count++;
    }
    if (color.resolve_texture) {
      count++;
    }
  }
  if (color0_.has_value()) {
    count++;
  }
  if (depth_.has_value()) {
    count++;
  }
  if (stencil_.has_value()) {
    count++;
  }
  return count;
}

std::string RenderTarget::ToString() const {
  std::stringstream stream;

  if (color0_.has_value()) {
    stream << SPrintF("Color[%d]=(%s)", 0,
                      ColorAttachmentToString(color0_.value()).c_str());
  }
  for (const auto& [index, color] : colors_) {
    stream << SPrintF("Color[%zu]=(%s)", index,
                      ColorAttachmentToString(color).c_str());
  }
  if (depth_) {
    stream << ",";
    stream << SPrintF("Depth=(%s)",
                      DepthAttachmentToString(depth_.value()).c_str());
  }
  if (stencil_) {
    stream << ",";
    stream << SPrintF("Stencil=(%s)",
                      StencilAttachmentToString(stencil_.value()).c_str());
  }
  return stream.str();
}

RenderTargetConfig RenderTarget::ToConfig() const {
  if (!color0_.has_value()) {
    return RenderTargetConfig{};
  }
  const auto& color_attachment = color0_.value();
  return RenderTargetConfig{
      .size = color_attachment.texture->GetSize(),
      .mip_count = color_attachment.texture->GetMipCount(),
      .has_msaa = color_attachment.resolve_texture != nullptr,
      .has_depth_stencil = depth_.has_value() && stencil_.has_value()};
}

RenderTargetAllocator::RenderTargetAllocator(
    std::shared_ptr<Allocator> allocator)
    : allocator_(std::move(allocator)) {}

void RenderTargetAllocator::Start() {}

void RenderTargetAllocator::End() {}

RenderTarget RenderTargetAllocator::CreateOffscreen(
    const Context& context,
    ISize size,
    int mip_count,
    std::string_view label,
    RenderTarget::AttachmentConfig color_attachment_config,
    std::optional<RenderTarget::AttachmentConfig> stencil_attachment_config,
    const std::shared_ptr<Texture>& existing_color_texture,
    const std::shared_ptr<Texture>& existing_depth_stencil_texture) {
  if (size.IsEmpty()) {
    return {};
  }

  RenderTarget target;

  std::shared_ptr<Texture> color0_tex;
  if (existing_color_texture) {
    color0_tex = existing_color_texture;
  } else {
    PixelFormat pixel_format =
        context.GetCapabilities()->GetDefaultColorFormat();
    TextureDescriptor color0_tex_desc;
    color0_tex_desc.storage_mode = color_attachment_config.storage_mode;
    color0_tex_desc.format = pixel_format;
    color0_tex_desc.size = size;
    color0_tex_desc.mip_count = mip_count;
    color0_tex_desc.usage =
        TextureUsage::kRenderTarget | TextureUsage::kShaderRead;
    color0_tex = allocator_->CreateTexture(color0_tex_desc);
    if (!color0_tex) {
      return {};
    }
  }
  color0_tex->SetLabel(label, "Color Texture");

  ColorAttachment color0;
  color0.clear_color = color_attachment_config.clear_color;
  color0.load_action = color_attachment_config.load_action;
  color0.store_action = color_attachment_config.store_action;
  color0.texture = color0_tex;
  target.SetColorAttachment(color0, 0u);

  if (stencil_attachment_config.has_value()) {
    target.SetupDepthStencilAttachments(
        context, *allocator_, size, false, label,
        stencil_attachment_config.value(), existing_depth_stencil_texture);
  } else {
    target.SetStencilAttachment(std::nullopt);
    target.SetDepthAttachment(std::nullopt);
  }

  return target;
}

RenderTarget RenderTargetAllocator::CreateOffscreenMSAA(
    const Context& context,
    ISize size,
    int mip_count,
    std::string_view label,
    RenderTarget::AttachmentConfigMSAA color_attachment_config,
    std::optional<RenderTarget::AttachmentConfig> stencil_attachment_config,
    const std::shared_ptr<Texture>& existing_color_msaa_texture,
    const std::shared_ptr<Texture>& existing_color_resolve_texture,
    const std::shared_ptr<Texture>& existing_depth_stencil_texture) {
  if (size.IsEmpty()) {
    return {};
  }

  RenderTarget target;
  PixelFormat pixel_format = context.GetCapabilities()->GetDefaultColorFormat();

  // Create MSAA color texture.
  std::shared_ptr<Texture> color0_msaa_tex;
  if (existing_color_msaa_texture) {
    color0_msaa_tex = existing_color_msaa_texture;
  } else {
    TextureDescriptor color0_tex_desc;
    color0_tex_desc.storage_mode = color_attachment_config.storage_mode;
    color0_tex_desc.type = TextureType::kTexture2DMultisample;
    color0_tex_desc.sample_count = SampleCount::kCount4;
    color0_tex_desc.format = pixel_format;
    color0_tex_desc.size = size;
    color0_tex_desc.usage = TextureUsage::kRenderTarget;
    if (context.GetCapabilities()->SupportsImplicitResolvingMSAA()) {
      // See below ("SupportsImplicitResolvingMSAA") for more details.
      color0_tex_desc.storage_mode = StorageMode::kDevicePrivate;
    }
    color0_msaa_tex = allocator_->CreateTexture(color0_tex_desc);
    if (!color0_msaa_tex) {
      VALIDATION_LOG << "Could not create multisample color texture.";
      return {};
    }
  }
  color0_msaa_tex->SetLabel(label, "Color Texture (Multisample)");

  // Create color resolve texture.
  std::shared_ptr<Texture> color0_resolve_tex;
  if (existing_color_resolve_texture) {
    color0_resolve_tex = existing_color_resolve_texture;
  } else {
    TextureDescriptor color0_resolve_tex_desc;
    color0_resolve_tex_desc.storage_mode =
        color_attachment_config.resolve_storage_mode;
    color0_resolve_tex_desc.format = pixel_format;
    color0_resolve_tex_desc.size = size;
    color0_resolve_tex_desc.compression_type = CompressionType::kLossy;
    color0_resolve_tex_desc.usage =
        TextureUsage::kRenderTarget | TextureUsage::kShaderRead;
    color0_resolve_tex_desc.mip_count = mip_count;
    color0_resolve_tex = allocator_->CreateTexture(color0_resolve_tex_desc);
    if (!color0_resolve_tex) {
      VALIDATION_LOG << "Could not create color texture.";
      return {};
    }
  }
  color0_resolve_tex->SetLabel(label, "Color Texture");

  // Color attachment.

  ColorAttachment color0;
  color0.clear_color = color_attachment_config.clear_color;
  color0.load_action = color_attachment_config.load_action;
  color0.store_action = color_attachment_config.store_action;
  color0.texture = color0_msaa_tex;
  color0.resolve_texture = color0_resolve_tex;

  if (context.GetCapabilities()->SupportsImplicitResolvingMSAA()) {
    // If implicit MSAA is supported, then the resolve texture is not needed
    // because the multisample texture is automatically resolved. We instead
    // provide a view of the multisample texture as the resolve texture (because
    // the HAL does expect a resolve texture).
    //
    // In practice, this is used for GLES 2.0 EXT_multisampled_render_to_texture
    // https://registry.khronos.org/OpenGL/extensions/EXT/EXT_multisampled_render_to_texture.txt
    color0.resolve_texture = color0_msaa_tex;
  }

  target.SetColorAttachment(color0, 0u);

  // Create MSAA stencil texture.

  if (stencil_attachment_config.has_value()) {
    target.SetupDepthStencilAttachments(context, *allocator_, size, true, label,
                                        stencil_attachment_config.value(),
                                        existing_depth_stencil_texture);
  } else {
    target.SetDepthAttachment(std::nullopt);
    target.SetStencilAttachment(std::nullopt);
  }

  return target;
}

void RenderTarget::SetupDepthStencilAttachments(
    const Context& context,
    Allocator& allocator,
    ISize size,
    bool msaa,
    std::string_view label,
    RenderTarget::AttachmentConfig stencil_attachment_config,
    const std::shared_ptr<Texture>& existing_depth_stencil_texture) {
  std::shared_ptr<Texture> depth_stencil_texture;
  if (existing_depth_stencil_texture) {
    depth_stencil_texture = existing_depth_stencil_texture;
  } else {
    TextureDescriptor depth_stencil_texture_desc;
    depth_stencil_texture_desc.storage_mode =
        stencil_attachment_config.storage_mode;
    if (msaa) {
      depth_stencil_texture_desc.type = TextureType::kTexture2DMultisample;
      depth_stencil_texture_desc.sample_count = SampleCount::kCount4;
    }
    depth_stencil_texture_desc.format =
        context.GetCapabilities()->GetDefaultDepthStencilFormat();
    depth_stencil_texture_desc.size = size;
    depth_stencil_texture_desc.usage = TextureUsage::kRenderTarget;
    depth_stencil_texture = allocator.CreateTexture(depth_stencil_texture_desc);
    if (!depth_stencil_texture) {
      return;  // Error messages are handled by `Allocator::CreateTexture`.
    }
  }

  DepthAttachment depth0;
  depth0.load_action = stencil_attachment_config.load_action;
  depth0.store_action = stencil_attachment_config.store_action;
  depth0.clear_depth = 0u;
  depth0.texture = depth_stencil_texture;

  StencilAttachment stencil0;
  stencil0.load_action = stencil_attachment_config.load_action;
  stencil0.store_action = stencil_attachment_config.store_action;
  stencil0.clear_stencil = 0u;
  stencil0.texture = std::move(depth_stencil_texture);
  stencil0.texture->SetLabel(label, "Depth+Stencil Texture");

  SetDepthAttachment(std::move(depth0));
  SetStencilAttachment(std::move(stencil0));
}

}  // namespace impeller
