// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/render_target_cache.h"
#include "impeller/core/formats.h"
#include "impeller/renderer/render_target.h"

namespace impeller {

RenderTargetCache::RenderTargetCache(std::shared_ptr<Allocator> allocator,
                                     uint32_t keep_alive_frame_count)
    : RenderTargetAllocator(std::move(allocator)),
      keep_alive_frame_count_(keep_alive_frame_count) {}

void RenderTargetCache::Start() {
  for (auto& td : render_target_data_) {
    td.used_this_frame = false;
  }
}

void RenderTargetCache::End() {
  std::vector<RenderTargetData> retain;

  for (RenderTargetData& td : render_target_data_) {
    if (td.used_this_frame) {
      retain.push_back(td);
    } else if (td.keep_alive_frame_count > 0) {
      td.keep_alive_frame_count--;
      retain.push_back(td);
    }
  }
  render_target_data_.swap(retain);
}

RenderTarget RenderTargetCache::CreateOffscreen(
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

  FML_DCHECK(existing_color_texture == nullptr &&
             existing_depth_stencil_texture == nullptr);
  auto config = RenderTargetConfig{
      .size = size,
      .mip_count = static_cast<size_t>(mip_count),
      .has_msaa = false,
      .has_depth_stencil = stencil_attachment_config.has_value(),
  };
  for (RenderTargetData& render_target_data : render_target_data_) {
    const RenderTargetConfig other_config = render_target_data.config;
    if (!render_target_data.used_this_frame && other_config == config) {
      render_target_data.used_this_frame = true;
      render_target_data.keep_alive_frame_count = keep_alive_frame_count_;
      ColorAttachment color0 =
          render_target_data.render_target.GetColorAttachment(0);
      std::optional<DepthAttachment> depth =
          render_target_data.render_target.GetDepthAttachment();
      std::shared_ptr<Texture> depth_tex = depth ? depth->texture : nullptr;
      return RenderTargetAllocator::CreateOffscreen(
          context, size, mip_count, label, color_attachment_config,
          stencil_attachment_config, color0.texture, depth_tex);
    }
  }
  RenderTarget created_target = RenderTargetAllocator::CreateOffscreen(
      context, size, mip_count, label, color_attachment_config,
      stencil_attachment_config);
  if (!created_target.IsValid()) {
    return created_target;
  }
  render_target_data_.push_back(RenderTargetData{
      .used_this_frame = true,                            //
      .keep_alive_frame_count = keep_alive_frame_count_,  //
      .config = config,                                   //
      .render_target = created_target                     //
  });
  return created_target;
}

RenderTarget RenderTargetCache::CreateOffscreenMSAA(
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

  FML_DCHECK(existing_color_msaa_texture == nullptr &&
             existing_color_resolve_texture == nullptr &&
             existing_depth_stencil_texture == nullptr);
  auto config = RenderTargetConfig{
      .size = size,
      .mip_count = static_cast<size_t>(mip_count),
      .has_msaa = true,
      .has_depth_stencil = stencil_attachment_config.has_value(),
  };
  for (RenderTargetData& render_target_data : render_target_data_) {
    const RenderTargetConfig other_config = render_target_data.config;
    if (!render_target_data.used_this_frame && other_config == config) {
      render_target_data.used_this_frame = true;
      render_target_data.keep_alive_frame_count = keep_alive_frame_count_;
      ColorAttachment color0 =
          render_target_data.render_target.GetColorAttachment(0);
      std::optional<DepthAttachment> depth =
          render_target_data.render_target.GetDepthAttachment();
      std::shared_ptr<Texture> depth_tex = depth ? depth->texture : nullptr;
      return RenderTargetAllocator::CreateOffscreenMSAA(
          context, size, mip_count, label, color_attachment_config,
          stencil_attachment_config, color0.texture, color0.resolve_texture,
          depth_tex);
    }
  }
  RenderTarget created_target = RenderTargetAllocator::CreateOffscreenMSAA(
      context, size, mip_count, label, color_attachment_config,
      stencil_attachment_config);
  if (!created_target.IsValid()) {
    return created_target;
  }
  render_target_data_.push_back(RenderTargetData{
      .used_this_frame = true,                            //
      .keep_alive_frame_count = keep_alive_frame_count_,  //
      .config = config,                                   //
      .render_target = created_target                     //
  });
  return created_target;
}

size_t RenderTargetCache::CachedTextureCount() const {
  return render_target_data_.size();
}

}  // namespace impeller
