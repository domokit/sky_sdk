// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/inline_pass_context.h"

#include <utility>

#include "flutter/fml/status.h"
#include "impeller/base/validation.h"
#include "impeller/core/formats.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/entity_pass_target.h"
#include "impeller/renderer/command_buffer.h"
#include "impeller/renderer/render_pass.h"
#include "impeller/renderer/texture_mipmap.h"

namespace impeller {

InlinePassContext::InlinePassContext(const ContentContext& renderer,
                                     EntityPassTarget& pass_target)
    : renderer_(renderer), pass_target_(pass_target) {}

InlinePassContext::~InlinePassContext() {
  EndPass();
}

bool InlinePassContext::IsValid() const {
  return pass_target_.IsValid();
}

bool InlinePassContext::IsActive() const {
  return pass_ != nullptr;
}

std::shared_ptr<Texture> InlinePassContext::GetTexture() {
  if (!IsValid()) {
    return nullptr;
  }
  return pass_target_.GetRenderTarget().GetRenderTargetTexture();
}

bool InlinePassContext::EndPass() {
  if (!IsActive()) {
    return true;
  }
  FML_DCHECK(command_buffer_);

  if (!pass_->EncodeCommands()) {
    VALIDATION_LOG << "Failed to encode and submit command buffer while ending "
                      "render pass.";
    return false;
  }

  const std::shared_ptr<Texture>& target_texture =
      GetPassTarget().GetRenderTarget().GetRenderTargetTexture();
  if (target_texture->GetMipCount() > 1) {
    fml::Status mip_status = AddMipmapGeneration(
        command_buffer_, renderer_.GetContext(), target_texture);
    if (!mip_status.ok()) {
      return false;
    }
  }

  pass_ = nullptr;
  return renderer_.GetContext()->EnqueueCommandBuffer(
      std::move(command_buffer_));
}

EntityPassTarget& InlinePassContext::GetPassTarget() const {
  return pass_target_;
}

const std::shared_ptr<RenderPass>& InlinePassContext::GetRenderPass() {
  if (IsActive()) {
    return pass_;
  }

  /// Create a new render pass if one isn't active. This path will run the first
  /// time this method is called, but it'll also run if the pass has been
  /// previously ended via `EndPass`.

  command_buffer_ = renderer_.GetContext()->CreateCommandBuffer();
  if (!command_buffer_) {
    VALIDATION_LOG << "Could not create command buffer.";
    return pass_;
  }

  command_buffer_->SetLabel("EntityPass Command Buffer");

  {
    // If the pass target has a resolve texture, then we're using MSAA.
    bool is_msaa =
        pass_target_.GetRenderTarget().GetColorAttachment(0).resolve_texture !=
        nullptr;
    if (pass_count_ > 0 && is_msaa) {
      pass_target_.Flip(renderer_);
    }
  }

  // Find the color attachment a second time, since the target may have just
  // flipped.
  ColorAttachment color0 = pass_target_.GetRenderTarget().GetColorAttachment(0);
  bool is_msaa = color0.resolve_texture != nullptr;

  if (pass_count_ > 0) {
    // When MSAA is being used, we end up overriding the entire backdrop by
    // drawing the previous pass texture, and so we don't have to clear it and
    // can use kDontCare.
    color0.load_action = is_msaa ? LoadAction::kDontCare : LoadAction::kLoad;
  } else {
    color0.load_action = LoadAction::kClear;
  }

  color0.store_action =
      is_msaa ? StoreAction::kMultisampleResolve : StoreAction::kStore;

  auto depth = pass_target_.GetRenderTarget().GetDepthAttachment();
  if (!depth.has_value()) {
    VALIDATION_LOG << "Depth attachment unexpectedly missing from the "
                      "EntityPass render target.";
    return pass_;
  }
  depth->load_action = LoadAction::kClear;
  depth->store_action = StoreAction::kDontCare;
  pass_target_.target_.SetDepthAttachment(depth.value());

  auto stencil = pass_target_.GetRenderTarget().GetStencilAttachment();
  if (!depth.has_value() || !stencil.has_value()) {
    VALIDATION_LOG << "Stencil/Depth attachment unexpectedly missing from the "
                      "EntityPass render target.";
    return pass_;
  }
  stencil->load_action = LoadAction::kClear;
  stencil->store_action = StoreAction::kDontCare;
  depth->load_action = LoadAction::kClear;
  depth->store_action = StoreAction::kDontCare;
  pass_target_.target_.SetDepthAttachment(depth);
  pass_target_.target_.SetStencilAttachment(stencil.value());
  pass_target_.target_.SetColorAttachment(color0, 0);

  pass_ = command_buffer_->CreateRenderPass(pass_target_.GetRenderTarget());
  if (!pass_) {
    VALIDATION_LOG << "Could not create render pass.";
    return pass_;
  }
  pass_->SetLabel("EntityPass Render Pass");

  ++pass_count_;
  return pass_;
}

uint32_t InlinePassContext::GetPassCount() const {
  return pass_count_;
}

}  // namespace impeller
