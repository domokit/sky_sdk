// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/surface_texture_external_texture_vk_impeller.h"

#include <chrono>

#include <GLES2/gl2.h>
#define GL_GLEXT_PROTOTYPES
#include <GLES2/gl2ext.h>

#include "flutter/fml/trace_event.h"
#include "flutter/impeller/display_list/dl_image_impeller.h"
#include "flutter/impeller/renderer/backend/vulkan/command_buffer_vk.h"
#include "flutter/impeller/renderer/backend/vulkan/surface_context_vk.h"
#include "flutter/impeller/renderer/backend/vulkan/texture_vk.h"
#include "flutter/impeller/toolkit/android/hardware_buffer.h"

namespace flutter {

using namespace impeller;

SurfaceTextureExternalTextureVKImpeller::
    SurfaceTextureExternalTextureVKImpeller(
        std::shared_ptr<impeller::ContextVK> context,
        int64_t id,
        const fml::jni::ScopedJavaGlobalRef<jobject>& surface_texture,
        const std::shared_ptr<PlatformViewAndroidJNI>& jni_facade)
    : SurfaceTextureExternalTexture(id, surface_texture, jni_facade),
      context_(std::move(context)),
      trampoline_(std::make_shared<glvk::Trampoline>()) {
  is_valid_ = trampoline_->IsValid();
}

SurfaceTextureExternalTextureVKImpeller::
    ~SurfaceTextureExternalTextureVKImpeller() = default;

enum class LayoutUpdateMode {
  kSync,
  kAsync,
};

static bool SetTextureLayout(const ContextVK& context,
                             const TextureSourceVK* texture,
                             vk::ImageLayout layout,
                             LayoutUpdateMode mode) {
  TRACE_EVENT0("flutter", __FUNCTION__);
  if (!texture) {
    return true;
  }
  auto command_buffer = context.CreateCommandBuffer();
  if (!command_buffer) {
    VALIDATION_LOG
        << "Could not create command buffer for texture layout update.";
    return false;
  }
  command_buffer->SetLabel("GLVKTextureLayoutUpdateCB");
  const CommandBufferVK& encoder = CommandBufferVK::Cast(*command_buffer);
  const auto command_buffer_vk = encoder.GetCommandBuffer();

  BarrierVK barrier;
  barrier.cmd_buffer = command_buffer_vk;
  barrier.new_layout = layout;
  barrier.src_stage = vk::PipelineStageFlagBits::eColorAttachmentOutput |
                      impeller::vk::PipelineStageFlagBits::eFragmentShader;
  barrier.src_access = vk::AccessFlagBits::eColorAttachmentWrite |
                       vk::AccessFlagBits::eShaderRead;
  barrier.dst_stage = impeller::vk::PipelineStageFlagBits::eFragmentShader;
  barrier.dst_access = vk::AccessFlagBits::eShaderRead;

  if (!texture->SetLayout(barrier).ok()) {
    VALIDATION_LOG << "Could not encoder layout transition.";
    return false;
  }

  encoder.EndCommandBuffer();

  vk::SubmitInfo submit_info;
  submit_info.setCommandBuffers(command_buffer_vk);

  // There is no need to track the fence in the encoder since we are going to do
  // a sync wait for completion.
  vk::UniqueFence fence;

  if (mode == LayoutUpdateMode::kSync) {
    auto fence_pair =
        context.GetDevice().createFenceUnique(vk::FenceCreateFlags{});
    if (fence_pair.result != impeller::vk::Result::eSuccess) {
      VALIDATION_LOG << "Could not create fence.";
      return false;
    }
    fence = std::move(fence_pair.value);
  }

  if (context.GetGraphicsQueue()->Submit(submit_info, fence.get()) !=
      impeller::vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not submit layout transition fence.";
    return false;
  }

  using namespace std::chrono_literals;

  if (fence &&
      context.GetDevice().waitForFences(
          fence.get(),                                                      //
          VK_TRUE,                                                          //
          std::chrono::duration_cast<std::chrono::nanoseconds>(1s).count()  //
          ) != impeller::vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not perform sync wait on fence.";
    return false;
  }

  return true;
}

// |SurfaceTextureExternalTexture|
void SurfaceTextureExternalTextureVKImpeller::ProcessFrame(
    PaintContext& context,
    const SkRect& bounds) {
  if (!is_valid_ || !context.aiks_context) {
    VALIDATION_LOG << "Invalid external texture.";
    return;
  }

  const auto& surface_context =
      SurfaceContextVK::Cast(*context.aiks_context->GetContext());
  const auto& context_vk = ContextVK::Cast(*surface_context.GetParent());

  auto dst_texture =
      GetCachedTextureSource(surface_context.GetParent(),                    //
                             ISize::MakeWH(bounds.width(), bounds.height())  //
      );
  if (!dst_texture || !dst_texture->IsValid()) {
    VALIDATION_LOG << "Could not fetch trampoline texture target.";
    return;
  }

  auto current_context = trampoline_->MakeCurrentContext();

  GLuint src_gl_texture = {};
  glGenTextures(1u, &src_gl_texture);
  Attach(src_gl_texture);
  Update();

  SetTextureLayout(context_vk, dst_texture.get(),
                   vk::ImageLayout::eColorAttachmentOptimal,
                   LayoutUpdateMode::kSync);

  impeller::Matrix uv_transformation;
  GetCurrentUVTransformation().getColMajor(
      reinterpret_cast<SkScalar*>(&uv_transformation));

  glvk::Trampoline::GLTextureInfo src_texture;
  src_texture.texture = src_gl_texture;
  src_texture.target = GL_TEXTURE_EXTERNAL_OES;
  src_texture.uv_transformation = uv_transformation;

  if (!trampoline_->BlitTextureOpenGLToVulkan(src_texture, *dst_texture)) {
    VALIDATION_LOG << "Texture copy failed.";
  }

  SetTextureLayout(context_vk, dst_texture.get(),
                   vk::ImageLayout::eShaderReadOnlyOptimal,
                   LayoutUpdateMode::kAsync);

  glDeleteTextures(1u, &src_gl_texture);

  dl_image_ = DlImageImpeller::Make(
      std::make_shared<TextureVK>(surface_context.GetParent(), dst_texture));
}

// |SurfaceTextureExternalTexture|
void SurfaceTextureExternalTextureVKImpeller::Detach() {
  // Detaching from the underlying surface texture requires a context to be
  // current. On the other hand, the texture source is a pure Vulkan construct
  // and has no EGL related requirements.
  auto context = trampoline_->MakeCurrentContext();
  SurfaceTextureExternalTexture::Detach();
  cached_texture_source_.reset();
}

std::shared_ptr<impeller::AHBTextureSourceVK>
SurfaceTextureExternalTextureVKImpeller::GetCachedTextureSource(
    const std::shared_ptr<Context>& context,
    const impeller::ISize& size) {
  if (cached_texture_source_ &&
      cached_texture_source_->GetTextureDescriptor().size == size) {
    return cached_texture_source_;
  }
  cached_texture_source_ = nullptr;

  android::HardwareBufferDescriptor ahb_descriptor;
  ahb_descriptor.format = android::HardwareBufferFormat::kR8G8B8A8UNormInt;
  ahb_descriptor.size = size.Max(ISize{1u, 1u});
  ahb_descriptor.usage =
      android::HardwareBufferUsageFlags::kFrameBufferAttachment |
      android::HardwareBufferUsageFlags::kSampledImage;

  if (!ahb_descriptor.IsAllocatable()) {
    VALIDATION_LOG << "Invalid hardware buffer texture descriptor.";
    return nullptr;
  }

  auto ahb = std::make_unique<android::HardwareBuffer>(ahb_descriptor);
  if (!ahb->IsValid()) {
    VALIDATION_LOG << "Could not allocate hardware buffer.";
    return nullptr;
  }

  auto texture_source =
      std::make_shared<AHBTextureSourceVK>(context, std::move(ahb), false);

  if (!texture_source->IsValid()) {
    VALIDATION_LOG << "Could not create trampoline texture source.";
    return nullptr;
  }
  cached_texture_source_ = std::move(texture_source);
  return cached_texture_source_;
}

// |SurfaceTextureExternalTexture|
void SurfaceTextureExternalTextureVKImpeller::DrawFrame(
    PaintContext& context,
    const SkRect& bounds,
    const DlImageSampling sampling) const {
  context.canvas->DrawImage(dl_image_, SkPoint{0, 0}, sampling, context.paint);
}

}  // namespace flutter
