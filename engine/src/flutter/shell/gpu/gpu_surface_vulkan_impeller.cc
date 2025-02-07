// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/gpu/gpu_surface_vulkan_impeller.h"

#include <memory>

#include "flow/surface_frame.h"
#include "flutter/fml/make_copyable.h"
#include "fml/trace_event.h"
#include "impeller/core/formats.h"
#include "impeller/core/texture_descriptor.h"
#include "impeller/display_list/dl_dispatcher.h"
#include "impeller/renderer/backend/vulkan/command_buffer_vk.h"
#include "impeller/renderer/backend/vulkan/context_vk.h"
#include "impeller/renderer/backend/vulkan/surface_context_vk.h"
#include "impeller/renderer/backend/vulkan/swapchain/surface_vk.h"
#include "impeller/renderer/render_target.h"
#include "impeller/renderer/surface.h"
#include "impeller/typographer/backends/skia/typographer_context_skia.h"

namespace flutter {

class WrappedTextureSourceVK : public impeller::TextureSourceVK {
 public:
  explicit WrappedTextureSourceVK(impeller::vk::Image image,
                                  impeller::vk::ImageView image_view,
                                  impeller::TextureDescriptor desc)
      : TextureSourceVK(desc), image_(image), image_view_(image_view) {}

  ~WrappedTextureSourceVK() {}

 private:
  impeller::vk::Image GetImage() const override { return image_; }

  impeller::vk::ImageView GetImageView() const override { return image_view_; }

  impeller::vk::ImageView GetRenderTargetView() const override {
    return image_view_;
  }

  bool IsSwapchainImage() const override { return true; }

  impeller::vk::Image image_;
  impeller::vk::ImageView image_view_;
};

GPUSurfaceVulkanImpeller::GPUSurfaceVulkanImpeller(
    GPUSurfaceVulkanDelegate* delegate,
    std::shared_ptr<impeller::Context> context)
    : delegate_(delegate) {
  if (!context || !context->IsValid()) {
    return;
  }

  auto aiks_context = std::make_shared<impeller::AiksContext>(
      context, impeller::TypographerContextSkia::Make());
  if (!aiks_context->IsValid()) {
    return;
  }

  impeller_context_ = std::move(context);
  aiks_context_ = std::move(aiks_context);
  is_valid_ = !!aiks_context_;
}

// |Surface|
GPUSurfaceVulkanImpeller::~GPUSurfaceVulkanImpeller() = default;

// |Surface|
bool GPUSurfaceVulkanImpeller::IsValid() {
  return is_valid_;
}

// |Surface|
std::unique_ptr<SurfaceFrame> GPUSurfaceVulkanImpeller::AcquireFrame(
    const SkISize& size) {
  if (!IsValid()) {
    FML_LOG(ERROR) << "Vulkan surface was invalid.";
    return nullptr;
  }

  if (size.isEmpty()) {
    FML_LOG(ERROR) << "Vulkan surface was asked for an empty frame.";
    return nullptr;
  }

  if (delegate_ == nullptr) {
    auto& context_vk = impeller::SurfaceContextVK::Cast(*impeller_context_);
    std::unique_ptr<impeller::Surface> surface =
        context_vk.AcquireNextSurface();

    if (!surface) {
      FML_LOG(ERROR) << "No surface available.";
      return nullptr;
    }

    impeller::RenderTarget render_target = surface->GetRenderTarget();
    auto cull_rect = render_target.GetRenderTargetSize();

    SurfaceFrame::EncodeCallback encode_callback = [aiks_context =
                                                        aiks_context_,  //
                                                    render_target,
                                                    cull_rect  //
    ](SurfaceFrame& surface_frame, DlCanvas* canvas) mutable -> bool {
      if (!aiks_context) {
        return false;
      }

      auto display_list = surface_frame.BuildDisplayList();
      if (!display_list) {
        FML_LOG(ERROR) << "Could not build display list for surface frame.";
        return false;
      }

      SkIRect sk_cull_rect = SkIRect::MakeWH(cull_rect.width, cull_rect.height);
      return impeller::RenderToTarget(
          aiks_context->GetContentContext(),                                //
          render_target,                                                    //
          display_list,                                                     //
          sk_cull_rect,                                                     //
          /*reset_host_buffer=*/surface_frame.submit_info().frame_boundary  //
      );
    };

    return std::make_unique<SurfaceFrame>(
        nullptr,                          // surface
        SurfaceFrame::FramebufferInfo{},  // framebuffer info
        encode_callback,                  // encode callback
        fml::MakeCopyable([surface = std::move(surface)](const SurfaceFrame&) {
          return surface->Present();
        }),       // submit callback
        size,     // frame size
        nullptr,  // context result
        true      // display list fallback
    );
  } else {
    FlutterVulkanImage flutter_image = delegate_->AcquireImage(size);
    if (!flutter_image.image) {
      FML_LOG(ERROR) << "Invalid VkImage given by the embedder.";
      return nullptr;
    }
    impeller::vk::Format vk_format =
        static_cast<impeller::vk::Format>(flutter_image.format);
    std::optional<impeller::PixelFormat> format =
        impeller::VkFormatToImpellerFormat(vk_format);
    if (!format.has_value()) {
      FML_LOG(ERROR) << "Unsupported pixel format: "
                     << impeller::vk::to_string(vk_format);
      return nullptr;
    }

    impeller::vk::Image vk_image =
        impeller::vk::Image(reinterpret_cast<VkImage>(flutter_image.image));

    impeller::TextureDescriptor desc;
    desc.format = format.value();
    desc.size = impeller::ISize{size.width(), size.height()};
    desc.storage_mode = impeller::StorageMode::kDevicePrivate;
    desc.mip_count = 1;
    desc.compression_type = impeller::CompressionType::kLossless;
    desc.usage = impeller::TextureUsage::kRenderTarget;

    impeller::ContextVK& context_vk =
        impeller::ContextVK::Cast(*impeller_context_);

    impeller::vk::ImageViewCreateInfo view_info = {};
    view_info.viewType = impeller::vk::ImageViewType::e2D;
    view_info.format = ToVKImageFormat(desc.format);
    view_info.subresourceRange.aspectMask =
        impeller::vk::ImageAspectFlagBits::eColor;
    view_info.subresourceRange.baseMipLevel = 0u;
    view_info.subresourceRange.baseArrayLayer = 0u;
    view_info.subresourceRange.levelCount = 1;
    view_info.subresourceRange.layerCount = 1;
    view_info.image = vk_image;

    auto [result, image_view] =
        context_vk.GetDevice().createImageView(view_info);
    if (result != impeller::vk::Result::eSuccess) {
      FML_LOG(ERROR) << "Failed to create image view for provided image: "
                     << impeller::vk::to_string(result);
      return nullptr;
    }

    if (transients_ == nullptr) {
      transients_ = std::make_shared<impeller::SwapchainTransientsVK>(
          impeller_context_, desc,
          /*enable_msaa=*/true);
    }

    auto wrapped_onscreen =
        std::make_shared<WrappedTextureSourceVK>(vk_image, image_view, desc);
    auto surface = impeller::SurfaceVK::WrapSwapchainImage(
        transients_, wrapped_onscreen, [&]() -> bool { return true; });
    impeller::RenderTarget render_target = surface->GetRenderTarget();
    auto cull_rect = render_target.GetRenderTargetSize();

    SurfaceFrame::EncodeCallback encode_callback = [aiks_context =
                                                        aiks_context_,  //
                                                    render_target,
                                                    cull_rect  //
    ](SurfaceFrame& surface_frame, DlCanvas* canvas) mutable -> bool {
      if (!aiks_context) {
        return false;
      }

      auto display_list = surface_frame.BuildDisplayList();
      if (!display_list) {
        FML_LOG(ERROR) << "Could not build display list for surface frame.";
        return false;
      }

      SkIRect sk_cull_rect = SkIRect::MakeWH(cull_rect.width, cull_rect.height);
      return impeller::RenderToTarget(aiks_context->GetContentContext(),  //
                                      render_target,                      //
                                      display_list,                       //
                                      sk_cull_rect,                       //
                                      /*reset_host_buffer=*/true          //
      );
    };

    SurfaceFrame::SubmitCallback submit_callback =
        [image = flutter_image, delegate = delegate_,
         impeller_context = impeller_context_,
         wrapped_onscreen](const SurfaceFrame&) -> bool {
      TRACE_EVENT0("flutter", "GPUSurfaceVulkan::PresentImage");

      {
        const auto& context = impeller::ContextVK::Cast(*impeller_context);

        //----------------------------------------------------------------------------
        /// Transition the image to color-attachment-optimal.
        ///
        auto cmd_buffer = context.CreateCommandBuffer();

        auto vk_final_cmd_buffer =
            impeller::CommandBufferVK::Cast(*cmd_buffer).GetCommandBuffer();
        {
          impeller::BarrierVK barrier;
          barrier.new_layout =
              impeller::vk::ImageLayout::eColorAttachmentOptimal;
          barrier.cmd_buffer = vk_final_cmd_buffer;
          barrier.src_access =
              impeller::vk::AccessFlagBits::eColorAttachmentWrite;
          barrier.src_stage =
              impeller::vk::PipelineStageFlagBits::eColorAttachmentOutput;
          barrier.dst_access = {};
          barrier.dst_stage =
              impeller::vk::PipelineStageFlagBits::eBottomOfPipe;

          if (!wrapped_onscreen->SetLayout(barrier).ok()) {
            return false;
          }
        }
        if (!context.GetCommandQueue()->Submit({cmd_buffer}).ok()) {
          return false;
        }
      }

      return delegate->PresentImage(reinterpret_cast<VkImage>(image.image),
                                    static_cast<VkFormat>(image.format));
    };

    SurfaceFrame::FramebufferInfo framebuffer_info{.supports_readback = true};

    return std::make_unique<SurfaceFrame>(nullptr,           // surface
                                          framebuffer_info,  // framebuffer info
                                          encode_callback,   // encode callback
                                          submit_callback,
                                          size,     // frame size
                                          nullptr,  // context result
                                          true      // display list fallback
    );
  }
}

// |Surface|
SkMatrix GPUSurfaceVulkanImpeller::GetRootTransformation() const {
  // This backend does not currently support root surface transformations. Just
  // return identity.
  return {};
}

// |Surface|
GrDirectContext* GPUSurfaceVulkanImpeller::GetContext() {
  // Impeller != Skia.
  return nullptr;
}

// |Surface|
std::unique_ptr<GLContextResult>
GPUSurfaceVulkanImpeller::MakeRenderContextCurrent() {
  // This backend has no such concept.
  return std::make_unique<GLContextDefaultResult>(true);
}

// |Surface|
bool GPUSurfaceVulkanImpeller::EnableRasterCache() const {
  return false;
}

// |Surface|
std::shared_ptr<impeller::AiksContext>
GPUSurfaceVulkanImpeller::GetAiksContext() const {
  return aiks_context_;
}

}  // namespace flutter
