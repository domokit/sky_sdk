// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/swapchain/ahb/ahb_swapchain_impl_vk.h"

#include "flutter/fml/trace_event.h"
#include "impeller/base/validation.h"
#include "impeller/renderer/backend/vulkan/barrier_vk.h"
#include "impeller/renderer/backend/vulkan/command_buffer_vk.h"
#include "impeller/renderer/backend/vulkan/fence_waiter_vk.h"
#include "impeller/renderer/backend/vulkan/gpu_tracer_vk.h"
#include "impeller/renderer/backend/vulkan/swapchain/ahb/ahb_formats.h"
#include "impeller/renderer/backend/vulkan/swapchain/surface_vk.h"
#include "impeller/toolkit/android/surface_transaction.h"
#include "impeller/toolkit/android/surface_transaction_stats.h"
#include "vulkan/vulkan_to_string.hpp"

namespace impeller {

//------------------------------------------------------------------------------
/// The maximum number of presents pending in the compositor after which the
/// acquire calls will block. This value is 2 images given to the system
/// compositor and one for the raster thread, Because the semaphore is acquired
/// when the CPU begins working on the texture
///
static constexpr const size_t kMaxPendingPresents = 3u;

static TextureDescriptor ToSwapchainTextureDescriptor(
    const android::HardwareBufferDescriptor& ahb_desc) {
  TextureDescriptor desc;
  desc.storage_mode = StorageMode::kDevicePrivate;
  desc.type = TextureType::kTexture2D;
  desc.format = ToPixelFormat(ahb_desc.format);
  desc.size = ahb_desc.size;
  desc.mip_count = 1u;
  desc.usage = TextureUsage::kRenderTarget;
  desc.sample_count = SampleCount::kCount1;
  desc.compression_type = CompressionType::kLossless;
  return desc;
}

std::shared_ptr<AHBSwapchainImplVK> AHBSwapchainImplVK::Create(
    const std::weak_ptr<Context>& context,
    std::weak_ptr<android::SurfaceControl> surface_control,
    const ISize& size,
    bool enable_msaa,
    size_t swapchain_image_count) {
  auto impl = std::shared_ptr<AHBSwapchainImplVK>(
      new AHBSwapchainImplVK(context, std::move(surface_control), size,
                             enable_msaa, swapchain_image_count));
  return impl->IsValid() ? impl : nullptr;
}

AHBSwapchainImplVK::AHBSwapchainImplVK(
    const std::weak_ptr<Context>& context,
    std::weak_ptr<android::SurfaceControl> surface_control,
    const ISize& size,
    bool enable_msaa,
    size_t swapchain_image_count)
    : surface_control_(std::move(surface_control)),
      pending_presents_(std::make_shared<fml::Semaphore>(kMaxPendingPresents)) {
  desc_ = android::HardwareBufferDescriptor::MakeForSwapchainImage(size);
  pool_ =
      std::make_shared<AHBTexturePoolVK>(context, desc_, swapchain_image_count);
  if (!pool_->IsValid()) {
    return;
  }
  transients_ = std::make_shared<SwapchainTransientsVK>(
      context, ToSwapchainTextureDescriptor(desc_), enable_msaa);

  auto control = surface_control_.lock();
  is_valid_ = control && control->IsValid();
}

AHBSwapchainImplVK::~AHBSwapchainImplVK() = default;

const ISize& AHBSwapchainImplVK::GetSize() const {
  return desc_.size;
}

bool AHBSwapchainImplVK::IsValid() const {
  return is_valid_;
}

const android::HardwareBufferDescriptor& AHBSwapchainImplVK::GetDescriptor()
    const {
  return desc_;
}

std::unique_ptr<Surface> AHBSwapchainImplVK::AcquireNextDrawable() {
  {
    TRACE_EVENT0("impeller", "CompositorPendingWait");
    if (!pending_presents_->Wait()) {
      return nullptr;
    }
  }

  AutoSemaSignaler auto_sema_signaler =
      std::make_shared<fml::ScopedCleanupClosure>(
          [sema = pending_presents_]() { sema->Signal(); });

  if (!is_valid_) {
    return nullptr;
  }

  auto pool_entry = pool_->Pop();

  if (!pool_entry.IsValid()) {
    VALIDATION_LOG << "Could not create AHB texture source.";
    return nullptr;
  }

  // Ask the GPU to wait for the render ready semaphore to be signaled before
  // performing rendering operations.
  if (!SubmitWaitForRenderReady(pool_entry.render_ready_fence,
                                pool_entry.texture)) {
    VALIDATION_LOG << "Could wait on render ready fence.";
    return nullptr;
  }

#if IMPELLER_DEBUG
  auto context = transients_->GetContext().lock();
  if (context) {
    ContextVK::Cast(*context).GetGPUTracer()->MarkFrameStart();
  }
#endif  // IMPELLER_DEBUG

  auto surface = SurfaceVK::WrapSwapchainImage(
      transients_, pool_entry.texture,
      [signaler = auto_sema_signaler, weak = weak_from_this(),
       texture = pool_entry.texture]() {
        auto thiz = weak.lock();
        if (!thiz) {
          VALIDATION_LOG << "Swapchain died before image could be presented.";
          return false;
        }
        return thiz->Present(signaler, texture);
      });

  if (!surface) {
    return nullptr;
  }

  return surface;
}

bool AHBSwapchainImplVK::Present(
    const AutoSemaSignaler& signaler,
    const std::shared_ptr<AHBTextureSourceVK>& texture) {
  auto control = surface_control_.lock();
  if (!control || !control->IsValid()) {
    VALIDATION_LOG << "Surface control died before swapchain image could be "
                      "presented.";
    return false;
  }

#if IMPELLER_DEBUG
  auto context = transients_->GetContext().lock();
  if (context) {
    ContextVK::Cast(*context).GetGPUTracer()->MarkFrameEnd();
  }
#endif  // IMPELLER_DEBUG

  if (!texture) {
    return false;
  }

  auto fence = SubmitSignalForPresentReady(texture);

  if (!fence) {
    VALIDATION_LOG << "Could not submit completion signal.";
    return false;
  }

  android::SurfaceTransaction transaction;
  if (!transaction.SetContents(control.get(),               //
                               texture->GetBackingStore(),  //
                               fence->CreateFD()            //
                               )) {
    VALIDATION_LOG << "Could not set swapchain image contents on the surface "
                      "control.";
    return false;
  }
  return transaction.Apply([signaler, texture, weak = weak_from_this()](
                               ASurfaceTransactionStats* stats) {
    auto thiz = weak.lock();
    if (!thiz) {
      return;
    }
    thiz->OnTextureUpdatedOnSurfaceControl(signaler, texture, stats);
  });
}

std::shared_ptr<ExternalFenceVK>
AHBSwapchainImplVK::SubmitSignalForPresentReady(
    const std::shared_ptr<AHBTextureSourceVK>& texture) const {
  auto context = transients_->GetContext().lock();
  if (!context) {
    return nullptr;
  }
  auto fence = std::make_shared<ExternalFenceVK>(context);
  if (!fence || !fence->IsValid()) {
    return nullptr;
  }

  auto command_buffer = context->CreateCommandBuffer();
  if (!command_buffer) {
    return nullptr;
  }
  command_buffer->SetLabel("AHBSubmitSignalForPresentReadyCB");
  CommandBufferVK& command_buffer_vk = CommandBufferVK::Cast(*command_buffer);

  const auto command_encoder_vk = command_buffer_vk.GetCommandBuffer();

  BarrierVK barrier;
  barrier.cmd_buffer = command_encoder_vk;
  barrier.new_layout = vk::ImageLayout::eGeneral;
  barrier.src_stage = vk::PipelineStageFlagBits::eColorAttachmentOutput;
  barrier.src_access = vk::AccessFlagBits::eColorAttachmentWrite;
  barrier.dst_stage = vk::PipelineStageFlagBits::eBottomOfPipe;
  barrier.dst_access = {};

  if (!texture->SetLayout(barrier).ok()) {
    return nullptr;
  }

  command_buffer_vk.Track(fence->GetSharedHandle());

  if (!command_buffer_vk.EndCommandBuffer()) {
    return nullptr;
  }

  vk::SubmitInfo submit_info;
  submit_info.setCommandBuffers(command_encoder_vk);

  auto result = ContextVK::Cast(*context).GetGraphicsQueue()->Submit(
      submit_info, fence->GetHandle());
  if (result != vk::Result::eSuccess) {
    return nullptr;
  }
  return fence;
}

vk::UniqueFence AHBSwapchainImplVK::CreateRenderReadyFence(
    const std::shared_ptr<fml::UniqueFD>& fd) const {
  if (!fd->is_valid()) {
    return {};
  }

  auto context = transients_->GetContext().lock();
  if (!context) {
    return {};
  }

  const auto& context_vk = ContextVK::Cast(*context);
  const auto& device = context_vk.GetDevice();

  auto signal_wait = device.createFenceUnique({});

  if (signal_wait.result != vk::Result::eSuccess) {
    return {};
  }

  context_vk.SetDebugName(*signal_wait.value, "AHBRenderReadyFence");

  vk::ImportFenceFdInfoKHR import_info;
  import_info.fence = *signal_wait.value;
  import_info.fd = fd->get();
  import_info.handleType = vk::ExternalFenceHandleTypeFlagBits::eSyncFd;
  // From the spec: Sync FDs can only be imported temporarily.
  import_info.flags = vk::FenceImportFlagBitsKHR::eTemporary;

  const auto import_result = device.importFenceFdKHR(import_info);

  if (import_result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not import semaphore FD: "
                   << vk::to_string(import_result);
    return {};
  }

  // From the spec: Importing a semaphore payload from a file descriptor
  // transfers ownership of the file descriptor from the application to the
  // Vulkan implementation. The application must not perform any operations on
  // the file descriptor after a successful import.
  [[maybe_unused]] auto released = fd->release();

  return std::move(signal_wait.value);
}

bool AHBSwapchainImplVK::SubmitWaitForRenderReady(
    const std::shared_ptr<fml::UniqueFD>& render_ready_fence,
    const std::shared_ptr<AHBTextureSourceVK>& texture) const {
  // If there is no render ready fence, we are already ready to render into
  // the texture. There is nothing more to do.
  if (!render_ready_fence || !render_ready_fence->is_valid()) {
    return true;
  }

  auto context = transients_->GetContext().lock();
  if (!context) {
    return false;
  }

  auto fence = CreateRenderReadyFence(render_ready_fence);

  auto result = ContextVK::Cast(*context).GetDevice().waitForFences(
      *fence,                               // fence
      true,                                 // wait all
      std::numeric_limits<uint64_t>::max()  // timeout (ns)
  );

  if (!(result == vk::Result::eSuccess || result == vk::Result::eTimeout)) {
    VALIDATION_LOG << "Encountered error while waiting on swapchain image: "
                   << vk::to_string(result);
    return false;
  }

  return true;
}

void AHBSwapchainImplVK::OnTextureUpdatedOnSurfaceControl(
    const AutoSemaSignaler& signaler,
    std::shared_ptr<AHBTextureSourceVK> texture,
    ASurfaceTransactionStats* stats) {
  auto control = surface_control_.lock();
  if (!control) {
    return;
  }

  // Ask for an FD that gets signaled when the previous buffer is released. This
  // can be invalid if there is no wait necessary.
  auto render_ready_fence =
      android::CreatePreviousReleaseFence(*control, stats);

  // The transaction completion indicates that the surface control now
  // references the hardware buffer. We can recycle the previous set buffer
  // safely.
  Lock lock(currently_displayed_texture_mutex_);
  auto old_texture = currently_displayed_texture_;
  currently_displayed_texture_ = std::move(texture);
  pool_->Push(std::move(old_texture), std::move(render_ready_fence));
}

}  // namespace impeller
