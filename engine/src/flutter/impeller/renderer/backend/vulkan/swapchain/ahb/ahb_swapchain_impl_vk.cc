// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/swapchain/ahb/ahb_swapchain_impl_vk.h"

#include "impeller/base/validation.h"
#include "impeller/renderer/backend/vulkan/command_buffer_vk.h"
#include "impeller/renderer/backend/vulkan/swapchain/ahb/ahb_formats.h"
#include "impeller/renderer/backend/vulkan/swapchain/ahb/external_semaphore_vk.h"
#include "impeller/renderer/backend/vulkan/swapchain/surface_vk.h"
#include "impeller/toolkit/android/surface_transaction.h"
#include "impeller/toolkit/android/surface_transaction_stats.h"

namespace impeller {

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

AHBFrameSynchronizerVK::AHBFrameSynchronizerVK(const vk::Device& device) {
  auto acquire_res = device.createFenceUnique(
      vk::FenceCreateInfo{vk::FenceCreateFlagBits::eSignaled});
  if (acquire_res.result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not create synchronizer.";
    return;
  }
  acquire = std::move(acquire_res.value);
  is_valid = true;
}

AHBFrameSynchronizerVK::~AHBFrameSynchronizerVK() = default;

bool AHBFrameSynchronizerVK::IsValid() const {
  return is_valid;
}

bool AHBFrameSynchronizerVK::WaitForFence(const vk::Device& device) {
  if (auto result = device.waitForFences(
          *acquire,                             // fence
          true,                                 // wait all
          std::numeric_limits<uint64_t>::max()  // timeout (ns)
      );
      result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Fence wait failed: " << vk::to_string(result);
    return false;
  }
  if (auto result = device.resetFences(*acquire);
      result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not reset fence: " << vk::to_string(result);
    return false;
  }
  return true;
}

std::shared_ptr<AHBSwapchainImplVK> AHBSwapchainImplVK::Create(
    const std::weak_ptr<Context>& context,
    std::weak_ptr<android::SurfaceControl> surface_control,
    const CreateTransactionCB& cb,
    const ISize& size,
    bool enable_msaa,
    size_t swapchain_image_count) {
  auto impl = std::shared_ptr<AHBSwapchainImplVK>(
      new AHBSwapchainImplVK(context, std::move(surface_control), cb, size,
                             enable_msaa, swapchain_image_count));
  return impl->IsValid() ? impl : nullptr;
}

AHBSwapchainImplVK::AHBSwapchainImplVK(
    const std::weak_ptr<Context>& context,
    std::weak_ptr<android::SurfaceControl> surface_control,
    const CreateTransactionCB& cb,
    const ISize& size,
    bool enable_msaa,
    size_t swapchain_image_count)
    : surface_control_(std::move(surface_control)), cb_(cb) {
  desc_ = android::HardwareBufferDescriptor::MakeForSwapchainImage(size);
  pool_ =
      std::make_shared<AHBTexturePoolVK>(context, desc_, swapchain_image_count);
  if (!pool_->IsValid()) {
    return;
  }
  transients_ = std::make_shared<SwapchainTransientsVK>(
      context, ToSwapchainTextureDescriptor(desc_), enable_msaa);

  for (auto i = 0u; i < kMaxPendingPresents; i++) {
    auto sync = std::make_unique<AHBFrameSynchronizerVK>(
        ContextVK::Cast(*context.lock()).GetDeviceHolder()->GetDevice());
    if (!sync->IsValid()) {
      return;
    }
    frame_data_.push_back(std::move(sync));
  }

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
  auto context = transients_->GetContext().lock();
  if (!context) {
    return nullptr;
  }

  frame_index_ = (frame_index_ + 1) % kMaxPendingPresents;

  if (!frame_data_[frame_index_]->WaitForFence(
          ContextVK::Cast(*context).GetDevice())) {
    return nullptr;
  }

  if (!is_valid_) {
    return nullptr;
  }

  auto pool_entry = pool_->Pop();

  if (!pool_entry.IsValid()) {
    VALIDATION_LOG << "Could not create AHB texture source.";
    return nullptr;
  }

  // Import the render ready semaphore that will block onscreen rendering until
  // it is ready.
  if (!ImportRenderReady(pool_entry.render_ready_fence, pool_entry.texture)) {
    VALIDATION_LOG << "Could wait on render ready fence.";
    return nullptr;
  }

#if IMPELLER_DEBUG
  if (context) {
    ContextVK::Cast(*context).GetGPUTracer()->MarkFrameStart();
  }
#endif  // IMPELLER_DEBUG

  auto surface = SurfaceVK::WrapSwapchainImage(
      transients_, pool_entry.texture,
      [weak = weak_from_this(), texture = pool_entry.texture]() {
        auto thiz = weak.lock();
        if (!thiz) {
          VALIDATION_LOG << "Swapchain died before image could be presented.";
          return false;
        }
        return thiz->Present(texture);
      });

  if (!surface) {
    return nullptr;
  }

  return surface;
}

bool AHBSwapchainImplVK::Present(
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

  auto present_ready = SubmitSignalForPresentReady(texture);

  if (!present_ready) {
    VALIDATION_LOG << "Could not submit completion signal.";
    return false;
  }

  android::SurfaceTransaction transaction = cb_();
  if (!transaction.SetContents(control.get(),               //
                               texture->GetBackingStore(),  //
                               present_ready->CreateFD()    //
                               )) {
    VALIDATION_LOG << "Could not set swapchain image contents on the surface "
                      "control.";
    return false;
  }
  return transaction.Apply(
      [texture, weak = weak_from_this()](ASurfaceTransactionStats* stats) {
        auto thiz = weak.lock();
        if (!thiz) {
          return;
        }
        thiz->OnTextureUpdatedOnSurfaceControl(texture, stats);
      });
}

void AHBSwapchainImplVK::AddFinalCommandBuffer(
    std::shared_ptr<CommandBuffer> cmd_buffer) {
  frame_data_[frame_index_]->final_cmd_buffer = std::move(cmd_buffer);
}

std::shared_ptr<ExternalSemaphoreVK>
AHBSwapchainImplVK::SubmitSignalForPresentReady(
    const std::shared_ptr<AHBTextureSourceVK>& texture) const {
  auto context = transients_->GetContext().lock();
  if (!context) {
    return nullptr;
  }

  auto present_ready = std::make_shared<ExternalSemaphoreVK>(context);
  if (!present_ready || !present_ready->IsValid()) {
    return nullptr;
  }

  auto& sync = frame_data_[frame_index_];
  auto command_buffer = sync->final_cmd_buffer;
  if (!command_buffer) {
    return nullptr;
  }
  CommandBufferVK& command_buffer_vk = CommandBufferVK::Cast(*command_buffer);
  const auto command_encoder_vk = command_buffer_vk.GetCommandBuffer();
  if (!command_buffer_vk.EndCommandBuffer()) {
    return nullptr;
  }
  sync->present_ready = present_ready;

  vk::SubmitInfo submit_info;
  vk::PipelineStageFlags wait_stage =
      vk::PipelineStageFlagBits::eColorAttachmentOutput;
  if (sync->render_ready) {
    submit_info.setPWaitSemaphores(&sync->render_ready.get());
    submit_info.setWaitSemaphoreCount(1);
    submit_info.setWaitDstStageMask(wait_stage);
  }
  submit_info.setCommandBuffers(command_encoder_vk);
  submit_info.setPSignalSemaphores(&sync->present_ready->GetHandle());
  submit_info.setSignalSemaphoreCount(1);

  auto result = ContextVK::Cast(*context).GetGraphicsQueue()->Submit(
      submit_info, *sync->acquire);
  if (result != vk::Result::eSuccess) {
    return nullptr;
  }
  return present_ready;
}

vk::UniqueSemaphore AHBSwapchainImplVK::CreateRenderReadySemaphore(
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

  auto signal_wait = device.createSemaphoreUnique({});
  if (signal_wait.result != vk::Result::eSuccess) {
    return {};
  }

  context_vk.SetDebugName(*signal_wait.value, "AHBRenderReadySemaphore");

  vk::ImportSemaphoreFdInfoKHR import_info;
  import_info.semaphore = *signal_wait.value;
  import_info.fd = fd->get();
  import_info.handleType = vk::ExternalSemaphoreHandleTypeFlagBits::eSyncFd;
  // From the spec: Sync FDs can only be imported temporarily.
  import_info.flags = vk::SemaphoreImportFlagBitsKHR::eTemporary;

  const auto import_result = device.importSemaphoreFdKHR(import_info);

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

bool AHBSwapchainImplVK::ImportRenderReady(
    const std::shared_ptr<fml::UniqueFD>& render_ready_fence,
    const std::shared_ptr<AHBTextureSourceVK>& texture) {
  auto context = transients_->GetContext().lock();
  if (!context) {
    return false;
  }

  // If there is no render ready fence, we are already ready to render into
  // the texture. There is nothing more to do.
  if (!render_ready_fence || !render_ready_fence->is_valid()) {
    frame_data_[frame_index_]->render_ready = {};
    return true;
  }

  auto semaphore = CreateRenderReadySemaphore(render_ready_fence);
  if (!semaphore) {
    return false;
  }
  // This semaphore will be later used to block the onscreen render pass
  // from starting until the system is done reading the onscreen.
  frame_data_[frame_index_]->render_ready = std::move(semaphore);
  return true;
}

void AHBSwapchainImplVK::OnTextureUpdatedOnSurfaceControl(
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
