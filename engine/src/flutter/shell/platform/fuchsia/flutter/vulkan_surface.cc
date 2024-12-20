// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vulkan_surface.h"

#include <fuchsia/sysmem/cpp/fidl.h>
#include <lib/async/default.h>

#include <algorithm>

#include "flutter/fml/logging.h"
#include "flutter/fml/trace_event.h"
#include "runtime/dart/utils/inlines.h"
#include "third_party/skia/include/core/SkCanvas.h"
#include "third_party/skia/include/core/SkColorSpace.h"
#include "third_party/skia/include/core/SkSurfaceProps.h"
#include "third_party/skia/include/gpu/ganesh/GrBackendSemaphore.h"
#include "third_party/skia/include/gpu/ganesh/GrBackendSurface.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"
#include "third_party/skia/include/gpu/ganesh/SkSurfaceGanesh.h"
#include "third_party/skia/include/gpu/ganesh/vk/GrVkBackendSurface.h"
#include "third_party/skia/include/gpu/ganesh/vk/GrVkTypes.h"
#include "third_party/skia/include/gpu/vk/VulkanTypes.h"
#include "vulkan/vulkan_core.h"
#include "vulkan/vulkan_fuchsia.h"

#define LOG_AND_RETURN(cond, msg) \
  if (cond) {                     \
    FML_LOG(ERROR) << msg;        \
    return false;                 \
  }

namespace flutter_runner {

namespace {

constexpr SkColorType kSkiaColorType = kRGBA_8888_SkColorType;
constexpr VkFormat kVulkanFormat = VK_FORMAT_R8G8B8A8_UNORM;
constexpr VkImageCreateFlags kVulkanImageCreateFlags = 0;
// TODO: We should only keep usages that are actually required by Skia.
constexpr VkImageUsageFlags kVkImageUsage =
    VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT |
    VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
const VkSysmemColorSpaceFUCHSIA kSrgbColorSpace = {
    .sType = VK_STRUCTURE_TYPE_SYSMEM_COLOR_SPACE_FUCHSIA,
    .pNext = nullptr,
    .colorSpace = static_cast<uint32_t>(fuchsia::images2::ColorSpace::SRGB),
};

}  // namespace

bool VulkanSurface::CreateVulkanImage(vulkan::VulkanProvider& vulkan_provider,
                                      const SkISize& size,
                                      VulkanImage* out_vulkan_image) {
  TRACE_EVENT0("flutter", "CreateVulkanImage");

  FML_CHECK(!size.isEmpty());
  FML_CHECK(out_vulkan_image != nullptr);

  out_vulkan_image->vk_collection_image_create_info = {
      .sType = VK_STRUCTURE_TYPE_BUFFER_COLLECTION_IMAGE_CREATE_INFO_FUCHSIA,
      .pNext = nullptr,
      .collection = collection_,
      .index = 0,
  };

  out_vulkan_image->vk_image_create_info = {
      .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
      .pNext = &out_vulkan_image->vk_collection_image_create_info,
      .flags = kVulkanImageCreateFlags,
      .imageType = VK_IMAGE_TYPE_2D,
      .format = kVulkanFormat,
      .extent = VkExtent3D{static_cast<uint32_t>(size.width()),
                           static_cast<uint32_t>(size.height()), 1},
      .mipLevels = 1,
      .arrayLayers = 1,
      .samples = VK_SAMPLE_COUNT_1_BIT,
      .tiling = VK_IMAGE_TILING_OPTIMAL,
      .usage = kVkImageUsage,
      .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
      .queueFamilyIndexCount = 0,
      .pQueueFamilyIndices = nullptr,
      .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
  };

  const VkImageFormatConstraintsInfoFUCHSIA format_constraints = {
      .sType = VK_STRUCTURE_TYPE_IMAGE_FORMAT_CONSTRAINTS_INFO_FUCHSIA,
      .pNext = nullptr,
      .imageCreateInfo = out_vulkan_image->vk_image_create_info,
      .requiredFormatFeatures = VK_FORMAT_FEATURE_COLOR_ATTACHMENT_BIT,
      .flags = {},
      .sysmemPixelFormat = {},
      .colorSpaceCount = 1,
      .pColorSpaces = &kSrgbColorSpace,
  };

  const VkImageConstraintsInfoFUCHSIA image_constraints = {
      .sType = VK_STRUCTURE_TYPE_IMAGE_CONSTRAINTS_INFO_FUCHSIA,
      .pNext = nullptr,
      .formatConstraintsCount = 1,
      .pFormatConstraints = &format_constraints,
      .bufferCollectionConstraints =
          {
              .sType =
                  VK_STRUCTURE_TYPE_BUFFER_COLLECTION_CONSTRAINTS_INFO_FUCHSIA,
              .pNext = nullptr,
              .minBufferCount = 1,
              // Using the default value 0 means that we don't have any other
              // constraints except for the minimum buffer count.
              .maxBufferCount = 0,
              .minBufferCountForCamping = 0,
              .minBufferCountForDedicatedSlack = 0,
              .minBufferCountForSharedSlack = 0,
          },
      .flags = {},
  };

  if (VK_CALL_LOG_ERROR(
          vulkan_provider.vk().SetBufferCollectionImageConstraintsFUCHSIA(
              vulkan_provider.vk_device(), collection_, &image_constraints)) !=
      VK_SUCCESS) {
    return false;
  }

  {
    VkImage vk_image = VK_NULL_HANDLE;
    if (VK_CALL_LOG_ERROR(vulkan_provider.vk().CreateImage(
            vulkan_provider.vk_device(),
            &out_vulkan_image->vk_image_create_info, nullptr, &vk_image)) !=
        VK_SUCCESS) {
      return false;
    }

    out_vulkan_image->vk_image = vulkan::VulkanHandle<VkImage_T*>{
        vk_image, [&vulkan_provider = vulkan_provider](VkImage image) {
          vulkan_provider.vk().DestroyImage(vulkan_provider.vk_device(), image,
                                            NULL);
        }};
  }

  vulkan_provider.vk().GetImageMemoryRequirements(
      vulkan_provider.vk_device(), out_vulkan_image->vk_image,
      &out_vulkan_image->vk_memory_requirements);

  return true;
}

VulkanSurface::VulkanSurface(
    vulkan::VulkanProvider& vulkan_provider,
    fuchsia::sysmem2::AllocatorSyncPtr& sysmem_allocator,
    fuchsia::ui::composition::AllocatorPtr& flatland_allocator,
    sk_sp<GrDirectContext> context,
    const SkISize& size)
    : vulkan_provider_(vulkan_provider), wait_(this) {
  FML_CHECK(flatland_allocator.is_bound());
  FML_CHECK(context != nullptr);

  if (!AllocateDeviceMemory(sysmem_allocator, flatland_allocator,
                            std::move(context), size)) {
    FML_LOG(ERROR) << "VulkanSurface: Could not allocate device memory.";
    return;
  }

  if (!CreateFences()) {
    FML_LOG(ERROR) << "VulkanSurface: Could not create signal fences.";
    return;
  }

  std::fill(size_history_.begin(), size_history_.end(), SkISize::MakeEmpty());

  wait_.set_object(release_event_.get());
  wait_.set_trigger(ZX_EVENT_SIGNALED);
  Reset();

  valid_ = true;
}

VulkanSurface::~VulkanSurface() {
  if (release_image_callback_) {
    release_image_callback_();
  }
  wait_.Cancel();
  wait_.set_object(ZX_HANDLE_INVALID);
}

bool VulkanSurface::IsValid() const {
  return valid_;
}

SkISize VulkanSurface::GetSize() const {
  if (!valid_) {
    return SkISize::Make(0, 0);
  }

  return SkISize::Make(sk_surface_->width(), sk_surface_->height());
}

vulkan::VulkanHandle<VkSemaphore> VulkanSurface::SemaphoreFromEvent(
    const zx::event& event) const {
  VkResult result;
  VkSemaphore semaphore;

  zx::event semaphore_event;
  zx_status_t status = event.duplicate(ZX_RIGHT_SAME_RIGHTS, &semaphore_event);
  if (status != ZX_OK) {
    FML_LOG(ERROR) << "VulkanSurface: Failed to duplicate semaphore event";
    return vulkan::VulkanHandle<VkSemaphore>();
  }

  VkSemaphoreCreateInfo create_info = {
      .sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO,
      .pNext = nullptr,
      .flags = 0,
  };

  result = VK_CALL_LOG_ERROR(vulkan_provider_.vk().CreateSemaphore(
      vulkan_provider_.vk_device(), &create_info, nullptr, &semaphore));
  if (result != VK_SUCCESS) {
    return vulkan::VulkanHandle<VkSemaphore>();
  }

  VkImportSemaphoreZirconHandleInfoFUCHSIA import_info = {
      .sType = VK_STRUCTURE_TYPE_IMPORT_SEMAPHORE_ZIRCON_HANDLE_INFO_FUCHSIA,
      .pNext = nullptr,
      .semaphore = semaphore,
      .handleType = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_ZIRCON_EVENT_BIT_FUCHSIA,
      .zirconHandle = static_cast<uint32_t>(semaphore_event.release())};

  result = VK_CALL_LOG_ERROR(
      vulkan_provider_.vk().ImportSemaphoreZirconHandleFUCHSIA(
          vulkan_provider_.vk_device(), &import_info));
  if (result != VK_SUCCESS) {
    return vulkan::VulkanHandle<VkSemaphore>();
  }

  return vulkan::VulkanHandle<VkSemaphore>(
      semaphore, [&vulkan_provider = vulkan_provider_](VkSemaphore semaphore) {
        vulkan_provider.vk().DestroySemaphore(vulkan_provider.vk_device(),
                                              semaphore, nullptr);
      });
}

bool VulkanSurface::CreateFences() {
  if (zx::event::create(0, &acquire_event_) != ZX_OK) {
    FML_LOG(ERROR) << "VulkanSurface: Failed to create acquire event";
    return false;
  }

  acquire_semaphore_ = SemaphoreFromEvent(acquire_event_);
  if (!acquire_semaphore_) {
    FML_LOG(ERROR) << "VulkanSurface: Failed to create acquire semaphore";
    return false;
  }

  if (zx::event::create(0, &release_event_) != ZX_OK) {
    FML_LOG(ERROR) << "VulkanSurface: Failed to create release event";
    return false;
  }

  command_buffer_fence_ = vulkan_provider_.CreateFence();

  return true;
}

bool VulkanSurface::AllocateDeviceMemory(
    fuchsia::sysmem2::AllocatorSyncPtr& sysmem_allocator,
    fuchsia::ui::composition::AllocatorPtr& flatland_allocator,
    sk_sp<GrDirectContext> context,
    const SkISize& size) {
  if (size.isEmpty()) {
    FML_LOG(ERROR)
        << "VulkanSurface: Failed to allocate surface, size is empty";
    return false;
  }

  fuchsia::sysmem2::BufferCollectionTokenSyncPtr vulkan_token;
  zx_status_t status = sysmem_allocator->AllocateSharedCollection(
      std::move(fuchsia::sysmem2::AllocatorAllocateSharedCollectionRequest{}
                    .set_token_request(vulkan_token.NewRequest())));
  LOG_AND_RETURN(status != ZX_OK,
                 "VulkanSurface: Failed to allocate collection");
  fuchsia::sysmem2::BufferCollectionTokenSyncPtr scenic_token;
  status = vulkan_token->Duplicate(
      std::move(fuchsia::sysmem2::BufferCollectionTokenDuplicateRequest{}
                    .set_rights_attenuation_mask(ZX_RIGHT_SAME_RIGHTS)
                    .set_token_request(scenic_token.NewRequest())));
  LOG_AND_RETURN(status != ZX_OK,
                 "VulkanSurface: Failed to duplicate collection token");
  fuchsia::sysmem2::Node_Sync_Result sync_result;
  status = vulkan_token->Sync(&sync_result);
  LOG_AND_RETURN(status != ZX_OK,
                 "VulkanSurface: Failed to sync collection token");

  fuchsia::ui::composition::BufferCollectionExportToken export_token;
  status = zx::eventpair::create(0, &export_token.value, &import_token_.value);

  fuchsia::ui::composition::RegisterBufferCollectionArgs args;
  args.set_export_token(std::move(export_token));
  args.set_buffer_collection_token(fuchsia::sysmem::BufferCollectionTokenHandle(
      scenic_token.Unbind().TakeChannel()));
  args.set_usage(
      fuchsia::ui::composition::RegisterBufferCollectionUsage::DEFAULT);
  flatland_allocator->RegisterBufferCollection(
      std::move(args),
      [](fuchsia::ui::composition::Allocator_RegisterBufferCollection_Result
             result) {
        if (result.is_err()) {
          FML_LOG(ERROR)
              << "RegisterBufferCollection call to Scenic Allocator failed";
        }
      });

  VkBufferCollectionCreateInfoFUCHSIA import_info{
      .sType = VK_STRUCTURE_TYPE_BUFFER_COLLECTION_CREATE_INFO_FUCHSIA,
      .pNext = nullptr,
      .collectionToken = vulkan_token.Unbind().TakeChannel().release(),
  };
  VkBufferCollectionFUCHSIA collection;
  if (VK_CALL_LOG_ERROR(vulkan_provider_.vk().CreateBufferCollectionFUCHSIA(
          vulkan_provider_.vk_device(), &import_info, nullptr, &collection)) !=
      VK_SUCCESS) {
    return false;
  }

  collection_ = vulkan::VulkanHandle<VkBufferCollectionFUCHSIA_T*>{
      collection, [&vulkan_provider =
                       vulkan_provider_](VkBufferCollectionFUCHSIA collection) {
        vulkan_provider.vk().DestroyBufferCollectionFUCHSIA(
            vulkan_provider.vk_device(), collection, nullptr);
      }};

  VulkanImage vulkan_image;
  LOG_AND_RETURN(!CreateVulkanImage(vulkan_provider_, size, &vulkan_image),
                 "VulkanSurface: Failed to create VkImage");

  vulkan_image_ = std::move(vulkan_image);
  const VkMemoryRequirements& memory_requirements =
      vulkan_image_.vk_memory_requirements;
  VkImageCreateInfo& image_create_info = vulkan_image_.vk_image_create_info;

  VkBufferCollectionPropertiesFUCHSIA properties{
      .sType = VK_STRUCTURE_TYPE_BUFFER_COLLECTION_PROPERTIES_FUCHSIA};
  if (VK_CALL_LOG_ERROR(
          vulkan_provider_.vk().GetBufferCollectionPropertiesFUCHSIA(
              vulkan_provider_.vk_device(), collection_, &properties)) !=
      VK_SUCCESS) {
    return false;
  }

  VkImportMemoryBufferCollectionFUCHSIA import_memory_info = {
      .sType = VK_STRUCTURE_TYPE_IMPORT_MEMORY_BUFFER_COLLECTION_FUCHSIA,
      .pNext = nullptr,
      .collection = collection_,
      .index = 0,
  };
  auto bits = memory_requirements.memoryTypeBits & properties.memoryTypeBits;
  FML_DCHECK(bits != 0);
  VkMemoryAllocateInfo allocation_info = {
      .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
      .pNext = &import_memory_info,
      .allocationSize = memory_requirements.size,
      .memoryTypeIndex = static_cast<uint32_t>(__builtin_ctz(bits)),
  };

  {
    TRACE_EVENT1("flutter", "vkAllocateMemory", "allocationSize",
                 allocation_info.allocationSize);
    VkDeviceMemory vk_memory = VK_NULL_HANDLE;
    if (VK_CALL_LOG_ERROR(vulkan_provider_.vk().AllocateMemory(
            vulkan_provider_.vk_device(), &allocation_info, NULL,
            &vk_memory)) != VK_SUCCESS) {
      return false;
    }

    vk_memory_ = vulkan::VulkanHandle<VkDeviceMemory_T*>{
        vk_memory,
        [&vulkan_provider = vulkan_provider_](VkDeviceMemory memory) {
          vulkan_provider.vk().FreeMemory(vulkan_provider.vk_device(), memory,
                                          NULL);
        }};

    vk_memory_info_ = allocation_info;
  }

  // Bind image memory.
  if (VK_CALL_LOG_ERROR(vulkan_provider_.vk().BindImageMemory(
          vulkan_provider_.vk_device(), vulkan_image_.vk_image, vk_memory_,
          0)) != VK_SUCCESS) {
    return false;
  }

  return SetupSkiaSurface(std::move(context), size, kSkiaColorType,
                          image_create_info, memory_requirements);
}

bool VulkanSurface::SetupSkiaSurface(sk_sp<GrDirectContext> context,
                                     const SkISize& size,
                                     SkColorType color_type,
                                     const VkImageCreateInfo& image_create_info,
                                     const VkMemoryRequirements& memory_reqs) {
  FML_CHECK(context != nullptr);

  skgpu::VulkanAlloc alloc;
  alloc.fMemory = vk_memory_;
  alloc.fOffset = 0;
  alloc.fSize = memory_reqs.size;
  alloc.fFlags = 0;

  GrVkImageInfo image_info;
  image_info.fImage = vulkan_image_.vk_image;
  image_info.fAlloc = alloc;
  image_info.fImageTiling = image_create_info.tiling;
  image_info.fImageLayout = image_create_info.initialLayout;
  image_info.fFormat = image_create_info.format;
  image_info.fImageUsageFlags = image_create_info.usage;
  image_info.fSampleCount = 1;
  image_info.fLevelCount = image_create_info.mipLevels;

  auto sk_render_target =
      GrBackendRenderTargets::MakeVk(size.width(), size.height(), image_info);

  SkSurfaceProps sk_surface_props(0, kUnknown_SkPixelGeometry);

  auto sk_surface =
      SkSurfaces::WrapBackendRenderTarget(context.get(),             //
                                          sk_render_target,          //
                                          kTopLeft_GrSurfaceOrigin,  //
                                          color_type,                //
                                          SkColorSpace::MakeSRGB(),  //
                                          &sk_surface_props          //
      );

  if (!sk_surface || sk_surface->getCanvas() == nullptr) {
    FML_LOG(ERROR)
        << "VulkanSurface: SkSurfaces::WrapBackendRenderTarget failed";
    return false;
  }
  sk_surface_ = std::move(sk_surface);

  return true;
}

void VulkanSurface::SetImageId(uint32_t image_id) {
  FML_CHECK(image_id_ == 0);
  image_id_ = image_id;
}

uint32_t VulkanSurface::GetImageId() {
  return image_id_;
}

sk_sp<SkSurface> VulkanSurface::GetSkiaSurface() const {
  return valid_ ? sk_surface_ : nullptr;
}

fuchsia::ui::composition::BufferCollectionImportToken
VulkanSurface::GetBufferCollectionImportToken() {
  fuchsia::ui::composition::BufferCollectionImportToken import_dup;
  import_token_.value.duplicate(ZX_RIGHT_SAME_RIGHTS, &import_dup.value);
  return import_dup;
}

zx::event VulkanSurface::GetAcquireFence() {
  zx::event fence;
  acquire_event_.duplicate(ZX_RIGHT_SAME_RIGHTS, &fence);
  return fence;
}

zx::event VulkanSurface::GetReleaseFence() {
  zx::event fence;
  release_event_.duplicate(ZX_RIGHT_SAME_RIGHTS, &fence);
  return fence;
}
void VulkanSurface::SetReleaseImageCallback(
    ReleaseImageCallback release_image_callback) {
  release_image_callback_ = release_image_callback;
}

size_t VulkanSurface::AdvanceAndGetAge() {
  size_history_[size_history_index_] = GetSize();
  size_history_index_ = (size_history_index_ + 1) % kSizeHistorySize;
  age_++;
  return age_;
}

bool VulkanSurface::FlushSessionAcquireAndReleaseEvents() {
  age_ = 0;
  return true;
}

void VulkanSurface::SignalWritesFinished(
    const std::function<void(void)>& on_writes_committed) {
  FML_CHECK(on_writes_committed);

  if (!valid_) {
    on_writes_committed();
    return;
  }

  FML_CHECK(pending_on_writes_committed_ == nullptr)
      << "Attempted to signal a write on the surface when the "
         "previous write has not yet been acknowledged by the "
         "compositor.";

  pending_on_writes_committed_ = on_writes_committed;
}

void VulkanSurface::Reset() {
  if (acquire_event_.signal(ZX_EVENT_SIGNALED, 0u) != ZX_OK ||
      release_event_.signal(ZX_EVENT_SIGNALED, 0u) != ZX_OK) {
    valid_ = false;
    FML_LOG(ERROR) << "Could not reset fences. The surface is no longer valid.";
  }

  VkFence fence = command_buffer_fence_;

  if (command_buffer_) {
    VK_CALL_LOG_FATAL(vulkan_provider_.vk().WaitForFences(
        vulkan_provider_.vk_device(), 1, &fence, VK_TRUE, UINT64_MAX));
    command_buffer_.reset();
  }

  VK_CALL_LOG_ERROR(vulkan_provider_.vk().ResetFences(
      vulkan_provider_.vk_device(), 1, &fence));

  // Need to make a new  acquire semaphore every frame or else validation layers
  // get confused about why no one is waiting on it in this VkInstance
  acquire_semaphore_.Reset();
  acquire_semaphore_ = SemaphoreFromEvent(acquire_event_);
  if (!acquire_semaphore_) {
    FML_LOG(ERROR) << "failed to create acquire semaphore";
  }

  wait_.Begin(async_get_default_dispatcher());

  // It is safe for the caller to collect the surface in the callback.
  auto callback = pending_on_writes_committed_;
  pending_on_writes_committed_ = nullptr;
  if (callback) {
    callback();
  }
}

void VulkanSurface::OnHandleReady(async_dispatcher_t* dispatcher,
                                  async::WaitBase* wait,
                                  zx_status_t status,
                                  const zx_packet_signal_t* signal) {
  if (status != ZX_OK)
    return;
  FML_DCHECK(signal->observed & ZX_EVENT_SIGNALED);
  Reset();
}

}  // namespace flutter_runner
