// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/allocator_vk.h"

#include <memory>

#include "flutter/fml/memory/ref_ptr.h"
#include "flutter/fml/trace_event.h"
#include "impeller/base/allocation_size.h"
#include "impeller/core/formats.h"
#include "impeller/renderer/backend/vulkan/capabilities_vk.h"
#include "impeller/renderer/backend/vulkan/device_buffer_vk.h"
#include "impeller/renderer/backend/vulkan/formats_vk.h"
#include "impeller/renderer/backend/vulkan/texture_vk.h"
#include "vulkan/vulkan_enums.hpp"

namespace impeller {

static constexpr vk::Flags<vk::MemoryPropertyFlagBits>
ToVKBufferMemoryPropertyFlags(StorageMode mode) {
  switch (mode) {
    case StorageMode::kHostVisible:
      return vk::MemoryPropertyFlagBits::eHostVisible;
    case StorageMode::kDevicePrivate:
      return vk::MemoryPropertyFlagBits::eDeviceLocal;
    case StorageMode::kDeviceTransient:
      return vk::MemoryPropertyFlagBits::eLazilyAllocated;
  }
  FML_UNREACHABLE();
}

static VmaAllocationCreateFlags ToVmaAllocationBufferCreateFlags(
    StorageMode mode,
    bool readback) {
  VmaAllocationCreateFlags flags = 0;
  switch (mode) {
    case StorageMode::kHostVisible:
      if (!readback) {
        flags |= VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
      } else {
        flags |= VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
      }
      flags |= VMA_ALLOCATION_CREATE_MAPPED_BIT;
      return flags;
    case StorageMode::kDevicePrivate:
      FML_DCHECK(!readback);
      return flags;
    case StorageMode::kDeviceTransient:
      FML_DCHECK(!readback);
      return flags;
  }
  FML_UNREACHABLE();
}

static PoolVMA CreateBufferPool(VmaAllocator allocator) {
  vk::BufferCreateInfo buffer_info;
  buffer_info.usage = vk::BufferUsageFlagBits::eVertexBuffer |
                      vk::BufferUsageFlagBits::eIndexBuffer |
                      vk::BufferUsageFlagBits::eUniformBuffer |
                      vk::BufferUsageFlagBits::eStorageBuffer |
                      vk::BufferUsageFlagBits::eTransferSrc |
                      vk::BufferUsageFlagBits::eTransferDst;
  buffer_info.size = 1u;  // doesn't matter
  buffer_info.sharingMode = vk::SharingMode::eExclusive;
  auto buffer_info_native =
      static_cast<vk::BufferCreateInfo::NativeType>(buffer_info);

  VmaAllocationCreateInfo allocation_info = {};
  allocation_info.usage = VMA_MEMORY_USAGE_AUTO;
  allocation_info.preferredFlags = static_cast<VkMemoryPropertyFlags>(
      ToVKBufferMemoryPropertyFlags(StorageMode::kHostVisible));
  allocation_info.flags = ToVmaAllocationBufferCreateFlags(
      StorageMode::kHostVisible, /*readback=*/false);

  uint32_t memTypeIndex;
  auto result = vk::Result{vmaFindMemoryTypeIndexForBufferInfo(
      allocator, &buffer_info_native, &allocation_info, &memTypeIndex)};
  if (result != vk::Result::eSuccess) {
    return {};
  }

  VmaPoolCreateInfo pool_create_info = {};
  pool_create_info.memoryTypeIndex = memTypeIndex;
  pool_create_info.flags = VMA_POOL_CREATE_IGNORE_BUFFER_IMAGE_GRANULARITY_BIT;
  pool_create_info.minBlockCount = 1;

  VmaPool pool = {};
  result = vk::Result{::vmaCreatePool(allocator, &pool_create_info, &pool)};
  if (result != vk::Result::eSuccess) {
    return {};
  }
  return {allocator, pool};
}

AllocatorVK::AllocatorVK(std::weak_ptr<Context> context,
                         uint32_t vulkan_api_version,
                         const vk::PhysicalDevice& physical_device,
                         const std::shared_ptr<DeviceHolderVK>& device_holder,
                         const vk::Instance& instance,
                         const CapabilitiesVK& capabilities)
    : context_(std::move(context)), device_holder_(device_holder) {
  auto limits = physical_device.getProperties().limits;
  max_texture_size_.width = max_texture_size_.height =
      limits.maxImageDimension2D;
  physical_device.getMemoryProperties(&memory_properties_);

  VmaVulkanFunctions proc_table = {};

#define BIND_VMA_PROC(x) proc_table.x = VULKAN_HPP_DEFAULT_DISPATCHER.x;
#define BIND_VMA_PROC_KHR(x)                                \
  proc_table.x##KHR = VULKAN_HPP_DEFAULT_DISPATCHER.x       \
                          ? VULKAN_HPP_DEFAULT_DISPATCHER.x \
                          : VULKAN_HPP_DEFAULT_DISPATCHER.x##KHR;
  BIND_VMA_PROC(vkGetInstanceProcAddr);
  BIND_VMA_PROC(vkGetDeviceProcAddr);
  BIND_VMA_PROC(vkGetPhysicalDeviceProperties);
  BIND_VMA_PROC(vkGetPhysicalDeviceMemoryProperties);
  BIND_VMA_PROC(vkAllocateMemory);
  BIND_VMA_PROC(vkFreeMemory);
  BIND_VMA_PROC(vkMapMemory);
  BIND_VMA_PROC(vkUnmapMemory);
  BIND_VMA_PROC(vkFlushMappedMemoryRanges);
  BIND_VMA_PROC(vkInvalidateMappedMemoryRanges);
  BIND_VMA_PROC(vkBindBufferMemory);
  BIND_VMA_PROC(vkBindImageMemory);
  BIND_VMA_PROC(vkGetBufferMemoryRequirements);
  BIND_VMA_PROC(vkGetImageMemoryRequirements);
  BIND_VMA_PROC(vkCreateBuffer);
  BIND_VMA_PROC(vkDestroyBuffer);
  BIND_VMA_PROC(vkCreateImage);
  BIND_VMA_PROC(vkDestroyImage);
  BIND_VMA_PROC(vkCmdCopyBuffer);
  BIND_VMA_PROC_KHR(vkGetBufferMemoryRequirements2);
  BIND_VMA_PROC_KHR(vkGetImageMemoryRequirements2);
  BIND_VMA_PROC_KHR(vkBindBufferMemory2);
  BIND_VMA_PROC_KHR(vkBindImageMemory2);
  BIND_VMA_PROC_KHR(vkGetPhysicalDeviceMemoryProperties2);
#undef BIND_VMA_PROC_KHR
#undef BIND_VMA_PROC

  VmaAllocatorCreateInfo allocator_info = {};
  allocator_info.vulkanApiVersion = vulkan_api_version;
  allocator_info.physicalDevice = physical_device;
  allocator_info.device = device_holder->GetDevice();
  allocator_info.instance = instance;
  // 4 MB, matching the default used by Skia Vulkan.
  allocator_info.preferredLargeHeapBlockSize = 4 * 1024 * 1024;
  allocator_info.pVulkanFunctions = &proc_table;

  VmaAllocator allocator = {};
  auto result = vk::Result{::vmaCreateAllocator(&allocator_info, &allocator)};
  if (result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not create memory allocator";
    return;
  }
  staging_buffer_pool_.reset(CreateBufferPool(allocator));
  created_buffer_pool_ &= staging_buffer_pool_.is_valid();
  allocator_.reset(allocator);
  supports_memoryless_textures_ =
      capabilities.SupportsDeviceTransientTextures();
  is_valid_ = true;
}

AllocatorVK::~AllocatorVK() = default;

// |Allocator|
bool AllocatorVK::IsValid() const {
  return is_valid_;
}

// |Allocator|
ISize AllocatorVK::GetMaxTextureSizeSupported() const {
  return max_texture_size_;
}

int32_t AllocatorVK::FindMemoryTypeIndex(
    uint32_t memory_type_bits_requirement,
    vk::PhysicalDeviceMemoryProperties& memory_properties) {
  int32_t type_index = -1;
  vk::MemoryPropertyFlagBits required_properties =
      vk::MemoryPropertyFlagBits::eDeviceLocal;

  const uint32_t memory_count = memory_properties.memoryTypeCount;
  for (uint32_t memory_index = 0; memory_index < memory_count; ++memory_index) {
    const uint32_t memory_type_bits = (1 << memory_index);
    const bool is_required_memory_type =
        memory_type_bits_requirement & memory_type_bits;

    const auto properties =
        memory_properties.memoryTypes[memory_index].propertyFlags;
    const bool has_required_properties =
        (properties & required_properties) == required_properties;

    if (is_required_memory_type && has_required_properties) {
      return static_cast<int32_t>(memory_index);
    }
  }

  return type_index;
}

vk::ImageUsageFlags AllocatorVK::ToVKImageUsageFlags(
    PixelFormat format,
    TextureUsageMask usage,
    StorageMode mode,
    bool supports_memoryless_textures) {
  vk::ImageUsageFlags vk_usage;

  switch (mode) {
    case StorageMode::kHostVisible:
    case StorageMode::kDevicePrivate:
      break;
    case StorageMode::kDeviceTransient:
      if (supports_memoryless_textures) {
        vk_usage |= vk::ImageUsageFlagBits::eTransientAttachment;
      }
      break;
  }

  if (usage & TextureUsage::kRenderTarget) {
    if (PixelFormatIsDepthStencil(format)) {
      vk_usage |= vk::ImageUsageFlagBits::eDepthStencilAttachment;
    } else {
      vk_usage |= vk::ImageUsageFlagBits::eColorAttachment;
      vk_usage |= vk::ImageUsageFlagBits::eInputAttachment;
    }
  }

  if (usage & TextureUsage::kShaderRead) {
    vk_usage |= vk::ImageUsageFlagBits::eSampled;
  }

  if (usage & TextureUsage::kShaderWrite) {
    vk_usage |= vk::ImageUsageFlagBits::eStorage;
  }

  if (mode != StorageMode::kDeviceTransient) {
    // Add transfer usage flags to support blit passes only if image isn't
    // device transient.
    vk_usage |= vk::ImageUsageFlagBits::eTransferSrc |
                vk::ImageUsageFlagBits::eTransferDst;
  }

  return vk_usage;
}

static constexpr VmaMemoryUsage ToVMAMemoryUsage() {
  return VMA_MEMORY_USAGE_AUTO;
}

static constexpr vk::Flags<vk::MemoryPropertyFlagBits>
ToVKTextureMemoryPropertyFlags(StorageMode mode,
                               bool supports_memoryless_textures) {
  switch (mode) {
    case StorageMode::kHostVisible:
      return vk::MemoryPropertyFlagBits::eHostVisible |
             vk::MemoryPropertyFlagBits::eDeviceLocal;
    case StorageMode::kDevicePrivate:
      return vk::MemoryPropertyFlagBits::eDeviceLocal;
    case StorageMode::kDeviceTransient:
      if (supports_memoryless_textures) {
        return vk::MemoryPropertyFlagBits::eLazilyAllocated |
               vk::MemoryPropertyFlagBits::eDeviceLocal;
      }
      return vk::MemoryPropertyFlagBits::eDeviceLocal;
  }
  FML_UNREACHABLE();
}

static VmaAllocationCreateFlags ToVmaAllocationCreateFlags(StorageMode mode) {
  VmaAllocationCreateFlags flags = 0;
  switch (mode) {
    case StorageMode::kHostVisible:
      return flags;
    case StorageMode::kDevicePrivate:
      return flags;
    case StorageMode::kDeviceTransient:
      return flags;
  }
  FML_UNREACHABLE();
}

class AllocatedTextureSourceVK final : public TextureSourceVK {
 public:
  AllocatedTextureSourceVK(const ContextVK& context,
                           const TextureDescriptor& desc,
                           VmaAllocator allocator,
                           vk::Device device,
                           bool supports_memoryless_textures)
      : TextureSourceVK(desc), resource_(context.GetResourceManager()) {
    FML_DCHECK(desc.format != PixelFormat::kUnknown);
    vk::StructureChain<vk::ImageCreateInfo, vk::ImageCompressionControlEXT>
        image_info_chain;
    auto& image_info = image_info_chain.get();
    image_info.flags = ToVKImageCreateFlags(desc.type);
    image_info.imageType = vk::ImageType::e2D;
    image_info.format = ToVKImageFormat(desc.format);
    image_info.extent = VkExtent3D{
        static_cast<uint32_t>(desc.size.width),   // width
        static_cast<uint32_t>(desc.size.height),  // height
        1u                                        // depth
    };
    image_info.samples = ToVKSampleCount(desc.sample_count);
    image_info.mipLevels = desc.mip_count;
    image_info.arrayLayers = ToArrayLayerCount(desc.type);
    image_info.tiling = vk::ImageTiling::eOptimal;
    image_info.initialLayout = vk::ImageLayout::eUndefined;
    image_info.usage = AllocatorVK::ToVKImageUsageFlags(
        desc.format, desc.usage, desc.storage_mode,
        supports_memoryless_textures);
    image_info.sharingMode = vk::SharingMode::eExclusive;

    vk::ImageCompressionFixedRateFlagsEXT frc_rates[1] = {
        vk::ImageCompressionFixedRateFlagBitsEXT::eNone};

    const auto frc_rate =
        CapabilitiesVK::Cast(*context.GetCapabilities())
            .GetSupportedFRCRate(desc.compression_type,
                                 FRCFormatDescriptor{image_info});
    if (frc_rate.has_value()) {
      // This array must not be in a temporary scope.
      frc_rates[0] = frc_rate.value();

      auto& compression_info =
          image_info_chain.get<vk::ImageCompressionControlEXT>();
      compression_info.pFixedRateFlags = frc_rates;
      compression_info.compressionControlPlaneCount = 1u;
      compression_info.flags =
          vk::ImageCompressionFlagBitsEXT::eFixedRateExplicit;
    } else {
      image_info_chain.unlink<vk::ImageCompressionControlEXT>();
    }

    VmaAllocationCreateInfo alloc_nfo = {};

    alloc_nfo.usage = ToVMAMemoryUsage();
    alloc_nfo.preferredFlags =
        static_cast<VkMemoryPropertyFlags>(ToVKTextureMemoryPropertyFlags(
            desc.storage_mode, supports_memoryless_textures));
    alloc_nfo.flags = ToVmaAllocationCreateFlags(desc.storage_mode);

    auto create_info_native =
        static_cast<vk::ImageCreateInfo::NativeType>(image_info);

    VkImage vk_image = VK_NULL_HANDLE;
    VmaAllocation allocation = {};
    VmaAllocationInfo allocation_info = {};
    {
      auto result = vk::Result{::vmaCreateImage(allocator,            //
                                                &create_info_native,  //
                                                &alloc_nfo,           //
                                                &vk_image,            //
                                                &allocation,          //
                                                &allocation_info      //
                                                )};
      if (result != vk::Result::eSuccess) {
        VALIDATION_LOG << "Unable to allocate Vulkan Image: "
                       << vk::to_string(result)
                       << " Type: " << TextureTypeToString(desc.type)
                       << " Mode: " << StorageModeToString(desc.storage_mode)
                       << " Usage: " << TextureUsageMaskToString(desc.usage)
                       << " [VK]Flags: " << vk::to_string(image_info.flags)
                       << " [VK]Format: " << vk::to_string(image_info.format)
                       << " [VK]Usage: " << vk::to_string(image_info.usage)
                       << " [VK]Mem. Flags: "
                       << vk::to_string(vk::MemoryPropertyFlags(
                              alloc_nfo.preferredFlags));
        return;
      }
    }

    auto image = vk::Image{vk_image};

    vk::ImageViewCreateInfo view_info = {};
    view_info.image = image;
    view_info.viewType = ToVKImageViewType(desc.type);
    view_info.format = image_info.format;
    view_info.subresourceRange.aspectMask = ToVKImageAspectFlags(desc.format);
    view_info.subresourceRange.levelCount = image_info.mipLevels;
    view_info.subresourceRange.layerCount = ToArrayLayerCount(desc.type);

    // Vulkan does not have an image format that is equivalent to
    // `MTLPixelFormatA8Unorm`, so we use `R8Unorm` instead. Given that the
    // shaders expect that alpha channel to be set in the cases, we swizzle.
    // See: https://github.com/flutter/flutter/issues/115461 for more details.
    if (desc.format == PixelFormat::kA8UNormInt) {
      view_info.components.a = vk::ComponentSwizzle::eR;
      view_info.components.r = vk::ComponentSwizzle::eA;
    }

    auto [result, image_view] = device.createImageViewUnique(view_info);
    if (result != vk::Result::eSuccess) {
      VALIDATION_LOG << "Unable to create an image view for allocation: "
                     << vk::to_string(result);
      return;
    }
    // Create a specialized view for render target attachments.
    view_info.subresourceRange.levelCount = 1u;
    auto [rt_result, rt_image_view] = device.createImageViewUnique(view_info);
    if (rt_result != vk::Result::eSuccess) {
      VALIDATION_LOG << "Unable to create an image view for allocation: "
                     << vk::to_string(rt_result);
      return;
    }

    resource_.Swap(ImageResource(ImageVMA{allocator, allocation, image},
                                 std::move(image_view),
                                 std::move(rt_image_view)));
    is_valid_ = true;
  }

  ~AllocatedTextureSourceVK() = default;

  bool IsValid() const { return is_valid_; }

  vk::Image GetImage() const override { return resource_->image.get().image; }

  vk::ImageView GetImageView() const override {
    return resource_->image_view.get();
  }

  vk::ImageView GetRenderTargetView() const override {
    return resource_->rt_image_view.get();
  }

  bool IsSwapchainImage() const override { return false; }

 private:
  struct ImageResource {
    UniqueImageVMA image;
    vk::UniqueImageView image_view;
    vk::UniqueImageView rt_image_view;

    ImageResource() = default;

    ImageResource(ImageVMA p_image,
                  vk::UniqueImageView p_image_view,
                  vk::UniqueImageView p_rt_image_view)
        : image(p_image),
          image_view(std::move(p_image_view)),
          rt_image_view(std::move(p_rt_image_view)) {}

    ImageResource(ImageResource&& o) = default;

    ImageResource(const ImageResource&) = delete;

    ImageResource& operator=(const ImageResource&) = delete;
  };

  UniqueResourceVKT<ImageResource> resource_;
  bool is_valid_ = false;

  AllocatedTextureSourceVK(const AllocatedTextureSourceVK&) = delete;

  AllocatedTextureSourceVK& operator=(const AllocatedTextureSourceVK&) = delete;
};

// |Allocator|
std::shared_ptr<Texture> AllocatorVK::OnCreateTexture(
    const TextureDescriptor& desc) {
  if (!IsValid()) {
    return nullptr;
  }
  auto device_holder = device_holder_.lock();
  if (!device_holder) {
    return nullptr;
  }
  auto context = context_.lock();
  if (!context) {
    return nullptr;
  }
  auto source = std::make_shared<AllocatedTextureSourceVK>(
      ContextVK::Cast(*context),     //
      desc,                          //
      allocator_.get(),              //
      device_holder->GetDevice(),    //
      supports_memoryless_textures_  //
  );
  if (!source->IsValid()) {
    return nullptr;
  }
  return std::make_shared<TextureVK>(context_, std::move(source));
}

// |Allocator|
std::shared_ptr<DeviceBuffer> AllocatorVK::OnCreateBuffer(
    const DeviceBufferDescriptor& desc) {
  vk::BufferCreateInfo buffer_info;
  buffer_info.usage = vk::BufferUsageFlagBits::eVertexBuffer |
                      vk::BufferUsageFlagBits::eIndexBuffer |
                      vk::BufferUsageFlagBits::eUniformBuffer |
                      vk::BufferUsageFlagBits::eStorageBuffer |
                      vk::BufferUsageFlagBits::eTransferSrc |
                      vk::BufferUsageFlagBits::eTransferDst;
  buffer_info.size = desc.size;
  buffer_info.sharingMode = vk::SharingMode::eExclusive;
  auto buffer_info_native =
      static_cast<vk::BufferCreateInfo::NativeType>(buffer_info);

  VmaAllocationCreateInfo allocation_info = {};
  allocation_info.usage = ToVMAMemoryUsage();
  allocation_info.preferredFlags = static_cast<VkMemoryPropertyFlags>(
      ToVKBufferMemoryPropertyFlags(desc.storage_mode));
  allocation_info.flags =
      ToVmaAllocationBufferCreateFlags(desc.storage_mode, desc.readback);
  if (created_buffer_pool_ && desc.storage_mode == StorageMode::kHostVisible &&
      !desc.readback) {
    allocation_info.pool = staging_buffer_pool_.get().pool;
  }
  VkBuffer buffer = {};
  VmaAllocation buffer_allocation = {};
  VmaAllocationInfo buffer_allocation_info = {};
  auto result = vk::Result{::vmaCreateBuffer(allocator_.get(),        //
                                             &buffer_info_native,     //
                                             &allocation_info,        //
                                             &buffer,                 //
                                             &buffer_allocation,      //
                                             &buffer_allocation_info  //
                                             )};

  auto type = memory_properties_.memoryTypes[buffer_allocation_info.memoryType];
  bool is_host_coherent =
      !!(type.propertyFlags & vk::MemoryPropertyFlagBits::eHostCoherent);

  if (result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Unable to allocate a device buffer: "
                   << vk::to_string(result);
    return {};
  }

  return std::make_shared<DeviceBufferVK>(
      desc,                                            //
      context_,                                        //
      UniqueBufferVMA{BufferVMA{allocator_.get(),      //
                                buffer_allocation,     //
                                vk::Buffer{buffer}}},  //
      buffer_allocation_info,                          //
      is_host_coherent);
}

Bytes AllocatorVK::DebugGetHeapUsage() const {
  auto count = memory_properties_.memoryHeapCount;
  std::vector<VmaBudget> budgets(count);
  vmaGetHeapBudgets(allocator_.get(), budgets.data());
  size_t total_usage = 0;
  for (auto i = 0u; i < count; i++) {
    const VmaBudget& budget = budgets[i];
    total_usage += budget.usage;
  }
  return Bytes{static_cast<double>(total_usage)};
}

void AllocatorVK::DebugTraceMemoryStatistics() const {
#ifdef IMPELLER_DEBUG
  FML_TRACE_COUNTER("flutter", "AllocatorVK",
                    reinterpret_cast<int64_t>(this),  // Trace Counter ID
                    "MemoryBudgetUsageMB",
                    DebugGetHeapUsage().ConvertTo<MebiBytes>().GetSize());
#endif  // IMPELLER_DEBUG
}

}  // namespace impeller
