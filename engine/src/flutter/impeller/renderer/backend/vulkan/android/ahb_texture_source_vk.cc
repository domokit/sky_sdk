// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/android/ahb_texture_source_vk.h"

#include "impeller/renderer/backend/vulkan/allocator_vk.h"
#include "impeller/renderer/backend/vulkan/context_vk.h"
#include "impeller/renderer/backend/vulkan/texture_source_vk.h"
#include "impeller/renderer/backend/vulkan/yuv_conversion_library_vk.h"

namespace impeller {

using AHBProperties = vk::StructureChain<
    // For VK_ANDROID_external_memory_android_hardware_buffer
    vk::AndroidHardwareBufferPropertiesANDROID,
    // For VK_ANDROID_external_memory_android_hardware_buffer
    vk::AndroidHardwareBufferFormatPropertiesANDROID>;

static vk::UniqueImage CreateVKImageWrapperForAndroidHarwareBuffer(
    const vk::Device& device,
    const AHBProperties& ahb_props,
    const AHardwareBuffer_Desc& ahb_desc) {
  const auto& ahb_format =
      ahb_props.get<vk::AndroidHardwareBufferFormatPropertiesANDROID>();

  vk::StructureChain<vk::ImageCreateInfo,
                     // For VK_KHR_external_memory
                     vk::ExternalMemoryImageCreateInfo,
                     // For VK_ANDROID_external_memory_android_hardware_buffer
                     vk::ExternalFormatANDROID>
      image_chain;

  auto& image_info = image_chain.get<vk::ImageCreateInfo>();

  vk::ImageUsageFlags image_usage_flags;
  if (ahb_desc.usage & AHARDWAREBUFFER_USAGE_GPU_SAMPLED_IMAGE) {
    image_usage_flags |= vk::ImageUsageFlagBits::eSampled;
  }
  if (ahb_desc.usage & AHARDWAREBUFFER_USAGE_GPU_FRAMEBUFFER) {
    image_usage_flags |= vk::ImageUsageFlagBits::eColorAttachment;
  }
  if (ahb_desc.usage & AHARDWAREBUFFER_USAGE_COMPOSER_OVERLAY) {
    image_usage_flags |= vk::ImageUsageFlagBits::eColorAttachment;
    image_usage_flags |= vk::ImageUsageFlagBits::eInputAttachment;
    image_usage_flags |= vk::ImageUsageFlagBits::eTransferDst;
  }

  vk::ImageCreateFlags image_create_flags;
  if (ahb_desc.usage & AHARDWAREBUFFER_USAGE_PROTECTED_CONTENT) {
    image_create_flags |= vk::ImageCreateFlagBits::eProtected;
  }
  if (ahb_desc.usage & AHARDWAREBUFFER_USAGE_GPU_CUBE_MAP) {
    image_create_flags |= vk::ImageCreateFlagBits::eCubeCompatible;
  }

  image_info.imageType = vk::ImageType::e2D;
  image_info.format = ahb_format.format;
  image_info.extent.width = ahb_desc.width;
  image_info.extent.height = ahb_desc.height;
  image_info.extent.depth = 1;
  image_info.mipLevels =
      (ahb_desc.usage & AHARDWAREBUFFER_USAGE_GPU_MIPMAP_COMPLETE)
          ? ISize{ahb_desc.width, ahb_desc.height}.MipCount()
          : 1u;
  image_info.arrayLayers = ahb_desc.layers;
  image_info.samples = vk::SampleCountFlagBits::e1;
  image_info.tiling = vk::ImageTiling::eOptimal;
  image_info.usage = image_usage_flags;
  image_info.flags = image_create_flags;
  image_info.sharingMode = vk::SharingMode::eExclusive;
  image_info.initialLayout = vk::ImageLayout::eUndefined;

  image_chain.get<vk::ExternalMemoryImageCreateInfo>().handleTypes =
      vk::ExternalMemoryHandleTypeFlagBits::eAndroidHardwareBufferANDROID;

  // If the format isn't natively supported by Vulkan (i.e, be a part of the
  // base vkFormat enum), an untyped "external format" must be specified when
  // creating the image and the image views. Usually includes YUV formats.
  if (ahb_format.format == vk::Format::eUndefined) {
    image_chain.get<vk::ExternalFormatANDROID>().externalFormat =
        ahb_format.externalFormat;
  } else {
    image_chain.unlink<vk::ExternalFormatANDROID>();
  }

  auto image = device.createImageUnique(image_chain.get());
  if (image.result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not create image for external buffer: "
                   << vk::to_string(image.result);
    return {};
  }

  return std::move(image.value);
}

static vk::UniqueDeviceMemory ImportVKDeviceMemoryFromAndroidHarwareBuffer(
    const vk::Device& device,
    const vk::PhysicalDevice& physical_device,
    const vk::Image& image,
    struct AHardwareBuffer* hardware_buffer,
    const AHBProperties& ahb_props) {
  vk::PhysicalDeviceMemoryProperties memory_properties;
  physical_device.getMemoryProperties(&memory_properties);
  int memory_type_index = AllocatorVK::FindMemoryTypeIndex(
      ahb_props.get().memoryTypeBits, memory_properties);
  if (memory_type_index < 0) {
    VALIDATION_LOG << "Could not find memory type of external image.";
    return {};
  }

  vk::StructureChain<vk::MemoryAllocateInfo,
                     // Core in 1.1
                     vk::MemoryDedicatedAllocateInfo,
                     // For VK_ANDROID_external_memory_android_hardware_buffer
                     vk::ImportAndroidHardwareBufferInfoANDROID>
      memory_chain;

  auto& mem_alloc_info = memory_chain.get<vk::MemoryAllocateInfo>();
  mem_alloc_info.allocationSize = ahb_props.get().allocationSize;
  mem_alloc_info.memoryTypeIndex = memory_type_index;

  auto& dedicated_alloc_info =
      memory_chain.get<vk::MemoryDedicatedAllocateInfo>();
  dedicated_alloc_info.image = image;

  auto& ahb_import_info =
      memory_chain.get<vk::ImportAndroidHardwareBufferInfoANDROID>();
  ahb_import_info.buffer = hardware_buffer;

  auto device_memory = device.allocateMemoryUnique(memory_chain.get());
  if (device_memory.result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not allocate device memory for external image : "
                   << vk::to_string(device_memory.result);
    return {};
  }

  return std::move(device_memory.value);
}

static std::shared_ptr<YUVConversionVK> CreateYUVConversion(
    const ContextVK& context,
    const AHBProperties& ahb_props) {
  YUVConversionDescriptorVK conversion_chain;

  const auto& ahb_format =
      ahb_props.get<vk::AndroidHardwareBufferFormatPropertiesANDROID>();

  const bool supports_linear_filtering =
      !!(ahb_format.formatFeatures &
         vk::FormatFeatureFlagBits::eSampledImageYcbcrConversionLinearFilter);

  auto& conversion_info = conversion_chain.get();

  conversion_info.format = ahb_format.format;
  conversion_info.ycbcrModel = ahb_format.suggestedYcbcrModel;
  conversion_info.ycbcrRange = ahb_format.suggestedYcbcrRange;
  conversion_info.components = ahb_format.samplerYcbcrConversionComponents;
  conversion_info.xChromaOffset = ahb_format.suggestedXChromaOffset;
  conversion_info.yChromaOffset = ahb_format.suggestedYChromaOffset;
  conversion_info.chromaFilter =
      supports_linear_filtering ? vk::Filter::eLinear : vk::Filter::eNearest;
  conversion_info.forceExplicitReconstruction = false;

  if (conversion_info.format == vk::Format::eUndefined) {
    auto& external_format = conversion_chain.get<vk::ExternalFormatANDROID>();
    external_format.externalFormat = ahb_format.externalFormat;
  } else {
    conversion_chain.unlink<vk::ExternalFormatANDROID>();
  }

  return context.GetYUVConversionLibrary()->GetConversion(conversion_chain);
}

static vk::UniqueImageView CreateVKImageView(
    const vk::Device& device,
    const vk::Image& image,
    const vk::SamplerYcbcrConversion& yuv_conversion,
    const AHBProperties& ahb_props,
    const AHardwareBuffer_Desc& ahb_desc) {
  const auto& ahb_format =
      ahb_props.get<vk::AndroidHardwareBufferFormatPropertiesANDROID>();

  vk::StructureChain<vk::ImageViewCreateInfo,
                     // Core in 1.1
                     vk::SamplerYcbcrConversionInfo>
      view_chain;

  auto& view_info = view_chain.get();

  view_info.image = image;
  view_info.viewType = vk::ImageViewType::e2D;
  view_info.format = ahb_format.format;
  view_info.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
  view_info.subresourceRange.baseMipLevel = 0u;
  view_info.subresourceRange.baseArrayLayer = 0u;
  view_info.subresourceRange.levelCount =
      (ahb_desc.usage & AHARDWAREBUFFER_USAGE_GPU_MIPMAP_COMPLETE)
          ? ISize{ahb_desc.width, ahb_desc.height}.MipCount()
          : 1u;
  view_info.subresourceRange.layerCount = ahb_desc.layers;

  // We need a custom YUV conversion only if we don't recognize the format.
  if (view_info.format == vk::Format::eUndefined) {
    view_chain.get<vk::SamplerYcbcrConversionInfo>().conversion =
        yuv_conversion;
  } else {
    view_chain.unlink<vk::SamplerYcbcrConversionInfo>();
  }

  auto image_view = device.createImageViewUnique(view_info);
  if (image_view.result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not create external image view: "
                   << vk::to_string(image_view.result);
    return {};
  }

  return std::move(image_view.value);
}

static PixelFormat ToPixelFormat(AHardwareBuffer_Format format) {
  switch (format) {
    case AHARDWAREBUFFER_FORMAT_R8G8B8A8_UNORM:
      return PixelFormat::kR8G8B8A8UNormInt;
    case AHARDWAREBUFFER_FORMAT_R16G16B16A16_FLOAT:
      return PixelFormat::kR16G16B16A16Float;
    case AHARDWAREBUFFER_FORMAT_D24_UNORM_S8_UINT:
      return PixelFormat::kD24UnormS8Uint;
    case AHARDWAREBUFFER_FORMAT_D32_FLOAT_S8_UINT:
      return PixelFormat::kD32FloatS8UInt;
    case AHARDWAREBUFFER_FORMAT_S8_UINT:
      return PixelFormat::kS8UInt;
    case AHARDWAREBUFFER_FORMAT_R8_UNORM:
      return PixelFormat::kR8UNormInt;
    case AHARDWAREBUFFER_FORMAT_R10G10B10A10_UNORM:
    case AHARDWAREBUFFER_FORMAT_R16G16_UINT:
    case AHARDWAREBUFFER_FORMAT_D32_FLOAT:
    case AHARDWAREBUFFER_FORMAT_R16_UINT:
    case AHARDWAREBUFFER_FORMAT_D24_UNORM:
    case AHARDWAREBUFFER_FORMAT_Y8Cb8Cr8_420:
    case AHARDWAREBUFFER_FORMAT_YCbCr_P010:
    case AHARDWAREBUFFER_FORMAT_BLOB:
    case AHARDWAREBUFFER_FORMAT_R10G10B10A2_UNORM:
    case AHARDWAREBUFFER_FORMAT_R5G6B5_UNORM:
    case AHARDWAREBUFFER_FORMAT_R8G8B8_UNORM:
    case AHARDWAREBUFFER_FORMAT_D16_UNORM:
    case AHARDWAREBUFFER_FORMAT_R8G8B8X8_UNORM:
      // Not understood by the rest of Impeller. Use a placeholder but create
      // the native image and image views using the right external format.
      break;
  }
  return PixelFormat::kR8G8B8A8UNormInt;
}

static TextureType ToTextureType(const AHardwareBuffer_Desc& ahb_desc) {
  if (ahb_desc.layers == 1u) {
    return TextureType::kTexture2D;
  }
  if (ahb_desc.layers % 6u == 0 &&
      (ahb_desc.usage & AHARDWAREBUFFER_USAGE_GPU_CUBE_MAP)) {
    return TextureType::kTextureCube;
  }
  // Our texture types seem to understand external OES textures. Should these be
  // wired up instead?
  return TextureType::kTexture2D;
}

static TextureDescriptor ToTextureDescriptor(
    const AHardwareBuffer_Desc& ahb_desc) {
  const auto ahb_size = ISize{ahb_desc.width, ahb_desc.height};
  TextureDescriptor desc;
  // We are not going to touch hardware buffers on the CPU or use them as
  // transient attachments. Just treat them as device private.
  desc.storage_mode = StorageMode::kDevicePrivate;
  desc.format =
      ToPixelFormat(static_cast<AHardwareBuffer_Format>(ahb_desc.format));
  desc.size = ahb_size;
  desc.type = ToTextureType(ahb_desc);
  desc.sample_count = SampleCount::kCount1;
  desc.compression_type = CompressionType::kLossless;
  desc.mip_count = (ahb_desc.usage & AHARDWAREBUFFER_USAGE_GPU_MIPMAP_COMPLETE)
                       ? ahb_size.MipCount()
                       : 1u;
  if (ahb_desc.usage & AHARDWAREBUFFER_USAGE_COMPOSER_OVERLAY) {
    desc.usage = TextureUsage::kRenderTarget;
  }
  return desc;
}

AHBTextureSourceVK::AHBTextureSourceVK(
    const std::shared_ptr<Context>& p_context,
    struct AHardwareBuffer* ahb,
    const AHardwareBuffer_Desc& ahb_desc)
    : TextureSourceVK(ToTextureDescriptor(ahb_desc)) {
  if (!p_context) {
    return;
  }

  const auto& context = ContextVK::Cast(*p_context);

  const auto& device = context.GetDevice();
  const auto& physical_device = context.GetPhysicalDevice();

  AHBProperties ahb_props;

  if (device.getAndroidHardwareBufferPropertiesANDROID(ahb, &ahb_props.get()) !=
      vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not determine properties of the Android hardware "
                      "buffer.";
    return;
  }

  const auto& ahb_format =
      ahb_props.get<vk::AndroidHardwareBufferFormatPropertiesANDROID>();

  // Create an image to refer to our external image.
  auto image =
      CreateVKImageWrapperForAndroidHarwareBuffer(device, ahb_props, ahb_desc);
  if (!image) {
    return;
  }

  // Create a device memory allocation to refer to our external image.
  auto device_memory = ImportVKDeviceMemoryFromAndroidHarwareBuffer(
      device, physical_device, image.get(), ahb, ahb_props);
  if (!device_memory) {
    return;
  }

  // Bind the image to the image memory.
  if (auto result = device.bindImageMemory(image.get(), device_memory.get(), 0);
      result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not bind external device memory to image : "
                   << vk::to_string(result);
    return;
  }

  // Figure out how to perform YUV conversions.
  auto yuv_conversion = CreateYUVConversion(context, ahb_props);
  if (!yuv_conversion || !yuv_conversion->IsValid()) {
    return;
  }

  // Create image view for the newly created image.
  auto image_view = CreateVKImageView(device,                           //
                                      image.get(),                      //
                                      yuv_conversion->GetConversion(),  //
                                      ahb_props,                        //
                                      ahb_desc                          //
  );
  if (!image_view) {
    return;
  }

  needs_yuv_conversion_ = ahb_format.format == vk::Format::eUndefined;
  device_memory_ = std::move(device_memory);
  image_ = std::move(image);
  yuv_conversion_ = std::move(yuv_conversion);
  image_view_ = std::move(image_view);

#ifdef IMPELLER_DEBUG
  context.SetDebugName(device_memory_.get(), "AHB Device Memory");
  context.SetDebugName(image_.get(), "AHB Image");
  context.SetDebugName(yuv_conversion_->GetConversion(), "AHB YUV Conversion");
  context.SetDebugName(image_view_.get(), "AHB ImageView");
#endif  // IMPELLER_DEBUG

  is_valid_ = true;
}

AHBTextureSourceVK::AHBTextureSourceVK(
    const std::shared_ptr<Context>& context,
    std::unique_ptr<android::HardwareBuffer> backing_store,
    bool is_swapchain_image)
    : AHBTextureSourceVK(context,
                         backing_store->GetHandle(),
                         backing_store->GetAndroidDescriptor()) {
  backing_store_ = std::move(backing_store);
  is_swapchain_image_ = is_swapchain_image;
}

// |TextureSourceVK|
AHBTextureSourceVK::~AHBTextureSourceVK() = default;

bool AHBTextureSourceVK::IsValid() const {
  return is_valid_;
}

// |TextureSourceVK|
vk::Image AHBTextureSourceVK::GetImage() const {
  return image_.get();
}

// |TextureSourceVK|
vk::ImageView AHBTextureSourceVK::GetImageView() const {
  return image_view_.get();
}

// |TextureSourceVK|
vk::ImageView AHBTextureSourceVK::GetRenderTargetView() const {
  return image_view_.get();
}

// |TextureSourceVK|
bool AHBTextureSourceVK::IsSwapchainImage() const {
  return is_swapchain_image_;
}

// |TextureSourceVK|
std::shared_ptr<YUVConversionVK> AHBTextureSourceVK::GetYUVConversion() const {
  return needs_yuv_conversion_ ? yuv_conversion_ : nullptr;
}

const android::HardwareBuffer* AHBTextureSourceVK::GetBackingStore() const {
  return backing_store_.get();
}

}  // namespace impeller
