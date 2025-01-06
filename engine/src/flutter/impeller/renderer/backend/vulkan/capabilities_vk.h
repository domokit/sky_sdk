// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_CAPABILITIES_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_CAPABILITIES_VK_H_

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "impeller/base/backend_cast.h"
#include "impeller/core/texture_descriptor.h"
#include "impeller/renderer/backend/vulkan/vk.h"
#include "impeller/renderer/backend/vulkan/workarounds_vk.h"
#include "impeller/renderer/capabilities.h"

namespace impeller {

class ContextVK;

//------------------------------------------------------------------------------
/// @brief      A device extension available on all platforms. Without the
///             presence of these extensions, context creation will fail.
///
enum class RequiredCommonDeviceExtensionVK : uint32_t {
  //----------------------------------------------------------------------------
  /// For displaying content in the window system.
  ///
  /// https://registry.khronos.org/vulkan/specs/1.3-extensions/man/html/VK_KHR_swapchain.html
  ///
  kKHRSwapchain,

  kLast,
};

//------------------------------------------------------------------------------
/// @brief      A device extension available on all Android platforms. Without
///             the presence of these extensions on Android, context creation
///             will fail.
///
///             Platform agnostic code can still check if these Android
///             extensions are present.
///
enum class RequiredAndroidDeviceExtensionVK : uint32_t {
  //----------------------------------------------------------------------------
  /// For importing hardware buffers used in external texture composition.
  ///
  /// https://registry.khronos.org/vulkan/specs/1.3-extensions/man/html/VK_ANDROID_external_memory_android_hardware_buffer.html
  ///
  kANDROIDExternalMemoryAndroidHardwareBuffer,

  //----------------------------------------------------------------------------
  /// Dependency of kANDROIDExternalMemoryAndroidHardwareBuffer.
  ///
  /// https://registry.khronos.org/vulkan/specs/1.3-extensions/man/html/VK_KHR_sampler_ycbcr_conversion.html
  ///
  kKHRSamplerYcbcrConversion,

  //----------------------------------------------------------------------------
  /// Dependency of kANDROIDExternalMemoryAndroidHardwareBuffer.
  ///
  /// https://registry.khronos.org/vulkan/specs/1.3-extensions/man/html/VK_KHR_external_memory.html
  ///
  kKHRExternalMemory,

  //----------------------------------------------------------------------------
  /// Dependency of kANDROIDExternalMemoryAndroidHardwareBuffer.
  ///
  /// https://registry.khronos.org/vulkan/specs/1.3-extensions/man/html/VK_EXT_queue_family_foreign.html
  ///
  kEXTQueueFamilyForeign,

  //----------------------------------------------------------------------------
  /// Dependency of kANDROIDExternalMemoryAndroidHardwareBuffer.
  ///
  /// https://registry.khronos.org/vulkan/specs/1.3-extensions/man/html/VK_KHR_dedicated_allocation.html
  ///
  kKHRDedicatedAllocation,

  //----------------------------------------------------------------------------
  /// For exporting file descriptors from fences to interact with platform APIs.
  ///
  /// https://registry.khronos.org/vulkan/specs/1.3-extensions/man/html/VK_KHR_external_fence_fd.html
  ///
  kKHRExternalFenceFd,

  //----------------------------------------------------------------------------
  /// Dependency of kKHRExternalFenceFd.
  ///
  /// https://registry.khronos.org/vulkan/specs/1.3-extensions/man/html/VK_KHR_external_fence.html
  ///
  kKHRExternalFence,

  //----------------------------------------------------------------------------
  /// For importing sync file descriptors as semaphores so the GPU can wait for
  /// semaphore to be signaled.
  ///
  /// https://registry.khronos.org/vulkan/specs/1.3-extensions/man/html/VK_KHR_external_semaphore_fd.html
  kKHRExternalSemaphoreFd,

  //----------------------------------------------------------------------------
  /// Dependency of kKHRExternalSemaphoreFd
  ///
  /// https://registry.khronos.org/vulkan/specs/1.3-extensions/man/html/VK_KHR_external_semaphore.html
  kKHRExternalSemaphore,

  kLast,
};

//------------------------------------------------------------------------------
/// @brief      A device extension enabled if available. Subsystems cannot
///             assume availability and must check if these extensions are
///             available.
///
/// @see        `CapabilitiesVK::HasExtension`.
///
enum class OptionalDeviceExtensionVK : uint32_t {
  //----------------------------------------------------------------------------
  /// To instrument and profile PSO creation.
  ///
  /// https://registry.khronos.org/vulkan/specs/1.3-extensions/man/html/VK_EXT_pipeline_creation_feedback.html
  ///
  kEXTPipelineCreationFeedback,

  //----------------------------------------------------------------------------
  /// To enable context creation on MoltenVK. A non-conformant Vulkan
  /// implementation.
  ///
  /// https://registry.khronos.org/vulkan/specs/1.3-extensions/man/html/VK_KHR_portability_subset.html
  ///
  kVKKHRPortabilitySubset,

  //----------------------------------------------------------------------------
  /// For fixed-rate compression of images.
  ///
  /// https://registry.khronos.org/vulkan/specs/1.3-extensions/man/html/VK_EXT_image_compression_control.html
  ///
  kEXTImageCompressionControl,

  kLast,
};

//------------------------------------------------------------------------------
/// @brief      A pixel format and usage that is sufficient to check if images
///             of that format and usage are suitable for use with fixed-rate
///             compression.
///
struct FRCFormatDescriptor {
  vk::Format format = vk::Format::eUndefined;
  vk::ImageType type = {};
  vk::ImageTiling tiling = {};
  vk::ImageUsageFlags usage = {};
  vk::ImageCreateFlags flags = {};

  explicit FRCFormatDescriptor(const vk::ImageCreateInfo& image_info)
      : format(image_info.format),
        type(image_info.imageType),
        tiling(image_info.tiling),
        usage(image_info.usage),
        flags(image_info.flags) {}
};

//------------------------------------------------------------------------------
/// @brief      The Vulkan layers and extensions wrangler.
///
class CapabilitiesVK final : public Capabilities,
                             public BackendCast<CapabilitiesVK, Capabilities> {
 public:
  explicit CapabilitiesVK(bool enable_validations,
                          bool fatal_missing_validations = false,
                          bool use_embedder_extensions = false,
                          std::vector<std::string> instance_extensions = {},
                          std::vector<std::string> device_extensions = {});

  ~CapabilitiesVK();

  bool IsValid() const;

  bool AreValidationsEnabled() const;

  bool HasExtension(RequiredCommonDeviceExtensionVK ext) const;

  bool HasExtension(RequiredAndroidDeviceExtensionVK ext) const;

  bool HasExtension(OptionalDeviceExtensionVK ext) const;

  std::optional<std::vector<std::string>> GetEnabledLayers() const;

  std::optional<std::vector<std::string>> GetEnabledInstanceExtensions() const;

  std::optional<std::vector<std::string>> GetEnabledDeviceExtensions(
      const vk::PhysicalDevice& physical_device) const;

  using PhysicalDeviceFeatures =
      vk::StructureChain<vk::PhysicalDeviceFeatures2,
                         vk::PhysicalDeviceSamplerYcbcrConversionFeaturesKHR,
                         vk::PhysicalDevice16BitStorageFeatures,
                         vk::PhysicalDeviceImageCompressionControlFeaturesEXT>;

  std::optional<PhysicalDeviceFeatures> GetEnabledDeviceFeatures(
      const vk::PhysicalDevice& physical_device) const;

  [[nodiscard]] bool SetPhysicalDevice(
      const vk::PhysicalDevice& physical_device,
      const PhysicalDeviceFeatures& enabled_features);

  const vk::PhysicalDeviceProperties& GetPhysicalDeviceProperties() const;

  void SetOffscreenFormat(PixelFormat pixel_format) const;

  // |Capabilities|
  bool SupportsOffscreenMSAA() const override;

  // |Capabilities|
  bool SupportsImplicitResolvingMSAA() const override;

  // |Capabilities|
  bool SupportsSSBO() const override;

  // |Capabilities|
  bool SupportsTextureToTextureBlits() const override;

  // |Capabilities|
  bool SupportsFramebufferFetch() const override;

  // |Capabilities|
  bool SupportsCompute() const override;

  // |Capabilities|
  bool SupportsComputeSubgroups() const override;

  // |Capabilities|
  bool SupportsReadFromResolve() const override;

  // |Capabilities|
  bool SupportsDecalSamplerAddressMode() const override;

  // |Capabilities|
  bool SupportsDeviceTransientTextures() const override;

  // |Capabilities|
  bool SupportsTriangleFan() const override;

  // |Capabilities|
  bool SupportsPrimitiveRestart() const override;

  // |Capabilities|
  PixelFormat GetDefaultColorFormat() const override;

  // |Capabilities|
  PixelFormat GetDefaultStencilFormat() const override;

  // |Capabilities|
  PixelFormat GetDefaultDepthStencilFormat() const override;

  // |Capabilities|
  PixelFormat GetDefaultGlyphAtlasFormat() const override;

  // |Capabilities|
  ISize GetMaximumRenderPassAttachmentSize() const override;

  //----------------------------------------------------------------------------
  /// @return     If fixed-rate compression for non-onscreen surfaces is
  ///             supported.
  ///
  bool SupportsTextureFixedRateCompression() const;

  //----------------------------------------------------------------------------
  /// @brief      Get the fixed compression rate supported by the context for
  ///             the given format and usage.
  ///
  /// @param[in]  compression_type  The compression type.
  /// @param[in]  desc              The format and usage of the image.
  ///
  /// @return     The supported fixed compression rate.
  ///
  std::optional<vk::ImageCompressionFixedRateFlagBitsEXT> GetSupportedFRCRate(
      CompressionType compression_type,
      const FRCFormatDescriptor& desc) const;

  //----------------------------------------------------------------------------
  /// @brief      Update capabilities for the given set of workarounds.
  void ApplyWorkarounds(const WorkaroundsVK& workarounds);

 private:
  bool validations_enabled_ = false;
  std::map<std::string, std::set<std::string>> exts_;
  std::set<RequiredCommonDeviceExtensionVK> required_common_device_extensions_;
  std::set<RequiredAndroidDeviceExtensionVK>
      required_android_device_extensions_;
  std::set<OptionalDeviceExtensionVK> optional_device_extensions_;
  mutable PixelFormat default_color_format_ = PixelFormat::kUnknown;
  PixelFormat default_stencil_format_ = PixelFormat::kUnknown;
  PixelFormat default_depth_stencil_format_ = PixelFormat::kUnknown;
  vk::PhysicalDevice physical_device_;
  vk::PhysicalDeviceProperties device_properties_;
  bool supports_compute_subgroups_ = false;
  bool supports_device_transient_textures_ = false;
  bool supports_texture_fixed_rate_compression_ = false;
  ISize max_render_pass_attachment_size_ = ISize{0, 0};
  bool has_triangle_fans_ = true;
  bool has_primitive_restart_ = true;
  bool is_valid_ = false;

  // The embedder.h API is responsible for providing the instance and device
  // extensions.
  bool use_embedder_extensions_ = false;
  std::vector<std::string> embedder_instance_extensions_;
  std::vector<std::string> embedder_device_extensions_;

  bool HasExtension(const std::string& ext) const;

  bool HasLayer(const std::string& layer) const;

  CapabilitiesVK(const CapabilitiesVK&) = delete;

  CapabilitiesVK& operator=(const CapabilitiesVK&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_CAPABILITIES_VK_H_
