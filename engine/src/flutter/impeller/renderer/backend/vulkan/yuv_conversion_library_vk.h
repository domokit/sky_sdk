// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_YUV_CONVERSION_LIBRARY_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_YUV_CONVERSION_LIBRARY_VK_H_

#include "impeller/renderer/backend/vulkan/yuv_conversion_vk.h"

namespace impeller {

class DeviceHolderVK;

//------------------------------------------------------------------------------
/// @brief      Due the way the Vulkan spec. treats "identically defined"
///             conversions, creating two conversion with identical descriptors,
///             using one with the image and the other with the sampler, is
///             invalid use.
///
///             A conversion library hashes and caches identical descriptors to
///             de-duplicate conversions.
///
///             There can only be one conversion library (the constructor is
///             private to force this) and it found in the context.
///
class YUVConversionLibraryVK {
 public:
  ~YUVConversionLibraryVK();

  YUVConversionLibraryVK(const YUVConversionLibraryVK&) = delete;

  YUVConversionLibraryVK& operator=(const YUVConversionLibraryVK&) = delete;

  //----------------------------------------------------------------------------
  /// @brief      Get a conversion for the given descriptor. If there is already
  ///             a conversion created for an equivalent descriptor, a reference
  ///             to that descriptor is returned instead.
  ///
  /// @param[in]  desc  The descriptor.
  ///
  /// @return     The conversion. A previously created conversion if one was
  ///             present and a new one if not. A newly created conversion is
  ///             cached for subsequent accesses.
  ///
  std::shared_ptr<YUVConversionVK> GetConversion(
      const YUVConversionDescriptorVK& chain);

 private:
  friend class ContextVK;

  using ConversionsMap = std::unordered_map<YUVConversionDescriptorVK,
                                            std::shared_ptr<YUVConversionVK>,
                                            YUVConversionDescriptorVKHash,
                                            YUVConversionDescriptorVKEqual>;

  std::weak_ptr<DeviceHolderVK> device_holder_;
  Mutex conversions_mutex_;
  ConversionsMap conversions_ IPLR_GUARDED_BY(conversions_mutex_);

  explicit YUVConversionLibraryVK(std::weak_ptr<DeviceHolderVK> device_holder);
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_YUV_CONVERSION_LIBRARY_VK_H_
