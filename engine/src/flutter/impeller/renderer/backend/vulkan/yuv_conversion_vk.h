// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_YUV_CONVERSION_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_YUV_CONVERSION_VK_H_

#include <unordered_map>

#include "flutter/fml/build_config.h"
#include "impeller/base/comparable.h"
#include "impeller/base/thread.h"
#include "impeller/core/sampler.h"
#include "impeller/renderer/backend/vulkan/sampler_vk.h"
#include "impeller/renderer/backend/vulkan/shared_object_vk.h"
#include "impeller/renderer/backend/vulkan/vk.h"

namespace impeller {

//------------------------------------------------------------------------------
/// A descriptor used to create a new YUV conversion in a conversion library.
///
using YUVConversionDescriptorVK =
    vk::StructureChain<vk::SamplerYcbcrConversionCreateInfo
#if FML_OS_ANDROID
                       // For VK_ANDROID_external_memory_android_hardware_buffer
                       ,
                       vk::ExternalFormatANDROID
#endif  // FML_OS_ANDROID
                       >;

class YUVConversionLibraryVK;

//------------------------------------------------------------------------------
/// @brief      It is sometimes necessary to deal with formats not native to
///             Vulkan. In such cases, extra information is necessary to access
///             images. A YUV conversion object is needed in such instances.
///
///             There are usually only a handful of viable conversions in a
///             given context. However, due to the way the Vulkan spec. treats
///             "identically defined" conversions, only a single conversion
///             object is valid for an equivalent `YUVConversionDescriptorVK`.
///             Because of this restriction, it is not possible to just create a
///             conversion from a descriptor (as the underlying handles will be
///             equivalent but different). Instead, a conversion may only be
///             obtained from a conversion library. Libraries handle hashing and
///             caching conversions by descriptor. Caller can find a library on
///             the top-level context. They may not create their own (the
///             constructor is private).
///
class YUVConversionVK final {
 public:
  ~YUVConversionVK();

  YUVConversionVK(const YUVConversionVK&) = delete;

  YUVConversionVK& operator=(const YUVConversionVK&) = delete;

  //----------------------------------------------------------------------------
  /// @return     `true` if this conversion is valid for use with images and
  ///             samplers.
  ///
  bool IsValid() const;

  //----------------------------------------------------------------------------
  /// @brief      Get the descriptor used to create this conversion.
  ///
  const YUVConversionDescriptorVK& GetDescriptor() const;

  //----------------------------------------------------------------------------
  /// @return     The Vulkan handle of the YUV conversion.
  ///
  vk::SamplerYcbcrConversion GetConversion() const;

 private:
  friend class YUVConversionLibraryVK;

  YUVConversionDescriptorVK chain_;
  vk::UniqueSamplerYcbcrConversion conversion_;

  YUVConversionVK(const vk::Device& device,
                  const YUVConversionDescriptorVK& chain);
};

struct YUVConversionDescriptorVKHash {
  std::size_t operator()(const YUVConversionDescriptorVK& object) const;
};

struct YUVConversionDescriptorVKEqual {
  bool operator()(const YUVConversionDescriptorVK& lhs,
                  const YUVConversionDescriptorVK& rhs) const;
};

struct ImmutableSamplerKeyVK : public Comparable<ImmutableSamplerKeyVK> {
  SamplerDescriptor sampler;
  YUVConversionDescriptorVK yuv_conversion;

  explicit ImmutableSamplerKeyVK(const SamplerVK& sampler);

  // |Comparable<ImmutableSamplerKey>|
  std::size_t GetHash() const override;

  // |Comparable<ImmutableSamplerKey>|
  bool IsEqual(const ImmutableSamplerKeyVK& other) const override;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_YUV_CONVERSION_VK_H_
