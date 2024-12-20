// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/yuv_conversion_vk.h"

#include "flutter/fml/hash_combine.h"
#include "impeller/base/validation.h"
#include "impeller/core/sampler_descriptor.h"
#include "impeller/renderer/backend/vulkan/device_holder_vk.h"
#include "impeller/renderer/backend/vulkan/sampler_vk.h"

namespace impeller {

YUVConversionVK::YUVConversionVK(const vk::Device& device,
                                 const YUVConversionDescriptorVK& chain)
    : chain_(chain) {
  auto conversion = device.createSamplerYcbcrConversionUnique(chain_.get());
  if (conversion.result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not create YUV conversion: "
                   << vk::to_string(conversion.result);
    return;
  }
  conversion_ = std::move(conversion.value);
}

YUVConversionVK::~YUVConversionVK() = default;

bool YUVConversionVK::IsValid() const {
  return conversion_ && !!conversion_.get();
}

vk::SamplerYcbcrConversion YUVConversionVK::GetConversion() const {
  return conversion_ ? conversion_.get()
                     : static_cast<vk::SamplerYcbcrConversion>(VK_NULL_HANDLE);
}

const YUVConversionDescriptorVK& YUVConversionVK::GetDescriptor() const {
  return chain_;
}

std::size_t YUVConversionDescriptorVKHash::operator()(
    const YUVConversionDescriptorVK& desc) const {
  // Hashers in Vulkan HPP hash the pNext member which isn't what we want for
  // these to be stable.
  const auto& conv = desc.get();

  std::size_t hash = fml::HashCombine(conv.format,                      //
                                      conv.ycbcrModel,                  //
                                      conv.ycbcrRange,                  //
                                      conv.components.r,                //
                                      conv.components.g,                //
                                      conv.components.b,                //
                                      conv.components.a,                //
                                      conv.xChromaOffset,               //
                                      conv.yChromaOffset,               //
                                      conv.chromaFilter,                //
                                      conv.forceExplicitReconstruction  //
  );
#if FML_OS_ANDROID
  const auto external_format = desc.get<vk::ExternalFormatANDROID>();
  fml::HashCombineSeed(hash, external_format.externalFormat);
#endif  // FML_OS_ANDROID

  return hash;
};

bool YUVConversionDescriptorVKEqual::operator()(
    const YUVConversionDescriptorVK& lhs_desc,
    const YUVConversionDescriptorVK& rhs_desc) const {
  // Default equality checks in Vulkan HPP checks pNext member members by
  // pointer which isn't what we want.
  {
    const auto& lhs = lhs_desc.get();
    const auto& rhs = rhs_desc.get();

    if (lhs.format != rhs.format ||                                         //
        lhs.ycbcrModel != rhs.ycbcrModel ||                                 //
        lhs.ycbcrRange != rhs.ycbcrRange ||                                 //
        lhs.components.r != rhs.components.r ||                             //
        lhs.components.g != rhs.components.g ||                             //
        lhs.components.b != rhs.components.b ||                             //
        lhs.components.a != rhs.components.a ||                             //
        lhs.xChromaOffset != rhs.xChromaOffset ||                           //
        lhs.yChromaOffset != rhs.yChromaOffset ||                           //
        lhs.chromaFilter != rhs.chromaFilter ||                             //
        lhs.forceExplicitReconstruction != rhs.forceExplicitReconstruction  //
    ) {
      return false;
    }
  }
#if FML_OS_ANDROID
  {
    const auto lhs = lhs_desc.get<vk::ExternalFormatANDROID>();
    const auto rhs = rhs_desc.get<vk::ExternalFormatANDROID>();
    return lhs.externalFormat == rhs.externalFormat;
  }
#else   // FML_OS_ANDROID
  return true;
#endif  // FML_OS_ANDROID
}

ImmutableSamplerKeyVK::ImmutableSamplerKeyVK(const SamplerVK& sampler)
    : sampler(sampler.GetDescriptor()) {
  if (const auto& conversion = sampler.GetYUVConversion()) {
    yuv_conversion = conversion->GetDescriptor();
  }
}

bool ImmutableSamplerKeyVK::IsEqual(const ImmutableSamplerKeyVK& other) const {
  return SamplerDescriptor::ToKey(sampler) ==
             SamplerDescriptor::ToKey(other.sampler) &&
         YUVConversionDescriptorVKEqual{}(yuv_conversion, other.yuv_conversion);
}

std::size_t ImmutableSamplerKeyVK::GetHash() const {
  return fml::HashCombine(SamplerDescriptor::ToKey(sampler),
                          YUVConversionDescriptorVKHash{}(yuv_conversion));
}

}  // namespace impeller
