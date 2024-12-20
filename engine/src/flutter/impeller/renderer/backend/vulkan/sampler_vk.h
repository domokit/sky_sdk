// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SAMPLER_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SAMPLER_VK_H_

#include "impeller/base/backend_cast.h"
#include "impeller/core/sampler.h"
#include "impeller/renderer/backend/vulkan/shared_object_vk.h"
#include "impeller/renderer/backend/vulkan/vk.h"

namespace impeller {

class SamplerLibraryVK;
class YUVConversionVK;

class SamplerVK final : public Sampler, public BackendCast<SamplerVK, Sampler> {
 public:
  SamplerVK(const vk::Device& device,
            const SamplerDescriptor&,
            std::shared_ptr<YUVConversionVK> yuv_conversion = {});

  // |Sampler|
  ~SamplerVK() override;

  vk::Sampler GetSampler() const;

  std::shared_ptr<SamplerVK> CreateVariantForConversion(
      std::shared_ptr<YUVConversionVK> conversion) const;

  const std::shared_ptr<YUVConversionVK>& GetYUVConversion() const;

 private:
  friend SamplerLibraryVK;

  const vk::Device device_;
  SharedHandleVK<vk::Sampler> sampler_;
  std::shared_ptr<YUVConversionVK> yuv_conversion_;
  bool is_valid_ = false;

  SamplerVK(const SamplerVK&) = delete;

  SamplerVK& operator=(const SamplerVK&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SAMPLER_VK_H_
