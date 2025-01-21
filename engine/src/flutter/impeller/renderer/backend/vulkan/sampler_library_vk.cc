// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/sampler_library_vk.h"

#include "impeller/core/formats.h"
#include "impeller/renderer/backend/vulkan/sampler_vk.h"

namespace impeller {

SamplerLibraryVK::SamplerLibraryVK(
    const std::weak_ptr<DeviceHolderVK>& device_holder)
    : device_holder_(device_holder) {}

SamplerLibraryVK::~SamplerLibraryVK() = default;

void SamplerLibraryVK::ApplyWorkarounds(const WorkaroundsVK& workarounds) {
  mips_disabled_workaround_ = workarounds.broken_mipmap_generation;
}

raw_ptr<const Sampler> SamplerLibraryVK::GetSampler(
    const SamplerDescriptor& desc) {
  SamplerDescriptor desc_copy = desc;
  if (mips_disabled_workaround_) {
    desc_copy.mip_filter = MipFilter::kBase;
  }

  uint64_t p_key = SamplerDescriptor::ToKey(desc_copy);
  for (const auto& [key, value] : samplers_) {
    if (key == p_key) {
      return raw_ptr(value);
    }
  }
  auto device_holder = device_holder_.lock();
  if (!device_holder || !device_holder->GetDevice()) {
    return raw_ptr<const Sampler>(nullptr);
  }
  samplers_.push_back(std::make_pair(
      p_key,
      std::make_shared<SamplerVK>(device_holder->GetDevice(), desc_copy)));
  return raw_ptr(samplers_.back().second);
}

}  // namespace impeller
