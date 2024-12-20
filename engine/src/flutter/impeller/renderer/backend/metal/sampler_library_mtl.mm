// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/metal/sampler_library_mtl.h"

#include "impeller/renderer/backend/metal/formats_mtl.h"
#include "impeller/renderer/backend/metal/sampler_mtl.h"

namespace impeller {

SamplerLibraryMTL::SamplerLibraryMTL(id<MTLDevice> device) : device_(device) {}

SamplerLibraryMTL::~SamplerLibraryMTL() = default;

raw_ptr<const Sampler> SamplerLibraryMTL::GetSampler(
    const SamplerDescriptor& descriptor) {
  uint64_t p_key = SamplerDescriptor::ToKey(descriptor);
  for (const auto& [key, value] : samplers_) {
    if (key == p_key) {
      return raw_ptr(value);
    }
  }
  if (!device_) {
    return raw_ptr<const Sampler>(nullptr);
  }
  auto desc = [[MTLSamplerDescriptor alloc] init];
  desc.minFilter = ToMTLSamplerMinMagFilter(descriptor.min_filter);
  desc.magFilter = ToMTLSamplerMinMagFilter(descriptor.mag_filter);
  desc.mipFilter = ToMTLSamplerMipFilter(descriptor.mip_filter);
  desc.sAddressMode = ToMTLSamplerAddressMode(descriptor.width_address_mode);
  desc.tAddressMode = ToMTLSamplerAddressMode(descriptor.height_address_mode);
  desc.rAddressMode = ToMTLSamplerAddressMode(descriptor.depth_address_mode);
  if (@available(iOS 14.0, macos 10.12, *)) {
    desc.borderColor = MTLSamplerBorderColorTransparentBlack;
  }
#ifdef IMPELLER_DEBUG
  if (!descriptor.label.empty()) {
    desc.label = @(descriptor.label.data());
  }
#endif  // IMPELLER_DEBUG

  auto mtl_sampler = [device_ newSamplerStateWithDescriptor:desc];
  if (!mtl_sampler) {
    return raw_ptr<const Sampler>(nullptr);
    ;
  }
  auto sampler =
      std::shared_ptr<SamplerMTL>(new SamplerMTL(descriptor, mtl_sampler));
  samplers_.push_back(std::make_pair(p_key, std::move(sampler)));
  return raw_ptr(samplers_.back().second);
}

}  // namespace impeller
