// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/gles/sampler_library_gles.h"

#include "impeller/base/config.h"
#include "impeller/base/validation.h"
#include "impeller/core/formats.h"
#include "impeller/core/sampler_descriptor.h"
#include "impeller/renderer/backend/gles/sampler_gles.h"

namespace impeller {

SamplerLibraryGLES::SamplerLibraryGLES(bool supports_decal_sampler_address_mode)
    : supports_decal_sampler_address_mode_(
          supports_decal_sampler_address_mode) {}

// |SamplerLibrary|
SamplerLibraryGLES::~SamplerLibraryGLES() = default;

// |SamplerLibrary|
raw_ptr<const Sampler> SamplerLibraryGLES::GetSampler(
    const SamplerDescriptor& descriptor) {
  if (!supports_decal_sampler_address_mode_ &&
      (descriptor.width_address_mode == SamplerAddressMode::kDecal ||
       descriptor.height_address_mode == SamplerAddressMode::kDecal ||
       descriptor.depth_address_mode == SamplerAddressMode::kDecal)) {
    VALIDATION_LOG << "SamplerAddressMode::kDecal is not supported by the "
                      "current OpenGLES backend.";
    return raw_ptr<const Sampler>{nullptr};
  }
  uint64_t p_key = SamplerDescriptor::ToKey(descriptor);
  for (const auto& [key, value] : samplers_) {
    if (key == p_key) {
      return raw_ptr(value);
    }
  }

  auto sampler = std::unique_ptr<SamplerGLES>(new SamplerGLES(descriptor));
  samplers_.push_back(std::make_pair(p_key, std::move(sampler)));

  return raw_ptr(samplers_.back().second);
}

}  // namespace impeller
