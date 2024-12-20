// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_SAMPLER_MTL_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_SAMPLER_MTL_H_

#include <Metal/Metal.h>

#include "impeller/base/backend_cast.h"
#include "impeller/core/sampler.h"

namespace impeller {

class SamplerLibraryMTL;

class SamplerMTL final : public Sampler,
                         public BackendCast<SamplerMTL, Sampler> {
 public:
  SamplerMTL();

  // |Sampler|
  ~SamplerMTL() override;

  id<MTLSamplerState> GetMTLSamplerState() const;

 private:
  friend SamplerLibraryMTL;

  id<MTLSamplerState> state_ = nullptr;

  SamplerMTL(const SamplerDescriptor& desc, id<MTLSamplerState> state);

  SamplerMTL(const SamplerMTL&) = delete;

  SamplerMTL& operator=(const SamplerMTL&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_SAMPLER_MTL_H_
