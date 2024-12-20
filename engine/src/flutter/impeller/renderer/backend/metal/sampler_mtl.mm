// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/metal/sampler_mtl.h"

namespace impeller {

SamplerMTL::SamplerMTL(const SamplerDescriptor& desc, id<MTLSamplerState> state)
    : Sampler(desc), state_(state) {
  FML_DCHECK(state_);
}

SamplerMTL::~SamplerMTL() = default;

id<MTLSamplerState> SamplerMTL::GetMTLSamplerState() const {
  return state_;
}

}  // namespace impeller
