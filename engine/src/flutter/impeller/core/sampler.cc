// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/core/sampler.h"

namespace impeller {

Sampler::Sampler(const SamplerDescriptor& desc) : desc_(desc) {}

Sampler::~Sampler() = default;

const SamplerDescriptor& Sampler::GetDescriptor() const {
  return desc_;
}

}  // namespace impeller
