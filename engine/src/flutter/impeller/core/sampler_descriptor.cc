// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/core/sampler_descriptor.h"

namespace impeller {

SamplerDescriptor::SamplerDescriptor() = default;

SamplerDescriptor::SamplerDescriptor(std::string_view label,
                                     MinMagFilter min_filter,
                                     MinMagFilter mag_filter,
                                     MipFilter mip_filter)
    : min_filter(min_filter),
      mag_filter(mag_filter),
      mip_filter(mip_filter),
      label(label) {}

}  // namespace impeller
