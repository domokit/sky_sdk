// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/interop/backend/vulkan/surface_vk.h"

namespace impeller::interop {

SurfaceVK::SurfaceVK(Context& context,
                     std::shared_ptr<impeller::Surface> surface)
    : Surface(context, std::move(surface)) {}

SurfaceVK::~SurfaceVK() = default;

}  // namespace impeller::interop
