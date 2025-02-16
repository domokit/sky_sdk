// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/interop/backend/metal/surface_mtl.h"

#include "impeller/renderer/backend/metal/surface_mtl.h"
#include "impeller/toolkit/interop/backend/metal/context_mtl.h"

namespace impeller::interop {

SurfaceMTL::SurfaceMTL(Context& context, void* metal_drawable)
    : SurfaceMTL(context,
                 impeller::SurfaceMTL::MakeFromMetalLayerDrawable(
                     context.GetContext(),
                     (__bridge id<CAMetalDrawable>)metal_drawable,
                     reinterpret_cast<interop::ContextMTL*>(&context)
                         ->GetSwapchainTransients())) {}

SurfaceMTL::SurfaceMTL(Context& context,
                       std::shared_ptr<impeller::Surface> surface)
    : Surface(context, std::move(surface)) {}

SurfaceMTL::~SurfaceMTL() = default;

}  // namespace impeller::interop
