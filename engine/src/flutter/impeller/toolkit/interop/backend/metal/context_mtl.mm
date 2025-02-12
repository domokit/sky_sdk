// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/interop/backend/metal/context_mtl.h"

#include "impeller/base/validation.h"
#include "impeller/entity/mtl/entity_shaders.h"
#include "impeller/entity/mtl/framebuffer_blend_shaders.h"
#include "impeller/entity/mtl/modern_shaders.h"
#include "impeller/renderer/backend/metal/context_mtl.h"
#include "impeller/renderer/mtl/compute_shaders.h"

namespace impeller::interop {

static std::vector<std::shared_ptr<fml::Mapping>>
CreateShaderLibraryMappings() {
  return {std::make_shared<fml::NonOwnedMapping>(
              impeller_entity_shaders_data, impeller_entity_shaders_length),
          std::make_shared<fml::NonOwnedMapping>(
              impeller_modern_shaders_data, impeller_modern_shaders_length),
          std::make_shared<fml::NonOwnedMapping>(
              impeller_framebuffer_blend_shaders_data,
              impeller_framebuffer_blend_shaders_length),
          std::make_shared<fml::NonOwnedMapping>(
              impeller_compute_shaders_data, impeller_compute_shaders_length)};
}

ScopedObject<Context> ContextMTL::Create() {
  auto impeller_context =
      impeller::ContextMTL::Create(CreateShaderLibraryMappings(),        //
                                   std::make_shared<fml::SyncSwitch>(),  //
                                   "Impeller"                            //
      );
  if (!impeller_context) {
    VALIDATION_LOG << "Could not create Impeller context.";
    return {};
  }
  return Create(std::move(impeller_context));
}

ScopedObject<Context> ContextMTL::Create(
    const std::shared_ptr<impeller::Context>& impeller_context) {
  // Can't call Create because of private constructor. Adopt the raw pointer
  // instead.
  auto context = Adopt<Context>(new ContextMTL(impeller_context));
  if (!context->IsValid()) {
    VALIDATION_LOG << " Could not create valid context.";
    return {};
  }
  return context;
}

ContextMTL::ContextMTL(const std::shared_ptr<impeller::Context>& context)
    : Context(context),
      swapchain_transients_(std::make_shared<SwapchainTransientsMTL>(
          context->GetResourceAllocator())) {}

ContextMTL::~ContextMTL() = default;

const std::shared_ptr<SwapchainTransientsMTL>&
ContextMTL::GetSwapchainTransients() const {
  return swapchain_transients_;
}

}  // namespace impeller::interop
