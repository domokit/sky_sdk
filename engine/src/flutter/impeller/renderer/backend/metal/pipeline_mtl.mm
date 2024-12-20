// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/metal/pipeline_mtl.h"

namespace impeller {

PipelineMTL::PipelineMTL(std::weak_ptr<PipelineLibrary> library,
                         const PipelineDescriptor& desc,
                         id<MTLRenderPipelineState> state,
                         id<MTLDepthStencilState> depth_stencil_state)
    : Pipeline(std::move(library), desc),
      pipeline_state_(state),
      depth_stencil_state_(depth_stencil_state) {
  if (!pipeline_state_) {
    return;
  }
  is_valid_ = true;
}

PipelineMTL::~PipelineMTL() = default;

bool PipelineMTL::IsValid() const {
  return is_valid_;
}

id<MTLRenderPipelineState> PipelineMTL::GetMTLRenderPipelineState() const {
  return pipeline_state_;
}

id<MTLDepthStencilState> PipelineMTL::GetMTLDepthStencilState() const {
  return depth_stencil_state_;
}

}  // namespace impeller
