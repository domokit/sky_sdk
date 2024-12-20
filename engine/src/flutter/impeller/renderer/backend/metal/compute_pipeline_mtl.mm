// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/metal/compute_pipeline_mtl.h"

namespace impeller {

ComputePipelineMTL::ComputePipelineMTL(std::weak_ptr<PipelineLibrary> library,
                                       const ComputePipelineDescriptor& desc,
                                       id<MTLComputePipelineState> state)
    : Pipeline(std::move(library), desc), pipeline_state_(state) {
  if (!pipeline_state_) {
    return;
  }
  is_valid_ = true;
}

ComputePipelineMTL::~ComputePipelineMTL() = default;

bool ComputePipelineMTL::IsValid() const {
  return is_valid_;
}

id<MTLComputePipelineState> ComputePipelineMTL::GetMTLComputePipelineState()
    const {
  return pipeline_state_;
}

}  // namespace impeller
