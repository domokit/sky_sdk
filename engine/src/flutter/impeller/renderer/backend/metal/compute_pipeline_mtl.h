// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_COMPUTE_PIPELINE_MTL_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_COMPUTE_PIPELINE_MTL_H_

#include <Metal/Metal.h>

#include "impeller/base/backend_cast.h"
#include "impeller/renderer/pipeline.h"

namespace impeller {

class ComputePipelineMTL final
    : public Pipeline<ComputePipelineDescriptor>,
      public BackendCast<ComputePipelineMTL,
                         Pipeline<ComputePipelineDescriptor>> {
 public:
  // |Pipeline|
  ~ComputePipelineMTL() override;

  id<MTLComputePipelineState> GetMTLComputePipelineState() const;

 private:
  friend class PipelineLibraryMTL;

  id<MTLComputePipelineState> pipeline_state_;
  bool is_valid_ = false;

  ComputePipelineMTL(std::weak_ptr<PipelineLibrary> library,
                     const ComputePipelineDescriptor& desc,
                     id<MTLComputePipelineState> state);

  // |Pipeline|
  bool IsValid() const override;

  ComputePipelineMTL(const ComputePipelineMTL&) = delete;

  ComputePipelineMTL& operator=(const ComputePipelineMTL&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_COMPUTE_PIPELINE_MTL_H_
