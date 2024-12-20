// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_PIPELINE_MTL_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_PIPELINE_MTL_H_

#include <Metal/Metal.h>

#include "impeller/base/backend_cast.h"
#include "impeller/renderer/pipeline.h"

namespace impeller {

class PipelineMTL final
    : public Pipeline<PipelineDescriptor>,
      public BackendCast<PipelineMTL, Pipeline<PipelineDescriptor>> {
 public:
  // |Pipeline|
  ~PipelineMTL() override;

  id<MTLRenderPipelineState> GetMTLRenderPipelineState() const;

  id<MTLDepthStencilState> GetMTLDepthStencilState() const;

 private:
  friend class PipelineLibraryMTL;

  id<MTLRenderPipelineState> pipeline_state_;
  id<MTLDepthStencilState> depth_stencil_state_;
  bool is_valid_ = false;

  PipelineMTL(std::weak_ptr<PipelineLibrary> library,
              const PipelineDescriptor& desc,
              id<MTLRenderPipelineState> state,
              id<MTLDepthStencilState> depth_stencil_state);

  // |Pipeline|
  bool IsValid() const override;

  PipelineMTL(const PipelineMTL&) = delete;

  PipelineMTL& operator=(const PipelineMTL&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_PIPELINE_MTL_H_
