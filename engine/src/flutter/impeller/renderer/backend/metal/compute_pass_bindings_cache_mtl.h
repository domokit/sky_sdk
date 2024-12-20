// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_COMPUTE_PASS_BINDINGS_CACHE_MTL_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_COMPUTE_PASS_BINDINGS_CACHE_MTL_H_

#include <Metal/Metal.h>

#include "impeller/renderer/compute_pass.h"
#include "impeller/renderer/pipeline_descriptor.h"

namespace impeller {

//-----------------------------------------------------------------------------
/// @brief      Ensures that bindings on the pass are not redundantly set or
///             updated. Avoids making the driver do additional checks and makes
///             the frame insights during profiling and instrumentation not
///             complain about the same.
///
///             There should be no change to rendering if this caching was
///             absent.
///
struct ComputePassBindingsCacheMTL {
  explicit ComputePassBindingsCacheMTL() {}

  ComputePassBindingsCacheMTL(const ComputePassBindingsCacheMTL&) = delete;

  ComputePassBindingsCacheMTL(ComputePassBindingsCacheMTL&&) = delete;

  void SetComputePipelineState(id<MTLComputePipelineState> pipeline);

  id<MTLComputePipelineState> GetPipeline() const;

  void SetEncoder(id<MTLComputeCommandEncoder> encoder);

  void SetBuffer(uint64_t index, uint64_t offset, id<MTLBuffer> buffer);

  void SetTexture(uint64_t index, id<MTLTexture> texture);

  void SetSampler(uint64_t index, id<MTLSamplerState> sampler);

 private:
  struct BufferOffsetPair {
    id<MTLBuffer> buffer = nullptr;
    size_t offset = 0u;
  };
  using BufferMap = std::map<uint64_t, BufferOffsetPair>;
  using TextureMap = std::map<uint64_t, id<MTLTexture>>;
  using SamplerMap = std::map<uint64_t, id<MTLSamplerState>>;

  id<MTLComputeCommandEncoder> encoder_;
  id<MTLComputePipelineState> pipeline_ = nullptr;
  BufferMap buffers_;
  TextureMap textures_;
  SamplerMap samplers_;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_COMPUTE_PASS_BINDINGS_CACHE_MTL_H_
