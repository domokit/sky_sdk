// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_PASS_BINDINGS_CACHE_MTL_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_PASS_BINDINGS_CACHE_MTL_H_

#include <Metal/Metal.h>

#include "impeller/renderer/render_pass.h"
#include "impeller/renderer/render_target.h"

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
struct PassBindingsCacheMTL {
  explicit PassBindingsCacheMTL() {}

  ~PassBindingsCacheMTL() = default;

  PassBindingsCacheMTL(const PassBindingsCacheMTL&) = delete;

  PassBindingsCacheMTL(PassBindingsCacheMTL&&) = delete;

  void SetEncoder(id<MTLRenderCommandEncoder> encoder);

  void SetRenderPipelineState(id<MTLRenderPipelineState> pipeline);

  void SetDepthStencilState(id<MTLDepthStencilState> depth_stencil);

  bool SetBuffer(ShaderStage stage,
                 uint64_t index,
                 uint64_t offset,
                 id<MTLBuffer> buffer);

  bool SetTexture(ShaderStage stage, uint64_t index, id<MTLTexture> texture);

  bool SetSampler(ShaderStage stage,
                  uint64_t index,
                  id<MTLSamplerState> sampler);

  void SetViewport(const Viewport& viewport);

  void SetScissor(const IRect& scissor);

 private:
  struct BufferOffsetPair {
    id<MTLBuffer> buffer = nullptr;
    size_t offset = 0u;
  };
  using BufferMap = std::map<uint64_t, BufferOffsetPair>;
  using TextureMap = std::map<uint64_t, id<MTLTexture>>;
  using SamplerMap = std::map<uint64_t, id<MTLSamplerState>>;

  id<MTLRenderCommandEncoder> encoder_;
  id<MTLRenderPipelineState> pipeline_ = nullptr;
  id<MTLDepthStencilState> depth_stencil_ = nullptr;
  std::map<ShaderStage, BufferMap> buffers_;
  std::map<ShaderStage, TextureMap> textures_;
  std::map<ShaderStage, SamplerMap> samplers_;
  std::optional<Viewport> viewport_;
  std::optional<IRect> scissor_;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_PASS_BINDINGS_CACHE_MTL_H_
