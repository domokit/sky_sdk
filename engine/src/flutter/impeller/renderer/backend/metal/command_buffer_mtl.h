// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_COMMAND_BUFFER_MTL_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_COMMAND_BUFFER_MTL_H_

#include <Metal/Metal.h>

#include "impeller/core/allocator.h"
#include "impeller/renderer/command_buffer.h"

namespace impeller {

class CommandBufferMTL final : public CommandBuffer {
 public:
  // |CommandBuffer|
  ~CommandBufferMTL() override;

 private:
  friend class ContextMTL;

  id<MTLCommandBuffer> buffer_ = nil;
  id<MTLDevice> device_ = nil;

  CommandBufferMTL(const std::weak_ptr<const Context>& context,
                   id<MTLDevice> device,
                   id<MTLCommandQueue> queue);

  // |CommandBuffer|
  void SetLabel(std::string_view label) const override;

  // |CommandBuffer|
  bool IsValid() const override;

  // |CommandBuffer|
  bool OnSubmitCommands(CompletionCallback callback) override;

  // |CommandBuffer|
  void OnWaitUntilCompleted() override;

  // |CommandBuffer|
  void OnWaitUntilScheduled() override;

  // |CommandBuffer|
  std::shared_ptr<RenderPass> OnCreateRenderPass(RenderTarget target) override;

  // |CommandBuffer|
  std::shared_ptr<BlitPass> OnCreateBlitPass() override;

  // |CommandBuffer|
  std::shared_ptr<ComputePass> OnCreateComputePass() override;

  CommandBufferMTL(const CommandBufferMTL&) = delete;

  CommandBufferMTL& operator=(const CommandBufferMTL&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_COMMAND_BUFFER_MTL_H_
