// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/gles/command_buffer_gles.h"

#include "impeller/base/config.h"
#include "impeller/renderer/backend/gles/blit_pass_gles.h"
#include "impeller/renderer/backend/gles/render_pass_gles.h"

namespace impeller {

CommandBufferGLES::CommandBufferGLES(std::weak_ptr<const Context> context,
                                     std::shared_ptr<ReactorGLES> reactor)
    : CommandBuffer(std::move(context)),
      reactor_(std::move(reactor)),
      is_valid_(reactor_ && reactor_->IsValid()) {}

CommandBufferGLES::~CommandBufferGLES() = default;

// |CommandBuffer|
void CommandBufferGLES::SetLabel(std::string_view label) const {
  // Cannot support.
}

// |CommandBuffer|
bool CommandBufferGLES::IsValid() const {
  return is_valid_;
}

// |CommandBuffer|
bool CommandBufferGLES::OnSubmitCommands(CompletionCallback callback) {
  const auto result = reactor_->React();
  if (callback) {
    callback(result ? CommandBuffer::Status::kCompleted
                    : CommandBuffer::Status::kError);
  }
  return result;
}

// |CommandBuffer|
void CommandBufferGLES::OnWaitUntilCompleted() {
  reactor_->GetProcTable().Finish();
}

// |CommandBuffer|
void CommandBufferGLES::OnWaitUntilScheduled() {
  reactor_->GetProcTable().Flush();
}

// |CommandBuffer|
std::shared_ptr<RenderPass> CommandBufferGLES::OnCreateRenderPass(
    RenderTarget target) {
  if (!IsValid()) {
    return nullptr;
  }
  auto context = context_.lock();
  if (!context) {
    return nullptr;
  }
  auto pass = std::shared_ptr<RenderPassGLES>(
      new RenderPassGLES(context, target, reactor_));
  if (!pass->IsValid()) {
    return nullptr;
  }
  return pass;
}

// |CommandBuffer|
std::shared_ptr<BlitPass> CommandBufferGLES::OnCreateBlitPass() {
  if (!IsValid()) {
    return nullptr;
  }
  auto pass = std::shared_ptr<BlitPassGLES>(new BlitPassGLES(reactor_));
  if (!pass->IsValid()) {
    return nullptr;
  }
  return pass;
}

// |CommandBuffer|
std::shared_ptr<ComputePass> CommandBufferGLES::OnCreateComputePass() {
  // Compute passes aren't supported until GLES 3.2, at which point Vulkan is
  // available anyway.
  return nullptr;
}

}  // namespace impeller
