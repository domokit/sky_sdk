// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/context.h"

#include <utility>

namespace impeller {

Context::~Context() = default;

Context::Context() = default;

bool Context::UpdateOffscreenLayerPixelFormat(PixelFormat format) {
  return false;
}

bool Context::EnqueueCommandBuffer(
    std::shared_ptr<CommandBuffer> command_buffer) {
  return GetCommandQueue()->Submit({std::move(command_buffer)}).ok();
}

bool Context::FlushCommandBuffers() {
  return true;
}

std::shared_ptr<const IdleWaiter> Context::GetIdleWaiter() const {
  return nullptr;
}

void Context::ResetThreadLocalState() const {
  // Nothing to do.
}

bool Context::AddTrackingFence(const std::shared_ptr<Texture>& texture) const {
  return false;
}

}  // namespace impeller
