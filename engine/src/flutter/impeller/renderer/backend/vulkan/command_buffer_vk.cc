// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/command_buffer_vk.h"

#include <memory>
#include <utility>

#include "fml/logging.h"
#include "impeller/renderer/backend/vulkan/blit_pass_vk.h"
#include "impeller/renderer/backend/vulkan/compute_pass_vk.h"
#include "impeller/renderer/backend/vulkan/context_vk.h"
#include "impeller/renderer/backend/vulkan/gpu_tracer_vk.h"
#include "impeller/renderer/backend/vulkan/render_pass_vk.h"
#include "impeller/renderer/backend/vulkan/texture_vk.h"
#include "impeller/renderer/command_buffer.h"
#include "impeller/renderer/render_target.h"

namespace impeller {

CommandBufferVK::CommandBufferVK(
    std::weak_ptr<const Context> context,
    std::weak_ptr<const DeviceHolderVK> device_holder,
    std::shared_ptr<TrackedObjectsVK> tracked_objects)
    : CommandBuffer(std::move(context)),
      device_holder_(std::move(device_holder)),
      tracked_objects_(std::move(tracked_objects)) {}

CommandBufferVK::~CommandBufferVK() = default;

void CommandBufferVK::SetLabel(std::string_view label) const {
#ifdef IMPELLER_DEBUG
  auto context = context_.lock();
  if (!context) {
    return;
  }
  ContextVK::Cast(*context).SetDebugName(GetCommandBuffer(), label);
#endif  // IMPELLER_DEBUG
}

bool CommandBufferVK::IsValid() const {
  return true;
}

bool CommandBufferVK::OnSubmitCommands(CompletionCallback callback) {
  FML_UNREACHABLE()
}

void CommandBufferVK::OnWaitUntilCompleted() {}

void CommandBufferVK::OnWaitUntilScheduled() {}

std::shared_ptr<RenderPass> CommandBufferVK::OnCreateRenderPass(
    RenderTarget target) {
  auto context = context_.lock();
  if (!context) {
    return nullptr;
  }
  auto pass =
      std::shared_ptr<RenderPassVK>(new RenderPassVK(context,            //
                                                     target,             //
                                                     shared_from_this()  //
                                                     ));
  if (!pass->IsValid()) {
    return nullptr;
  }
  return pass;
}

std::shared_ptr<BlitPass> CommandBufferVK::OnCreateBlitPass() {
  if (!IsValid()) {
    return nullptr;
  }
  auto pass = std::shared_ptr<BlitPassVK>(new BlitPassVK(shared_from_this()));
  if (!pass->IsValid()) {
    return nullptr;
  }
  return pass;
}

std::shared_ptr<ComputePass> CommandBufferVK::OnCreateComputePass() {
  if (!IsValid()) {
    return nullptr;
  }
  auto context = context_.lock();
  if (!context) {
    return nullptr;
  }
  auto pass =
      std::shared_ptr<ComputePassVK>(new ComputePassVK(context,            //
                                                       shared_from_this()  //
                                                       ));
  if (!pass->IsValid()) {
    return nullptr;
  }
  return pass;
}

bool CommandBufferVK::EndCommandBuffer() const {
  InsertDebugMarker("QueueSubmit");

  auto command_buffer = GetCommandBuffer();
  tracked_objects_->GetGPUProbe().RecordCmdBufferEnd(command_buffer);

  auto status = command_buffer.end();
  if (status != vk::Result::eSuccess) {
    VALIDATION_LOG << "Failed to end command buffer: " << vk::to_string(status);
    return false;
  }
  return true;
}

vk::CommandBuffer CommandBufferVK::GetCommandBuffer() const {
  if (tracked_objects_) {
    return tracked_objects_->GetCommandBuffer();
  }
  return {};
}

bool CommandBufferVK::Track(const std::shared_ptr<SharedObjectVK>& object) {
  if (!IsValid()) {
    return false;
  }
  tracked_objects_->Track(object);
  return true;
}

bool CommandBufferVK::Track(const std::shared_ptr<const DeviceBuffer>& buffer) {
  if (!IsValid()) {
    return false;
  }
  tracked_objects_->Track(buffer);
  return true;
}

bool CommandBufferVK::Track(
    const std::shared_ptr<const TextureSourceVK>& texture) {
  if (!IsValid()) {
    return false;
  }
  tracked_objects_->Track(texture);
  return true;
}

bool CommandBufferVK::Track(const std::shared_ptr<const Texture>& texture) {
  if (!IsValid()) {
    return false;
  }
  if (!texture) {
    return true;
  }
  return Track(TextureVK::Cast(*texture).GetTextureSource());
}

fml::StatusOr<vk::DescriptorSet> CommandBufferVK::AllocateDescriptorSets(
    const vk::DescriptorSetLayout& layout,
    const ContextVK& context) {
  if (!IsValid()) {
    return fml::Status(fml::StatusCode::kUnknown, "command encoder invalid");
  }

  return tracked_objects_->GetDescriptorPool().AllocateDescriptorSets(layout,
                                                                      context);
}

void CommandBufferVK::PushDebugGroup(std::string_view label) const {
  if (!HasValidationLayers()) {
    return;
  }
  vk::DebugUtilsLabelEXT label_info;
  label_info.pLabelName = label.data();
  if (auto command_buffer = GetCommandBuffer()) {
    command_buffer.beginDebugUtilsLabelEXT(label_info);
  }
}

void CommandBufferVK::PopDebugGroup() const {
  if (!HasValidationLayers()) {
    return;
  }
  if (auto command_buffer = GetCommandBuffer()) {
    command_buffer.endDebugUtilsLabelEXT();
  }
}

void CommandBufferVK::InsertDebugMarker(std::string_view label) const {
  if (!HasValidationLayers()) {
    return;
  }
  vk::DebugUtilsLabelEXT label_info;
  label_info.pLabelName = label.data();
  if (auto command_buffer = GetCommandBuffer()) {
    command_buffer.insertDebugUtilsLabelEXT(label_info);
  }
}

DescriptorPoolVK& CommandBufferVK::GetDescriptorPool() const {
  return tracked_objects_->GetDescriptorPool();
}

}  // namespace impeller
