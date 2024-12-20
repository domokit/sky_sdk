// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_COMMAND_BUFFER_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_COMMAND_BUFFER_VK_H_

#include "fml/status_or.h"
#include "impeller/base/backend_cast.h"
#include "impeller/renderer/backend/vulkan/command_queue_vk.h"
#include "impeller/renderer/backend/vulkan/descriptor_pool_vk.h"
#include "impeller/renderer/backend/vulkan/device_holder_vk.h"
#include "impeller/renderer/backend/vulkan/texture_source_vk.h"
#include "impeller/renderer/backend/vulkan/tracked_objects_vk.h"
#include "impeller/renderer/backend/vulkan/vk.h"
#include "impeller/renderer/command_buffer.h"

namespace impeller {

class ContextVK;
class CommandEncoderFactoryVK;
class CommandEncoderVK;

class CommandBufferVK final
    : public CommandBuffer,
      public BackendCast<CommandBufferVK, CommandBuffer>,
      public std::enable_shared_from_this<CommandBufferVK> {
 public:
  // |CommandBuffer|
  ~CommandBufferVK() override;

  // Encoder Functionality

  /// @brief Ensure that [object] is kept alive until this command buffer
  ///        completes execution.
  bool Track(const std::shared_ptr<SharedObjectVK>& object);

  /// @brief Ensure that [buffer] is kept alive until this command buffer
  ///        completes execution.
  bool Track(const std::shared_ptr<const DeviceBuffer>& buffer);

  /// @brief Ensure that [texture] is kept alive until this command buffer
  ///       completes execution.
  bool Track(const std::shared_ptr<const Texture>& texture);

  /// @brief Ensure that [texture] is kept alive until this command buffer
  ///        completes execution.
  bool Track(const std::shared_ptr<const TextureSourceVK>& texture);

  /// @brief Retrieve the native command buffer from this object.
  vk::CommandBuffer GetCommandBuffer() const;

  /// @brief Push a debug group.
  ///
  /// This label is only visible in debuggers like RenderDoc. This function is
  /// ignored in release builds.
  void PushDebugGroup(std::string_view label) const;

  /// @brief Pop the previous debug group.
  ///
  /// This label is only visible in debuggers like RenderDoc. This function is
  /// ignored in release builds.
  void PopDebugGroup() const;

  /// @brief Insert a new debug marker.
  ///
  /// This label is only visible in debuggers like RenderDoc. This function is
  /// ignored in release builds.
  void InsertDebugMarker(std::string_view label) const;

  /// @brief End recording of the current command buffer.
  bool EndCommandBuffer() const;

  /// @brief Allocate a new descriptor set for the given [layout].
  fml::StatusOr<vk::DescriptorSet> AllocateDescriptorSets(
      const vk::DescriptorSetLayout& layout,
      const ContextVK& context);

  // Visible for testing.
  DescriptorPoolVK& GetDescriptorPool() const;

 private:
  friend class ContextVK;
  friend class CommandQueueVK;

  std::weak_ptr<const DeviceHolderVK> device_holder_;
  std::shared_ptr<TrackedObjectsVK> tracked_objects_;

  CommandBufferVK(std::weak_ptr<const Context> context,
                  std::weak_ptr<const DeviceHolderVK> device_holder,
                  std::shared_ptr<TrackedObjectsVK> tracked_objects);

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

  CommandBufferVK(const CommandBufferVK&) = delete;

  CommandBufferVK& operator=(const CommandBufferVK&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_COMMAND_BUFFER_VK_H_
