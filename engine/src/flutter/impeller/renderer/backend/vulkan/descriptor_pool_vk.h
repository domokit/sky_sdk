// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_DESCRIPTOR_POOL_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_DESCRIPTOR_POOL_VK_H_

#include <cstdint>

#include "fml/status_or.h"
#include "impeller/renderer/backend/vulkan/context_vk.h"
#include "vulkan/vulkan_handles.hpp"

namespace impeller {

//------------------------------------------------------------------------------
/// @brief      A per-frame descriptor pool. Descriptors
///             from this pool don't need to be freed individually. Instead, the
///             pool must be collected after all the descriptors allocated from
///             it are done being used.
///
///             The pool or it's descriptors may not be accessed from multiple
///             threads.
///
///             Encoders create pools as necessary as they have the same
///             threading and lifecycle restrictions.
class DescriptorPoolVK {
 public:
  explicit DescriptorPoolVK(std::weak_ptr<const ContextVK> context);

  ~DescriptorPoolVK();

  fml::StatusOr<vk::DescriptorSet> AllocateDescriptorSets(
      const vk::DescriptorSetLayout& layout,
      const ContextVK& context_vk);

 private:
  std::weak_ptr<const ContextVK> context_;
  std::vector<vk::UniqueDescriptorPool> pools_;

  fml::Status CreateNewPool(const ContextVK& context_vk);

  DescriptorPoolVK(const DescriptorPoolVK&) = delete;

  DescriptorPoolVK& operator=(const DescriptorPoolVK&) = delete;
};

//------------------------------------------------------------------------------
/// @brief      Creates and manages the lifecycle of |vk::DescriptorPoolVK|
///             objects.
class DescriptorPoolRecyclerVK final
    : public std::enable_shared_from_this<DescriptorPoolRecyclerVK> {
 public:
  ~DescriptorPoolRecyclerVK() = default;

  /// The maximum number of descriptor pools this recycler will hold onto.
  static constexpr size_t kMaxRecycledPools = 32u;

  /// @brief      Creates a recycler for the given |ContextVK|.
  ///
  /// @param[in]  context The context to create the recycler for.
  explicit DescriptorPoolRecyclerVK(std::weak_ptr<ContextVK> context)
      : context_(std::move(context)) {}

  /// @brief      Gets a descriptor pool.
  ///
  ///             This may create a new descriptor pool if no existing pools had
  ///             the necessary capacity.
  vk::UniqueDescriptorPool Get();

  /// @brief      Returns the descriptor pool to be reset on a background
  ///             thread.
  ///
  /// @param[in]  pool The pool to recycler.
  void Reclaim(vk::UniqueDescriptorPool&& pool);

 private:
  std::weak_ptr<ContextVK> context_;

  Mutex recycled_mutex_;
  std::vector<vk::UniqueDescriptorPool> recycled_ IPLR_GUARDED_BY(
      recycled_mutex_);

  /// @brief      Creates a new |vk::CommandPool|.
  ///
  /// @returns    Returns a |std::nullopt| if a pool could not be created.
  vk::UniqueDescriptorPool Create();

  /// @brief      Reuses a recycled |vk::CommandPool|, if available.
  ///
  /// @returns    Returns a |std::nullopt| if a pool was not available.
  std::optional<vk::UniqueDescriptorPool> Reuse();

  DescriptorPoolRecyclerVK(const DescriptorPoolRecyclerVK&) = delete;

  DescriptorPoolRecyclerVK& operator=(const DescriptorPoolRecyclerVK&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_DESCRIPTOR_POOL_VK_H_
