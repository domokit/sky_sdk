// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_COMMAND_POOL_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_COMMAND_POOL_VK_H_

#include <memory>
#include <optional>
#include <utility>

#include "impeller/base/thread.h"
#include "impeller/renderer/backend/vulkan/vk.h"  // IWYU pragma: keep.
#include "vulkan/vulkan_handles.hpp"

namespace impeller {

class ContextVK;
class CommandPoolRecyclerVK;

//------------------------------------------------------------------------------
/// @brief      Manages the lifecycle of a single |vk::CommandPool|.
///
/// A |vk::CommandPool| is expensive to create and reset. This class manages
/// the lifecycle of a single |vk::CommandPool| by returning to the origin
/// (|CommandPoolRecyclerVK|) when it is destroyed to be reused.
///
/// @warning    This class is not thread-safe.
///
/// @see        |CommandPoolRecyclerVK|
class CommandPoolVK final {
 public:
  ~CommandPoolVK();

  /// @brief      Creates a resource that manages the life of a command pool.
  ///
  /// @param[in]  pool      The command pool to manage.
  /// @param[in]  buffers   Zero or more command buffers in an initial state.
  /// @param[in]  recycler  The context that will be notified on destruction.
  CommandPoolVK(vk::UniqueCommandPool pool,
                std::vector<vk::UniqueCommandBuffer>&& buffers,
                std::weak_ptr<ContextVK>& context)
      : pool_(std::move(pool)),
        unused_command_buffers_(std::move(buffers)),
        context_(context) {}

  /// @brief      Creates and returns a new |vk::CommandBuffer|.
  ///
  /// @return     Always returns a new |vk::CommandBuffer|, but if for any
  ///             reason a valid command buffer could not be created, it will be
  ///             a `{}` default instance (i.e. while being torn down).
  vk::UniqueCommandBuffer CreateCommandBuffer();

  /// @brief      Collects the given |vk::CommandBuffer| to be retained.
  ///
  /// @param[in]  buffer  The |vk::CommandBuffer| to collect.
  ///
  /// @see        |GarbageCollectBuffersIfAble|
  void CollectCommandBuffer(vk::UniqueCommandBuffer&& buffer);

  /// @brief      Delete all Vulkan objects in this command pool.
  void Destroy();

 private:
  CommandPoolVK(const CommandPoolVK&) = delete;

  CommandPoolVK& operator=(const CommandPoolVK&) = delete;

  Mutex pool_mutex_;
  vk::UniqueCommandPool pool_ IPLR_GUARDED_BY(pool_mutex_);
  std::vector<vk::UniqueCommandBuffer> unused_command_buffers_;
  std::weak_ptr<ContextVK>& context_;

  // Used to retain a reference on these until the pool is reset.
  std::vector<vk::UniqueCommandBuffer> collected_buffers_ IPLR_GUARDED_BY(
      pool_mutex_);
};

//------------------------------------------------------------------------------
/// @brief      Creates and manages the lifecycle of |vk::CommandPool| objects.
///
/// A |vk::CommandPool| is expensive to create and reset. This class manages
/// the lifecycle of |vk::CommandPool| objects by creating and recycling them;
/// or in other words, a pool for command pools.
///
/// A single instance should be created per |ContextVK|.
///
/// Every "frame", a single |CommandPoolResourceVk| is made available for each
/// thread that calls |Get|. After calling |Dispose|, the current thread's pool
/// is moved to a background thread, reset, and made available for the next time
/// |Get| is called and needs to create a command pool.
///
/// Commands in the command pool are not necessarily done executing when the
/// pool is recycled, when all references are dropped to the pool, they are
/// reset and returned to the pool of available pools.
///
/// @note       This class is thread-safe.
///
/// @see        |vk::CommandPoolResourceVk|
/// @see        |ContextVK|
/// @see
/// https://arm-software.github.io/vulkan_best_practice_for_mobile_developers/samples/performance/command_buffer_usage/command_buffer_usage_tutorial.html
class CommandPoolRecyclerVK final
    : public std::enable_shared_from_this<CommandPoolRecyclerVK> {
 public:
  ~CommandPoolRecyclerVK();

  /// A unique command pool and zero or more recycled command buffers.
  struct RecycledData {
    vk::UniqueCommandPool pool;
    std::vector<vk::UniqueCommandBuffer> buffers;
  };

  /// @brief      Clean up resources held by all per-thread command pools
  ///             associated with the given context.
  ///
  /// @param[in]  context The context.
  static void DestroyThreadLocalPools(const ContextVK* context);

  /// @brief      Creates a recycler for the given |ContextVK|.
  ///
  /// @param[in]  context The context to create the recycler for.
  explicit CommandPoolRecyclerVK(std::weak_ptr<ContextVK> context)
      : context_(std::move(context)) {}

  /// @brief      Gets a command pool for the current thread.
  ///
  /// @warning    Returns a |nullptr| if a pool could not be created.
  std::shared_ptr<CommandPoolVK> Get();

  /// @brief      Returns a command pool to be reset on a background thread.
  ///
  /// @param[in]  pool The pool to recycler.
  void Reclaim(vk::UniqueCommandPool&& pool,
               std::vector<vk::UniqueCommandBuffer>&& buffers);

  /// @brief      Clears all recycled command pools to let them be reclaimed.
  void Dispose();

 private:
  std::weak_ptr<ContextVK> context_;

  Mutex recycled_mutex_;
  std::vector<RecycledData> recycled_ IPLR_GUARDED_BY(recycled_mutex_);

  /// @brief      Creates a new |vk::CommandPool|.
  ///
  /// @returns    Returns a |std::nullopt| if a pool could not be created.
  std::optional<CommandPoolRecyclerVK::RecycledData> Create();

  /// @brief      Reuses a recycled |RecycledData|, if available.
  ///
  /// @returns    Returns a |std::nullopt| if a pool was not available.
  std::optional<RecycledData> Reuse();

  CommandPoolRecyclerVK(const CommandPoolRecyclerVK&) = delete;

  CommandPoolRecyclerVK& operator=(const CommandPoolRecyclerVK&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_COMMAND_POOL_VK_H_
