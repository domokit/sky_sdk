// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/descriptor_pool_vk.h"

#include <optional>

#include "impeller/base/validation.h"
#include "impeller/renderer/backend/vulkan/resource_manager_vk.h"
#include "vulkan/vulkan_enums.hpp"
#include "vulkan/vulkan_handles.hpp"

namespace impeller {

struct DescriptorPoolSize {
  size_t buffer_bindings;
  size_t texture_bindings;
  size_t storage_bindings;
  size_t subpass_bindings;
};

/// Descriptor pools are always allocated with the following sizes.
static const constexpr DescriptorPoolSize kDefaultBindingSize =
    DescriptorPoolSize{
        .buffer_bindings = 512u,   // Buffer Bindings
        .texture_bindings = 256u,  // Texture Bindings
        .storage_bindings = 32,
        .subpass_bindings = 4u  // Subpass Bindings
    };

// Holds the command pool in a background thread, recyling it when not in use.
class BackgroundDescriptorPoolVK final {
 public:
  BackgroundDescriptorPoolVK(BackgroundDescriptorPoolVK&&) = default;

  explicit BackgroundDescriptorPoolVK(
      vk::UniqueDescriptorPool&& pool,
      std::weak_ptr<DescriptorPoolRecyclerVK> recycler)
      : pool_(std::move(pool)), recycler_(std::move(recycler)) {}

  ~BackgroundDescriptorPoolVK() {
    auto const recycler = recycler_.lock();

    // Not only does this prevent recycling when the context is being destroyed,
    // but it also prevents the destructor from effectively being called twice;
    // once for the original BackgroundCommandPoolVK() and once for the moved
    // BackgroundCommandPoolVK().
    if (!recycler) {
      return;
    }

    recycler->Reclaim(std::move(pool_));
  }

 private:
  BackgroundDescriptorPoolVK(const BackgroundDescriptorPoolVK&) = delete;

  BackgroundDescriptorPoolVK& operator=(const BackgroundDescriptorPoolVK&) =
      delete;

  vk::UniqueDescriptorPool pool_;
  uint32_t allocated_capacity_;
  std::weak_ptr<DescriptorPoolRecyclerVK> recycler_;
};

DescriptorPoolVK::DescriptorPoolVK(std::weak_ptr<const ContextVK> context)
    : context_(std::move(context)) {}

DescriptorPoolVK::~DescriptorPoolVK() {
  if (pools_.empty()) {
    return;
  }

  auto const context = context_.lock();
  if (!context) {
    return;
  }
  auto const recycler = context->GetDescriptorPoolRecycler();
  if (!recycler) {
    return;
  }

  for (auto i = 0u; i < pools_.size(); i++) {
    auto reset_pool_when_dropped =
        BackgroundDescriptorPoolVK(std::move(pools_[i]), recycler);

    UniqueResourceVKT<BackgroundDescriptorPoolVK> pool(
        context->GetResourceManager(), std::move(reset_pool_when_dropped));
  }
  pools_.clear();
}

fml::StatusOr<vk::DescriptorSet> DescriptorPoolVK::AllocateDescriptorSets(
    const vk::DescriptorSetLayout& layout,
    const ContextVK& context_vk) {
  if (pools_.empty()) {
    CreateNewPool(context_vk);
  }

  vk::DescriptorSetAllocateInfo set_info;
  set_info.setDescriptorPool(pools_.back().get());
  set_info.setPSetLayouts(&layout);
  set_info.setDescriptorSetCount(1);

  vk::DescriptorSet set;
  auto result = context_vk.GetDevice().allocateDescriptorSets(&set_info, &set);
  if (result == vk::Result::eErrorOutOfPoolMemory) {
    // If the pool ran out of memory, we need to create a new pool.
    CreateNewPool(context_vk);
    set_info.setDescriptorPool(pools_.back().get());
    result = context_vk.GetDevice().allocateDescriptorSets(&set_info, &set);
  }

  if (result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not allocate descriptor sets: "
                   << vk::to_string(result);
    return fml::Status(fml::StatusCode::kUnknown, "");
  }
  return set;
}

fml::Status DescriptorPoolVK::CreateNewPool(const ContextVK& context_vk) {
  auto new_pool = context_vk.GetDescriptorPoolRecycler()->Get();
  if (!new_pool) {
    return fml::Status(fml::StatusCode::kUnknown,
                       "Failed to create descriptor pool");
  }
  pools_.emplace_back(std::move(new_pool));
  return fml::Status();
}

void DescriptorPoolRecyclerVK::Reclaim(vk::UniqueDescriptorPool&& pool) {
  // Reset the pool on a background thread.
  auto strong_context = context_.lock();
  if (!strong_context) {
    return;
  }
  auto device = strong_context->GetDevice();
  device.resetDescriptorPool(pool.get());

  // Move the pool to the recycled list.
  Lock recycled_lock(recycled_mutex_);

  if (recycled_.size() < kMaxRecycledPools) {
    recycled_.push_back(std::move(pool));
    return;
  }
}

vk::UniqueDescriptorPool DescriptorPoolRecyclerVK::Get() {
  // Recycle a pool with a matching minumum capcity if it is available.
  auto recycled_pool = Reuse();
  if (recycled_pool.has_value()) {
    return std::move(recycled_pool.value());
  }
  return Create();
}

vk::UniqueDescriptorPool DescriptorPoolRecyclerVK::Create() {
  auto strong_context = context_.lock();
  if (!strong_context) {
    VALIDATION_LOG << "Unable to create a descriptor pool";
    return {};
  }

  std::vector<vk::DescriptorPoolSize> pools = {
      vk::DescriptorPoolSize{vk::DescriptorType::eCombinedImageSampler,
                             kDefaultBindingSize.texture_bindings},
      vk::DescriptorPoolSize{vk::DescriptorType::eUniformBuffer,
                             kDefaultBindingSize.buffer_bindings},
      vk::DescriptorPoolSize{vk::DescriptorType::eStorageBuffer,
                             kDefaultBindingSize.storage_bindings},
      vk::DescriptorPoolSize{vk::DescriptorType::eInputAttachment,
                             kDefaultBindingSize.subpass_bindings}};
  vk::DescriptorPoolCreateInfo pool_info;
  pool_info.setMaxSets(kDefaultBindingSize.texture_bindings +
                       kDefaultBindingSize.buffer_bindings +
                       kDefaultBindingSize.storage_bindings +
                       kDefaultBindingSize.subpass_bindings);
  pool_info.setPoolSizes(pools);
  auto [result, pool] =
      strong_context->GetDevice().createDescriptorPoolUnique(pool_info);
  if (result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Unable to create a descriptor pool";
  }
  return std::move(pool);
}

std::optional<vk::UniqueDescriptorPool> DescriptorPoolRecyclerVK::Reuse() {
  Lock lock(recycled_mutex_);
  if (recycled_.empty()) {
    return std::nullopt;
  }

  auto recycled = std::move(recycled_[recycled_.size() - 1]);
  recycled_.pop_back();
  return recycled;
}

}  // namespace impeller
