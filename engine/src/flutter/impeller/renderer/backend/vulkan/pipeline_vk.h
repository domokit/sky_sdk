// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_PIPELINE_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_PIPELINE_VK_H_

#include <future>
#include <memory>

#include "impeller/base/backend_cast.h"
#include "impeller/base/thread.h"
#include "impeller/core/texture.h"
#include "impeller/renderer/backend/vulkan/device_holder_vk.h"
#include "impeller/renderer/backend/vulkan/formats_vk.h"
#include "impeller/renderer/backend/vulkan/pipeline_cache_vk.h"
#include "impeller/renderer/backend/vulkan/sampler_vk.h"
#include "impeller/renderer/backend/vulkan/vk.h"
#include "impeller/renderer/backend/vulkan/yuv_conversion_vk.h"
#include "impeller/renderer/pipeline.h"

namespace impeller {

// Limit on the total number of buffer and image bindings that allow the Vulkan
// backend to avoid dynamic heap allocations.
static constexpr size_t kMaxBindings = 32;

class PipelineVK final
    : public Pipeline<PipelineDescriptor>,
      public BackendCast<PipelineVK, Pipeline<PipelineDescriptor>> {
 public:
  static std::unique_ptr<PipelineVK> Create(
      const PipelineDescriptor& desc,
      const std::shared_ptr<DeviceHolderVK>& device_holder,
      const std::weak_ptr<PipelineLibrary>& weak_library,
      std::shared_ptr<SamplerVK> immutable_sampler = {});

  // |Pipeline|
  ~PipelineVK() override;

  vk::Pipeline GetPipeline() const;

  const vk::PipelineLayout& GetPipelineLayout() const;

  const vk::DescriptorSetLayout& GetDescriptorSetLayout() const;

  std::shared_ptr<PipelineVK> CreateVariantForImmutableSamplers(
      const std::shared_ptr<SamplerVK>& immutable_sampler) const;

 private:
  friend class PipelineLibraryVK;

  using ImmutableSamplerVariants =
      std::unordered_map<ImmutableSamplerKeyVK,
                         std::shared_ptr<PipelineVK>,
                         ComparableHash<ImmutableSamplerKeyVK>,
                         ComparableEqual<ImmutableSamplerKeyVK>>;

  std::weak_ptr<DeviceHolderVK> device_holder_;
  vk::UniquePipeline pipeline_;
  vk::UniqueRenderPass render_pass_;
  vk::UniquePipelineLayout layout_;
  vk::UniqueDescriptorSetLayout descriptor_set_layout_;
  std::shared_ptr<SamplerVK> immutable_sampler_;
  mutable Mutex immutable_sampler_variants_mutex_;
  mutable ImmutableSamplerVariants immutable_sampler_variants_ IPLR_GUARDED_BY(
      immutable_sampler_variants_mutex_);
  bool is_valid_ = false;

  PipelineVK(std::weak_ptr<DeviceHolderVK> device_holder,
             std::weak_ptr<PipelineLibrary> library,
             const PipelineDescriptor& desc,
             vk::UniquePipeline pipeline,
             vk::UniqueRenderPass render_pass,
             vk::UniquePipelineLayout layout,
             vk::UniqueDescriptorSetLayout descriptor_set_layout,
             std::shared_ptr<SamplerVK> immutable_sampler);

  // |Pipeline|
  bool IsValid() const override;

  PipelineVK(const PipelineVK&) = delete;

  PipelineVK& operator=(const PipelineVK&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_PIPELINE_VK_H_
