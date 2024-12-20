// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_COMPUTE_PIPELINE_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_COMPUTE_PIPELINE_VK_H_

#include <memory>

#include "impeller/base/backend_cast.h"
#include "impeller/renderer/backend/vulkan/device_holder_vk.h"
#include "impeller/renderer/backend/vulkan/vk.h"
#include "impeller/renderer/pipeline.h"

namespace impeller {

class ComputePipelineVK final
    : public Pipeline<ComputePipelineDescriptor>,
      public BackendCast<ComputePipelineVK,
                         Pipeline<ComputePipelineDescriptor>> {
 public:
  ComputePipelineVK(std::weak_ptr<DeviceHolderVK> device_holder,
                    std::weak_ptr<PipelineLibrary> library,
                    const ComputePipelineDescriptor& desc,
                    vk::UniquePipeline pipeline,
                    vk::UniquePipelineLayout layout,
                    vk::UniqueDescriptorSetLayout descriptor_set_layout);

  // |Pipeline|
  ~ComputePipelineVK() override;

  const vk::Pipeline& GetPipeline() const;

  const vk::PipelineLayout& GetPipelineLayout() const;

  const vk::DescriptorSetLayout& GetDescriptorSetLayout() const;

 private:
  friend class PipelineLibraryVK;

  std::weak_ptr<DeviceHolderVK> device_holder_;
  vk::UniquePipeline pipeline_;
  vk::UniquePipelineLayout layout_;
  vk::UniqueDescriptorSetLayout descriptor_set_layout_;
  bool is_valid_ = false;

  // |Pipeline|
  bool IsValid() const override;

  ComputePipelineVK(const ComputePipelineVK&) = delete;

  ComputePipelineVK& operator=(const ComputePipelineVK&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_COMPUTE_PIPELINE_VK_H_
