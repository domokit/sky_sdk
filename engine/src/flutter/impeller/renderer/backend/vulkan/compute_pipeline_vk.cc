// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/compute_pipeline_vk.h"

namespace impeller {

ComputePipelineVK::ComputePipelineVK(
    std::weak_ptr<DeviceHolderVK> device_holder,
    std::weak_ptr<PipelineLibrary> library,
    const ComputePipelineDescriptor& desc,
    vk::UniquePipeline pipeline,
    vk::UniquePipelineLayout layout,
    vk::UniqueDescriptorSetLayout descriptor_set_layout)
    : Pipeline(std::move(library), desc),
      device_holder_(std::move(device_holder)),
      pipeline_(std::move(pipeline)),
      layout_(std::move(layout)),
      descriptor_set_layout_(std::move(descriptor_set_layout)) {
  is_valid_ = pipeline_ && layout_ && descriptor_set_layout_;
}

ComputePipelineVK::~ComputePipelineVK() {
  std::shared_ptr<DeviceHolderVK> device_holder = device_holder_.lock();
  if (device_holder) {
    descriptor_set_layout_.reset();
    layout_.reset();
    pipeline_.reset();
  } else {
    descriptor_set_layout_.release();
    layout_.release();
    pipeline_.release();
  }
}

bool ComputePipelineVK::IsValid() const {
  return is_valid_;
}

const vk::Pipeline& ComputePipelineVK::GetPipeline() const {
  return *pipeline_;
}

const vk::PipelineLayout& ComputePipelineVK::GetPipelineLayout() const {
  return *layout_;
}

const vk::DescriptorSetLayout& ComputePipelineVK::GetDescriptorSetLayout()
    const {
  return *descriptor_set_layout_;
}

}  // namespace impeller
