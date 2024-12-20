// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/shader_function_vk.h"

namespace impeller {

ShaderFunctionVK::ShaderFunctionVK(
    const std::weak_ptr<DeviceHolderVK>& device_holder,
    UniqueID parent_library_id,
    std::string name,
    ShaderStage stage,
    vk::UniqueShaderModule module)
    : ShaderFunction(parent_library_id, std::move(name), stage),
      module_(std::move(module)),
      device_holder_(device_holder) {}

ShaderFunctionVK::~ShaderFunctionVK() {
  std::shared_ptr<DeviceHolderVK> device_holder = device_holder_.lock();
  if (device_holder) {
    module_.reset();
  } else {
    module_.release();
  }
}

const vk::ShaderModule& ShaderFunctionVK::GetModule() const {
  return module_.get();
}

}  // namespace impeller
