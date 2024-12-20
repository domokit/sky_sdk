// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SHADER_FUNCTION_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SHADER_FUNCTION_VK_H_

#include "impeller/base/backend_cast.h"
#include "impeller/renderer/backend/vulkan/device_holder_vk.h"
#include "impeller/renderer/backend/vulkan/shader_function_vk.h"
#include "impeller/renderer/backend/vulkan/vk.h"
#include "impeller/renderer/shader_function.h"

namespace impeller {

class ShaderFunctionVK final
    : public ShaderFunction,
      public BackendCast<ShaderFunctionVK, ShaderFunction> {
 public:
  // |ShaderFunction|
  ~ShaderFunctionVK() override;

  const vk::ShaderModule& GetModule() const;

 private:
  friend class ShaderLibraryVK;

  vk::UniqueShaderModule module_;
  std::weak_ptr<DeviceHolderVK> device_holder_;

  ShaderFunctionVK(const std::weak_ptr<DeviceHolderVK>& device_holder,
                   UniqueID parent_library_id,
                   std::string name,
                   ShaderStage stage,
                   vk::UniqueShaderModule module);

  ShaderFunctionVK(const ShaderFunctionVK&) = delete;

  ShaderFunctionVK& operator=(const ShaderFunctionVK&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SHADER_FUNCTION_VK_H_
