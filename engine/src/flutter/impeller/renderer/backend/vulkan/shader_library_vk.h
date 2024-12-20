// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SHADER_LIBRARY_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SHADER_LIBRARY_VK_H_

#include "impeller/base/comparable.h"
#include "impeller/base/thread.h"
#include "impeller/renderer/backend/vulkan/device_holder_vk.h"
#include "impeller/renderer/backend/vulkan/vk.h"
#include "impeller/renderer/shader_key.h"
#include "impeller/renderer/shader_library.h"

namespace impeller {

class ShaderLibraryVK final : public ShaderLibrary {
 public:
  // |ShaderLibrary|
  ~ShaderLibraryVK() override;

  // |ShaderLibrary|
  bool IsValid() const override;

 private:
  friend class ContextVK;
  std::weak_ptr<DeviceHolderVK> device_holder_;
  const UniqueID library_id_;
  mutable RWMutex functions_mutex_;
  ShaderFunctionMap functions_ IPLR_GUARDED_BY(functions_mutex_);
  bool is_valid_ = false;

  ShaderLibraryVK(
      std::weak_ptr<DeviceHolderVK> device_holder,
      const std::vector<std::shared_ptr<fml::Mapping>>& shader_libraries_data);

  // |ShaderLibrary|
  std::shared_ptr<const ShaderFunction> GetFunction(std::string_view name,
                                                    ShaderStage stage) override;

  // |ShaderLibrary|
  void RegisterFunction(std::string name,
                        ShaderStage stage,
                        std::shared_ptr<fml::Mapping> code,
                        RegistrationCallback callback) override;

  bool RegisterFunction(const std::string& name,
                        ShaderStage stage,
                        const std::shared_ptr<fml::Mapping>& code);

  // |ShaderLibrary|
  void UnregisterFunction(std::string name, ShaderStage stage) override;

  ShaderLibraryVK(const ShaderLibraryVK&) = delete;

  ShaderLibraryVK& operator=(const ShaderLibraryVK&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_SHADER_LIBRARY_VK_H_
