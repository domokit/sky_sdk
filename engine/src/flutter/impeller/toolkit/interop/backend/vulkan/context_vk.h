// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_VULKAN_CONTEXT_VK_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_VULKAN_CONTEXT_VK_H_

#include "impeller/renderer/backend/vulkan/vk.h"
#include "impeller/toolkit/interop/context.h"
#include "impeller/toolkit/interop/impeller.h"

namespace impeller::interop {

class ContextVK final : public Context {
 public:
  struct Settings {
    std::function<PFN_vkVoidFunction(VkInstance instance,
                                     const char* proc_name)>
        instance_proc_address_callback;
    bool enable_validation = false;

    explicit Settings(const ImpellerContextVulkanSettings& settings);

    bool IsValid() const;
  };

  static ScopedObject<Context> Create(const Settings& settings);

  static ScopedObject<Context> Create(
      std::shared_ptr<impeller::Context> context);

  // |Context|
  ~ContextVK() override;

  ContextVK(const ContextVK&) = delete;

  ContextVK& operator=(const ContextVK&) = delete;

  bool GetInfo(ImpellerContextVulkanInfo& info) const;

 private:
  explicit ContextVK(std::shared_ptr<impeller::Context> context);
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_BACKEND_VULKAN_CONTEXT_VK_H_
