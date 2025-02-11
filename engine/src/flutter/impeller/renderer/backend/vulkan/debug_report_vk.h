// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_DEBUG_REPORT_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_DEBUG_REPORT_VK_H_

#include "impeller/renderer/backend/vulkan/vk.h"

namespace impeller {

class CapabilitiesVK;

class DebugReportVK {
 public:
  DebugReportVK(const CapabilitiesVK& caps, const vk::Instance& instance);

  ~DebugReportVK();

  bool IsValid() const;

 private:
  vk::UniqueDebugUtilsMessengerEXT messenger_;
  bool is_valid_ = false;

  enum class Result {
    kContinue,
    kAbort,
  };

  Result OnDebugCallback(vk::DebugUtilsMessageSeverityFlagBitsEXT severity,
                         vk::DebugUtilsMessageTypeFlagsEXT type,
                         const vk::DebugUtilsMessengerCallbackDataEXT* data);

  static VKAPI_ATTR VkBool32 VKAPI_CALL DebugUtilsMessengerCallback(
      vk::DebugUtilsMessageSeverityFlagBitsEXT severity,
      vk::DebugUtilsMessageTypeFlagsEXT type,
      const vk::DebugUtilsMessengerCallbackDataEXT* callback_data,
      void* user_data);

  DebugReportVK(const DebugReportVK&) = delete;

  DebugReportVK& operator=(const DebugReportVK&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_DEBUG_REPORT_VK_H_
