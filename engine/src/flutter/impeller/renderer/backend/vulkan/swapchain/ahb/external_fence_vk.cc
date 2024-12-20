// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/swapchain/ahb/external_fence_vk.h"

#include "impeller/base/validation.h"
#include "impeller/renderer/backend/vulkan/context_vk.h"

namespace impeller {

ExternalFenceVK::ExternalFenceVK(const std::shared_ptr<Context>& context) {
  if (!context) {
    return;
  }
  vk::StructureChain<vk::FenceCreateInfo, vk::ExportFenceCreateInfoKHR> info;

  info.get<vk::ExportFenceCreateInfoKHR>().handleTypes =
      vk::ExternalFenceHandleTypeFlagBits::eSyncFd;

  const auto& context_vk = ContextVK::Cast(*context);
  auto [result, fence] = context_vk.GetDevice().createFenceUnique(info.get());
  if (result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not create external fence: "
                   << vk::to_string(result);
    return;
  }

  context_vk.SetDebugName(fence.get(), "ExternalFenceSyncFD");

  fence_ = MakeSharedVK(std::move(fence));
}

ExternalFenceVK::~ExternalFenceVK() = default;

bool ExternalFenceVK::IsValid() const {
  return !!fence_;
}

fml::UniqueFD ExternalFenceVK::CreateFD() const {
  if (!IsValid()) {
    return {};
  }
  vk::FenceGetFdInfoKHR info;
  info.fence = fence_->Get();
  info.handleType = vk::ExternalFenceHandleTypeFlagBits::eSyncFd;
  auto [result, fd] = fence_->GetUniqueWrapper().getOwner().getFenceFdKHR(info);
  if (result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not export external fence FD: "
                   << vk::to_string(result);
    return {};
  }
  return fml::UniqueFD{fd};
}

const vk::Fence& ExternalFenceVK::GetHandle() const {
  return fence_->Get();
}

const SharedHandleVK<vk::Fence>& ExternalFenceVK::GetSharedHandle() const {
  return fence_;
}

}  // namespace impeller
