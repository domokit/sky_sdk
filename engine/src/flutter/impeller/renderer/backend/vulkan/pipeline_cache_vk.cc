// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// FLUTTER_NOLINT: https://github.com/flutter/flutter/issues/123467

#include "impeller/renderer/backend/vulkan/pipeline_cache_vk.h"

#include <sstream>

#include "flutter/fml/mapping.h"
#include "impeller/base/allocation_size.h"
#include "impeller/base/validation.h"
#include "impeller/renderer/backend/vulkan/pipeline_cache_data_vk.h"

namespace impeller {

PipelineCacheVK::PipelineCacheVK(std::shared_ptr<const Capabilities> caps,
                                 std::shared_ptr<DeviceHolderVK> device_holder,
                                 fml::UniqueFD cache_directory)
    : caps_(std::move(caps)),
      device_holder_(device_holder),
      cache_directory_(std::move(cache_directory)) {
  if (!caps_ || !device_holder->GetDevice()) {
    return;
  }

  const auto& vk_caps = CapabilitiesVK::Cast(*caps_);

  auto existing_cache_data = PipelineCacheDataRetrieve(
      cache_directory_, vk_caps.GetPhysicalDeviceProperties());

  vk::PipelineCacheCreateInfo cache_info;
  if (existing_cache_data) {
    cache_info.initialDataSize = existing_cache_data->GetSize();
    cache_info.pInitialData = existing_cache_data->GetMapping();
  }

  auto [result, existing_cache] =
      device_holder->GetDevice().createPipelineCacheUnique(cache_info);

  if (result == vk::Result::eSuccess) {
    cache_ = std::move(existing_cache);
  } else {
    // Even though we perform consistency checks because we don't trust the
    // driver, the driver may have additional information that may cause it to
    // reject the cache too.
    FML_LOG(INFO) << "Existing pipeline cache was invalid: "
                  << vk::to_string(result) << ". Starting with a fresh cache.";
    cache_info.pInitialData = nullptr;
    cache_info.initialDataSize = 0u;
    auto [result2, new_cache] =
        device_holder->GetDevice().createPipelineCacheUnique(cache_info);
    if (result2 == vk::Result::eSuccess) {
      cache_ = std::move(new_cache);
    } else {
      VALIDATION_LOG << "Could not create new pipeline cache: "
                     << vk::to_string(result2);
    }
  }

  is_valid_ = !!cache_;
}

PipelineCacheVK::~PipelineCacheVK() {
  std::shared_ptr<DeviceHolderVK> device_holder = device_holder_.lock();
  if (device_holder) {
    cache_.reset();
  } else {
    cache_.release();
  }
}

bool PipelineCacheVK::IsValid() const {
  return is_valid_;
}

vk::UniquePipeline PipelineCacheVK::CreatePipeline(
    const vk::GraphicsPipelineCreateInfo& info) {
  std::shared_ptr<DeviceHolderVK> strong_device = device_holder_.lock();
  if (!strong_device) {
    return {};
  }

  auto [result, pipeline] =
      strong_device->GetDevice().createGraphicsPipelineUnique(*cache_, info);
  if (result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not create graphics pipeline: "
                   << vk::to_string(result);
  }
  return std::move(pipeline);
}

vk::UniquePipeline PipelineCacheVK::CreatePipeline(
    const vk::ComputePipelineCreateInfo& info) {
  std::shared_ptr<DeviceHolderVK> strong_device = device_holder_.lock();
  if (!strong_device) {
    return {};
  }

  auto [result, pipeline] =
      strong_device->GetDevice().createComputePipelineUnique(*cache_, info);
  if (result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not create compute pipeline: "
                   << vk::to_string(result);
  }
  return std::move(pipeline);
}

void PipelineCacheVK::PersistCacheToDisk() const {
  if (!is_valid_) {
    return;
  }
  const auto& vk_caps = CapabilitiesVK::Cast(*caps_);
  PipelineCacheDataPersist(cache_directory_,                       //
                           vk_caps.GetPhysicalDeviceProperties(),  //
                           cache_                                  //
  );
}

const CapabilitiesVK* PipelineCacheVK::GetCapabilities() const {
  return CapabilitiesVK::Cast(caps_.get());
}

}  // namespace impeller
