// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/pipeline_cache_data_vk.h"

#include "flutter/fml/file.h"
#include "impeller/base/allocation.h"
#include "impeller/base/validation.h"

namespace impeller {

static constexpr const char* kPipelineCacheFileName =
    "flutter.impeller.vkcache";

bool PipelineCacheDataPersist(const fml::UniqueFD& cache_directory,
                              const VkPhysicalDeviceProperties& props,
                              const vk::UniquePipelineCache& cache) {
  if (!cache_directory.is_valid()) {
    return false;
  }
  size_t data_size = 0u;
  if (cache.getOwner().getPipelineCacheData(*cache, &data_size, nullptr) !=
      vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not fetch pipeline cache size.";
    return false;
  }
  if (data_size == 0u) {
    return true;
  }
  auto allocation = std::make_shared<Allocation>();
  if (!allocation->Truncate(Bytes{sizeof(PipelineCacheHeaderVK) + data_size},
                            false)) {
    VALIDATION_LOG << "Could not allocate pipeline cache data staging buffer.";
    return false;
  }
  const auto header = PipelineCacheHeaderVK{props, data_size};
  std::memcpy(allocation->GetBuffer(), &header, sizeof(header));
  if (cache.getOwner().getPipelineCacheData(
          *cache, &data_size, allocation->GetBuffer() + sizeof(header)) !=
      vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not copy pipeline cache data.";
    return false;
  }

  auto allocation_mapping = CreateMappingFromAllocation(allocation);
  if (!allocation_mapping) {
    return false;
  }
  if (!fml::WriteAtomically(cache_directory, kPipelineCacheFileName,
                            *allocation_mapping)) {
    VALIDATION_LOG << "Could not write cache file to disk.";
    return false;
  }
  return true;
}

std::unique_ptr<fml::Mapping> PipelineCacheDataRetrieve(
    const fml::UniqueFD& cache_directory,
    const VkPhysicalDeviceProperties& props) {
  if (!cache_directory.is_valid()) {
    return nullptr;
  }
  std::shared_ptr<fml::FileMapping> on_disk_data =
      fml::FileMapping::CreateReadOnly(cache_directory, kPipelineCacheFileName);
  if (!on_disk_data) {
    return nullptr;
  }
  if (on_disk_data->GetSize() < sizeof(PipelineCacheHeaderVK)) {
    VALIDATION_LOG << "Pipeline cache data size is too small.";
    return nullptr;
  }
  auto on_disk_header = PipelineCacheHeaderVK{};
  std::memcpy(&on_disk_header,             //
              on_disk_data->GetMapping(),  //
              sizeof(on_disk_header)       //
  );
  const auto current_header = PipelineCacheHeaderVK{props, 0u};
  if (!on_disk_header.IsCompatibleWith(current_header)) {
    FML_LOG(WARNING)
        << "Persisted pipeline cache is not compatible with current "
           "Vulkan context. Ignoring.";
    return nullptr;
  }
  // Zero sized data is known to cause issues.
  if (on_disk_header.data_size == 0u) {
    return nullptr;
  }
  return std::make_unique<fml::NonOwnedMapping>(
      on_disk_data->GetMapping() + sizeof(on_disk_header),
      on_disk_header.data_size, [on_disk_data](auto, auto) {});
}

PipelineCacheHeaderVK::PipelineCacheHeaderVK() = default;

PipelineCacheHeaderVK::PipelineCacheHeaderVK(
    const VkPhysicalDeviceProperties& props,
    uint64_t p_data_size)
    : driver_version(props.driverVersion),
      vendor_id(props.vendorID),
      device_id(props.deviceID),
      data_size(p_data_size) {
  std::memcpy(uuid, props.pipelineCacheUUID, VK_UUID_SIZE);
}

bool PipelineCacheHeaderVK::IsCompatibleWith(
    const PipelineCacheHeaderVK& o) const {
  // Check for everything but the data size.
  return magic == o.magic &&                    //
         driver_version == o.driver_version &&  //
         vendor_id == o.vendor_id &&            //
         device_id == o.device_id &&            //
         abi == o.abi &&                        //
         std::memcmp(uuid, o.uuid, VK_UUID_SIZE) == 0;
}

}  // namespace impeller
