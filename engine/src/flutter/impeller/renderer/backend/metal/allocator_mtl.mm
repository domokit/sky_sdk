// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/metal/allocator_mtl.h"

#include "flutter/fml/build_config.h"
#include "flutter/fml/logging.h"
#include "fml/trace_event.h"
#include "impeller/base/allocation_size.h"
#include "impeller/base/validation.h"
#include "impeller/core/formats.h"
#include "impeller/renderer/backend/metal/device_buffer_mtl.h"
#include "impeller/renderer/backend/metal/formats_mtl.h"
#include "impeller/renderer/backend/metal/texture_mtl.h"

namespace impeller {

static bool DeviceSupportsDeviceTransientTargets(id<MTLDevice> device) {
  // Refer to the "Memoryless render targets" feature in the table below:
  // https://developer.apple.com/metal/Metal-Feature-Set-Tables.pdf
  if (@available(ios 13.0, tvos 13.0, macos 10.15, *)) {
    return [device supportsFamily:MTLGPUFamilyApple2];
  } else {
#if FML_OS_IOS
    // This is perhaps redundant. But, just in case we somehow get into a case
    // where Impeller runs on iOS versions less than 8.0 and/or without A8
    // GPUs, we explicitly check feature set support.
    return [device supportsFeatureSet:MTLFeatureSet_iOS_GPUFamily2_v1];
#else
    // MacOS devices with Apple GPUs are only available with macos 10.15 and
    // above. So, if we are here, it is safe to assume that memory-less targets
    // are not supported.
    return false;
#endif
  }
  FML_UNREACHABLE();
}

static bool DeviceHasUnifiedMemoryArchitecture(id<MTLDevice> device) {
  if (@available(ios 13.0, tvos 13.0, macOS 10.15, *)) {
    return [device hasUnifiedMemory];
  } else {
#if FML_OS_IOS
    // iOS devices where the availability check can fail always have had UMA.
    return true;
#else
    // Mac devices where the availability check can fail have never had UMA.
    return false;
#endif
  }
  FML_UNREACHABLE();
}

ISize DeviceMaxTextureSizeSupported(id<MTLDevice> device) {
  // Since Apple didn't expose API for us to get the max texture size, we have
  // to use hardcoded data from
  // https://developer.apple.com/metal/Metal-Feature-Set-Tables.pdf
  // According to the feature set table, there are two supported max sizes :
  // 16384 and 8192 for devices flutter support. The former is used on macs and
  // latest ios devices. The latter is used on old ios devices.
  if (@available(macOS 10.15, iOS 13, tvOS 13, *)) {
    if ([device supportsFamily:MTLGPUFamilyApple3] ||
        [device supportsFamily:MTLGPUFamilyMacCatalyst1] ||
        [device supportsFamily:MTLGPUFamilyMac1]) {
      return {16384, 16384};
    }
    return {8192, 8192};
  } else {
#if FML_OS_IOS
    if ([device supportsFeatureSet:MTLFeatureSet_iOS_GPUFamily4_v1] ||
        [device supportsFeatureSet:MTLFeatureSet_iOS_GPUFamily3_v1]) {
      return {16384, 16384};
    }
#endif
#if FML_OS_MACOSX
    return {16384, 16384};
#endif
    return {8192, 8192};
  }
}

static bool SupportsLossyTextureCompression(id<MTLDevice> device) {
#ifdef FML_OS_IOS_SIMULATOR
  return false;
#else
  if (@available(macOS 10.15, iOS 13, tvOS 13, *)) {
    return [device supportsFamily:MTLGPUFamilyApple8];
  }
  return false;
#endif
}

void DebugAllocatorStats::Increment(size_t size) {
  size_.fetch_add(size, std::memory_order_relaxed);
}

void DebugAllocatorStats::Decrement(size_t size) {
  size_.fetch_sub(size, std::memory_order_relaxed);
}

Bytes DebugAllocatorStats::GetAllocationSize() {
  return Bytes{size_.load()};
}

AllocatorMTL::AllocatorMTL(id<MTLDevice> device, std::string label)
    : device_(device), allocator_label_(std::move(label)) {
  if (!device_) {
    return;
  }

  supports_memoryless_targets_ = DeviceSupportsDeviceTransientTargets(device_);
  supports_uma_ = DeviceHasUnifiedMemoryArchitecture(device_);
  max_texture_supported_ = DeviceMaxTextureSizeSupported(device_);

  is_valid_ = true;
}

AllocatorMTL::~AllocatorMTL() = default;

bool AllocatorMTL::IsValid() const {
  return is_valid_;
}

static MTLResourceOptions ToMTLResourceOptions(StorageMode type,
                                               bool supports_memoryless_targets,
                                               bool supports_uma) {
  switch (type) {
    case StorageMode::kHostVisible:
#if FML_OS_IOS
      return MTLResourceStorageModeShared;
#else
      if (supports_uma) {
        return MTLResourceStorageModeShared;
      } else {
        return MTLResourceStorageModeManaged;
      }
#endif
    case StorageMode::kDevicePrivate:
      return MTLResourceStorageModePrivate;
    case StorageMode::kDeviceTransient:
      if (supports_memoryless_targets) {
        // Device may support but the OS has not been updated.
        if (@available(macOS 11.0, *)) {
          return MTLResourceStorageModeMemoryless;
        } else {
          return MTLResourceStorageModePrivate;
        }
      } else {
        return MTLResourceStorageModePrivate;
      }
      FML_UNREACHABLE();
  }
  FML_UNREACHABLE();
}

static MTLStorageMode ToMTLStorageMode(StorageMode mode,
                                       bool supports_memoryless_targets,
                                       bool supports_uma) {
  switch (mode) {
    case StorageMode::kHostVisible:
#if FML_OS_IOS
      return MTLStorageModeShared;
#else
      if (supports_uma) {
        return MTLStorageModeShared;
      } else {
        return MTLStorageModeManaged;
      }
#endif
    case StorageMode::kDevicePrivate:
      return MTLStorageModePrivate;
    case StorageMode::kDeviceTransient:
      if (supports_memoryless_targets) {
        // Device may support but the OS has not been updated.
        if (@available(macOS 11.0, *)) {
          return MTLStorageModeMemoryless;
        } else {
          return MTLStorageModePrivate;
        }
      } else {
        return MTLStorageModePrivate;
      }
      FML_UNREACHABLE();
  }
  FML_UNREACHABLE();
}

std::shared_ptr<DeviceBuffer> AllocatorMTL::OnCreateBuffer(
    const DeviceBufferDescriptor& desc) {
  const auto resource_options = ToMTLResourceOptions(
      desc.storage_mode, supports_memoryless_targets_, supports_uma_);
  const auto storage_mode = ToMTLStorageMode(
      desc.storage_mode, supports_memoryless_targets_, supports_uma_);

  auto buffer = [device_ newBufferWithLength:desc.size
                                     options:resource_options];
  if (!buffer) {
    return nullptr;
  }
  return std::shared_ptr<DeviceBufferMTL>(new DeviceBufferMTL(desc,         //
                                                              buffer,       //
                                                              storage_mode  //
                                                              ));
}

std::shared_ptr<Texture> AllocatorMTL::OnCreateTexture(
    const TextureDescriptor& desc) {
  if (!IsValid()) {
    return nullptr;
  }

  auto mtl_texture_desc = ToMTLTextureDescriptor(desc);

  if (!mtl_texture_desc) {
    VALIDATION_LOG << "Texture descriptor was invalid.";
    return nullptr;
  }

  mtl_texture_desc.storageMode = ToMTLStorageMode(
      desc.storage_mode, supports_memoryless_targets_, supports_uma_);

  if (@available(macOS 12.5, ios 15.0, *)) {
    if (desc.compression_type == CompressionType::kLossy &&
        SupportsLossyTextureCompression(device_)) {
      mtl_texture_desc.compressionType = MTLTextureCompressionTypeLossy;
    }
  }

#ifdef IMPELLER_DEBUG
  if (desc.storage_mode != StorageMode::kDeviceTransient) {
    debug_allocater_->Increment(desc.GetByteSizeOfAllMipLevels());
  }
#endif  // IMPELLER_DEBUG

  auto texture = [device_ newTextureWithDescriptor:mtl_texture_desc];
  if (!texture) {
    return nullptr;
  }
  std::shared_ptr<TextureMTL> result_texture =
      TextureMTL::Create(desc, texture);
#ifdef IMPELLER_DEBUG
  result_texture->SetDebugAllocator(debug_allocater_);
#endif  // IMPELLER_DEBUG

  return result_texture;
}

uint16_t AllocatorMTL::MinimumBytesPerRow(PixelFormat format) const {
  return static_cast<uint16_t>([device_
      minimumLinearTextureAlignmentForPixelFormat:ToMTLPixelFormat(format)]);
}

ISize AllocatorMTL::GetMaxTextureSizeSupported() const {
  return max_texture_supported_;
}

Bytes AllocatorMTL::DebugGetHeapUsage() const {
#ifdef IMPELLER_DEBUG
  return debug_allocater_->GetAllocationSize();
#else
  return {};
#endif  // IMPELLER_DEBUG
}

void AllocatorMTL::DebugTraceMemoryStatistics() const {
#ifdef IMPELLER_DEBUG
  FML_TRACE_COUNTER("flutter", "AllocatorMTL",
                    reinterpret_cast<int64_t>(this),  // Trace Counter ID
                    "MemoryBudgetUsageMB",
                    DebugGetHeapUsage().ConvertTo<MebiBytes>().GetSize());
#endif  // IMPELLER_DEBUG
}

}  // namespace impeller
