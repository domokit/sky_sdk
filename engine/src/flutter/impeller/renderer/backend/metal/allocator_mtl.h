// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_ALLOCATOR_MTL_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_ALLOCATOR_MTL_H_

#include <Metal/Metal.h>
#include <atomic>

#include "impeller/base/thread.h"
#include "impeller/core/allocator.h"

namespace impeller {

class DebugAllocatorStats {
 public:
  DebugAllocatorStats() {}

  ~DebugAllocatorStats() {}

  /// Increment the tracked allocation size in bytes.
  void Increment(size_t size);

  /// Decrement the tracked allocation size in bytes.
  void Decrement(size_t size);

  /// Get the current tracked allocation size.
  Bytes GetAllocationSize();

 private:
  std::atomic<size_t> size_ = 0;
};

ISize DeviceMaxTextureSizeSupported(id<MTLDevice> device);

class AllocatorMTL final : public Allocator {
 public:
  AllocatorMTL();

  // |Allocator|
  ~AllocatorMTL() override;

  // |Allocator|
  Bytes DebugGetHeapUsage() const override;

 private:
  friend class ContextMTL;

  id<MTLDevice> device_;
  std::string allocator_label_;
  bool supports_memoryless_targets_ = false;
  bool supports_uma_ = false;
  bool is_valid_ = false;

#ifdef IMPELLER_DEBUG
  std::shared_ptr<DebugAllocatorStats> debug_allocater_ =
      std::make_shared<DebugAllocatorStats>();
#endif  // IMPELLER_DEBUG

  ISize max_texture_supported_;

  AllocatorMTL(id<MTLDevice> device, std::string label);

  // |Allocator|
  bool IsValid() const;

  // |Allocator|
  std::shared_ptr<DeviceBuffer> OnCreateBuffer(
      const DeviceBufferDescriptor& desc) override;

  // |Allocator|
  std::shared_ptr<Texture> OnCreateTexture(
      const TextureDescriptor& desc) override;

  // |Allocator|
  uint16_t MinimumBytesPerRow(PixelFormat format) const override;

  // |Allocator|
  ISize GetMaxTextureSizeSupported() const override;

  // |Allocator|
  void DebugTraceMemoryStatistics() const override;

  AllocatorMTL(const AllocatorMTL&) = delete;

  AllocatorMTL& operator=(const AllocatorMTL&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_ALLOCATOR_MTL_H_
