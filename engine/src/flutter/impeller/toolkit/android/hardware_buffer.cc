// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/android/hardware_buffer.h"

#include "impeller/base/validation.h"

namespace impeller::android {

static AHardwareBuffer_Format ToAHardwareBufferFormat(
    HardwareBufferFormat format) {
  switch (format) {
    case HardwareBufferFormat::kR8G8B8A8UNormInt:
      return AHARDWAREBUFFER_FORMAT_R8G8B8A8_UNORM;
  }
  FML_UNREACHABLE();
}

static AHardwareBuffer_Desc ToAHardwareBufferDesc(
    const HardwareBufferDescriptor& desc) {
  AHardwareBuffer_Desc ahb_desc = {};
  ahb_desc.width = desc.size.width;
  ahb_desc.height = desc.size.height;
  ahb_desc.format = ToAHardwareBufferFormat(desc.format);
  ahb_desc.layers = 1u;
  if (desc.usage & HardwareBufferUsageFlags::kFrameBufferAttachment) {
    ahb_desc.usage |= (AHARDWAREBUFFER_USAGE_GPU_FRAMEBUFFER |
                       AHARDWAREBUFFER_USAGE_GPU_COLOR_OUTPUT);
  }
  if (desc.usage & HardwareBufferUsageFlags::kCompositorOverlay) {
    ahb_desc.usage |= AHARDWAREBUFFER_USAGE_COMPOSER_OVERLAY;
  }
  if (desc.usage & HardwareBufferUsageFlags::kSampledImage) {
    ahb_desc.usage |= AHARDWAREBUFFER_USAGE_GPU_SAMPLED_IMAGE;
  }
  if (desc.usage & HardwareBufferUsageFlags::kCPUReadRarely) {
    ahb_desc.usage |= AHARDWAREBUFFER_USAGE_CPU_READ_RARELY;
  }
  if (desc.usage & HardwareBufferUsageFlags::kCPUReadOften) {
    ahb_desc.usage |= AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN;
  }
  if (desc.usage & HardwareBufferUsageFlags::kCPUWriteRarely) {
    ahb_desc.usage |= AHARDWAREBUFFER_USAGE_CPU_WRITE_RARELY;
  }
  if (desc.usage & HardwareBufferUsageFlags::kCPUWriteOften) {
    ahb_desc.usage |= AHARDWAREBUFFER_USAGE_CPU_WRITE_OFTEN;
  }
  return ahb_desc;
}

bool HardwareBufferDescriptor::IsAllocatable() const {
  const auto desc = ToAHardwareBufferDesc(*this);
  return GetProcTable().AHardwareBuffer_isSupported(&desc) != 0u;
}

HardwareBuffer::HardwareBuffer(HardwareBufferDescriptor descriptor)
    : descriptor_(descriptor),
      android_descriptor_(ToAHardwareBufferDesc(descriptor_)) {
  if (!descriptor_.IsAllocatable()) {
    VALIDATION_LOG << "The hardware buffer descriptor is not allocatable.";
    return;
  }
  const auto& proc_table = GetProcTable();

  AHardwareBuffer* buffer = nullptr;
  if (auto result =
          proc_table.AHardwareBuffer_allocate(&android_descriptor_, &buffer);
      result != 0 || buffer == nullptr) {
    VALIDATION_LOG << "Could not allocate hardware buffer. Error: " << result;
    return;
  }
  buffer_.reset(buffer);
  is_valid_ = true;
}

HardwareBuffer::~HardwareBuffer() = default;

bool HardwareBuffer::IsValid() const {
  return is_valid_;
}

AHardwareBuffer* HardwareBuffer::GetHandle() const {
  return buffer_.get();
}

HardwareBufferDescriptor HardwareBufferDescriptor::MakeForSwapchainImage(
    const ISize& size) {
  HardwareBufferDescriptor desc;
  desc.format = HardwareBufferFormat::kR8G8B8A8UNormInt;
  // Zero sized hardware buffers cannot be allocated.
  desc.size = size.Max(ISize{1u, 1u});
  desc.usage = HardwareBufferUsageFlags::kFrameBufferAttachment |
               HardwareBufferUsageFlags::kCompositorOverlay |
               HardwareBufferUsageFlags::kSampledImage;
  return desc;
}

const HardwareBufferDescriptor& HardwareBuffer::GetDescriptor() const {
  return descriptor_;
}

const AHardwareBuffer_Desc& HardwareBuffer::GetAndroidDescriptor() const {
  return android_descriptor_;
}

bool HardwareBuffer::IsAvailableOnPlatform() {
  return GetProcTable().IsValid() && GetProcTable().AHardwareBuffer_isSupported;
}

std::optional<uint64_t> HardwareBuffer::GetSystemUniqueID() const {
  return GetSystemUniqueID(GetHandle());
}

std::optional<uint64_t> HardwareBuffer::GetSystemUniqueID(
    AHardwareBuffer* buffer) {
  if (!GetProcTable().AHardwareBuffer_getId) {
    return std::nullopt;
  }
  uint64_t out_id = 0u;
  if (GetProcTable().AHardwareBuffer_getId(buffer, &out_id) != 0) {
    return std::nullopt;
  }
  return out_id;
}

std::optional<AHardwareBuffer_Desc> HardwareBuffer::Describe(
    AHardwareBuffer* buffer) {
  if (!buffer || !GetProcTable().AHardwareBuffer_describe) {
    return std::nullopt;
  }
  AHardwareBuffer_Desc desc = {};
  GetProcTable().AHardwareBuffer_describe(buffer, &desc);
  return desc;
}

void* HardwareBuffer::Lock(CPUAccessType type) const {
  if (!is_valid_ || !GetProcTable().AHardwareBuffer_lock) {
    return nullptr;
  }
  uint64_t usage = 0;
  switch (type) {
    case CPUAccessType::kRead:
      usage |= AHARDWAREBUFFER_USAGE_CPU_READ_MASK;
      break;
    case CPUAccessType::kWrite:
      usage |= AHARDWAREBUFFER_USAGE_CPU_WRITE_MASK;
      break;
  }
  void* buffer = nullptr;
  const auto result = GetProcTable().AHardwareBuffer_lock(  //
      buffer_.get(),                                        // buffer
      usage,                                                // usage
      -1,                                                   // fence
      nullptr,                                              // rect
      &buffer                                               // out-addr
  );
  return result == 0 ? buffer : nullptr;
}

bool HardwareBuffer::Unlock() const {
  if (!is_valid_ || !GetProcTable().AHardwareBuffer_unlock) {
    return false;
  }
  const auto result =
      GetProcTable().AHardwareBuffer_unlock(buffer_.get(), nullptr);
  return result == 0;
}

}  // namespace impeller::android
