// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_ANDROID_HARDWARE_BUFFER_H_
#define FLUTTER_IMPELLER_TOOLKIT_ANDROID_HARDWARE_BUFFER_H_

#include <optional>

#include "flutter/fml/unique_fd.h"
#include "flutter/fml/unique_object.h"
#include "impeller/base/mask.h"
#include "impeller/geometry/size.h"
#include "impeller/toolkit/android/proc_table.h"

namespace impeller::android {

enum class HardwareBufferFormat {
  //----------------------------------------------------------------------------
  /// This format is guaranteed to be supported on all versions of Android. This
  /// format can also be converted to an Impeller and Vulkan format.
  ///
  /// @see        Vulkan Format: VK_FORMAT_R8G8B8A8_UNORM
  /// @see        OpenGL ES Format: GL_RGBA8
  ///
  /// Why have many format when one format do trick?
  ///
  kR8G8B8A8UNormInt,
};

enum class HardwareBufferUsageFlags {
  kNone = 0u,
  kFrameBufferAttachment = 1u << 0u,
  kCompositorOverlay = 1u << 1u,
  kSampledImage = 1u << 2u,
  kCPUReadRarely = 1u << 3u,
  kCPUReadOften = 1u << 4u,
  kCPUWriteRarely = 1u << 5u,
  kCPUWriteOften = 1u << 6u,
};

using HardwareBufferUsage = Mask<HardwareBufferUsageFlags>;

//------------------------------------------------------------------------------
/// @brief      A descriptor use to specify hardware buffer allocations.
///
struct HardwareBufferDescriptor {
  HardwareBufferFormat format = HardwareBufferFormat::kR8G8B8A8UNormInt;
  ISize size;
  HardwareBufferUsage usage = HardwareBufferUsageFlags::kNone;

  //----------------------------------------------------------------------------
  /// @brief      Create a descriptor of the given size that is suitable for use
  ///             as a swapchain image.
  ///
  /// @warning    Descriptors of zero size are not allocatable. The next best
  ///             valid size is picked. So make sure to check the actual size of
  ///             the descriptor after this call is made to determine the size
  ///             of the allocated hardware buffer.
  ///
  /// @param[in]  size  The size. See the restrictions about valid sizes above.
  ///
  /// @return     The hardware buffer descriptor.
  ///
  static HardwareBufferDescriptor MakeForSwapchainImage(const ISize& size);

  //----------------------------------------------------------------------------
  /// @brief      If hardware buffers can be created using this descriptor.
  ///             Allocatable descriptors may still cause failing allocations in
  ///             case of resource exhaustion.
  ///
  /// @return     `true` if allocatable (unless resource exhaustion).
  ///
  bool IsAllocatable() const;

  constexpr bool operator==(const HardwareBufferDescriptor& o) const {
    return format == o.format && size == o.size && usage == o.usage;
  }

  constexpr bool operator!=(const HardwareBufferDescriptor& o) const {
    return !(*this == o);
  }
};

//------------------------------------------------------------------------------
/// @brief      A wrapper for AHardwareBuffer
///             https://developer.android.com/ndk/reference/group/a-hardware-buffer
///
///             This wrapper creates and owns a handle to a managed hardware
///             buffer. That is, there is no ability to take a reference to an
///             externally created hardware buffer.
///
///             This wrapper is only available on Android API 29 and above.
///
class HardwareBuffer {
 public:
  static bool IsAvailableOnPlatform();

  explicit HardwareBuffer(HardwareBufferDescriptor descriptor);

  ~HardwareBuffer();

  HardwareBuffer(const HardwareBuffer&) = delete;

  HardwareBuffer& operator=(const HardwareBuffer&) = delete;

  bool IsValid() const;

  AHardwareBuffer* GetHandle() const;

  const HardwareBufferDescriptor& GetDescriptor() const;

  const AHardwareBuffer_Desc& GetAndroidDescriptor() const;

  static std::optional<AHardwareBuffer_Desc> Describe(AHardwareBuffer* buffer);

  //----------------------------------------------------------------------------
  /// @brief      Get the system wide unique ID of the hardware buffer if
  ///             possible. This is only available on Android API 31 and above.
  ///             Within the process, the handle are unique.
  ///
  /// @return     The system unique id if one can be obtained.
  ///
  std::optional<uint64_t> GetSystemUniqueID() const;

  //----------------------------------------------------------------------------
  /// @brief      Get the system wide unique ID of the hardware buffer if
  ///             possible. This is only available on Android API 31 and above.
  ///             Within the process, the handle are unique.
  ///
  /// @return     The system unique id if one can be obtained.
  ///
  static std::optional<uint64_t> GetSystemUniqueID(AHardwareBuffer* buffer);

  enum class CPUAccessType {
    kRead,
    kWrite,
  };
  //----------------------------------------------------------------------------
  /// @brief      Lock the buffer for CPU access. This call may fail if the
  ///             buffer was not created with one the usages that allow for CPU
  ///             access.
  ///
  /// @param[in]  type  The type
  ///
  /// @return     A host-accessible buffer if there was no error related to
  ///             usage or buffer validity.
  ///
  void* Lock(CPUAccessType type) const;

  //----------------------------------------------------------------------------
  /// @brief      Unlock a mapping previously locked for CPU access.
  ///
  /// @return     If the unlock was successful.
  ///
  bool Unlock() const;

 private:
  struct UniqueAHardwareBufferTraits {
    static AHardwareBuffer* InvalidValue() { return nullptr; }

    static bool IsValid(AHardwareBuffer* value) {
      return value != InvalidValue();
    }

    static void Free(AHardwareBuffer* value) {
      GetProcTable().AHardwareBuffer_release(value);
    }
  };

  const HardwareBufferDescriptor descriptor_;
  const AHardwareBuffer_Desc android_descriptor_;
  fml::UniqueObject<AHardwareBuffer*, UniqueAHardwareBufferTraits> buffer_;
  bool is_valid_ = false;
};

}  // namespace impeller::android

namespace impeller {

IMPELLER_ENUM_IS_MASK(android::HardwareBufferUsageFlags);

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_TOOLKIT_ANDROID_HARDWARE_BUFFER_H_
