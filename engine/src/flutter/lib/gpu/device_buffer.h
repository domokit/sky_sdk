// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_GPU_DEVICE_BUFFER_H_
#define FLUTTER_LIB_GPU_DEVICE_BUFFER_H_

#include "flutter/lib/gpu/context.h"
#include "flutter/lib/gpu/export.h"
#include "flutter/lib/ui/dart_wrapper.h"

#include "third_party/tonic/typed_data/dart_byte_data.h"

namespace flutter {
namespace gpu {

class DeviceBuffer : public RefCountedDartWrappable<DeviceBuffer> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(DeviceBuffer);

 public:
  explicit DeviceBuffer(std::shared_ptr<impeller::DeviceBuffer> device_buffer);

  ~DeviceBuffer() override;

  std::shared_ptr<impeller::DeviceBuffer> GetBuffer();

  bool Overwrite(const tonic::DartByteData& source_bytes,
                 size_t destination_offset_in_bytes);

 private:
  std::shared_ptr<impeller::DeviceBuffer> device_buffer_;

  FML_DISALLOW_COPY_AND_ASSIGN(DeviceBuffer);
};

}  // namespace gpu
}  // namespace flutter

//----------------------------------------------------------------------------
/// Exports
///

extern "C" {

FLUTTER_GPU_EXPORT
extern bool InternalFlutterGpu_DeviceBuffer_Initialize(
    Dart_Handle wrapper,
    flutter::gpu::Context* gpu_context,
    int storage_mode,
    int size_in_bytes);

FLUTTER_GPU_EXPORT
extern bool InternalFlutterGpu_DeviceBuffer_InitializeWithHostData(
    Dart_Handle wrapper,
    flutter::gpu::Context* gpu_context,
    Dart_Handle byte_data);

FLUTTER_GPU_EXPORT
extern bool InternalFlutterGpu_DeviceBuffer_Overwrite(
    flutter::gpu::DeviceBuffer* wrapper,
    Dart_Handle source_byte_data,
    int destination_offset_in_bytes);

FLUTTER_GPU_EXPORT
extern bool InternalFlutterGpu_DeviceBuffer_Flush(
    flutter::gpu::DeviceBuffer* wrapper,
    int offset_in_bytes,
    int size_in_bytes);

}  // extern "C"

#endif  // FLUTTER_LIB_GPU_DEVICE_BUFFER_H_
