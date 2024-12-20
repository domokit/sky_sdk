// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_GPU_TEXTURE_H_
#define FLUTTER_LIB_GPU_TEXTURE_H_

#include "flutter/lib/gpu/context.h"
#include "flutter/lib/gpu/export.h"
#include "flutter/lib/ui/dart_wrapper.h"
#include "impeller/core/formats.h"
#include "third_party/tonic/typed_data/dart_byte_data.h"

namespace flutter {
namespace gpu {

class Texture : public RefCountedDartWrappable<Texture> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(Texture);

 public:
  explicit Texture(std::shared_ptr<impeller::Texture> texture);

  ~Texture() override;

  std::shared_ptr<impeller::Texture> GetTexture();

  void SetCoordinateSystem(impeller::TextureCoordinateSystem coordinate_system);

  bool Overwrite(const tonic::DartByteData& source_bytes);

  size_t GetBytesPerTexel();

  Dart_Handle AsImage() const;

 private:
  std::shared_ptr<impeller::Texture> texture_;

  FML_DISALLOW_COPY_AND_ASSIGN(Texture);
};

}  // namespace gpu
}  // namespace flutter

//----------------------------------------------------------------------------
/// Exports
///

extern "C" {

FLUTTER_GPU_EXPORT
extern bool InternalFlutterGpu_Texture_Initialize(
    Dart_Handle wrapper,
    flutter::gpu::Context* gpu_context,
    int storage_mode,
    int format,
    int width,
    int height,
    int sample_count,
    int coordinate_system,
    bool enable_render_target_usage,
    bool enable_shader_read_usage,
    bool enable_shader_write_usage);

FLUTTER_GPU_EXPORT
extern void InternalFlutterGpu_Texture_SetCoordinateSystem(
    flutter::gpu::Texture* wrapper,
    int coordinate_system);

FLUTTER_GPU_EXPORT
extern bool InternalFlutterGpu_Texture_Overwrite(flutter::gpu::Texture* wrapper,
                                                 Dart_Handle source_byte_data);

FLUTTER_GPU_EXPORT
extern int InternalFlutterGpu_Texture_BytesPerTexel(
    flutter::gpu::Texture* wrapper);

FLUTTER_GPU_EXPORT
extern Dart_Handle InternalFlutterGpu_Texture_AsImage(
    flutter::gpu::Texture* wrapper);

}  // extern "C"

#endif  // FLUTTER_LIB_GPU_TEXTURE_H_
