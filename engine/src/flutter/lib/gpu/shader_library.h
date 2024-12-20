// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_GPU_SHADER_LIBRARY_H_
#define FLUTTER_LIB_GPU_SHADER_LIBRARY_H_

#include <memory>
#include <string>
#include <unordered_map>

#include "flutter/lib/gpu/export.h"
#include "flutter/lib/gpu/shader.h"
#include "flutter/lib/ui/dart_wrapper.h"
#include "fml/memory/ref_ptr.h"

namespace flutter {
namespace gpu {

/// An immutable collection of shaders loaded from a shader bundle asset.
class ShaderLibrary : public RefCountedDartWrappable<ShaderLibrary> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(ShaderLibrary);

 public:
  using ShaderMap = std::unordered_map<std::string, fml::RefPtr<Shader>>;

  static fml::RefPtr<ShaderLibrary> MakeFromAsset(
      impeller::Context::BackendType backend_type,
      const std::string& name,
      std::string& out_error);

  static fml::RefPtr<ShaderLibrary> MakeFromShaders(ShaderMap shaders);

  static fml::RefPtr<ShaderLibrary> MakeFromFlatbuffer(
      impeller::Context::BackendType backend_type,
      std::shared_ptr<fml::Mapping> payload);

  /// Sets a return override for `MakeFromAsset` for testing purposes.
  static void SetOverride(fml::RefPtr<ShaderLibrary> override_shader_library);

  fml::RefPtr<Shader> GetShader(const std::string& shader_name,
                                Dart_Handle shader_wrapper) const;

  ~ShaderLibrary() override;

 private:
  /// A global override used to inject a ShaderLibrary when running with the
  /// Impeller playground. When set, `MakeFromAsset` will always just return
  /// this library.
  static fml::RefPtr<ShaderLibrary> override_shader_library_;

  std::shared_ptr<fml::Mapping> payload_;
  ShaderMap shaders_;

  explicit ShaderLibrary(std::shared_ptr<fml::Mapping> payload,
                         ShaderMap shaders);

  FML_DISALLOW_COPY_AND_ASSIGN(ShaderLibrary);
};

}  // namespace gpu
}  // namespace flutter

//----------------------------------------------------------------------------
/// Exports
///

extern "C" {

FLUTTER_GPU_EXPORT
extern Dart_Handle InternalFlutterGpu_ShaderLibrary_InitializeWithAsset(
    Dart_Handle wrapper,
    Dart_Handle asset_name);

FLUTTER_GPU_EXPORT
extern Dart_Handle InternalFlutterGpu_ShaderLibrary_GetShader(
    flutter::gpu::ShaderLibrary* wrapper,
    Dart_Handle shader_name,
    Dart_Handle shader_wrapper);

}  // extern "C"

#endif  // FLUTTER_LIB_GPU_SHADER_LIBRARY_H_
