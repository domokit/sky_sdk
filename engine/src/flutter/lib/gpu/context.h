// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_GPU_CONTEXT_H_
#define FLUTTER_LIB_GPU_CONTEXT_H_

#include "dart_api.h"
#include "flutter/lib/gpu/export.h"
#include "flutter/lib/ui/dart_wrapper.h"
#include "impeller/renderer/context.h"

namespace flutter {
namespace gpu {

bool SupportsNormalOffscreenMSAA(const impeller::Context& context);

class Context : public RefCountedDartWrappable<Context> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(Context);

 public:
  static void SetOverrideContext(std::shared_ptr<impeller::Context> context);

  static std::shared_ptr<impeller::Context> GetOverrideContext();

  static std::shared_ptr<impeller::Context> GetDefaultContext(
      std::optional<std::string>& out_error);

  explicit Context(std::shared_ptr<impeller::Context> context);
  ~Context() override;

  std::shared_ptr<impeller::Context> GetContext();

 private:
  /// An Impeller context that takes precedent over the IO state context when
  /// set. This is used to inject the context when running with the Impeller
  /// playground, which doesn't instantiate an Engine instance.
  static std::shared_ptr<impeller::Context> default_context_;

  std::shared_ptr<impeller::Context> context_;

  FML_DISALLOW_COPY_AND_ASSIGN(Context);
};

}  // namespace gpu
}  // namespace flutter

//----------------------------------------------------------------------------
/// Exports
///

extern "C" {

FLUTTER_GPU_EXPORT
extern Dart_Handle InternalFlutterGpu_Context_InitializeDefault(
    Dart_Handle wrapper);

FLUTTER_GPU_EXPORT
extern int InternalFlutterGpu_Context_GetBackendType(
    flutter::gpu::Context* wrapper);

FLUTTER_GPU_EXPORT
extern int InternalFlutterGpu_Context_GetDefaultColorFormat(
    flutter::gpu::Context* wrapper);

FLUTTER_GPU_EXPORT
extern int InternalFlutterGpu_Context_GetDefaultStencilFormat(
    flutter::gpu::Context* wrapper);

FLUTTER_GPU_EXPORT
extern int InternalFlutterGpu_Context_GetDefaultDepthStencilFormat(
    flutter::gpu::Context* wrapper);

FLUTTER_GPU_EXPORT
extern int InternalFlutterGpu_Context_GetMinimumUniformByteAlignment(
    flutter::gpu::Context* wrapper);

FLUTTER_GPU_EXPORT
extern bool InternalFlutterGpu_Context_GetSupportsOffscreenMSAA(
    flutter::gpu::Context* wrapper);

}  // extern "C"

#endif  // FLUTTER_LIB_GPU_CONTEXT_H_
