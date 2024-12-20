// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/gpu/gpu_surface_gl_delegate.h"

#include "flutter/fml/build_config.h"

#include <cstring>

#include "third_party/skia/include/gpu/ganesh/gl/GrGLAssembleInterface.h"
#include "third_party/skia/include/gpu/ganesh/gl/GrGLInterface.h"

#if defined(FML_OS_ANDROID)
#include "third_party/skia/include/gpu/ganesh/gl/egl/GrGLMakeEGLInterface.h"
#endif

#if defined(FML_OS_LINUX) && defined(SK_GLX)
#include "third_party/skia/include/gpu/ganesh/gl/glx/GrGLMakeGLXInterface.h"
#endif

#if defined(FML_OS_MACOSX)
#include "third_party/skia/include/gpu/ganesh/gl/mac/GrGLMakeMacInterface.h"
#endif

#if defined(FML_OS_IOS)
#include "third_party/skia/include/gpu/ganesh/gl/ios/GrGLMakeIOSInterface.h"
#endif

namespace flutter {

GPUSurfaceGLDelegate::~GPUSurfaceGLDelegate() = default;

bool GPUSurfaceGLDelegate::GLContextFBOResetAfterPresent() const {
  return false;
}

SurfaceFrame::FramebufferInfo GPUSurfaceGLDelegate::GLContextFramebufferInfo()
    const {
  SurfaceFrame::FramebufferInfo res;
  res.supports_readback = true;
  return res;
}

SkMatrix GPUSurfaceGLDelegate::GLContextSurfaceTransformation() const {
  SkMatrix matrix;
  matrix.setIdentity();
  return matrix;
}

GPUSurfaceGLDelegate::GLProcResolver GPUSurfaceGLDelegate::GetGLProcResolver()
    const {
  return nullptr;
}

static bool IsProcResolverOpenGLES(
    const GPUSurfaceGLDelegate::GLProcResolver& proc_resolver) {
  // Version string prefix that identifies an OpenGL ES implementation.
#define GPU_GL_VERSION 0x1F02
  constexpr char kGLESVersionPrefix[] = "OpenGL ES";

#ifdef WIN32
  using GLGetStringProc = const char*(__stdcall*)(uint32_t);
#else
  using GLGetStringProc = const char* (*)(uint32_t);
#endif

  GLGetStringProc gl_get_string =
      reinterpret_cast<GLGetStringProc>(proc_resolver("glGetString"));

  FML_CHECK(gl_get_string)
      << "The GL proc resolver could not resolve glGetString";

  const char* gl_version_string = gl_get_string(GPU_GL_VERSION);

  FML_CHECK(gl_version_string)
      << "The GL proc resolver's glGetString(GL_VERSION) failed";

  return strncmp(gl_version_string, kGLESVersionPrefix,
                 strlen(kGLESVersionPrefix)) == 0;
}

static sk_sp<const GrGLInterface> CreateGLInterface(
    const GPUSurfaceGLDelegate::GLProcResolver& proc_resolver) {
  if (proc_resolver == nullptr) {
#if defined(FML_OS_ANDROID)
    return GrGLInterfaces::MakeEGL();
#elif defined(FML_OS_LINUX)
#if defined(SK_GLX)
    return GrGLInterfaces::MakeGLX();
#else
    return nullptr;
#endif  // defined(SK_GLX)
#elif defined(FML_OS_IOS)
    return GrGLInterfaces::MakeIOS();
#elif defined(FML_OS_MACOSX)
    return GrGLInterfaces::MakeMac();
#else
    // TODO(kjlubick) update this when Skia has a Windows target for making GL
    // interfaces. For now, ask Skia to guess the native
    // interface. This often leads to interesting results on most platforms.
    return GrGLMakeNativeInterface();
#endif
  }

  struct ProcResolverContext {
    GPUSurfaceGLDelegate::GLProcResolver resolver;
  };

  ProcResolverContext context = {proc_resolver};

  GrGLGetProc gl_get_proc = [](void* context,
                               const char gl_proc_name[]) -> GrGLFuncPtr {
    auto proc_resolver_context =
        reinterpret_cast<ProcResolverContext*>(context);
    return reinterpret_cast<GrGLFuncPtr>(
        proc_resolver_context->resolver(gl_proc_name));
  };

  // glGetString indicates an OpenGL ES interface.
  if (IsProcResolverOpenGLES(proc_resolver)) {
    return GrGLMakeAssembledGLESInterface(&context, gl_get_proc);
  }

  // Fallback to OpenGL.
  if (auto interface = GrGLMakeAssembledGLInterface(&context, gl_get_proc)) {
    return interface;
  }

  FML_LOG(ERROR) << "Could not create a valid GL interface.";
  return nullptr;
}

sk_sp<const GrGLInterface> GPUSurfaceGLDelegate::GetGLInterface() const {
  return CreateGLInterface(GetGLProcResolver());
}

sk_sp<const GrGLInterface>
GPUSurfaceGLDelegate::GetDefaultPlatformGLInterface() {
  return CreateGLInterface(nullptr);
}

bool GPUSurfaceGLDelegate::AllowsDrawingWhenGpuDisabled() const {
  return true;
}

}  // namespace flutter
