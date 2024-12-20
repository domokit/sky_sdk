// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_EXTERNAL_TEXTURE_D3D_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_EXTERNAL_TEXTURE_D3D_H_

#include <memory>

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/common/public/flutter_texture_registrar.h"
#include "flutter/shell/platform/windows/egl/manager.h"
#include "flutter/shell/platform/windows/egl/proc_table.h"
#include "flutter/shell/platform/windows/external_texture.h"

namespace flutter {

// An external texture that is backed by a DXGI surface.
class ExternalTextureD3d : public ExternalTexture {
 public:
  ExternalTextureD3d(
      FlutterDesktopGpuSurfaceType type,
      const FlutterDesktopGpuSurfaceTextureCallback texture_callback,
      void* user_data,
      const egl::Manager* egl_manager,
      std::shared_ptr<egl::ProcTable> gl);
  virtual ~ExternalTextureD3d();

  // |ExternalTexture|
  bool PopulateTexture(size_t width,
                       size_t height,
                       FlutterOpenGLTexture* opengl_texture) override;

 private:
  // Creates or updates the backing texture and associates it with the provided
  // surface.
  bool CreateOrUpdateTexture(
      const FlutterDesktopGpuSurfaceDescriptor* descriptor);
  // Detaches the previously attached surface, if any.
  void ReleaseImage();

  FlutterDesktopGpuSurfaceType type_;
  const FlutterDesktopGpuSurfaceTextureCallback texture_callback_;
  void* const user_data_;
  const egl::Manager* egl_manager_;
  std::shared_ptr<egl::ProcTable> gl_;
  GLuint gl_texture_ = 0;
  EGLSurface egl_surface_ = EGL_NO_SURFACE;
  void* last_surface_handle_ = nullptr;

  FML_DISALLOW_COPY_AND_ASSIGN(ExternalTextureD3d);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_EXTERNAL_TEXTURE_D3D_H_
