// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_SURFACE_GL_SKIA_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_SURFACE_GL_SKIA_H_

#include "flutter/fml/macros.h"
#include "flutter/shell/gpu/gpu_surface_gl_skia.h"
#include "flutter/shell/platform/embedder/embedder_external_view_embedder.h"
#include "flutter/shell/platform/embedder/embedder_surface.h"

namespace flutter {

class EmbedderSurfaceGLSkia final : public EmbedderSurface,
                                    public GPUSurfaceGLDelegate {
 public:
  struct GLDispatchTable {
    std::function<bool(void)> gl_make_current_callback;           // required
    std::function<bool(void)> gl_clear_current_callback;          // required
    std::function<bool(GLPresentInfo)> gl_present_callback;       // required
    std::function<intptr_t(GLFrameInfo)> gl_fbo_callback;         // required
    std::function<bool(void)> gl_make_resource_current_callback;  // optional
    std::function<SkMatrix(void)>
        gl_surface_transformation_callback;                          // optional
    std::function<void*(const char*)> gl_proc_resolver;              // optional
    std::function<GLFBOInfo(intptr_t)> gl_populate_existing_damage;  // required
  };

  EmbedderSurfaceGLSkia(
      GLDispatchTable gl_dispatch_table,
      bool fbo_reset_after_present,
      std::shared_ptr<EmbedderExternalViewEmbedder> external_view_embedder);

  ~EmbedderSurfaceGLSkia() override;

 private:
  bool valid_ = false;
  GLDispatchTable gl_dispatch_table_;
  bool fbo_reset_after_present_;

  std::shared_ptr<EmbedderExternalViewEmbedder> external_view_embedder_;

  // |EmbedderSurface|
  bool IsValid() const override;

  // |EmbedderSurface|
  std::unique_ptr<Surface> CreateGPUSurface() override;

  // |EmbedderSurface|
  sk_sp<GrDirectContext> CreateResourceContext() const override;

  // |GPUSurfaceGLDelegate|
  std::unique_ptr<GLContextResult> GLContextMakeCurrent() override;

  // |GPUSurfaceGLDelegate|
  bool GLContextClearCurrent() override;

  // |GPUSurfaceGLDelegate|
  bool GLContextPresent(const GLPresentInfo& present_info) override;

  // |GPUSurfaceGLDelegate|
  GLFBOInfo GLContextFBO(GLFrameInfo frame_info) const override;

  // |GPUSurfaceGLDelegate|
  bool GLContextFBOResetAfterPresent() const override;

  // |GPUSurfaceGLDelegate|
  SkMatrix GLContextSurfaceTransformation() const override;

  // |GPUSurfaceGLDelegate|
  GLProcResolver GetGLProcResolver() const override;

  // |GPUSurfaceGLDelegate|
  SurfaceFrame::FramebufferInfo GLContextFramebufferInfo() const override;

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderSurfaceGLSkia);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_SURFACE_GL_SKIA_H_
