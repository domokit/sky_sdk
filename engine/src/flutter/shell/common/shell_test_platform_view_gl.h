// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_SHELL_TEST_PLATFORM_VIEW_GL_H_
#define FLUTTER_SHELL_COMMON_SHELL_TEST_PLATFORM_VIEW_GL_H_

#include "flutter/shell/common/shell_test_external_view_embedder.h"
#include "flutter/shell/common/shell_test_platform_view.h"
#include "flutter/shell/gpu/gpu_surface_gl_delegate.h"
#include "flutter/testing/test_gl_surface.h"
#include "impeller/renderer/backend/gles/context_gles.h"

namespace flutter {
namespace testing {

class ShellTestPlatformViewGL : public ShellTestPlatformView,
                                public GPUSurfaceGLDelegate {
 public:
  ShellTestPlatformViewGL(PlatformView::Delegate& delegate,
                          const TaskRunners& task_runners,
                          std::shared_ptr<ShellTestVsyncClock> vsync_clock,
                          CreateVsyncWaiter create_vsync_waiter,
                          std::shared_ptr<ShellTestExternalViewEmbedder>
                              shell_test_external_view_embedder);

  // |ShellTestPlatformView|
  virtual ~ShellTestPlatformViewGL() override;

  // |ShellTestPlatformView|
  virtual void SimulateVSync() override;

  // |PlatformView|
  std::shared_ptr<impeller::Context> GetImpellerContext() const {
    return impeller_context_;
  }

 private:
  std::shared_ptr<impeller::ContextGLES> impeller_context_;

  TestGLSurface gl_surface_;

  CreateVsyncWaiter create_vsync_waiter_;

  std::shared_ptr<ShellTestVsyncClock> vsync_clock_;

  std::shared_ptr<ShellTestExternalViewEmbedder>
      shell_test_external_view_embedder_;

  // |PlatformView|
  std::unique_ptr<Surface> CreateRenderingSurface() override;

  // |PlatformView|
  std::shared_ptr<ExternalViewEmbedder> CreateExternalViewEmbedder() override;

  // |PlatformView|
  std::unique_ptr<VsyncWaiter> CreateVSyncWaiter() override;

  // |PlatformView|
  PointerDataDispatcherMaker GetDispatcherMaker() override;

  // |GPUSurfaceGLDelegate|
  std::unique_ptr<GLContextResult> GLContextMakeCurrent() override;

  // |GPUSurfaceGLDelegate|
  bool GLContextClearCurrent() override;

  // |GPUSurfaceGLDelegate|
  bool GLContextPresent(const GLPresentInfo& present_info) override;

  // |GPUSurfaceGLDelegate|
  GLFBOInfo GLContextFBO(GLFrameInfo frame_info) const override;

  // |GPUSurfaceGLDelegate|
  GLProcResolver GetGLProcResolver() const override;

  FML_DISALLOW_COPY_AND_ASSIGN(ShellTestPlatformViewGL);
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_COMMON_SHELL_TEST_PLATFORM_VIEW_GL_H_
