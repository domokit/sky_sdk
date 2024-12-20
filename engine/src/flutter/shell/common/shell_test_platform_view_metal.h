// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_SHELL_TEST_PLATFORM_VIEW_METAL_H_
#define FLUTTER_SHELL_COMMON_SHELL_TEST_PLATFORM_VIEW_METAL_H_

#include "flutter/shell/common/shell_test_platform_view.h"

#import <Metal/Metal.h>

#include "flutter/fml/macros.h"
#include "flutter/shell/gpu/gpu_surface_metal_delegate.h"
#include "flutter/shell/platform/darwin/graphics/FlutterDarwinContextMetalImpeller.h"
#include "flutter/shell/platform/darwin/graphics/FlutterDarwinContextMetalSkia.h"

namespace flutter::testing {

class ShellTestPlatformViewMetal final : public ShellTestPlatformView,
                                         public GPUSurfaceMetalDelegate {
 public:
  ShellTestPlatformViewMetal(PlatformView::Delegate& delegate,
                             const TaskRunners& task_runners,
                             std::shared_ptr<ShellTestVsyncClock> vsync_clock,
                             CreateVsyncWaiter create_vsync_waiter,
                             std::shared_ptr<ShellTestExternalViewEmbedder>
                                 shell_test_external_view_embedder,
                             const std::shared_ptr<const fml::SyncSwitch>&
                                 is_gpu_disabled_sync_switch);

  // |ShellTestPlatformView|
  virtual ~ShellTestPlatformViewMetal() override;

 private:
  FlutterDarwinContextMetalSkia* skia_context_;
  FlutterDarwinContextMetalImpeller* impeller_context_;
  id<MTLTexture> offscreen_texture_;
  const CreateVsyncWaiter create_vsync_waiter_;
  const std::shared_ptr<ShellTestVsyncClock> vsync_clock_;
  const std::shared_ptr<ShellTestExternalViewEmbedder>
      shell_test_external_view_embedder_;

  // |ShellTestPlatformView|
  virtual void SimulateVSync() override;

  // |PlatformView|
  std::unique_ptr<VsyncWaiter> CreateVSyncWaiter() override;

  // |PlatformView|
  std::shared_ptr<ExternalViewEmbedder> CreateExternalViewEmbedder() override;

  // |PlatformView|
  PointerDataDispatcherMaker GetDispatcherMaker() override;

  // |PlatformView|
  std::unique_ptr<Surface> CreateRenderingSurface() override;

  // |PlatformView|
  std::shared_ptr<impeller::Context> GetImpellerContext() const override;

  // |GPUSurfaceMetalDelegate|
  GPUCAMetalLayerHandle GetCAMetalLayer(
      const SkISize& frame_info) const override;

  // |GPUSurfaceMetalDelegate|
  bool PresentDrawable(GrMTLHandle drawable) const override;

  // |GPUSurfaceMetalDelegate|
  GPUMTLTextureInfo GetMTLTexture(const SkISize& frame_info) const override;

  // |GPUSurfaceMetalDelegate|
  bool PresentTexture(GPUMTLTextureInfo texture) const override;

  FML_DISALLOW_COPY_AND_ASSIGN(ShellTestPlatformViewMetal);
};

}  // namespace flutter::testing

#endif  // FLUTTER_SHELL_COMMON_SHELL_TEST_PLATFORM_VIEW_METAL_H_
