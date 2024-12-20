// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/common/shell_test_platform_view_metal.h"

#include <utility>

#include "flutter/shell/gpu/gpu_surface_metal_impeller.h"
#include "flutter/shell/gpu/gpu_surface_metal_skia.h"

FLUTTER_ASSERT_ARC

namespace flutter::testing {

std::unique_ptr<ShellTestPlatformView> ShellTestPlatformView::CreateMetal(
    PlatformView::Delegate& delegate,
    const TaskRunners& task_runners,
    const std::shared_ptr<ShellTestVsyncClock>& vsync_clock,
    const CreateVsyncWaiter& create_vsync_waiter,
    const std::shared_ptr<ShellTestExternalViewEmbedder>& shell_test_external_view_embedder,
    const std::shared_ptr<const fml::SyncSwitch>& is_gpu_disabled_sync_switch) {
  return std::make_unique<ShellTestPlatformViewMetal>(
      delegate, task_runners, vsync_clock, create_vsync_waiter, shell_test_external_view_embedder,
      is_gpu_disabled_sync_switch);
}

ShellTestPlatformViewMetal::ShellTestPlatformViewMetal(
    PlatformView::Delegate& delegate,
    const TaskRunners& task_runners,
    std::shared_ptr<ShellTestVsyncClock> vsync_clock,
    CreateVsyncWaiter create_vsync_waiter,
    std::shared_ptr<ShellTestExternalViewEmbedder> shell_test_external_view_embedder,
    const std::shared_ptr<const fml::SyncSwitch>& is_gpu_disabled_sync_switch)
    : ShellTestPlatformView(delegate, task_runners),
      GPUSurfaceMetalDelegate(MTLRenderTargetType::kMTLTexture),
      create_vsync_waiter_(std::move(create_vsync_waiter)),
      vsync_clock_(std::move(vsync_clock)),
      shell_test_external_view_embedder_(std::move(shell_test_external_view_embedder)) {
  id<MTLDevice> device = nil;
  if (GetSettings().enable_impeller) {
    impeller_context_ =
        [[FlutterDarwinContextMetalImpeller alloc] init:is_gpu_disabled_sync_switch];
    FML_CHECK(impeller_context_.context);
    device = impeller_context_.context->GetMTLDevice();
  } else {
    skia_context_ = [[FlutterDarwinContextMetalSkia alloc] initWithDefaultMTLDevice];
    FML_CHECK(skia_context_.mainContext);
    device = skia_context_.device;
  }
  auto descriptor =
      [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
                                                         width:800
                                                        height:600
                                                     mipmapped:NO];
  descriptor.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
  offscreen_texture_ = [device newTextureWithDescriptor:descriptor];
}

ShellTestPlatformViewMetal::~ShellTestPlatformViewMetal() = default;

std::unique_ptr<VsyncWaiter> ShellTestPlatformViewMetal::CreateVSyncWaiter() {
  return create_vsync_waiter_();
}

// |ShellTestPlatformView|
void ShellTestPlatformViewMetal::SimulateVSync() {
  vsync_clock_->SimulateVSync();
}

// |PlatformView|
std::shared_ptr<ExternalViewEmbedder> ShellTestPlatformViewMetal::CreateExternalViewEmbedder() {
  return shell_test_external_view_embedder_;
}

// |PlatformView|
PointerDataDispatcherMaker ShellTestPlatformViewMetal::GetDispatcherMaker() {
  return [](DefaultPointerDataDispatcher::Delegate& delegate) {
    return std::make_unique<SmoothPointerDataDispatcher>(delegate);
  };
}

// |PlatformView|
std::unique_ptr<Surface> ShellTestPlatformViewMetal::CreateRenderingSurface() {
  if (GetSettings().enable_impeller) {
    auto context = impeller_context_.context;
    return std::make_unique<GPUSurfaceMetalImpeller>(
        this, std::make_shared<impeller::AiksContext>(context, nullptr));
  }
  return std::make_unique<GPUSurfaceMetalSkia>(this, skia_context_.mainContext);
}

// |PlatformView|
std::shared_ptr<impeller::Context> ShellTestPlatformViewMetal::GetImpellerContext() const {
  return impeller_context_.context;
}

// |GPUSurfaceMetalDelegate|
GPUCAMetalLayerHandle ShellTestPlatformViewMetal::GetCAMetalLayer(const SkISize& frame_info) const {
  FML_CHECK(false) << "A Metal Delegate configured with MTLRenderTargetType::kMTLTexture was asked "
                      "to acquire a layer.";
  return nullptr;
}

// |GPUSurfaceMetalDelegate|
bool ShellTestPlatformViewMetal::PresentDrawable(GrMTLHandle drawable) const {
  FML_CHECK(false) << "A Metal Delegate configured with MTLRenderTargetType::kMTLTexture was asked "
                      "to present a layer drawable.";
  return true;
}

// |GPUSurfaceMetalDelegate|
GPUMTLTextureInfo ShellTestPlatformViewMetal::GetMTLTexture(const SkISize& frame_info) const {
  return {
      .texture_id = 0,
      .texture = (__bridge GPUMTLTextureHandle)offscreen_texture_,
  };
}

// |GPUSurfaceMetalDelegate|
bool ShellTestPlatformViewMetal::PresentTexture(GPUMTLTextureInfo texture) const {
  // The texture resides offscreen. There is nothing to render to.
  return true;
}

}  // namespace flutter::testing
