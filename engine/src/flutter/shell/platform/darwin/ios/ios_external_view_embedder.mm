// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/ios_external_view_embedder.h"
#include "fml/task_runner.h"

#include "flutter/common/constants.h"

FLUTTER_ASSERT_ARC

namespace flutter {

IOSExternalViewEmbedder::IOSExternalViewEmbedder(
    __weak FlutterPlatformViewsController* platform_views_controller,
    const std::shared_ptr<IOSContext>& context)
    : platform_views_controller_(platform_views_controller), ios_context_(context) {
  FML_CHECK(ios_context_);
}

IOSExternalViewEmbedder::~IOSExternalViewEmbedder() = default;

// |ExternalViewEmbedder|
DlCanvas* IOSExternalViewEmbedder::GetRootCanvas() {
  // On iOS, the root surface is created from the on-screen render target. Only the surfaces for the
  // various overlays are controlled by this class.
  return nullptr;
}

// |ExternalViewEmbedder|
void IOSExternalViewEmbedder::CancelFrame() {
  TRACE_EVENT0("flutter", "IOSExternalViewEmbedder::CancelFrame");
  FML_CHECK(platform_views_controller_);
  [platform_views_controller_ cancelFrame];
}

// |ExternalViewEmbedder|
void IOSExternalViewEmbedder::BeginFrame(
    GrDirectContext* context,
    const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {}

// |ExternalViewEmbedder|
void IOSExternalViewEmbedder::PrepareFlutterView(SkISize frame_size, double device_pixel_ratio) {
  FML_CHECK(platform_views_controller_);
  [platform_views_controller_ beginFrameWithSize:frame_size];
}

// |ExternalViewEmbedder|
void IOSExternalViewEmbedder::PrerollCompositeEmbeddedView(
    int64_t view_id,
    std::unique_ptr<EmbeddedViewParams> params) {
  TRACE_EVENT0("flutter", "IOSExternalViewEmbedder::PrerollCompositeEmbeddedView");
  FML_CHECK(platform_views_controller_);
  [platform_views_controller_ prerollCompositeEmbeddedView:view_id withParams:std::move(params)];
}

// |ExternalViewEmbedder|
PostPrerollResult IOSExternalViewEmbedder::PostPrerollAction(
    const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {
  TRACE_EVENT0("flutter", "IOSExternalViewEmbedder::PostPrerollAction");
  FML_CHECK(platform_views_controller_);
  BOOL impeller_enabled = ios_context_->GetBackend() != IOSRenderingBackend::kSkia;
  PostPrerollResult result =
      [platform_views_controller_ postPrerollActionWithThreadMerger:raster_thread_merger
                                                    impellerEnabled:impeller_enabled];
  return result;
}

// |ExternalViewEmbedder|
DlCanvas* IOSExternalViewEmbedder::CompositeEmbeddedView(int64_t view_id) {
  TRACE_EVENT0("flutter", "IOSExternalViewEmbedder::CompositeEmbeddedView");
  FML_CHECK(platform_views_controller_);
  return [platform_views_controller_ compositeEmbeddedViewWithId:view_id];
}

// |ExternalViewEmbedder|
void IOSExternalViewEmbedder::SubmitFlutterView(
    int64_t flutter_view_id,
    GrDirectContext* context,
    const std::shared_ptr<impeller::AiksContext>& aiks_context,
    std::unique_ptr<SurfaceFrame> frame) {
  TRACE_EVENT0("flutter", "IOSExternalViewEmbedder::SubmitFlutterView");

  // TODO(dkwingsmt): This class only supports rendering into the implicit view.
  // Properly support multi-view in the future.
  FML_DCHECK(flutter_view_id == kFlutterImplicitViewId);
  FML_CHECK(platform_views_controller_);
  [platform_views_controller_ submitFrame:std::move(frame)
                           withIosContext:ios_context_
                                grContext:context];
  TRACE_EVENT0("flutter", "IOSExternalViewEmbedder::DidSubmitFrame");
}

// |ExternalViewEmbedder|
void IOSExternalViewEmbedder::EndFrame(
    bool should_resubmit_frame,
    const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {
  TRACE_EVENT0("flutter", "IOSExternalViewEmbedder::EndFrame");
  BOOL impeller_enabled = ios_context_->GetBackend() != IOSRenderingBackend::kSkia;
  [platform_views_controller_ endFrameWithResubmit:should_resubmit_frame
                                      threadMerger:raster_thread_merger
                                   impellerEnabled:impeller_enabled];
}

// |ExternalViewEmbedder|
bool IOSExternalViewEmbedder::SupportsDynamicThreadMerging() {
// TODO(jonahwilliams): remove this once Software backend is removed for iOS Sim.
#if FML_OS_IOS_SIMULATOR
  return true;
#else
  return ios_context_->GetBackend() == IOSRenderingBackend::kSkia;
#endif  // FML_OS_IOS_SIMULATOR
}

// |ExternalViewEmbedder|
void IOSExternalViewEmbedder::PushFilterToVisitedPlatformViews(
    const std::shared_ptr<DlImageFilter>& filter,
    const SkRect& filter_rect) {
  [platform_views_controller_ pushFilterToVisitedPlatformViews:filter withRect:filter_rect];
}

// |ExternalViewEmbedder|
void IOSExternalViewEmbedder::PushVisitedPlatformView(int64_t view_id) {
  [platform_views_controller_ pushVisitedPlatformViewId:view_id];
}

}  // namespace flutter
