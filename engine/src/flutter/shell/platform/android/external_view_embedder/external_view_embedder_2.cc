// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/external_view_embedder/external_view_embedder_2.h"
#include "flow/view_slicer.h"
#include "flutter/common/constants.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/fml/trace_event.h"
#include "fml/make_copyable.h"

namespace flutter {

AndroidExternalViewEmbedder2::AndroidExternalViewEmbedder2(
    const AndroidContext& android_context,
    std::shared_ptr<PlatformViewAndroidJNI> jni_facade,
    std::shared_ptr<AndroidSurfaceFactory> surface_factory,
    const TaskRunners& task_runners)
    : ExternalViewEmbedder(),
      android_context_(android_context),
      jni_facade_(std::move(jni_facade)),
      surface_factory_(std::move(surface_factory)),
      surface_pool_(
          std::make_unique<SurfacePool>(/*use_new_surface_methods=*/true)),
      task_runners_(task_runners) {}

// |ExternalViewEmbedder|
void AndroidExternalViewEmbedder2::PrerollCompositeEmbeddedView(
    int64_t view_id,
    std::unique_ptr<EmbeddedViewParams> params) {
  TRACE_EVENT0("flutter",
               "AndroidExternalViewEmbedder2::PrerollCompositeEmbeddedView");

  SkRect view_bounds = SkRect::Make(frame_size_);
  std::unique_ptr<EmbedderViewSlice> view;
  view = std::make_unique<DisplayListEmbedderViewSlice>(view_bounds);
  slices_.insert_or_assign(view_id, std::move(view));

  composition_order_.push_back(view_id);
  // Update params only if they changed.
  if (view_params_.count(view_id) == 1 &&
      view_params_.at(view_id) == *params.get()) {
    return;
  }
  view_params_.insert_or_assign(view_id, EmbeddedViewParams(*params.get()));
}

// |ExternalViewEmbedder|
DlCanvas* AndroidExternalViewEmbedder2::CompositeEmbeddedView(int64_t view_id) {
  if (slices_.count(view_id) == 1) {
    return slices_.at(view_id)->canvas();
  }
  return nullptr;
}

SkRect AndroidExternalViewEmbedder2::GetViewRect(
    int64_t view_id,
    const std::unordered_map<int64_t, EmbeddedViewParams>& view_params) {
  const EmbeddedViewParams& params = view_params.at(view_id);
  // https://github.com/flutter/flutter/issues/59821
  return SkRect::MakeXYWH(params.finalBoundingRect().x(),      //
                          params.finalBoundingRect().y(),      //
                          params.finalBoundingRect().width(),  //
                          params.finalBoundingRect().height()  //
  );
}

// |ExternalViewEmbedder|
void AndroidExternalViewEmbedder2::SubmitFlutterView(
    int64_t flutter_view_id,
    GrDirectContext* context,
    const std::shared_ptr<impeller::AiksContext>& aiks_context,
    std::unique_ptr<SurfaceFrame> frame) {
  TRACE_EVENT0("flutter", "AndroidExternalViewEmbedder2::SubmitFlutterView");

  if (!FrameHasPlatformLayers()) {
    frame->Submit();
    jni_facade_->applyTransaction();
    return;
  }

  std::unordered_map<int64_t, SkRect> view_rects;
  for (auto platform_id : composition_order_) {
    view_rects[platform_id] = GetViewRect(platform_id, view_params_);
  }

  std::unordered_map<int64_t, SkRect> overlay_layers =
      SliceViews(frame->Canvas(),     //
                 composition_order_,  //
                 slices_,             //
                 view_rects           //
      );

  // Create Overlay frame.
  surface_pool_->TrimLayers();
  std::unique_ptr<SurfaceFrame> overlay_frame;
  if (surface_pool_->HasLayers()) {
    for (int64_t view_id : composition_order_) {
      std::unordered_map<int64_t, SkRect>::const_iterator overlay =
          overlay_layers.find(view_id);

      if (overlay == overlay_layers.end()) {
        continue;
      }
      if (overlay_frame == nullptr) {
        std::shared_ptr<OverlayLayer> layer = surface_pool_->GetLayer(
            context, android_context_, jni_facade_, surface_factory_);
        overlay_frame = layer->surface->AcquireFrame(frame_size_);
      }

      DlCanvas* overlay_canvas = overlay_frame->Canvas();
      int restore_count = overlay_canvas->GetSaveCount();
      overlay_canvas->Save();
      overlay_canvas->ClipRect(overlay->second);
      overlay_canvas->Clear(DlColor::kTransparent());
      slices_[view_id]->render_into(overlay_canvas);
      overlay_canvas->RestoreToCount(restore_count);
    }
  }
  if (overlay_frame != nullptr) {
    overlay_frame->set_submit_info({.frame_boundary = false});
    overlay_frame->Submit();
  }
  frame->Submit();

  task_runners_.GetPlatformTaskRunner()->PostTask(fml::MakeCopyable(
      [&, composition_order = composition_order_, view_params = view_params_,
       jni_facade = jni_facade_, device_pixel_ratio = device_pixel_ratio_,
       slices = std::move(slices_)]() -> void {
        jni_facade->swapTransaction();
        for (int64_t view_id : composition_order) {
          SkRect view_rect = GetViewRect(view_id, view_params);
          const EmbeddedViewParams& params = view_params.at(view_id);
          // Display the platform view. If it's already displayed, then it's
          // just positioned and sized.
          jni_facade->FlutterViewOnDisplayPlatformView(
              view_id,             //
              view_rect.x(),       //
              view_rect.y(),       //
              view_rect.width(),   //
              view_rect.height(),  //
              params.sizePoints().width() * device_pixel_ratio,
              params.sizePoints().height() * device_pixel_ratio,
              params.mutatorsStack()  //
          );
        }
        if (!surface_pool_->HasLayers()) {
          surface_pool_->GetLayer(context, android_context_, jni_facade_,
                                  surface_factory_);
        }
        jni_facade->FlutterViewEndFrame();
      }));
}

// |ExternalViewEmbedder|
std::unique_ptr<SurfaceFrame>
AndroidExternalViewEmbedder2::CreateSurfaceIfNeeded(GrDirectContext* context,
                                                    int64_t view_id,
                                                    EmbedderViewSlice* slice,
                                                    const SkRect& rect) {
  std::shared_ptr<OverlayLayer> layer = surface_pool_->GetLayer(
      context, android_context_, jni_facade_, surface_factory_);

  std::unique_ptr<SurfaceFrame> frame =
      layer->surface->AcquireFrame(frame_size_);

  DlCanvas* overlay_canvas = frame->Canvas();
  overlay_canvas->Clear(DlColor::kTransparent());
  // Offset the picture since its absolute position on the scene is determined
  // by the position of the overlay view.
  slice->render_into(overlay_canvas);
  return frame;
}

// |ExternalViewEmbedder|
PostPrerollResult AndroidExternalViewEmbedder2::PostPrerollAction(
    const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {
  return PostPrerollResult::kSuccess;
}

bool AndroidExternalViewEmbedder2::FrameHasPlatformLayers() {
  return !composition_order_.empty();
}

// |ExternalViewEmbedder|
DlCanvas* AndroidExternalViewEmbedder2::GetRootCanvas() {
  // On Android, the root surface is created from the on-screen render target.
  return nullptr;
}

void AndroidExternalViewEmbedder2::Reset() {
  previous_frame_view_count_ = composition_order_.size();

  composition_order_.clear();
  slices_.clear();
}

// |ExternalViewEmbedder|
void AndroidExternalViewEmbedder2::BeginFrame(
    GrDirectContext* context,
    const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {}

// |ExternalViewEmbedder|
void AndroidExternalViewEmbedder2::PrepareFlutterView(
    SkISize frame_size,
    double device_pixel_ratio) {
  Reset();

  // The surface size changed. Therefore, destroy existing surfaces as
  // the existing surfaces in the pool can't be recycled.
  if (frame_size_ != frame_size) {
    DestroySurfaces();
  }
  surface_pool_->SetFrameSize(frame_size);

  frame_size_ = frame_size;
  device_pixel_ratio_ = device_pixel_ratio;
}

// |ExternalViewEmbedder|
void AndroidExternalViewEmbedder2::CancelFrame() {
  Reset();
}

// |ExternalViewEmbedder|
void AndroidExternalViewEmbedder2::EndFrame(
    bool should_resubmit_frame,
    const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {}

// |ExternalViewEmbedder|
bool AndroidExternalViewEmbedder2::SupportsDynamicThreadMerging() {
  return false;
}

// |ExternalViewEmbedder|
void AndroidExternalViewEmbedder2::Teardown() {
  DestroySurfaces();
}

// |ExternalViewEmbedder|
void AndroidExternalViewEmbedder2::DestroySurfaces() {
  if (!surface_pool_->HasLayers()) {
    return;
  }
  fml::AutoResetWaitableEvent latch;
  fml::TaskRunner::RunNowOrPostTask(task_runners_.GetPlatformTaskRunner(),
                                    [&]() {
                                      surface_pool_->DestroyLayers(jni_facade_);
                                      latch.Signal();
                                    });
  latch.Wait();
}

}  // namespace flutter
