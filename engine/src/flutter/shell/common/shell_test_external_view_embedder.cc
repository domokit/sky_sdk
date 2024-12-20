// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "shell_test_external_view_embedder.h"

namespace flutter {

ShellTestExternalViewEmbedder::ShellTestExternalViewEmbedder(
    const EndFrameCallBack& end_frame_call_back,
    PostPrerollResult post_preroll_result,
    bool support_thread_merging)
    : end_frame_call_back_(end_frame_call_back),
      post_preroll_result_(post_preroll_result),
      support_thread_merging_(support_thread_merging),
      submitted_frame_count_(0) {}

void ShellTestExternalViewEmbedder::UpdatePostPrerollResult(
    PostPrerollResult post_preroll_result) {
  post_preroll_result_ = post_preroll_result;
}

int ShellTestExternalViewEmbedder::GetSubmittedFrameCount() {
  return submitted_frame_count_;
}

SkISize ShellTestExternalViewEmbedder::GetLastSubmittedFrameSize() {
  return last_submitted_frame_size_;
}

std::vector<int64_t> ShellTestExternalViewEmbedder::GetVisitedPlatformViews() {
  return visited_platform_views_;
}

MutatorsStack ShellTestExternalViewEmbedder::GetStack(int64_t view_id) {
  return mutators_stacks_[view_id];
}

// |ExternalViewEmbedder|
void ShellTestExternalViewEmbedder::CancelFrame() {}

// |ExternalViewEmbedder|
void ShellTestExternalViewEmbedder::BeginFrame(
    GrDirectContext* context,
    const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {}

// |ExternalViewEmbedder|
void ShellTestExternalViewEmbedder::PrepareFlutterView(
    SkISize frame_size,
    double device_pixel_ratio) {
  visited_platform_views_.clear();
  mutators_stacks_.clear();
  current_composition_params_.clear();
}

// |ExternalViewEmbedder|
void ShellTestExternalViewEmbedder::PrerollCompositeEmbeddedView(
    int64_t view_id,
    std::unique_ptr<EmbeddedViewParams> params) {
  SkRect view_bounds = SkRect::Make(frame_size_);
  auto view = std::make_unique<DisplayListEmbedderViewSlice>(view_bounds);
  slices_.insert_or_assign(view_id, std::move(view));
}

// |ExternalViewEmbedder|
PostPrerollResult ShellTestExternalViewEmbedder::PostPrerollAction(
    const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {
  FML_DCHECK(raster_thread_merger);
  return post_preroll_result_;
}

// |ExternalViewEmbedder|
void ShellTestExternalViewEmbedder::PushVisitedPlatformView(int64_t view_id) {
  visited_platform_views_.push_back(view_id);
}

// |ExternalViewEmbedder|
void ShellTestExternalViewEmbedder::PushFilterToVisitedPlatformViews(
    const std::shared_ptr<DlImageFilter>& filter,
    const SkRect& filter_rect) {
  for (int64_t id : visited_platform_views_) {
    EmbeddedViewParams params = current_composition_params_[id];
    params.PushImageFilter(filter, filter_rect);
    current_composition_params_[id] = params;
    mutators_stacks_[id] = params.mutatorsStack();
  }
}

DlCanvas* ShellTestExternalViewEmbedder::CompositeEmbeddedView(
    int64_t view_id) {
  return slices_[view_id]->canvas();
}

// |ExternalViewEmbedder|
void ShellTestExternalViewEmbedder::SubmitFlutterView(
    int64_t flutter_view_id,
    GrDirectContext* context,
    const std::shared_ptr<impeller::AiksContext>& aiks_context,
    std::unique_ptr<SurfaceFrame> frame) {
  if (!frame) {
    return;
  }
  frame->Submit();
  if (frame->SkiaSurface()) {
    last_submitted_frame_size_ = SkISize::Make(frame->SkiaSurface()->width(),
                                               frame->SkiaSurface()->height());
  } else {
    last_submitted_frame_size_ = SkISize::MakeEmpty();
  }
  submitted_frame_count_++;
}

// |ExternalViewEmbedder|
void ShellTestExternalViewEmbedder::EndFrame(
    bool should_resubmit_frame,
    const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {
  end_frame_call_back_(should_resubmit_frame, raster_thread_merger);
}

// |ExternalViewEmbedder|
DlCanvas* ShellTestExternalViewEmbedder::GetRootCanvas() {
  return nullptr;
}

bool ShellTestExternalViewEmbedder::SupportsDynamicThreadMerging() {
  return support_thread_merging_;
}

}  // namespace flutter
