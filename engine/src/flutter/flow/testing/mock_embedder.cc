// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/testing/mock_embedder.h"

namespace flutter {
namespace testing {

MockViewEmbedder::MockViewEmbedder() = default;

MockViewEmbedder::~MockViewEmbedder() = default;

void MockViewEmbedder::AddCanvas(DlCanvas* canvas) {
  contexts_.emplace_back(canvas);
}

// |ExternalViewEmbedder|
DlCanvas* MockViewEmbedder::GetRootCanvas() {
  return nullptr;
}

// |ExternalViewEmbedder|
void MockViewEmbedder::CancelFrame() {}

// |ExternalViewEmbedder|
void MockViewEmbedder::BeginFrame(
    GrDirectContext* context,
    const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {}

// |ExternalViewEmbedder|
void MockViewEmbedder::PrepareFlutterView(SkISize frame_size,
                                          double device_pixel_ratio) {}

// |ExternalViewEmbedder|
void MockViewEmbedder::PrerollCompositeEmbeddedView(
    int64_t view_id,
    std::unique_ptr<EmbeddedViewParams> params) {
  prerolled_views_.emplace_back(view_id);
}

// |ExternalViewEmbedder|
DlCanvas* MockViewEmbedder::CompositeEmbeddedView(int64_t view_id) {
  painted_views_.emplace_back(view_id);
  DlCanvas* canvas = contexts_.front();
  contexts_.pop_front();
  return canvas;
}

}  // namespace testing
}  // namespace flutter
