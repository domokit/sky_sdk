// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_TESTING_MOCK_EMBEDDER_H_
#define FLUTTER_FLOW_TESTING_MOCK_EMBEDDER_H_

#include "flutter/flow/embedded_views.h"

namespace flutter {
namespace testing {

class MockViewEmbedder : public ExternalViewEmbedder {
 public:
  MockViewEmbedder();

  ~MockViewEmbedder();

  void AddCanvas(DlCanvas* canvas);

  // |ExternalViewEmbedder|
  DlCanvas* GetRootCanvas() override;

  // |ExternalViewEmbedder|
  void CancelFrame() override;

  // |ExternalViewEmbedder|
  void BeginFrame(GrDirectContext* context,
                  const fml::RefPtr<fml::RasterThreadMerger>&
                      raster_thread_merger) override;

  // |ExternalViewEmbedder|
  void PrepareFlutterView(SkISize frame_size,
                          double device_pixel_ratio) override;

  // |ExternalViewEmbedder|
  void PrerollCompositeEmbeddedView(
      int64_t view_id,
      std::unique_ptr<EmbeddedViewParams> params) override;

  // |ExternalViewEmbedder|
  DlCanvas* CompositeEmbeddedView(int64_t view_id) override;

  std::vector<int64_t> prerolled_views() const { return prerolled_views_; }
  std::vector<int64_t> painted_views() const { return painted_views_; }

 private:
  std::deque<DlCanvas*> contexts_;
  std::vector<int64_t> prerolled_views_;
  std::vector<int64_t> painted_views_;
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_FLOW_TESTING_MOCK_EMBEDDER_H_
