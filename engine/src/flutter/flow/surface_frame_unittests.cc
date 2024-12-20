// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include "flutter/flow/surface_frame.h"
#include "flutter/testing/testing.h"

namespace flutter {

TEST(FlowTest, SurfaceFrameDoesNotSubmitInDtor) {
  SurfaceFrame::FramebufferInfo framebuffer_info;
  auto callback = [](const SurfaceFrame&) {
    EXPECT_FALSE(true);
    return true;
  };
  auto encode_callback = [](const SurfaceFrame&, DlCanvas*) {
    EXPECT_FALSE(true);
    return true;
  };

  auto surface_frame = std::make_unique<SurfaceFrame>(
      /*surface=*/nullptr,
      /*framebuffer_info=*/framebuffer_info,
      /*encode_callback=*/encode_callback,
      /*submit_callback=*/callback,
      /*frame_size=*/SkISize::Make(800, 600));
  surface_frame.reset();
}

TEST(FlowTest, SurfaceFrameDoesNotHaveEmptyCanvas) {
  SurfaceFrame::FramebufferInfo framebuffer_info;
  auto callback = [](const SurfaceFrame&, DlCanvas*) { return true; };
  auto submit_callback = [](const SurfaceFrame&) { return true; };
  SurfaceFrame frame(
      /*surface=*/nullptr,
      /*framebuffer_info=*/framebuffer_info,
      /*encode_callback=*/callback,
      /*submit_callback=*/submit_callback,
      /*frame_size=*/SkISize::Make(800, 600),
      /*context_result=*/nullptr,
      /*display_list_fallback=*/true);

  EXPECT_FALSE(frame.Canvas()->GetLocalClipBounds().isEmpty());
  EXPECT_FALSE(frame.Canvas()->QuickReject(SkRect::MakeLTRB(10, 10, 50, 50)));
}

TEST(FlowTest, SurfaceFrameDoesNotPrepareRtree) {
  SurfaceFrame::FramebufferInfo framebuffer_info;
  auto callback = [](const SurfaceFrame&, DlCanvas*) { return true; };
  auto submit_callback = [](const SurfaceFrame&) { return true; };
  auto surface_frame = std::make_unique<SurfaceFrame>(
      /*surface=*/nullptr,
      /*framebuffer_info=*/framebuffer_info,
      /*encode_callback=*/callback,
      /*submit_callback=*/submit_callback,
      /*frame_size=*/SkISize::Make(800, 600),
      /*context_result=*/nullptr,
      /*display_list_fallback=*/true);
  surface_frame->Canvas()->DrawRect(SkRect::MakeWH(100, 100), DlPaint());
  EXPECT_FALSE(surface_frame->BuildDisplayList()->has_rtree());
}

}  // namespace flutter
