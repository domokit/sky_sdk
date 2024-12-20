// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include <memory>

#include "flutter/shell/platform/android/external_view_embedder/external_view_embedder.h"

#include "flutter/flow/embedded_views.h"
#include "flutter/flow/surface.h"
#include "flutter/fml/raster_thread_merger.h"
#include "flutter/fml/thread.h"
#include "flutter/shell/platform/android/jni/jni_mock.h"
#include "flutter/shell/platform/android/surface/android_surface.h"
#include "flutter/shell/platform/android/surface/android_surface_mock.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"

namespace flutter {
namespace testing {

using ::testing::ByMove;
using ::testing::Return;

constexpr int64_t kImplicitViewId = 0;

class TestAndroidSurfaceFactory : public AndroidSurfaceFactory {
 public:
  using TestSurfaceProducer =
      std::function<std::unique_ptr<AndroidSurface>(void)>;
  explicit TestAndroidSurfaceFactory(TestSurfaceProducer&& surface_producer) {
    surface_producer_ = surface_producer;
  }

  ~TestAndroidSurfaceFactory() override = default;

  std::unique_ptr<AndroidSurface> CreateSurface() override {
    return surface_producer_();
  }

 private:
  TestSurfaceProducer surface_producer_;
};

class SurfaceMock : public Surface {
 public:
  MOCK_METHOD(bool, IsValid, (), (override));

  MOCK_METHOD(std::unique_ptr<SurfaceFrame>,
              AcquireFrame,
              (const SkISize& size),
              (override));

  MOCK_METHOD(SkMatrix, GetRootTransformation, (), (const, override));

  MOCK_METHOD(GrDirectContext*, GetContext, (), (override));

  MOCK_METHOD(std::unique_ptr<GLContextResult>,
              MakeRenderContextCurrent,
              (),
              (override));
};

fml::RefPtr<fml::RasterThreadMerger> GetThreadMergerFromPlatformThread(
    fml::Thread* rasterizer_thread = nullptr) {
  // Assume the current thread is the platform thread.
  fml::MessageLoop::EnsureInitializedForCurrentThread();
  auto platform_queue_id = fml::MessageLoop::GetCurrentTaskQueueId();

  if (!rasterizer_thread) {
    return fml::MakeRefCounted<fml::RasterThreadMerger>(platform_queue_id,
                                                        platform_queue_id);
  }
  auto rasterizer_queue_id =
      rasterizer_thread->GetTaskRunner()->GetTaskQueueId();
  return fml::MakeRefCounted<fml::RasterThreadMerger>(platform_queue_id,
                                                      rasterizer_queue_id);
}

fml::RefPtr<fml::RasterThreadMerger> GetThreadMergerFromRasterThread(
    fml::Thread* platform_thread) {
  auto platform_queue_id = platform_thread->GetTaskRunner()->GetTaskQueueId();

  // Assume the current thread is the raster thread.
  fml::MessageLoop::EnsureInitializedForCurrentThread();
  auto rasterizer_queue_id = fml::MessageLoop::GetCurrentTaskQueueId();

  return fml::MakeRefCounted<fml::RasterThreadMerger>(platform_queue_id,
                                                      rasterizer_queue_id);
}

TaskRunners GetTaskRunnersForFixture() {
  fml::MessageLoop::EnsureInitializedForCurrentThread();
  auto& loop = fml::MessageLoop::GetCurrent();
  return {
      "test",
      loop.GetTaskRunner(),  // platform
      loop.GetTaskRunner(),  // raster
      loop.GetTaskRunner(),  // ui
      loop.GetTaskRunner()   // io
  };
}

TEST(AndroidExternalViewEmbedder, CompositeEmbeddedView) {
  auto android_context = AndroidContext(AndroidRenderingAPI::kSoftware);
  auto embedder = std::make_unique<AndroidExternalViewEmbedder>(
      android_context, nullptr, nullptr, GetTaskRunnersForFixture());

  ASSERT_EQ(nullptr, embedder->CompositeEmbeddedView(0));
  embedder->PrerollCompositeEmbeddedView(
      0, std::make_unique<EmbeddedViewParams>());
  ASSERT_NE(nullptr, embedder->CompositeEmbeddedView(0));

  ASSERT_EQ(nullptr, embedder->CompositeEmbeddedView(1));
  embedder->PrerollCompositeEmbeddedView(
      1, std::make_unique<EmbeddedViewParams>());
  ASSERT_NE(nullptr, embedder->CompositeEmbeddedView(1));
}

TEST(AndroidExternalViewEmbedder, CancelFrame) {
  auto android_context = AndroidContext(AndroidRenderingAPI::kSoftware);
  auto embedder = std::make_unique<AndroidExternalViewEmbedder>(
      android_context, nullptr, nullptr, GetTaskRunnersForFixture());

  embedder->PrerollCompositeEmbeddedView(
      0, std::make_unique<EmbeddedViewParams>());
  embedder->CancelFrame();

  ASSERT_EQ(embedder->CompositeEmbeddedView(0), nullptr);
}

TEST(AndroidExternalViewEmbedder, RasterizerRunsOnPlatformThread) {
  auto jni_mock = std::make_shared<JNIMock>();
  auto android_context = AndroidContext(AndroidRenderingAPI::kSoftware);
  auto embedder = std::make_unique<AndroidExternalViewEmbedder>(
      android_context, jni_mock, nullptr, GetTaskRunnersForFixture());

  fml::Thread rasterizer_thread("rasterizer");
  auto raster_thread_merger =
      GetThreadMergerFromPlatformThread(&rasterizer_thread);
  ASSERT_FALSE(raster_thread_merger->IsMerged());

  EXPECT_CALL(*jni_mock, FlutterViewBeginFrame());
  embedder->BeginFrame(nullptr, raster_thread_merger);
  embedder->PrepareFlutterView(SkISize::Make(10, 20), 1.0);

  // Push a platform view.
  embedder->PrerollCompositeEmbeddedView(
      0, std::make_unique<EmbeddedViewParams>());

  auto postpreroll_result = embedder->PostPrerollAction(raster_thread_merger);
  ASSERT_EQ(PostPrerollResult::kSkipAndRetryFrame, postpreroll_result);

  EXPECT_CALL(*jni_mock, FlutterViewEndFrame());
  embedder->EndFrame(/*should_resubmit_frame=*/true, raster_thread_merger);

  ASSERT_TRUE(raster_thread_merger->IsMerged());

  int pending_frames = 0;
  while (raster_thread_merger->IsMerged()) {
    raster_thread_merger->DecrementLease();
    pending_frames++;
  }
  ASSERT_EQ(10, pending_frames);  // kDefaultMergedLeaseDuration
}

TEST(AndroidExternalViewEmbedder, RasterizerRunsOnRasterizerThread) {
  auto jni_mock = std::make_shared<JNIMock>();
  auto android_context = AndroidContext(AndroidRenderingAPI::kSoftware);
  auto embedder = std::make_unique<AndroidExternalViewEmbedder>(
      android_context, jni_mock, nullptr, GetTaskRunnersForFixture());

  fml::Thread rasterizer_thread("rasterizer");
  auto raster_thread_merger =
      GetThreadMergerFromPlatformThread(&rasterizer_thread);
  ASSERT_FALSE(raster_thread_merger->IsMerged());

  PostPrerollResult result = embedder->PostPrerollAction(raster_thread_merger);
  ASSERT_EQ(PostPrerollResult::kSuccess, result);

  EXPECT_CALL(*jni_mock, FlutterViewEndFrame());
  embedder->EndFrame(/*should_resubmit_frame=*/true, raster_thread_merger);

  ASSERT_FALSE(raster_thread_merger->IsMerged());
}

TEST(AndroidExternalViewEmbedder, PlatformViewRect) {
  auto jni_mock = std::make_shared<JNIMock>();

  auto android_context = AndroidContext(AndroidRenderingAPI::kSoftware);
  auto embedder = std::make_unique<AndroidExternalViewEmbedder>(
      android_context, jni_mock, nullptr, GetTaskRunnersForFixture());
  fml::Thread rasterizer_thread("rasterizer");
  auto raster_thread_merger =
      GetThreadMergerFromPlatformThread(&rasterizer_thread);

  EXPECT_CALL(*jni_mock, FlutterViewBeginFrame());
  embedder->BeginFrame(nullptr, raster_thread_merger);
  embedder->PrepareFlutterView(SkISize::Make(100, 100), 1.5);

  MutatorsStack stack;
  SkMatrix matrix;
  matrix.setIdentity();
  // The framework always push a scale matrix based on the screen ratio.
  matrix.setConcat(matrix, SkMatrix::Scale(1.5, 1.5));
  matrix.setConcat(matrix, SkMatrix::Translate(10, 20));
  auto view_params =
      std::make_unique<EmbeddedViewParams>(matrix, SkSize::Make(30, 40), stack);

  auto view_id = 0;
  embedder->PrerollCompositeEmbeddedView(view_id, std::move(view_params));
  ASSERT_EQ(SkRect::MakeXYWH(15, 30, 45, 60), embedder->GetViewRect(view_id));
}

TEST(AndroidExternalViewEmbedder, PlatformViewRectChangedParams) {
  auto jni_mock = std::make_shared<JNIMock>();

  auto android_context = AndroidContext(AndroidRenderingAPI::kSoftware);
  auto embedder = std::make_unique<AndroidExternalViewEmbedder>(
      android_context, jni_mock, nullptr, GetTaskRunnersForFixture());
  fml::Thread rasterizer_thread("rasterizer");
  auto raster_thread_merger =
      GetThreadMergerFromPlatformThread(&rasterizer_thread);

  EXPECT_CALL(*jni_mock, FlutterViewBeginFrame());
  embedder->BeginFrame(nullptr, raster_thread_merger);
  embedder->PrepareFlutterView(SkISize::Make(100, 100), 1.5);

  auto view_id = 0;

  MutatorsStack stack1;
  SkMatrix matrix1;
  matrix1.setIdentity();
  // The framework always push a scale matrix based on the screen ratio.
  matrix1.setConcat(SkMatrix::Scale(1.5, 1.5), SkMatrix::Translate(10, 20));
  auto view_params_1 = std::make_unique<EmbeddedViewParams>(
      matrix1, SkSize::Make(30, 40), stack1);

  embedder->PrerollCompositeEmbeddedView(view_id, std::move(view_params_1));

  MutatorsStack stack2;
  SkMatrix matrix2;
  matrix2.setIdentity();
  // The framework always push a scale matrix based on the screen ratio.
  matrix2.setConcat(matrix2, SkMatrix::Scale(1.5, 1.5));
  matrix2.setConcat(matrix2, SkMatrix::Translate(50, 60));
  auto view_params_2 = std::make_unique<EmbeddedViewParams>(
      matrix2, SkSize::Make(70, 80), stack2);

  embedder->PrerollCompositeEmbeddedView(view_id, std::move(view_params_2));

  ASSERT_EQ(SkRect::MakeXYWH(75, 90, 105, 120), embedder->GetViewRect(view_id));
}

TEST(AndroidExternalViewEmbedder, SubmitFlutterView) {
  auto jni_mock = std::make_shared<JNIMock>();
  auto android_context =
      std::make_shared<AndroidContext>(AndroidRenderingAPI::kSoftware);

  auto window = fml::MakeRefCounted<AndroidNativeWindow>(nullptr);
  auto gr_context = GrDirectContext::MakeMock(nullptr);
  auto frame_size = SkISize::Make(1000, 1000);
  SurfaceFrame::FramebufferInfo framebuffer_info;
  auto surface_factory = std::make_shared<TestAndroidSurfaceFactory>(
      [gr_context, window, frame_size, framebuffer_info]() {
        auto surface_frame_1 = std::make_unique<SurfaceFrame>(
            SkSurfaces::Null(1000, 1000), framebuffer_info,
            [](const SurfaceFrame& surface_frame, DlCanvas* canvas) {
              return true;
            },
            [](const SurfaceFrame& surface_frame) { return true; },
            /*frame_size=*/SkISize::Make(800, 600));
        auto surface_frame_2 = std::make_unique<SurfaceFrame>(
            SkSurfaces::Null(1000, 1000), framebuffer_info,
            [](const SurfaceFrame& surface_frame, DlCanvas* canvas) {
              return true;
            },
            [](const SurfaceFrame& surface_frame) { return true; },
            /*frame_size=*/SkISize::Make(800, 600));

        auto surface_mock = std::make_unique<SurfaceMock>();
        EXPECT_CALL(*surface_mock, AcquireFrame(frame_size))
            .Times(2 /* frames */)
            .WillOnce(Return(ByMove(std::move(surface_frame_1))))
            .WillOnce(Return(ByMove(std::move(surface_frame_2))));

        auto android_surface_mock = std::make_unique<AndroidSurfaceMock>();
        EXPECT_CALL(*android_surface_mock, IsValid()).WillOnce(Return(true));

        EXPECT_CALL(*android_surface_mock, CreateGPUSurface(gr_context.get()))
            .WillOnce(Return(ByMove(std::move(surface_mock))));

        EXPECT_CALL(*android_surface_mock, SetNativeWindow(window));

        return android_surface_mock;
      });
  auto embedder = std::make_unique<AndroidExternalViewEmbedder>(
      *android_context, jni_mock, surface_factory, GetTaskRunnersForFixture());

  auto raster_thread_merger = GetThreadMergerFromPlatformThread();

  // ------------------ First frame ------------------ //
  {
    auto did_submit_frame = false;
    auto surface_frame = std::make_unique<SurfaceFrame>(
        SkSurfaces::Null(1000, 1000), framebuffer_info,
        [&did_submit_frame](const SurfaceFrame& surface_frame,
                            DlCanvas* canvas) mutable {
          if (canvas != nullptr) {
            did_submit_frame = true;
          }
          return true;
        },
        [](const SurfaceFrame& surface_frame) { return true; },
        /*frame_size=*/SkISize::Make(800, 600));

    embedder->SubmitFlutterView(kImplicitViewId, gr_context.get(), nullptr,
                                std::move(surface_frame));
    // Submits frame if no Android view in the current frame.
    EXPECT_TRUE(did_submit_frame);
    // Doesn't resubmit frame.
    auto postpreroll_result = embedder->PostPrerollAction(raster_thread_merger);
    ASSERT_EQ(PostPrerollResult::kSuccess, postpreroll_result);

    EXPECT_CALL(*jni_mock, FlutterViewEndFrame());
    embedder->EndFrame(/*should_resubmit_frame=*/false, raster_thread_merger);
  }

  // ------------------ Second frame ------------------ //
  {
    EXPECT_CALL(*jni_mock, FlutterViewBeginFrame());
    embedder->BeginFrame(nullptr, raster_thread_merger);
    embedder->PrepareFlutterView(frame_size, 1.5);

    // Add an Android view.
    MutatorsStack stack1;
    SkMatrix matrix1;
    matrix1.setIdentity();
    SkMatrix scale = SkMatrix::Scale(1.5, 1.5);
    SkMatrix trans = SkMatrix::Translate(100, 100);
    matrix1.setConcat(scale, trans);
    stack1.PushTransform(scale);
    stack1.PushTransform(trans);
    // TODO(egarciad): Investigate why Flow applies the device pixel ratio to
    // the offsetPixels, but not the sizePoints.
    auto view_params_1 = std::make_unique<EmbeddedViewParams>(
        matrix1, SkSize::Make(200, 200), stack1);

    embedder->PrerollCompositeEmbeddedView(0, std::move(view_params_1));
    // This is the recording canvas flow writes to.
    auto canvas_1 = embedder->CompositeEmbeddedView(0);

    auto rect_paint = DlPaint();
    rect_paint.setColor(DlColor::kCyan());
    rect_paint.setDrawStyle(DlDrawStyle::kFill);

    // This simulates Flutter UI that doesn't intersect with the Android view.
    canvas_1->DrawRect(SkRect::MakeXYWH(0, 0, 50, 50), rect_paint);
    // This simulates Flutter UI that intersects with the Android view.
    canvas_1->DrawRect(SkRect::MakeXYWH(50, 50, 200, 200), rect_paint);
    canvas_1->DrawRect(SkRect::MakeXYWH(150, 150, 100, 100), rect_paint);

    // Create a new overlay surface.
    EXPECT_CALL(*jni_mock, FlutterViewCreateOverlaySurface())
        .WillOnce(Return(
            ByMove(std::make_unique<PlatformViewAndroidJNI::OverlayMetadata>(
                0, window))));
    // The JNI call to display the Android view.
    EXPECT_CALL(*jni_mock, FlutterViewOnDisplayPlatformView(
                               0, 150, 150, 300, 300, 300, 300, stack1));
    // The JNI call to display the overlay surface.
    EXPECT_CALL(*jni_mock,
                FlutterViewDisplayOverlaySurface(0, 150, 150, 100, 100));

    auto did_submit_frame = false;
    auto surface_frame = std::make_unique<SurfaceFrame>(
        SkSurfaces::Null(1000, 1000), framebuffer_info,
        [&did_submit_frame](const SurfaceFrame& surface_frame,
                            DlCanvas* canvas) mutable {
          if (canvas != nullptr) {
            did_submit_frame = true;
          }
          return true;
        },
        [](const SurfaceFrame& surface_frame) { return true; },
        /*frame_size=*/SkISize::Make(800, 600));

    embedder->SubmitFlutterView(kImplicitViewId, gr_context.get(), nullptr,
                                std::move(surface_frame));
    // Doesn't submit frame if there aren't Android views in the previous frame.
    EXPECT_FALSE(did_submit_frame);
    // Resubmits frame.
    auto postpreroll_result = embedder->PostPrerollAction(raster_thread_merger);
    ASSERT_EQ(PostPrerollResult::kResubmitFrame, postpreroll_result);

    EXPECT_CALL(*jni_mock, FlutterViewEndFrame());
    embedder->EndFrame(/*should_resubmit_frame=*/false, raster_thread_merger);
  }

  // ------------------ Third frame ------------------ //
  {
    EXPECT_CALL(*jni_mock, FlutterViewBeginFrame());
    embedder->BeginFrame(nullptr, raster_thread_merger);
    embedder->PrepareFlutterView(frame_size, 1.5);

    // Add an Android view.
    MutatorsStack stack1;
    SkMatrix matrix1;
    matrix1.setIdentity();
    SkMatrix scale = SkMatrix::Scale(1.5, 1.5);
    SkMatrix trans = SkMatrix::Translate(100, 100);
    matrix1.setConcat(scale, trans);
    stack1.PushTransform(scale);
    stack1.PushTransform(trans);
    // TODO(egarciad): Investigate why Flow applies the device pixel ratio to
    // the offsetPixels, but not the sizePoints.
    auto view_params_1 = std::make_unique<EmbeddedViewParams>(
        matrix1, SkSize::Make(200, 200), stack1);

    embedder->PrerollCompositeEmbeddedView(0, std::move(view_params_1));
    // This is the recording canvas flow writes to.
    auto canvas_1 = embedder->CompositeEmbeddedView(0);

    auto rect_paint = DlPaint();
    rect_paint.setColor(DlColor::kCyan());
    rect_paint.setDrawStyle(DlDrawStyle::kFill);

    // This simulates Flutter UI that doesn't intersect with the Android view.
    canvas_1->DrawRect(SkRect::MakeXYWH(0, 0, 50, 50), rect_paint);
    // This simulates Flutter UI that intersects with the Android view.
    canvas_1->DrawRect(SkRect::MakeXYWH(50, 50, 200, 200), rect_paint);
    canvas_1->DrawRect(SkRect::MakeXYWH(150, 150, 100, 100), rect_paint);

    // Don't create a new overlay surface since it's recycled from the first
    // frame.
    EXPECT_CALL(*jni_mock, FlutterViewCreateOverlaySurface()).Times(0);
    // The JNI call to display the Android view.
    EXPECT_CALL(*jni_mock, FlutterViewOnDisplayPlatformView(
                               0, 150, 150, 300, 300, 300, 300, stack1));
    // The JNI call to display the overlay surface.
    EXPECT_CALL(*jni_mock,
                FlutterViewDisplayOverlaySurface(0, 150, 150, 100, 100));

    auto did_submit_frame = false;
    auto surface_frame = std::make_unique<SurfaceFrame>(
        SkSurfaces::Null(1000, 1000), framebuffer_info,
        [&did_submit_frame](const SurfaceFrame& surface_frame,
                            DlCanvas* canvas) mutable {
          if (canvas != nullptr) {
            did_submit_frame = true;
          }
          return true;
        },
        [](const SurfaceFrame& surface_frame) { return true; },
        /*frame_size=*/SkISize::Make(800, 600));
    embedder->SubmitFlutterView(kImplicitViewId, gr_context.get(), nullptr,
                                std::move(surface_frame));
    // Submits frame if there are Android views in the previous frame.
    EXPECT_TRUE(did_submit_frame);
    // Doesn't resubmit frame.
    auto postpreroll_result = embedder->PostPrerollAction(raster_thread_merger);
    ASSERT_EQ(PostPrerollResult::kSuccess, postpreroll_result);

    EXPECT_CALL(*jni_mock, FlutterViewEndFrame());
    embedder->EndFrame(/*should_resubmit_frame=*/false, raster_thread_merger);
  }
}

TEST(AndroidExternalViewEmbedder, OverlayCoverTwoPlatformViews) {
  // In this test we will simulate two Android views appearing on the screen
  // with a rect intersecting both of them

  auto jni_mock = std::make_shared<JNIMock>();
  auto android_context =
      std::make_shared<AndroidContext>(AndroidRenderingAPI::kSoftware);

  auto window = fml::MakeRefCounted<AndroidNativeWindow>(nullptr);
  auto gr_context = GrDirectContext::MakeMock(nullptr);
  auto frame_size = SkISize::Make(1000, 1000);
  SurfaceFrame::FramebufferInfo framebuffer_info;
  auto surface_factory = std::make_shared<TestAndroidSurfaceFactory>(
      [gr_context, window, frame_size, framebuffer_info]() {
        auto surface_frame_1 = std::make_unique<SurfaceFrame>(
            SkSurfaces::Null(1000, 1000), framebuffer_info,
            [](const SurfaceFrame& surface_frame, DlCanvas* canvas) {
              return true;
            },
            [](const SurfaceFrame& surface_frame) { return true; },
            /*frame_size=*/SkISize::Make(800, 600));

        auto surface_mock = std::make_unique<SurfaceMock>();
        EXPECT_CALL(*surface_mock, AcquireFrame(frame_size))
            .Times(1 /* frames */)
            .WillOnce(Return(ByMove(std::move(surface_frame_1))));

        auto android_surface_mock = std::make_unique<AndroidSurfaceMock>();
        EXPECT_CALL(*android_surface_mock, IsValid()).WillOnce(Return(true));

        EXPECT_CALL(*android_surface_mock, CreateGPUSurface(gr_context.get()))
            .WillOnce(Return(ByMove(std::move(surface_mock))));

        EXPECT_CALL(*android_surface_mock, SetNativeWindow(window));
        return android_surface_mock;
      });
  auto embedder = std::make_unique<AndroidExternalViewEmbedder>(
      *android_context, jni_mock, surface_factory, GetTaskRunnersForFixture());

  auto raster_thread_merger = GetThreadMergerFromPlatformThread();

  EXPECT_CALL(*jni_mock, FlutterViewBeginFrame());
  embedder->BeginFrame(nullptr, raster_thread_merger);
  embedder->PrepareFlutterView(frame_size, 1.5);

  {
    // Add first Android view.
    SkMatrix matrix = SkMatrix::Translate(100, 100);
    MutatorsStack stack;
    embedder->PrerollCompositeEmbeddedView(
        0, std::make_unique<EmbeddedViewParams>(matrix, SkSize::Make(100, 100),
                                                stack));
    // The JNI call to display the Android view.
    EXPECT_CALL(*jni_mock, FlutterViewOnDisplayPlatformView(
                               0, 100, 100, 100, 100, 150, 150, stack));
  }

  {
    // Add second Android view.
    SkMatrix matrix = SkMatrix::Translate(300, 100);
    MutatorsStack stack;
    embedder->PrerollCompositeEmbeddedView(
        1, std::make_unique<EmbeddedViewParams>(matrix, SkSize::Make(100, 100),
                                                stack));
    // The JNI call to display the Android view.
    EXPECT_CALL(*jni_mock, FlutterViewOnDisplayPlatformView(
                               1, 300, 100, 100, 100, 150, 150, stack));
  }
  auto rect_paint = DlPaint();
  rect_paint.setColor(DlColor::kCyan());
  rect_paint.setDrawStyle(DlDrawStyle::kFill);

  // This simulates Flutter UI that intersects with the two Android views.
  // Since we will compute the intersection for each android view in turn, and
  // finally merge The final size of the overlay will be smaller than the
  // width and height of the rect.
  embedder->CompositeEmbeddedView(1)->DrawRect(
      SkRect::MakeXYWH(150, 50, 200, 200), rect_paint);

  EXPECT_CALL(*jni_mock, FlutterViewCreateOverlaySurface())
      .WillRepeatedly([&]() {
        return std::make_unique<PlatformViewAndroidJNI::OverlayMetadata>(
            1, window);
      });

  // The JNI call to display the overlay surface.
  EXPECT_CALL(*jni_mock,
              FlutterViewDisplayOverlaySurface(1, 150, 100, 200, 100))
      .Times(1);

  auto surface_frame = std::make_unique<SurfaceFrame>(
      SkSurfaces::Null(1000, 1000), framebuffer_info,
      [](const SurfaceFrame& surface_frame, DlCanvas* canvas) mutable {
        return true;
      },
      [](const SurfaceFrame& surface_frame) { return true; },
      /*frame_size=*/SkISize::Make(800, 600));

  embedder->SubmitFlutterView(kImplicitViewId, gr_context.get(), nullptr,
                              std::move(surface_frame));

  EXPECT_CALL(*jni_mock, FlutterViewEndFrame());
  embedder->EndFrame(/*should_resubmit_frame=*/false, raster_thread_merger);
}

TEST(AndroidExternalViewEmbedder, SubmitFrameOverlayComposition) {
  auto jni_mock = std::make_shared<JNIMock>();
  auto android_context =
      std::make_shared<AndroidContext>(AndroidRenderingAPI::kSoftware);

  auto window = fml::MakeRefCounted<AndroidNativeWindow>(nullptr);
  auto gr_context = GrDirectContext::MakeMock(nullptr);
  auto frame_size = SkISize::Make(1000, 1000);
  SurfaceFrame::FramebufferInfo framebuffer_info;
  auto surface_factory = std::make_shared<TestAndroidSurfaceFactory>(
      [gr_context, window, frame_size, framebuffer_info]() {
        auto surface_frame_1 = std::make_unique<SurfaceFrame>(
            SkSurfaces::Null(1000, 1000), framebuffer_info,
            [](const SurfaceFrame& surface_frame, DlCanvas* canvas) {
              return true;
            },
            [](const SurfaceFrame& surface_frame) { return true; },
            /*frame_size=*/SkISize::Make(800, 600));

        auto surface_mock = std::make_unique<SurfaceMock>();
        EXPECT_CALL(*surface_mock, AcquireFrame(frame_size))
            .Times(1 /* frames */)
            .WillOnce(Return(ByMove(std::move(surface_frame_1))));

        auto android_surface_mock = std::make_unique<AndroidSurfaceMock>();
        EXPECT_CALL(*android_surface_mock, IsValid()).WillOnce(Return(true));

        EXPECT_CALL(*android_surface_mock, CreateGPUSurface(gr_context.get()))
            .WillOnce(Return(ByMove(std::move(surface_mock))));

        EXPECT_CALL(*android_surface_mock, SetNativeWindow(window));
        return android_surface_mock;
      });
  auto embedder = std::make_unique<AndroidExternalViewEmbedder>(
      *android_context, jni_mock, surface_factory, GetTaskRunnersForFixture());

  auto raster_thread_merger = GetThreadMergerFromPlatformThread();

  EXPECT_CALL(*jni_mock, FlutterViewBeginFrame());
  embedder->BeginFrame(nullptr, raster_thread_merger);
  embedder->PrepareFlutterView(frame_size, 1.5);

  {
    // Add first Android view.
    SkMatrix matrix;
    MutatorsStack stack;
    stack.PushTransform(SkMatrix::Translate(0, 0));

    embedder->PrerollCompositeEmbeddedView(
        0, std::make_unique<EmbeddedViewParams>(matrix, SkSize::Make(200, 200),
                                                stack));
    EXPECT_CALL(*jni_mock, FlutterViewOnDisplayPlatformView(0, 0, 0, 200, 200,
                                                            300, 300, stack));
  }

  auto rect_paint = DlPaint();
  rect_paint.setColor(DlColor::kCyan());
  rect_paint.setDrawStyle(DlDrawStyle::kFill);

  // This simulates Flutter UI that intersects with the first Android view.
  embedder->CompositeEmbeddedView(0)->DrawRect(
      SkRect::MakeXYWH(25, 25, 80, 150), rect_paint);

  {
    // Add second Android view.
    SkMatrix matrix;
    MutatorsStack stack;
    stack.PushTransform(SkMatrix::Translate(0, 100));

    embedder->PrerollCompositeEmbeddedView(
        1, std::make_unique<EmbeddedViewParams>(matrix, SkSize::Make(100, 100),
                                                stack));
    EXPECT_CALL(*jni_mock, FlutterViewOnDisplayPlatformView(1, 0, 0, 100, 100,
                                                            150, 150, stack));
  }
  // This simulates Flutter UI that intersects with the first and second Android
  // views.
  embedder->CompositeEmbeddedView(1)->DrawRect(SkRect::MakeXYWH(25, 25, 80, 50),
                                               rect_paint);

  embedder->CompositeEmbeddedView(1)->DrawRect(
      SkRect::MakeXYWH(75, 75, 30, 100), rect_paint);

  EXPECT_CALL(*jni_mock, FlutterViewCreateOverlaySurface())
      .WillRepeatedly([&]() {
        return std::make_unique<PlatformViewAndroidJNI::OverlayMetadata>(
            1, window);
      });

  EXPECT_CALL(*jni_mock, FlutterViewDisplayOverlaySurface(1, 25, 25, 80, 150))
      .Times(2);

  auto surface_frame = std::make_unique<SurfaceFrame>(
      SkSurfaces::Null(1000, 1000), framebuffer_info,
      [](const SurfaceFrame& surface_frame, DlCanvas* canvas) mutable {
        return true;
      },
      [](const SurfaceFrame& surface_frame) { return true; },
      /*frame_size=*/SkISize::Make(800, 600));

  embedder->SubmitFlutterView(kImplicitViewId, gr_context.get(), nullptr,
                              std::move(surface_frame));

  EXPECT_CALL(*jni_mock, FlutterViewEndFrame());
  embedder->EndFrame(/*should_resubmit_frame=*/false, raster_thread_merger);
}

TEST(AndroidExternalViewEmbedder, SubmitFramePlatformViewWithoutAnyOverlay) {
  auto jni_mock = std::make_shared<JNIMock>();
  auto android_context =
      std::make_shared<AndroidContext>(AndroidRenderingAPI::kSoftware);

  auto window = fml::MakeRefCounted<AndroidNativeWindow>(nullptr);
  auto gr_context = GrDirectContext::MakeMock(nullptr);
  auto frame_size = SkISize::Make(1000, 1000);
  SurfaceFrame::FramebufferInfo framebuffer_info;
  auto surface_factory = std::make_shared<TestAndroidSurfaceFactory>(
      [gr_context, window, frame_size, framebuffer_info]() {
        auto surface_frame_1 = std::make_unique<SurfaceFrame>(
            SkSurfaces::Null(1000, 1000), framebuffer_info,
            [](const SurfaceFrame& surface_frame, DlCanvas* canvas) {
              return true;
            },
            [](const SurfaceFrame& surface_frame) { return true; },
            /*frame_size=*/SkISize::Make(800, 600));

        auto surface_mock = std::make_unique<SurfaceMock>();
        EXPECT_CALL(*surface_mock, AcquireFrame(frame_size))
            .Times(1 /* frames */)
            .WillOnce(Return(ByMove(std::move(surface_frame_1))));

        auto android_surface_mock = std::make_unique<AndroidSurfaceMock>();
        EXPECT_CALL(*android_surface_mock, IsValid()).WillOnce(Return(true));

        EXPECT_CALL(*android_surface_mock, CreateGPUSurface(gr_context.get()))
            .WillOnce(Return(ByMove(std::move(surface_mock))));

        EXPECT_CALL(*android_surface_mock, SetNativeWindow(window));
        return android_surface_mock;
      });
  auto embedder = std::make_unique<AndroidExternalViewEmbedder>(
      *android_context, jni_mock, surface_factory, GetTaskRunnersForFixture());

  auto raster_thread_merger = GetThreadMergerFromPlatformThread();

  EXPECT_CALL(*jni_mock, FlutterViewBeginFrame());
  embedder->BeginFrame(nullptr, raster_thread_merger);
  embedder->PrepareFlutterView(frame_size, 1.5);

  {
    // Add Android view.
    SkMatrix matrix;
    MutatorsStack stack;
    stack.PushTransform(SkMatrix::Translate(0, 0));

    embedder->PrerollCompositeEmbeddedView(
        0, std::make_unique<EmbeddedViewParams>(matrix, SkSize::Make(200, 200),
                                                stack));
    EXPECT_CALL(*jni_mock, FlutterViewOnDisplayPlatformView(0, 0, 0, 200, 200,
                                                            300, 300, stack));
  }

  EXPECT_CALL(*jni_mock, FlutterViewCreateOverlaySurface()).Times(0);

  auto surface_frame = std::make_unique<SurfaceFrame>(
      SkSurfaces::Null(1000, 1000), framebuffer_info,
      [](const SurfaceFrame& surface_frame, DlCanvas* canvas) mutable {
        return true;
      },
      [](const SurfaceFrame& surface_frame) { return true; },
      /*frame_size=*/SkISize::Make(800, 600));

  embedder->SubmitFlutterView(kImplicitViewId, gr_context.get(), nullptr,
                              std::move(surface_frame));

  EXPECT_CALL(*jni_mock, FlutterViewEndFrame());
  embedder->EndFrame(/*should_resubmit_frame=*/false, raster_thread_merger);
}

TEST(AndroidExternalViewEmbedder, DoesNotCallJNIPlatformThreadOnlyMethods) {
  auto jni_mock = std::make_shared<JNIMock>();

  auto android_context = AndroidContext(AndroidRenderingAPI::kSoftware);
  auto embedder = std::make_unique<AndroidExternalViewEmbedder>(
      android_context, jni_mock, nullptr, GetTaskRunnersForFixture());

  // While on the raster thread, don't make JNI calls as these methods can only
  // run on the platform thread.
  fml::Thread platform_thread("platform");
  auto raster_thread_merger = GetThreadMergerFromRasterThread(&platform_thread);

  EXPECT_CALL(*jni_mock, FlutterViewBeginFrame()).Times(0);
  embedder->BeginFrame(nullptr, raster_thread_merger);
  embedder->PrepareFlutterView(SkISize::Make(10, 20), 1.0);

  EXPECT_CALL(*jni_mock, FlutterViewEndFrame()).Times(0);
  embedder->EndFrame(/*should_resubmit_frame=*/false, raster_thread_merger);
}

TEST(AndroidExternalViewEmbedder, DestroyOverlayLayersOnSizeChange) {
  auto jni_mock = std::make_shared<JNIMock>();

  auto android_context =
      std::make_shared<AndroidContext>(AndroidRenderingAPI::kSoftware);
  auto window = fml::MakeRefCounted<AndroidNativeWindow>(nullptr);
  auto gr_context = GrDirectContext::MakeMock(nullptr);
  auto frame_size = SkISize::Make(1000, 1000);
  SurfaceFrame::FramebufferInfo framebuffer_info;
  auto surface_factory = std::make_shared<TestAndroidSurfaceFactory>(
      [gr_context, window, frame_size, framebuffer_info]() {
        auto surface_frame_1 = std::make_unique<SurfaceFrame>(
            SkSurfaces::Null(1000, 1000), framebuffer_info,
            [](const SurfaceFrame& surface_frame, DlCanvas* canvas) {
              return true;
            },
            [](const SurfaceFrame& surface_frame) { return true; },
            /*frame_size=*/SkISize::Make(800, 600));

        auto surface_mock = std::make_unique<SurfaceMock>();
        EXPECT_CALL(*surface_mock, AcquireFrame(frame_size))
            .WillOnce(Return(ByMove(std::move(surface_frame_1))));

        auto android_surface_mock = std::make_unique<AndroidSurfaceMock>();
        EXPECT_CALL(*android_surface_mock, IsValid()).WillOnce(Return(true));

        EXPECT_CALL(*android_surface_mock, CreateGPUSurface(gr_context.get()))
            .WillOnce(Return(ByMove(std::move(surface_mock))));

        EXPECT_CALL(*android_surface_mock, SetNativeWindow(window));

        return android_surface_mock;
      });

  auto embedder = std::make_unique<AndroidExternalViewEmbedder>(
      *android_context, jni_mock, surface_factory, GetTaskRunnersForFixture());
  fml::Thread rasterizer_thread("rasterizer");
  auto raster_thread_merger =
      GetThreadMergerFromPlatformThread(&rasterizer_thread);

  // ------------------ First frame ------------------ //
  {
    EXPECT_CALL(*jni_mock, FlutterViewBeginFrame());
    embedder->BeginFrame(nullptr, raster_thread_merger);
    embedder->PrepareFlutterView(frame_size, 1.5);

    // Add an Android view.
    MutatorsStack stack1;
    // TODO(egarciad): Investigate why Flow applies the device pixel ratio to
    // the offsetPixels, but not the sizePoints.
    auto view_params_1 = std::make_unique<EmbeddedViewParams>(
        SkMatrix(), SkSize::Make(200, 200), stack1);

    embedder->PrerollCompositeEmbeddedView(0, std::move(view_params_1));

    // This simulates Flutter UI that intersects with the Android view.
    embedder->CompositeEmbeddedView(0)->DrawRect(
        SkRect::MakeXYWH(50, 50, 200, 200), DlPaint());

    // Create a new overlay surface.
    EXPECT_CALL(*jni_mock, FlutterViewCreateOverlaySurface())
        .WillOnce(Return(
            ByMove(std::make_unique<PlatformViewAndroidJNI::OverlayMetadata>(
                0, window))));
    // The JNI call to display the Android view.
    EXPECT_CALL(*jni_mock, FlutterViewOnDisplayPlatformView(0, 0, 0, 200, 200,
                                                            300, 300, stack1));
    EXPECT_CALL(*jni_mock,
                FlutterViewDisplayOverlaySurface(0, 50, 50, 150, 150));

    SurfaceFrame::FramebufferInfo framebuffer_info;
    auto surface_frame = std::make_unique<SurfaceFrame>(
        SkSurfaces::Null(1000, 1000), framebuffer_info,
        [](const SurfaceFrame& surface_frame, DlCanvas* canvas) {
          return true;
        },
        [](const SurfaceFrame& surface_frame) { return true; },
        /*frame_size=*/SkISize::Make(800, 600));
    embedder->SubmitFlutterView(kImplicitViewId, gr_context.get(), nullptr,
                                std::move(surface_frame));

    EXPECT_CALL(*jni_mock, FlutterViewEndFrame());
    embedder->EndFrame(/*should_resubmit_frame=*/false, raster_thread_merger);
  }

  EXPECT_CALL(*jni_mock, FlutterViewDestroyOverlaySurfaces());
  EXPECT_CALL(*jni_mock, FlutterViewBeginFrame());
  // Change the frame size.
  embedder->BeginFrame(nullptr, raster_thread_merger);
  embedder->PrepareFlutterView(SkISize::Make(30, 40), 1.0);
}

TEST(AndroidExternalViewEmbedder, DoesNotDestroyOverlayLayersOnSizeChange) {
  auto jni_mock = std::make_shared<JNIMock>();
  auto android_context =
      std::make_shared<AndroidContext>(AndroidRenderingAPI::kSoftware);

  auto window = fml::MakeRefCounted<AndroidNativeWindow>(nullptr);
  auto gr_context = GrDirectContext::MakeMock(nullptr);
  auto frame_size = SkISize::Make(1000, 1000);
  SurfaceFrame::FramebufferInfo framebuffer_info;
  auto surface_factory = std::make_shared<TestAndroidSurfaceFactory>(
      [gr_context, window, frame_size, framebuffer_info]() {
        auto surface_frame_1 = std::make_unique<SurfaceFrame>(
            SkSurfaces::Null(1000, 1000), framebuffer_info,
            [](const SurfaceFrame& surface_frame, DlCanvas* canvas) {
              return true;
            },
            [](const SurfaceFrame& surface_frame) { return true; },
            /*frame_size=*/SkISize::Make(800, 600));

        auto surface_mock = std::make_unique<SurfaceMock>();
        EXPECT_CALL(*surface_mock, AcquireFrame(frame_size))
            .WillOnce(Return(ByMove(std::move(surface_frame_1))));

        auto android_surface_mock = std::make_unique<AndroidSurfaceMock>();
        EXPECT_CALL(*android_surface_mock, IsValid()).WillOnce(Return(true));

        EXPECT_CALL(*android_surface_mock, CreateGPUSurface(gr_context.get()))
            .WillOnce(Return(ByMove(std::move(surface_mock))));

        EXPECT_CALL(*android_surface_mock, SetNativeWindow(window));

        return android_surface_mock;
      });

  auto embedder = std::make_unique<AndroidExternalViewEmbedder>(
      *android_context, jni_mock, surface_factory, GetTaskRunnersForFixture());

  // ------------------ First frame ------------------ //
  {
    fml::Thread rasterizer_thread("rasterizer");
    auto raster_thread_merger =
        GetThreadMergerFromPlatformThread(&rasterizer_thread);
    EXPECT_CALL(*jni_mock, FlutterViewBeginFrame());
    embedder->BeginFrame(nullptr, raster_thread_merger);
    embedder->PrepareFlutterView(frame_size, 1.5);

    // Add an Android view.
    MutatorsStack stack1;
    // TODO(egarciad): Investigate why Flow applies the device pixel ratio to
    // the offsetPixels, but not the sizePoints.
    auto view_params_1 = std::make_unique<EmbeddedViewParams>(
        SkMatrix(), SkSize::Make(200, 200), stack1);

    embedder->PrerollCompositeEmbeddedView(0, std::move(view_params_1));

    // This simulates Flutter UI that intersects with the Android view.
    embedder->CompositeEmbeddedView(0)->DrawRect(
        SkRect::MakeXYWH(50, 50, 200, 200), DlPaint());

    // Create a new overlay surface.
    EXPECT_CALL(*jni_mock, FlutterViewCreateOverlaySurface())
        .WillOnce(Return(
            ByMove(std::make_unique<PlatformViewAndroidJNI::OverlayMetadata>(
                0, window))));
    // The JNI call to display the Android view.
    EXPECT_CALL(*jni_mock, FlutterViewOnDisplayPlatformView(0, 0, 0, 200, 200,
                                                            300, 300, stack1));
    EXPECT_CALL(*jni_mock,
                FlutterViewDisplayOverlaySurface(0, 50, 50, 150, 150));

    auto surface_frame = std::make_unique<SurfaceFrame>(
        SkSurfaces::Null(1000, 1000), framebuffer_info,
        [](const SurfaceFrame& surface_frame, DlCanvas* canvas) {
          return true;
        },
        [](const SurfaceFrame& surface_frame) { return true; },
        /*frame_size=*/SkISize::Make(800, 600));
    embedder->SubmitFlutterView(kImplicitViewId, gr_context.get(), nullptr,
                                std::move(surface_frame));

    EXPECT_CALL(*jni_mock, FlutterViewEndFrame());
    embedder->EndFrame(/*should_resubmit_frame=*/false, raster_thread_merger);
  }

  EXPECT_CALL(*jni_mock, FlutterViewDestroyOverlaySurfaces()).Times(1);
  EXPECT_CALL(*jni_mock, FlutterViewBeginFrame()).Times(0);

  fml::Thread platform_thread("platform");
  embedder->BeginFrame(nullptr,
                       GetThreadMergerFromRasterThread(&platform_thread));
  embedder->PrepareFlutterView(SkISize::Make(30, 40), 1.0);
}

TEST(AndroidExternalViewEmbedder, SupportsDynamicThreadMerging) {
  auto jni_mock = std::make_shared<JNIMock>();
  auto android_context = AndroidContext(AndroidRenderingAPI::kSoftware);
  auto embedder = std::make_unique<AndroidExternalViewEmbedder>(
      android_context, jni_mock, nullptr, GetTaskRunnersForFixture());
  ASSERT_TRUE(embedder->SupportsDynamicThreadMerging());
}

TEST(AndroidExternalViewEmbedder, DisableThreadMerger) {
  auto jni_mock = std::make_shared<JNIMock>();
  auto android_context = AndroidContext(AndroidRenderingAPI::kSoftware);
  auto embedder = std::make_unique<AndroidExternalViewEmbedder>(
      android_context, jni_mock, nullptr, GetTaskRunnersForFixture());

  fml::Thread platform_thread("platform");
  auto raster_thread_merger = GetThreadMergerFromRasterThread(&platform_thread);
  ASSERT_FALSE(raster_thread_merger->IsMerged());

  // The shell may disable the thread merger during `OnPlatformViewDestroyed`.
  raster_thread_merger->Disable();

  EXPECT_CALL(*jni_mock, FlutterViewBeginFrame()).Times(0);

  embedder->BeginFrame(nullptr, raster_thread_merger);
  embedder->PrepareFlutterView(SkISize::Make(10, 20), 1.0);
  // Push a platform view.
  embedder->PrerollCompositeEmbeddedView(
      0, std::make_unique<EmbeddedViewParams>());

  auto postpreroll_result = embedder->PostPrerollAction(raster_thread_merger);
  ASSERT_EQ(PostPrerollResult::kSkipAndRetryFrame, postpreroll_result);

  EXPECT_CALL(*jni_mock, FlutterViewEndFrame()).Times(0);
  embedder->EndFrame(/*should_resubmit_frame=*/true, raster_thread_merger);

  ASSERT_FALSE(raster_thread_merger->IsMerged());
}

TEST(AndroidExternalViewEmbedder, Teardown) {
  auto jni_mock = std::make_shared<JNIMock>();
  auto android_context =
      std::make_shared<AndroidContext>(AndroidRenderingAPI::kSoftware);
  auto window = fml::MakeRefCounted<AndroidNativeWindow>(nullptr);
  auto gr_context = GrDirectContext::MakeMock(nullptr);
  auto frame_size = SkISize::Make(1000, 1000);
  auto surface_factory = std::make_shared<TestAndroidSurfaceFactory>(
      [gr_context, window, frame_size]() {
        SurfaceFrame::FramebufferInfo framebuffer_info;
        auto surface_frame_1 = std::make_unique<SurfaceFrame>(
            SkSurfaces::Null(1000, 1000), framebuffer_info,
            [](const SurfaceFrame& surface_frame, DlCanvas* canvas) {
              return true;
            },
            [](const SurfaceFrame& surface_frame) { return true; },
            /*frame_size=*/SkISize::Make(800, 600));

        auto surface_mock = std::make_unique<SurfaceMock>();
        EXPECT_CALL(*surface_mock, AcquireFrame(frame_size))
            .WillOnce(Return(ByMove(std::move(surface_frame_1))));

        auto android_surface_mock = std::make_unique<AndroidSurfaceMock>();
        EXPECT_CALL(*android_surface_mock, IsValid()).WillOnce(Return(true));
        EXPECT_CALL(*android_surface_mock, CreateGPUSurface(gr_context.get()))
            .WillOnce(Return(ByMove(std::move(surface_mock))));

        return android_surface_mock;
      });

  auto embedder = std::make_unique<AndroidExternalViewEmbedder>(
      *android_context, jni_mock, surface_factory, GetTaskRunnersForFixture());
  fml::Thread rasterizer_thread("rasterizer");
  auto raster_thread_merger =
      GetThreadMergerFromPlatformThread(&rasterizer_thread);

  embedder->BeginFrame(nullptr, raster_thread_merger);
  embedder->PrepareFlutterView(frame_size, 1.5);

  // Add an Android view.
  MutatorsStack stack;
  auto view_params = std::make_unique<EmbeddedViewParams>(
      SkMatrix(), SkSize::Make(200, 200), stack);

  embedder->PrerollCompositeEmbeddedView(0, std::move(view_params));

  // This simulates Flutter UI that intersects with the Android view.
  embedder->CompositeEmbeddedView(0)->DrawRect(
      SkRect::MakeXYWH(50, 50, 200, 200), DlPaint());

  // Create a new overlay surface.
  EXPECT_CALL(*jni_mock, FlutterViewCreateOverlaySurface())
      .WillOnce(Return(
          ByMove(std::make_unique<PlatformViewAndroidJNI::OverlayMetadata>(
              0, window))));

  SurfaceFrame::FramebufferInfo framebuffer_info;
  auto surface_frame = std::make_unique<SurfaceFrame>(
      SkSurfaces::Null(1000, 1000), framebuffer_info,
      [](const SurfaceFrame& surface_frame, DlCanvas* canvas) { return true; },
      [](const SurfaceFrame& surface_frame) { return true; },
      /*frame_size=*/SkISize::Make(800, 600));
  embedder->SubmitFlutterView(kImplicitViewId, gr_context.get(), nullptr,
                              std::move(surface_frame));

  embedder->EndFrame(/*should_resubmit_frame=*/false, raster_thread_merger);

  EXPECT_CALL(*jni_mock, FlutterViewDestroyOverlaySurfaces());
  // Teardown.
  embedder->Teardown();
}

TEST(AndroidExternalViewEmbedder, TeardownDoesNotCallJNIMethod) {
  auto jni_mock = std::make_shared<JNIMock>();
  auto android_context =
      std::make_shared<AndroidContext>(AndroidRenderingAPI::kSoftware);
  auto embedder = std::make_unique<AndroidExternalViewEmbedder>(
      *android_context, jni_mock, nullptr, GetTaskRunnersForFixture());

  EXPECT_CALL(*jni_mock, FlutterViewDestroyOverlaySurfaces()).Times(0);
  embedder->Teardown();
}

}  // namespace testing
}  // namespace flutter
