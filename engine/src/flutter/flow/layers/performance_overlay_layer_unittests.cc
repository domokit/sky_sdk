// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/layers/performance_overlay_layer.h"

#include <cstdint>
#include <sstream>

#include "flutter/display_list/utils/dl_receiver_utils.h"
#include "flutter/flow/flow_test_utils.h"
#include "flutter/flow/raster_cache.h"
#include "flutter/flow/testing/layer_test.h"
#include "flutter/shell/common/base64.h"
#include "third_party/skia/include/core/SkData.h"
#include "third_party/skia/include/core/SkImage.h"
#include "third_party/skia/include/core/SkSerialProcs.h"
#include "third_party/skia/include/core/SkStream.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/core/SkTextBlob.h"
#include "third_party/skia/include/encode/SkPngEncoder.h"

namespace flutter {
namespace testing {
namespace {

// To get the size of kMockedTimes in compile time.
template <class T, std::size_t N>
constexpr int size(const T (&array)[N]) noexcept {
  return N;
}

constexpr int kMockedTimes[] = {17, 1,  4,  24, 4,  25, 30, 4,  13, 34,
                                14, 0,  18, 9,  32, 36, 26, 23, 5,  8,
                                32, 18, 29, 16, 29, 18, 0,  36, 33, 10};

static std::string GetGoldenFilePath(int refresh_rate, bool is_new) {
  std::stringstream ss;
  // This unit test should only be run on Linux (not even on Mac since it's a
  // golden test). Hence we don't have to worry about the "/" vs. "\".
  ss << flutter::GetGoldenDir() << "/" << "performance_overlay_gold_"
     << refresh_rate << "fps" << (is_new ? "_new" : "") << ".png";
  return ss.str();
}

static void TestPerformanceOverlayLayerGold(int refresh_rate) {
  std::string golden_file_path = GetGoldenFilePath(refresh_rate, false);
  std::string new_golden_file_path = GetGoldenFilePath(refresh_rate, true);

  FixedRefreshRateStopwatch mock_stopwatch(
      fml::RefreshRateToFrameBudget(refresh_rate));
  for (int i = 0; i < size(kMockedTimes); ++i) {
    mock_stopwatch.SetLapTime(
        fml::TimeDelta::FromMilliseconds(kMockedTimes[i]));
  }

  const SkImageInfo image_info = SkImageInfo::MakeN32Premul(1000, 1000);
  sk_sp<SkSurface> surface = SkSurfaces::Raster(image_info);
  DlSkCanvasAdapter canvas(surface->getCanvas());

  ASSERT_TRUE(surface != nullptr);

  LayerStateStack state_stack;
  state_stack.set_delegate(&canvas);

  flutter::PaintContext paint_context = {
      // clang-format off
      .state_stack                   = state_stack,
      .canvas                        = &canvas,
      .gr_context                    = nullptr,
      .view_embedder                 = nullptr,
      .raster_time                   = mock_stopwatch,
      .ui_time                       = mock_stopwatch,
      .texture_registry              = nullptr,
      .raster_cache                  = nullptr,
      // clang-format on
  };

  // Specify font file to ensure the same font across different operation
  // systems.
  flutter::PerformanceOverlayLayer layer(
      flutter::kDisplayRasterizerStatistics |
          flutter::kVisualizeRasterizerStatistics |
          flutter::kDisplayEngineStatistics |
          flutter::kVisualizeEngineStatistics,
      flutter::GetFontFile().c_str());
  layer.set_paint_bounds(DlRect::MakeWH(1000, 400));
  surface->getCanvas()->clear(SK_ColorTRANSPARENT);
  layer.Paint(paint_context);

  sk_sp<SkImage> snapshot = surface->makeImageSnapshot();
  sk_sp<SkData> snapshot_data =
      SkPngEncoder::Encode(nullptr, snapshot.get(), {});

  sk_sp<SkData> golden_data =
      SkData::MakeFromFileName(golden_file_path.c_str());
  EXPECT_TRUE(golden_data != nullptr)
      << "Golden file not found: " << golden_file_path << ".\n"
      << "Please either set --golden-dir, or make sure that the unit test is "
      << "run from the right directory (e.g., flutter/engine/src).";

  // TODO(https://github.com/flutter/flutter/issues/53784): enable this on all
  // platforms.
#if !defined(FML_OS_LINUX)
  GTEST_SKIP() << "Skipping golden tests on non-Linux OSes";
#else
  const bool golden_data_matches = golden_data->equals(snapshot_data.get());
  if (!golden_data_matches) {
    SkFILEWStream wstream(new_golden_file_path.c_str());
    wstream.write(snapshot_data->data(), snapshot_data->size());
    wstream.flush();

    size_t b64_size = Base64::EncodedSize(snapshot_data->size());
    sk_sp<SkData> b64_data = SkData::MakeUninitialized(b64_size + 1);
    char* b64_char = static_cast<char*>(b64_data->writable_data());
    Base64::Encode(snapshot_data->data(), snapshot_data->size(), b64_char);
    b64_char[b64_size] = 0;  // make it null terminated for printing

    EXPECT_TRUE(golden_data_matches)
        << "Golden file mismatch. Please check " << "the difference between "
        << golden_file_path << " and " << new_golden_file_path
        << ", and  replace the former "
        << "with the latter if the difference looks good.\nS\n"
        << "See also the base64 encoded " << new_golden_file_path << ":\n"
        << b64_char;
  }
#endif  // FML_OS_LINUX
}

}  // namespace

using PerformanceOverlayLayerTest = LayerTest;

class ImageSizeTextBlobInspector : public virtual DlOpReceiver,
                                   virtual IgnoreAttributeDispatchHelper,
                                   virtual IgnoreTransformDispatchHelper,
                                   virtual IgnoreClipDispatchHelper,
                                   virtual IgnoreDrawDispatchHelper {
 public:
  void drawImage(const sk_sp<DlImage> image,
                 const DlPoint& point,
                 DlImageSampling sampling,
                 bool render_with_attributes) override {
    sizes_.push_back(image->GetBounds().GetSize());
  }

  void drawTextBlob(const sk_sp<SkTextBlob> blob,
                    DlScalar x,
                    DlScalar y) override {
    text_blobs_.push_back(blob);
    text_positions_.push_back(DlPoint(x, y));
  }

  const std::vector<DlISize>& sizes() { return sizes_; }
  const std::vector<sk_sp<SkTextBlob>> text_blobs() { return text_blobs_; }
  const std::vector<DlPoint> text_positions() { return text_positions_; }

 private:
  std::vector<DlISize> sizes_;
  std::vector<sk_sp<SkTextBlob>> text_blobs_;
  std::vector<DlPoint> text_positions_;
};

TEST_F(PerformanceOverlayLayerTest, PaintingEmptyLayerDies) {
  const uint64_t overlay_opts = kVisualizeRasterizerStatistics;
  auto layer = std::make_shared<PerformanceOverlayLayer>(overlay_opts);

  layer->Preroll(preroll_context());
  EXPECT_EQ(layer->paint_bounds(), DlRect());
  EXPECT_FALSE(layer->needs_painting(paint_context()));

  // Crashes reading a nullptr.
  EXPECT_DEATH_IF_SUPPORTED(layer->Paint(paint_context()), "");
}

TEST_F(PerformanceOverlayLayerTest, InvalidOptions) {
  const DlRect layer_bounds = DlRect::MakeLTRB(0.0f, 0.0f, 64.0f, 64.0f);
  const uint64_t overlay_opts = 0;
  auto layer = std::make_shared<PerformanceOverlayLayer>(overlay_opts);

  // TODO(): Note calling code has to call set_paint_bounds right now.  Make
  // this a constructor parameter and move the set_paint_bounds into Preroll
  layer->set_paint_bounds(layer_bounds);

  layer->Preroll(preroll_context());
  EXPECT_EQ(layer->paint_bounds(), layer_bounds);
  EXPECT_TRUE(layer->needs_painting(paint_context()));

  // Nothing is drawn if options are invalid (0).
  layer->Paint(display_list_paint_context());

  DisplayListBuilder expected_builder;
  auto expected_dl = expected_builder.Build();

  EXPECT_TRUE(DisplayListsEQ_Verbose(display_list(), expected_dl));
}

TEST_F(PerformanceOverlayLayerTest, SimpleRasterizerStatistics) {
  const DlRect layer_bounds = DlRect::MakeLTRB(0.0f, 0.0f, 64.0f, 64.0f);
  const uint64_t overlay_opts = kDisplayRasterizerStatistics;
  auto layer = std::make_shared<PerformanceOverlayLayer>(overlay_opts);
  auto font = PerformanceOverlayLayer::MakeStatisticsFont("");

  // TODO(): Note calling code has to call set_paint_bounds right now.  Make
  // this a constructor parameter and move the set_paint_bounds into Preroll
  layer->set_paint_bounds(layer_bounds);

  layer->Preroll(preroll_context());
  EXPECT_EQ(layer->paint_bounds(), layer_bounds);
  EXPECT_TRUE(layer->needs_painting(paint_context()));

  layer->Paint(display_list_paint_context());
  auto overlay_text = PerformanceOverlayLayer::MakeStatisticsText(
      display_list_paint_context().raster_time, font, "Raster");
  auto overlay_text_data = overlay_text->serialize(SkSerialProcs{});
  // Historically SK_ColorGRAY (== 0xFF888888) was used here
  DlPaint text_paint(DlColor(0xFF888888));
  DlPoint text_position = DlPoint(16.0f, 22.0f);
  ImageSizeTextBlobInspector inspector;
  display_list()->Dispatch(inspector);

  // TODO(https://github.com/flutter/flutter/issues/82202): Remove once the
  // performance overlay can use Fuchsia's font manager instead of the empty
  // default.
#if defined(OS_FUCHSIA)
  GTEST_SKIP() << "Expectation requires a valid default font manager";
#endif  // OS_FUCHSIA
  ASSERT_EQ(inspector.sizes().size(), 0u);
  ASSERT_EQ(inspector.text_blobs().size(), 1u);
  ASSERT_EQ(inspector.text_positions().size(), 1u);
  auto text_data = inspector.text_blobs().front()->serialize(SkSerialProcs{});
  EXPECT_TRUE(text_data->equals(overlay_text_data.get()));
  EXPECT_EQ(inspector.text_positions().front(), text_position);
}

TEST_F(PerformanceOverlayLayerTest, MarkAsDirtyWhenResized) {
  // Regression test for https://github.com/flutter/flutter/issues/54188

  // Create a PerformanceOverlayLayer.
  const uint64_t overlay_opts = kVisualizeRasterizerStatistics;
  auto layer = std::make_shared<PerformanceOverlayLayer>(overlay_opts);
  layer->set_paint_bounds(DlRect::MakeLTRB(0.0f, 0.0f, 48.0f, 48.0f));
  layer->Preroll(preroll_context());
  layer->Paint(display_list_paint_context());
  DlISize first_draw_size;
  {
    ImageSizeTextBlobInspector inspector;
    display_list()->Dispatch(inspector);
    ASSERT_EQ(inspector.sizes().size(), 1u);
    ASSERT_EQ(inspector.text_blobs().size(), 0u);
    ASSERT_EQ(inspector.text_positions().size(), 0u);
    first_draw_size = inspector.sizes().front();
  }

  // Create a second PerformanceOverlayLayer with different bounds.
  layer = std::make_shared<PerformanceOverlayLayer>(overlay_opts);
  layer->set_paint_bounds(DlRect::MakeLTRB(0.0f, 0.0f, 64.0f, 64.0f));
  layer->Preroll(preroll_context());
  reset_display_list();
  layer->Paint(display_list_paint_context());
  {
    ImageSizeTextBlobInspector inspector;
    display_list()->Dispatch(inspector);
    ASSERT_EQ(inspector.sizes().size(), 1u);
    ASSERT_EQ(inspector.text_blobs().size(), 0u);
    ASSERT_EQ(inspector.text_positions().size(), 0u);
    EXPECT_NE(first_draw_size, inspector.sizes().front());
  }
}

TEST(PerformanceOverlayLayerDefault, Gold) {
  TestPerformanceOverlayLayerGold(60);
}

TEST(PerformanceOverlayLayer90fps, Gold) {
  TestPerformanceOverlayLayerGold(90);
}

TEST(PerformanceOverlayLayer120fps, Gold) {
  TestPerformanceOverlayLayerGold(120);
}

}  // namespace testing
}  // namespace flutter
