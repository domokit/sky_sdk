// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/layers/performance_overlay_layer.h"

#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include "flow/stopwatch.h"
#include "flow/stopwatch_dl.h"
#include "flow/stopwatch_sk.h"
#include "third_party/skia/include/core/SkFont.h"
#include "third_party/skia/include/core/SkFontMgr.h"
#include "third_party/skia/include/core/SkTextBlob.h"
#include "third_party/skia/include/core/SkTypeface.h"
#include "txt/platform.h"
#ifdef IMPELLER_SUPPORTS_RENDERING
#include "impeller/typographer/backends/skia/text_frame_skia.h"  // nogncheck
#endif  // IMPELLER_SUPPORTS_RENDERING

namespace flutter {
namespace {

void VisualizeStopWatch(DlCanvas* canvas,
                        const bool impeller_enabled,
                        const Stopwatch& stopwatch,
                        SkScalar x,
                        SkScalar y,
                        SkScalar width,
                        SkScalar height,
                        bool show_graph,
                        bool show_labels,
                        const std::string& label_prefix,
                        std::vector<DlPoint>& point_storage,
                        std::vector<DlColor>& color_storage,
                        const SkFont& font) {
  const int label_x = 8;    // distance from x
  const int label_y = -10;  // distance from y+height

  if (show_graph) {
    DlRect visualization_rect = DlRect::MakeXYWH(x, y, width, height);
    if (impeller_enabled) {
      DlStopwatchVisualizer(stopwatch, point_storage, color_storage)
          .Visualize(canvas, visualization_rect);
    } else {
      SkStopwatchVisualizer(stopwatch).Visualize(canvas, visualization_rect);
    }
  }

  if (show_labels) {
    auto text = PerformanceOverlayLayer::MakeStatisticsText(stopwatch, font,
                                                            label_prefix);
    // Historically SK_ColorGRAY (== 0xFF888888) was used here
    DlPaint paint(DlColor(0xFF888888));
#ifdef IMPELLER_SUPPORTS_RENDERING
    if (impeller_enabled) {
      canvas->DrawTextFrame(impeller::MakeTextFrameFromTextBlobSkia(text),
                            x + label_x, y + height + label_y, paint);
      return;
    }
#endif  // IMPELLER_SUPPORTS_RENDERING
    canvas->DrawTextBlob(text, x + label_x, y + height + label_y, paint);
  }
}

}  // namespace

// static
SkFont PerformanceOverlayLayer::MakeStatisticsFont(std::string_view font_path) {
  sk_sp<SkFontMgr> font_mgr = txt::GetDefaultFontManager();
  if (font_path == "") {
    if (sk_sp<SkTypeface> face = font_mgr->matchFamilyStyle(nullptr, {})) {
      return SkFont(face, 15);
    } else {
      // In Skia's Android fontmgr, matchFamilyStyle can return null instead
      // of falling back to a default typeface. If that's the case, we can use
      // legacyMakeTypeface, which *does* use that default typeface.
      return SkFont(font_mgr->legacyMakeTypeface(nullptr, {}), 15);
    }
  } else {
    return SkFont(font_mgr->makeFromFile(font_path.data()), 15);
  }
}

// static
sk_sp<SkTextBlob> PerformanceOverlayLayer::MakeStatisticsText(
    const Stopwatch& stopwatch,
    const SkFont& font,
    std::string_view label_prefix) {
  // Make sure there's not an empty typeface returned, or we won't see any text.
  FML_DCHECK(font.getTypeface()->countGlyphs() > 0);

  double max_ms_per_frame = stopwatch.MaxDelta().ToMillisecondsF();
  double average_ms_per_frame = stopwatch.AverageDelta().ToMillisecondsF();
  std::stringstream stream;
  stream.setf(std::ios::fixed | std::ios::showpoint);
  stream << std::setprecision(1);
  stream << label_prefix << "  " << "max " << max_ms_per_frame << " ms/frame, "
         << "avg " << average_ms_per_frame << " ms/frame";
  auto text = stream.str();
  return SkTextBlob::MakeFromText(text.c_str(), text.size(), font,
                                  SkTextEncoding::kUTF8);
}

PerformanceOverlayLayer::PerformanceOverlayLayer(uint64_t options,
                                                 const char* font_path)
    : options_(options) {
  if (font_path != nullptr) {
    font_path_ = font_path;
  }
}

void PerformanceOverlayLayer::Diff(DiffContext* context,
                                   const Layer* old_layer) {
  DiffContext::AutoSubtreeRestore subtree(context);
  if (!context->IsSubtreeDirty()) {
    FML_DCHECK(old_layer);
    auto prev = old_layer->as_performance_overlay_layer();
    context->MarkSubtreeDirty(context->GetOldLayerPaintRegion(prev));
  }
  context->AddLayerBounds(paint_bounds());
  context->SetLayerPaintRegion(this, context->CurrentSubtreeRegion());
}

void PerformanceOverlayLayer::Paint(PaintContext& context) const {
  const int padding = 8;

  if (!options_) {
    return;
  }

  DlScalar x = paint_bounds().GetX() + padding;
  DlScalar y = paint_bounds().GetY() + padding;
  DlScalar width = paint_bounds().GetWidth() - (padding * 2);
  DlScalar height = paint_bounds().GetHeight() / 2;
  auto mutator = context.state_stack.save();
  // Cached storage for vertex output.
  std::vector<DlPoint> vertices_storage;
  std::vector<DlColor> color_storage;
  SkFont font = MakeStatisticsFont(font_path_);

  VisualizeStopWatch(context.canvas, context.impeller_enabled,
                     context.raster_time, x, y, width, height - padding,
                     options_ & kVisualizeRasterizerStatistics,
                     options_ & kDisplayRasterizerStatistics, "Raster",
                     vertices_storage, color_storage, font);

  VisualizeStopWatch(context.canvas, context.impeller_enabled, context.ui_time,
                     x, y + height, width, height - padding,
                     options_ & kVisualizeEngineStatistics,
                     options_ & kDisplayEngineStatistics, "UI",
                     vertices_storage, color_storage, font);
}

}  // namespace flutter
