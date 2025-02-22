// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/benchmarking/dl_complexity_metal.h"

// The numbers and weightings used in this file stem from taking the
// data from the DisplayListBenchmarks suite run on an iPhone 12 and
// applying very rough analysis on them to identify the approximate
// trends.
//
// See the comments in display_list_complexity_helper.h for details on the
// process and rationale behind coming up with these numbers.

namespace flutter {

DisplayListMetalComplexityCalculator*
    DisplayListMetalComplexityCalculator::instance_ = nullptr;

DisplayListMetalComplexityCalculator*
DisplayListMetalComplexityCalculator::GetInstance() {
  if (instance_ == nullptr) {
    instance_ = new DisplayListMetalComplexityCalculator();
  }
  return instance_;
}

unsigned int
DisplayListMetalComplexityCalculator::MetalHelper::BatchedComplexity() {
  // Calculate the impact of saveLayer.
  unsigned int save_layer_complexity;
  if (save_layer_count_ == 0) {
    save_layer_complexity = 0;
  } else {
    // saveLayer seems to have two trends; if the count is < 200,
    // then the individual cost of a saveLayer is higher than if
    // the count is > 200.
    //
    // However, the trend is strange and we should gather more data to
    // get a better idea of how to represent the trend. That being said, it's
    // very unlikely we'll ever hit a DisplayList with 200+ saveLayer calls
    // in it, so we will calculate based on the more reasonably anticipated
    // range of less than 200, with the trend line more weighted towards the
    // lower end of that range (as the data itself doesn't present as a straight
    // line). Further, we will easily hit our cache thresholds with such a
    // large number of saveLayer calls.
    //
    // m = 1/2
    // c = 1
    save_layer_complexity = (save_layer_count_ + 2) * 100000;
  }

  unsigned int draw_text_blob_complexity;
  if (draw_text_blob_count_ == 0) {
    draw_text_blob_complexity = 0;
  } else {
    // m = 1/240
    // c = 0.75
    draw_text_blob_complexity = (draw_text_blob_count_ + 180) * 2500 / 3;
  }

  return save_layer_complexity + draw_text_blob_complexity;
}

void DisplayListMetalComplexityCalculator::MetalHelper::saveLayer(
    const DlRect& bounds,
    const SaveLayerOptions options,
    const DlImageFilter* backdrop,
    std::optional<int64_t> backdrop_id) {
  if (IsComplex()) {
    return;
  }
  if (backdrop) {
    // Flutter does not offer this operation so this value can only ever be
    // non-null for a frame-wide builder which is not currently evaluated for
    // complexity.
    AccumulateComplexity(Ceiling());
  }
  save_layer_count_++;
}

void DisplayListMetalComplexityCalculator::MetalHelper::drawLine(
    const DlPoint& p0,
    const DlPoint& p1) {
  if (IsComplex()) {
    return;
  }
  // The curve here may be log-linear, although it doesn't really match up that
  // well. To avoid costly computations, try and do a best fit of the data onto
  // a linear graph as a very rough first order approximation.

  float non_hairline_penalty = 1.0f;
  float aa_penalty = 1.0f;

  if (!IsHairline()) {
    non_hairline_penalty = 1.15f;
  }
  if (IsAntiAliased()) {
    aa_penalty = 1.4f;
  }

  // Use an approximation for the distance to avoid floating point or
  // sqrt() calls.
  DlScalar distance = abs(p0.x - p1.x) + abs(p0.y - p1.y);

  // The baseline complexity is for a hairline stroke with no AA.
  // m = 1/45
  // c = 5
  unsigned int complexity =
      ((distance + 225) * 4 / 9) * non_hairline_penalty * aa_penalty;

  AccumulateComplexity(complexity);
}

void DisplayListMetalComplexityCalculator::MetalHelper::drawDashedLine(
    const DlPoint& p0,
    const DlPoint& p1,
    DlScalar on_length,
    DlScalar off_length) {
  // Dashing is slightly more complex than a regular drawLine, but this
  // op is so rare it is not worth measuring the difference.
  drawLine(p0, p1);
}

void DisplayListMetalComplexityCalculator::MetalHelper::drawRect(
    const DlRect& rect) {
  if (IsComplex()) {
    return;
  }

  unsigned int complexity;

  // If stroked, cost scales linearly with the rectangle width/height.
  // If filled, it scales with the area.
  //
  // Hairline stroke vs non hairline has no real penalty at smaller lengths,
  // but it increases at larger lengths. There isn't enough data to get a good
  // idea of the penalty at lengths > 1000px.
  //
  // There is also a kStrokeAndFill_Style that Skia exposes, but we do not
  // currently use it anywhere in Flutter.
  if (DrawStyle() == DlDrawStyle::kFill) {
    // No real difference for AA with filled styles.
    unsigned int area = rect.GetWidth() * rect.GetHeight();

    // m = 1/9000
    // c = 0
    complexity = area / 225;
  } else {
    // Take the average of the width and height.
    unsigned int length = (rect.GetWidth() + rect.GetHeight()) / 2;

    // There is a penalty for AA being *disabled*.
    if (IsAntiAliased()) {
      // m = 1/65
      // c = 0
      complexity = length * 8 / 13;
    } else {
      // m = 1/35
      // c = 0
      complexity = length * 8 / 7;
    }
  }

  AccumulateComplexity(complexity);
}

void DisplayListMetalComplexityCalculator::MetalHelper::drawOval(
    const DlRect& bounds) {
  if (IsComplex()) {
    return;
  }
  // DrawOval scales very roughly linearly with the bounding box width/height
  // (not area) for stroked styles without AA.
  //
  // Filled styles and stroked styles with AA scale linearly with the bounding
  // box area.
  unsigned int area = bounds.GetWidth() * bounds.GetHeight();

  unsigned int complexity;

  // There is also a kStrokeAndFill_Style that Skia exposes, but we do not
  // currently use it anywhere in Flutter.
  if (DrawStyle() == DlDrawStyle::kFill) {
    // With filled styles, there is no significant AA penalty.
    // m = 1/16000
    // c = 0
    complexity = area / 80;
  } else {
    if (IsAntiAliased()) {
      // m = 1/7500
      // c = 0
      complexity = area * 2 / 75;
    } else {
      // Take the average of the width and height.
      unsigned int length = (bounds.GetWidth() + bounds.GetHeight()) / 2;

      // m = 1/80
      // c = 0
      complexity = length * 5 / 2;
    }
  }

  AccumulateComplexity(complexity);
}

void DisplayListMetalComplexityCalculator::MetalHelper::drawCircle(
    const DlPoint& center,
    DlScalar radius) {
  if (IsComplex()) {
    return;
  }

  unsigned int complexity;

  // There is also a kStrokeAndFill_Style that Skia exposes, but we do not
  // currently use it anywhere in Flutter.
  if (DrawStyle() == DlDrawStyle::kFill) {
    // We can ignore pi here.
    unsigned int area = radius * radius;
    // m = 1/1300
    // c = 5
    complexity = (area + 6500) * 2 / 65;

    // Penalty of around 5% when AA is disabled.
    if (!IsAntiAliased()) {
      complexity *= 1.05f;
    }
  } else {
    // Hairline vs non-hairline has no significant performance difference.
    if (IsAntiAliased()) {
      // m = 1/7
      // c = 7
      complexity = (radius + 49) * 40 / 7;
    } else {
      // m = 1/16
      // c = 8
      complexity = (radius + 128) * 5 / 2;
    }
  }

  AccumulateComplexity(complexity);
}

void DisplayListMetalComplexityCalculator::MetalHelper::drawRoundRect(
    const DlRoundRect& rrect) {
  if (IsComplex()) {
    return;
  }
  // RRects scale linearly with the area of the bounding rect.
  unsigned int area = rrect.GetBounds().Area();

  // Drawing RRects is split into two performance tiers; an expensive
  // one and a less expensive one. Both scale linearly with area.
  //
  // Expensive: All filled style, symmetric w/AA.
  bool expensive = (DrawStyle() == DlDrawStyle::kFill) ||
                   (rrect.GetRadii().AreAllCornersSame() && IsAntiAliased());

  unsigned int complexity;

  // These values were worked out by creating a straight line graph (y=mx+c)
  // approximately matching the measured data, normalising the data so that
  // 0.0005ms resulted in a score of 100 then simplifying down the formula.
  if (expensive) {
    // m = 1/25000
    // c = 2
    // An area of 7000px^2 ~= baseline timing of 0.0005ms.
    complexity = (area + 10500) / 175;
  } else {
    // m = 1/7000
    // c = 1.5
    // An area of 16000px^2 ~= baseline timing of 0.0005ms.
    complexity = (area + 50000) / 625;
  }

  AccumulateComplexity(complexity);
}

void DisplayListMetalComplexityCalculator::MetalHelper::drawDiffRoundRect(
    const DlRoundRect& outer,
    const DlRoundRect& inner) {
  if (IsComplex()) {
    return;
  }
  // There are roughly four classes here:
  // a) Filled style with AA enabled.
  // b) Filled style with AA disabled.
  // c) Complex RRect type with AA enabled and filled style.
  // d) Everything else.
  //
  // a) and c) scale linearly with the area, b) and d) scale linearly with
  // a single dimension (length). In all cases, the dimensions refer to
  // the outer RRect.
  unsigned int length =
      (outer.GetBounds().GetWidth() + outer.GetBounds().GetHeight()) / 2;

  unsigned int complexity;

  // These values were worked out by creating a straight line graph (y=mx+c)
  // approximately matching the measured data, normalising the data so that
  // 0.0005ms resulted in a score of 100 then simplifying down the formula.
  //
  // There is also a kStrokeAndFill_Style that Skia exposes, but we do not
  // currently use it anywhere in Flutter.
  if (DrawStyle() == DlDrawStyle::kFill) {
    unsigned int area = outer.GetBounds().Area();
    if (!outer.GetRadii().AreAllCornersSame()) {
      // m = 1/1000
      // c = 1
      complexity = (area + 1000) / 10;
    } else {
      if (IsAntiAliased()) {
        // m = 1/3500
        // c = 1.5
        complexity = (area + 5250) / 35;
      } else {
        // m = 1/30
        // c = 1
        complexity = (300 + (10 * length)) / 3;
      }
    }
  } else {
    // m = 1/60
    // c = 1.75
    complexity = ((10 * length) + 1050) / 6;
  }

  AccumulateComplexity(complexity);
}

void DisplayListMetalComplexityCalculator::MetalHelper::drawRoundSuperellipse(
    const DlRoundSuperellipse& rse) {
  // Drawing RSEs on Skia falls back to RRect.
  drawRoundRect(rse.ToApproximateRoundRect());
}

void DisplayListMetalComplexityCalculator::MetalHelper::drawPath(
    const DlPath& path) {
  if (IsComplex()) {
    return;
  }
  // There is negligible effect on the performance for hairline vs. non-hairline
  // stroke widths.
  //
  // The data for filled styles is currently suspicious, so for now we are going
  // to assign scores based on stroked styles.

  unsigned int line_verb_cost, quad_verb_cost, conic_verb_cost, cubic_verb_cost;

  if (IsAntiAliased()) {
    line_verb_cost = 75;
    quad_verb_cost = 100;
    conic_verb_cost = 160;
    cubic_verb_cost = 210;
  } else {
    line_verb_cost = 67;
    quad_verb_cost = 80;
    conic_verb_cost = 140;
    cubic_verb_cost = 210;
  }

  // There seems to be a fixed cost of around 1ms for calling drawPath.
  unsigned int complexity =
      200000 + CalculatePathComplexity(path, line_verb_cost, quad_verb_cost,
                                       conic_verb_cost, cubic_verb_cost);

  AccumulateComplexity(complexity);
}

void DisplayListMetalComplexityCalculator::MetalHelper::drawArc(
    const DlRect& oval_bounds,
    DlScalar start_degrees,
    DlScalar sweep_degrees,
    bool use_center) {
  if (IsComplex()) {
    return;
  }
  // Hairline vs non-hairline makes no difference to the performance.
  // Stroked styles without AA scale linearly with the diameter.
  // Stroked styles with AA scale linearly with the area except for small
  // values. Filled styles scale linearly with the area.
  unsigned int diameter =
      (oval_bounds.GetWidth() + oval_bounds.GetHeight()) / 2;
  unsigned int area = oval_bounds.GetWidth() * oval_bounds.GetHeight();

  unsigned int complexity;

  // These values were worked out by creating a straight line graph (y=mx+c)
  // approximately matching the measured data, normalising the data so that
  // 0.0005ms resulted in a score of 100 then simplifying down the formula.
  //
  // There is also a kStrokeAndFill_Style that Skia exposes, but we do not
  // currently use it anywhere in Flutter.
  if (DrawStyle() == DlDrawStyle::kStroke) {
    if (IsAntiAliased()) {
      // m = 1/8500
      // c = 16
      complexity = (area + 136000) * 2 / 765;
    } else {
      // m = 1/60
      // c = 3
      complexity = (diameter + 180) * 10 / 27;
    }
  } else {
    if (IsAntiAliased()) {
      // m = 1/20000
      // c = 20
      complexity = (area + 400000) / 900;
    } else {
      // m = 1/2100
      // c = 8
      complexity = (area + 16800) * 2 / 189;
    }
  }

  AccumulateComplexity(complexity);
}

void DisplayListMetalComplexityCalculator::MetalHelper::drawPoints(
    DlPointMode mode,
    uint32_t count,
    const DlPoint points[]) {
  if (IsComplex()) {
    return;
  }
  unsigned int complexity;

  // If AA is off then they all behave similarly, and scale
  // linearly with the point count.
  if (!IsAntiAliased()) {
    // m = 1/16000
    // c = 0.75
    complexity = (count + 12000) * 25 / 2;
  } else {
    if (mode == DlPointMode::kPolygon) {
      // m = 1/1250
      // c = 1
      complexity = (count + 1250) * 160;
    } else {
      if (IsHairline() && mode == DlPointMode::kPoints) {
        // This is a special case, it triggers an extremely fast path.
        // m = 1/14500
        // c = 0
        complexity = count * 400 / 29;
      } else {
        // m = 1/2200
        // c = 0.75
        complexity = (count + 1650) * 1000 / 11;
      }
    }
  }
  AccumulateComplexity(complexity);
}

void DisplayListMetalComplexityCalculator::MetalHelper::drawVertices(
    const std::shared_ptr<DlVertices>& vertices,
    DlBlendMode mode) {
  // There is currently no way for us to get the VertexMode from the SkVertices
  // object, but for future reference:
  //
  // TriangleStrip is roughly 25% more expensive than TriangleFan.
  // TriangleFan is roughly 5% more expensive than Triangles.

  // For the baseline, it's hard to identify the trend. It might be O(n^1/2).
  // For now, treat it as linear as an approximation.
  //
  // m = 1/4000
  // c = 1
  unsigned int complexity = (vertices->vertex_count() + 4000) * 50;

  AccumulateComplexity(complexity);
}

void DisplayListMetalComplexityCalculator::MetalHelper::drawImage(
    const sk_sp<DlImage> image,
    const DlPoint& point,
    DlImageSampling sampling,
    bool render_with_attributes) {
  if (IsComplex()) {
    return;
  }
  // AA vs non-AA has a cost but it's dwarfed by the overall cost of the
  // drawImage call.
  //
  // The main difference is if the image is backed by a texture already or not
  // If we don't need to upload, then the cost scales linearly with the
  // area of the image. If it needs uploading, the cost scales linearly
  // with the square of the area (!!!).
  DlISize dimensions = image->GetSize();
  unsigned int area = dimensions.Area();

  // m = 1/17000
  // c = 3
  unsigned int complexity = (area + 51000) * 4 / 170;

  if (!image->isTextureBacked()) {
    // We can't square the area here as we'll overflow, so let's approximate
    // by taking the calculated complexity score and applying a multiplier to
    // it.
    //
    // (complexity * area / 35000) + 1200 gives a reasonable approximation.
    float multiplier = area / 35000.0f;
    complexity = complexity * multiplier + 1200;
  }

  AccumulateComplexity(complexity);
}

void DisplayListMetalComplexityCalculator::MetalHelper::ImageRect(
    const DlISize& size,
    bool texture_backed,
    bool render_with_attributes,
    bool enforce_src_edges) {
  if (IsComplex()) {
    return;
  }
  // Two main groups here - texture-backed and non-texture-backed images.
  //
  // Within each group, they all perform within a few % of each other *except*
  // when we have a strict constraint and anti-aliasing enabled.
  unsigned int area = size.Area();

  // These values were worked out by creating a straight line graph (y=mx+c)
  // approximately matching the measured data, normalising the data so that
  // 0.0005ms resulted in a score of 100 then simplifying down the formula.
  unsigned int complexity;
  if (texture_backed) {
    // Baseline for texture-backed SkImages.
    // m = 1/23000
    // c = 2.3
    complexity = (area + 52900) * 2 / 115;
    if (render_with_attributes && enforce_src_edges && IsAntiAliased()) {
      // There's about a 30% performance penalty from the baseline.
      complexity *= 1.3f;
    }
  } else {
    if (render_with_attributes && enforce_src_edges && IsAntiAliased()) {
      // m = 1/12200
      // c = 2.75
      complexity = (area + 33550) * 2 / 61;
    } else {
      // m = 1/14500
      // c = 2.5
      complexity = (area + 36250) * 4 / 145;
    }
  }

  AccumulateComplexity(complexity);
}

void DisplayListMetalComplexityCalculator::MetalHelper::drawImageNine(
    const sk_sp<DlImage> image,
    const DlIRect& center,
    const DlRect& dst,
    DlFilterMode filter,
    bool render_with_attributes) {
  if (IsComplex()) {
    return;
  }
  // Whether uploading or not, the performance is comparable across all
  // variations.
  DlISize dimensions = image->GetSize();
  unsigned int area = dimensions.Area();

  // m = 1/8000
  // c = 3
  unsigned int complexity = (area + 24000) / 20;
  AccumulateComplexity(complexity);
}

void DisplayListMetalComplexityCalculator::MetalHelper::drawDisplayList(
    const sk_sp<DisplayList> display_list,
    DlScalar opacity) {
  if (IsComplex()) {
    return;
  }
  MetalHelper helper(Ceiling() - CurrentComplexityScore());
  if (opacity < SK_Scalar1 && !display_list->can_apply_group_opacity()) {
    auto bounds = display_list->GetBounds();
    helper.saveLayer(bounds, SaveLayerOptions::kWithAttributes, nullptr,
                     /*backdrop_id=*/-1);
  }
  display_list->Dispatch(helper);
  AccumulateComplexity(helper.ComplexityScore());
}

void DisplayListMetalComplexityCalculator::MetalHelper::drawTextBlob(
    const sk_sp<SkTextBlob> blob,
    DlScalar x,
    DlScalar y) {
  if (IsComplex()) {
    return;
  }

  // DrawTextBlob has a high fixed cost, but if we call it multiple times
  // per frame, that fixed cost is greatly reduced per subsequent call. This
  // is likely because there is batching being done in SkCanvas.

  // Increment draw_text_blob_count_ and calculate the cost at the end.
  draw_text_blob_count_++;
}

void DisplayListMetalComplexityCalculator::MetalHelper::drawTextFrame(
    const std::shared_ptr<impeller::TextFrame>& text_frame,
    DlScalar x,
    DlScalar y) {}

void DisplayListMetalComplexityCalculator::MetalHelper::drawShadow(
    const DlPath& path,
    const DlColor color,
    const DlScalar elevation,
    bool transparent_occluder,
    DlScalar dpr) {
  if (IsComplex()) {
    return;
  }

  // Elevation has no significant effect on the timings. Whether the shadow
  // is cast by a transparent occluder or not has a small impact of around 5%.
  //
  // The path verbs do have an effect but only if the verb type is cubic; line,
  // quad and conic all perform similarly.
  float occluder_penalty = 1.0f;
  if (transparent_occluder) {
    occluder_penalty = 1.05f;
  }

  // The benchmark uses a test path of around 10 path elements. This is likely
  // to be similar to what we see in real world usage, but we should benchmark
  // different path lengths to see how much impact there is from varying the
  // path length.
  //
  // For now, we will assume that there is no fixed overhead and that the time
  // spent rendering the shadow for a path is split evenly amongst all the verbs
  // enumerated.
  unsigned int line_verb_cost = 20000;   // 0.1ms
  unsigned int quad_verb_cost = 20000;   // 0.1ms
  unsigned int conic_verb_cost = 20000;  // 0.1ms
  unsigned int cubic_verb_cost = 80000;  // 0.4ms

  unsigned int complexity =
      0 + CalculatePathComplexity(path, line_verb_cost, quad_verb_cost,
                                  conic_verb_cost, cubic_verb_cost);

  AccumulateComplexity(complexity * occluder_penalty);
}

}  // namespace flutter
