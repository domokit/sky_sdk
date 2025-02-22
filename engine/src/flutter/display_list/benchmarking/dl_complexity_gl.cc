// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/benchmarking/dl_complexity_gl.h"

// The numbers and weightings used in this file stem from taking the
// data from the DisplayListBenchmarks suite run on an Pixel 4 and
// applying very rough analysis on them to identify the approximate
// trends.
//
// See the comments in display_list_complexity_helper.h for details on the
// process and rationale behind coming up with these numbers.

namespace flutter {

DisplayListGLComplexityCalculator*
    DisplayListGLComplexityCalculator::instance_ = nullptr;

DisplayListGLComplexityCalculator*
DisplayListGLComplexityCalculator::GetInstance() {
  if (instance_ == nullptr) {
    instance_ = new DisplayListGLComplexityCalculator();
  }
  return instance_;
}

unsigned int DisplayListGLComplexityCalculator::GLHelper::BatchedComplexity() {
  // Calculate the impact of saveLayer.
  unsigned int save_layer_complexity;
  if (save_layer_count_ == 0) {
    save_layer_complexity = 0;
  } else {
    // m = 1/5
    // c = 10
    save_layer_complexity = (save_layer_count_ + 50) * 40000;
  }

  unsigned int draw_text_blob_complexity;
  if (draw_text_blob_count_ == 0) {
    draw_text_blob_complexity = 0;
  } else {
    // m = 1/240
    // c = 0.25
    draw_text_blob_complexity = (draw_text_blob_count_ + 60) * 2500 / 3;
  }

  return save_layer_complexity + draw_text_blob_complexity;
}

void DisplayListGLComplexityCalculator::GLHelper::saveLayer(
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

void DisplayListGLComplexityCalculator::GLHelper::drawLine(const DlPoint& p0,
                                                           const DlPoint& p1) {
  if (IsComplex()) {
    return;
  }

  // There is a relatively high fixed overhead cost for drawLine on OpenGL.
  // Further, there is a strange bump where the cost of drawing a line of
  // length ~500px is actually more costly than drawing a line of length
  // ~1000px. The calculations here will be for a linear graph that
  // approximate the overall trend.

  float non_hairline_penalty = 1.0f;
  unsigned int aa_penalty = 1;

  // The non-hairline penalty is insignificant when AA is on.
  if (!IsHairline() && !IsAntiAliased()) {
    non_hairline_penalty = 1.15f;
  }
  if (IsAntiAliased()) {
    aa_penalty = 2;
  }

  // Use an approximation for the distance to avoid floating point or
  // sqrt() calls.
  DlScalar distance = abs(p0.x - p1.x) + abs(p0.y - p1.y);

  // The baseline complexity is for a hairline stroke with no AA.
  // m = 1/40
  // c = 13
  unsigned int complexity =
      ((distance + 520) / 2) * non_hairline_penalty * aa_penalty;

  AccumulateComplexity(complexity);
}

void DisplayListGLComplexityCalculator::GLHelper::drawDashedLine(
    const DlPoint& p0,
    const DlPoint& p1,
    DlScalar on_length,
    DlScalar off_length) {
  // Dashing is slightly more complex than a regular drawLine, but this
  // op is so rare it is not worth measuring the difference.
  drawLine(p0, p1);
}

void DisplayListGLComplexityCalculator::GLHelper::drawRect(const DlRect& rect) {
  if (IsComplex()) {
    return;
  }

  unsigned int complexity;

  // If stroked, cost scales linearly with the rectangle width/height.
  // If filled, it scales with the area.
  //
  // Hairline stroke vs non hairline has no significant penalty.
  //
  // There is also a kStrokeAndFill_Style that Skia exposes, but we do not
  // currently use it anywhere in Flutter.
  if (DrawStyle() == DlDrawStyle::kFill) {
    // No real difference for AA with filled styles
    unsigned int area = rect.GetWidth() * rect.GetHeight();

    // m = 1/3500
    // c = 0
    complexity = area * 2 / 175;
  } else {
    // Take the average of the width and height.
    unsigned int length = (rect.GetWidth() + rect.GetHeight()) / 2;

    if (IsAntiAliased()) {
      // m = 1/30
      // c = 0
      complexity = length * 4 / 3;
    } else {
      // If AA is disabled, the data shows that at larger sizes the overall
      // cost comes down, peaking at around 1000px. As we don't anticipate
      // rasterising rects with AA disabled to be all that frequent, just treat
      // it as a straight line that peaks at 1000px, beyond which it stays
      // constant. The rationale here is that it makes more sense to
      // overestimate than to start decreasing the cost as the length goes up.
      //
      // This should be a reasonable approximation as it doesn't decrease by
      // much from 1000px to 2000px.
      //
      // m = 1/20
      // c = 0
      complexity = std::min(length, 1000u) * 2;
    }
  }

  AccumulateComplexity(complexity);
}

void DisplayListGLComplexityCalculator::GLHelper::drawOval(
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
    // m = 1/6000
    // c = 0
    complexity = area / 30;
  } else {
    if (IsAntiAliased()) {
      // m = 1/4000
      // c = 0
      complexity = area / 20;
    } else {
      // Take the average of the width and height.
      unsigned int length = (bounds.GetWidth() + bounds.GetHeight()) / 2;

      // m = 1/75
      // c = 0
      complexity = length * 8 / 3;
    }
  }

  AccumulateComplexity(complexity);
}

void DisplayListGLComplexityCalculator::GLHelper::drawCircle(
    const DlPoint& center,
    DlScalar radius) {
  if (IsComplex()) {
    return;
  }

  unsigned int complexity;

  // There is also a kStrokeAndFill_Style that Skia exposes, but we do not
  // currently use it anywhere in Flutter.
  if (DrawStyle() == DlDrawStyle::kFill) {
    // We can ignore pi here
    unsigned int area = radius * radius;
    // m = 1/525
    // c = 50
    complexity = (area + 26250) * 8 / 105;

    // Penalty of around 8% when AA is disabled.
    if (!IsAntiAliased()) {
      complexity *= 1.08f;
    }
  } else {
    // Hairline vs non-hairline has no significant performance difference.
    if (IsAntiAliased()) {
      // m = 1/3
      // c = 10
      complexity = (radius + 30) * 40 / 3;
    } else {
      // m = 1/10
      // c = 20
      complexity = (radius + 200) * 4;
    }
  }

  AccumulateComplexity(complexity);
}

void DisplayListGLComplexityCalculator::GLHelper::drawRoundRect(
    const DlRoundRect& rrect) {
  if (IsComplex()) {
    return;
  }

  // Drawing RRects is split into three performance tiers:
  //
  // 1) All stroked styles without AA *except* simple/symmetric RRects.
  // 2) All filled styles and symmetric stroked styles w/AA.
  // 3) Remaining stroked styles with AA.
  //
  // 1) and 3) scale linearly with length, 2) scales with area.

  unsigned int complexity;

  // These values were worked out by creating a straight line graph (y=mx+c)
  // approximately matching the measured data, normalising the data so that
  // 0.0005ms resulted in a score of 100 then simplifying down the formula.
  if (DrawStyle() == DlDrawStyle::kFill ||
      ((rrect.GetRadii().AreAllCornersSame()) && IsAntiAliased())) {
    unsigned int area = rrect.GetBounds().Area();
    // m = 1/3200
    // c = 0.5
    complexity = (area + 1600) / 80;
  } else {
    // Take the average of the width and height.
    unsigned int length =
        (rrect.GetBounds().GetWidth() + rrect.GetBounds().GetHeight()) / 2;

    // There is some difference between hairline and non-hairline performance
    // but the spread is relatively inconsistent and it's pretty much a wash.
    if (IsAntiAliased()) {
      // m = 1/25
      // c = 1
      complexity = (length + 25) * 8 / 5;
    } else {
      // m = 1/50
      // c = 0.75
      complexity = ((length * 2) + 75) * 2 / 5;
    }
  }

  AccumulateComplexity(complexity);
}

void DisplayListGLComplexityCalculator::GLHelper::drawDiffRoundRect(
    const DlRoundRect& outer,
    const DlRoundRect& inner) {
  if (IsComplex()) {
    return;
  }
  // There are roughly four classes here:
  // a) Filled style.
  // b) Complex RRect type with AA enabled and filled style.
  // c) Stroked style with AA enabled.
  // d) Stroked style with AA disabled.
  //
  // a) and b) scale linearly with the area, c) and d) scale linearly with
  // a single dimension (length). In all cases, the dimensions refer to
  // the outer RRect.

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
      // m = 1/500
      // c = 0.5
      complexity = (area + 250) / 5;
    } else {
      // m = 1/1600
      // c = 2
      complexity = (area + 3200) / 16;
    }
  } else {
    unsigned int length =
        (outer.GetBounds().GetWidth() + outer.GetBounds().GetHeight()) / 2;
    if (IsAntiAliased()) {
      // m = 1/15
      // c = 1
      complexity = (length + 15) * 20 / 3;
    } else {
      // m = 1/27
      // c = 0.5
      complexity = ((length * 2) + 27) * 50 / 27;
    }
  }

  AccumulateComplexity(complexity);
}

void DisplayListGLComplexityCalculator::GLHelper::drawRoundSuperellipse(
    const DlRoundSuperellipse& rse) {
  // Drawing RSEs on Skia falls back to RRect.
  drawRoundRect(rse.ToApproximateRoundRect());
}

void DisplayListGLComplexityCalculator::GLHelper::drawPath(const DlPath& path) {
  if (IsComplex()) {
    return;
  }
  // There is negligible effect on the performance for hairline vs. non-hairline
  // stroke widths.
  //
  // The data for filled styles is currently suspicious, so for now we are going
  // to assign scores based on stroked styles.

  unsigned int line_verb_cost, quad_verb_cost, conic_verb_cost, cubic_verb_cost;
  unsigned int complexity;

  if (IsAntiAliased()) {
    // There seems to be a fixed cost of around 1ms for calling drawPath with
    // AA.
    complexity = 200000;

    line_verb_cost = 235;
    quad_verb_cost = 365;
    conic_verb_cost = 365;
    cubic_verb_cost = 725;
  } else {
    // There seems to be a fixed cost of around 0.25ms for calling drawPath.
    // without AA
    complexity = 50000;

    line_verb_cost = 135;
    quad_verb_cost = 150;
    conic_verb_cost = 200;
    cubic_verb_cost = 235;
  }

  complexity += CalculatePathComplexity(path, line_verb_cost, quad_verb_cost,
                                        conic_verb_cost, cubic_verb_cost);

  AccumulateComplexity(complexity);
}

void DisplayListGLComplexityCalculator::GLHelper::drawArc(
    const DlRect& oval_bounds,
    DlScalar start_degrees,
    DlScalar sweep_degrees,
    bool use_center) {
  if (IsComplex()) {
    return;
  }
  // Hairline vs non-hairline makes no difference to the performance.
  // Stroked styles without AA scale linearly with the log of the diameter.
  // Stroked styles with AA scale linearly with the area.
  // Filled styles scale lienarly with the area.
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
      // m = 1/3800
      // c = 12
      complexity = (area + 45600) / 171;
    } else {
      unsigned int diameter =
          (oval_bounds.GetWidth() + oval_bounds.GetHeight()) / 2;
      // m = 15
      // c = -100
      // This should never go negative though, so use std::max to ensure
      // c is never larger than 15*log_diameter.
      //
      // Pre-multiply by 15 here so we get a little bit more precision.
      unsigned int log_diameter = 15 * log(diameter);
      complexity = (log_diameter - std::max(log_diameter, 100u)) * 200 / 9;
    }
  } else {
    if (IsAntiAliased()) {
      // m = 1/1000
      // c = 10
      complexity = (area + 10000) / 45;
    } else {
      // m = 1/6500
      // c = 12
      complexity = (area + 52000) * 2 / 585;
    }
  }

  AccumulateComplexity(complexity);
}

void DisplayListGLComplexityCalculator::GLHelper::drawPoints(
    DlPointMode mode,
    uint32_t count,
    const DlPoint points[]) {
  if (IsComplex()) {
    return;
  }
  unsigned int complexity;

  if (IsAntiAliased()) {
    if (mode == DlPointMode::kPoints) {
      if (IsHairline()) {
        // This is a special case, it triggers an extremely fast path.
        // m = 1/4500
        // c = 0
        complexity = count * 400 / 9;
      } else {
        // m = 1/500
        // c = 0
        complexity = count * 400;
      }
    } else if (mode == DlPointMode::kLines) {
      if (IsHairline()) {
        // m = 1/750
        // c = 0
        complexity = count * 800 / 3;
      } else {
        // m = 1/500
        // c = 0
        complexity = count * 400;
      }
    } else {
      if (IsHairline()) {
        // m = 1/350
        // c = 0
        complexity = count * 4000 / 7;
      } else {
        // m = 1/250
        // c = 0
        complexity = count * 800;
      }
    }
  } else {
    if (mode == DlPointMode::kPoints) {
      // Hairline vs non hairline makes no difference for points without AA.
      // m = 1/18000
      // c = 0.25
      complexity = (count + 4500) * 100 / 9;
    } else if (mode == DlPointMode::kLines) {
      if (IsHairline()) {
        // m = 1/8500
        // c = 0.25
        complexity = (count + 2125) * 400 / 17;
      } else {
        // m = 1/9000
        // c = 0.25
        complexity = (count + 2250) * 200 / 9;
      }
    } else {
      // Polygon only really diverges for hairline vs non hairline at large
      // point counts, and only by a few %.
      // m = 1/7500
      // c = 0.25
      complexity = (count + 1875) * 80 / 3;
    }
  }

  AccumulateComplexity(complexity);
}

void DisplayListGLComplexityCalculator::GLHelper::drawVertices(
    const std::shared_ptr<DlVertices>& vertices,
    DlBlendMode mode) {
  // There is currently no way for us to get the VertexMode from the SkVertices
  // object, but for future reference:
  //
  // TriangleStrip is roughly 25% more expensive than TriangleFan.
  // TriangleFan is roughly 5% more expensive than Triangles.

  // For the baseline, it's hard to identify the trend. It might be O(n^1/2)
  // For now, treat it as linear as an approximation.
  //
  // m = 1/1600
  // c = 1
  unsigned int complexity = (vertices->vertex_count() + 1600) * 250 / 2;

  AccumulateComplexity(complexity);
}

void DisplayListGLComplexityCalculator::GLHelper::drawImage(
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
  // length of the image. If it needs uploading, the cost scales linearly
  // with the square of the area (!!!).
  DlISize dimensions = image->GetSize();
  unsigned int length = (dimensions.width + dimensions.height) / 2;
  unsigned int area = dimensions.Area();

  // m = 1/13
  // c = 0
  unsigned int complexity = length * 400 / 13;

  if (!image->isTextureBacked()) {
    // We can't square the area here as we'll overflow, so let's approximate
    // by taking the calculated complexity score and applying a multiplier to
    // it.
    //
    // (complexity * area / 60000) + 4000 gives a reasonable approximation with
    // AA (complexity * area / 19000) gives a reasonable approximation without
    // AA.
    float multiplier;
    if (IsAntiAliased()) {
      multiplier = area / 60000.0f;
      complexity = complexity * multiplier + 4000;
    } else {
      multiplier = area / 19000.0f;
      complexity = complexity * multiplier;
    }
  }

  AccumulateComplexity(complexity);
}

void DisplayListGLComplexityCalculator::GLHelper::ImageRect(
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

  // These values were worked out by creating a straight line graph (y=mx+c)
  // approximately matching the measured data, normalising the data so that
  // 0.0005ms resulted in a score of 100 then simplifying down the formula.
  unsigned int complexity;
  if (!texture_backed || (texture_backed && render_with_attributes &&
                          enforce_src_edges && IsAntiAliased())) {
    unsigned int area = size.Area();
    // m = 1/4000
    // c = 5
    complexity = (area + 20000) / 10;
  } else {
    unsigned int length = (size.width + size.height) / 2;
    // There's a little bit of spread here but the numbers are pretty large
    // anyway.
    //
    // m = 1/22
    // c = 0
    complexity = length * 200 / 11;
  }

  AccumulateComplexity(complexity);
}

void DisplayListGLComplexityCalculator::GLHelper::drawImageNine(
    const sk_sp<DlImage> image,
    const DlIRect& center,
    const DlRect& dst,
    DlFilterMode filter,
    bool render_with_attributes) {
  if (IsComplex()) {
    return;
  }

  DlISize dimensions = image->GetSize();
  unsigned int area = dimensions.Area();

  // m = 1/3600
  // c = 3
  unsigned int complexity = (area + 10800) / 9;

  // Uploading incurs about a 40% performance penalty.
  if (!image->isTextureBacked()) {
    complexity *= 1.4f;
  }

  AccumulateComplexity(complexity);
}

void DisplayListGLComplexityCalculator::GLHelper::drawDisplayList(
    const sk_sp<DisplayList> display_list,
    DlScalar opacity) {
  if (IsComplex()) {
    return;
  }
  GLHelper helper(Ceiling() - CurrentComplexityScore());
  if (opacity < SK_Scalar1 && !display_list->can_apply_group_opacity()) {
    auto bounds = display_list->GetBounds();
    helper.saveLayer(bounds, SaveLayerOptions::kWithAttributes, nullptr,
                     /*backdrop_id=*/-1);
  }
  display_list->Dispatch(helper);
  AccumulateComplexity(helper.ComplexityScore());
}

void DisplayListGLComplexityCalculator::GLHelper::drawTextBlob(
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

void DisplayListGLComplexityCalculator::GLHelper::drawTextFrame(
    const std::shared_ptr<impeller::TextFrame>& text_frame,
    DlScalar x,
    DlScalar y) {}

void DisplayListGLComplexityCalculator::GLHelper::drawShadow(
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
    occluder_penalty = 1.20f;
  }

  // The benchmark uses a test path of around 10 path elements. This is likely
  // to be similar to what we see in real world usage, but we should benchmark
  // different path lengths to see how much impact there is from varying the
  // path length.
  //
  // For now, we will assume that there is no fixed overhead and that the time
  // spent rendering the shadow for a path is split evenly amongst all the verbs
  // enumerated.
  unsigned int line_verb_cost = 17000;    // 0.085ms
  unsigned int quad_verb_cost = 20000;    // 0.1ms
  unsigned int conic_verb_cost = 20000;   // 0.1ms
  unsigned int cubic_verb_cost = 120000;  // 0.6ms

  unsigned int complexity = CalculatePathComplexity(
      path, line_verb_cost, quad_verb_cost, conic_verb_cost, cubic_verb_cost);

  AccumulateComplexity(complexity * occluder_penalty);
}

}  // namespace flutter
