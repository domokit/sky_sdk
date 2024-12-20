// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/save_layer_utils.h"
#include "impeller/geometry/scalar.h"

namespace impeller {

namespace {
bool SizeDifferenceUnderThreshold(Size a, Size b, Scalar threshold) {
  return (std::abs(a.width - b.width) / b.width) < threshold &&
         (std::abs(a.height - b.height) / b.height) < threshold;
}

static constexpr Scalar kDefaultSizeThreshold = 0.3;
}  // namespace

std::optional<Rect> ComputeSaveLayerCoverage(
    const Rect& content_coverage,
    const Matrix& effect_transform,
    const Rect& coverage_limit,
    const std::shared_ptr<FilterContents>& image_filter,
    bool flood_output_coverage,
    bool flood_input_coverage) {
  Rect coverage = content_coverage;
  // There are three conditions that should cause input coverage to flood, the
  // first is the presence of a backdrop filter on the saveLayer. The second is
  // the presence of a color filter that effects transparent black on the
  // saveLayer. The last is the presence of unbounded content within the
  // saveLayer (such as a drawPaint, bdf, et cetera). Note that unbounded
  // coverage is handled in the display list dispatcher.
  //
  // Backdrop filters apply before the saveLayer is restored. The presence of
  // a backdrop filter causes the content coverage of the saveLayer to be
  // unbounded.
  //
  // If there is a color filter that needs to flood its output. The color filter
  // is applied before any image filters, so this floods input coverage and not
  // the output coverage. Technically, we only need to flood the output of the
  // color filter and could allocate a render target sized just to the content,
  // but we don't currenty have the means to do so. Flooding the coverage is a
  // non-optimal but technically correct way to render this.
  //
  // If the saveLayer contains unbounded content, then at this point the
  // dl_dispatcher will have set content coverage to Rect::MakeMaximum().
  if (flood_input_coverage) {
    coverage = Rect::MakeMaximum();
  }

  // The content coverage must be scaled by any image filters present on the
  // saveLayer paint. For example, if a saveLayer has a coverage limit of
  // 100x100, but it has a Matrix image filter that scales by one half, the
  // actual coverage limit is 200x200.
  if (image_filter) {
    // Transform the input coverage into the global coordinate space before
    // computing the bounds limit intersection. This is the "worst case"
    // coverage value before we intersect with the content coverage below.
    std::optional<Rect> source_coverage_limit =
        image_filter->GetSourceCoverage(effect_transform, coverage_limit);
    if (!source_coverage_limit.has_value()) {
      // No intersection with parent coverage limit.
      return std::nullopt;
    }
    // The image filter may change the coverage limit required to flood
    // the parent layer. Returning the source coverage limit so that we
    // can guarantee the render target is larger enough.
    //
    // See note below on flood_output_coverage.
    if (flood_output_coverage || coverage.IsMaximum()) {
      return source_coverage_limit;
    }

    // Trimming the content coverage by the coverage limit can reduce memory
    // bandwith. But in cases where there are animated matrix filters, such as
    // in the framework's zoom transition, the changing scale values continually
    // change the source_coverage_limit. Intersecting the source_coverage_limit
    // with the coverage may result in slightly different texture sizes each
    // frame of the animation. This leads to non-optimal allocation patterns as
    // differently sized textures cannot be reused. Hence the following
    // herustic: If the coverage is within a semi-arbitrary percentage of the
    // intersected coverage, then just use the transformed coverage. In other
    // cases, use the intersection.
    auto transformed_coverage = coverage.TransformBounds(effect_transform);
    auto intersected_coverage =
        transformed_coverage.Intersection(source_coverage_limit.value());
    if (intersected_coverage.has_value() &&
        SizeDifferenceUnderThreshold(transformed_coverage.GetSize(),
                                     intersected_coverage->GetSize(),
                                     kDefaultSizeThreshold)) {
      // Returning the transformed coverage is always correct, it just may
      // be larger than the clip area or onscreen texture.
      return transformed_coverage;
    }
    return intersected_coverage;
  }

  // If the input coverage is maximum, just return the coverage limit that
  // is already in the global coordinate space.
  //
  // If flood_output_coverage is true, then the restore is applied with a
  // destructive blend mode that requires flooding to the coverage limit.
  // Technically we could only allocated a render target as big as the input
  // coverage and then use a decal sampling mode to perform the flood. Returning
  // the coverage limit is a correct but non optimal means of ensuring correct
  // rendering.
  if (flood_output_coverage || coverage.IsMaximum()) {
    return coverage_limit;
  }

  // Transform the input coverage into the global coordinate space before
  // computing the bounds limit intersection.
  Rect transformed_coverage = coverage.TransformBounds(effect_transform);
  std::optional<Rect> intersection =
      transformed_coverage.Intersection(coverage_limit);
  if (!intersection.has_value()) {
    return std::nullopt;
  }
  // The the resulting coverage rect is nearly the same as the coverage_limit,
  // round up to the coverage_limit.
  Rect intersect_rect = intersection.value();
  if (SizeDifferenceUnderThreshold(intersect_rect.GetSize(),
                                   coverage_limit.GetSize(),
                                   kDefaultSizeThreshold)) {
    return coverage_limit;
  }

  return intersect_rect;
}

}  // namespace impeller
