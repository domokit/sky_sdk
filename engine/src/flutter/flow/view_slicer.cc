// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/view_slicer.h"

#include <unordered_map>
#include "flow/embedded_views.h"
#include "fml/logging.h"

namespace flutter {

std::unordered_map<int64_t, SkRect> SliceViews(
    DlCanvas* background_canvas,
    const std::vector<int64_t>& composition_order,
    const std::unordered_map<int64_t, std::unique_ptr<EmbedderViewSlice>>&
        slices,
    const std::unordered_map<int64_t, SkRect>& view_rects) {
  std::unordered_map<int64_t, SkRect> overlay_layers;

  auto current_frame_view_count = composition_order.size();

  // Restore the clip context after exiting this method since it's changed
  // below.
  DlAutoCanvasRestore save(background_canvas, /*do_save=*/true);

  for (size_t i = 0; i < current_frame_view_count; i++) {
    int64_t view_id = composition_order[i];
    EmbedderViewSlice* slice = slices.at(view_id).get();
    if (slice->canvas() == nullptr) {
      continue;
    }

    slice->end_recording();

    DlRect full_joined_rect;

    // Determinate if Flutter UI intersects with any of the previous
    // platform views stacked by z position.
    //
    // This is done by querying the r-tree that holds the records for the
    // picture recorder corresponding to the flow layers added after a platform
    // view layer.
    for (int j = i; j >= 0; j--) {
      int64_t current_view_id = composition_order[j];
      auto maybe_rect = view_rects.find(current_view_id);
      FML_DCHECK(maybe_rect != view_rects.end());
      if (maybe_rect == view_rects.end()) {
        continue;
      }

      const SkRect current_view_rect = maybe_rect->second;
      const DlIRect rounded_in_platform_view_rect =
          DlIRect::RoundIn(ToDlRect(current_view_rect));
      const DlRect rounded_out_platform_view_rect =
          DlRect::RoundOut(ToDlRect(current_view_rect));

      // Each rect corresponds to a native view that renders Flutter UI.
      std::vector<DlIRect> intersection_rects =
          slice->region(ToDlRect(current_view_rect)).getRects();

      // Ignore intersections of single width/height on the edge of the platform
      // view.
      // This is to address the following performance issue when interleaving
      // adjacent platform views and layers: Since we `roundOut` both platform
      // view rects and the layer rects, as long as the coordinate is
      // fractional, there will be an intersection of a single pixel width (or
      // height) after rounding out, even if they do not intersect before
      // rounding out. We have to round out both platform view rect and the
      // layer rect. Rounding in platform view rect will result in missing pixel
      // on the intersection edge. Rounding in layer rect will result in missing
      // pixel on the edge of the layer on top of the platform view.
      for (auto it = intersection_rects.begin(); it != intersection_rects.end();
           /*no-op*/) {
        // If intersection_rect does not intersect with the *rounded in*
        // platform view rect, then the intersection must be a single pixel
        // width (or height) on edge.
        if (!it->IntersectsWithRect(rounded_in_platform_view_rect)) {
          it = intersection_rects.erase(it);
        } else {
          ++it;
        }
      }

      // Limit the number of native views, so it doesn't grow forever.
      //
      // In this case, the rects are merged into a single one that is the union
      // of all the rects.
      DlRect partial_joined_rect;
      for (const DlIRect& rect : intersection_rects) {
        partial_joined_rect = partial_joined_rect.Union(DlRect::Make(rect));
      }

      // Get the intersection rect with the `current_view_rect`,
      if (partial_joined_rect.IntersectsWithRect(
              rounded_out_platform_view_rect)) {
        // Join the `partial_joined_rect` into `full_joined_rect` to get the
        // rect above the current `slice`, only if it intersects the indicated
        // view. This should always be the case because we just deleted any
        // rects that don't intersect the "rounded-in" view, so they must
        // all intersect the "rounded-out" view (or the partial join could
        // be empty in which case this would be a NOP). Either way, the
        // penalty for not checking the return value of the intersect method
        // would be to join a non-overlapping rectangle into the overlay
        // bounds - if the above implementation ever changes - so we check it.
        full_joined_rect = full_joined_rect.Union(partial_joined_rect);
      }
    }

    if (!full_joined_rect.IsEmpty()) {
      overlay_layers.insert({view_id, ToSkRect(full_joined_rect)});

      // Clip the background canvas, so it doesn't contain any of the pixels
      // drawn on the overlay layer.
      background_canvas->ClipRect(full_joined_rect, DlClipOp::kDifference);
    }
    slice->render_into(background_canvas);
  }

  // Manually trigger the DlAutoCanvasRestore before we submit the frame
  save.Restore();

  return overlay_layers;
}

}  // namespace flutter
