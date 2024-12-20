// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_STOPWATCH_SK_H_
#define FLUTTER_FLOW_STOPWATCH_SK_H_

#include "flow/stopwatch.h"
#include "include/core/SkSurface.h"

namespace flutter {

//------------------------------------------------------------------------------
/// A stopwatch visualizer that uses Skia (|SkCanvas|) to draw the stopwatch.
///
/// @see DlStopwatchVisualizer for the newer non-backend specific version.
class SkStopwatchVisualizer : public StopwatchVisualizer {
 public:
  explicit SkStopwatchVisualizer(const Stopwatch& stopwatch)
      : StopwatchVisualizer(stopwatch) {}

  void Visualize(DlCanvas* canvas, const DlRect& rect) const override;

 private:
  /// Initializes the |SkSurface| used for drawing the stopwatch.
  ///
  /// Draws the base background and any timing data from before the initial
  /// call to |Visualize|.
  void InitVisualizeSurface(SkISize size) const;

  // Mutable data cache for performance optimization of the graphs.
  // Prevents expensive redrawing of old data.
  mutable bool cache_dirty_ = true;
  mutable sk_sp<SkSurface> visualize_cache_surface_;
  mutable size_t prev_drawn_sample_index_ = 0;
};

}  // namespace flutter

#endif  // FLUTTER_FLOW_STOPWATCH_SK_H_
