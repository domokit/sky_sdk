// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_STOPWATCH_H_
#define FLUTTER_FLOW_STOPWATCH_H_

#include <vector>

#include "flutter/display_list/dl_canvas.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/time/time_delta.h"
#include "flutter/fml/time/time_point.h"

namespace flutter {

class Stopwatch {
 public:
  /// The refresh rate interface for `Stopwatch`.
  class RefreshRateUpdater {
   public:
    /// Time limit for a smooth frame.
    /// See: `DisplayManager::GetMainDisplayRefreshRate`.
    virtual fml::Milliseconds GetFrameBudget() const = 0;
  };

  /// The constructor with a updater parameter, it will update frame_budget
  /// everytime when `GetFrameBudget()` is called.
  explicit Stopwatch(const RefreshRateUpdater& updater);

  ~Stopwatch();

  const fml::TimeDelta& GetLap(size_t index) const;

  /// Return a reference to all the laps.
  size_t GetLapsCount() const;

  size_t GetCurrentSample() const;

  const fml::TimeDelta& LastLap() const;

  fml::TimeDelta MaxDelta() const;

  fml::TimeDelta AverageDelta() const;

  void Start();

  void Stop();

  void SetLapTime(const fml::TimeDelta& delta);

  /// All places which want to get frame_budget should call this function.
  fml::Milliseconds GetFrameBudget() const;

 private:
  const RefreshRateUpdater& refresh_rate_updater_;
  fml::TimePoint start_;
  std::vector<fml::TimeDelta> laps_;
  size_t current_sample_ = 0;

  FML_DISALLOW_COPY_AND_ASSIGN(Stopwatch);
};

/// Used for fixed refresh rate query cases.
class FixedRefreshRateUpdater : public Stopwatch::RefreshRateUpdater {
  fml::Milliseconds GetFrameBudget() const override;

 public:
  explicit FixedRefreshRateUpdater(
      fml::Milliseconds fixed_frame_budget = fml::kDefaultFrameBudget);

 private:
  fml::Milliseconds fixed_frame_budget_;
};

/// Used for fixed refresh rate cases.
class FixedRefreshRateStopwatch : public Stopwatch {
 public:
  explicit FixedRefreshRateStopwatch(
      fml::Milliseconds fixed_frame_budget = fml::kDefaultFrameBudget);

 private:
  FixedRefreshRateUpdater fixed_delegate_;
};

//------------------------------------------------------------------------------
/// @brief        Abstract class for visualizing (i.e. drawing) a stopwatch.
///
/// @note         This was originally folded into the |Stopwatch| class, but
///               was separated out to make it easier to change the underlying
///               implementation (which relied directly on Skia, not showing on
///               Impeller: https://github.com/flutter/flutter/issues/126009).
class StopwatchVisualizer {
 public:
  explicit StopwatchVisualizer(const Stopwatch& stopwatch)
      : stopwatch_(stopwatch) {
    // Looking up the frame budget from the stopwatch delegate class may call
    // into JNI or make platform calls which are slow. This value is safe to
    // cache since the StopwatchVisualizer is recreated on each frame.
    frame_budget_ = stopwatch_.GetFrameBudget();
  }

  virtual ~StopwatchVisualizer() = default;

  /// @brief      Renders the stopwatch as a graph.
  ///
  /// @param      canvas  The canvas to draw on.
  /// @param[in]  rect    The rectangle to draw in.
  virtual void Visualize(DlCanvas* canvas, const DlRect& rect) const = 0;

  FML_DISALLOW_COPY_AND_ASSIGN(StopwatchVisualizer);

 protected:
  /// @brief      Converts a raster time to a unit interval.
  double UnitFrameInterval(double time_ms) const;

  /// @brief      Converts a raster time to a unit height.
  double UnitHeight(double time_ms, double max_height) const;

  fml::Milliseconds GetFrameBudget() const { return frame_budget_; }

  const Stopwatch& stopwatch_;
  fml::Milliseconds frame_budget_;
};

}  // namespace flutter

#endif  // FLUTTER_FLOW_STOPWATCH_H_
