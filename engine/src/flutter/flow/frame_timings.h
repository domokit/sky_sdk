// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_FRAME_TIMINGS_H_
#define FLUTTER_FLOW_FRAME_TIMINGS_H_

#include <mutex>

#include "flutter/common/settings.h"
#include "flutter/flow/raster_cache.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/status.h"
#include "flutter/fml/time/time_delta.h"
#include "flutter/fml/time/time_point.h"

#define TRACE_EVENT_WITH_FRAME_NUMBER(recorder, category_group, name,       \
                                      flow_id_count, flow_ids)              \
  TRACE_EVENT1_WITH_FLOW_IDS(category_group, name, flow_id_count, flow_ids, \
                             "frame_number",                                \
                             recorder->GetFrameNumberTraceArg())

namespace flutter {

/// Records timestamps for various phases of a frame rendering process.
///
/// Recorder is created on vsync and destroyed after the rasterization of the
/// frame. This class is thread safe and doesn't require additional
/// synchronization.
class FrameTimingsRecorder {
 public:
  /// Various states that the recorder can be in. When created the recorder is
  /// in an unitialized state and transtions in sequential order of the states.
  // After adding an item to this enum, modify StateToString accordingly.
  enum class State : uint32_t {
    kUninitialized,
    kVsync,
    kBuildStart,
    kBuildEnd,
    kRasterStart,
    kRasterEnd,
  };

  /// Default constructor, initializes the recorder with State::kUninitialized.
  FrameTimingsRecorder();

  /// Constructor with a pre-populated frame number.
  explicit FrameTimingsRecorder(uint64_t frame_number);

  ~FrameTimingsRecorder();

  /// Timestamp of the vsync signal.
  fml::TimePoint GetVsyncStartTime() const;

  /// Timestamp of when the frame was targeted to be presented.
  ///
  /// This is typically the next vsync signal timestamp.
  fml::TimePoint GetVsyncTargetTime() const;

  /// Timestamp of when the frame building started.
  fml::TimePoint GetBuildStartTime() const;

  /// Timestamp of when the frame was finished building.
  fml::TimePoint GetBuildEndTime() const;

  /// Timestamp of when the frame rasterization started.
  fml::TimePoint GetRasterStartTime() const;

  /// Timestamp of when the frame rasterization finished.
  fml::TimePoint GetRasterEndTime() const;

  /// Timestamp of when the frame rasterization is complete in wall-time.
  fml::TimePoint GetRasterEndWallTime() const;

  /// Duration of the frame build time.
  fml::TimeDelta GetBuildDuration() const;

  /// Count of the layer cache entries
  size_t GetLayerCacheCount() const;

  /// Total Bytes in all layer cache entries
  size_t GetLayerCacheBytes() const;

  /// Count of the picture cache entries
  size_t GetPictureCacheCount() const;

  /// Total Bytes in all picture cache entries
  size_t GetPictureCacheBytes() const;

  /// Records a vsync event.
  void RecordVsync(fml::TimePoint vsync_start, fml::TimePoint vsync_target);

  /// Records a build start event.
  void RecordBuildStart(fml::TimePoint build_start);

  /// Records a build end event.
  void RecordBuildEnd(fml::TimePoint build_end);

  /// Records a raster start event.
  void RecordRasterStart(fml::TimePoint raster_start);

  /// Clones the recorder until (and including) the specified state.
  std::unique_ptr<FrameTimingsRecorder> CloneUntil(State state);

  /// Records a raster end event, and builds a `FrameTiming` that summarizes all
  /// the events. This summary is sent to the framework.
  FrameTiming RecordRasterEnd(const RasterCache* cache = nullptr);

  /// Returns the frame number. Frame number is unique per frame and a frame
  /// built earlier will have a frame number less than a frame that has been
  /// built at a later point of time.
  uint64_t GetFrameNumber() const;

  /// Returns the frame number in a fml tracing friendly format.
  const char* GetFrameNumberTraceArg() const;

  /// Returns the recorded time from when `RecordRasterEnd` is called.
  FrameTiming GetRecordedTime() const;

  /// Asserts in unopt builds that the recorder is current at the specified
  /// state.
  ///
  /// Instead of adding a `GetState` method and asserting on the result, this
  /// method prevents other logic from relying on the state.
  ///
  /// In release builds, this call is a no-op.
  void AssertInState(State state) const;

 private:
  FML_FRIEND_TEST(FrameTimingsRecorderTest, ThrowWhenRecordBuildBeforeVsync);
  FML_FRIEND_TEST(FrameTimingsRecorderTest,
                  ThrowWhenRecordRasterBeforeBuildEnd);

  [[nodiscard]] fml::Status RecordVsyncImpl(fml::TimePoint vsync_start,
                                            fml::TimePoint vsync_target);
  [[nodiscard]] fml::Status RecordBuildStartImpl(fml::TimePoint build_start);
  [[nodiscard]] fml::Status RecordBuildEndImpl(fml::TimePoint build_end);
  [[nodiscard]] fml::Status RecordRasterStartImpl(fml::TimePoint raster_start);

  static std::atomic<uint64_t> frame_number_gen_;

  mutable std::mutex state_mutex_;
  State state_ = State::kUninitialized;

  const uint64_t frame_number_;
  const std::string frame_number_trace_arg_val_;

  fml::TimePoint vsync_start_;
  fml::TimePoint vsync_target_;
  fml::TimePoint build_start_;
  fml::TimePoint build_end_;
  fml::TimePoint raster_start_;
  fml::TimePoint raster_end_;
  fml::TimePoint raster_end_wall_time_;

  size_t layer_cache_count_;
  size_t layer_cache_bytes_;
  size_t picture_cache_count_;
  size_t picture_cache_bytes_;

  // Set when `RecordRasterEnd` is called. Cannot be reset once set.
  FrameTiming timing_;

  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(FrameTimingsRecorder);
};

}  // namespace flutter

#endif  // FLUTTER_FLOW_FRAME_TIMINGS_H_
