// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/frame_timings.h"

#include <memory>
#include <string>

#include "flutter/common/settings.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/time/time_point.h"

namespace flutter {

namespace {

const char* StateToString(FrameTimingsRecorder::State state) {
#ifndef NDEBUG
  switch (state) {
    case FrameTimingsRecorder::State::kUninitialized:
      return "kUninitialized";
    case FrameTimingsRecorder::State::kVsync:
      return "kVsync";
    case FrameTimingsRecorder::State::kBuildStart:
      return "kBuildStart";
    case FrameTimingsRecorder::State::kBuildEnd:
      return "kBuildEnd";
    case FrameTimingsRecorder::State::kRasterStart:
      return "kRasterStart";
    case FrameTimingsRecorder::State::kRasterEnd:
      return "kRasterEnd";
  };
  FML_UNREACHABLE();
#endif
  return "";
}

}  // namespace

std::atomic<uint64_t> FrameTimingsRecorder::frame_number_gen_ = {1};

FrameTimingsRecorder::FrameTimingsRecorder()
    : frame_number_(frame_number_gen_++),
      frame_number_trace_arg_val_(std::to_string(frame_number_)) {}

FrameTimingsRecorder::FrameTimingsRecorder(uint64_t frame_number)
    : frame_number_(frame_number),
      frame_number_trace_arg_val_(std::to_string(frame_number_)) {}

FrameTimingsRecorder::~FrameTimingsRecorder() = default;

fml::TimePoint FrameTimingsRecorder::GetVsyncStartTime() const {
  std::scoped_lock state_lock(state_mutex_);
  FML_DCHECK(state_ >= State::kVsync);
  return vsync_start_;
}

fml::TimePoint FrameTimingsRecorder::GetVsyncTargetTime() const {
  std::scoped_lock state_lock(state_mutex_);
  FML_DCHECK(state_ >= State::kVsync);
  return vsync_target_;
}

fml::TimePoint FrameTimingsRecorder::GetBuildStartTime() const {
  std::scoped_lock state_lock(state_mutex_);
  FML_DCHECK(state_ >= State::kBuildStart);
  return build_start_;
}

fml::TimePoint FrameTimingsRecorder::GetBuildEndTime() const {
  std::scoped_lock state_lock(state_mutex_);
  FML_DCHECK(state_ >= State::kBuildEnd);
  return build_end_;
}

fml::TimePoint FrameTimingsRecorder::GetRasterStartTime() const {
  std::scoped_lock state_lock(state_mutex_);
  FML_DCHECK(state_ >= State::kRasterStart);
  return raster_start_;
}

fml::TimePoint FrameTimingsRecorder::GetRasterEndTime() const {
  std::scoped_lock state_lock(state_mutex_);
  FML_DCHECK(state_ >= State::kRasterEnd);
  return raster_end_;
}

fml::TimePoint FrameTimingsRecorder::GetRasterEndWallTime() const {
  std::scoped_lock state_lock(state_mutex_);
  FML_DCHECK(state_ >= State::kRasterEnd);
  return raster_end_wall_time_;
}

fml::TimeDelta FrameTimingsRecorder::GetBuildDuration() const {
  std::scoped_lock state_lock(state_mutex_);
  FML_DCHECK(state_ >= State::kBuildEnd);
  return build_end_ - build_start_;
}

/// Count of the layer cache entries
size_t FrameTimingsRecorder::GetLayerCacheCount() const {
  std::scoped_lock state_lock(state_mutex_);
  FML_DCHECK(state_ >= State::kRasterEnd);
  return layer_cache_count_;
}

/// Total bytes in all layer cache entries
size_t FrameTimingsRecorder::GetLayerCacheBytes() const {
  std::scoped_lock state_lock(state_mutex_);
  FML_DCHECK(state_ >= State::kRasterEnd);
  return layer_cache_bytes_;
}

/// Count of the picture cache entries
size_t FrameTimingsRecorder::GetPictureCacheCount() const {
  std::scoped_lock state_lock(state_mutex_);
  FML_DCHECK(state_ >= State::kRasterEnd);
  return picture_cache_count_;
}

/// Total bytes in all picture cache entries
size_t FrameTimingsRecorder::GetPictureCacheBytes() const {
  std::scoped_lock state_lock(state_mutex_);
  FML_DCHECK(state_ >= State::kRasterEnd);
  return picture_cache_bytes_;
}

void FrameTimingsRecorder::RecordVsync(fml::TimePoint vsync_start,
                                       fml::TimePoint vsync_target) {
  fml::Status status = RecordVsyncImpl(vsync_start, vsync_target);
  FML_DCHECK(status.ok());
  (void)status;
}

void FrameTimingsRecorder::RecordBuildStart(fml::TimePoint build_start) {
  fml::Status status = RecordBuildStartImpl(build_start);
  FML_DCHECK(status.ok());
  (void)status;
}

void FrameTimingsRecorder::RecordBuildEnd(fml::TimePoint build_end) {
  fml::Status status = RecordBuildEndImpl(build_end);
  FML_DCHECK(status.ok());
  (void)status;
}

void FrameTimingsRecorder::RecordRasterStart(fml::TimePoint raster_start) {
  fml::Status status = RecordRasterStartImpl(raster_start);
  FML_DCHECK(status.ok());
  (void)status;
}

fml::Status FrameTimingsRecorder::RecordVsyncImpl(fml::TimePoint vsync_start,
                                                  fml::TimePoint vsync_target) {
  std::scoped_lock state_lock(state_mutex_);
  if (state_ != State::kUninitialized) {
    return fml::Status(fml::StatusCode::kFailedPrecondition,
                       "Check failed: state_ == State::kUninitialized.");
  }
  state_ = State::kVsync;
  vsync_start_ = vsync_start;
  vsync_target_ = vsync_target;
  return fml::Status();
}

fml::Status FrameTimingsRecorder::RecordBuildStartImpl(
    fml::TimePoint build_start) {
  std::scoped_lock state_lock(state_mutex_);
  if (state_ != State::kVsync) {
    return fml::Status(fml::StatusCode::kFailedPrecondition,
                       "Check failed: state_ == State::kVsync.");
  }
  state_ = State::kBuildStart;
  build_start_ = build_start;
  return fml::Status();
}

fml::Status FrameTimingsRecorder::RecordBuildEndImpl(fml::TimePoint build_end) {
  std::scoped_lock state_lock(state_mutex_);
  if (state_ != State::kBuildStart) {
    return fml::Status(fml::StatusCode::kFailedPrecondition,
                       "Check failed: state_ == State::kBuildStart.");
  }
  state_ = State::kBuildEnd;
  build_end_ = build_end;
  return fml::Status();
}

fml::Status FrameTimingsRecorder::RecordRasterStartImpl(
    fml::TimePoint raster_start) {
  std::scoped_lock state_lock(state_mutex_);
  if (state_ != State::kBuildEnd) {
    return fml::Status(fml::StatusCode::kFailedPrecondition,
                       "Check failed: state_ == State::kBuildEnd.");
  }
  state_ = State::kRasterStart;
  raster_start_ = raster_start;
  return fml::Status();
}

FrameTiming FrameTimingsRecorder::RecordRasterEnd(const RasterCache* cache) {
  std::scoped_lock state_lock(state_mutex_);
  FML_DCHECK(state_ == State::kRasterStart);
  state_ = State::kRasterEnd;
  raster_end_ = fml::TimePoint::Now();
  raster_end_wall_time_ = fml::TimePoint::CurrentWallTime();
  if (cache) {
#if !SLIMPELLER
    const RasterCacheMetrics& layer_metrics = cache->layer_metrics();
    const RasterCacheMetrics& picture_metrics = cache->picture_metrics();
    layer_cache_count_ = layer_metrics.total_count();
    layer_cache_bytes_ = layer_metrics.total_bytes();
    picture_cache_count_ = picture_metrics.total_count();
    picture_cache_bytes_ = picture_metrics.total_bytes();
#endif  //  !SLIMPELLER
  } else {
    layer_cache_count_ = layer_cache_bytes_ = picture_cache_count_ =
        picture_cache_bytes_ = 0;
  }
  timing_.Set(FrameTiming::kVsyncStart, vsync_start_);
  timing_.Set(FrameTiming::kBuildStart, build_start_);
  timing_.Set(FrameTiming::kBuildFinish, build_end_);
  timing_.Set(FrameTiming::kRasterStart, raster_start_);
  timing_.Set(FrameTiming::kRasterFinish, raster_end_);
  timing_.Set(FrameTiming::kRasterFinishWallTime, raster_end_wall_time_);
  timing_.SetFrameNumber(GetFrameNumber());
  timing_.SetRasterCacheStatistics(layer_cache_count_, layer_cache_bytes_,
                                   picture_cache_count_, picture_cache_bytes_);
  return timing_;
}

FrameTiming FrameTimingsRecorder::GetRecordedTime() const {
  std::scoped_lock state_lock(state_mutex_);
  FML_DCHECK(state_ == State::kRasterEnd);
  return timing_;
}

std::unique_ptr<FrameTimingsRecorder> FrameTimingsRecorder::CloneUntil(
    State state) {
  std::scoped_lock state_lock(state_mutex_);
  std::unique_ptr<FrameTimingsRecorder> recorder =
      std::make_unique<FrameTimingsRecorder>(frame_number_);
  FML_DCHECK(state_ >= state);
  recorder->state_ = state;

  if (state >= State::kVsync) {
    recorder->vsync_start_ = vsync_start_;
    recorder->vsync_target_ = vsync_target_;
  }

  if (state >= State::kBuildStart) {
    recorder->build_start_ = build_start_;
  }

  if (state >= State::kBuildEnd) {
    recorder->build_end_ = build_end_;
  }

  if (state >= State::kRasterStart) {
    recorder->raster_start_ = raster_start_;
  }

  if (state >= State::kRasterEnd) {
    recorder->raster_end_ = raster_end_;
    recorder->raster_end_wall_time_ = raster_end_wall_time_;
    recorder->layer_cache_count_ = layer_cache_count_;
    recorder->layer_cache_bytes_ = layer_cache_bytes_;
    recorder->picture_cache_count_ = picture_cache_count_;
    recorder->picture_cache_bytes_ = picture_cache_bytes_;
  }

  return recorder;
}

uint64_t FrameTimingsRecorder::GetFrameNumber() const {
  return frame_number_;
}

const char* FrameTimingsRecorder::GetFrameNumberTraceArg() const {
  return frame_number_trace_arg_val_.c_str();
}

void FrameTimingsRecorder::AssertInState(State state) const {
  FML_DCHECK(state_ == state) << "Expected state " << StateToString(state)
                              << ", actual state " << StateToString(state_);
}

}  // namespace flutter
