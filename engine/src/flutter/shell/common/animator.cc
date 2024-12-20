// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/common/animator.h"

#include "flutter/common/constants.h"
#include "flutter/flow/frame_timings.h"
#include "flutter/fml/time/time_point.h"
#include "flutter/fml/trace_event.h"
#include "third_party/dart/runtime/include/dart_tools_api.h"

namespace flutter {

namespace {

// Wait 51 milliseconds (which is 1 more milliseconds than 3 frames at 60hz)
// before notifying the engine that we are idle.  See comments in |BeginFrame|
// for further discussion on why this is necessary.
constexpr fml::TimeDelta kNotifyIdleTaskWaitTime =
    fml::TimeDelta::FromMilliseconds(51);

}  // namespace

Animator::Animator(Delegate& delegate,
                   const TaskRunners& task_runners,
                   std::unique_ptr<VsyncWaiter> waiter)
    : delegate_(delegate),
      task_runners_(task_runners),
      waiter_(std::move(waiter)),
#if SHELL_ENABLE_METAL
      layer_tree_pipeline_(std::make_shared<FramePipeline>(2)),
#else   // SHELL_ENABLE_METAL
      // TODO(dnfield): We should remove this logic and set the pipeline depth
      // back to 2 in this case. See
      // https://github.com/flutter/engine/pull/9132 for discussion.
      layer_tree_pipeline_(std::make_shared<FramePipeline>(
          task_runners.GetPlatformTaskRunner() ==
                  task_runners.GetRasterTaskRunner()
              ? 1
              : 2)),
#endif  // SHELL_ENABLE_METAL
      pending_frame_semaphore_(1),
      weak_factory_(this) {
}

Animator::~Animator() = default;

void Animator::EnqueueTraceFlowId(uint64_t trace_flow_id) {
  fml::TaskRunner::RunNowOrPostTask(
      task_runners_.GetUITaskRunner(),
      [self = weak_factory_.GetWeakPtr(), trace_flow_id] {
        if (!self) {
          return;
        }
        self->trace_flow_ids_.push_back(trace_flow_id);
        self->ScheduleMaybeClearTraceFlowIds();
      });
}

void Animator::BeginFrame(
    std::unique_ptr<FrameTimingsRecorder> frame_timings_recorder) {
  TRACE_EVENT_ASYNC_END0("flutter", "Frame Request Pending",
                         frame_request_number_);
  // Clear layer trees rendered out of a frame. Only Animator::Render called
  // within a frame is used.
  layer_trees_tasks_.clear();

  frame_request_number_++;

  frame_timings_recorder_ = std::move(frame_timings_recorder);
  frame_timings_recorder_->RecordBuildStart(fml::TimePoint::Now());

  size_t flow_id_count = trace_flow_ids_.size();
  std::unique_ptr<uint64_t[]> flow_ids =
      std::make_unique<uint64_t[]>(flow_id_count);
  for (size_t i = 0; i < flow_id_count; ++i) {
    flow_ids.get()[i] = trace_flow_ids_.at(i);
  }

  TRACE_EVENT_WITH_FRAME_NUMBER(frame_timings_recorder_, "flutter",
                                "Animator::BeginFrame", flow_id_count,
                                flow_ids.get());

  while (!trace_flow_ids_.empty()) {
    uint64_t trace_flow_id = trace_flow_ids_.front();
    TRACE_FLOW_END("flutter", "PointerEvent", trace_flow_id);
    trace_flow_ids_.pop_front();
  }

  frame_scheduled_ = false;
  regenerate_layer_trees_ = false;
  pending_frame_semaphore_.Signal();

  if (!producer_continuation_) {
    // We may already have a valid pipeline continuation in case a previous
    // begin frame did not result in an Animator::Render. Simply reuse that
    // instead of asking the pipeline for a fresh continuation.
    producer_continuation_ = layer_tree_pipeline_->Produce();

    if (!producer_continuation_) {
      // If we still don't have valid continuation, the pipeline is currently
      // full because the consumer is being too slow. Try again at the next
      // frame interval.
      TRACE_EVENT0("flutter", "PipelineFull");
      RequestFrame();
      return;
    }
  }

  // We have acquired a valid continuation from the pipeline and are ready
  // to service potential frame.
  FML_DCHECK(producer_continuation_);
  const fml::TimePoint frame_target_time =
      frame_timings_recorder_->GetVsyncTargetTime();
  dart_frame_deadline_ = frame_target_time.ToEpochDelta();
  uint64_t frame_number = frame_timings_recorder_->GetFrameNumber();
  delegate_.OnAnimatorBeginFrame(frame_target_time, frame_number);
}

void Animator::EndFrame() {
  if (frame_timings_recorder_ == nullptr) {
    // `EndFrame` has been called in this frame. This happens if the engine has
    // called `OnAllViewsRendered` and then the end of the vsync task calls
    // `EndFrame` again.
    return;
  }
  if (!layer_trees_tasks_.empty()) {
    // The build is completed in OnAnimatorBeginFrame.
    frame_timings_recorder_->RecordBuildEnd(fml::TimePoint::Now());

    delegate_.OnAnimatorUpdateLatestFrameTargetTime(
        frame_timings_recorder_->GetVsyncTargetTime());

    // Commit the pending continuation.
    std::vector<std::unique_ptr<LayerTreeTask>> layer_tree_task_list;
    layer_tree_task_list.reserve(layer_trees_tasks_.size());
    for (auto& [view_id, layer_tree_task] : layer_trees_tasks_) {
      layer_tree_task_list.push_back(std::move(layer_tree_task));
    }
    layer_trees_tasks_.clear();
    PipelineProduceResult result = producer_continuation_.Complete(
        std::make_unique<FrameItem>(std::move(layer_tree_task_list),
                                    std::move(frame_timings_recorder_)));

    if (!result.success) {
      FML_DLOG(INFO) << "Failed to commit to the pipeline";
    } else if (!result.is_first_item) {
      // Do nothing. It has been successfully pushed to the pipeline but not as
      // the first item. Eventually the 'Rasterizer' will consume it, so we
      // don't need to notify the delegate.
    } else {
      delegate_.OnAnimatorDraw(layer_tree_pipeline_);
    }
  }
  frame_timings_recorder_ = nullptr;

  if (!frame_scheduled_ && has_rendered_) {
    // Wait a tad more than 3 60hz frames before reporting a big idle period.
    // This is a heuristic that is meant to avoid giving false positives to the
    // VM when we are about to schedule a frame in the next vsync, the idea
    // being that if there have been three vsyncs with no frames it's a good
    // time to start doing GC work.
    task_runners_.GetUITaskRunner()->PostDelayedTask(
        [self = weak_factory_.GetWeakPtr()]() {
          if (!self) {
            return;
          }
          // If there's a frame scheduled, bail.
          // If there's no frame scheduled, but we're not yet past the last
          // vsync deadline, bail.
          if (!self->frame_scheduled_) {
            auto now =
                fml::TimeDelta::FromMicroseconds(Dart_TimelineGetMicros());
            if (now > self->dart_frame_deadline_) {
              TRACE_EVENT0("flutter", "BeginFrame idle callback");
              self->delegate_.OnAnimatorNotifyIdle(
                  now + fml::TimeDelta::FromMilliseconds(100));
            }
          }
        },
        kNotifyIdleTaskWaitTime);
  }
  FML_DCHECK(layer_trees_tasks_.empty());
  FML_DCHECK(frame_timings_recorder_ == nullptr);
}

void Animator::Render(int64_t view_id,
                      std::unique_ptr<flutter::LayerTree> layer_tree,
                      float device_pixel_ratio) {
  has_rendered_ = true;

  if (!frame_timings_recorder_) {
    // Framework can directly call render with a built scene. A major reason is
    // to render warm up frames.
    frame_timings_recorder_ = std::make_unique<FrameTimingsRecorder>();
    const fml::TimePoint placeholder_time = fml::TimePoint::Now();
    frame_timings_recorder_->RecordVsync(placeholder_time, placeholder_time);
    frame_timings_recorder_->RecordBuildStart(placeholder_time);
  }

  TRACE_EVENT_WITH_FRAME_NUMBER(frame_timings_recorder_, "flutter",
                                "Animator::Render", /*flow_id_count=*/0,
                                /*flow_ids=*/nullptr);

  // Only inserts if the view ID has not been rendered before, ignoring
  // duplicate Render calls.
  layer_trees_tasks_.try_emplace(
      view_id, std::make_unique<LayerTreeTask>(view_id, std::move(layer_tree),
                                               device_pixel_ratio));
}

const std::weak_ptr<VsyncWaiter> Animator::GetVsyncWaiter() const {
  std::weak_ptr<VsyncWaiter> weak = waiter_;
  return weak;
}

bool Animator::CanReuseLastLayerTrees() {
  return !regenerate_layer_trees_;
}

void Animator::DrawLastLayerTrees(
    std::unique_ptr<FrameTimingsRecorder> frame_timings_recorder) {
  // This method is very cheap, but this makes it explicitly clear in trace
  // files.
  TRACE_EVENT0("flutter", "Animator::DrawLastLayerTrees");

  pending_frame_semaphore_.Signal();
  // In this case BeginFrame doesn't get called, we need to
  // adjust frame timings to update build start and end times,
  // given that the frame doesn't get built in this case, we
  // will use Now() for both start and end times as an indication.
  const auto now = fml::TimePoint::Now();
  frame_timings_recorder->RecordBuildStart(now);
  frame_timings_recorder->RecordBuildEnd(now);
  delegate_.OnAnimatorDrawLastLayerTrees(std::move(frame_timings_recorder));
}

void Animator::RequestFrame(bool regenerate_layer_trees) {
  if (regenerate_layer_trees && !regenerate_layer_trees_) {
    // This event will be closed by BeginFrame. BeginFrame will only be called
    // if regenerating the layer trees. If a frame has been requested to update
    // an external texture, this will be false and no BeginFrame call will
    // happen.
    TRACE_EVENT_ASYNC_BEGIN0("flutter", "Frame Request Pending",
                             frame_request_number_);
    regenerate_layer_trees_ = true;
  }

  if (!pending_frame_semaphore_.TryWait()) {
    // Multiple calls to Animator::RequestFrame will still result in a
    // single request to the VsyncWaiter.
    return;
  }

  // The AwaitVSync is going to call us back at the next VSync. However, we want
  // to be reasonably certain that the UI thread is not in the middle of a
  // particularly expensive callout. We post the AwaitVSync to run right after
  // an idle. This does NOT provide a guarantee that the UI thread has not
  // started an expensive operation right after posting this message however.
  // To support that, we need edge triggered wakes on VSync.

  task_runners_.GetUITaskRunner()->PostTask(
      [self = weak_factory_.GetWeakPtr()]() {
        if (!self) {
          return;
        }
        self->AwaitVSync();
      });
  frame_scheduled_ = true;
}

void Animator::AwaitVSync() {
  waiter_->AsyncWaitForVsync(
      [self = weak_factory_.GetWeakPtr()](
          std::unique_ptr<FrameTimingsRecorder> frame_timings_recorder) {
        if (self) {
          if (self->CanReuseLastLayerTrees()) {
            self->DrawLastLayerTrees(std::move(frame_timings_recorder));
          } else {
            self->BeginFrame(std::move(frame_timings_recorder));
            self->EndFrame();
          }
        }
      });
  if (has_rendered_) {
    delegate_.OnAnimatorNotifyIdle(dart_frame_deadline_);
  }
}

void Animator::OnAllViewsRendered() {
  if (!layer_trees_tasks_.empty()) {
    EndFrame();
  }
}

void Animator::ScheduleSecondaryVsyncCallback(uintptr_t id,
                                              const fml::closure& callback) {
  waiter_->ScheduleSecondaryCallback(id, callback);
}

void Animator::ScheduleMaybeClearTraceFlowIds() {
  waiter_->ScheduleSecondaryCallback(
      reinterpret_cast<uintptr_t>(this), [self = weak_factory_.GetWeakPtr()] {
        if (!self) {
          return;
        }
        if (!self->frame_scheduled_ && !self->trace_flow_ids_.empty()) {
          size_t flow_id_count = self->trace_flow_ids_.size();
          std::unique_ptr<uint64_t[]> flow_ids =
              std::make_unique<uint64_t[]>(flow_id_count);
          for (size_t i = 0; i < flow_id_count; ++i) {
            flow_ids.get()[i] = self->trace_flow_ids_.at(i);
          }

          TRACE_EVENT0_WITH_FLOW_IDS(
              "flutter", "Animator::ScheduleMaybeClearTraceFlowIds - callback",
              flow_id_count, flow_ids.get());

          while (!self->trace_flow_ids_.empty()) {
            auto flow_id = self->trace_flow_ids_.front();
            TRACE_FLOW_END("flutter", "PointerEvent", flow_id);
            self->trace_flow_ids_.pop_front();
          }
        }
      });
}

}  // namespace flutter
