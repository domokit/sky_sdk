// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/common/vsync_waiter.h"

#include "flow/frame_timings.h"
#include "flutter/fml/task_runner.h"
#include "flutter/fml/trace_event.h"
#include "fml/logging.h"
#include "fml/message_loop_task_queues.h"
#include "fml/task_queue_id.h"
#include "fml/time/time_point.h"

namespace flutter {

static constexpr const char* kVsyncFlowName = "VsyncFlow";

static constexpr const char* kVsyncTraceName = "VsyncProcessCallback";

VsyncWaiter::VsyncWaiter(const TaskRunners& task_runners)
    : task_runners_(task_runners) {}

VsyncWaiter::~VsyncWaiter() = default;

// Public method invoked by the animator.
void VsyncWaiter::AsyncWaitForVsync(const Callback& callback) {
  if (!callback) {
    return;
  }

  TRACE_EVENT0("flutter", "AsyncWaitForVsync");

  {
    std::scoped_lock lock(callback_mutex_);
    if (callback_) {
      // The animator may request a frame more than once within a frame
      // interval. Multiple calls to request frame must result in a single
      // callback per frame interval.
      TRACE_EVENT_INSTANT0("flutter", "MultipleCallsToVsyncInFrameInterval");
      return;
    }
    callback_ = callback;
    if (!secondary_callbacks_.empty()) {
      // Return directly as `AwaitVSync` is already called by
      // `ScheduleSecondaryCallback`.
      return;
    }
  }
  AwaitVSync();
}

void VsyncWaiter::ScheduleSecondaryCallback(uintptr_t id,
                                            const fml::closure& callback) {
  FML_DCHECK(task_runners_.GetUITaskRunner()->RunsTasksOnCurrentThread());

  if (!callback) {
    return;
  }

  TRACE_EVENT0("flutter", "ScheduleSecondaryCallback");

  {
    std::scoped_lock lock(callback_mutex_);
    bool secondary_callbacks_originally_empty = secondary_callbacks_.empty();
    auto [_, inserted] = secondary_callbacks_.emplace(id, callback);
    if (!inserted) {
      // Multiple schedules must result in a single callback per frame interval.
      TRACE_EVENT_INSTANT0("flutter",
                           "MultipleCallsToSecondaryVsyncInFrameInterval");
      return;
    }
    if (callback_) {
      // Return directly as `AwaitVSync` is already called by
      // `AsyncWaitForVsync`.
      return;
    }
    if (!secondary_callbacks_originally_empty) {
      // Return directly as `AwaitVSync` is already called by
      // `ScheduleSecondaryCallback`.
      return;
    }
  }
  AwaitVSyncForSecondaryCallback();
}

void VsyncWaiter::FireCallback(fml::TimePoint frame_start_time,
                               fml::TimePoint frame_target_time,
                               bool pause_secondary_tasks) {
  FML_DCHECK(fml::TimePoint::Now() >= frame_start_time);

  Callback callback;
  std::vector<fml::closure> secondary_callbacks;

  {
    std::scoped_lock lock(callback_mutex_);
    callback = std::move(callback_);
    for (auto& pair : secondary_callbacks_) {
      secondary_callbacks.push_back(std::move(pair.second));
    }
    secondary_callbacks_.clear();
  }

  if (!callback && secondary_callbacks.empty()) {
    // This means that the vsync waiter implementation fired a callback for a
    // request we did not make. This is a paranoid check but we still want to
    // make sure we catch misbehaving vsync implementations.
    TRACE_EVENT_INSTANT0("flutter", "MismatchedFrameCallback");
    return;
  }

  if (callback) {
    const uint64_t flow_identifier = fml::tracing::TraceNonce();
    if (pause_secondary_tasks) {
      PauseDartEventLoopTasks();
    }

    // The base trace ensures that flows have a root to begin from if one does
    // not exist. The trace viewer will ignore traces that have no base event
    // trace. While all our message loops insert a base trace trace
    // (MessageLoop::RunExpiredTasks), embedders may not.
    TRACE_EVENT0_WITH_FLOW_IDS("flutter", "VsyncFireCallback",
                               /*flow_id_count=*/1,
                               /*flow_ids=*/&flow_identifier);

    TRACE_FLOW_BEGIN("flutter", kVsyncFlowName, flow_identifier);

    fml::TaskQueueId ui_task_queue_id =
        task_runners_.GetUITaskRunner()->GetTaskQueueId();
    task_runners_.GetUITaskRunner()->PostTask(
        [ui_task_queue_id, callback, flow_identifier, frame_start_time,
         frame_target_time, pause_secondary_tasks]() {
          FML_TRACE_EVENT_WITH_FLOW_IDS(
              "flutter", kVsyncTraceName, /*flow_id_count=*/1,
              /*flow_ids=*/&flow_identifier, "StartTime", frame_start_time,
              "TargetTime", frame_target_time);
          std::unique_ptr<FrameTimingsRecorder> frame_timings_recorder =
              std::make_unique<FrameTimingsRecorder>();
          frame_timings_recorder->RecordVsync(frame_start_time,
                                              frame_target_time);
          callback(std::move(frame_timings_recorder));
          TRACE_FLOW_END("flutter", kVsyncFlowName, flow_identifier);
          if (pause_secondary_tasks) {
            ResumeDartEventLoopTasks(ui_task_queue_id);
          }
        });
  }

  for (auto& secondary_callback : secondary_callbacks) {
    task_runners_.GetUITaskRunner()->PostTask(secondary_callback);
  }
}

void VsyncWaiter::PauseDartEventLoopTasks() {
  auto ui_task_queue_id = task_runners_.GetUITaskRunner()->GetTaskQueueId();
  auto task_queues = fml::MessageLoopTaskQueues::GetInstance();
  if (ui_task_queue_id.is_valid()) {
    task_queues->PauseSecondarySource(ui_task_queue_id);
  }
}

void VsyncWaiter::ResumeDartEventLoopTasks(fml::TaskQueueId ui_task_queue_id) {
  auto task_queues = fml::MessageLoopTaskQueues::GetInstance();
  if (ui_task_queue_id.is_valid()) {
    task_queues->ResumeSecondarySource(ui_task_queue_id);
  }
}

}  // namespace flutter
