// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_ANIMATOR_H_
#define FLUTTER_SHELL_COMMON_ANIMATOR_H_

#include <deque>

#include "flutter/common/task_runners.h"
#include "flutter/flow/frame_timings.h"
#include "flutter/fml/memory/ref_ptr.h"
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/fml/synchronization/semaphore.h"
#include "flutter/fml/time/time_point.h"
#include "flutter/shell/common/pipeline.h"
#include "flutter/shell/common/rasterizer.h"
#include "flutter/shell/common/vsync_waiter.h"

namespace flutter {

namespace testing {
class ShellTest;
}

/// Executor of animations.
///
/// In conjunction with the |VsyncWaiter| it allows callers (typically Dart
/// code) to schedule work that ends up generating a |LayerTree|.
class Animator final {
 public:
  class Delegate {
   public:
    virtual void OnAnimatorBeginFrame(fml::TimePoint frame_target_time,
                                      uint64_t frame_number) = 0;

    virtual void OnAnimatorNotifyIdle(fml::TimeDelta deadline) = 0;

    virtual void OnAnimatorUpdateLatestFrameTargetTime(
        fml::TimePoint frame_target_time) = 0;

    virtual void OnAnimatorDraw(std::shared_ptr<FramePipeline> pipeline) = 0;

    virtual void OnAnimatorDrawLastLayerTrees(
        std::unique_ptr<FrameTimingsRecorder> frame_timings_recorder) = 0;
  };

  Animator(Delegate& delegate,
           const TaskRunners& task_runners,
           std::unique_ptr<VsyncWaiter> waiter);

  ~Animator();

  void RequestFrame(bool regenerate_layer_trees = true);

  //--------------------------------------------------------------------------
  /// @brief    Tells the Animator that all views that should render for this
  ///           frame have been rendered.
  ///
  ///           In regular frames, since all `Render` calls must take place
  ///           during a vsync task, the Animator knows that all views have
  ///           been rendered at the end of the vsync task, therefore calling
  ///           this method is not needed.
  ///
  ///           However, the engine might decide to start it a bit earlier, for
  ///           example, if the engine decides that no more views can be
  ///           rendered, so that the rasterization can start a bit earlier.
  ///
  ///           This method is also useful in warm-up frames. In a warm up
  ///           frame, `Animator::Render` is called out of vsync tasks, and
  ///           Animator requires an explicit end-of-frame call to know when to
  ///           send the layer trees to the pipeline.
  ///
  ///           For more about warm up frames, see
  ///           `PlatformDispatcher.scheduleWarmUpFrame`.
  ///
  void OnAllViewsRendered();

  //--------------------------------------------------------------------------
  /// @brief    Tells the Animator that this frame needs to render another view.
  ///
  ///           This method must be called during a vsync callback, or
  ///           technically, between Animator::BeginFrame and Animator::EndFrame
  ///           (both private methods). Otherwise, this call will be ignored.
  ///
  void Render(int64_t view_id,
              std::unique_ptr<flutter::LayerTree> layer_tree,
              float device_pixel_ratio);

  const std::weak_ptr<VsyncWaiter> GetVsyncWaiter() const;

  //--------------------------------------------------------------------------
  /// @brief    Schedule a secondary callback to be executed right after the
  ///           main `VsyncWaiter::AsyncWaitForVsync` callback (which is added
  ///           by `Animator::RequestFrame`).
  ///
  ///           Like the callback in `AsyncWaitForVsync`, this callback is
  ///           only scheduled to be called once, and it's supposed to be
  ///           called in the UI thread. If there is no AsyncWaitForVsync
  ///           callback (`Animator::RequestFrame` is not called), this
  ///           secondary callback will still be executed at vsync.
  ///
  ///           This callback is used to provide the vsync signal needed by
  ///           `SmoothPointerDataDispatcher`, and for our own flow events.
  ///
  /// @see      `PointerDataDispatcher::ScheduleSecondaryVsyncCallback`.
  void ScheduleSecondaryVsyncCallback(uintptr_t id,
                                      const fml::closure& callback);

  // Enqueue |trace_flow_id| into |trace_flow_ids_|.  The flow event will be
  // ended at either the next frame, or the next vsync interval with no active
  // rendering.
  void EnqueueTraceFlowId(uint64_t trace_flow_id);

 private:
  // Animator's work during a vsync is split into two methods, BeginFrame and
  // EndFrame. The two methods should be called synchronously back-to-back to
  // avoid being interrupted by a regular vsync. The reason to split them is to
  // allow ShellTest::PumpOneFrame to insert a Render in between.

  void BeginFrame(std::unique_ptr<FrameTimingsRecorder> frame_timings_recorder);
  void EndFrame();

  bool CanReuseLastLayerTrees();

  void DrawLastLayerTrees(
      std::unique_ptr<FrameTimingsRecorder> frame_timings_recorder);

  void AwaitVSync();

  // Clear |trace_flow_ids_| if |frame_scheduled_| is false.
  void ScheduleMaybeClearTraceFlowIds();

  Delegate& delegate_;
  TaskRunners task_runners_;
  std::shared_ptr<VsyncWaiter> waiter_;

  std::unique_ptr<FrameTimingsRecorder> frame_timings_recorder_;
  std::unordered_map<int64_t, std::unique_ptr<LayerTreeTask>>
      layer_trees_tasks_;
  uint64_t frame_request_number_ = 1;
  fml::TimeDelta dart_frame_deadline_;
  std::shared_ptr<FramePipeline> layer_tree_pipeline_;
  fml::Semaphore pending_frame_semaphore_;
  FramePipeline::ProducerContinuation producer_continuation_;
  bool regenerate_layer_trees_ = false;
  bool frame_scheduled_ = false;
  std::deque<uint64_t> trace_flow_ids_;
  bool has_rendered_ = false;

  fml::WeakPtrFactory<Animator> weak_factory_;

  friend class testing::ShellTest;

  FML_DISALLOW_COPY_AND_ASSIGN(Animator);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_COMMON_ANIMATOR_H_
