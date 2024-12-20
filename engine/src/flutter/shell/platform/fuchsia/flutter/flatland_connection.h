// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Maintains a connection to Fuchsia's Flatland protocol used for rendering
// 2D graphics scenes.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_FLATLAND_CONNECTION_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_FLATLAND_CONNECTION_H_

#include <fuchsia/ui/composition/cpp/fidl.h>
#include <lib/async/default.h>

#include "flutter/fml/closure.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/time/time_delta.h"
#include "flutter/fml/time/time_point.h"

#include "vsync_waiter.h"

#include <cstdint>
#include <mutex>
#include <queue>
#include <string>

namespace flutter_runner {

// The maximum number of fences that can be signaled at a time.
static constexpr size_t kMaxFences =
    fuchsia::ui::composition::MAX_ACQUIRE_RELEASE_FENCE_COUNT;

// A helper to ferry around multiplexed events for signaling. Helps move
// non-copyable events into closures.
class Overflow {
 public:
  std::vector<zx::event> fences_;
  zx::event event_;
  Overflow() { fences_.reserve(kMaxFences); };
};

using on_frame_presented_event =
    std::function<void(fuchsia::scenic::scheduling::FramePresentedInfo)>;

// 10ms interval to target vsync is only used until Scenic sends presentation
// feedback.
static constexpr fml::TimeDelta kInitialFlatlandVsyncOffset =
    fml::TimeDelta::FromMilliseconds(10);

// The component residing on the raster thread that is responsible for
// maintaining the Flatland instance connection and presenting updates.
class FlatlandConnection final {
 public:
  FlatlandConnection(
      const std::string& debug_label,
      fuchsia::ui::composition::FlatlandHandle flatland,
      fml::closure error_callback,
      on_frame_presented_event on_frame_presented_callback,
      async_dispatcher_t* dispatcher = async_get_default_dispatcher());

  ~FlatlandConnection();

  void Present();

  // Used to implement VsyncWaiter functionality.
  // Note that these two methods are called from the UI thread while the
  // rest of the methods on this class are called from the raster thread.
  void AwaitVsync(FireCallbackCallback callback);
  void AwaitVsyncForSecondaryCallback(FireCallbackCallback callback);

  fuchsia::ui::composition::Flatland* flatland() { return flatland_.get(); }

  fuchsia::ui::composition::TransformId NextTransformId() {
    return {++next_transform_id_};
  }

  fuchsia::ui::composition::ContentId NextContentId() {
    return {++next_content_id_};
  }

  // Adds a new acquire fence to be sent out to the next Present() call.
  //
  // Acquire fences must all be signaled by the user.
  //
  // PERFORMANCE NOTES:
  //
  // Enqueuing more than 16 fences per frame incurs a performance penalty, so
  // use them sparingly.
  //
  // Skipped frames may cause the number of fences to increase, leading to
  // more performance issues. Ideally, the flow of the frames should be smooth.
  void EnqueueAcquireFence(zx::event fence);

  // Adds a new release fence to be sent out to the next Present() call.
  //
  // Release fences are all signaled by Scenic (Flatland server).
  //
  // See the performance notes on EnqueueAcquireFence for performance details.
  void EnqueueReleaseFence(zx::event fence);

 private:
  void OnError(fuchsia::ui::composition::FlatlandError error);

  void OnNextFrameBegin(
      fuchsia::ui::composition::OnNextFrameBeginValues values);
  void OnFramePresented(fuchsia::scenic::scheduling::FramePresentedInfo info);
  void DoPresent();

  fml::TimePoint GetNextPresentationTime(const fml::TimePoint& now);
  bool MaybeRunInitialVsyncCallback(const fml::TimePoint& now,
                                    FireCallbackCallback& callback);
  void RunVsyncCallback(const fml::TimePoint& now,
                        FireCallbackCallback& callback);

  async_dispatcher_t* dispatcher_;

  fuchsia::ui::composition::FlatlandPtr flatland_;

  fml::closure error_callback_;

  uint64_t next_transform_id_ = 0;
  uint64_t next_content_id_ = 0;

  on_frame_presented_event on_frame_presented_callback_;
  bool present_waiting_for_credit_ = false;

  // A flow event trace id for following |Flatland::Present| calls into Scenic.
  uint64_t next_present_trace_id_ = 0;

  // This struct contains state that is accessed from both from the UI thread
  // (in AwaitVsync) and the raster thread (in OnNextFrameBegin and Present).
  // You should always lock mutex_ before touching anything in this struct
  struct {
    std::mutex mutex_;
    std::queue<fml::TimePoint> next_presentation_times_;
    fml::TimeDelta vsync_interval_ = kInitialFlatlandVsyncOffset;
    fml::TimeDelta vsync_offset_ = kInitialFlatlandVsyncOffset;
    fml::TimePoint last_presentation_time_;
    FireCallbackCallback pending_fire_callback_;
    uint32_t present_credits_ = 1;
    bool initial_vsync_callback_ran_ = false;
  } threadsafe_state_;

  // Acquire fences sent to Flatland.
  std::vector<zx::event> acquire_fences_;

  // Multiplexed acquire fences over the critical number of ~16.
  std::shared_ptr<Overflow> acquire_overflow_;

  // Release fences sent to Flatland. Similar to acquire fences above.
  std::vector<zx::event> current_present_release_fences_;
  std::shared_ptr<Overflow> current_release_overflow_;

  // Release fences from the prior call to DoPresent().
  // Similar to acquire fences above.
  std::vector<zx::event> previous_present_release_fences_;
  std::shared_ptr<Overflow> previous_release_overflow_;

  std::string debug_label_;

  FML_DISALLOW_COPY_AND_ASSIGN(FlatlandConnection);
};

}  // namespace flutter_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_FLATLAND_CONNECTION_H_
