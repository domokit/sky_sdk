// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flatland_connection.h"

#include <lib/async/cpp/task.h>
#include <lib/async/default.h>

#include <zircon/rights.h>
#include <zircon/status.h>
#include <zircon/types.h>

#include <utility>

#include "flutter/fml/logging.h"

namespace flutter_runner {

namespace {

// Helper function for traces.
double DeltaFromNowInNanoseconds(const fml::TimePoint& now,
                                 const fml::TimePoint& time) {
  return (time - now).ToNanoseconds();
}

}  // namespace

FlatlandConnection::FlatlandConnection(
    const std::string& debug_label,
    fuchsia::ui::composition::FlatlandHandle flatland,
    fml::closure error_callback,
    on_frame_presented_event on_frame_presented_callback,
    async_dispatcher_t* dispatcher)
    : dispatcher_(dispatcher),
      flatland_(flatland.Bind()),
      error_callback_(std::move(error_callback)),
      on_frame_presented_callback_(std::move(on_frame_presented_callback)) {
  flatland_.set_error_handler([callback = error_callback_](zx_status_t status) {
    FML_LOG(ERROR) << "Flatland disconnected: " << zx_status_get_string(status);
    callback();
  });
  debug_label_ = debug_label;
  flatland_->SetDebugName(debug_label);
  flatland_.events().OnError =
      fit::bind_member(this, &FlatlandConnection::OnError);
  flatland_.events().OnFramePresented =
      fit::bind_member(this, &FlatlandConnection::OnFramePresented);
  flatland_.events().OnNextFrameBegin =
      fit::bind_member(this, &FlatlandConnection::OnNextFrameBegin);
}

FlatlandConnection::~FlatlandConnection() = default;

// This method is called from the raster thread.
void FlatlandConnection::Present() {
  TRACE_DURATION("flutter", "FlatlandConnection::Present");
  std::scoped_lock<std::mutex> lock(threadsafe_state_.mutex_);
  if (threadsafe_state_.present_credits_ > 0) {
    DoPresent();
  } else {
    present_waiting_for_credit_ = true;
  }
}

// This method is called from the raster thread.
void FlatlandConnection::DoPresent() {
  TRACE_DURATION("flutter", "FlatlandConnection::DoPresent");

  std::string per_app_tracing_name =
      "Flatland::PerAppPresent[" + debug_label_ + "]";
  TRACE_FLOW_BEGIN("gfx", per_app_tracing_name.c_str(), next_present_trace_id_);
  ++next_present_trace_id_;

  FML_CHECK(threadsafe_state_.present_credits_ > 0);
  --threadsafe_state_.present_credits_;

  fuchsia::ui::composition::PresentArgs present_args;
  present_args.set_requested_presentation_time(0);
  present_args.set_acquire_fences(std::move(acquire_fences_));

  // Schedule acquire fence overflow signaling if there is one.
  if (acquire_overflow_ != nullptr) {
    FML_CHECK(acquire_overflow_->event_.is_valid());
    async::PostTask(dispatcher_, [dispatcher = dispatcher_,
                                  overflow = acquire_overflow_]() {
      const size_t fences_size = overflow->fences_.size();
      std::shared_ptr<size_t> fences_completed = std::make_shared<size_t>(0);
      std::shared_ptr<std::vector<async::WaitOnce>> closures;

      for (auto i = 0u; i < fences_size; i++) {
        auto wait = std::make_unique<async::WaitOnce>(
            overflow->fences_[i].get(), ZX_EVENT_SIGNALED, 0u);
        auto wait_ptr = wait.get();
        wait_ptr->Begin(
            dispatcher,
            [wait = std::move(wait), overflow, fences_size, fences_completed,
             closures](async_dispatcher_t*, async::WaitOnce*,
                       zx_status_t status, const zx_packet_signal_t*) {
              (*fences_completed)++;
              FML_CHECK(status == ZX_OK)
                  << "status: " << zx_status_get_string(status);
              if (*fences_completed == fences_size) {
                // Signal the acquire fence passed on to Flatland.
                const zx_status_t status =
                    overflow->event_.signal(0, ZX_EVENT_SIGNALED);
                FML_CHECK(status == ZX_OK)
                    << "status: " << zx_status_get_string(status);
              }
            });
      }
    });
    acquire_overflow_.reset();
  }
  FML_CHECK(acquire_overflow_ == nullptr);

  present_args.set_release_fences(std::move(previous_present_release_fences_));
  // Frame rate over latency.
  present_args.set_unsquashable(true);
  flatland_->Present(std::move(present_args));

  // In Flatland, release fences apply to the content of the previous present.
  // Keeping track of the old frame's release fences and swapping ensure we set
  // the correct ones for VulkanSurface's interpretation.
  previous_present_release_fences_.clear();
  previous_present_release_fences_.swap(current_present_release_fences_);
  previous_release_overflow_ = current_release_overflow_;
  current_release_overflow_ = nullptr;

  // Similar to the treatment of acquire_fences_overflow_ above. Except in
  // the other direction.
  if (previous_release_overflow_ != nullptr) {
    FML_CHECK(previous_release_overflow_->event_.is_valid());

    std::shared_ptr<Overflow> fences = previous_release_overflow_;

    async::PostTask(dispatcher_, [dispatcher = dispatcher_,
                                  fences = previous_release_overflow_]() {
      FML_CHECK(fences != nullptr);
      FML_CHECK(fences->event_.is_valid());

      auto wait = std::make_unique<async::WaitOnce>(fences->event_.get(),
                                                    ZX_EVENT_SIGNALED, 0u);
      auto wait_ptr = wait.get();

      wait_ptr->Begin(
          dispatcher, [_wait = std::move(wait), fences](
                          async_dispatcher_t*, async::WaitOnce*,
                          zx_status_t status, const zx_packet_signal_t*) {
            FML_CHECK(status == ZX_OK)
                << "status: " << zx_status_get_string(status);

            // Multiplex signaling all events.
            for (auto& event : fences->fences_) {
              const zx_status_t status = event.signal(0, ZX_EVENT_SIGNALED);
              FML_CHECK(status == ZX_OK)
                  << "status: " << zx_status_get_string(status);
            }
          });
    });
    previous_release_overflow_ = nullptr;
  }
  FML_CHECK(previous_release_overflow_ == nullptr);  // Moved.

  acquire_fences_.clear();
}

// This method is called from the UI thread.
void FlatlandConnection::AwaitVsync(FireCallbackCallback callback) {
  TRACE_DURATION("flutter", "FlatlandConnection::AwaitVsync");

  std::scoped_lock<std::mutex> lock(threadsafe_state_.mutex_);
  threadsafe_state_.pending_fire_callback_ = nullptr;
  const auto now = fml::TimePoint::Now();

  // Initial case.
  if (MaybeRunInitialVsyncCallback(now, callback)) {
    return;
  }

  // Throttle case.
  if (threadsafe_state_.present_credits_ == 0) {
    threadsafe_state_.pending_fire_callback_ = std::move(callback);
    return;
  }

  // Regular case.
  RunVsyncCallback(now, callback);
}

// This method is called from the UI thread.
void FlatlandConnection::AwaitVsyncForSecondaryCallback(
    FireCallbackCallback callback) {
  TRACE_DURATION("flutter",
                 "FlatlandConnection::AwaitVsyncForSecondaryCallback");

  std::scoped_lock<std::mutex> lock(threadsafe_state_.mutex_);
  const auto now = fml::TimePoint::Now();

  // Initial case.
  if (MaybeRunInitialVsyncCallback(now, callback)) {
    return;
  }

  // Regular case.
  RunVsyncCallback(now, callback);
}

// This method is called from the raster thread.
void FlatlandConnection::OnError(
    fuchsia::ui::composition::FlatlandError error) {
  FML_LOG(ERROR) << "Flatland error: " << static_cast<int>(error);
  error_callback_();
}

// This method is called from the raster thread.
void FlatlandConnection::OnNextFrameBegin(
    fuchsia::ui::composition::OnNextFrameBeginValues values) {
  // Collect now before locking because this is an important timing information
  // from Scenic.
  const auto now = fml::TimePoint::Now();

  std::scoped_lock<std::mutex> lock(threadsafe_state_.mutex_);
  threadsafe_state_.present_credits_ += values.additional_present_credits();
  TRACE_DURATION("flutter", "FlatlandConnection::OnNextFrameBegin",
                 "present_credits", threadsafe_state_.present_credits_);

  if (present_waiting_for_credit_ && threadsafe_state_.present_credits_ > 0) {
    DoPresent();
    present_waiting_for_credit_ = false;
  }

  // Update vsync_interval_ by calculating the difference between the first two
  // presentation times. Flatland always returns >1 presentation_infos, so this
  // check is to guard against any changes to this assumption.
  if (values.has_future_presentation_infos() &&
      values.future_presentation_infos().size() > 1) {
    threadsafe_state_.vsync_interval_ = fml::TimeDelta::FromNanoseconds(
        values.future_presentation_infos().at(1).presentation_time() -
        values.future_presentation_infos().at(0).presentation_time());
  } else {
    FML_LOG(WARNING)
        << "Flatland didn't send enough future_presentation_infos to update "
           "vsync interval.";
  }

  // Update next_presentation_times_.
  std::queue<fml::TimePoint> new_times;
  for (const auto& info : values.future_presentation_infos()) {
    new_times.emplace(fml::TimePoint::FromEpochDelta(
        fml::TimeDelta::FromNanoseconds(info.presentation_time())));
  }
  threadsafe_state_.next_presentation_times_.swap(new_times);

  // Update vsync_offset_.
  // We use modulo here because Flatland may point to the following vsync if
  // OnNextFrameBegin() is called after the current frame's latch point.
  auto vsync_offset =
      (fml::TimePoint::FromEpochDelta(fml::TimeDelta::FromNanoseconds(
           values.future_presentation_infos().front().presentation_time())) -
       now) %
      threadsafe_state_.vsync_interval_;
  // Thread contention may result in OnNextFrameBegin() being called after the
  // presentation time. Ignore these outliers.
  if (vsync_offset > fml::TimeDelta::Zero()) {
    threadsafe_state_.vsync_offset_ = vsync_offset;
  }

  // Throttle case.
  if (threadsafe_state_.pending_fire_callback_ &&
      threadsafe_state_.present_credits_ > 0) {
    RunVsyncCallback(now, threadsafe_state_.pending_fire_callback_);
    threadsafe_state_.pending_fire_callback_ = nullptr;
  }
}

// This method is called from the raster thread.
void FlatlandConnection::OnFramePresented(
    fuchsia::scenic::scheduling::FramePresentedInfo info) {
  on_frame_presented_callback_(std::move(info));
}

// Parses and updates next_presentation_times_.
fml::TimePoint FlatlandConnection::GetNextPresentationTime(
    const fml::TimePoint& now) {
  const fml::TimePoint& cutoff =
      now > threadsafe_state_.last_presentation_time_
          ? now
          : threadsafe_state_.last_presentation_time_;

  // Remove presentation times that may have been passed. This may happen after
  // a long draw call.
  while (!threadsafe_state_.next_presentation_times_.empty() &&
         threadsafe_state_.next_presentation_times_.front() <= cutoff) {
    threadsafe_state_.next_presentation_times_.pop();
  }

  // Calculate a presentation time based on
  // |threadsafe_state_.last_presentation_time_| that is later than cutoff using
  // |vsync_interval| increments if we don't have any future presentation times
  // left.
  if (threadsafe_state_.next_presentation_times_.empty()) {
    auto result = threadsafe_state_.last_presentation_time_;
    while (result <= cutoff) {
      result = result + threadsafe_state_.vsync_interval_;
    }
    return result;
  }

  // Return the next presentation time in the queue for the regular case.
  const auto result = threadsafe_state_.next_presentation_times_.front();
  threadsafe_state_.next_presentation_times_.pop();
  return result;
}

// This method is called from the UI thread.
bool FlatlandConnection::MaybeRunInitialVsyncCallback(
    const fml::TimePoint& now,
    FireCallbackCallback& callback) {
  // Only sent maybe_run_initial_vsync once.
  if (threadsafe_state_.initial_vsync_callback_ran_) {
    return false;
  }
  TRACE_DURATION("flutter", "FlatlandConnection::MaybeRunInitialVsyncCallback");
  const auto frame_end = now + kInitialFlatlandVsyncOffset;
  threadsafe_state_.last_presentation_time_ = frame_end;
  threadsafe_state_.initial_vsync_callback_ran_ = true;
  callback(now, frame_end);
  return true;
}

// This method may be called from the raster or UI thread, but it is safe
// because VsyncWaiter posts the vsync callback on UI thread.
void FlatlandConnection::RunVsyncCallback(const fml::TimePoint& now,
                                          FireCallbackCallback& callback) {
  const auto& frame_end = GetNextPresentationTime(now);
  const auto& frame_start = frame_end - threadsafe_state_.vsync_offset_;
  threadsafe_state_.last_presentation_time_ = frame_end;
  TRACE_DURATION("flutter", "FlatlandConnection::RunVsyncCallback",
                 "frame_start_delta",
                 DeltaFromNowInNanoseconds(now, frame_start), "frame_end_delta",
                 DeltaFromNowInNanoseconds(now, frame_end));
  callback(frame_start, frame_end);
}

// Enqueue a single fence into either the "base" vector of fences, or a
// "special" overflow multiplexer.
//
// Args:
//   - fence: the fence to add
//   - fences: the "regular" fences vector to add to.
//   - overflow: the overflow fences vector. Fences added here if there are
//     more than can fit in `fences`.
static void Enqueue(zx::event fence,
                    std::vector<zx::event>* fences,
                    std::shared_ptr<Overflow>* overflow) {
  constexpr size_t kMaxFences =
      fuchsia::ui::composition::MAX_ACQUIRE_RELEASE_FENCE_COUNT;

  // Number of all previously added fences, plus this one.
  const auto num_all_fences =
      fences->size() + 1 +
      ((*overflow == nullptr) ? 0 : (*overflow)->fences_.size());

  // If more than max number of fences come in, schedule any further fences into
  // an overflow. The overflow fences are scheduled for processing here, but are
  // processed in DoPresent().
  if (num_all_fences <= kMaxFences) {
    fences->push_back(std::move(fence));
  } else if (num_all_fences == kMaxFences + 1) {
    // The ownership of the overflow will be handed over to the signaling
    // closure on DoPresent call. So we always expect that we enter here with
    // overflow not set.
    FML_CHECK((*overflow) == nullptr) << "overflow is still active";
    *overflow = std::make_shared<Overflow>();

    // Set up the overflow fences. Creates an overflow handle, places it
    // into `fences` instead of the previous fence, and puts the prior fence
    // and this one into overflow.
    zx::event overflow_handle = std::move(fences->back());
    fences->pop_back();

    zx::event overflow_fence;
    zx_status_t status = zx::event::create(0, &overflow_fence);
    FML_CHECK(status == ZX_OK) << "status: " << zx_status_get_string(status);

    // Every DoPresent should invalidate this handle.  Holler if not.
    FML_CHECK(!(*overflow)->event_.is_valid()) << "overflow valid";
    status =
        overflow_fence.duplicate(ZX_RIGHT_SAME_RIGHTS, &(*overflow)->event_);
    FML_CHECK(status == ZX_OK) << "status: " << zx_status_get_string(status);
    fences->push_back(std::move(overflow_fence));

    // Prepare for wait_many call.
    (*overflow)->fences_.push_back(std::move(overflow_handle));
    (*overflow)->fences_.push_back(std::move(fence));

    FML_LOG(INFO) << "Enqueue using fence overflow, expect a performance hit.";
  } else {
    FML_CHECK((*overflow) != nullptr);
    // Just add to the overflow fences.
    (*overflow)->fences_.push_back(std::move(fence));
  }
}

// This method is called from the raster thread.
void FlatlandConnection::EnqueueAcquireFence(zx::event fence) {
  Enqueue(std::move(fence), &acquire_fences_, &acquire_overflow_);
}

// This method is called from the raster thread.
void FlatlandConnection::EnqueueReleaseFence(zx::event fence) {
  Enqueue(std::move(fence), &current_present_release_fences_,
          &current_release_overflow_);
}

}  // namespace flutter_runner
